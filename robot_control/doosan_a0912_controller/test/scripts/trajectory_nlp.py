#!/usr/bin/env python3

from __future__ import annotations

import argparse
import atexit
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import sys
import time
import xml.etree.ElementTree as ET

import casadi as ca
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parents[2]
DEFAULT_TRAJECTORY_CSV = [
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_000.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_001.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_002.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_003.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_004.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_005.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_006.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_007.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_008.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_009.csv",
]
DEFAULT_URDF = (
    PROJECT_ROOT
    / "robot_control/doosan_a0912_controller/urdf/a0912.white.urdf"
)
DEFAULT_OUTPUT_DIR = Path("/tmp/trajectory_nlp")

TCP_FRAME = "link_6"
COMMAND_TIME_SEC = 0.001
DONE_MARGIN_SEC = 2.0


@dataclass
class TaskPoint:
    t: float
    s: float
    path_speed: float
    path_acceleration: float
    position: np.ndarray
    rotation: np.ndarray


@dataclass
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    velocity: float


def parse_float_list(text: str, expected: int, name: str) -> list[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if len(values) != expected:
        raise ValueError(f"{name} must contain {expected} comma-separated values")
    return values


def rpy_to_matrix_np(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = [float(v) for v in rpy]
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def quaternion_xyzw_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-12:
        return np.eye(3)
    qx, qy, qz, qw = q / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qw * qz),
                2.0 * (qx * qz + qw * qy),
            ],
            [
                2.0 * (qx * qy + qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qw * qx),
            ],
            [
                2.0 * (qx * qz - qw * qy),
                2.0 * (qy * qz + qw * qx),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=float,
    )


def parse_xyz(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    values = [float(part) for part in text.split()]
    if len(values) != 3:
        raise ValueError(f"expected 3 values, got: {text}")
    return np.array(values, dtype=float)


def parse_urdf_chain(urdf_path: Path, tcp_frame: str) -> list[JointSpec]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints_by_child: dict[str, JointSpec] = {}

    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        joint_type = joint.attrib.get("type", "fixed")
        parent_el = joint.find("parent")
        child_el = joint.find("child")
        if parent_el is None or child_el is None:
            continue

        origin_el = joint.find("origin")
        axis_el = joint.find("axis")
        limit_el = joint.find("limit")
        origin_xyz = parse_xyz(
            origin_el.attrib.get("xyz") if origin_el is not None else None,
            (0.0, 0.0, 0.0),
        )
        origin_rpy = parse_xyz(
            origin_el.attrib.get("rpy") if origin_el is not None else None,
            (0.0, 0.0, 0.0),
        )
        axis = parse_xyz(
            axis_el.attrib.get("xyz") if axis_el is not None else None,
            (0.0, 0.0, 1.0),
        )
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm > 1.0e-12:
            axis = axis / axis_norm

        if joint_type == "continuous":
            lower, upper = -math.pi, math.pi
        elif limit_el is not None and joint_type != "fixed":
            lower = float(limit_el.attrib.get("lower", -math.pi))
            upper = float(limit_el.attrib.get("upper", math.pi))
        else:
            lower, upper = 0.0, 0.0
        velocity = (
            float(limit_el.attrib.get("velocity", math.inf))
            if limit_el is not None
            else math.inf
        )

        spec = JointSpec(
            name=name,
            joint_type=joint_type,
            parent=parent_el.attrib["link"],
            child=child_el.attrib["link"],
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis=axis,
            lower=lower,
            upper=upper,
            velocity=velocity,
        )
        joints_by_child[spec.child] = spec

    chain: list[JointSpec] = []
    link = tcp_frame
    while link in joints_by_child:
        spec = joints_by_child[link]
        chain.append(spec)
        link = spec.parent
    chain.reverse()
    if not chain:
        raise ValueError(f"could not find a URDF chain ending at frame/link {tcp_frame}")
    return chain


def ca_cross(a, b):
    return ca.vertcat(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def ca_skew(axis: np.ndarray):
    x, y, z = [float(v) for v in axis]
    return ca.DM([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def ca_axis_angle(axis: np.ndarray, q):
    k = ca_skew(axis)
    return ca.DM.eye(3) + ca.sin(q) * k + (1.0 - ca.cos(q)) * (k @ k)


def ca_rot_to_vec_row_major(rotation):
    return ca.vertcat(
        rotation[0, 0],
        rotation[0, 1],
        rotation[0, 2],
        rotation[1, 0],
        rotation[1, 1],
        rotation[1, 2],
        rotation[2, 0],
        rotation[2, 1],
        rotation[2, 2],
    )


def ca_rotation_from_row_major(values):
    return ca.vertcat(
        ca.horzcat(values[0], values[1], values[2]),
        ca.horzcat(values[3], values[4], values[5]),
        ca.horzcat(values[6], values[7], values[8]),
    )


def ca_orientation_error(current_rotation, target_rotation):
    return 0.5 * (
        ca_cross(current_rotation[:, 0], target_rotation[:, 0])
        + ca_cross(current_rotation[:, 1], target_rotation[:, 1])
        + ca_cross(current_rotation[:, 2], target_rotation[:, 2])
    )


class KinematicModel:
    def __init__(self, chain: list[JointSpec]) -> None:
        self.chain = chain
        self.active_joints = [joint for joint in chain if joint.joint_type != "fixed"]
        self.nq = len(self.active_joints)
        self.joint_names = [joint.name for joint in self.active_joints]
        self.lower = np.array([joint.lower for joint in self.active_joints], dtype=float)
        self.upper = np.array([joint.upper for joint in self.active_joints], dtype=float)
        self.velocity = np.array([joint.velocity for joint in self.active_joints], dtype=float)

    def fk_symbolic(self, q):
        rotation = ca.DM.eye(3)
        position = ca.DM.zeros(3, 1)
        joint_origins = []
        joint_axes = []
        active_index = 0

        for joint in self.chain:
            position = position + rotation @ ca.DM(joint.origin_xyz.reshape(3, 1))
            rotation = rotation @ ca.DM(rpy_to_matrix_np(joint.origin_rpy))

            if joint.joint_type == "fixed":
                continue

            axis_local = ca.DM(joint.axis.reshape(3, 1))
            axis_world = rotation @ axis_local
            if joint.joint_type in ("revolute", "continuous"):
                joint_origins.append(position)
                joint_axes.append(axis_world)
                rotation = rotation @ ca_axis_angle(joint.axis, q[active_index])
            elif joint.joint_type == "prismatic":
                joint_origins.append(position)
                joint_axes.append(axis_world)
                position = position + axis_world * q[active_index]
            else:
                raise ValueError(f"unsupported joint type: {joint.joint_type}")
            active_index += 1

        columns = []
        for joint, origin, axis in zip(self.active_joints, joint_origins, joint_axes):
            if joint.joint_type in ("revolute", "continuous"):
                linear = ca_cross(axis, position - origin)
                angular = axis
            else:
                linear = axis
                angular = ca.DM.zeros(3, 1)
            columns.append(ca.vertcat(linear, angular))
        jacobian = ca.horzcat(*columns)
        return position, rotation, jacobian

    def residual_function(self):
        q = ca.MX.sym("q", self.nq)
        target = ca.MX.sym("target", 12)
        position, rotation, _jacobian = self.fk_symbolic(q)
        target_position = target[0:3]
        target_rotation = ca_rotation_from_row_major(target[3:12])
        residual = ca.vertcat(
            position - target_position,
            ca_orientation_error(rotation, target_rotation),
        )
        residual_jacobian = ca.jacobian(residual, q)
        return ca.Function("residual_jacobian", [q, target], [residual, residual_jacobian])

    def fk_function(self):
        q = ca.MX.sym("q", self.nq)
        position, rotation, jacobian = self.fk_symbolic(q)
        return ca.Function(
            "fk",
            [q],
            [position, ca_rot_to_vec_row_major(rotation), jacobian],
        )


def load_task_points(csv_path: Path, stride: int, max_points: int) -> list[TaskPoint]:
    with csv_path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = [
            "t",
            "s",
            "position_x",
            "position_y",
            "position_z",
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "orientation_w",
            "path_speed",
            "path_acceleration",
        ]
        missing = [name for name in required if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{csv_path} missing columns: {', '.join(missing)}")

        points = []
        first_t = None
        first_s = None
        previous_row_index = None
        for row in reader:
            row_index_text = str(row.get("row_index", "")).strip()
            if row_index_text:
                row_index = int(float(row_index_text))
                if previous_row_index is None:
                    previous_row_index = row_index
                elif row_index != previous_row_index:
                    raise ValueError(
                        f"{csv_path} contains multiple row_index values. "
                        "Optimize one trajectory_XXX.csv file at a time."
                    )

            t = float(row["t"])
            if first_t is None:
                first_t = t
            s = float(row["s"])
            if first_s is None:
                first_s = s
            position = np.array(
                [float(row["position_x"]), float(row["position_y"]), float(row["position_z"])],
                dtype=float,
            )
            rotation = quaternion_xyzw_to_matrix(
                float(row["orientation_x"]),
                float(row["orientation_y"]),
                float(row["orientation_z"]),
                float(row["orientation_w"]),
            )
            points.append(
                TaskPoint(
                    t=t - first_t,
                    s=s - first_s,
                    path_speed=float(row["path_speed"]),
                    path_acceleration=float(row["path_acceleration"]),
                    position=position,
                    rotation=rotation,
                )
            )

    if len(points) < 2:
        raise ValueError(f"{csv_path} needs at least two trajectory points")

    stride = max(1, int(stride))
    selected = points[::stride]
    if selected[-1] is not points[-1]:
        selected.append(points[-1])

    if max_points > 0 and len(selected) > max_points:
        indices = np.unique(np.rint(np.linspace(0, len(selected) - 1, max_points)).astype(int))
        selected = [selected[int(index)] for index in indices]
        if selected[-1] is not points[-1]:
            selected.append(points[-1])

    for prev, cur in zip(selected, selected[1:]):
        if cur.t <= prev.t:
            raise ValueError(f"{csv_path} has non-increasing time after downsampling")
        if cur.s <= prev.s:
            raise ValueError(f"{csv_path} has non-increasing arc length after downsampling")
    return selected


def target_vector(point: TaskPoint) -> np.ndarray:
    return np.concatenate([point.position, point.rotation.reshape(-1)])


def seed_path(
    model: KinematicModel,
    points: list[TaskPoint],
    initial_q: np.ndarray,
    max_step_rad: float,
) -> np.ndarray:
    residual_fun = model.residual_function()
    q = np.clip(initial_q.astype(float), model.lower, model.upper)
    out = []
    damping = 1.0e-3

    for point_index, point in enumerate(points):
        target = target_vector(point)
        for _ in range(120):
            residual, jacobian = residual_fun(q, target)
            residual = np.asarray(residual, dtype=float).reshape(-1)
            jacobian = np.asarray(jacobian, dtype=float)
            if (
                np.linalg.norm(residual[:3]) < 2.0e-3
                and np.linalg.norm(residual[3:]) < math.radians(1.0)
            ):
                break
            lhs = jacobian.T @ jacobian + damping * damping * np.eye(model.nq)
            rhs = -jacobian.T @ residual
            try:
                dq = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                dq = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            step_norm = float(np.linalg.norm(dq, ord=np.inf))
            if step_norm > max_step_rad:
                dq *= max_step_rad / step_norm
            q = np.clip(q + dq, model.lower, model.upper)

        residual, _jacobian = residual_fun(q, target)
        residual = np.asarray(residual, dtype=float).reshape(-1)
        if point_index % 25 == 0 or point_index == len(points) - 1:
            print(
                f"[nlp] seed {point_index + 1}/{len(points)} "
                f"pos_err={np.linalg.norm(residual[:3]):.5f}m "
                f"rot_err={math.degrees(np.linalg.norm(residual[3:])):.3f}deg"
            )
        out.append(q.copy())
    return np.asarray(out, dtype=float)


def finite_difference(times: np.ndarray, positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    velocity = np.zeros_like(positions)
    acceleration = np.zeros_like(positions)

    for index in range(len(times)):
        lo = max(0, index - 1)
        hi = min(len(times) - 1, index + 1)
        dt = times[hi] - times[lo]
        if dt <= 0.0:
            raise ValueError("invalid trajectory time spacing")
        velocity[index] = (positions[hi] - positions[lo]) / dt

    for index in range(len(times)):
        lo = max(0, index - 1)
        hi = min(len(times) - 1, index + 1)
        dt = times[hi] - times[lo]
        if dt <= 0.0:
            raise ValueError("invalid trajectory time spacing")
        acceleration[index] = (velocity[hi] - velocity[lo]) / dt

    velocity[0] = 0.0
    velocity[-1] = 0.0
    acceleration[0] = 0.0
    acceleration[-1] = 0.0
    return velocity, acceleration


def derivative_by_axis(axis: np.ndarray, values: np.ndarray) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).reshape(-1)
    values = np.asarray(values, dtype=float)
    if len(axis) != len(values):
        raise ValueError("axis and values length mismatch")
    if len(axis) < 2:
        raise ValueError("at least two samples are required")
    if np.any(np.diff(axis) <= 0.0):
        raise ValueError("axis must be strictly increasing")

    edge_order = 2 if len(axis) > 2 else 1
    derivative = np.zeros_like(values)
    for column in range(values.shape[1]):
        derivative[:, column] = np.gradient(values[:, column], axis, edge_order=edge_order)
    return derivative


def interpolate_columns(axis: np.ndarray, values: np.ndarray, samples: np.ndarray) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).reshape(-1)
    values = np.asarray(values, dtype=float)
    samples = np.asarray(samples, dtype=float).reshape(-1)
    out = np.zeros((len(samples), values.shape[1]), dtype=float)
    for column in range(values.shape[1]):
        out[:, column] = np.interp(samples, axis, values[:, column])
    return out


def joint_motion_from_arc_profile(
    points: list[TaskPoint],
    q_rad: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    arc = np.array([point.s for point in points], dtype=float)
    path_speed = np.array([point.path_speed for point in points], dtype=float)
    path_acceleration = np.array([point.path_acceleration for point in points], dtype=float)

    dq_ds = derivative_by_axis(arc, q_rad)
    d2q_ds2 = derivative_by_axis(arc, dq_ds)
    qd_rad = dq_ds * path_speed.reshape(-1, 1)
    qdd_rad = (
        d2q_ds2 * (path_speed * path_speed).reshape(-1, 1)
        + dq_ds * path_acceleration.reshape(-1, 1)
    )
    qd_rad[0] = 0.0
    qd_rad[-1] = 0.0
    qdd_rad[0] = 0.0
    qdd_rad[-1] = 0.0
    return qd_rad, qdd_rad


def build_command_trajectory(
    points: list[TaskPoint],
    q_rad: np.ndarray,
    command_dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    source_times = np.array([point.t for point in points], dtype=float)
    source_arc = np.array([point.s for point in points], dtype=float)
    source_speed = np.array([point.path_speed for point in points], dtype=float)

    dt = max(float(command_dt), 1.0e-4)
    duration = float(source_times[-1])
    command_times = np.arange(0.0, duration, dt, dtype=float)
    if command_times.size == 0 or not np.isclose(command_times[-1], duration, atol=1.0e-10):
        command_times = np.concatenate([command_times, [duration]])
    command_times = np.unique(command_times)

    command_arc = np.interp(command_times, source_times, source_arc)
    command_speed = np.interp(command_times, source_times, source_speed)
    dq_ds = derivative_by_axis(source_arc, q_rad)

    q_command = interpolate_columns(source_arc, q_rad, command_arc)
    dq_ds_command = interpolate_columns(source_arc, dq_ds, command_arc)
    qd_command = dq_ds_command * command_speed.reshape(-1, 1)
    qdd_command = finite_difference(command_times, qd_command)[0]
    qd_command[0] = 0.0
    qd_command[-1] = 0.0
    qdd_command[0] = 0.0
    qdd_command[-1] = 0.0
    return command_times, command_arc, q_command, qd_command, qdd_command


def optimize_trajectory(
    model: KinematicModel,
    points: list[TaskPoint],
    seed: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray:
    nq = model.nq
    count = len(points)
    times = np.array([point.t for point in points], dtype=float)
    if count < 2:
        raise ValueError("NLP needs at least two points")

    opti = ca.Opti()
    q_var = opti.variable(nq, count)
    objective = 0.0

    q_lower = ca.DM(model.lower)
    q_upper = ca.DM(model.upper)
    q_mid = ca.DM(0.5 * (model.lower + model.upper))
    q_range = ca.DM(np.maximum(model.upper - model.lower, 1.0e-6))

    urdf_velocity_limit = np.where(np.isfinite(model.velocity), model.velocity, np.inf)
    user_velocity_limit = np.full(nq, math.radians(args.max_joint_velocity_deg_s), dtype=float)
    velocity_limit = np.minimum(user_velocity_limit, urdf_velocity_limit)
    velocity_limit = np.where(np.isfinite(velocity_limit), velocity_limit, user_velocity_limit)
    acceleration_limit = np.full(nq, math.radians(args.max_joint_acceleration_deg_s2), dtype=float)
    v_limit_dm = ca.DM(velocity_limit)
    a_limit_dm = ca.DM(acceleration_limit)

    position_tol = float(args.position_tolerance)
    orientation_tol = math.radians(float(args.orientation_tolerance_deg))

    for index, point in enumerate(points):
        qk = q_var[:, index]
        position, rotation, jacobian = model.fk_symbolic(qk)
        pos_err = position - ca.DM(point.position.reshape(3, 1))
        rot_err = ca_orientation_error(rotation, ca.DM(point.rotation))

        opti.subject_to(opti.bounded(q_lower, qk, q_upper))
        opti.subject_to(opti.bounded(-position_tol, pos_err, position_tol))
        opti.subject_to(opti.bounded(-orientation_tol, rot_err, orientation_tol))

        objective += args.position_weight * ca.sumsqr(pos_err)
        objective += args.orientation_weight * ca.sumsqr(rot_err)
        objective += args.center_weight * ca.sumsqr((qk - q_mid) / q_range)

        if args.singularity_weight > 0.0:
            gram = jacobian @ jacobian.T
            regularized_gram = gram + 1.0e-8 * ca.DM.eye(6)
            objective += args.singularity_weight * ca.trace(
                ca.solve(regularized_gram, ca.DM.eye(6))
            )
        if args.min_manipulability > 0.0:
            raise ValueError(
                "--min-manipulability is disabled in this NLP path because "
                "CasADi's determinant node is not evaluable by this Ipopt build."
            )

    for index in range(count - 1):
        dt = times[index + 1] - times[index]
        if dt <= 0.0:
            raise ValueError("trajectory time must be strictly increasing")
        velocity = (q_var[:, index + 1] - q_var[:, index]) / dt
        opti.subject_to(opti.bounded(-v_limit_dm, velocity, v_limit_dm))
        objective += args.velocity_weight * ca.sumsqr(velocity / v_limit_dm)

    for index in range(1, count - 1):
        prev_dt = times[index] - times[index - 1]
        next_dt = times[index + 1] - times[index]
        prev_velocity = (q_var[:, index] - q_var[:, index - 1]) / prev_dt
        next_velocity = (q_var[:, index + 1] - q_var[:, index]) / next_dt
        acceleration = 2.0 * (next_velocity - prev_velocity) / (prev_dt + next_dt)
        opti.subject_to(opti.bounded(-a_limit_dm, acceleration, a_limit_dm))
        objective += args.acceleration_weight * ca.sumsqr(acceleration / a_limit_dm)

    opti.minimize(objective)
    opti.set_initial(q_var, seed.T)
    opti.solver(
        "ipopt",
        {"expand": False},
        {
            "max_iter": int(args.ipopt_max_iter),
            "tol": float(args.ipopt_tol),
            "print_level": int(args.ipopt_print_level),
            "sb": "yes",
        },
    )

    print(f"[nlp] solving NLP points={count} joints={nq}")
    try:
        solution = opti.solve()
        q_solution = np.asarray(solution.value(q_var), dtype=float).T
    except RuntimeError:
        print("[nlp] solver failed")
        raise
    return q_solution


def evaluate_solution(model: KinematicModel, points: list[TaskPoint], q_solution: np.ndarray) -> dict[str, float]:
    fk_fun = model.fk_function()
    max_pos_error = 0.0
    max_rot_error = 0.0
    min_manipulability = math.inf
    for point, q in zip(points, q_solution):
        position, rotation_vec, jacobian = fk_fun(q)
        position = np.asarray(position, dtype=float).reshape(3)
        rotation = np.asarray(rotation_vec, dtype=float).reshape(3, 3)
        jacobian = np.asarray(jacobian, dtype=float)
        rot_err = np.asarray(
            ca_orientation_error(ca.DM(rotation), ca.DM(point.rotation)),
            dtype=float,
        ).reshape(3)
        max_pos_error = max(max_pos_error, float(np.linalg.norm(position - point.position)))
        max_rot_error = max(max_rot_error, float(np.linalg.norm(rot_err)))
        det_value = float(np.linalg.det(jacobian @ jacobian.T))
        min_manipulability = min(min_manipulability, math.sqrt(max(0.0, det_value)))
    return {
        "max_position_error_m": max_pos_error,
        "max_orientation_error_deg": math.degrees(max_rot_error),
        "min_manipulability": min_manipulability,
    }


def write_joint_csv(
    output_csv: Path,
    times: np.ndarray,
    arc_lengths: np.ndarray,
    q_rad: np.ndarray,
    qd_rad: np.ndarray,
    qdd_rad: np.ndarray,
) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    q_deg = np.degrees(q_rad)
    qd_deg = np.degrees(qd_rad)
    qdd_deg = np.degrees(qdd_rad)

    header = ["t", "s"]
    header += [f"q{index}" for index in range(1, q_rad.shape[1] + 1)]
    header += [f"qd{index}" for index in range(1, q_rad.shape[1] + 1)]
    header += [f"qdd{index}" for index in range(1, q_rad.shape[1] + 1)]

    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index, t in enumerate(times):
            writer.writerow(
                [f"{t:.9f}", f"{arc_lengths[index]:.9f}"]
                + [f"{value:.9f}" for value in q_deg[index]]
                + [f"{value:.9f}" for value in qd_deg[index]]
                + [f"{value:.9f}" for value in qdd_deg[index]]
            )


def output_path_for(input_csv: Path, output: Path, multiple: bool) -> Path:
    output = output.expanduser().resolve()
    if multiple or output.suffix.lower() != ".csv":
        return output / f"{input_csv.stem}_nlp_joint.csv"
    return output


def command_output_path_for(input_csv: Path, output: Path, multiple: bool) -> Path:
    output = output.expanduser().resolve()
    if multiple or output.suffix.lower() != ".csv":
        return output / f"{input_csv.stem}_nlp_command.csv"
    return output.with_name(f"{output.stem}_command{output.suffix}")


def trajectory_duration_sec(csv_path: Path) -> float:
    with csv_path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        last_t = None
        for row in reader:
            value = str(row.get("t", "")).strip()
            if value:
                last_t = float(value)
    if last_t is None:
        raise ValueError(f"{csv_path} does not contain trajectory rows")
    return max(last_t, 0.0)


def run_robot(outputs: list[Path], args: argparse.Namespace) -> None:
    from joint_target_test import DoosanRobot, IP, MODE, PORT, RT_IP, RT_PORT, TIMEOUT_SEC

    robot = DoosanRobot()

    def cleanup() -> None:
        robot.shutdown()

    def handle_signal(signum, _frame) -> None:
        print(f"\n[python] signal {signum} received")
        cleanup()
        raise SystemExit(128 + signum)

    atexit.register(cleanup)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, handle_signal)

    print("[python] initialize robot")
    robot.initialize(
        ip=IP,
        port=PORT,
        mode=MODE,
        rt_ip=RT_IP,
        rt_port=RT_PORT,
        enable_rt=True,
        timeout_sec=TIMEOUT_SEC,
    )

    for index, csv_path in enumerate(outputs, start=1):
        duration = trajectory_duration_sec(csv_path)
        print(f"[python] optimized joint CSV #{index}/{len(outputs)}: {csv_path}")
        print(f"[python] trajectory duration: {duration:.3f}s")
        robot.joint_trajectory_csv_velocity_control(
            csv_path,
            command_time_sec=args.command_time_sec,
            position_gain=args.joint_position_gain,
            max_joint_step_deg=args.max_joint_step_deg,
            max_joint_velocity_deg_s=args.max_joint_velocity_deg_s,
            max_joint_acceleration_deg_s2=args.max_joint_acceleration_deg_s2,
            preposition_max_joint_velocity_deg_s=args.preposition_max_joint_velocity_deg_s,
            preposition_max_joint_acceleration_deg_s2=args.preposition_max_joint_acceleration_deg_s2,
        )
        robot.wait_until_motion_done(duration + DONE_MARGIN_SEC)
        print(f"[python] optimized joint trajectory #{index} done")

    print("[python] robot is running. Press Ctrl+C to servo off and exit.")
    while True:
        time.sleep(1.0)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline CasADi NLP trajectory optimization for A0912 Cartesian painting CSVs."
    )
    parser.add_argument("csv", nargs="*", type=Path, help="Input trajectory_XXX.csv file(s).")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--tcp-frame", default=TCP_FRAME)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-points", type=int, default=0)
    parser.add_argument("--initial-joint-deg", default="90,0,90,-90,0,0")
    parser.add_argument("--position-tolerance", type=float, default=0.002)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--max-joint-step-deg", type=float, default=20.0)
    parser.add_argument("--max-joint-velocity-deg-s", type=float, default=70.0)
    parser.add_argument("--max-joint-acceleration-deg-s2", type=float, default=500.0)
    parser.add_argument("--min-manipulability", type=float, default=0.0)
    parser.add_argument("--position-weight", type=float, default=1000.0)
    parser.add_argument("--orientation-weight", type=float, default=100.0)
    parser.add_argument("--velocity-weight", type=float, default=1.0e-3)
    parser.add_argument("--acceleration-weight", type=float, default=1.0e-4)
    parser.add_argument("--center-weight", type=float, default=1.0e-3)
    parser.add_argument("--singularity-weight", type=float, default=1.0e-8)
    parser.add_argument("--ipopt-max-iter", type=int, default=800)
    parser.add_argument("--ipopt-tol", type=float, default=1.0e-5)
    parser.add_argument("--ipopt-print-level", type=int, default=5)
    parser.add_argument("--execute", action="store_true", help="Send optimized joint CSV to the robot after solving.")
    parser.add_argument("--command-dt", type=float, default=COMMAND_TIME_SEC)
    parser.add_argument("--command-time-sec", type=float, default=COMMAND_TIME_SEC)
    parser.add_argument("--joint-position-gain", type=float, default=4.0)
    parser.add_argument("--preposition-max-joint-velocity-deg-s", type=float, default=10.0)
    parser.add_argument("--preposition-max-joint-acceleration-deg-s2", type=float, default=100.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    input_csvs = [path.expanduser().resolve() for path in args.csv]
    if not input_csvs:
        input_csvs = [path.resolve() for path in DEFAULT_TRAJECTORY_CSV]
    for csv_path in input_csvs:
        if not csv_path.is_file():
            raise FileNotFoundError(csv_path)
    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(urdf_path)

    chain = parse_urdf_chain(urdf_path, args.tcp_frame)
    model = KinematicModel(chain)
    if model.nq != 6:
        raise ValueError(
            f"controller currently expects 6 joints, but URDF chain has {model.nq}: {model.joint_names}"
        )

    initial_q = np.radians(np.array(parse_float_list(args.initial_joint_deg, model.nq, "--initial-joint-deg")))
    seed_initial_q = initial_q.copy()
    max_seed_step_rad = math.radians(max(1.0, min(args.max_joint_step_deg, 15.0)))

    print(f"[nlp] URDF: {urdf_path}")
    print(f"[nlp] joints: {', '.join(model.joint_names)}")
    print(f"[nlp] output: {args.output}")

    command_outputs = []
    for csv_index, input_csv in enumerate(input_csvs, start=1):
        print(f"[nlp] input #{csv_index}/{len(input_csvs)}: {input_csv}")
        points = load_task_points(input_csv, stride=args.stride, max_points=args.max_points)
        print(
            f"[nlp] loaded points={len(points)} duration={points[-1].t:.3f}s "
            f"stride={args.stride} max_points={args.max_points}"
        )

        seed = seed_path(model, points, seed_initial_q, max_seed_step_rad)
        q_solution = optimize_trajectory(model, points, seed, args)
        seed_initial_q = q_solution[-1].copy()
        metrics = evaluate_solution(model, points, q_solution)
        times = np.array([point.t for point in points], dtype=float)
        arc_lengths = np.array([point.s for point in points], dtype=float)
        qd_solution, qdd_solution = joint_motion_from_arc_profile(points, q_solution)
        print(
            "[nlp] metrics "
            f"max_pos_err={metrics['max_position_error_m']:.6f}m "
            f"max_rot_err={metrics['max_orientation_error_deg']:.4f}deg "
            f"min_manip={metrics['min_manipulability']:.6e} "
            f"max_qdot={np.max(np.abs(np.degrees(qd_solution))):.3f}deg/s "
            f"max_qddot={np.max(np.abs(np.degrees(qdd_solution))):.3f}deg/s^2"
        )

        output_csv = output_path_for(input_csv, args.output, multiple=len(input_csvs) > 1)
        write_joint_csv(output_csv, times, arc_lengths, q_solution, qd_solution, qdd_solution)
        print(f"[nlp] saved optimized joint trajectory: {output_csv}")

        command_times, command_arc, command_q, command_qd, command_qdd = build_command_trajectory(
            points,
            q_solution,
            args.command_dt,
        )
        command_csv = command_output_path_for(input_csv, args.output, multiple=len(input_csvs) > 1)
        write_joint_csv(command_csv, command_times, command_arc, command_q, command_qd, command_qdd)
        command_qd_deg = np.degrees(command_qd)
        command_qdd_deg = np.degrees(command_qdd)
        print(
            f"[nlp] saved {args.command_dt:.4f}s speedj command reference: {command_csv} "
            f"points={len(command_times)} max_qdot={np.max(np.abs(command_qd_deg)):.3f}deg/s "
            f"max_qddot={np.max(np.abs(command_qdd_deg)):.3f}deg/s^2"
        )
        command_outputs.append(command_csv)

    if args.execute:
        run_robot(command_outputs, args)
    else:
        print("[nlp] dry run complete. Add --execute to track the optimized command CSV on the robot.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
