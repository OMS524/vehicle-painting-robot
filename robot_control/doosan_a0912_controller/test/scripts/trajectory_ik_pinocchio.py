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

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    pin = None


ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parents[2]
DEFAULT_TRAJECTORY_CSV = [
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_000.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_001.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_002.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_003.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_004.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_005.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_006.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_007.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_008.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_009.csv",
    
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_bumper_small/trajectory_000.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_bumper_small/trajectory_001.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_bumper_small/trajectory_002.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_bumper_small/trajectory_003.csv",
]
DEFAULT_URDF = (
    PROJECT_ROOT
    / "robot_control/doosan_a0912_controller/urdf/a0912.white.urdf"
)
DEFAULT_OUTPUT_DIR = Path("/tmp/trajectory_ik")

TCP_FRAME = "link_6"
TCP_SPRAY_AXIS = "z"
TCP_PATH_AXIS = "x"
COMMAND_TIME_SEC = 0.001
DONE_MARGIN_SEC = 2.0

INITIAL_JOINT_DEG = "90,0,90,-90,0,0"
POSITION_TOLERANCE_M = 0.0001
ORIENTATION_TOLERANCE_DEG = 1.0
MAX_JOINT_STEP_DEG = 20.0
MAX_JOINT_VELOCITY_DEG_S = 70.0
MAX_JOINT_ACCELERATION_DEG_S2 = 500.0
MIN_MANIPULABILITY = 0.0
MIN_SINGULAR_VALUE = 0.01 #1.0e-4

IK_MAX_ITER = 100
IK_DAMPING = 0.03
IK_MAX_STEP_DEG = 5.0
IK_POSITION_GAIN = 1.0
IK_ORIENTATION_GAIN = 1.0
IK_PROGRESS_INTERVAL = 25
IK_MAX_SEEDS_PER_POINT = 48
IK_MAX_CANDIDATES_PER_POINT = 12
IK_DUPLICATE_CANDIDATE_DEG = 0.5
IK_SEED_OFFSETS_DEG = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 45.0, 0.0, -45.0),
    (0.0, 0.0, 0.0, -45.0, 0.0, 45.0),
    (0.0, 0.0, 0.0, 90.0, 0.0, -90.0),
    (0.0, 0.0, 0.0, -90.0, 0.0, 90.0),
    (0.0, 0.0, 0.0, 180.0, 0.0, -180.0),
    (0.0, 0.0, 0.0, -180.0, 0.0, 180.0),
    (0.0, 0.0, 0.0, 0.0, 20.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, -20.0, 0.0),
    (0.0, 60.0, -60.0, 0.0, 0.0, 0.0),
    (0.0, -60.0, 60.0, 0.0, 0.0, 0.0),
    (180.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (-180.0, 0.0, 0.0, 0.0, 0.0, 0.0),
)
DP_TRACKING_WEIGHT = 0.1
DP_JUMP_WEIGHT = 1.0
DP_VELOCITY_WEIGHT = 1.0
DP_ACCELERATION_WEIGHT = 1.0
DP_SINGULARITY_WEIGHT = 0.2
DP_LIMIT_MARGIN_WEIGHT = 0.02

JOINT_POSITION_GAIN = 4.0
PREPOSITION_MAX_JOINT_VELOCITY_DEG_S = 15.0
PREPOSITION_MAX_JOINT_ACCELERATION_DEG_S2 = 100.0


@dataclass
class TaskPoint:
    t: float
    s: float
    path_speed: float
    path_acceleration: float
    position: np.ndarray
    rotation: np.ndarray


@dataclass
class IkPointResult:
    q: np.ndarray
    iterations: int
    position_error_m: float
    orientation_error_deg: float
    converged: bool


@dataclass
class IkCandidate:
    q: np.ndarray
    position_error_m: float
    orientation_error_deg: float
    min_sigma: float
    manipulability: float
    iterations: int
    seed_index: int


def require_pinocchio() -> None:
    if pin is None:
        raise RuntimeError(
            "Python module 'pinocchio' is not available in this Python environment. "
            "If you installed robotpkg Pinocchio, run this from a shell where the "
            "OpenRobots environment is sourced, for example: "
            "source /opt/openrobots/setup.bash"
        )


def parse_float_list(text: str, expected: int, name: str) -> list[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if len(values) != expected:
        raise ValueError(f"{name} must contain {expected} comma-separated values")
    return values


def parse_signed_axis(axis: str) -> tuple[int, float]:
    value = str(axis).strip().lower()
    sign = 1.0
    if value.startswith("+"):
        value = value[1:]
    elif value.startswith("-"):
        value = value[1:]
        sign = -1.0

    indices = {"x": 0, "y": 1, "z": 2}
    if value not in indices:
        raise ValueError(f"invalid axis '{axis}'. Use one of x, y, z, -x, -y, -z")
    return indices[value], sign


def remap_planner_rotation_to_tcp(
    planner_rotation: np.ndarray,
    tcp_spray_axis: str,
    tcp_path_axis: str,
) -> np.ndarray:
    spray_index, spray_sign = parse_signed_axis(tcp_spray_axis)
    path_index, path_sign = parse_signed_axis(tcp_path_axis)
    if spray_index == path_index:
        raise ValueError("--tcp-spray-axis and --tcp-path-axis must use different axes")

    planner_rotation = np.asarray(planner_rotation, dtype=float).reshape(3, 3)
    planner_path_axis = planner_rotation[:, 0]
    planner_spray_axis = planner_rotation[:, 2]

    tcp_axes: list[np.ndarray | None] = [None, None, None]
    tcp_axes[spray_index] = spray_sign * planner_spray_axis

    path_axis = planner_path_axis - float(np.dot(planner_path_axis, tcp_axes[spray_index])) * tcp_axes[spray_index]
    path_norm = float(np.linalg.norm(path_axis))
    if path_norm <= 1.0e-9:
        path_axis = planner_rotation[:, 1]
        path_axis = path_axis - float(np.dot(path_axis, tcp_axes[spray_index])) * tcp_axes[spray_index]
        path_norm = float(np.linalg.norm(path_axis))
    if path_norm <= 1.0e-9:
        raise ValueError("could not build TCP target rotation from planner rotation")
    tcp_axes[path_index] = path_sign * (path_axis / path_norm)

    missing_index = next(index for index, axis in enumerate(tcp_axes) if axis is None)
    if missing_index == 0:
        tcp_axes[0] = np.cross(tcp_axes[1], tcp_axes[2])
    elif missing_index == 1:
        tcp_axes[1] = np.cross(tcp_axes[2], tcp_axes[0])
    else:
        tcp_axes[2] = np.cross(tcp_axes[0], tcp_axes[1])

    tcp_rotation = np.column_stack([axis / max(float(np.linalg.norm(axis)), 1.0e-12) for axis in tcp_axes])
    if float(np.linalg.det(tcp_rotation)) < 0.0:
        tcp_rotation[:, missing_index] *= -1.0
    return tcp_rotation


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


def load_task_points(
    csv_path: Path,
    stride: int,
    max_points: int,
    tcp_spray_axis: str,
    tcp_path_axis: str,
) -> list[TaskPoint]:
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
                        "Use one trajectory_XXX.csv file at a time."
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
            rotation = remap_planner_rotation_to_tcp(
                rotation,
                tcp_spray_axis=tcp_spray_axis,
                tcp_path_axis=tcp_path_axis,
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


def create_pinocchio_model(urdf_path: Path, tcp_frame: str):
    require_pinocchio()
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    frame_id = model.getFrameId(tcp_frame)
    if frame_id >= len(model.frames):
        raise ValueError(f"TCP frame '{tcp_frame}' not found in URDF")
    if model.nq != model.nv:
        raise ValueError(f"expected nq == nv for this controller, got nq={model.nq} nv={model.nv}")
    return model, data, frame_id


def finite_limits(model) -> tuple[np.ndarray, np.ndarray]:
    lower = np.asarray(model.lowerPositionLimit, dtype=float).copy()
    upper = np.asarray(model.upperPositionLimit, dtype=float).copy()
    lower = np.where(np.isfinite(lower), lower, -math.pi)
    upper = np.where(np.isfinite(upper), upper, math.pi)
    return lower, upper


def frame_pose_and_jacobian(model, data, frame_id: int, q: np.ndarray):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pose = data.oMf[frame_id]
    jacobian = pin.computeFrameJacobian(
        model,
        data,
        q,
        frame_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )
    return pose, np.asarray(jacobian, dtype=float)


def task_errors(point: TaskPoint, pose) -> tuple[np.ndarray, np.ndarray, float, float]:
    position_error = np.asarray(point.position - pose.translation, dtype=float).reshape(3)
    rotation_error = np.asarray(pin.log3(point.rotation @ pose.rotation.T), dtype=float).reshape(3)
    position_norm = float(np.linalg.norm(position_error))
    rotation_norm = float(np.linalg.norm(rotation_error))
    return position_error, rotation_error, position_norm, math.degrees(rotation_norm)


def solve_point_dls_ik(
    model,
    data,
    frame_id: int,
    point: TaskPoint,
    q_seed: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    args: argparse.Namespace,
) -> IkPointResult:
    q = np.clip(np.asarray(q_seed, dtype=float).copy(), lower, upper)
    damping = max(float(args.ik_damping), 1.0e-9)
    max_step_rad = math.radians(max(float(args.ik_max_step_deg), 0.1))
    position_tolerance = float(args.position_tolerance)
    orientation_tolerance_deg = float(args.orientation_tolerance_deg)

    best_q = q.copy()
    best_pos_error = math.inf
    best_rot_error_deg = math.inf

    for iteration in range(1, int(args.ik_max_iter) + 1):
        pose, jacobian = frame_pose_and_jacobian(model, data, frame_id, q)
        pos_error, rot_error, pos_norm, rot_deg = task_errors(point, pose)
        score = (
            pos_norm / max(position_tolerance, 1.0e-9)
            + math.radians(rot_deg) / max(math.radians(orientation_tolerance_deg), 1.0e-9)
        )
        if pos_norm < best_pos_error or (abs(pos_norm - best_pos_error) < 1.0e-12 and rot_deg < best_rot_error_deg):
            best_q = q.copy()
            best_pos_error = pos_norm
            best_rot_error_deg = rot_deg
        if pos_norm <= position_tolerance and rot_deg <= orientation_tolerance_deg:
            return IkPointResult(q=q.copy(), iterations=iteration, position_error_m=pos_norm, orientation_error_deg=rot_deg, converged=True)

        weighted_jacobian = jacobian.copy()
        weighted_jacobian[:3, :] *= float(args.ik_position_gain)
        weighted_jacobian[3:, :] *= float(args.ik_orientation_gain)
        weighted_error = np.concatenate(
            [
                float(args.ik_position_gain) * pos_error,
                float(args.ik_orientation_gain) * rot_error,
            ]
        )

        system = weighted_jacobian @ weighted_jacobian.T + (damping * damping) * np.eye(6)
        try:
            dq = weighted_jacobian.T @ np.linalg.solve(system, weighted_error)
        except np.linalg.LinAlgError:
            dq = weighted_jacobian.T @ np.linalg.lstsq(system, weighted_error, rcond=None)[0]

        max_abs_step = float(np.linalg.norm(dq, ord=np.inf))
        if max_abs_step > max_step_rad:
            dq *= max_step_rad / max_abs_step

        accepted = False
        for scale in (1.0, 0.5, 0.25, 0.1):
            candidate = np.clip(q + scale * dq, lower, upper)
            candidate_pose, _candidate_jacobian = frame_pose_and_jacobian(model, data, frame_id, candidate)
            _candidate_pos, _candidate_rot, candidate_pos_norm, candidate_rot_deg = task_errors(point, candidate_pose)
            candidate_score = (
                candidate_pos_norm / max(position_tolerance, 1.0e-9)
                + math.radians(candidate_rot_deg) / max(math.radians(orientation_tolerance_deg), 1.0e-9)
            )
            if candidate_score <= score:
                q = candidate
                accepted = True
                break
        if not accepted:
            q = np.clip(q + 0.1 * dq, lower, upper)

    return IkPointResult(
        q=best_q,
        iterations=int(args.ik_max_iter),
        position_error_m=best_pos_error,
        orientation_error_deg=best_rot_error_deg,
        converged=False,
    )


def q_is_duplicate(q: np.ndarray, existing: list[np.ndarray], duplicate_deg: float) -> bool:
    threshold = math.radians(max(float(duplicate_deg), 1.0e-6))
    return any(float(np.linalg.norm(q - other, ord=np.inf)) <= threshold for other in existing)


def add_unique_seed(
    seeds: list[np.ndarray],
    q: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    duplicate_deg: float,
    max_count: int,
) -> None:
    if len(seeds) >= max_count:
        return
    q = np.clip(np.asarray(q, dtype=float).reshape(-1), lower, upper)
    if not q_is_duplicate(q, seeds, duplicate_deg):
        seeds.append(q)


def build_seed_bank(
    initial_q: np.ndarray,
    previous_candidates: list[IkCandidate],
    lower: np.ndarray,
    upper: np.ndarray,
    args: argparse.Namespace,
) -> list[np.ndarray]:
    max_seeds = max(1, int(args.ik_max_seeds_per_point))
    duplicate_deg = float(args.ik_duplicate_candidate_deg)
    offsets = [np.radians(np.array(offset, dtype=float)) for offset in IK_SEED_OFFSETS_DEG]
    seeds: list[np.ndarray] = []

    for candidate in previous_candidates:
        add_unique_seed(seeds, candidate.q, lower, upper, duplicate_deg, max_seeds)

    for offset in offsets:
        add_unique_seed(seeds, initial_q + offset, lower, upper, duplicate_deg, max_seeds)

    for candidate in previous_candidates[:3]:
        for offset in offsets:
            add_unique_seed(seeds, candidate.q + offset, lower, upper, duplicate_deg, max_seeds)

    return seeds


def candidate_quality_cost(candidate: IkCandidate, model, args: argparse.Namespace) -> float:
    position_ratio = candidate.position_error_m / max(float(args.position_tolerance), 1.0e-9)
    orientation_ratio = candidate.orientation_error_deg / max(float(args.orientation_tolerance_deg), 1.0e-9)
    singular_ratio = float(args.min_singular_value) / max(candidate.min_sigma, 1.0e-9)

    lower, upper = finite_limits(model)
    lower_margin = np.maximum(candidate.q - lower, 1.0e-6)
    upper_margin = np.maximum(upper - candidate.q, 1.0e-6)
    joint_range = np.maximum(upper - lower, 1.0e-6)
    limit_margin = np.minimum(lower_margin, upper_margin)
    limit_ratio = np.maximum(0.0, 0.05 * joint_range / limit_margin - 1.0)

    return (
        float(args.dp_tracking_weight) * (position_ratio * position_ratio + orientation_ratio * orientation_ratio)
        + float(args.dp_singularity_weight) * singular_ratio * singular_ratio
        + float(args.dp_limit_margin_weight) * float(np.sum(limit_ratio * limit_ratio))
    )


def make_candidate_from_result(
    model,
    data,
    frame_id: int,
    point: TaskPoint,
    result: IkPointResult,
    seed_index: int,
) -> IkCandidate:
    pose, jacobian = frame_pose_and_jacobian(model, data, frame_id, result.q)
    _pos_error, _rot_error, pos_norm, rot_deg = task_errors(point, pose)
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    min_sigma = float(np.min(singular_values))
    det_value = float(np.linalg.det(jacobian @ jacobian.T))
    manipulability = math.sqrt(max(0.0, det_value))
    return IkCandidate(
        q=result.q.copy(),
        position_error_m=pos_norm,
        orientation_error_deg=rot_deg,
        min_sigma=min_sigma,
        manipulability=manipulability,
        iterations=result.iterations,
        seed_index=seed_index,
    )


def candidate_is_valid(candidate: IkCandidate, args: argparse.Namespace) -> bool:
    if candidate.position_error_m > float(args.position_tolerance):
        return False
    if candidate.orientation_error_deg > float(args.orientation_tolerance_deg):
        return False
    if candidate.min_sigma < float(args.min_singular_value):
        return False
    if args.min_manipulability > 0.0 and candidate.manipulability < float(args.min_manipulability):
        return False
    return True


def generate_point_candidates(
    model,
    data,
    frame_id: int,
    point: TaskPoint,
    initial_q: np.ndarray,
    previous_candidates: list[IkCandidate],
    lower: np.ndarray,
    upper: np.ndarray,
    args: argparse.Namespace,
) -> list[IkCandidate]:
    seeds = build_seed_bank(initial_q, previous_candidates, lower, upper, args)
    duplicate_deg = float(args.ik_duplicate_candidate_deg)
    candidates: list[IkCandidate] = []
    rejected: list[IkCandidate] = []
    candidate_qs: list[np.ndarray] = []

    for seed_index, seed in enumerate(seeds):
        result = solve_point_dls_ik(
            model=model,
            data=data,
            frame_id=frame_id,
            point=point,
            q_seed=seed,
            lower=lower,
            upper=upper,
            args=args,
        )
        if not result.converged:
            continue
        candidate = make_candidate_from_result(model, data, frame_id, point, result, seed_index)
        if q_is_duplicate(candidate.q, candidate_qs, duplicate_deg):
            continue
        candidate_qs.append(candidate.q)
        if candidate_is_valid(candidate, args):
            candidates.append(candidate)
        else:
            rejected.append(candidate)

    candidates.sort(key=lambda candidate: candidate_quality_cost(candidate, model, args))
    candidates = candidates[: max(1, int(args.ik_max_candidates_per_point))]
    if candidates:
        return candidates

    rejected.sort(
        key=lambda candidate: (
            candidate.position_error_m / max(float(args.position_tolerance), 1.0e-9)
            + candidate.orientation_error_deg / max(float(args.orientation_tolerance_deg), 1.0e-9)
            + float(args.min_singular_value) / max(candidate.min_sigma, 1.0e-9)
        )
    )
    if rejected:
        best = rejected[0]
        raise RuntimeError(
            "no valid DLS IK candidate after FK/singularity filtering: "
            f"pos_err={best.position_error_m:.6f}m "
            f"rot_err={best.orientation_error_deg:.4f}deg "
            f"min_sigma={best.min_sigma:.6e} "
            f"manip={best.manipulability:.6e} "
            f"valid_candidates=0 rejected={len(rejected)} seeds={len(seeds)}"
        )
    raise RuntimeError(f"no converged DLS IK candidate from {len(seeds)} seeds")


def transition_cost(
    prev: IkCandidate,
    cur: IkCandidate,
    dt: float,
    args: argparse.Namespace,
) -> float:
    dt = max(float(dt), 1.0e-9)
    dq = cur.q - prev.q
    velocity = dq / dt
    jump_limit = math.radians(max(float(args.max_joint_step_deg), 1.0e-6))
    velocity_limit = math.radians(max(float(args.max_joint_velocity_deg_s), 1.0e-6))
    jump_ratio = dq / jump_limit
    velocity_ratio = velocity / velocity_limit
    hard_velocity_excess = np.maximum(0.0, np.abs(velocity_ratio) - 1.0)
    return (
        float(args.dp_jump_weight) * float(np.sum(jump_ratio * jump_ratio))
        + float(args.dp_velocity_weight) * float(np.sum(velocity_ratio * velocity_ratio))
        + 1000.0 * float(np.sum(hard_velocity_excess * hard_velocity_excess))
    )


def transition_is_feasible(
    prev: IkCandidate,
    cur: IkCandidate,
    dt: float,
    args: argparse.Namespace,
) -> bool:
    dt = max(float(dt), 1.0e-9)
    dq_deg = max_abs_deg((cur.q - prev.q).reshape(1, -1))
    velocity_deg = max_abs_deg(((cur.q - prev.q) / dt).reshape(1, -1))
    return (
        dq_deg <= float(args.max_joint_step_deg)
        and velocity_deg <= float(args.max_joint_velocity_deg_s)
    )


def acceleration_cost(
    prev_prev: IkCandidate,
    prev: IkCandidate,
    cur: IkCandidate,
    prev_dt: float,
    cur_dt: float,
    args: argparse.Namespace,
) -> float:
    prev_dt = max(float(prev_dt), 1.0e-9)
    cur_dt = max(float(cur_dt), 1.0e-9)
    prev_velocity = (prev.q - prev_prev.q) / prev_dt
    cur_velocity = (cur.q - prev.q) / cur_dt
    acceleration = 2.0 * (cur_velocity - prev_velocity) / (prev_dt + cur_dt)
    acceleration_limit = math.radians(max(float(args.max_joint_acceleration_deg_s2), 1.0e-6))
    acceleration_ratio = acceleration / acceleration_limit
    hard_acceleration_excess = np.maximum(0.0, np.abs(acceleration_ratio) - 1.0)
    return (
        float(args.dp_acceleration_weight) * float(np.sum(acceleration_ratio * acceleration_ratio))
        + 1000.0 * float(np.sum(hard_acceleration_excess * hard_acceleration_excess))
    )


def acceleration_is_feasible(
    prev_prev: IkCandidate,
    prev: IkCandidate,
    cur: IkCandidate,
    prev_dt: float,
    cur_dt: float,
    args: argparse.Namespace,
) -> bool:
    prev_dt = max(float(prev_dt), 1.0e-9)
    cur_dt = max(float(cur_dt), 1.0e-9)
    prev_velocity = (prev.q - prev_prev.q) / prev_dt
    cur_velocity = (cur.q - prev.q) / cur_dt
    acceleration = 2.0 * (cur_velocity - prev_velocity) / (prev_dt + cur_dt)
    return max_abs_deg(acceleration.reshape(1, -1)) <= float(args.max_joint_acceleration_deg_s2)


def select_candidate_path_dp(
    candidate_sets: list[list[IkCandidate]],
    points: list[TaskPoint],
    model,
    args: argparse.Namespace,
) -> list[IkCandidate]:
    count = len(candidate_sets)
    if count == 0:
        raise ValueError("candidate_sets is empty")
    if count == 1:
        return [min(candidate_sets[0], key=lambda candidate: candidate_quality_cost(candidate, model, args))]

    node_costs = [
        [candidate_quality_cost(candidate, model, args) for candidate in candidates]
        for candidates in candidate_sets
    ]

    states: dict[tuple[int, int], float] = {}
    parents: dict[tuple[int, int, int], int] = {}
    first_dt = points[1].t - points[0].t
    for first_index, first in enumerate(candidate_sets[0]):
        for second_index, second in enumerate(candidate_sets[1]):
            if not transition_is_feasible(first, second, first_dt, args):
                continue
            states[(first_index, second_index)] = (
                node_costs[0][first_index]
                + node_costs[1][second_index]
                + transition_cost(first, second, first_dt, args)
            )
    if not states:
        raise RuntimeError("DP found no velocity-feasible connection between points 1 and 2")

    for point_index in range(2, count):
        prev_dt = points[point_index - 1].t - points[point_index - 2].t
        cur_dt = points[point_index].t - points[point_index - 1].t
        new_states: dict[tuple[int, int], float] = {}
        for (prev_prev_index, prev_index), old_cost in states.items():
            prev_prev = candidate_sets[point_index - 2][prev_prev_index]
            prev = candidate_sets[point_index - 1][prev_index]
            for cur_index, cur in enumerate(candidate_sets[point_index]):
                if not transition_is_feasible(prev, cur, cur_dt, args):
                    continue
                if not acceleration_is_feasible(prev_prev, prev, cur, prev_dt, cur_dt, args):
                    continue
                cost = (
                    old_cost
                    + node_costs[point_index][cur_index]
                    + transition_cost(prev, cur, cur_dt, args)
                    + acceleration_cost(prev_prev, prev, cur, prev_dt, cur_dt, args)
                )
                key = (prev_index, cur_index)
                if key not in new_states or cost < new_states[key]:
                    new_states[key] = cost
                    parents[(point_index, prev_index, cur_index)] = prev_prev_index
        states = new_states
        if not states:
            raise RuntimeError(
                f"DP found no jump/velocity/acceleration-feasible candidate connection "
                f"at point {point_index + 1}"
            )

    best_pair, best_cost = min(states.items(), key=lambda item: item[1])
    sequence_indices = [0] * count
    sequence_indices[count - 2], sequence_indices[count - 1] = best_pair
    prev_index, cur_index = best_pair
    for point_index in range(count - 1, 1, -1):
        prev_prev_index = parents[(point_index, prev_index, cur_index)]
        sequence_indices[point_index - 2] = prev_prev_index
        cur_index = prev_index
        prev_index = prev_prev_index

    print(f"[ik] DP selected candidate path cost={best_cost:.6f}")
    return [
        candidate_sets[point_index][candidate_index]
        for point_index, candidate_index in enumerate(sequence_indices)
    ]


def solve_dls_ik_path(
    model,
    data,
    frame_id: int,
    points: list[TaskPoint],
    initial_q: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray:
    lower, upper = finite_limits(model)
    initial_q = np.clip(np.asarray(initial_q, dtype=float), lower, upper)
    candidate_sets: list[list[IkCandidate]] = []
    previous_candidates: list[IkCandidate] = []

    for index, point in enumerate(points):
        try:
            candidates = generate_point_candidates(
                model=model,
                data=data,
                frame_id=frame_id,
                point=point,
                initial_q=initial_q,
                previous_candidates=previous_candidates,
                lower=lower,
                upper=upper,
                args=args,
            )
        except RuntimeError as error:
            raise RuntimeError(f"DLS IK candidate generation failed at point {index + 1}/{len(points)}: {error}") from error

        candidate_sets.append(candidates)
        previous_candidates = candidates
        best_candidate = candidates[0]
        max_iterations = max(candidate.iterations for candidate in candidates)
        if (
            index % max(1, int(args.ik_progress_interval)) == 0
            or index == len(points) - 1
        ):
            print(
                f"[ik] point {index + 1}/{len(points)} "
                f"candidates={len(candidates)} "
                f"best_pos_err={best_candidate.position_error_m:.6f}m "
                f"best_rot_err={best_candidate.orientation_error_deg:.4f}deg "
                f"best_sigma={best_candidate.min_sigma:.6e} "
                f"max_iter={max_iterations}"
            )

    selected_candidates = select_candidate_path_dp(candidate_sets, points, model, args)
    q_path = np.asarray([candidate.q for candidate in selected_candidates], dtype=float)
    max_pos_error = max(candidate.position_error_m for candidate in selected_candidates)
    max_rot_error = max(candidate.orientation_error_deg for candidate in selected_candidates)
    min_sigma = min(candidate.min_sigma for candidate in selected_candidates)
    candidate_counts = [len(candidates) for candidates in candidate_sets]
    print(
        f"[ik] DLS candidate graph solved max_pos_err={max_pos_error:.6f}m "
        f"max_rot_err={max_rot_error:.4f}deg min_sigma={min_sigma:.6e} "
        f"candidate_count_min={min(candidate_counts)} "
        f"candidate_count_max={max(candidate_counts)} "
        f"candidate_count_avg={sum(candidate_counts) / len(candidate_counts):.2f}"
    )
    return q_path


def evaluate_solution(model, data, frame_id: int, points: list[TaskPoint], q_solution: np.ndarray) -> dict[str, float]:
    max_pos_error = 0.0
    max_rot_error = 0.0
    min_manipulability = math.inf
    min_sigma = math.inf

    for point, q in zip(points, q_solution):
        pose, jacobian = frame_pose_and_jacobian(model, data, frame_id, q)
        _pos_error, _rot_error, pos_norm, rot_deg = task_errors(point, pose)
        max_pos_error = max(max_pos_error, pos_norm)
        max_rot_error = max(max_rot_error, rot_deg)
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        min_sigma = min(min_sigma, float(np.min(singular_values)))
        det_value = float(np.linalg.det(jacobian @ jacobian.T))
        min_manipulability = min(min_manipulability, math.sqrt(max(0.0, det_value)))

    return {
        "max_position_error_m": max_pos_error,
        "max_orientation_error_deg": max_rot_error,
        "min_manipulability": min_manipulability,
        "min_sigma": min_sigma,
    }


def check_joint_path(
    model,
    q_solution: np.ndarray,
    qd_solution: np.ndarray,
    qdd_solution: np.ndarray,
    args: argparse.Namespace,
) -> list[str]:
    lower, upper = finite_limits(model)
    violations: list[str] = []

    lower_margin = float(np.min(q_solution - lower.reshape(1, -1)))
    upper_margin = float(np.min(upper.reshape(1, -1) - q_solution))
    if lower_margin < -1.0e-6 or upper_margin < -1.0e-6:
        violations.append(
            f"joint limit violation lower_margin={math.degrees(lower_margin):.3f}deg "
            f"upper_margin={math.degrees(upper_margin):.3f}deg"
        )

    if len(q_solution) > 1:
        max_jump_deg = float(np.max(np.abs(np.degrees(np.diff(q_solution, axis=0)))))
    else:
        max_jump_deg = 0.0
    if max_jump_deg > float(args.max_joint_step_deg):
        violations.append(
            f"joint jump too large max={max_jump_deg:.3f}deg limit={args.max_joint_step_deg:.3f}deg"
        )

    max_qdot_deg = float(np.max(np.abs(np.degrees(qd_solution)))) if qd_solution.size else 0.0
    if max_qdot_deg > float(args.max_joint_velocity_deg_s):
        violations.append(
            f"qdot too large max={max_qdot_deg:.3f}deg/s limit={args.max_joint_velocity_deg_s:.3f}deg/s"
        )

    max_qddot_deg = float(np.max(np.abs(np.degrees(qdd_solution)))) if qdd_solution.size else 0.0
    if max_qddot_deg > float(args.max_joint_acceleration_deg_s2):
        violations.append(
            f"qddot too large max={max_qddot_deg:.3f}deg/s^2 "
            f"limit={args.max_joint_acceleration_deg_s2:.3f}deg/s^2"
        )

    return violations


def max_abs_deg(values_rad: np.ndarray) -> float:
    if values_rad.size == 0:
        return 0.0
    return float(np.max(np.abs(np.degrees(values_rad))))


def joint_limit_margins_deg(model, q_rad: np.ndarray) -> tuple[float, float]:
    lower, upper = finite_limits(model)
    lower_margin = float(np.min(q_rad - lower.reshape(1, -1)))
    upper_margin = float(np.min(upper.reshape(1, -1) - q_rad))
    return math.degrees(lower_margin), math.degrees(upper_margin)


def max_joint_jump_deg(q_rad: np.ndarray) -> float:
    if len(q_rad) <= 1:
        return 0.0
    return max_abs_deg(np.diff(q_rad, axis=0))


def format_check_line(label: str, ok: bool, detail: str) -> str:
    status = "OK" if ok else "FAIL"
    return f"[ik][summary]   {label:<22} {status}  {detail}"


def print_check_line(label: str, ok: bool, detail: str) -> None:
    print(format_check_line(label, ok, detail))


def build_path_summary_lines(
    input_csv: Path,
    model,
    metrics: dict[str, float],
    q_solution: np.ndarray,
    qd_solution: np.ndarray,
    qdd_solution: np.ndarray,
    command_q: np.ndarray,
    command_qd: np.ndarray,
    command_qdd: np.ndarray,
    args: argparse.Namespace,
) -> None:
    source_lower_margin_deg, source_upper_margin_deg = joint_limit_margins_deg(model, q_solution)
    command_lower_margin_deg, command_upper_margin_deg = joint_limit_margins_deg(model, command_q)
    source_jump_deg = max_joint_jump_deg(q_solution)
    command_jump_deg = max_joint_jump_deg(command_q)
    source_qdot_deg = max_abs_deg(qd_solution)
    source_qddot_deg = max_abs_deg(qdd_solution)
    command_qdot_deg = max_abs_deg(command_qd)
    command_qddot_deg = max_abs_deg(command_qdd)

    tracking_position_ok = metrics["max_position_error_m"] <= float(args.position_tolerance)
    tracking_orientation_ok = metrics["max_orientation_error_deg"] <= float(args.orientation_tolerance_deg)
    source_joint_limit_ok = source_lower_margin_deg >= -1.0e-4 and source_upper_margin_deg >= -1.0e-4
    command_joint_limit_ok = command_lower_margin_deg >= -1.0e-4 and command_upper_margin_deg >= -1.0e-4
    source_jump_ok = source_jump_deg <= float(args.max_joint_step_deg)
    command_jump_ok = command_jump_deg <= float(args.max_joint_step_deg)
    source_velocity_ok = source_qdot_deg <= float(args.max_joint_velocity_deg_s)
    command_velocity_ok = command_qdot_deg <= float(args.max_joint_velocity_deg_s)
    source_acceleration_ok = source_qddot_deg <= float(args.max_joint_acceleration_deg_s2)
    command_acceleration_ok = command_qddot_deg <= float(args.max_joint_acceleration_deg_s2)
    singular_value_ok = metrics["min_sigma"] >= float(args.min_singular_value)
    manipulability_ok = (
        args.min_manipulability <= 0.0
        or metrics["min_manipulability"] >= float(args.min_manipulability)
    )

    lines = [f"[ik][summary] {input_csv.name}"]
    lines.append(format_check_line(
        "position tracking",
        tracking_position_ok,
        f"max={metrics['max_position_error_m']:.6f}m limit={args.position_tolerance:.6f}m",
    ))
    lines.append(format_check_line(
        "orientation tracking",
        tracking_orientation_ok,
        f"max={metrics['max_orientation_error_deg']:.4f}deg limit={args.orientation_tolerance_deg:.4f}deg",
    ))
    lines.append(format_check_line(
        "joint limit source",
        source_joint_limit_ok,
        f"lower_margin={source_lower_margin_deg:.3f}deg upper_margin={source_upper_margin_deg:.3f}deg",
    ))
    lines.append(format_check_line(
        "joint limit command",
        command_joint_limit_ok,
        f"lower_margin={command_lower_margin_deg:.3f}deg upper_margin={command_upper_margin_deg:.3f}deg",
    ))
    lines.append(format_check_line(
        "joint jump source",
        source_jump_ok,
        f"max={source_jump_deg:.3f}deg limit={args.max_joint_step_deg:.3f}deg",
    ))
    lines.append(format_check_line(
        "joint jump command",
        command_jump_ok,
        f"max={command_jump_deg:.3f}deg limit={args.max_joint_step_deg:.3f}deg",
    ))
    lines.append(format_check_line(
        "velocity source",
        source_velocity_ok,
        f"max={source_qdot_deg:.3f}deg/s limit={args.max_joint_velocity_deg_s:.3f}deg/s",
    ))
    lines.append(format_check_line(
        "velocity command",
        command_velocity_ok,
        f"max={command_qdot_deg:.3f}deg/s limit={args.max_joint_velocity_deg_s:.3f}deg/s",
    ))
    lines.append(format_check_line(
        "acceleration source",
        source_acceleration_ok,
        f"max={source_qddot_deg:.3f}deg/s^2 limit={args.max_joint_acceleration_deg_s2:.3f}deg/s^2",
    ))
    lines.append(format_check_line(
        "acceleration command",
        command_acceleration_ok,
        f"max={command_qddot_deg:.3f}deg/s^2 limit={args.max_joint_acceleration_deg_s2:.3f}deg/s^2",
    ))
    lines.append(format_check_line(
        "singular value",
        singular_value_ok,
        f"min_sigma={metrics['min_sigma']:.6e} limit={args.min_singular_value:.6e}",
    ))
    if args.min_manipulability > 0.0:
        lines.append(format_check_line(
            "manipulability",
            manipulability_ok,
            f"min={metrics['min_manipulability']:.6e} limit={args.min_manipulability:.6e}",
        ))
    else:
        lines.append(
            f"[ik][summary]   manipulability          INFO  "
            f"min={metrics['min_manipulability']:.6e} threshold=disabled"
        )
    return lines


def print_path_summary(lines: list[str]) -> None:
    for line in lines:
        print(line)


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
        return output / f"{input_csv.stem}_ik_joint.csv"
    return output


def command_output_path_for(input_csv: Path, output: Path, multiple: bool) -> Path:
    output = output.expanduser().resolve()
    if multiple or output.suffix.lower() != ".csv":
        return output / f"{input_csv.stem}_ik_command.csv"
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
        print(f"[python] IK joint CSV #{index}/{len(outputs)}: {csv_path}")
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
        print(f"[python] IK joint trajectory #{index} done")

    print("[python] robot is running. Press Ctrl+C to servo off and exit.")
    while True:
        time.sleep(1.0)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline Pinocchio DLS IK trajectory generation for A0912 Cartesian painting CSVs."
    )
    parser.add_argument("csv", nargs="*", type=Path, help="Input trajectory_XXX.csv file(s).")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--tcp-frame", default=TCP_FRAME)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-points", type=int, default=0)
    parser.add_argument("--initial-joint-deg", default=INITIAL_JOINT_DEG)
    parser.add_argument("--position-tolerance", type=float, default=POSITION_TOLERANCE_M)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=ORIENTATION_TOLERANCE_DEG)
    parser.add_argument("--tcp-spray-axis", default=TCP_SPRAY_AXIS)
    parser.add_argument("--tcp-path-axis", default=TCP_PATH_AXIS)
    parser.add_argument("--max-joint-step-deg", type=float, default=MAX_JOINT_STEP_DEG)
    parser.add_argument("--max-joint-velocity-deg-s", type=float, default=MAX_JOINT_VELOCITY_DEG_S)
    parser.add_argument("--max-joint-acceleration-deg-s2", type=float, default=MAX_JOINT_ACCELERATION_DEG_S2)
    parser.add_argument("--min-manipulability", type=float, default=MIN_MANIPULABILITY)
    parser.add_argument("--min-singular-value", type=float, default=MIN_SINGULAR_VALUE)
    parser.add_argument("--ik-max-iter", type=int, default=IK_MAX_ITER)
    parser.add_argument("--ik-damping", type=float, default=IK_DAMPING)
    parser.add_argument("--ik-max-step-deg", type=float, default=IK_MAX_STEP_DEG)
    parser.add_argument("--ik-position-gain", type=float, default=IK_POSITION_GAIN)
    parser.add_argument("--ik-orientation-gain", type=float, default=IK_ORIENTATION_GAIN)
    parser.add_argument("--ik-progress-interval", type=int, default=IK_PROGRESS_INTERVAL)
    parser.add_argument("--ik-max-seeds-per-point", type=int, default=IK_MAX_SEEDS_PER_POINT)
    parser.add_argument("--ik-max-candidates-per-point", type=int, default=IK_MAX_CANDIDATES_PER_POINT)
    parser.add_argument("--ik-duplicate-candidate-deg", type=float, default=IK_DUPLICATE_CANDIDATE_DEG)
    parser.add_argument("--dp-tracking-weight", type=float, default=DP_TRACKING_WEIGHT)
    parser.add_argument("--dp-jump-weight", type=float, default=DP_JUMP_WEIGHT)
    parser.add_argument("--dp-velocity-weight", type=float, default=DP_VELOCITY_WEIGHT)
    parser.add_argument("--dp-acceleration-weight", type=float, default=DP_ACCELERATION_WEIGHT)
    parser.add_argument("--dp-singularity-weight", type=float, default=DP_SINGULARITY_WEIGHT)
    parser.add_argument("--dp-limit-margin-weight", type=float, default=DP_LIMIT_MARGIN_WEIGHT)
    parser.add_argument("--execute", action="store_true", help="Send generated IK joint CSV to the robot after solving.")
    parser.add_argument("--allow-check-failure", action="store_true", help="Do not abort execution when offline path checks fail.")
    parser.add_argument("--command-dt", type=float, default=COMMAND_TIME_SEC)
    parser.add_argument("--command-time-sec", type=float, default=COMMAND_TIME_SEC)
    parser.add_argument("--joint-position-gain", type=float, default=JOINT_POSITION_GAIN)
    parser.add_argument(
        "--preposition-max-joint-velocity-deg-s",
        type=float,
        default=PREPOSITION_MAX_JOINT_VELOCITY_DEG_S,
    )
    parser.add_argument(
        "--preposition-max-joint-acceleration-deg-s2",
        type=float,
        default=PREPOSITION_MAX_JOINT_ACCELERATION_DEG_S2,
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    input_csvs = [path.expanduser().resolve() for path in args.csv]
    if not input_csvs:
        input_csvs = [path.resolve() for path in DEFAULT_TRAJECTORY_CSV]
    if not input_csvs:
        raise ValueError("pass one or more trajectory CSV files, or edit DEFAULT_TRAJECTORY_CSV")
    for csv_path in input_csvs:
        if not csv_path.is_file():
            raise FileNotFoundError(csv_path)
    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(urdf_path)

    parse_signed_axis(args.tcp_spray_axis)
    parse_signed_axis(args.tcp_path_axis)
    if parse_signed_axis(args.tcp_spray_axis)[0] == parse_signed_axis(args.tcp_path_axis)[0]:
        raise ValueError("--tcp-spray-axis and --tcp-path-axis must use different axes")

    model, data, frame_id = create_pinocchio_model(urdf_path, args.tcp_frame)
    if model.nq != 6:
        raise ValueError(f"controller currently expects 6 joints, but Pinocchio model has nq={model.nq}")

    initial_q = np.radians(np.array(parse_float_list(args.initial_joint_deg, model.nq, "--initial-joint-deg")))
    seed_initial_q = initial_q.copy()

    joint_names = [model.names[index] for index in range(1, model.njoints)]
    print(f"[ik] URDF: {urdf_path}")
    print(f"[ik] joints: {', '.join(joint_names)}")
    print(f"[ik] TCP frame: {args.tcp_frame}")
    print(f"[ik] TCP spray axis: {args.tcp_spray_axis} path axis: {args.tcp_path_axis}")
    print(f"[ik] output: {args.output}")

    command_outputs = []
    pending_violations: list[str] = []
    path_summaries: list[list[str]] = []
    for csv_index, input_csv in enumerate(input_csvs, start=1):
        print(f"[ik] input #{csv_index}/{len(input_csvs)}: {input_csv}")
        points = load_task_points(
            input_csv,
            stride=args.stride,
            max_points=args.max_points,
            tcp_spray_axis=args.tcp_spray_axis,
            tcp_path_axis=args.tcp_path_axis,
        )
        print(
            f"[ik] loaded points={len(points)} duration={points[-1].t:.3f}s "
            f"stride={args.stride} max_points={args.max_points}"
        )

        q_solution = solve_dls_ik_path(model, data, frame_id, points, seed_initial_q, args)
        seed_initial_q = q_solution[-1].copy()
        metrics = evaluate_solution(model, data, frame_id, points, q_solution)
        times = np.array([point.t for point in points], dtype=float)
        arc_lengths = np.array([point.s for point in points], dtype=float)
        qd_solution, qdd_solution = joint_motion_from_arc_profile(points, q_solution)
        source_violations = check_joint_path(model, q_solution, qd_solution, qdd_solution, args)
        violations = [f"source: {message}" for message in source_violations]
        if args.min_manipulability > 0.0 and metrics["min_manipulability"] < args.min_manipulability:
            violations.append(
                f"manipulability too low min={metrics['min_manipulability']:.6e} "
                f"limit={args.min_manipulability:.6e}"
            )
        if metrics["min_sigma"] < args.min_singular_value:
            violations.append(
                f"singular value too low min_sigma={metrics['min_sigma']:.6e} "
                f"limit={args.min_singular_value:.6e}"
            )
        pending_violations.extend([f"{input_csv.name}: {message}" for message in violations])

        print(
            "[ik] metrics "
            f"max_pos_err={metrics['max_position_error_m']:.6f}m "
            f"max_rot_err={metrics['max_orientation_error_deg']:.4f}deg "
            f"min_manip={metrics['min_manipulability']:.6e} "
            f"min_sigma={metrics['min_sigma']:.6e} "
            f"max_qdot={np.max(np.abs(np.degrees(qd_solution))):.3f}deg/s "
            f"max_qddot={np.max(np.abs(np.degrees(qdd_solution))):.3f}deg/s^2"
        )
        for violation in source_violations:
            print(f"[ik] check warning: source: {violation}")

        output_csv = output_path_for(input_csv, args.output, multiple=len(input_csvs) > 1)
        write_joint_csv(output_csv, times, arc_lengths, q_solution, qd_solution, qdd_solution)
        print(f"[ik] saved IK joint trajectory: {output_csv}")

        command_times, command_arc, command_q, command_qd, command_qdd = build_command_trajectory(
            points,
            q_solution,
            args.command_dt,
        )
        command_csv = command_output_path_for(input_csv, args.output, multiple=len(input_csvs) > 1)
        write_joint_csv(command_csv, command_times, command_arc, command_q, command_qd, command_qdd)
        command_qd_deg = np.degrees(command_qd)
        command_qdd_deg = np.degrees(command_qdd)
        command_violations = check_joint_path(model, command_q, command_qd, command_qdd, args)
        pending_violations.extend(
            [f"{input_csv.name}: command: {message}" for message in command_violations]
        )
        print(
            f"[ik] saved {args.command_dt:.4f}s speedj command reference: {command_csv} "
            f"points={len(command_times)} max_qdot={np.max(np.abs(command_qd_deg)):.3f}deg/s "
            f"max_qddot={np.max(np.abs(command_qdd_deg)):.3f}deg/s^2"
        )
        for violation in command_violations:
            print(f"[ik] check warning: command: {violation}")
        path_summaries.append(build_path_summary_lines(
            input_csv=input_csv,
            model=model,
            metrics=metrics,
            q_solution=q_solution,
            qd_solution=qd_solution,
            qdd_solution=qdd_solution,
            command_q=command_q,
            command_qd=command_qd,
            command_qdd=command_qdd,
            args=args,
        ))
        command_outputs.append(command_csv)

    if path_summaries:
        print("[ik][summary] final result")
        for index, summary_lines in enumerate(path_summaries, start=1):
            if index > 1:
                print("[ik][summary]")
            print_path_summary(summary_lines)

    if args.execute:
        if pending_violations and not args.allow_check_failure:
            joined = "\n  - ".join(pending_violations)
            raise RuntimeError(
                "offline IK path checks failed. Refusing to execute.\n"
                f"  - {joined}\n"
                "Use --allow-check-failure only if you intentionally want the controller-side checks to decide."
            )
        run_robot(command_outputs, args)
    else:
        print("[ik] dry run complete. Add --execute to track the generated command CSV on the robot.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
