#!/usr/bin/env python3

from dataclasses import dataclass, field

import numpy as np


Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


@dataclass(slots=True)
class TrajectoryPoint:
    t: float
    s: float
    position: Vec3
    orientation: Quat
    linear_velocity: Vec3
    angular_velocity: Vec3
    path_speed: float
    path_acceleration: float
    paint: bool


@dataclass(slots=True)
class TrajectoryRow:
    row_index: int
    points: list[TrajectoryPoint] = field(default_factory=list)


@dataclass(slots=True)
class PaintTrajectory:
    frame_id: str
    control_dt: float
    desired_speed: float
    max_acceleration: float
    rows: list[TrajectoryRow] = field(default_factory=list)


PaintingTrajectory = PaintTrajectory


def _as_vec3(values) -> Vec3:
    arr = np.asarray(values, float).reshape(3)
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _as_quat(values) -> Quat:
    arr = np.asarray(values, float).reshape(4)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    arr = arr / norm
    return (float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))


def _polyline_arclength(points):
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.zeros(0, dtype=float)
    if len(pts) == 1:
        return np.zeros(1, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _spline_start_descending(spline_start_side) -> bool:
    side = str(spline_start_side).strip().lower()
    aliases = {
        "top": "top",
        "from_top": "top",
        "top_to_bottom": "top",
        "bottom": "bottom",
        "from_bottom": "bottom",
        "bottom_to_top": "bottom",
        "left": "left",
        "from_left": "left",
        "left_to_right": "left",
        "right": "right",
        "from_right": "right",
        "right_to_left": "right",
    }
    side = aliases.get(side, side)
    if side in ("top", "right"):
        return True
    if side in ("bottom", "left"):
        return False
    raise ValueError("spline_start_side must be one of: top, bottom, left, right")


def _ordered_row_indices(paint_spline_result, row_count: int) -> list[int]:
    slice_positions = np.asarray(
        paint_spline_result.get("paint_spline_row_slice_positions", []),
        dtype=float,
    ).reshape(-1)
    if row_count <= 0 or len(slice_positions) != row_count:
        return list(range(row_count))

    descending = _spline_start_descending(
        paint_spline_result.get("spline_start_side", "top")
    )
    finite = np.isfinite(slice_positions)
    return sorted(
        range(row_count),
        key=lambda idx: (
            not bool(finite[idx]),
            -float(slice_positions[idx]) if descending else float(slice_positions[idx]),
            idx,
        ),
    )


def _array_rows(source, primary_key, fallback_key=None):
    rows = source.get(primary_key, None)
    if rows is None and fallback_key is not None:
        rows = source.get(fallback_key, None)
    if rows is None:
        return []
    return list(rows)


def _continuous_quaternions(orientations):
    quats = np.asarray(orientations, float).copy()
    if quats.ndim != 2 or quats.shape[1] != 4:
        return quats
    for idx in range(len(quats)):
        norm = float(np.linalg.norm(quats[idx]))
        if norm <= 1e-12:
            quats[idx] = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        else:
            quats[idx] = quats[idx] / norm
        if idx > 0 and float(np.dot(quats[idx], quats[idx - 1])) < 0.0:
            quats[idx] = -quats[idx]
    return quats


def _row_arrays(paint_spline_result, row_index, control_dt):
    position_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_position_rows",
        "paint_spline_rows",
    )
    orientation_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_orientation_rows_xyzw",
        "paint_spline_orientation_rows_xyzw",
    )
    paint_mask_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_paint_mask_rows",
        "paint_spline_paint_mask_rows",
    )

    positions = np.asarray(position_rows[row_index], float)
    orientations = _continuous_quaternions(np.asarray(orientation_rows[row_index], float))
    paint_mask = np.asarray(paint_mask_rows[row_index], dtype=bool).reshape(-1)
    if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) == 0:
        return None
    if orientations.ndim != 2 or orientations.shape != (len(positions), 4):
        return None
    if len(paint_mask) != len(positions):
        return None

    count = len(positions)
    time_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_time_rows",
        "paint_spline_time_rows",
    )
    arc_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_arc_length_rows",
        "paint_spline_arc_length_rows",
    )
    linear_velocity_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_linear_velocity_rows_world",
        "paint_spline_linear_velocity_rows_world",
    )
    angular_velocity_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_angular_velocity_rows_world",
        "paint_spline_angular_velocity_rows_world",
    )
    path_speed_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_path_speed_rows",
        "paint_spline_path_speed_rows",
    )
    path_acceleration_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_path_acceleration_rows",
        "paint_spline_path_acceleration_rows",
    )

    times = (
        np.asarray(time_rows[row_index], float).reshape(-1)
        if row_index < len(time_rows)
        else np.arange(count, dtype=float) * float(control_dt)
    )
    arcs = (
        np.asarray(arc_rows[row_index], float).reshape(-1)
        if row_index < len(arc_rows)
        else _polyline_arclength(positions)
    )
    linear_velocities = (
        np.asarray(linear_velocity_rows[row_index], float)
        if row_index < len(linear_velocity_rows)
        else np.zeros((count, 3), dtype=float)
    )
    angular_velocities = (
        np.asarray(angular_velocity_rows[row_index], float)
        if row_index < len(angular_velocity_rows)
        else np.zeros((count, 3), dtype=float)
    )
    path_speeds = (
        np.asarray(path_speed_rows[row_index], float).reshape(-1)
        if row_index < len(path_speed_rows)
        else np.zeros(count, dtype=float)
    )
    path_accelerations = (
        np.asarray(path_acceleration_rows[row_index], float).reshape(-1)
        if row_index < len(path_acceleration_rows)
        else np.zeros(count, dtype=float)
    )

    if (
        len(times) != count
        or len(arcs) != count
        or linear_velocities.shape != (count, 3)
        or angular_velocities.shape != (count, 3)
        or len(path_speeds) != count
        or len(path_accelerations) != count
    ):
        return None

    return {
        "times": times,
        "arcs": arcs,
        "positions": positions,
        "orientations": orientations,
        "linear_velocities": linear_velocities,
        "angular_velocities": angular_velocities,
        "path_speeds": path_speeds,
        "path_accelerations": path_accelerations,
        "paint_mask": paint_mask,
    }


def structure_painting_trajectory(
    paint_spline_result,
    frame_id="world",
):
    control_dt = float(paint_spline_result.get("paint_trajectory_control_dt", 0.0))
    desired_speed = float(paint_spline_result.get("paint_trajectory_desired_speed", 0.0))
    max_acceleration = float(
        paint_spline_result.get("paint_trajectory_max_acceleration", 0.0)
    )

    position_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_position_rows",
        "paint_spline_rows",
    )
    orientation_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_orientation_rows_xyzw",
        "paint_spline_orientation_rows_xyzw",
    )
    paint_mask_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_paint_mask_rows",
        "paint_spline_paint_mask_rows",
    )
    row_count = min(len(position_rows), len(orientation_rows), len(paint_mask_rows))

    rows = []
    for trajectory_row_index, source_row_index in enumerate(
        _ordered_row_indices(paint_spline_result, row_count)
    ):
        arrays = _row_arrays(
            paint_spline_result,
            source_row_index,
            control_dt=control_dt,
        )
        if arrays is None:
            continue

        points = [
            TrajectoryPoint(
                t=float(t),
                s=float(s),
                position=_as_vec3(position),
                orientation=_as_quat(orientation),
                linear_velocity=_as_vec3(linear_velocity),
                angular_velocity=_as_vec3(angular_velocity),
                path_speed=float(path_speed),
                path_acceleration=float(path_acceleration),
                paint=bool(paint),
            )
            for (
                t,
                s,
                position,
                orientation,
                linear_velocity,
                angular_velocity,
                path_speed,
                path_acceleration,
                paint,
            ) in zip(
                arrays["times"],
                arrays["arcs"],
                arrays["positions"],
                arrays["orientations"],
                arrays["linear_velocities"],
                arrays["angular_velocities"],
                arrays["path_speeds"],
                arrays["path_accelerations"],
                arrays["paint_mask"],
            )
        ]
        rows.append(TrajectoryRow(row_index=trajectory_row_index, points=points))

    return PaintTrajectory(
        frame_id=str(frame_id),
        control_dt=control_dt,
        desired_speed=desired_speed,
        max_acceleration=max_acceleration,
        rows=rows,
    )


__all__ = [
    "Vec3",
    "Quat",
    "TrajectoryPoint",
    "TrajectoryRow",
    "PaintTrajectory",
    "PaintingTrajectory",
    "structure_painting_trajectory",
]
