#!/usr/bin/env python3

from dataclasses import dataclass, field

import numpy as np


Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


@dataclass(slots=True)
class Waypoint:
    position: Vec3
    orientation: Quat
    paint: bool
    time_from_start: float


@dataclass(slots=True)
class Path:
    waypoints: list[Waypoint] = field(default_factory=list)


@dataclass(slots=True)
class PaintPath:
    frame_id: str
    paths: list[Path] = field(default_factory=list)


def _as_vec3(values) -> Vec3:
    arr = np.asarray(values, float).reshape(3)
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _as_quat(values) -> Quat:
    arr = np.asarray(values, float).reshape(4)
    return (float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))


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


def structure_paint_path(
    paint_spline_result,
    frame_id="world",
    raster_zigzag=None,
):
    if raster_zigzag is None:
        raster_zigzag = bool(paint_spline_result.get("paint_spline_raster_zigzag", True))

    rows = list(paint_spline_result.get("paint_spline_rows", []))
    orientation_rows = list(
        paint_spline_result.get("paint_spline_orientation_rows_xyzw", [])
    )
    paint_mask_rows = list(paint_spline_result.get("paint_spline_paint_mask_rows", []))
    time_rows = list(paint_spline_result.get("paint_spline_time_rows", []))

    paths = []
    row_count = min(len(rows), len(orientation_rows), len(paint_mask_rows))
    for path_index, row_index in enumerate(
        _ordered_row_indices(paint_spline_result, row_count)
    ):
        row = rows[row_index]
        orientation_row = orientation_rows[row_index]
        paint_mask_row = paint_mask_rows[row_index]
        time_row = time_rows[row_index] if row_index < len(time_rows) else []
        points = np.asarray(row, float)
        orientations = np.asarray(orientation_row, float)
        paint_mask = np.asarray(paint_mask_row, dtype=bool).reshape(-1)
        times = np.asarray(time_row, float).reshape(-1)
        if (
            points.ndim != 2
            or points.shape[1] != 3
            or orientations.ndim != 2
            or orientations.shape != (len(points), 4)
            or len(paint_mask) != len(points)
            or len(points) == 0
        ):
            continue
        if len(times) != len(points) or not np.all(np.isfinite(times)):
            times = np.zeros(len(points), dtype=float)
        times = times - float(times[0])
        times = np.maximum.accumulate(times)
        if bool(raster_zigzag) and path_index % 2 == 1:
            points = points[::-1]
            orientations = orientations[::-1]
            paint_mask = paint_mask[::-1]
            times = float(times[-1]) - times[::-1]

        waypoints = [
            Waypoint(
                position=_as_vec3(point),
                orientation=_as_quat(orientation),
                paint=bool(paint),
                time_from_start=float(time_from_start),
            )
            for point, orientation, paint, time_from_start in zip(
                points,
                orientations,
                paint_mask,
                times,
            )
        ]
        paths.append(Path(waypoints=waypoints))

    paint_path = PaintPath(
        frame_id=str(frame_id),
        paths=paths,
    )
    return paint_path


__all__ = [
    "Vec3",
    "Quat",
    "Waypoint",
    "Path",
    "PaintPath",
    "structure_paint_path",
]
