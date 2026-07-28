#!/usr/bin/env python3

import numpy as np


def _safe_normalize(v):
    v = np.asarray(v, float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return v / n


def _canonical_direction(v):
    return _safe_normalize(v)


def _spline_scan_config(spline_start_side):
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
    if side == "top":
        return {"side": side, "slice_axis": 1, "row_axis": 0, "descending": True}
    if side == "bottom":
        return {"side": side, "slice_axis": 1, "row_axis": 0, "descending": False}
    if side == "left":
        return {"side": side, "slice_axis": 0, "row_axis": 1, "descending": False}
    if side == "right":
        return {"side": side, "slice_axis": 0, "row_axis": 1, "descending": True}
    raise ValueError(
        "spline_start_side must be one of: top, bottom, left, right"
    )


def _build_work_frame(points_world, paint_dir_world, scan_dir_world):
    p = np.asarray(points_world, float)
    c = np.mean(p, axis=0)

    view_dir = _safe_normalize(paint_dir_world)   # gun -> part
    surface_dir = -view_dir                       # part -> gun

    row_dir = _canonical_direction(scan_dir_world)
    x_axis = row_dir - np.dot(row_dir, surface_dir) * surface_dir
    if np.linalg.norm(x_axis) < 1e-9:
        tmp = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(np.dot(tmp, surface_dir)) > 0.9:
            tmp = np.array([0.0, 1.0, 0.0], dtype=float)
        x_axis = tmp - np.dot(tmp, surface_dir) * surface_dir
    x_axis = _safe_normalize(x_axis)

    y_axis = _safe_normalize(np.cross(surface_dir, x_axis))
    x_axis = _safe_normalize(np.cross(y_axis, surface_dir))

    r_work = np.column_stack([x_axis, y_axis, surface_dir])
    if np.linalg.det(r_work) < 0.0:
        r_work[:, 1] *= -1.0

    return c, r_work, view_dir, surface_dir


def generate_surface_spline_rows(
    point_cloud,
    paint_dir,
    scan_dir,
    spline_spacing=0.06,
    spline_point_spacing=0.02,
    spline_half_width=0.002,
    spline_start_side="top",
    spline_start_offset=0.0,
):
    def _empty_result(points_used, r_work=None, origin=None, view_dir=None, surface_dir=None):
        return {
            "point_cloud_used": np.asarray(points_used, float),
            "work_frame_world": r_work,
            "work_origin_world": origin,
            "view_dir_world": view_dir,
            "surface_dir_world": surface_dir,
            "spline_start_side": str(spline_start_side),
            "spline_start_offset": float(spline_start_offset),
            "spline_point_spacing": float(spline_point_spacing),
            "slice_profiles": [],
        }

    p = np.asarray(point_cloud, float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 2:
        return _empty_result(p)

    p_used = p.copy()
    if len(p_used) < 2:
        return _empty_result(p_used)

    c, r_work, view_dir, surface_dir = _build_work_frame(
        p_used,
        paint_dir,
        scan_dir,
    )
    work_to_world = lambda x: np.asarray(x, float) @ r_work.T + c
    pw = (np.asarray(p_used, float) - c) @ r_work

    slice_cfg = _spline_scan_config(spline_start_side)
    slice_axis_idx = int(slice_cfg["slice_axis"])
    row_axis_idx = int(slice_cfg["row_axis"])
    descending = bool(slice_cfg["descending"])
    spline_spacing = max(float(spline_spacing), 1e-4)
    spline_point_spacing = max(float(spline_point_spacing), 1e-4)
    spline_half_width = max(float(spline_half_width), 1e-6)
    spline_start_offset = max(float(spline_start_offset), 0.0)

    slice_min = float(np.min(pw[:, slice_axis_idx]))
    slice_max = float(np.max(pw[:, slice_axis_idx]))
    if (not np.isfinite([slice_min, slice_max]).all()) or (slice_max < slice_min):
        return _empty_result(
            p_used,
            r_work=r_work,
            origin=c,
            view_dir=view_dir,
            surface_dir=surface_dir,
        )

    if descending:
        first_slice = slice_max - spline_start_offset
        slice_positions = np.arange(first_slice, slice_min - 1e-12, -spline_spacing)
    else:
        first_slice = slice_min + spline_start_offset
        slice_positions = np.arange(first_slice, slice_max + 1e-12, spline_spacing)

    if len(slice_positions) == 0:
        return _empty_result(
            p_used,
            r_work=r_work,
            origin=c,
            view_dir=view_dir,
            surface_dir=surface_dir,
        )

    slice_profiles = []

    for slice_center in slice_positions:
        in_slice = (
            np.abs(pw[:, slice_axis_idx] - float(slice_center))
            <= float(spline_half_width)
        )
        slice_pts = pw[in_slice]
        if len(slice_pts) < 2:
            continue

        slice_points_work = np.asarray(slice_pts, float).copy()
        slice_points_work[:, slice_axis_idx] = float(slice_center)
        slice_points_world = work_to_world(slice_points_work)
        slice_profiles.append(
            {
                "slice_position": float(slice_center),
                "slice_points_work": slice_points_work,
                "slice_points_world": slice_points_world,
            }
        )

    return {
        "point_cloud_used": p_used,
        "work_frame_world": r_work,
        "work_origin_world": c,
        "view_dir_world": view_dir,
        "surface_dir_world": surface_dir,
        "spline_start_side": str(slice_cfg["side"]),
        "spline_start_offset": float(spline_start_offset),
        "spline_point_spacing": float(spline_point_spacing),
        "slice_profiles": slice_profiles,
    }


__all__ = ["generate_surface_spline_rows"]
