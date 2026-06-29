#!/usr/bin/env python3

import heapq

import numpy as np
from scipy.spatial import cKDTree


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


def _estimate_surface_normals_work(points_work, neighbor_count):
    pts = np.asarray(points_work, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.empty((0, 3), float)
    if len(pts) < 3:
        return np.repeat(np.array([[0.0, 0.0, 1.0]], dtype=float), len(pts), axis=0)

    k = min(max(int(neighbor_count), 3), len(pts))
    tree = cKDTree(pts)
    _, indices = tree.query(pts, k=k)
    if indices.ndim == 1:
        indices = indices.reshape(-1, 1)

    normals = np.zeros_like(pts)
    for idx, neighbors in enumerate(indices):
        local = pts[np.asarray(neighbors, dtype=int)]
        local = local - np.mean(local, axis=0, keepdims=True)
        cov = local.T @ local
        try:
            _, vecs = np.linalg.eigh(cov)
            normal = vecs[:, 0]
        except np.linalg.LinAlgError:
            normal = np.array([0.0, 0.0, 1.0], dtype=float)
        normal = _safe_normalize(normal)
        if normal[2] < 0.0:
            normal = -normal
        normals[idx] = normal
    return normals


def _build_knn_adjacency(points, neighbor_count, edge_max_factor):
    pts = np.asarray(points, float)
    if pts.ndim != 2 or len(pts) < 2:
        return [[] for _ in range(len(pts))]

    neighbor_count = min(max(int(neighbor_count), 1), len(pts) - 1)
    tree = cKDTree(pts)
    distances, indices = tree.query(pts, k=neighbor_count + 1)
    if distances.ndim == 1:
        distances = distances.reshape(-1, 1)
        indices = indices.reshape(-1, 1)

    finite_distances = distances[:, 1:][np.isfinite(distances[:, 1:])]
    if len(finite_distances) == 0 or float(edge_max_factor) <= 0.0:
        max_edge = np.inf
    else:
        max_edge = float(np.median(finite_distances)) * float(edge_max_factor)

    adjacency = [[] for _ in range(len(pts))]
    used_edges = set()
    for idx in range(len(pts)):
        for dist, nei in zip(distances[idx, 1:], indices[idx, 1:]):
            if not np.isfinite(dist) or float(dist) > max_edge:
                continue
            a = int(idx)
            b = int(nei)
            if a == b:
                continue
            key = (min(a, b), max(a, b))
            if key in used_edges:
                continue
            used_edges.add(key)
            weight = float(dist)
            adjacency[a].append((b, weight))
            adjacency[b].append((a, weight))
    return adjacency


def _multi_source_dijkstra(adjacency, source_indices):
    n = len(adjacency)
    dist = np.full(n, np.inf, dtype=float)
    heap = []

    for source in np.asarray(source_indices, dtype=int).reshape(-1):
        if source < 0 or source >= n or np.isfinite(dist[source]):
            continue
        dist[source] = 0.0
        heapq.heappush(heap, (0.0, int(source)))

    while heap:
        cur_dist, node = heapq.heappop(heap)
        if cur_dist > dist[node] + 1e-12:
            continue
        for nei, weight in adjacency[node]:
            cand = cur_dist + float(weight)
            if cand + 1e-12 < dist[nei]:
                dist[nei] = cand
                heapq.heappush(heap, (cand, int(nei)))
    return dist


def _geodesic_seed_indices(points_work, slice_axis_idx, descending, seed_width):
    coords = np.asarray(points_work, float)[:, int(slice_axis_idx)]
    if len(coords) == 0:
        return np.empty(0, dtype=int)

    width = max(float(seed_width), 0.0)
    if descending:
        seed = np.flatnonzero(coords >= float(np.max(coords)) - width)
    else:
        seed = np.flatnonzero(coords <= float(np.min(coords)) + width)

    min_seed_count = min(len(coords), max(5, int(np.ceil(len(coords) * 0.01))))
    if len(seed) < min_seed_count:
        order = np.argsort(coords)
        seed = order[-min_seed_count:] if descending else order[:min_seed_count]
    return np.asarray(seed, dtype=int)


def _generate_axis_slice_profiles(
    pw,
    work_to_world,
    slice_cfg,
    spline_spacing,
    spline_half_width,
    spline_start_offset,
):
    slice_axis_idx = int(slice_cfg["slice_axis"])
    descending = bool(slice_cfg["descending"])

    slice_min = float(np.min(pw[:, slice_axis_idx]))
    slice_max = float(np.max(pw[:, slice_axis_idx]))
    if (not np.isfinite([slice_min, slice_max]).all()) or (slice_max < slice_min):
        return []

    if descending:
        first_slice = slice_max - spline_start_offset
        slice_positions = np.arange(first_slice, slice_min - 1e-12, -spline_spacing)
    else:
        first_slice = slice_min + spline_start_offset
        slice_positions = np.arange(first_slice, slice_max + 1e-12, spline_spacing)

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
        slice_profiles.append(
            {
                "slice_position": float(slice_center),
                "slice_points_work": slice_points_work,
                "slice_points_world": work_to_world(slice_points_work),
                "slicing_method": "axis",
            }
        )
    return slice_profiles


def _generate_geodesic_slice_profiles(
    pw,
    work_to_world,
    slice_cfg,
    spline_spacing,
    spline_half_width,
    spline_start_offset,
    geodesic_neighbor_count,
    geodesic_normal_neighbor_count,
    geodesic_edge_max_factor,
    geodesic_seed_width,
    geodesic_band_half_width,
    geodesic_min_band_points,
):
    slice_axis_idx = int(slice_cfg["slice_axis"])
    descending = bool(slice_cfg["descending"])

    normals_work = _estimate_surface_normals_work(
        pw,
        neighbor_count=geodesic_normal_neighbor_count,
    )
    adjacency = _build_knn_adjacency(
        pw,
        neighbor_count=geodesic_neighbor_count,
        edge_max_factor=geodesic_edge_max_factor,
    )
    seed_width = float(geodesic_seed_width)
    if seed_width <= 0.0:
        seed_width = float(spline_half_width)
    seed_indices = _geodesic_seed_indices(
        pw,
        slice_axis_idx=slice_axis_idx,
        descending=descending,
        seed_width=seed_width,
    )
    if len(seed_indices) == 0:
        return []

    geodesic_distances = _multi_source_dijkstra(adjacency, seed_indices)
    finite = np.isfinite(geodesic_distances)
    if not np.any(finite):
        return []

    max_distance = float(np.max(geodesic_distances[finite]))
    first_distance = max(float(spline_start_offset), 0.0)
    if max_distance <= first_distance + 1e-9:
        return []

    levels = np.arange(first_distance, max_distance + 1e-12, float(spline_spacing))
    band_half_width = float(geodesic_band_half_width)
    if band_half_width <= 0.0:
        band_half_width = float(spline_half_width)
    min_band_points = max(int(geodesic_min_band_points), 2)

    slice_profiles = []
    for level in levels:
        in_band = finite & (np.abs(geodesic_distances - float(level)) <= band_half_width)
        indices = np.flatnonzero(in_band)
        if len(indices) < min_band_points:
            continue
        slice_points_work = pw[indices].copy()
        slice_profiles.append(
            {
                "slice_position": float(level),
                "slice_points_work": slice_points_work,
                "slice_points_world": work_to_world(slice_points_work),
                "slice_normals_work": normals_work[indices].copy(),
                "slice_geodesic_distances": geodesic_distances[indices].copy(),
                "slicing_method": "geodesic",
            }
        )
    return slice_profiles


def generate_surface_spline_rows(
    point_cloud,
    paint_dir,
    scan_dir,
    spline_spacing=0.06,
    spline_point_spacing=0.02,
    spline_half_width=0.002,
    spline_start_side="top",
    spline_start_offset=0.0,
    slicing_method="axis",
    geodesic_neighbor_count=12,
    geodesic_normal_neighbor_count=20,
    geodesic_edge_max_factor=4.0,
    geodesic_seed_width=0.0,
    geodesic_band_half_width=0.0,
    geodesic_min_band_points=8,
):
    def _empty_result(points_used, r_work=None, origin=None, view_dir=None, surface_dir=None):
        return {
            "point_cloud_used": np.asarray(points_used, float),
            "work_frame_world": r_work,
            "work_origin_world": origin,
            "view_dir_world": view_dir,
            "surface_dir_world": surface_dir,
            "slicing_method": str(slicing_method),
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
    spline_spacing = max(float(spline_spacing), 1e-4)
    spline_point_spacing = max(float(spline_point_spacing), 1e-4)
    spline_half_width = max(float(spline_half_width), 1e-6)
    spline_start_offset = max(float(spline_start_offset), 0.0)
    method = str(slicing_method).strip().lower()

    if method == "axis":
        slice_profiles = _generate_axis_slice_profiles(
            pw,
            work_to_world=work_to_world,
            slice_cfg=slice_cfg,
            spline_spacing=spline_spacing,
            spline_half_width=spline_half_width,
            spline_start_offset=spline_start_offset,
        )
    elif method == "geodesic":
        slice_profiles = _generate_geodesic_slice_profiles(
            pw,
            work_to_world=work_to_world,
            slice_cfg=slice_cfg,
            spline_spacing=spline_spacing,
            spline_half_width=spline_half_width,
            spline_start_offset=spline_start_offset,
            geodesic_neighbor_count=geodesic_neighbor_count,
            geodesic_normal_neighbor_count=geodesic_normal_neighbor_count,
            geodesic_edge_max_factor=geodesic_edge_max_factor,
            geodesic_seed_width=geodesic_seed_width,
            geodesic_band_half_width=geodesic_band_half_width,
            geodesic_min_band_points=geodesic_min_band_points,
        )
    else:
        raise ValueError("slicing_method must be one of: axis, geodesic")

    return {
        "point_cloud_used": p_used,
        "work_frame_world": r_work,
        "work_origin_world": c,
        "view_dir_world": view_dir,
        "surface_dir_world": surface_dir,
        "slicing_method": method,
        "spline_start_side": str(slice_cfg["side"]),
        "spline_start_offset": float(spline_start_offset),
        "spline_point_spacing": float(spline_point_spacing),
        "slice_axis_idx": int(slice_axis_idx),
        "slice_profiles": slice_profiles,
    }


__all__ = ["generate_surface_spline_rows"]
