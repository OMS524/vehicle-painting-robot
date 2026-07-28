import numpy as np
from scipy.spatial import cKDTree

from painting_trajectory_planner_2_correction import (
    _extract_surface_polyline_info as _extract_guide_polyline_info,
    _resample_polyline_linear_2d as _resample_guide_polyline_2d,
    _select_hull_chain_from_endpoints as _select_guide_hull_chain,
)


def _safe_normalize(v):
    arr = np.asarray(v, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-12:
        return np.zeros_like(arr, dtype=float)
    return arr / norm


def _orient_vectors_consistently(vectors, reference=None):
    values = np.asarray(vectors, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) == 0:
        return np.empty((0, 3), dtype=float)

    oriented = np.zeros_like(values, dtype=float)
    reference_vec = None
    if reference is not None:
        reference_vec = _safe_normalize(reference)
        if np.linalg.norm(reference_vec) <= 1e-12:
            reference_vec = None

    previous = reference_vec
    for idx, value in enumerate(values):
        current = _safe_normalize(value)
        if np.linalg.norm(current) <= 1e-12:
            if previous is not None:
                current = previous.copy()
            elif reference_vec is not None:
                current = reference_vec.copy()
            else:
                current = np.array([0.0, 0.0, 1.0], dtype=float)

        if previous is not None and float(np.dot(current, previous)) < 0.0:
            current *= -1.0

        oriented[idx] = current
        previous = current

    return oriented


def _spline_scan_config(spline_start_side):
    side = str(spline_start_side).strip().lower()
    if side == "top":
        return {"side": side, "slice_axis": 1, "row_axis": 0, "descending": True}
    if side == "bottom":
        return {"side": side, "slice_axis": 1, "row_axis": 0, "descending": False}
    if side == "left":
        return {"side": side, "slice_axis": 0, "row_axis": 1, "descending": False}
    if side == "right":
        return {"side": side, "slice_axis": 0, "row_axis": 1, "descending": True}
    raise ValueError("spline_start_side must be one of: top, bottom, left, right")


def _build_work_frame(points_world, paint_dir_world, scan_dir_world):
    p = np.asarray(points_world, dtype=float)
    origin = np.mean(p, axis=0)

    view_dir = _safe_normalize(paint_dir_world)
    if np.linalg.norm(view_dir) <= 1e-12:
        view_dir = np.array([0.0, 0.0, -1.0], dtype=float)

    surface_dir = _safe_normalize(-view_dir)
    row_dir = _safe_normalize(scan_dir_world)
    if np.linalg.norm(row_dir) <= 1e-12:
        row_dir = np.array([1.0, 0.0, 0.0], dtype=float)

    x_axis = row_dir - float(np.dot(row_dir, surface_dir)) * surface_dir
    if np.linalg.norm(x_axis) <= 1e-12:
        tmp = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(tmp, surface_dir))) > 0.9:
            tmp = np.array([0.0, 1.0, 0.0], dtype=float)
        x_axis = tmp - float(np.dot(tmp, surface_dir)) * surface_dir
    x_axis = _safe_normalize(x_axis)

    y_axis = _safe_normalize(np.cross(surface_dir, x_axis))
    x_axis = _safe_normalize(np.cross(y_axis, surface_dir))
    r_work = np.column_stack([x_axis, y_axis, surface_dir])
    return origin, r_work, view_dir, surface_dir


def _axis_unit(axis_idx):
    axis = np.zeros(3, dtype=float)
    axis[int(axis_idx)] = 1.0
    return axis


def _slice_normal_direction(slice_axis_idx, descending):
    direction = _axis_unit(slice_axis_idx)
    if bool(descending):
        direction *= -1.0
    return direction


def _estimate_surface_normals_work(points_work, neighbor_count):
    pts = np.asarray(points_work, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.empty((0, 3), dtype=float)
    if len(pts) < 3:
        return np.repeat(np.array([[0.0, 0.0, 1.0]], dtype=float), len(pts), axis=0)

    k = min(max(int(neighbor_count), 3), len(pts))
    tree = cKDTree(pts)
    _, indices = tree.query(pts, k=k)
    indices = np.asarray(indices, dtype=int)
    if indices.ndim == 1:
        indices = indices.reshape(-1, 1)

    normals = np.zeros_like(pts)
    for idx, neigh in enumerate(indices):
        local = pts[neigh]
        local = local - np.mean(local, axis=0, keepdims=True)
        try:
            _, _, vh = np.linalg.svd(local, full_matrices=False)
            normal = vh[-1]
        except np.linalg.LinAlgError:
            normal = np.array([0.0, 0.0, 1.0], dtype=float)
        normal = _safe_normalize(normal)
        normals[idx] = normal
    return normals


def _estimate_row_bin_spacing(points_work, row_axis_idx):
    pts = np.asarray(points_work, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        return 0.0025
    coords = np.unique(np.round(pts[:, int(row_axis_idx)], decimals=6))
    if len(coords) < 2:
        return 0.0025
    diffs = np.diff(np.sort(coords))
    diffs = diffs[diffs > 1e-8]
    if len(diffs) == 0:
        return 0.0025
    return float(np.median(diffs))


def _build_knn_adjacency(points, neighbor_count, edge_max_factor):
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or len(pts) == 0:
        return []
    if len(pts) == 1:
        return [[]]

    k = min(max(int(neighbor_count) + 1, 2), len(pts))
    tree = cKDTree(pts)
    distances, indices = tree.query(pts, k=k)
    distances = np.asarray(distances, dtype=float)
    indices = np.asarray(indices, dtype=int)
    if distances.ndim == 1:
        distances = distances.reshape(-1, 1)
        indices = indices.reshape(-1, 1)

    positive = distances[:, 1:][distances[:, 1:] > 1e-12]
    edge_limit = np.inf
    if len(positive):
        edge_limit = float(np.median(positive) * max(float(edge_max_factor), 1.0))

    adjacency = [dict() for _ in range(len(pts))]
    for i in range(len(pts)):
        for d, j in zip(distances[i, 1:], indices[i, 1:]):
            j = int(j)
            if j == i or not np.isfinite(d) or d > edge_limit:
                continue
            w = float(d)
            old = adjacency[i].get(j)
            if old is None or w < old:
                adjacency[i][j] = w
                adjacency[j][i] = w
    return [[(j, w) for j, w in neigh.items()] for neigh in adjacency]


def _connected_component_from_mask(indices, adjacency, anchor_index, points_work, anchor_point):
    indices = np.asarray(indices, dtype=int).reshape(-1)
    if len(indices) == 0:
        return indices
    index_set = set(int(i) for i in indices)
    if int(anchor_index) in index_set:
        start = int(anchor_index)
    else:
        local_points = np.asarray(points_work, dtype=float)[indices]
        start = int(indices[int(np.argmin(np.linalg.norm(local_points - np.asarray(anchor_point, float), axis=1)))])

    visited = {start}
    stack = [start]
    while stack:
        node = stack.pop()
        for nei, _ in adjacency[node]:
            nei = int(nei)
            if nei in index_set and nei not in visited:
                visited.add(nei)
                stack.append(nei)
    return np.asarray(sorted(visited), dtype=int)


def _project_to_tangent_plane(vector, normal, fallback=None):
    normal = _safe_normalize(normal)
    v = np.asarray(vector, dtype=float)
    v = v - float(np.dot(v, normal)) * normal
    if np.linalg.norm(v) <= 1e-12:
        if fallback is None:
            fallback = np.array([1.0, 0.0, 0.0], dtype=float)
        v = np.asarray(fallback, dtype=float)
        v = v - float(np.dot(v, normal)) * normal
    if np.linalg.norm(v) <= 1e-12:
        v = np.array([1.0, 0.0, 0.0], dtype=float)
        v = v - float(np.dot(v, normal)) * normal
    return _safe_normalize(v)


def _surface_row_axis_from_guide(surface_normal, guide_tangent, row_axis_idx):
    surface_normal = _safe_normalize(surface_normal)
    guide_tangent = _project_to_tangent_plane(guide_tangent, surface_normal, fallback=_axis_unit(row_axis_idx))
    row_axis = np.cross(surface_normal, guide_tangent)
    row_axis = _project_to_tangent_plane(row_axis, surface_normal, fallback=_axis_unit(row_axis_idx))
    if float(np.dot(row_axis, _axis_unit(row_axis_idx))) < 0.0:
        row_axis *= -1.0
    return row_axis


def _central_hull_guide_curve(
    points_work,
    normals_work,
    surface_tree,
    slice_axis_idx,
    row_axis_idx,
    descending,
    strip_half_width,
    guide_spacing,
):
    pts = np.asarray(points_work, dtype=float)
    normals = np.asarray(normals_work, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        return None
    if normals.shape != pts.shape:
        normals = np.repeat(np.array([[0.0, 0.0, 1.0]], dtype=float), len(pts), axis=0)

    row_coords = pts[:, int(row_axis_idx)]
    center_value = float(np.median(row_coords))
    base_half_width = max(float(strip_half_width), 1e-6)

    strip_indices = np.empty(0, dtype=int)
    for scale in (1.0, 1.5, 2.0, 3.0, 4.0, 6.0):
        candidates = np.flatnonzero(np.abs(row_coords - center_value) <= base_half_width * scale).astype(int)
        if len(candidates) >= 20:
            strip_indices = candidates
            break
        if len(candidates) > len(strip_indices):
            strip_indices = candidates
    if len(strip_indices) < 2:
        return None

    strip_points = pts[strip_indices]
    strip_normals = normals[strip_indices]
    guide_2d_points = np.column_stack([
        strip_points[:, int(slice_axis_idx)],
        strip_points[:, 2],
    ])

    observed = _extract_guide_polyline_info(
        guide_2d_points,
        row_point_spacing=max(float(guide_spacing) * 0.25, 0.0025),
        endpoint_quantize_step=max(base_half_width * 0.25, 0.001),
        endpoint_graph_neighbor_count=10,
        endpoint_component_min_arc_length=max(float(guide_spacing) * 0.5, 0.003),
        endpoint_component_merge_backtrack_tolerance=0.0,
    )
    observed_path = np.asarray(observed.get("path", []), dtype=float)
    if observed_path.ndim != 2 or observed_path.shape[1] != 2 or len(observed_path) < 2:
        return None

    hull_chain = _select_guide_hull_chain(
        guide_2d_points,
        endpoint_a=observed_path[0],
        endpoint_b=observed_path[-1],
    )
    if hull_chain is None or len(hull_chain) < 2:
        hull_chain = observed_path
    hull_chain = np.asarray(hull_chain, dtype=float)

    if bool(descending):
        if hull_chain[0, 0] < hull_chain[-1, 0]:
            hull_chain = hull_chain[::-1].copy()
    else:
        if hull_chain[0, 0] > hull_chain[-1, 0]:
            hull_chain = hull_chain[::-1].copy()

    sampled_2d = _resample_guide_polyline_2d(hull_chain, point_spacing=max(float(guide_spacing), 1e-4))
    if sampled_2d is None or len(sampled_2d) < 2:
        sampled_2d = hull_chain.copy()

    hull_chain_points = np.zeros((len(hull_chain), 3), dtype=float)
    hull_chain_points[:, int(row_axis_idx)] = center_value
    hull_chain_points[:, int(slice_axis_idx)] = hull_chain[:, 0]
    hull_chain_points[:, 2] = hull_chain[:, 1]

    query_points = np.zeros((len(sampled_2d), 3), dtype=float)
    query_points[:, int(row_axis_idx)] = center_value
    query_points[:, int(slice_axis_idx)] = sampled_2d[:, 0]
    query_points[:, 2] = sampled_2d[:, 1]

    strip_tree = cKDTree(strip_points)
    _, local_indices = strip_tree.query(query_points, k=1)
    local_indices = np.asarray(local_indices, dtype=int).reshape(-1)
    guide_indices = strip_indices[local_indices]
    guide_points = query_points.copy()
    guide_normals = strip_normals[local_indices].copy()

    keep = np.concatenate([
        [True],
        np.linalg.norm(np.diff(guide_points[:, [slice_axis_idx, 2]], axis=0), axis=1) > 1e-6,
    ])
    guide_points = guide_points[keep]
    guide_normals = guide_normals[keep]
    guide_indices = guide_indices[keep]
    if len(guide_points) < 2:
        return None

    guide_normals = _orient_vectors_consistently(
        guide_normals,
        reference=np.array([0.0, 0.0, 1.0], dtype=float),
    )

    if surface_tree is not None:
        _, global_indices = surface_tree.query(guide_points, k=1)
        guide_indices = np.asarray(global_indices, dtype=int).reshape(-1)

    return {
        "points": guide_points,
        "indices": guide_indices.astype(int),
        "normals": guide_normals,
        "strip_points": strip_points,
        "observed_path_2d": observed_path,
        "hull_chain_2d": hull_chain,
        "hull_chain_points": hull_chain_points,
    }


def _boundary_seed_indices(points_work, slice_axis_idx, row_axis_idx, descending, seed_width, seed_bin_spacing):
    pts = np.asarray(points_work, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.empty(0, dtype=int)

    slice_coords = pts[:, int(slice_axis_idx)]
    row_coords = pts[:, int(row_axis_idx)]
    width = max(float(seed_width), 0.0)
    if bool(descending):
        boundary_value = float(np.max(slice_coords))
        candidate_mask = slice_coords >= boundary_value - width
    else:
        boundary_value = float(np.min(slice_coords))
        candidate_mask = slice_coords <= boundary_value + width

    candidates = np.flatnonzero(candidate_mask)
    min_seed_count = min(len(pts), max(5, int(np.ceil(len(pts) * 0.01))))
    if len(candidates) < min_seed_count:
        order = np.argsort(slice_coords)
        candidates = order[-min_seed_count:] if bool(descending) else order[:min_seed_count]
    if len(candidates) == 0:
        return np.empty(0, dtype=int)

    bin_spacing = float(seed_bin_spacing)
    if bin_spacing <= 0.0:
        bin_spacing = _estimate_row_bin_spacing(pts[candidates], row_axis_idx)
    bin_spacing = max(bin_spacing, 1e-4)

    cand_rows = row_coords[candidates]
    row_min = float(np.min(cand_rows))
    bin_ids = np.floor((cand_rows - row_min) / bin_spacing).astype(int)
    seed = []
    for bin_id in np.unique(bin_ids):
        bin_candidates = candidates[bin_ids == bin_id]
        values = slice_coords[bin_candidates]
        selected = bin_candidates[int(np.argmax(values) if bool(descending) else np.argmin(values))]
        seed.append(int(selected))

    if len(seed) < 2:
        order = np.argsort(row_coords[candidates], kind="mergesort")
        seed = [int(idx) for idx in candidates[order]]
    else:
        seed = sorted(set(seed), key=lambda idx: float(row_coords[idx]))
    return np.asarray(seed, dtype=int)


def _world_direction(work_to_world, origin_work, direction_work):
    origin_world = work_to_world(np.asarray(origin_work, dtype=float).reshape(1, 3))[0]
    tip_world = work_to_world((np.asarray(origin_work, dtype=float) + np.asarray(direction_work, dtype=float)).reshape(1, 3))[0]
    return _safe_normalize(tip_world - origin_world)


def _make_profile(
    row_index,
    slice_position,
    slice_points_work,
    normals_work,
    normal_indices,
    plane_origin_work,
    plane_normal_work,
    plane_row_axis_work,
    plane_other_axis_work,
    work_to_world,
    slicing_method,
    extra=None,
):
    plane_origin_work = np.asarray(plane_origin_work, dtype=float)
    plane_normal_work = _safe_normalize(plane_normal_work)
    plane_row_axis_work = _safe_normalize(plane_row_axis_work)
    plane_other_axis_work = _safe_normalize(plane_other_axis_work)
    plane_origin_world = work_to_world(plane_origin_work.reshape(1, 3))[0]
    profile = {
        "row_index": int(row_index),
        "slice_position": float(slice_position),
        "slice_points_work": np.asarray(slice_points_work, dtype=float).copy(),
        "slice_points_world": work_to_world(slice_points_work),
        "slice_plane_origin_work": plane_origin_work.copy(),
        "slice_plane_normal_work": plane_normal_work.copy(),
        "slice_plane_row_axis_work": plane_row_axis_work.copy(),
        "slice_plane_other_axis_work": plane_other_axis_work.copy(),
        "slice_plane_origin_world": plane_origin_world,
        "slice_plane_normal_world": _world_direction(work_to_world, plane_origin_work, plane_normal_work),
        "slice_plane_row_axis_world": _world_direction(work_to_world, plane_origin_work, plane_row_axis_work),
        "slice_plane_other_axis_world": _world_direction(work_to_world, plane_origin_work, plane_other_axis_work),
        "slicing_method": str(slicing_method),
    }
    if normals_work is not None and normal_indices is not None and len(normal_indices) == len(slice_points_work):
        profile["slice_normals_work"] = np.asarray(normals_work, dtype=float)[np.asarray(normal_indices, dtype=int)].copy()
    if extra:
        profile.update(extra)
    return profile


def _generate_axis_slice_profiles(pw, work_to_world, slice_cfg, spline_spacing, spline_half_width, spline_start_offset):
    slice_axis_idx = int(slice_cfg["slice_axis"])
    row_axis_idx = int(slice_cfg["row_axis"])
    descending = bool(slice_cfg["descending"])

    slice_min = float(np.min(pw[:, slice_axis_idx]))
    slice_max = float(np.max(pw[:, slice_axis_idx]))
    if not np.isfinite([slice_min, slice_max]).all() or slice_max < slice_min:
        return []

    if descending:
        first_slice = slice_max - float(spline_start_offset)
        slice_positions = np.arange(first_slice, slice_min - 1e-12, -float(spline_spacing))
        plane_normal_work = _slice_normal_direction(slice_axis_idx, descending=True)
    else:
        first_slice = slice_min + float(spline_start_offset)
        slice_positions = np.arange(first_slice, slice_max + 1e-12, float(spline_spacing))
        plane_normal_work = _slice_normal_direction(slice_axis_idx, descending=False)

    row_axis_work = _axis_unit(row_axis_idx)
    plane_other_axis_work = _safe_normalize(np.cross(plane_normal_work, row_axis_work))

    profiles = []
    for row_index, slice_center in enumerate(slice_positions):
        indices = np.flatnonzero(np.abs(pw[:, slice_axis_idx] - float(slice_center)) <= float(spline_half_width)).astype(int)
        if len(indices) < 2:
            continue
        slice_points_work = pw[indices].copy()
        slice_points_work[:, slice_axis_idx] = float(slice_center)
        plane_origin_work = np.mean(slice_points_work, axis=0)
        profiles.append(
            _make_profile(
                row_index=row_index,
                slice_position=float(slice_center),
                slice_points_work=slice_points_work,
                normals_work=None,
                normal_indices=None,
                plane_origin_work=plane_origin_work,
                plane_normal_work=plane_normal_work,
                plane_row_axis_work=row_axis_work,
                plane_other_axis_work=plane_other_axis_work,
                work_to_world=work_to_world,
                slicing_method="axis",
            )
        )
    return profiles


def _generate_surface_marching_slice_profiles(
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
    geodesic_seed_bin_spacing,
    geodesic_band_half_width,
    geodesic_min_band_points,
):
    slice_axis_idx = int(slice_cfg["slice_axis"])
    row_axis_idx = int(slice_cfg["row_axis"])
    descending = bool(slice_cfg["descending"])

    normals_work = _estimate_surface_normals_work(pw, neighbor_count=geodesic_normal_neighbor_count)
    adjacency = _build_knn_adjacency(pw, neighbor_count=geodesic_neighbor_count, edge_max_factor=geodesic_edge_max_factor)
    surface_tree = cKDTree(pw)

    band_half_width = float(geodesic_band_half_width)
    if band_half_width <= 0.0:
        band_half_width = float(spline_half_width)
    seed_width = float(geodesic_seed_width)
    if seed_width <= 0.0:
        seed_width = float(spline_half_width)

    seed_indices = _boundary_seed_indices(
        pw, slice_axis_idx, row_axis_idx, descending, seed_width, geodesic_seed_bin_spacing
    )
    end_indices = _boundary_seed_indices(
        pw, slice_axis_idx, row_axis_idx, not descending, seed_width, geodesic_seed_bin_spacing
    )
    seed_points_work = pw[seed_indices].copy() if len(seed_indices) else np.empty((0, 3), dtype=float)
    end_points_work = pw[end_indices].copy() if len(end_indices) else np.empty((0, 3), dtype=float)

    guide = _central_hull_guide_curve(
        pw,
        normals_work,
        surface_tree=surface_tree,
        slice_axis_idx=slice_axis_idx,
        row_axis_idx=row_axis_idx,
        descending=descending,
        strip_half_width=band_half_width,
        guide_spacing=spline_spacing,
    )
    if guide is None:
        return []

    guide_points = np.asarray(guide.get("points", []), dtype=float)
    guide_indices = np.asarray(guide.get("indices", []), dtype=int)
    guide_normals = np.asarray(guide.get("normals", []), dtype=float)
    if guide_points.ndim != 2 or guide_points.shape[1] != 3 or len(guide_points) < 2:
        return []
    if guide_normals.shape != guide_points.shape:
        guide_normals = normals_work[guide_indices]
    guide_normals = _orient_vectors_consistently(
        guide_normals,
        reference=np.array([0.0, 0.0, 1.0], dtype=float),
    )

    desired_axis = _slice_normal_direction(slice_axis_idx, descending)
    scan_row_axis = _axis_unit(row_axis_idx)
    outward_reference = _axis_unit(2)
    guide_tangents = []
    for idx in range(len(guide_points)):
        if idx == 0:
            tangent_source = guide_points[1] - guide_points[0]
        elif idx == len(guide_points) - 1:
            tangent_source = guide_points[-1] - guide_points[-2]
        else:
            tangent_source = guide_points[idx + 1] - guide_points[idx - 1]
        tangent = _safe_normalize(tangent_source)
        if np.linalg.norm(tangent) <= 1e-12:
            tangent = desired_axis.copy()
        if float(np.dot(tangent, desired_axis)) < 0.0:
            tangent *= -1.0
        guide_tangents.append(tangent)
    guide_tangents = np.asarray(guide_tangents, dtype=float)

    min_band_points = max(int(geodesic_min_band_points), 2)
    profiles = []
    for row_index, (origin_work, plane_normal_work, surface_normal_work, anchor_index) in enumerate(
        zip(guide_points, guide_tangents, guide_normals, guide_indices)
    ):
        origin_work = np.asarray(origin_work, dtype=float)
        plane_normal_work = _safe_normalize(plane_normal_work)
        fallback_surface_normal_work = _safe_normalize(surface_normal_work)
        plane_row_axis_work = scan_row_axis - float(np.dot(scan_row_axis, plane_normal_work)) * plane_normal_work
        if np.linalg.norm(plane_row_axis_work) <= 1e-9:
            plane_row_axis_work = _surface_row_axis_from_guide(
                fallback_surface_normal_work,
                plane_normal_work,
                row_axis_idx=row_axis_idx,
            )
        else:
            plane_row_axis_work = _safe_normalize(plane_row_axis_work)

        plane_other_axis_work = np.cross(plane_normal_work, plane_row_axis_work)
        if np.linalg.norm(plane_other_axis_work) <= 1e-9:
            plane_other_axis_work = fallback_surface_normal_work
        else:
            plane_other_axis_work = _safe_normalize(plane_other_axis_work)
        if float(np.dot(plane_other_axis_work, outward_reference)) < 0.0:
            plane_other_axis_work *= -1.0

        signed_distance = (pw - origin_work.reshape(1, 3)) @ plane_normal_work
        best_indices = np.empty(0, dtype=int)
        for width_scale in (1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0):
            width = band_half_width * width_scale
            raw_indices = np.flatnonzero(np.abs(signed_distance) <= width).astype(int)
            component = _connected_component_from_mask(
                raw_indices,
                adjacency=adjacency,
                anchor_index=int(anchor_index),
                points_work=pw,
                anchor_point=origin_work,
            )
            if len(component) > len(best_indices):
                best_indices = component
            if len(component) >= min_band_points:
                break
        if len(best_indices) < min_band_points:
            continue

        extra = {
            "slice_surface_coordinates": np.full(len(best_indices), row_index * float(spline_spacing), dtype=float),
            "geodesic_seed_points_work": seed_points_work,
            "geodesic_seed_points_world": work_to_world(seed_points_work) if len(seed_points_work) else np.empty((0, 3), dtype=float),
            "geodesic_end_points_work": end_points_work,
            "geodesic_end_points_world": work_to_world(end_points_work) if len(end_points_work) else np.empty((0, 3), dtype=float),
            "guide_curve_points_work": guide_points.copy(),
            "guide_curve_points_world": work_to_world(guide_points),
            "guide_hull_points_work": np.asarray(guide.get("hull_chain_points", []), dtype=float).copy(),
            "guide_hull_points_world": work_to_world(np.asarray(guide.get("hull_chain_points", []), dtype=float))
            if len(np.asarray(guide.get("hull_chain_points", []), dtype=float))
            else np.empty((0, 3), dtype=float),
            "guide_strip_points_work": np.asarray(guide.get("strip_points", []), dtype=float).copy(),
            "guide_strip_points_world": work_to_world(np.asarray(guide.get("strip_points", []), dtype=float))
            if len(np.asarray(guide.get("strip_points", []), dtype=float))
            else np.empty((0, 3), dtype=float),
        }
        profiles.append(
            _make_profile(
                row_index=row_index,
                slice_position=float(row_index),
                slice_points_work=pw[best_indices].copy(),
                normals_work=normals_work,
                normal_indices=best_indices,
                plane_origin_work=origin_work,
                plane_normal_work=plane_normal_work,
                plane_row_axis_work=plane_row_axis_work,
                plane_other_axis_work=plane_other_axis_work,
                work_to_world=work_to_world,
                slicing_method="surface_marching",
                extra=extra,
            )
        )
    return profiles


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
    geodesic_seed_bin_spacing=0.0,
    geodesic_band_half_width=0.0,
    geodesic_min_band_points=8,
):
    def _empty_result(points_used, r_work=None, origin=None, view_dir=None, surface_dir=None):
        return {
            "point_cloud_used": np.asarray(points_used, dtype=float),
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

    points = np.asarray(point_cloud, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
        return _empty_result(points)

    origin, r_work, view_dir, surface_dir = _build_work_frame(points, paint_dir, scan_dir)
    work_to_world = lambda x: np.asarray(x, dtype=float) @ r_work.T + origin
    points_work = (points - origin) @ r_work

    slice_cfg = _spline_scan_config(spline_start_side)
    spline_spacing = max(float(spline_spacing), 1e-4)
    spline_point_spacing = max(float(spline_point_spacing), 1e-4)
    spline_half_width = max(float(spline_half_width), 1e-6)
    spline_start_offset = max(float(spline_start_offset), 0.0)
    method = str(slicing_method).strip().lower()

    if method == "axis":
        slice_profiles = _generate_axis_slice_profiles(
            points_work,
            work_to_world=work_to_world,
            slice_cfg=slice_cfg,
            spline_spacing=spline_spacing,
            spline_half_width=spline_half_width,
            spline_start_offset=spline_start_offset,
        )
    elif method in {"geodesic", "surface_marching", "surface_following", "guide_curve"}:
        method = "surface_marching"
        slice_profiles = _generate_surface_marching_slice_profiles(
            points_work,
            work_to_world=work_to_world,
            slice_cfg=slice_cfg,
            spline_spacing=spline_spacing,
            spline_half_width=spline_half_width,
            spline_start_offset=spline_start_offset,
            geodesic_neighbor_count=geodesic_neighbor_count,
            geodesic_normal_neighbor_count=geodesic_normal_neighbor_count,
            geodesic_edge_max_factor=geodesic_edge_max_factor,
            geodesic_seed_width=geodesic_seed_width,
            geodesic_seed_bin_spacing=geodesic_seed_bin_spacing,
            geodesic_band_half_width=geodesic_band_half_width,
            geodesic_min_band_points=geodesic_min_band_points,
        )
    else:
        raise ValueError("slicing_method must be one of: axis, surface_marching")

    return {
        "point_cloud_used": points.copy(),
        "work_frame_world": r_work,
        "work_origin_world": origin,
        "view_dir_world": view_dir,
        "surface_dir_world": surface_dir,
        "slicing_method": method,
        "spline_start_side": str(slice_cfg["side"]),
        "spline_start_offset": float(spline_start_offset),
        "spline_point_spacing": float(spline_point_spacing),
        "slice_axis_idx": int(slice_cfg["slice_axis"]),
        "row_axis_idx": int(slice_cfg["row_axis"]),
        "slice_profiles": slice_profiles,
    }


__all__ = ["generate_surface_spline_rows"]
