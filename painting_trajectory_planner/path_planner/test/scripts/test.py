#!/usr/bin/env python3

import sys
import webbrowser
from pathlib import Path

import numpy as np
import open3d as o3d
import plotly.graph_objects as go


PLANNER_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = PLANNER_ROOT / "scripts"
DATA_DIR = PLANNER_ROOT / "test" / "data"
CSV_DIR = PLANNER_ROOT / "test" / "csv"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from painting_trajectory_planner import generate_painting_trajectory_debug  # noqa: E402


SAVE_DEBUG_HTML = True
OPEN_DEBUG_HTML = True
DEBUG_HTML_NAME = "trajectory_debug.html"
DEBUG_DIRECTION_MAX_SEGMENTS = 1000
DEBUG_DIRECTION_LENGTH = 0.05
DEBUG_SLICE_PLANE_STRIDE = 1
DEBUG_SLICE_PLANE_OPACITY = 0.18
DEBUG_SLICE_PLANE_VISIBLE = "legendonly"

BUMPER_PAINT_DIR = [0.0, 0.0, -1.0]
BUMPER_SCAN_DIR = [1.0, 0.0, 0.0]

DOOR_PAINT_DIR = [0.0, 1.0, 0.0]
DOOR_SCAN_DIR = [1.0, 0.0, 0.0]

# 각 case의 slicing_method는 "axis" 또는 "surface_adaptive"로 지정합니다.
CASES = [
    # {
    #     "name": "car_bumper_small",
    #     "input_csv": DATA_DIR / "car_bumper_small_surface_points.csv",
    #     "output_dir": CSV_DIR / "car_bumper_small",
    #     "paint_dir": BUMPER_PAINT_DIR,
    #     "scan_dir": BUMPER_SCAN_DIR,
    # },
    # {
    #     "name": "car_door_small",
    #     "input_csv": DATA_DIR / "car_door_small_surface_points.csv",
    #     "output_dir": CSV_DIR / "car_door_small",
    #     "paint_dir": DOOR_PAINT_DIR,
    #     "scan_dir": DOOR_SCAN_DIR,
    # },



    {
        "name": "ES300h_front_bumper",
        "input_csv": DATA_DIR / "ES300h_front_bumper.ply",
        "output_dir": CSV_DIR / "ES300h_front_bumper",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [0.0, 1.0, 0.0],
        "slicing_method": "axis",
    },
    {
        "name": "ES300h_hood",
        "input_csv": DATA_DIR / "ES300h_hood.ply",
        "output_dir": CSV_DIR / "ES300h_hood",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [-1.0, 0.0, 0.0],
        "slicing_method": "surface_adaptive",
    },
    {
        "name": "ES300h_left_fender",
        "input_csv": DATA_DIR / "ES300h_left_fender.ply",
        "output_dir": CSV_DIR / "ES300h_left_fender",
        "paint_dir": [0.0, -1.0, 0.0],
        "scan_dir": [-1.0, 0.0, 0.0],
        "slicing_method": "surface_adaptive",
    },
    {
        "name": "ES300h_trunk",
        "input_csv": DATA_DIR / "ES300h_trunk.ply",
        "output_dir": CSV_DIR / "ES300h_trunk",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [0.0, 1.0, 0.0],
        "slicing_method": "surface_adaptive",
    },
    {
        "name": "NX350h_right_fender",
        "input_csv": DATA_DIR / "NX350h_right_fender.ply",
        "output_dir": CSV_DIR / "NX350h_right_fender",
        "paint_dir": [0.0, -1.0, 0.0],
        "scan_dir": [-1.0, 0.0, 0.0],
        "slicing_method": "surface_adaptive",
    },
    {
        "name": "Tesla_Model_Y_front_bumper",
        "input_csv": DATA_DIR / "Tesla_Model_Y_front_bumper.ply",
        "output_dir": CSV_DIR / "Tesla_Model_Y_front_bumper",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [0.0, 1.0, 0.0],
        "slicing_method": "axis",
    },


    # {
    #     "name": "front_hood",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "front_hood" / "solid_pearl_default" / "model-final-new.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "front_hood" / "solid_pearl_default" / "test",
    #     "paint_dir": [0.0, 0.0, -1.0],
    #     "scan_dir": [0.0, 1.0, 0.0],
    # },
    # {
    #     "name": "right_front_door",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "right_front_door" / "solid_pearl_default" / "model-final-new.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "right_front_door" / "solid_pearl_default" / "test",
    #     "paint_dir": [0.0, 1.0, 0.0],
    #     "scan_dir": [1.0, 0.0, 0.0],
    # },
    # {
    #     "name": "right_front_fender",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "right_front_fender" / "solid_pearl_default" / "model-final-new.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "right_front_fender" / "solid_pearl_default" / "test",
    #     "paint_dir": [0.0, 1.0, 0.0],
    #     "scan_dir": [1.0, 0.0, 0.0],
    # },
    # {
    #     "name": "right_rear_door",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "right_rear_door" / "solid_pearl_default" / "model-final-new.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "right_rear_door" / "solid_pearl_default" / "test",
    #     "paint_dir": [0.0, -1.0, 0.0],
    #     "scan_dir": [-1.0, 0.0, 0.0],
    # },
    # {
    #     "name": "trunk",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "trunk" / "ES300h rear hood.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "trunk" / "test",
    #     "paint_dir": [0.0, 0.0, -1.0],
    #     "scan_dir": [0.0, 1.0, 0.0],
    # },
]


def load_csv_surface_data(csv_path: Path) -> tuple[np.ndarray, list[str]]:
    points = np.loadtxt(csv_path, delimiter=",", skiprows=1, usecols=(0, 1, 2))
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{csv_path} must contain x,y,z columns")
    return points, ["rgb(0, 184, 140)"] * len(points)


def _rgb_string(values) -> str:
    rgb = np.asarray(values, dtype=float).reshape(3)
    if np.max(rgb) <= 1.0:
        rgb = rgb * 255.0
    rgb = np.clip(np.rint(rgb), 0, 255).astype(int)
    return f"rgb({rgb[0]}, {rgb[1]}, {rgb[2]})"


def load_ply_surface_data(ply_path: Path) -> tuple[np.ndarray, list[str]]:
    point_cloud = o3d.io.read_point_cloud(str(ply_path))
    if point_cloud.is_empty():
        raise ValueError(f"{ply_path}에서 포인트 클라우드를 읽지 못했습니다.")

    points = np.asarray(point_cloud.points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{ply_path} must contain vertex x,y,z data")
    if point_cloud.has_colors():
        colors = [_rgb_string(color) for color in np.asarray(point_cloud.colors)]
    else:
        colors = ["rgb(0, 184, 140)"] * len(points)
    return points, colors


def load_surface_data(input_path: Path) -> tuple[np.ndarray, list[str]]:
    suffix = input_path.suffix.lower()
    if suffix == ".csv":
        return load_csv_surface_data(input_path)
    if suffix == ".ply":
        return load_ply_surface_data(input_path)
    raise ValueError(f"지원하지 않는 표면 데이터 형식입니다: {suffix}. CSV 또는 PLY만 지원합니다.")


def _points_array(values) -> np.ndarray:
    points = np.asarray(values, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        return np.empty((0, 3), dtype=float)
    return points


def _rows_to_line_xyz(rows) -> tuple[list, list, list]:
    xs, ys, zs = [], [], []
    for row in rows or []:
        points = _points_array(row)
        if len(points) == 0:
            continue
        xs.extend(points[:, 0].tolist())
        ys.extend(points[:, 1].tolist())
        zs.extend(points[:, 2].tolist())
        xs.append(None)
        ys.append(None)
        zs.append(None)
    return xs, ys, zs


def _masked_points(rows, mask_rows) -> np.ndarray:
    selected = []
    for row, mask in zip(rows or [], mask_rows or []):
        points = _points_array(row)
        mask = np.asarray(mask, dtype=bool).reshape(-1)
        if len(points) == 0 or len(mask) != len(points):
            continue
        selected.append(points[mask])
    if not selected:
        return np.empty((0, 3), dtype=float)
    return np.vstack(selected)


def _direction_segments(position_rows, direction_rows) -> tuple[list, list, list]:
    segments = []
    for positions, directions in zip(position_rows or [], direction_rows or []):
        points = _points_array(positions)
        dirs = _points_array(directions)
        if len(points) == 0 or len(points) != len(dirs):
            continue
        segments.extend(zip(points, dirs))

    if not segments:
        return [], [], []
    if len(segments) > DEBUG_DIRECTION_MAX_SEGMENTS:
        indices = np.linspace(0, len(segments) - 1, DEBUG_DIRECTION_MAX_SEGMENTS, dtype=int)
        segments = [segments[index] for index in indices]

    xs, ys, zs = [], [], []
    for point, direction in segments:
        direction_norm = np.linalg.norm(direction)
        if direction_norm <= 1e-12:
            continue
        end = point + DEBUG_DIRECTION_LENGTH * direction / direction_norm
        xs.extend([point[0], end[0], None])
        ys.extend([point[1], end[1], None])
        zs.extend([point[2], end[2], None])
    return xs, ys, zs


def _add_marker_trace(
    fig: go.Figure,
    name: str,
    points,
    color,
    size: float,
    opacity: float = 1.0,
    visible=True,
) -> None:
    points = _points_array(points)
    if len(points) == 0:
        return
    fig.add_trace(
        go.Scatter3d(
            x=points[:, 0],
            y=points[:, 1],
            z=points[:, 2],
            mode="markers",
            name=name,
            marker={"size": size, "color": color, "opacity": opacity},
            visible=visible,
        )
    )


def _add_line_trace(
    fig: go.Figure,
    name: str,
    rows,
    color: str,
    width: float,
    visible=True,
) -> None:
    xs, ys, zs = _rows_to_line_xyz(rows)
    if not xs:
        return
    fig.add_trace(
        go.Scatter3d(
            x=xs,
            y=ys,
            z=zs,
            mode="lines",
            name=name,
            line={"color": color, "width": width},
            visible=visible,
        )
    )


def _add_slice_plane_mesh(
    fig: go.Figure,
    slice_profiles,
    point_cloud_xyz: np.ndarray,
    visible=True,
) -> None:
    profiles = list(slice_profiles or [])
    cloud = _points_array(point_cloud_xyz)
    if not profiles or len(cloud) == 0:
        return

    xs, ys, zs = [], [], []
    tri_i, tri_j, tri_k = [], [], []
    vertex_offset = 0

    for profile in profiles[:: max(int(DEBUG_SLICE_PLANE_STRIDE), 1)]:
        origin = np.asarray(profile.get("slice_plane_origin_world", []), dtype=float).reshape(-1)
        row_axis = np.asarray(profile.get("slice_plane_row_axis_world", []), dtype=float).reshape(-1)
        other_axis = np.asarray(profile.get("slice_plane_other_axis_world", []), dtype=float).reshape(-1)
        if origin.size != 3 or row_axis.size != 3 or other_axis.size != 3:
            continue

        row_axis = row_axis / max(float(np.linalg.norm(row_axis)), 1e-12)
        other_axis = other_axis / max(float(np.linalg.norm(other_axis)), 1e-12)
        if not np.isfinite(origin).all() or not np.isfinite(row_axis).all() or not np.isfinite(other_axis).all():
            continue

        slice_points = _points_array(profile.get("slice_points_world", []))
        row_source = slice_points if len(slice_points) >= 2 else cloud
        row_values = row_source @ row_axis
        other_values = cloud @ other_axis
        row_half = max(0.5 * float(np.ptp(row_values)), 0.03)
        other_half = max(0.55 * float(np.ptp(other_values)), 0.05)

        corners = np.asarray(
            [
                origin - row_half * row_axis - other_half * other_axis,
                origin + row_half * row_axis - other_half * other_axis,
                origin + row_half * row_axis + other_half * other_axis,
                origin - row_half * row_axis + other_half * other_axis,
            ],
            dtype=float,
        )
        xs.extend(corners[:, 0].tolist())
        ys.extend(corners[:, 1].tolist())
        zs.extend(corners[:, 2].tolist())
        tri_i.extend([vertex_offset, vertex_offset])
        tri_j.extend([vertex_offset + 1, vertex_offset + 2])
        tri_k.extend([vertex_offset + 2, vertex_offset + 3])
        vertex_offset += 4

    if not xs:
        return

    fig.add_trace(
        go.Mesh3d(
            x=xs,
            y=ys,
            z=zs,
            i=tri_i,
            j=tri_j,
            k=tri_k,
            name="local slicing planes",
            color="rgb(255, 210, 40)",
            opacity=DEBUG_SLICE_PLANE_OPACITY,
            visible=visible,
            showlegend=True,
            hoverinfo="skip",
        )
    )


def write_debug_plotly_html(
    case: dict,
    point_cloud_xyz: np.ndarray,
    surface_colors: list[str],
    result: dict,
) -> Path:
    fig = go.Figure()

    _add_marker_trace(
        fig,
        name=f"surface points ({len(point_cloud_xyz)})",
        points=point_cloud_xyz,
        color=surface_colors,
        size=2.0,
        opacity=0.85,
    )

    slice_points = [
        profile.get("slice_points_world", [])
        for profile in result.get("slice_profiles", [])
    ]
    guide_curve_rows = []
    guide_hull_rows = []
    guide_strip_points = []
    seen_guide = set()
    seen_guide_hull = set()
    seen_guide_strip = set()
    for profile in result.get("slice_profiles", []):
        guide = _points_array(profile.get("guide_curve_points_world", []))
        if len(guide) > 0:
            key = tuple(np.round(guide.reshape(-1), 6).tolist())
            if key not in seen_guide:
                seen_guide.add(key)
                guide_curve_rows.append(guide)

        guide_hull = _points_array(profile.get("guide_hull_points_world", []))
        if len(guide_hull) > 0:
            key = tuple(np.round(guide_hull.reshape(-1), 6).tolist())
            if key not in seen_guide_hull:
                seen_guide_hull.add(key)
                guide_hull_rows.append(guide_hull)

        guide_strip = _points_array(profile.get("guide_strip_points_world", []))
        if len(guide_strip) > 0:
            key = tuple(np.round(guide_strip.reshape(-1), 6).tolist())
            if key not in seen_guide_strip:
                seen_guide_strip.add(key)
                guide_strip_points.append(guide_strip)

    _add_marker_trace(
        fig,
        name="slice raw points",
        points=np.vstack([_points_array(row) for row in slice_points if len(_points_array(row)) > 0])
        if slice_points
        else np.empty((0, 3)),
        color="rgba(255, 70, 210, 0.95)",
        size=2.1,
        opacity=0.95,
        visible=True,
    )
    _add_line_trace(
        fig,
        name="guide curve",
        rows=guide_curve_rows,
        color="rgb(255, 120, 0)",
        width=6.0,
        visible=True,
    )
    _add_line_trace(
        fig,
        name="guide upper hull",
        rows=guide_hull_rows,
        color="rgb(255, 230, 0)",
        width=4.0,
        visible="legendonly",
    )
    _add_marker_trace(
        fig,
        name="guide strip points",
        points=np.vstack(guide_strip_points) if guide_strip_points else np.empty((0, 3)),
        color="rgba(255, 170, 0, 0.8)",
        size=2.2,
        opacity=0.8,
        visible="legendonly",
    )

    _add_slice_plane_mesh(
        fig,
        result.get("slice_profiles", []),
        point_cloud_xyz,
        visible=DEBUG_SLICE_PLANE_VISIBLE,
    )

    _add_line_trace(
        fig,
        name="correction components",
        rows=result.get("correction_observed_path_rows_world", []),
        color="rgba(160, 160, 160, 0.75)",
        width=2.0,
        visible="legendonly",
    )
    _add_line_trace(
        fig,
        name="correction connected component",
        rows=result.get("correction_connected_component_rows_world", []),
        color="rgba(80, 255, 180, 0.95)",
        width=4.0,
        visible="legendonly",
    )
    _add_line_trace(
        fig,
        name="correction convex hull",
        rows=result.get("correction_convex_hull_rows_world", []),
        color="rgba(255, 255, 255, 0.70)",
        width=3.0,
        visible="legendonly",
    )
    _add_line_trace(
        fig,
        name="correction selected hull",
        rows=result.get("correction_selected_hull_rows_world", []),
        color="rgba(255, 120, 60, 0.85)",
        width=3.0,
        visible="legendonly",
    )
    _add_marker_trace(
        fig,
        name="correction supported samples",
        points=np.vstack(
            [
                _points_array(row)
                for row in result.get("correction_supported_sample_rows_world", [])
                if len(_points_array(row)) > 0
            ]
        )
        if result.get("correction_supported_sample_rows_world", [])
        else np.empty((0, 3)),
        color="cyan",
        size=2.0,
        visible="legendonly",
    )
    _add_marker_trace(
        fig,
        name="correction endpoints",
        points=result.get("correction_endpoint_points_world", []),
        color="red",
        size=5.0,
    )
    _add_line_trace(
        fig,
        name="corrected rows",
        rows=result.get("corrected_rows", []),
        color="deepskyblue",
        width=3.0,
    )
    _add_line_trace(
        fig,
        name="offset rows",
        rows=result.get("offset_rows", []),
        color="orange",
        width=3.0,
    )
    _add_marker_trace(
        fig,
        name="extension points",
        points=result.get("paint_spline_extension_points_world", []),
        color="purple",
        size=2.4,
        visible="legendonly",
    )
    _add_line_trace(
        fig,
        name="paint trajectory",
        rows=result.get("paint_trajectory_position_rows", []),
        color="yellow",
        width=4.0,
    )
    _add_marker_trace(
        fig,
        name="paint ON points",
        points=_masked_points(
            result.get("paint_trajectory_position_rows", []),
            result.get("paint_trajectory_paint_mask_rows", []),
        ),
        color="lime",
        size=2.4,
    )

    dir_x, dir_y, dir_z = _direction_segments(
        result.get("paint_trajectory_position_rows", []),
        result.get("paint_spline_direction_rows_world", []),
    )
    if dir_x:
        fig.add_trace(
            go.Scatter3d(
                x=dir_x,
                y=dir_y,
                z=dir_z,
                mode="lines",
                name="spray directions",
                line={"color": "magenta", "width": 2.0},
                visible="legendonly",
            )
        )

    axis_style = {
        "showbackground": False,
        "showgrid": False,
        "showline": True,
        "showspikes": False,
        "zeroline": False,
        "color": "rgb(160, 190, 230)",
        "tickfont": {"color": "rgb(120, 160, 210)"},
        "linecolor": "rgba(160, 190, 230, 0.35)",
    }

    fig.update_layout(
        title=f"{case['name']} painting trajectory debug",
        scene={
            "aspectmode": "data",
            "bgcolor": "rgb(24, 26, 30)",
            "xaxis": {"title": "X", **axis_style},
            "yaxis": {"title": "Y", **axis_style},
            "zaxis": {"title": "Z", **axis_style},
        },
        legend={
            "itemsizing": "constant",
            "x": 0.02,
            "y": 0.98,
            "bgcolor": "rgba(20, 20, 20, 0.45)",
            "font": {"color": "white"},
        },
        paper_bgcolor="rgb(24, 26, 30)",
        plot_bgcolor="rgb(24, 26, 30)",
        margin={"l": 0, "r": 0, "t": 42, "b": 0},
    )

    html_path = case["output_dir"] / DEBUG_HTML_NAME
    fig.write_html(
        html_path,
        include_plotlyjs=True,
        full_html=True,
        config={"displaylogo": False},
    )
    return html_path


def generate_case(case: dict) -> dict:
    point_cloud_xyz, surface_colors = load_surface_data(case["input_csv"])
    case["output_dir"].mkdir(parents=True, exist_ok=True)
    for stale_csv in case["output_dir"].glob("trajectory_*.csv"):
        stale_csv.unlink()

    result = generate_painting_trajectory_debug(
        point_cloud_xyz,
        extraction_kwargs={
            "paint_dir": case["paint_dir"],
            "scan_dir": case["scan_dir"],
            "slicing_method": case["slicing_method"],
        },
        structuring_kwargs={
            "save_csv": True,
            "csv_output_dir": str(case["output_dir"]),
            "painting_trajectory_csv_name": "painting_trajectory.csv",
            "trajectory_csv_name_format": "trajectory_{row_index:03d}.csv",
        },
    )

    row_count = len(result["painting_trajectory"].rows)
    point_count = sum(len(row.points) for row in result["painting_trajectory"].rows)
    print(
        f"[{case['name']}] points={len(point_cloud_xyz)} "
        f"mode={case['slicing_method']} rows={row_count} "
        f"trajectory_points={point_count}"
    )
    print(f"[{case['name']}] csv={result['painting_trajectory_csv_files']}")
    if SAVE_DEBUG_HTML:
        debug_html_path = write_debug_plotly_html(
            case,
            point_cloud_xyz,
            surface_colors,
            result,
        )
        print(f"[{case['name']}] debug_html={debug_html_path}")
        if OPEN_DEBUG_HTML:
            webbrowser.open(debug_html_path.as_uri())
    return result


def main() -> int:
    for case in CASES:
        generate_case(case)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
