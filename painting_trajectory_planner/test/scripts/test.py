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
OPEN_DEBUG_HTML = False
DEBUG_HTML_NAME = "trajectory_debug.html"
DEBUG_DIRECTION_MAX_SEGMENTS = 1000
DEBUG_DIRECTION_LENGTH = 0.05

BUMPER_PAINT_DIR = [0.0, 0.0, -1.0]
BUMPER_SCAN_DIR = [1.0, 0.0, 0.0]

DOOR_PAINT_DIR = [0.0, 1.0, 0.0]
DOOR_SCAN_DIR = [1.0, 0.0, 0.0]

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
    },
    {
        "name": "ES300h_hood",
        "input_csv": DATA_DIR / "ES300h_hood.ply",
        "output_dir": CSV_DIR / "ES300h_hood",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [-1.0, 0.0, 0.0],
    },
    {
        "name": "ES300h_left_fender",
        "input_csv": DATA_DIR / "ES300h_left_fender.ply",
        "output_dir": CSV_DIR / "ES300h_left_fender",
        "paint_dir": [0.0, -1.0, 0.0],
        "scan_dir": [-1.0, 0.0, 0.0],
    },
    {
        "name": "ES300h_trunk",
        "input_csv": DATA_DIR / "ES300h_trunk.ply",
        "output_dir": CSV_DIR / "ES300h_trunk",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [0.0, 1.0, 0.0],
    },
    {
        "name": "NX350h_right_fender",
        "input_csv": DATA_DIR / "NX350h_right_fender.ply",
        "output_dir": CSV_DIR / "NX350h_right_fender",
        "paint_dir": [0.0, -1.0, 0.0],
        "scan_dir": [-1.0, 0.0, 0.0],
    },
    {
        "name": "Tesla_Model_Y_front_bumper",
        "input_csv": DATA_DIR / "Tesla_Model_Y_front_bumper.ply",
        "output_dir": CSV_DIR / "Tesla_Model_Y_front_bumper",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [0.0, 1.0, 0.0],
    },
]


def load_csv_surface_points(csv_path: Path) -> np.ndarray:
    points = np.loadtxt(csv_path, delimiter=",", skiprows=1, usecols=(0, 1, 2))
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{csv_path} must contain x,y,z columns")
    return points


def load_ply_surface_points(ply_path: Path) -> np.ndarray:
    point_cloud = o3d.io.read_point_cloud(str(ply_path))
    if point_cloud.is_empty():
        raise ValueError(f"{ply_path}에서 포인트 클라우드를 읽지 못했습니다.")

    points = np.asarray(point_cloud.points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{ply_path} must contain vertex x,y,z data")
    return points


def load_surface_points(input_path: Path) -> np.ndarray:
    suffix = input_path.suffix.lower()
    if suffix == ".csv":
        return load_csv_surface_points(input_path)
    if suffix == ".ply":
        return load_ply_surface_points(input_path)
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
    color: str,
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


def write_debug_plotly_html(case: dict, point_cloud_xyz: np.ndarray, result: dict) -> Path:
    fig = go.Figure()

    _add_marker_trace(
        fig,
        name=f"surface points ({len(point_cloud_xyz)})",
        points=point_cloud_xyz,
        color="rgba(120, 125, 135, 0.35)",
        size=2.0,
        opacity=0.35,
    )

    slice_points = [
        profile.get("slice_points_world", [])
        for profile in result.get("slice_profiles", [])
    ]
    _add_marker_trace(
        fig,
        name="slice raw points",
        points=np.vstack([_points_array(row) for row in slice_points if len(_points_array(row)) > 0])
        if slice_points
        else np.empty((0, 3)),
        color="rgba(130, 160, 255, 0.45)",
        size=1.6,
        opacity=0.45,
        visible="legendonly",
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
    point_cloud_xyz = load_surface_points(case["input_csv"])
    case["output_dir"].mkdir(parents=True, exist_ok=True)

    result = generate_painting_trajectory_debug(
        point_cloud_xyz,
        extraction_kwargs={
            "paint_dir": case["paint_dir"],
            "scan_dir": case["scan_dir"],
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
        f"rows={row_count} trajectory_points={point_count}"
    )
    print(f"[{case['name']}] csv={result['painting_trajectory_csv_files']}")
    if SAVE_DEBUG_HTML:
        debug_html_path = write_debug_plotly_html(case, point_cloud_xyz, result)
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
