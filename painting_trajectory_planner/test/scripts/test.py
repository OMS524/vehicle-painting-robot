#!/usr/bin/env python3

import sys
from pathlib import Path

import numpy as np


PLANNER_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = PLANNER_ROOT / "scripts"
DATA_DIR = PLANNER_ROOT / "test" / "data"
CSV_DIR = PLANNER_ROOT / "test" / "csv"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from painting_trajectory_planner import generate_painting_trajectory_debug  # noqa: E402


BUMPER_PAINT_DIR = [0.0, 0.0, -1.0]
BUMPER_SCAN_DIR = [1.0, 0.0, 0.0]

DOOR_PAINT_DIR = [0.0, 1.0, 0.0]
DOOR_SCAN_DIR = [1.0, 0.0, 0.0]

CASES = [
    {
        "name": "car_bumper_small",
        "input_csv": DATA_DIR / "car_bumper_small_surface_points.csv",
        "output_dir": CSV_DIR / "car_bumper_small",
        "paint_dir": BUMPER_PAINT_DIR,
        "scan_dir": BUMPER_SCAN_DIR,
    },
    {
        "name": "car_door_small",
        "input_csv": DATA_DIR / "car_door_small_surface_points.csv",
        "output_dir": CSV_DIR / "car_door_small",
        "paint_dir": DOOR_PAINT_DIR,
        "scan_dir": DOOR_SCAN_DIR,
    },
]


def load_surface_points(csv_path: Path) -> np.ndarray:
    points = np.loadtxt(csv_path, delimiter=",", skiprows=1, usecols=(0, 1, 2))
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{csv_path} must contain x,y,z columns")
    return points


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
    return result


def main() -> int:
    for case in CASES:
        generate_case(case)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
