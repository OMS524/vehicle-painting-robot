#!/usr/bin/env python3

import csv
import copy
from dataclasses import MISSING, asdict, dataclass, fields, is_dataclass
import os
import sys

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_NAME = "painting_trajectory_planner"
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from paint_trajectory_planner_1_surface_spline_extraction import generate_surface_spline_rows
from paint_trajectory_planner_2_correction import correct_surface_spline_rows
from paint_trajectory_planner_3_offset import generate_offset_rows
from paint_trajectory_planner_4_spline import generate_paint_spline_rows
from paint_trajectory_planner_5_structuring import structure_paint_trajectory


@dataclass
class SurfaceExtractionConfig:
    spline_start_side: str
    spline_start_offset: float
    spline_spacing: float
    spline_half_width: float

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)

    def to_kwargs(self):
        return asdict(self)


@dataclass
class CorrectionEndpointConfig:
    quantize_step: float
    graph_neighbor_count: int
    component_min_arc_length: float

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)


@dataclass
class CorrectionSupportConfig:
    support_point_spacing: float
    tangent_half_width: float
    anchor_max_distance: float
    transition_count: int
    endpoint_transition_count: int

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)


@dataclass
class CorrectionOptimizationConfig:
    lambda_data: float
    lambda_stretch: float
    lambda_bend: float
    lambda_end: float
    regularizer: float

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)


@dataclass
class CorrectionConfig:
    endpoint: CorrectionEndpointConfig
    support: CorrectionSupportConfig
    optimization: CorrectionOptimizationConfig

    @classmethod
    def from_dict(cls, data=None):
        source = _mapping_from_config(data)
        support_data = dict(source.get("support", {}) or {})
        if "support_point_spacing" not in support_data:
            if source.get("support_point_spacing") is not None:
                support_data["support_point_spacing"] = source["support_point_spacing"]
            else:
                legacy_sampling = dict(source.get("sampling", {}) or {})
                if legacy_sampling.get("point_spacing") is not None:
                    support_data["support_point_spacing"] = legacy_sampling["point_spacing"]
        return cls(
            endpoint=CorrectionEndpointConfig.from_dict(source.get("endpoint", {})),
            support=CorrectionSupportConfig.from_dict(support_data),
            optimization=CorrectionOptimizationConfig.from_dict(
                source.get("optimization", {})
            ),
        )

    def to_kwargs(self):
        endpoint = self.endpoint
        support = self.support
        optimization = self.optimization
        return {
            "row_point_spacing": support.support_point_spacing,
            "endpoint_quantize_step": endpoint.quantize_step,
            "endpoint_graph_neighbor_count": endpoint.graph_neighbor_count,
            "endpoint_component_min_arc_length": endpoint.component_min_arc_length,
            "support_tangent_half_width": support.tangent_half_width,
            "support_anchor_max_distance": support.anchor_max_distance,
            "transition_count": support.transition_count,
            "endpoint_transition_count": support.endpoint_transition_count,
            "lambda_data": optimization.lambda_data,
            "lambda_stretch": optimization.lambda_stretch,
            "lambda_bend": optimization.lambda_bend,
            "lambda_end": optimization.lambda_end,
            "regularizer": optimization.regularizer,
        }


@dataclass
class OffsetConfig:
    offset_point_spacing: float
    offset_distance: float

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)

    def to_kwargs(self):
        return asdict(self)


@dataclass
class SplineConfig:
    spline_point_spacing: float
    tcp_speed: float
    endpoint_extension_length: float
    paint_check_distance: float
    paint_check_half_width: float
    paint_min_false_length: float
    paint_min_true_length: float

    @classmethod
    def from_dict(cls, data=None):
        source = _mapping_from_config(data)
        if "spline_point_spacing" not in source:
            legacy_point_spacing = source.get("paint_spline_point_spacing")
            if legacy_point_spacing is not None:
                source["spline_point_spacing"] = legacy_point_spacing
        return _dataclass_from_dict(cls, source)

    def to_kwargs(self):
        return {
            "paint_spline_point_spacing": self.spline_point_spacing,
            "tcp_speed": self.tcp_speed,
            "endpoint_extension_length": self.endpoint_extension_length,
            "paint_check_distance": self.paint_check_distance,
            "paint_check_half_width": self.paint_check_half_width,
            "paint_min_false_length": self.paint_min_false_length,
            "paint_min_true_length": self.paint_min_true_length,
        }


PaintSplineConfig = SplineConfig


@dataclass
class StructuringConfig:
    frame_id: str = "world"
    raster_zigzag: bool = True
    save_csv: bool = True
    csv_output_dir: str = ""
    paint_trajectory_csv_name: str = "paint_trajectory.csv"
    trajectory_csv_name_format: str = "trajectory_{trajectory_index:03d}.csv"

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)

    def to_kwargs(self):
        return asdict(self)


@dataclass
class PaintPathConfig:
    surface_extraction: SurfaceExtractionConfig
    correction: CorrectionConfig
    offset: OffsetConfig
    spline: SplineConfig
    structuring: StructuringConfig

    @classmethod
    def from_yaml(cls, path=None, overrides=None):
        return cls.from_dict(_load_paint_path_config(path), overrides=overrides)

    @classmethod
    def from_dict(cls, data=None, overrides=None):
        source = _mapping_from_config(data)
        if "paint_path" in source:
            source = _mapping_from_config(source["paint_path"])
        if overrides:
            source = _deep_update_dict(source, overrides)
        legacy_spline = _mapping_from_config(source.get("paint_spline", {}))
        spline_data = _mapping_from_config(source.get("spline", {}))
        if legacy_spline:
            spline_data = _deep_update_dict(spline_data, legacy_spline)
        structuring_data = _mapping_from_config(source.get("structuring", {}))
        if (
            "raster_zigzag" not in structuring_data
            and "raster_zigzag" in legacy_spline
        ):
            structuring_data["raster_zigzag"] = legacy_spline["raster_zigzag"]
        return cls(
            surface_extraction=SurfaceExtractionConfig.from_dict(
                source.get("surface_extraction", {})
            ),
            correction=CorrectionConfig.from_dict(source.get("correction", {})),
            offset=OffsetConfig.from_dict(source.get("offset", {})),
            spline=SplineConfig.from_dict(spline_data),
            structuring=StructuringConfig.from_dict(structuring_data),
        )

    def to_dict(self):
        return asdict(self)

    def copy_with_overrides(self, overrides=None):
        return PaintPathConfig.from_dict(self.to_dict(), overrides=overrides)

    @property
    def paint_spline(self):
        return self.spline


def _mapping_from_config(config):
    if config is None:
        return {}
    if isinstance(config, PaintPathConfig):
        return config.to_dict()
    if is_dataclass(config):
        return asdict(config)
    if isinstance(config, dict):
        return copy.deepcopy(config)
    raise TypeError(f"Unsupported paint path config type: {type(config).__name__}")


def _dataclass_from_dict(cls, data=None):
    source = _mapping_from_config(data)
    class_fields = fields(cls)
    field_names = [field_info.name for field_info in class_fields]
    missing = [
        field_info.name
        for field_info in class_fields
        if (
            field_info.name not in source
            and field_info.default is MISSING
            and field_info.default_factory is MISSING
        )
    ]
    if missing:
        raise ValueError(
            f"Missing {cls.__name__} field(s) in paint path config: "
            + ", ".join(missing)
        )
    names = set(field_names)
    return cls(**{name: source[name] for name in names if name in source})


def _deep_update_dict(base, updates):
    result = _mapping_from_config(base)
    for key, value in _mapping_from_config(updates).items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, dict)
        ):
            result[key] = _deep_update_dict(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_paint_path_config(path=None, overrides=None) -> PaintPathConfig:
    return PaintPathConfig.from_yaml(path=path, overrides=overrides)


_DEFAULT_PAINT_PATH_CONFIG = None


def get_default_paint_path_config() -> PaintPathConfig:
    global _DEFAULT_PAINT_PATH_CONFIG
    if _DEFAULT_PAINT_PATH_CONFIG is None:
        _DEFAULT_PAINT_PATH_CONFIG = PaintPathConfig.from_yaml()
    return _DEFAULT_PAINT_PATH_CONFIG.copy_with_overrides()


def _resolve_config_path(filename):
    install_prefix = os.path.dirname(PROJECT_DIR)
    candidates = [
        os.path.join(PROJECT_DIR, "config", filename),
        os.path.join(SCRIPT_DIR, filename),
        os.path.join(SCRIPT_DIR, "config", filename),
        os.path.join(install_prefix, "share", PROJECT_NAME, "config", filename),
        os.path.join(os.getcwd(), "config", filename),
        os.path.join(os.getcwd(), filename),
    ]

    seen = set()
    for path in candidates:
        normalized = os.path.abspath(path)
        if normalized in seen:
            continue
        seen.add(normalized)
        if os.path.isfile(normalized):
            return normalized
    if candidates:
        return os.path.abspath(candidates[0])
    return filename


def _load_paint_path_config(path=None):
    config_path = path or _resolve_config_path("paint_path.yaml")
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}

    if not isinstance(loaded, dict) or not isinstance(loaded.get("paint_path"), dict):
        raise ValueError(
            f"Config file must contain top-level 'paint_path' mapping: {config_path}"
        )
    return dict(loaded["paint_path"])


def _resolve_paint_path_config(config=None, config_path=None, overrides=None):
    if config is None and config_path is None:
        return get_default_paint_path_config().copy_with_overrides(overrides)
    if config is None:
        return PaintPathConfig.from_yaml(config_path, overrides=overrides)
    if isinstance(config, PaintPathConfig):
        return config.copy_with_overrides(overrides)
    return PaintPathConfig.from_dict(config, overrides=overrides)


PAINT_TRAJECTORY_CSV_HEADER = [
    "trajectory_index",
    "waypoint_index",
    "frame_id",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "paint",
    "time_from_start",
]


def _default_csv_output_dir():
    return os.path.join(PROJECT_DIR, "csv")


def _resolve_csv_output_dir(csv_output_dir):
    path = "" if csv_output_dir is None else str(csv_output_dir).strip()
    if not path:
        return _default_csv_output_dir()
    return os.path.abspath(os.path.expanduser(path))


def _csv_name(name, default):
    value = str(name or "").strip() or str(default)
    root, ext = os.path.splitext(value)
    if not ext:
        value = f"{value}.csv"
    return value


def _trajectory_csv_name(template, trajectory_index):
    name_template = _csv_name(
        template,
        "trajectory_{trajectory_index:03d}.csv",
    )
    try:
        return name_template.format(
            trajectory_index=int(trajectory_index),
            trajectory_number=int(trajectory_index) + 1,
            index=int(trajectory_index),
            number=int(trajectory_index) + 1,
        )
    except (IndexError, KeyError, ValueError):
        return name_template


def _unique_csv_path(path, used_paths):
    candidate = os.path.abspath(path)
    if candidate not in used_paths:
        used_paths.add(candidate)
        return candidate

    root, ext = os.path.splitext(candidate)
    suffix = 1
    while True:
        candidate = f"{root}_{suffix:03d}{ext}"
        if candidate not in used_paths:
            used_paths.add(candidate)
            return candidate
        suffix += 1


def _csv_row(frame_id, trajectory_index, waypoint_index, waypoint):
    position = list(getattr(waypoint, "position", (0.0, 0.0, 0.0)))
    orientation = list(getattr(waypoint, "orientation", (0.0, 0.0, 0.0, 1.0)))
    return [
        int(trajectory_index),
        int(waypoint_index),
        str(frame_id),
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(orientation[0]),
        float(orientation[1]),
        float(orientation[2]),
        float(orientation[3]),
        int(bool(getattr(waypoint, "paint", False))),
        float(getattr(waypoint, "time_from_start", 0.0)),
    ]


def _write_csv(path, rows):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(PAINT_TRAJECTORY_CSV_HEADER)
        writer.writerows(rows)


def save_paint_trajectory_csv_files(
    paint_trajectory,
    csv_output_dir="",
    paint_trajectory_csv_name="paint_trajectory.csv",
    trajectory_csv_name_format="trajectory_{trajectory_index:03d}.csv",
):
    output_dir = _resolve_csv_output_dir(csv_output_dir)
    os.makedirs(output_dir, exist_ok=True)

    frame_id = str(getattr(paint_trajectory, "frame_id", ""))
    trajectories = list(getattr(paint_trajectory, "trajectories", []))
    all_rows = []
    trajectory_paths = []
    used_paths = set()

    full_path = _unique_csv_path(
        os.path.join(
            output_dir,
            _csv_name(paint_trajectory_csv_name, "paint_trajectory.csv"),
        ),
        used_paths,
    )

    for trajectory_index, trajectory in enumerate(trajectories):
        waypoints = list(getattr(trajectory, "waypoints", []))
        rows = [
            _csv_row(frame_id, trajectory_index, waypoint_index, waypoint)
            for waypoint_index, waypoint in enumerate(waypoints)
        ]
        all_rows.extend(rows)

        trajectory_path = _unique_csv_path(
            os.path.join(
                output_dir,
                _trajectory_csv_name(trajectory_csv_name_format, trajectory_index),
            ),
            used_paths,
        )
        _write_csv(trajectory_path, rows)
        trajectory_paths.append(trajectory_path)

    _write_csv(full_path, all_rows)
    return {
        "paint_trajectory": full_path,
        "trajectories": trajectory_paths,
    }


def _legacy_raster_zigzag_from_kwargs(*kwargs_maps):
    for kwargs in reversed(kwargs_maps):
        source = dict(kwargs or {})
        if "raster_zigzag" in source:
            return bool(source["raster_zigzag"])
    return None


def _structure_and_export_paint_trajectory(
    painted,
    structuring_config,
    structuring_kwargs=None,
    legacy_spline_kwargs=None,
    spline_kwargs=None,
):
    explicit_structuring_kwargs = dict(structuring_kwargs or {})
    options = {
        **structuring_config.to_kwargs(),
        **explicit_structuring_kwargs,
    }
    if "raster_zigzag" not in explicit_structuring_kwargs:
        legacy_raster_zigzag = _legacy_raster_zigzag_from_kwargs(
            legacy_spline_kwargs,
            spline_kwargs,
        )
        if legacy_raster_zigzag is not None:
            options["raster_zigzag"] = legacy_raster_zigzag

    painted["paint_spline_raster_zigzag"] = bool(options.get("raster_zigzag", True))
    paint_trajectory = structure_paint_trajectory(
        painted,
        frame_id=options.get("frame_id", "world"),
        raster_zigzag=options.get("raster_zigzag"),
    )

    csv_files = {}
    if bool(options.get("save_csv", False)):
        csv_files = save_paint_trajectory_csv_files(
            paint_trajectory,
            csv_output_dir=options.get("csv_output_dir", ""),
            paint_trajectory_csv_name=options.get(
                "paint_trajectory_csv_name",
                "paint_trajectory.csv",
            ),
            trajectory_csv_name_format=options.get(
                "trajectory_csv_name_format",
                "trajectory_{trajectory_index:03d}.csv",
            ),
        )
    return paint_trajectory, csv_files


def _generate_stage4_result(
    point_cloud_xyz,
    config=None,
    config_path=None,
    overrides=None,
    extraction_kwargs=None,
    correction_kwargs=None,
    offset_kwargs=None,
    paint_spline_kwargs=None,
    spline_kwargs=None,
    return_config=False,
):
    paint_config = _resolve_paint_path_config(
        config=config,
        config_path=config_path,
        overrides=overrides,
    )

    extraction_kwargs = {
        **paint_config.surface_extraction.to_kwargs(),
        **dict(extraction_kwargs or {}),
    }
    correction_kwargs = {
        **paint_config.correction.to_kwargs(),
        **dict(correction_kwargs or {}),
    }
    offset_kwargs = {
        **paint_config.offset.to_kwargs(),
        **dict(offset_kwargs or {}),
    }
    paint_spline_kwargs = {
        **paint_config.spline.to_kwargs(),
        **dict(paint_spline_kwargs or {}),
        **dict(spline_kwargs or {}),
    }

    extraction = generate_surface_spline_rows(point_cloud_xyz, **extraction_kwargs)
    corrected = correct_surface_spline_rows(extraction, **correction_kwargs)
    offset = generate_offset_rows(corrected, **offset_kwargs)
    painted = generate_paint_spline_rows(offset, **paint_spline_kwargs)
    if return_config:
        return painted, paint_config
    return painted


def generate_paint_trajectory_debug(
    point_cloud_xyz,
    config=None,
    config_path=None,
    overrides=None,
    extraction_kwargs=None,
    correction_kwargs=None,
    offset_kwargs=None,
    paint_spline_kwargs=None,
    spline_kwargs=None,
    structuring_kwargs=None,
):
    painted, paint_config = _generate_stage4_result(
        point_cloud_xyz,
        config=config,
        config_path=config_path,
        overrides=overrides,
        extraction_kwargs=extraction_kwargs,
        correction_kwargs=correction_kwargs,
        offset_kwargs=offset_kwargs,
        paint_spline_kwargs=paint_spline_kwargs,
        spline_kwargs=spline_kwargs,
        return_config=True,
    )
    paint_trajectory, csv_files = _structure_and_export_paint_trajectory(
        painted,
        paint_config.structuring,
        structuring_kwargs=structuring_kwargs,
        legacy_spline_kwargs=paint_spline_kwargs,
        spline_kwargs=spline_kwargs,
    )
    debug_result = dict(painted)
    debug_result["paint_trajectory"] = paint_trajectory
    debug_result["paint_trajectory_csv_files"] = csv_files
    return debug_result


def generate_paint_trajectory(
    point_cloud_xyz,
    config=None,
    config_path=None,
    overrides=None,
    extraction_kwargs=None,
    correction_kwargs=None,
    offset_kwargs=None,
    paint_spline_kwargs=None,
    spline_kwargs=None,
    structuring_kwargs=None,
):
    painted, paint_config = _generate_stage4_result(
        point_cloud_xyz,
        config=config,
        config_path=config_path,
        overrides=overrides,
        extraction_kwargs=extraction_kwargs,
        correction_kwargs=correction_kwargs,
        offset_kwargs=offset_kwargs,
        paint_spline_kwargs=paint_spline_kwargs,
        spline_kwargs=spline_kwargs,
        return_config=True,
    )
    paint_trajectory, _ = _structure_and_export_paint_trajectory(
        painted,
        paint_config.structuring,
        structuring_kwargs=structuring_kwargs,
        legacy_spline_kwargs=paint_spline_kwargs,
        spline_kwargs=spline_kwargs,
    )
    return paint_trajectory


generate_paint_path_debug = generate_paint_trajectory_debug
generate_paint_path = generate_paint_trajectory
