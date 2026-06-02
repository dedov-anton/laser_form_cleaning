from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any, Optional

from src.common.project import (
    DEFAULT_BEAM_WIDTH_MM,
    DEFAULT_POINT_MM,
    DEFAULT_ROBOT_ACCELERATION,
    DEFAULT_ROBOT_BLEND_RADIUS_MM,
    DEFAULT_ROBOT_IP,
    DEFAULT_ROBOT_VELOCITY,
    DEFAULT_TOOL_MOUNT_ROTATION_DEG,
    FormProject,
)
from src.generators.bottom_ring.params import ProfileWaypoint
from src.storage.trajectory_codec import trajectory_from_dict, trajectory_to_dict

LEGACY_PARAMS_FILE = "last_params.json"
PROJECT_FILE = "project.json"
TRAJECTORIES_DIR = "trajectories"

GENERATOR_IDS = ("bottom_ring", "top_ring", "cylinder_wall")


def _load_profile_waypoint(data: dict, prefix: str) -> ProfileWaypoint:
    nested = data.get(prefix)
    if isinstance(nested, dict):
        return ProfileWaypoint(
            distance_from_outer_mm=float(nested.get("distance_from_outer_mm", 0.0)),
            z_offset_mm=float(nested.get("z_offset_mm", 0.0)),
            tilt_deg=float(nested.get("tilt_deg", 0.0)),
        )
    return ProfileWaypoint(
        distance_from_outer_mm=float(data.get(f"{prefix}_dist_mm", 0.0)),
        z_offset_mm=float(data.get(f"{prefix}_z_mm", 0.0)),
        tilt_deg=float(data.get(f"{prefix}_tilt_deg", 0.0)),
    )


def project_path(project_root: Optional[Path] = None) -> Path:
    root = project_root or Path(__file__).resolve().parents[2]
    return root / "config" / PROJECT_FILE


def legacy_params_path(project_root: Optional[Path] = None) -> Path:
    root = project_root or Path(__file__).resolve().parents[2]
    return root / "config" / LEGACY_PARAMS_FILE


def trajectories_dir(project_root: Optional[Path] = None) -> Path:
    root = project_root or Path(__file__).resolve().parents[2]
    return root / "config" / TRAJECTORIES_DIR


def _default_trajectory_slots() -> dict[str, None]:
    return {generator_id: None for generator_id in GENERATOR_IDS}


def _resolve_trajectory_path(project_root: Path, relative_path: str) -> Path:
    return project_root / "config" / relative_path.replace("\\", "/")


def _world_trajectory_filename(generator_id: str) -> str:
    return f"{generator_id}.json"


def _local_trajectory_filename(generator_id: str) -> str:
    return f"{generator_id}_local.json"


def _relative_trajectory_path(filename: str) -> str:
    return f"{TRAJECTORIES_DIR}/{filename}"


def save_trajectory_file(path: Path, trajectory) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(trajectory_to_dict(trajectory), file, indent=2, ensure_ascii=False)


def load_trajectory_file(path: Path):
    with path.open(encoding="utf-8") as file:
        return trajectory_from_dict(json.load(file))


def _bottom_ring_params_from_legacy(data: dict[str, Any]) -> dict[str, Any]:
    return {
        "inner_radius_mm": data.get("inner_radius_mm"),
        "ring_width_mm": data.get("ring_width_mm"),
        "sector_count": data.get("sector_count"),
        "entry_sector_start_deg": data.get("entry_sector_start_deg"),
        "entry_sector_end_deg": data.get("entry_sector_end_deg"),
        "inner_tilt_deg": float(data.get("inner_tilt_deg", 0.0)),
        "outer_tilt_deg": float(data.get("outer_tilt_deg", 0.0)),
        "profile_point_1": asdict(_load_profile_waypoint(data, "profile_point_1")),
        "profile_point_2": asdict(_load_profile_waypoint(data, "profile_point_2")),
    }


def _migrate_legacy_params(project_root: Path) -> Optional[FormProject]:
    legacy_path = legacy_params_path(project_root)
    if not legacy_path.exists():
        return None

    with legacy_path.open(encoding="utf-8") as file:
        data = json.load(file)

    return FormProject(
        beam_width_mm=float(data.get("beam_width_mm", DEFAULT_BEAM_WIDTH_MM)),
        start_x_mm=float(data.get("start_x_mm", DEFAULT_POINT_MM)),
        start_y_mm=float(data.get("start_y_mm", DEFAULT_POINT_MM)),
        start_z_mm=float(data.get("start_z_mm", DEFAULT_POINT_MM)),
        finish_x_mm=float(data.get("finish_x_mm", DEFAULT_POINT_MM)),
        finish_y_mm=float(data.get("finish_y_mm", DEFAULT_POINT_MM)),
        finish_z_mm=float(data.get("finish_z_mm", DEFAULT_POINT_MM)),
        bottom_ring=_bottom_ring_params_from_legacy(data),
    )


def _migrate_start_finish_to_bottom_ring(data: dict[str, Any]) -> dict[str, Any]:
    bottom_ring = dict(data.get("bottom_ring", {}))
    for key in (
        "start_x_mm",
        "start_y_mm",
        "start_z_mm",
        "finish_x_mm",
        "finish_y_mm",
        "finish_z_mm",
    ):
        if key not in bottom_ring and key in data:
            bottom_ring[key] = data[key]
    return bottom_ring


def _load_trajectory_slots(data: dict[str, Any], key: str) -> dict[str, Optional[dict]]:
    return dict(
        data.get(
            key,
            _default_trajectory_slots(),
        )
    )


def _load_trajectory_file_slots(data: dict[str, Any], key: str) -> dict[str, Optional[str]]:
    return dict(
        data.get(
            key,
            _default_trajectory_slots(),
        )
    )


def load_project(project_root: Optional[Path] = None) -> FormProject:
    root = project_root or Path(__file__).resolve().parents[2]
    path = project_path(root)

    if not path.exists():
        migrated = _migrate_legacy_params(root)
        return migrated or FormProject()

    with path.open(encoding="utf-8") as file:
        data = json.load(file)

    bottom_ring = _migrate_start_finish_to_bottom_ring(data)

    return FormProject(
        version=str(data.get("version", "1")),
        beam_width_mm=float(data.get("beam_width_mm", DEFAULT_BEAM_WIDTH_MM)),
        start_x_mm=float(data.get("start_x_mm", DEFAULT_POINT_MM)),
        start_y_mm=float(data.get("start_y_mm", DEFAULT_POINT_MM)),
        start_z_mm=float(data.get("start_z_mm", DEFAULT_POINT_MM)),
        finish_x_mm=float(data.get("finish_x_mm", DEFAULT_POINT_MM)),
        finish_y_mm=float(data.get("finish_y_mm", DEFAULT_POINT_MM)),
        finish_z_mm=float(data.get("finish_z_mm", DEFAULT_POINT_MM)),
        fixture_x_mm=float(data.get("fixture_x_mm", DEFAULT_POINT_MM)),
        fixture_y_mm=float(data.get("fixture_y_mm", DEFAULT_POINT_MM)),
        fixture_z_mm=float(data.get("fixture_z_mm", DEFAULT_POINT_MM)),
        fixture_rx_deg=float(data.get("fixture_rx_deg", DEFAULT_POINT_MM)),
        fixture_ry_deg=float(data.get("fixture_ry_deg", DEFAULT_POINT_MM)),
        fixture_rz_deg=float(data.get("fixture_rz_deg", DEFAULT_POINT_MM)),
        robot_blend_radius_mm=float(
            data.get("robot_blend_radius_mm", DEFAULT_ROBOT_BLEND_RADIUS_MM)
        ),
        robot_velocity=float(data.get("robot_velocity", DEFAULT_ROBOT_VELOCITY)),
        robot_acceleration=float(
            data.get("robot_acceleration", DEFAULT_ROBOT_ACCELERATION)
        ),
        robot_ip=str(data.get("robot_ip", DEFAULT_ROBOT_IP)),
        tool_mount_rotation_deg=float(
            data.get("tool_mount_rotation_deg", DEFAULT_TOOL_MOUNT_ROTATION_DEG)
        ),
        bottom_ring=bottom_ring,
        top_ring=dict(data.get("top_ring", {})),
        cylinder_wall=dict(data.get("cylinder_wall", {})),
        trajectories=_load_trajectory_slots(data, "trajectories"),
        trajectories_local=_load_trajectory_slots(data, "trajectories_local"),
        trajectory_files=_load_trajectory_file_slots(data, "trajectory_files"),
        trajectory_files_local=_load_trajectory_file_slots(data, "trajectory_files_local"),
    )


def _migrate_embedded_trajectory_to_file(
    project: FormProject,
    project_root: Path,
    generator_id: str,
    *,
    local: bool,
) -> None:
    embedded = (
        project.trajectories_local.get(generator_id)
        if local
        else project.trajectories.get(generator_id)
    )
    if embedded is None:
        return

    filename = (
        _local_trajectory_filename(generator_id)
        if local
        else _world_trajectory_filename(generator_id)
    )
    relative_path = _relative_trajectory_path(filename)
    file_path = _resolve_trajectory_path(project_root, relative_path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    with file_path.open("w", encoding="utf-8") as file:
        json.dump(embedded, file, indent=2, ensure_ascii=False)

    if local:
        project.trajectory_files_local[generator_id] = relative_path
        project.trajectories_local[generator_id] = None
    else:
        project.trajectory_files[generator_id] = relative_path
        project.trajectories[generator_id] = None


def _migrate_embedded_trajectories(project: FormProject, project_root: Path) -> None:
    for generator_id in GENERATOR_IDS:
        _migrate_embedded_trajectory_to_file(project, project_root, generator_id, local=False)
        _migrate_embedded_trajectory_to_file(project, project_root, generator_id, local=True)


def save_project(project: FormProject, project_root: Optional[Path] = None) -> None:
    root = project_root or Path(__file__).resolve().parents[2]
    _migrate_embedded_trajectories(project, root)

    path = project_path(root)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8") as file:
        json.dump(asdict(project), file, indent=2, ensure_ascii=False)


def _load_trajectory_from_project(
    project: FormProject,
    generator_id: str,
    *,
    local: bool,
    project_root: Optional[Path] = None,
):
    root = project_root or Path(__file__).resolve().parents[2]
    file_slots = project.trajectory_files_local if local else project.trajectory_files
    embedded_slots = project.trajectories_local if local else project.trajectories

    relative_path = file_slots.get(generator_id)
    if relative_path:
        file_path = _resolve_trajectory_path(root, relative_path)
        if file_path.exists():
            return load_trajectory_file(file_path)

    raw = embedded_slots.get(generator_id)
    if raw is None:
        return None
    return trajectory_from_dict(raw)


def load_trajectory(project: FormProject, generator_id: str, project_root: Optional[Path] = None):
    return _load_trajectory_from_project(project, generator_id, local=False, project_root=project_root)


def store_trajectory(
    project: FormProject,
    generator_id: str,
    trajectory,
    project_root: Optional[Path] = None,
) -> None:
    root = project_root or Path(__file__).resolve().parents[2]
    relative_path = _relative_trajectory_path(_world_trajectory_filename(generator_id))
    file_path = _resolve_trajectory_path(root, relative_path)
    save_trajectory_file(file_path, trajectory)
    project.trajectory_files[generator_id] = relative_path
    project.trajectories[generator_id] = None


def load_local_trajectory(
    project: FormProject,
    generator_id: str,
    project_root: Optional[Path] = None,
):
    return _load_trajectory_from_project(project, generator_id, local=True, project_root=project_root)


def store_local_trajectory(
    project: FormProject,
    generator_id: str,
    trajectory,
    project_root: Optional[Path] = None,
) -> None:
    root = project_root or Path(__file__).resolve().parents[2]
    relative_path = _relative_trajectory_path(_local_trajectory_filename(generator_id))
    file_path = _resolve_trajectory_path(root, relative_path)
    save_trajectory_file(file_path, trajectory)
    project.trajectory_files_local[generator_id] = relative_path
    project.trajectories_local[generator_id] = None
