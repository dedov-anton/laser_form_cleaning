from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any, Optional

from src.common.project import DEFAULT_BEAM_WIDTH_MM, DEFAULT_POINT_MM, FormProject
from src.generators.bottom_ring.params import ProfileWaypoint
from src.storage.trajectory_codec import trajectory_from_dict, trajectory_to_dict

LEGACY_PARAMS_FILE = "last_params.json"
PROJECT_FILE = "project.json"


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
        bottom_ring=bottom_ring,
        top_ring=dict(data.get("top_ring", {})),
        cylinder_wall=dict(data.get("cylinder_wall", {})),
        trajectories=dict(
            data.get(
                "trajectories",
                {"bottom_ring": None, "top_ring": None, "cylinder_wall": None},
            )
        ),
        trajectories_local=dict(
            data.get(
                "trajectories_local",
                {"bottom_ring": None, "top_ring": None, "cylinder_wall": None},
            )
        ),
    )


def save_project(project: FormProject, project_root: Optional[Path] = None) -> None:
    path = project_path(project_root)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8") as file:
        json.dump(asdict(project), file, indent=2, ensure_ascii=False)


def load_trajectory(project: FormProject, generator_id: str):
    from src.common.trajectory import WorkTrajectory

    raw = project.trajectories.get(generator_id)
    if raw is None:
        return None
    return trajectory_from_dict(raw)


def store_trajectory(project: FormProject, generator_id: str, trajectory) -> None:
    project.trajectories[generator_id] = trajectory_to_dict(trajectory)


def load_local_trajectory(project: FormProject, generator_id: str):
    raw = project.trajectories_local.get(generator_id)
    if raw is None:
        return None
    return trajectory_from_dict(raw)


def store_local_trajectory(project: FormProject, generator_id: str, trajectory) -> None:
    project.trajectories_local[generator_id] = trajectory_to_dict(trajectory)
