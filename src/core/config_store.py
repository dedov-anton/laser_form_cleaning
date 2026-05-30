from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

from src.core.models import ProfileWaypoint

DEFAULT_BEAM_WIDTH_MM = 100.0
DEFAULT_POINT_MM = 0.0


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


@dataclass
class SavedParams:
    inner_radius_mm: Optional[float] = None
    ring_width_mm: Optional[float] = None
    beam_width_mm: float = DEFAULT_BEAM_WIDTH_MM
    sector_count: Optional[int] = None
    start_x_mm: float = DEFAULT_POINT_MM
    start_y_mm: float = DEFAULT_POINT_MM
    start_z_mm: float = DEFAULT_POINT_MM
    finish_x_mm: float = DEFAULT_POINT_MM
    finish_y_mm: float = DEFAULT_POINT_MM
    finish_z_mm: float = DEFAULT_POINT_MM
    entry_sector_start_deg: Optional[float] = None
    entry_sector_end_deg: Optional[float] = None
    inner_tilt_deg: float = 0.0
    outer_tilt_deg: float = 0.0
    profile_point_1: ProfileWaypoint = field(default_factory=ProfileWaypoint)
    profile_point_2: ProfileWaypoint = field(default_factory=ProfileWaypoint)

    def to_dict(self) -> dict:
        data = asdict(self)
        return {
            key: value
            for key, value in data.items()
            if value is not None or key.startswith(("start_", "finish_"))
        }


def config_path(project_root: Optional[Path] = None) -> Path:
    root = project_root or Path(__file__).resolve().parents[2]
    return root / "config" / "last_params.json"


def load_params(project_root: Optional[Path] = None) -> SavedParams:
    path = config_path(project_root)
    if not path.exists():
        return SavedParams()

    with path.open(encoding="utf-8") as file:
        data = json.load(file)

    return SavedParams(
        inner_radius_mm=data.get("inner_radius_mm"),
        ring_width_mm=data.get("ring_width_mm"),
        beam_width_mm=data.get("beam_width_mm", DEFAULT_BEAM_WIDTH_MM),
        sector_count=data.get("sector_count"),
        start_x_mm=float(data.get("start_x_mm", DEFAULT_POINT_MM)),
        start_y_mm=float(data.get("start_y_mm", DEFAULT_POINT_MM)),
        start_z_mm=float(data.get("start_z_mm", DEFAULT_POINT_MM)),
        finish_x_mm=float(data.get("finish_x_mm", DEFAULT_POINT_MM)),
        finish_y_mm=float(data.get("finish_y_mm", DEFAULT_POINT_MM)),
        finish_z_mm=float(data.get("finish_z_mm", DEFAULT_POINT_MM)),
        entry_sector_start_deg=data.get("entry_sector_start_deg"),
        entry_sector_end_deg=data.get("entry_sector_end_deg"),
        inner_tilt_deg=float(data.get("inner_tilt_deg", 0.0)),
        outer_tilt_deg=float(data.get("outer_tilt_deg", 0.0)),
        profile_point_1=_load_profile_waypoint(data, "profile_point_1"),
        profile_point_2=_load_profile_waypoint(data, "profile_point_2"),
    )


def save_params(params: SavedParams, project_root: Optional[Path] = None) -> None:
    path = config_path(project_root)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8") as file:
        json.dump(params.to_dict(), file, indent=2, ensure_ascii=False)
