from __future__ import annotations

from dataclasses import asdict, dataclass, field


SECTOR_COUNT = 8
SECTOR_WIDTH_DEG = 360.0 / SECTOR_COUNT


@dataclass(frozen=True)
class WallProfileWaypoint:
    z_down_mm: float = 0.0
    radial_inward_mm: float = 0.0
    tilt_deg: float = 0.0


@dataclass
class CylinderWallParams:
    inner_radius_mm: float
    wall_height_mm: float
    z_top_mm: float
    beam_width_mm: float
    passes_per_sector: int
    start_sector: int
    clockwise: bool
    top_tilt_deg: float = 0.0
    bottom_tilt_deg: float = 0.0
    profile_point_1: WallProfileWaypoint = field(default_factory=WallProfileWaypoint)
    profile_point_2: WallProfileWaypoint = field(default_factory=WallProfileWaypoint)

    def to_dict(self) -> dict:
        return asdict(self)
