from __future__ import annotations

from dataclasses import asdict, dataclass


SECTOR_COUNT = 8
SECTOR_WIDTH_DEG = 360.0 / SECTOR_COUNT


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

    def to_dict(self) -> dict:
        return asdict(self)
