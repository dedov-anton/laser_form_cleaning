from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Optional


@dataclass(frozen=True)
class ProfileWaypoint:
    distance_from_outer_mm: float = 0.0
    z_offset_mm: float = 0.0
    tilt_deg: float = 0.0


@dataclass
class BottomRingParams:
    inner_radius_mm: float
    ring_width_mm: float
    beam_width_mm: float
    sector_count: int
    entry_sector_start_deg: Optional[float] = None
    entry_sector_end_deg: Optional[float] = None
    inner_tilt_deg: float = 0.0
    outer_tilt_deg: float = 0.0
    profile_point_1: ProfileWaypoint = field(default_factory=ProfileWaypoint)
    profile_point_2: ProfileWaypoint = field(default_factory=ProfileWaypoint)

    @property
    def outer_radius_mm(self) -> float:
        return self.inner_radius_mm + self.ring_width_mm

    @property
    def has_entry_sector(self) -> bool:
        return (
            self.entry_sector_start_deg is not None
            and self.entry_sector_end_deg is not None
        )

    def to_dict(self) -> dict:
        return asdict(self)


RingParams = BottomRingParams
