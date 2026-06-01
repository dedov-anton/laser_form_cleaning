from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from src.generators.bottom_ring.params import ProfileWaypoint

__all__ = ["PassStop", "ProfileWaypoint", "RadialPass", "RingTrajectory"]


@dataclass
class PassStop:
    position: tuple[float, float, float]
    pose_type: str
    tilt_deg: float


@dataclass
class RadialPass:
    index: int
    angle_deg: float
    stops: list[PassStop]
    outer_to_inner: bool

    @property
    def points(self) -> list[tuple[float, float, float]]:
        return [stop.position for stop in self.stops]

    @property
    def start(self) -> tuple[float, float, float]:
        return self.stops[0].position

    @property
    def finish(self) -> tuple[float, float, float]:
        return self.stops[-1].position


@dataclass
class RingTrajectory:
    inner_radius_mm: float
    outer_radius_mm: float
    beam_width_mm: float
    sector_count: int
    overlap_outer_mm: float
    overlap_inner_mm: float
    start_point_mm: tuple[float, float, float]
    finish_point_mm: tuple[float, float, float]
    entry_sector_start_deg: Optional[float]
    entry_sector_end_deg: Optional[float]
    passes: list[RadialPass]
    travel_segments: list
    poses: list
