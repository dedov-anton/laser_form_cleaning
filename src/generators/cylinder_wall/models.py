from __future__ import annotations

from dataclasses import dataclass

from src.common.geometry import Point3, Pose6D, TravelSegment


@dataclass
class PassStop:
    position: Point3
    pose_type: str
    tilt_deg: float


@dataclass
class VerticalPass:
    index: int
    sector_index: int
    angle_deg: float
    stops: list[PassStop]

    @property
    def points(self) -> list[Point3]:
        return [stop.position for stop in self.stops]

    @property
    def start(self) -> Point3:
        return self.stops[0].position

    @property
    def finish(self) -> Point3:
        return self.stops[-1].position


@dataclass
class CylinderTrajectory:
    inner_radius_mm: float
    wall_height_mm: float
    z_top_mm: float
    beam_width_mm: float
    passes_per_sector: int
    overlap_arc_mm: float
    start_sector: int
    clockwise: bool
    start_point_mm: Point3
    finish_point_mm: Point3
    passes: list[VerticalPass]
    travel_segments: list[TravelSegment]
    poses: list[Pose6D]
