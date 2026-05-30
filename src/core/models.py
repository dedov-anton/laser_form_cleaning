from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Point3 = Tuple[float, float, float]

DEFAULT_TOOL_AXIS_Z: Point3 = (0.0, 0.0, -1.0)
TOOL_AXIS_VIS_LENGTH_MM = 100.0
TOOL_RADIAL_VIS_LENGTH_MM = 30.0


def axis_endpoint(position: Point3, axis: Point3, length_mm: float) -> Point3:
    return (
        position[0] + axis[0] * length_mm,
        position[1] + axis[1] * length_mm,
        position[2] + axis[2] * length_mm,
    )


def approach_line(position: Point3, tool_axis_z: Point3, length_mm: float) -> Tuple[Point3, Point3]:
    tool_tip = position
    tool_base = axis_endpoint(position, tool_axis_z, -length_mm)
    return tool_base, tool_tip


@dataclass(frozen=True)
class ProfileWaypoint:
    distance_from_outer_mm: float = 0.0
    z_offset_mm: float = 0.0
    tilt_deg: float = 0.0


@dataclass
class PassStop:
    position: Point3
    pose_type: str
    tilt_deg: float


@dataclass
class Pose6D:
    index: int
    pose_type: str
    position: Point3
    tool_axis_z: Point3
    tool_axis_x: Point3

    @property
    def tool_axis(self) -> Point3:
        return self.tool_axis_z


@dataclass
class RadialPass:
    index: int
    angle_deg: float
    stops: List[PassStop]
    outer_to_inner: bool

    @property
    def points(self) -> List[Point3]:
        return [stop.position for stop in self.stops]

    @property
    def start(self) -> Point3:
        return self.stops[0].position

    @property
    def finish(self) -> Point3:
        return self.stops[-1].position


@dataclass
class TravelSegment:
    index: int
    segment_type: str
    start: Point3
    finish: Point3


@dataclass
class RingTrajectory:
    inner_radius_mm: float
    outer_radius_mm: float
    beam_width_mm: float
    sector_count: int
    overlap_outer_mm: float
    overlap_inner_mm: float
    start_point_mm: Point3
    finish_point_mm: Point3
    entry_sector_start_deg: Optional[float]
    entry_sector_end_deg: Optional[float]
    passes: List[RadialPass]
    travel_segments: List[TravelSegment]
    poses: List[Pose6D]


@dataclass
class RingParams:
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
