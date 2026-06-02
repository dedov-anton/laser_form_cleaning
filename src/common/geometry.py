from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

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


@dataclass
class Pose6D:
    """6D tool pose at a trajectory stop.

    tool_axis_z — laser beam direction into the surface (STEP / trajectory convention).
    tool_axis_x — path tangent at the stop, projected onto the plane perpendicular to tool_axis_z.
    """

    index: int
    pose_type: str
    position: Point3
    tool_axis_z: Point3
    tool_axis_x: Point3

    @property
    def tool_axis(self) -> Point3:
        return self.tool_axis_z


@dataclass
class TravelSegment:
    index: int
    segment_type: str
    start: Point3
    finish: Point3
