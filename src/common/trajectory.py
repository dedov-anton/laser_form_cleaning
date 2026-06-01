from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from src.common.geometry import Point3, Pose6D, TravelSegment


@dataclass
class WorkSegment:
    pass_index: int
    start: Point3
    finish: Point3
    group: str


@dataclass
class WorkTrajectory:
    generator_id: str
    generator_version: str
    params_snapshot: dict
    start_point_mm: Point3
    finish_point_mm: Point3
    poses: List[Pose6D]
    travel_segments: List[TravelSegment]
    work_segments: List[WorkSegment]
    reference_geometry: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def pass_count(self) -> int:
        if not self.work_segments:
            return 0
        return max(segment.pass_index for segment in self.work_segments) + 1

    @property
    def pose_count(self) -> int:
        return len(self.poses)

    def circle_radii_mm(self) -> List[float]:
        circles = self.reference_geometry.get("circles", [])
        return [float(item["radius_mm"]) for item in circles]
