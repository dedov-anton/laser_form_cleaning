from __future__ import annotations

from dataclasses import asdict

from src.common.geometry import Point3
from src.common.trajectory import WorkSegment, WorkTrajectory
from src.generators.bottom_ring.models import RingTrajectory
from src.generators.bottom_ring.params import BottomRingParams

GENERATOR_ID = "bottom_ring"
GENERATOR_VERSION = "1.0"


def _work_group(pass_index: int, pass_count: int) -> str:
    if pass_index == 0:
        return "work_first"
    if pass_count > 1 and pass_index == pass_count - 1:
        return "work_last"
    return "work"


def to_work_trajectory(
    ring_trajectory: RingTrajectory,
    params: BottomRingParams,
) -> WorkTrajectory:
    pass_count = len(ring_trajectory.passes)
    work_segments: list[WorkSegment] = []

    for pass_index, radial_pass in enumerate(ring_trajectory.passes):
        group = _work_group(pass_index, pass_count)
        for point_index in range(len(radial_pass.points) - 1):
            work_segments.append(
                WorkSegment(
                    pass_index=pass_index,
                    start=radial_pass.points[point_index],
                    finish=radial_pass.points[point_index + 1],
                    group=group,
                )
            )

    return WorkTrajectory(
        generator_id=GENERATOR_ID,
        generator_version=GENERATOR_VERSION,
        params_snapshot=asdict(params),
        start_point_mm=ring_trajectory.start_point_mm,
        finish_point_mm=ring_trajectory.finish_point_mm,
        poses=ring_trajectory.poses,
        travel_segments=ring_trajectory.travel_segments,
        work_segments=work_segments,
        reference_geometry={
            "circles": [
                {
                    "radius_mm": ring_trajectory.inner_radius_mm,
                    "label": "reference_inner",
                },
                {
                    "radius_mm": ring_trajectory.outer_radius_mm,
                    "label": "reference_outer",
                },
            ]
        },
        metadata={
            "inner_radius_mm": ring_trajectory.inner_radius_mm,
            "outer_radius_mm": ring_trajectory.outer_radius_mm,
            "beam_width_mm": ring_trajectory.beam_width_mm,
            "sector_count": ring_trajectory.sector_count,
            "overlap_outer_mm": ring_trajectory.overlap_outer_mm,
            "overlap_inner_mm": ring_trajectory.overlap_inner_mm,
            "entry_sector_start_deg": ring_trajectory.entry_sector_start_deg,
            "entry_sector_end_deg": ring_trajectory.entry_sector_end_deg,
            "pass_count": pass_count,
        },
    )
