from __future__ import annotations

from dataclasses import asdict

from src.common.trajectory import WorkSegment, WorkTrajectory
from src.generators.cylinder_wall.calc import build_sector_boundary_polylines
from src.generators.cylinder_wall.models import CylinderTrajectory
from src.generators.cylinder_wall.params import CylinderWallParams

GENERATOR_ID = "cylinder_wall"
GENERATOR_VERSION = "1.0"


def _work_group(pass_index: int, pass_count: int) -> str:
    if pass_index == 0:
        return "work_first"
    if pass_count > 1 and pass_index == pass_count - 1:
        return "work_last"
    return "work"


def to_work_trajectory(
    cylinder_trajectory: CylinderTrajectory,
    params: CylinderWallParams,
) -> WorkTrajectory:
    pass_count = len(cylinder_trajectory.passes)
    work_segments: list[WorkSegment] = []

    for pass_index, vertical_pass in enumerate(cylinder_trajectory.passes):
        group = _work_group(pass_index, pass_count)
        for point_index in range(len(vertical_pass.points) - 1):
            work_segments.append(
                WorkSegment(
                    pass_index=pass_index,
                    start=vertical_pass.points[point_index],
                    finish=vertical_pass.points[point_index + 1],
                    group=group,
                )
            )

    return WorkTrajectory(
        generator_id=GENERATOR_ID,
        generator_version=GENERATOR_VERSION,
        params_snapshot=asdict(params),
        start_point_mm=cylinder_trajectory.start_point_mm,
        finish_point_mm=cylinder_trajectory.finish_point_mm,
        poses=cylinder_trajectory.poses,
        travel_segments=cylinder_trajectory.travel_segments,
        work_segments=work_segments,
        reference_geometry={
            "circles": [
                {
                    "radius_mm": cylinder_trajectory.inner_radius_mm,
                    "z_mm": cylinder_trajectory.z_top_mm,
                    "label": "reference_top",
                },
                {
                    "radius_mm": cylinder_trajectory.inner_radius_mm,
                    "z_mm": cylinder_trajectory.z_top_mm - cylinder_trajectory.wall_height_mm,
                    "label": "reference_bottom",
                },
            ],
            "polylines": build_sector_boundary_polylines(params),
        },
        metadata={
            "inner_radius_mm": cylinder_trajectory.inner_radius_mm,
            "wall_height_mm": cylinder_trajectory.wall_height_mm,
            "z_top_mm": cylinder_trajectory.z_top_mm,
            "beam_width_mm": cylinder_trajectory.beam_width_mm,
            "passes_per_sector": cylinder_trajectory.passes_per_sector,
            "overlap_arc_mm": cylinder_trajectory.overlap_arc_mm,
            "start_sector": cylinder_trajectory.start_sector,
            "clockwise": cylinder_trajectory.clockwise,
            "pass_count": pass_count,
        },
    )
