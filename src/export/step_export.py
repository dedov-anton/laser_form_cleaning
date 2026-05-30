from __future__ import annotations

from pathlib import Path

from src.core.models import (
    TOOL_AXIS_VIS_LENGTH_MM,
    TOOL_RADIAL_VIS_LENGTH_MM,
    RingTrajectory,
    approach_line,
    axis_endpoint,
)
from src.export.step_writer import (
    FINISH_POINT_COLOR,
    FIRST_WORK_COLOR,
    LAST_WORK_COLOR,
    REFERENCE_COLOR,
    START_POINT_COLOR,
    TOOL_VIS_COLOR,
    TRAVEL_COLOR,
    WORK_COLOR,
    StepWriter,
)

try:
    from src.export.xcaf_writer import XcafStepWriter

    _XCAF_AVAILABLE = True
except ImportError:
    _XCAF_AVAILABLE = False


def _work_pass_color(pass_index: int, pass_count: int):
    if pass_index == 0:
        return FIRST_WORK_COLOR
    if pass_count > 1 and pass_index == pass_count - 1:
        return LAST_WORK_COLOR
    return WORK_COLOR


def _fill_writer(writer, trajectory: RingTrajectory) -> None:
    for travel_segment in trajectory.travel_segments:
        writer.add_line(travel_segment.start, travel_segment.finish, TRAVEL_COLOR, "travel")

    pass_count = len(trajectory.passes)
    for pass_index, radial_pass in enumerate(trajectory.passes):
        color = _work_pass_color(pass_index, pass_count)
        group = "work_first" if pass_index == 0 else (
            "work_last" if pass_count > 1 and pass_index == pass_count - 1 else "work"
        )
        for point_index in range(len(radial_pass.points) - 1):
            writer.add_line(
                radial_pass.points[point_index],
                radial_pass.points[point_index + 1],
                color,
                group,
            )

    writer.add_circle(trajectory.inner_radius_mm, REFERENCE_COLOR, "reference_inner")
    writer.add_circle(trajectory.outer_radius_mm, REFERENCE_COLOR, "reference_outer")
    writer.add_point_marker(trajectory.start_point_mm, START_POINT_COLOR, "start_point")
    writer.add_point_marker(trajectory.finish_point_mm, FINISH_POINT_COLOR, "finish_point")

    for pose in trajectory.poses:
        tool_base, tool_tip = approach_line(pose.position, pose.tool_axis_z, TOOL_AXIS_VIS_LENGTH_MM)
        writer.add_line(tool_base, tool_tip, TOOL_VIS_COLOR, "tool_axis")
        radial_end = axis_endpoint(pose.position, pose.tool_axis_x, TOOL_RADIAL_VIS_LENGTH_MM)
        writer.add_line(pose.position, radial_end, TOOL_VIS_COLOR, "tool_axis")


def export_ring_trajectory(trajectory: RingTrajectory, filepath: Path) -> None:
    writer = XcafStepWriter() if _XCAF_AVAILABLE else StepWriter()
    _fill_writer(writer, trajectory)
    writer.write(filepath)
