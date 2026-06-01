from __future__ import annotations

from pathlib import Path

from src.common.geometry import (
    TOOL_AXIS_VIS_LENGTH_MM,
    TOOL_RADIAL_VIS_LENGTH_MM,
    approach_line,
    axis_endpoint,
)
from src.common.trajectory import WorkTrajectory
from src.exporters.step.writer import (
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
    from src.exporters.step.xcaf_writer import XcafStepWriter

    _XCAF_AVAILABLE = True
except ImportError:
    _XCAF_AVAILABLE = False


def _work_pass_color(pass_index: int, pass_count: int):
    if pass_index == 0:
        return FIRST_WORK_COLOR
    if pass_count > 1 and pass_index == pass_count - 1:
        return LAST_WORK_COLOR
    return WORK_COLOR


def _fill_writer(writer, trajectory: WorkTrajectory) -> None:
    for travel_segment in trajectory.travel_segments:
        writer.add_line(travel_segment.start, travel_segment.finish, TRAVEL_COLOR, "travel")

    pass_count = trajectory.pass_count
    for work_segment in trajectory.work_segments:
        color = _work_pass_color(work_segment.pass_index, pass_count)
        writer.add_line(
            work_segment.start,
            work_segment.finish,
            color,
            work_segment.group,
        )

    for circle in trajectory.reference_geometry.get("circles", []):
        writer.add_circle(
            float(circle["radius_mm"]),
            REFERENCE_COLOR,
            str(circle.get("label", "reference")),
            z_mm=float(circle.get("z_mm", 0.0)),
        )

    for polyline in trajectory.reference_geometry.get("polylines", []):
        writer.add_line(
            polyline["start"],
            polyline["finish"],
            REFERENCE_COLOR,
            str(polyline.get("group", "reference")),
        )

    writer.add_point_marker(trajectory.start_point_mm, START_POINT_COLOR, "start_point")
    writer.add_point_marker(trajectory.finish_point_mm, FINISH_POINT_COLOR, "finish_point")

    for pose in trajectory.poses:
        tool_base, tool_tip = approach_line(
            pose.position, pose.tool_axis_z, TOOL_AXIS_VIS_LENGTH_MM
        )
        writer.add_line(tool_base, tool_tip, TOOL_VIS_COLOR, "tool_axis")
        radial_end = axis_endpoint(
            pose.position, pose.tool_axis_x, TOOL_RADIAL_VIS_LENGTH_MM
        )
        writer.add_line(pose.position, radial_end, TOOL_VIS_COLOR, "tool_axis")


def export_trajectory(trajectory: WorkTrajectory, filepath: Path) -> None:
    writer = XcafStepWriter() if _XCAF_AVAILABLE else StepWriter()
    _fill_writer(writer, trajectory)
    writer.write(filepath)


# Backward-compatible alias.
export_ring_trajectory = export_trajectory
