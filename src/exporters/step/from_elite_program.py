from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

from src.common.geometry import TOOL_AXIS_VIS_LENGTH_MM, Point3, approach_line
from src.exporters.robot.orientation import RollPitchYaw, rpy_to_rotation_rows
from src.exporters.step.writer import TCP_POINT_COLOR, TOOL_VIS_COLOR, StepWriter

_POSE_LINE_PATTERN = re.compile(
    r"pose_(?P<name>work_\d+)\s*=\s*\["
    r"(?P<x>[-\d.]+),\s*(?P<y>[-\d.]+),\s*(?P<z>[-\d.]+),\s*"
    r"(?P<roll>[-\d.]+),\s*(?P<pitch>[-\d.]+),\s*(?P<yaw>[-\d.]+)\s*\]"
)


@dataclass(frozen=True)
class EliteProgramWorkPose:
    """One pose_work_N line from Elite UP (meters, radians)."""

    index: int
    position_m: Point3
    rpy: RollPitchYaw


def parse_elite_program_work_poses(program: str) -> List[EliteProgramWorkPose]:
    matches = list(_POSE_LINE_PATTERN.finditer(program))
    if not matches:
        raise ValueError("В тексте УП не найдено ни одной строки pose_work_N")

    poses: List[EliteProgramWorkPose] = []
    for match in matches:
        name = match.group("name")
        work_index = int(name.split("_", 1)[1])
        poses.append(
            EliteProgramWorkPose(
                index=work_index,
                position_m=(
                    float(match.group("x")),
                    float(match.group("y")),
                    float(match.group("z")),
                ),
                rpy=(
                    float(match.group("roll")),
                    float(match.group("pitch")),
                    float(match.group("yaw")),
                ),
            )
        )

    poses.sort(key=lambda pose: pose.index)
    expected = list(range(len(poses)))
    actual = [pose.index for pose in poses]
    if actual != expected:
        raise ValueError(f"Индексы pose_work_N не непрерывны: {actual}")
    return poses


def _m_to_mm(point_m: Point3) -> Point3:
    return (point_m[0] * 1000.0, point_m[1] * 1000.0, point_m[2] * 1000.0)


def _fill_writer_from_elite_work_poses(
    writer: StepWriter, work_poses: Sequence[EliteProgramWorkPose]
) -> None:
    for pose in work_poses:
        position_mm = _m_to_mm(pose.position_m)
        writer.add_point_marker(position_mm, TCP_POINT_COLOR, "tcp")
        _, _, tcp_z = rpy_to_rotation_rows(*pose.rpy)
        axis_base, axis_tip = approach_line(
            position_mm, tcp_z, TOOL_AXIS_VIS_LENGTH_MM
        )
        writer.add_line(axis_base, axis_tip, TOOL_VIS_COLOR, "tool_axis")


def export_step_from_elite_program(program: str, filepath: Path | str) -> None:
    """STEP from Elite UP: TCP markers + one TCP +Z arrow per pose_work_N (no path links)."""
    work_poses = parse_elite_program_work_poses(program)
    writer = StepWriter()
    _fill_writer_from_elite_work_poses(writer, work_poses)
    writer.write(Path(filepath))
