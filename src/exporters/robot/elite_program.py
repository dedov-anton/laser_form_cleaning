from __future__ import annotations

from typing import List, Tuple

from src.common.geometry import Point3, Pose6D
from src.common.trajectory import WorkTrajectory
from src.exporters.robot.orientation import RollPitchYaw, pose_to_elite_rpy
from src.exporters.robot.settings import RobotExportSettings

Pose6 = Tuple[float, float, float, float, float, float]


class RobotExportError(ValueError):
    pass


def _work_poses(trajectory: WorkTrajectory) -> List[Pose6D]:
    work = [pose for pose in trajectory.poses if pose.pose_type == "work"]
    if not work:
        work = [pose for pose in trajectory.poses if pose.pose_type not in ("start", "finish")]
    if not work:
        raise RobotExportError("В траектории нет рабочих точек для экспорта УП")
    return work


def _build_work_targets(
    work_poses: List[Pose6D],
    settings: RobotExportSettings,
) -> List[Tuple[Point3, RollPitchYaw]]:
    mount_rad = settings.tool_mount_rotation_rad
    targets: List[Tuple[Point3, RollPitchYaw]] = []
    for pose in work_poses:
        rpy = pose_to_elite_rpy(pose.tool_axis_z, pose.tool_axis_x, mount_rad)
        targets.append((pose.position, rpy))
    return targets


def _format_pose(name: str, position_m: Point3, rpy: RollPitchYaw) -> str:
    roll, pitch, yaw = rpy
    x, y, z = position_m
    return (
        f" pose_{name} = [{x:.4f}, {y:.4f}, {z:.4f}, {roll:.4f}, {pitch:.4f}, {yaw:.4f}]\n"
    )


def _format_movel(pose_name: str, settings: RobotExportSettings, blend_m: float) -> str:
    return (
        f" movel(pose_{pose_name}, a={settings.acceleration}, v={settings.velocity}, "
        f"t=0, r={blend_m:.4f})\n"
    )


def _mm_to_m(point: Point3) -> Point3:
    return (point[0] / 1000.0, point[1] / 1000.0, point[2] / 1000.0)


def build_elite_program(trajectory: WorkTrajectory, settings: RobotExportSettings) -> str:
    work_targets = _build_work_targets(_work_poses(trajectory), settings)

    first_rpy = work_targets[0][1]
    start_m = _mm_to_m(trajectory.start_point_mm)
    finish_m = _mm_to_m(trajectory.finish_point_mm)
    blend_m = settings.blend_radius_m

    lines: List[str] = ["def welding_program():\n"]
    lines.append(_format_pose("start", start_m, first_rpy))
    lines.append(_format_movel("start", settings, 0.0))
    lines.append("\n")

    for index, (position_mm, rpy) in enumerate(work_targets):
        position_m = _mm_to_m(position_mm)
        is_last = index == len(work_targets) - 1
        radius = 0.0 if is_last else blend_m
        name = f"work_{index}"
        lines.append(_format_pose(name, position_m, rpy))
        lines.append(_format_movel(name, settings, radius))

    last_rpy = work_targets[-1][1]
    lines.append("\n")
    lines.append(_format_pose("finish", finish_m, last_rpy))
    lines.append(_format_movel("finish", settings, 0.0))
    lines.append("end\n")

    metadata = f"# TRAJECTORY_LAYER: {trajectory.generator_id}\n"
    return metadata + "".join(lines)
