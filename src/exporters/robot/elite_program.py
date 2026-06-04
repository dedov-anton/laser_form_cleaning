from __future__ import annotations

from typing import List, Tuple

from src.common.geometry import Point3, Pose6D
from src.common.trajectory import WorkTrajectory
from src.exporters.robot.orientation import RollPitchYaw, pose_to_elite_rpy
from src.exporters.robot.ring_arc import (
    ArcPassPlan,
    RingArcGeometry,
    build_ccw_semicircle_poses,
    build_cw_semicircle_poses,
    geometry_with_track_radius,
    plan_arc_passes_from_trajectory,
    ring_arc_geometry_from_trajectory,
)
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


def _format_movec(
    via_pose_name: str,
    end_pose_name: str,
    settings: RobotExportSettings,
    blend_m: float = 0.0,
    *,
    mode: int = 0,
) -> str:
    """Elite movec(p_via, p_to, a, v, r, mode) — no time parameter t (CS Script Manual 2.1.2)."""
    return (
        f" movec(pose_{via_pose_name}, pose_{end_pose_name}, "
        f"a={settings.acceleration}, v={settings.velocity}, r={blend_m:.4f}, mode={mode})\n"
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


def build_ring_movec_test_program(
    trajectory: WorkTrajectory,
    settings: RobotExportSettings,
    *,
    geometry: RingArcGeometry | None = None,
) -> str:
    """
    Test program: movel to 180°, movec via 90° to 0° on mid-ring radius (CW semicircle).
    """
    arc_geometry = geometry or ring_arc_geometry_from_trajectory(trajectory)
    start_pose, via_pose, end_pose = build_cw_semicircle_poses(
        arc_geometry, start_deg=180.0, via_deg=90.0, end_deg=0.0
    )
    mount_rad = settings.tool_mount_rotation_rad
    start_rpy = pose_to_elite_rpy(
        start_pose.tool_axis_z, start_pose.tool_axis_x, mount_rad
    )
    via_rpy = pose_to_elite_rpy(via_pose.tool_axis_z, via_pose.tool_axis_x, mount_rad)
    end_rpy = pose_to_elite_rpy(end_pose.tool_axis_z, end_pose.tool_axis_x, mount_rad)

    cx, cy = arc_geometry.center_xy
    lines: List[str] = [
        f"# TRAJECTORY_LAYER: {arc_geometry.generator_id}\n",
        "# RING_ARC_TEST: CW semicircle 180 -> 90 -> 0 deg\n",
        f"# center_xy_mm: {cx:.4f}, {cy:.4f}\n",
        f"# track_radius_mm: {arc_geometry.track_radius_mm:.4f}\n",
        f"# z_mm: {arc_geometry.z_mm:.4f}\n",
        "def welding_program():\n",
    ]
    start_m = _mm_to_m(start_pose.position)
    via_m = _mm_to_m(via_pose.position)
    end_m = _mm_to_m(end_pose.position)
    lines.append(_format_pose("arc_start_180", start_m, start_rpy))
    lines.append(_format_movel("arc_start_180", settings, 0.0))
    lines.append("\n")
    lines.append(_format_pose("arc_via_90", via_m, via_rpy))
    lines.append(_format_pose("arc_end_0", end_m, end_rpy))
    lines.append(_format_movec("arc_via_90", "arc_end_0", settings, 0.0))
    lines.append("\n")
    lines.append(_format_pose("safe_return", start_m, start_rpy))
    lines.append(_format_movel("safe_return", settings, 0.0))
    lines.append("end\n")
    return "".join(lines)


def _append_pose_and_movel(
    lines: List[str],
    pose_name: str,
    pose: Pose6D,
    settings: RobotExportSettings,
    mount_rad: float,
    blend_m: float,
) -> None:
    position_m = _mm_to_m(pose.position)
    rpy = pose_to_elite_rpy(pose.tool_axis_z, pose.tool_axis_x, mount_rad)
    lines.append(_format_pose(pose_name, position_m, rpy))
    lines.append(_format_movel(pose_name, settings, blend_m))


def _append_cw_semicircle_movec(
    lines: List[str],
    geometry: RingArcGeometry,
    settings: RobotExportSettings,
    mount_rad: float,
    name_prefix: str,
) -> None:
    start_pose, via_pose, end_pose = build_cw_semicircle_poses(
        geometry, start_deg=180.0, via_deg=90.0, end_deg=0.0
    )
    _append_pose_and_movel(lines, f"{name_prefix}_cw_180", start_pose, settings, mount_rad, 0.0)
    lines.append("\n")
    via_m = _mm_to_m(via_pose.position)
    end_m = _mm_to_m(end_pose.position)
    via_rpy = pose_to_elite_rpy(via_pose.tool_axis_z, via_pose.tool_axis_x, mount_rad)
    end_rpy = pose_to_elite_rpy(end_pose.tool_axis_z, end_pose.tool_axis_x, mount_rad)
    lines.append(_format_pose(f"{name_prefix}_cw_via_90", via_m, via_rpy))
    lines.append(_format_pose(f"{name_prefix}_cw_end_0", end_m, end_rpy))
    lines.append(_format_movec(f"{name_prefix}_cw_via_90", f"{name_prefix}_cw_end_0", settings, 0.0))
    lines.append("\n")


def _append_ccw_semicircle_movec(
    lines: List[str],
    geometry: RingArcGeometry,
    settings: RobotExportSettings,
    mount_rad: float,
    name_prefix: str,
) -> None:
    start_pose, via_pose, end_pose = build_ccw_semicircle_poses(
        geometry, start_deg=180.0, via_deg=270.0, end_deg=0.0
    )
    _append_pose_and_movel(lines, f"{name_prefix}_ccw_180", start_pose, settings, mount_rad, 0.0)
    lines.append("\n")
    via_m = _mm_to_m(via_pose.position)
    end_m = _mm_to_m(end_pose.position)
    via_rpy = pose_to_elite_rpy(via_pose.tool_axis_z, via_pose.tool_axis_x, mount_rad)
    end_rpy = pose_to_elite_rpy(end_pose.tool_axis_z, end_pose.tool_axis_x, mount_rad)
    lines.append(_format_pose(f"{name_prefix}_ccw_via_270", via_m, via_rpy))
    lines.append(_format_pose(f"{name_prefix}_ccw_end_0", end_m, end_rpy))
    lines.append(
        _format_movec(f"{name_prefix}_ccw_via_270", f"{name_prefix}_ccw_end_0", settings, 0.0)
    )
    lines.append("\n")


def _ring_arc_program_endpoints(
    geometry: RingArcGeometry,
    pass_plan: ArcPassPlan,
) -> Tuple[Pose6D, Pose6D]:
    """First CW start (180°) and last CCW end (0°) on outermost track."""
    first_track = geometry_with_track_radius(geometry, pass_plan.radii_mm[0])
    last_track = geometry_with_track_radius(geometry, pass_plan.radii_mm[-1])
    program_start, _, _ = build_cw_semicircle_poses(
        first_track, start_deg=180.0, via_deg=90.0, end_deg=0.0
    )
    _, _, program_finish = build_ccw_semicircle_poses(
        last_track, start_deg=180.0, via_deg=270.0, end_deg=0.0
    )
    return program_start, program_finish


def build_ring_arc_program(
    settings: RobotExportSettings,
    *,
    geometry: RingArcGeometry,
    pass_plan: ArcPassPlan,
) -> str:
    """Full ring cleaning: for each track radius, CW semicircle then CCW semicircle (movec)."""
    mount_rad = settings.tool_mount_rotation_rad
    program_start, program_finish = _ring_arc_program_endpoints(geometry, pass_plan)
    start_rpy = pose_to_elite_rpy(
        program_start.tool_axis_z, program_start.tool_axis_x, mount_rad
    )
    finish_rpy = pose_to_elite_rpy(
        program_finish.tool_axis_z, program_finish.tool_axis_x, mount_rad
    )

    cx, cy = geometry.center_xy
    radii_text = ", ".join(f"{radius:.2f}" for radius in pass_plan.radii_mm)
    lines: List[str] = [
        f"# TRAJECTORY_LAYER: {geometry.generator_id}\n",
        f"# RING_ARC: {pass_plan.pass_count} pass(es), overlap_mm={pass_plan.overlap_mm:.4f}\n",
        f"# center_xy_mm: {cx:.4f}, {cy:.4f}\n",
        f"# track_radii_mm: {radii_text}\n",
        f"# z_mm: {geometry.z_mm:.4f}\n",
        "def welding_program():\n",
    ]

    lines.append(_format_pose("start", _mm_to_m(program_start.position), start_rpy))
    lines.append(_format_movel("start", settings, 0.0))
    lines.append("\n")

    for index, radius_mm in enumerate(pass_plan.radii_mm):
        track_geometry = geometry_with_track_radius(geometry, radius_mm)
        prefix = f"track_{index}"
        _append_cw_semicircle_movec(lines, track_geometry, settings, mount_rad, prefix)
        _append_ccw_semicircle_movec(lines, track_geometry, settings, mount_rad, prefix)

    lines.append(_format_pose("finish", _mm_to_m(program_finish.position), finish_rpy))
    lines.append(_format_movel("finish", settings, 0.0))
    lines.append("end\n")
    return "".join(lines)


def build_ring_arc_program_from_trajectory(
    trajectory: WorkTrajectory,
    settings: RobotExportSettings,
    *,
    geometry: RingArcGeometry | None = None,
    pass_plan: ArcPassPlan | None = None,
) -> str:
    """Build arc program from trajectory (tests)."""
    base_geometry = geometry or ring_arc_geometry_from_trajectory(trajectory)
    plan = pass_plan or plan_arc_passes_from_trajectory(trajectory)
    return build_ring_arc_program(
        settings,
        geometry=base_geometry,
        pass_plan=plan,
    )
