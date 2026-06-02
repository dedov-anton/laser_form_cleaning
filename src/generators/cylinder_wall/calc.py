from __future__ import annotations

import math
from typing import List

from src.common.geometry import DEFAULT_TOOL_AXIS_Z, Point3, Pose6D, TravelSegment
from src.common.tool_orientation import normalize_vector, orthogonal_tool_axis_x, tilt_tool_axis_radial
from src.generators.cylinder_wall.models import CylinderTrajectory, PassStop, VerticalPass
from src.generators.cylinder_wall.params import (
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    CylinderWallParams,
    WallProfileWaypoint,
)

VERTICAL_DOWN: Point3 = (0.0, 0.0, -1.0)


def sector_arc_mm(inner_radius_mm: float, sector_count: int = SECTOR_COUNT) -> float:
    return 2 * math.pi * inner_radius_mm / sector_count


def recommended_passes_per_sector(
    inner_radius_mm: float,
    beam_width_mm: float,
    sector_count: int = SECTOR_COUNT,
) -> int:
    if inner_radius_mm <= 0 or beam_width_mm <= 0:
        raise ValueError("inner_radius_mm and beam_width_mm must be positive")
    arc_mm = sector_arc_mm(inner_radius_mm, sector_count)
    return max(1, math.floor(arc_mm / beam_width_mm) + 1)


def overlap_per_sector(
    inner_radius_mm: float,
    beam_width_mm: float,
    passes_per_sector: int,
    sector_count: int = SECTOR_COUNT,
) -> float:
    if passes_per_sector < 1:
        raise ValueError("passes_per_sector must be >= 1")
    arc_mm = sector_arc_mm(inner_radius_mm, sector_count)
    pitch = arc_mm / passes_per_sector
    return beam_width_mm - pitch


def overlap_percent(overlap_mm_value: float, beam_width_mm: float) -> float:
    if beam_width_mm <= 0:
        return 0.0
    return 100.0 * overlap_mm_value / beam_width_mm


def polar_to_xyz(radius_mm: float, angle_deg: float, z_mm: float) -> Point3:
    angle_rad = math.radians(angle_deg)
    return (
        radius_mm * math.cos(angle_rad),
        radius_mm * math.sin(angle_rad),
        z_mm,
    )


def sector_pass_angles(sector_index: int, passes_per_sector: int) -> List[float]:
    start_deg = sector_index * SECTOR_WIDTH_DEG
    step = SECTOR_WIDTH_DEG / passes_per_sector
    return [start_deg + (index + 0.5) * step for index in range(passes_per_sector)]


def ordered_sector_indices(start_sector: int, clockwise: bool) -> List[int]:
    if start_sector < 0 or start_sector >= SECTOR_COUNT:
        raise ValueError(f"start_sector must be in 0…{SECTOR_COUNT - 1}")
    if clockwise:
        return [(start_sector - offset) % SECTOR_COUNT for offset in range(SECTOR_COUNT)]
    return [(start_sector + offset) % SECTOR_COUNT for offset in range(SECTOR_COUNT)]


def active_profile_waypoints(params: CylinderWallParams) -> List[WallProfileWaypoint]:
    waypoints = [params.profile_point_1, params.profile_point_2]
    return [waypoint for waypoint in waypoints if waypoint.z_down_mm > 0.0]


def build_pass_stops(angle_deg: float, params: CylinderWallParams) -> List[PassStop]:
    z_bottom = params.z_top_mm - params.wall_height_mm
    profile_waypoints = sorted(active_profile_waypoints(params), key=lambda w: w.z_down_mm)

    stops: List[PassStop] = [
        PassStop(
            position=polar_to_xyz(params.inner_radius_mm, angle_deg, params.z_top_mm),
            pose_type="top",
            tilt_deg=params.top_tilt_deg,
        )
    ]

    for index, waypoint in enumerate(profile_waypoints, start=1):
        radius_mm = params.inner_radius_mm - waypoint.radial_inward_mm
        if radius_mm <= 0.0:
            raise ValueError(
                f"Промеж. точка {index}: смещение к оси слишком большое "
                f"({waypoint.radial_inward_mm:.2f} мм, радиус ≤ 0)"
            )
        z_mm = params.z_top_mm - waypoint.z_down_mm
        stops.append(
            PassStop(
                position=polar_to_xyz(radius_mm, angle_deg, z_mm),
                pose_type=f"work_profile_{index}",
                tilt_deg=waypoint.tilt_deg,
            )
        )

    stops.append(
        PassStop(
            position=polar_to_xyz(params.inner_radius_mm, angle_deg, z_bottom),
            pose_type="bottom",
            tilt_deg=params.bottom_tilt_deg,
        )
    )
    return stops


def build_vertical_pass(
    index: int,
    sector_index: int,
    angle_deg: float,
    params: CylinderWallParams,
) -> VerticalPass:
    return VerticalPass(
        index=index,
        sector_index=sector_index,
        angle_deg=angle_deg,
        stops=build_pass_stops(angle_deg, params),
    )


def order_processing_passes(params: CylinderWallParams) -> List[VerticalPass]:
    passes: List[VerticalPass] = []
    pass_index = 0
    for sector_index in ordered_sector_indices(params.start_sector, params.clockwise):
        for angle_deg in sector_pass_angles(sector_index, params.passes_per_sector):
            passes.append(build_vertical_pass(pass_index, sector_index, angle_deg, params))
            pass_index += 1
    return passes


def build_travel_segments(
    ordered_passes: List[VerticalPass],
    start_point_mm: Point3,
    finish_point_mm: Point3,
) -> List[TravelSegment]:
    if not ordered_passes:
        return []

    segments: List[TravelSegment] = [
        TravelSegment(
            index=0,
            segment_type="approach",
            start=start_point_mm,
            finish=ordered_passes[0].start,
        )
    ]

    for pass_index in range(len(ordered_passes) - 1):
        segments.append(
            TravelSegment(
                index=len(segments),
                segment_type="transfer",
                start=ordered_passes[pass_index].finish,
                finish=ordered_passes[pass_index + 1].start,
            )
        )

    segments.append(
        TravelSegment(
            index=len(segments),
            segment_type="departure",
            start=ordered_passes[-1].finish,
            finish=finish_point_mm,
        )
    )
    return segments


def _subtract(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def stop_path_tangent(stops: List[PassStop], index: int) -> Point3:
    """Path tangent at stop index (same logic as welding_path_generator_bk curr_tangent)."""
    if index == 0:
        direction = _subtract(stops[1].position, stops[0].position)
    elif index == len(stops) - 1:
        direction = _subtract(stops[-1].position, stops[-2].position)
    else:
        prev = normalize_vector(_subtract(stops[index].position, stops[index - 1].position))
        next_seg = normalize_vector(_subtract(stops[index + 1].position, stops[index].position))
        direction = _add(prev, next_seg)
    return normalize_vector(direction)


def build_cylinder_work_pose(
    index: int,
    pose_type: str,
    position: Point3,
    tilt_deg: float,
    path_tangent: Point3,
) -> Pose6D:
    tool_axis_z = tilt_tool_axis_radial(position, tilt_deg)
    tool_axis_x = orthogonal_tool_axis_x(path_tangent, tool_axis_z)
    return Pose6D(
        index=index,
        pose_type=pose_type,
        position=position,
        tool_axis_z=tool_axis_z,
        tool_axis_x=tool_axis_x,
    )


def build_pose(
    index: int,
    pose_type: str,
    position: Point3,
    tool_axis_z: Point3 = DEFAULT_TOOL_AXIS_Z,
) -> Pose6D:
    tool_axis_x = orthogonal_tool_axis_x(VERTICAL_DOWN, tool_axis_z)
    return Pose6D(
        index=index,
        pose_type=pose_type,
        position=position,
        tool_axis_z=tool_axis_z,
        tool_axis_x=tool_axis_x,
    )


def build_poses(
    start_point_mm: Point3,
    finish_point_mm: Point3,
    ordered_passes: List[VerticalPass],
) -> List[Pose6D]:
    poses: List[Pose6D] = [build_pose(0, "start", start_point_mm)]

    for vertical_pass in ordered_passes:
        stops = vertical_pass.stops
        for stop_index, stop in enumerate(stops):
            path_tangent = stop_path_tangent(stops, stop_index)
            poses.append(
                build_cylinder_work_pose(
                    len(poses),
                    stop.pose_type,
                    stop.position,
                    stop.tilt_deg,
                    path_tangent,
                )
            )

    poses.append(build_pose(len(poses), "finish", finish_point_mm))
    return poses


def build_sector_boundary_polylines(params: CylinderWallParams) -> List[dict]:
    z_bottom = params.z_top_mm - params.wall_height_mm
    polylines: List[dict] = []
    for boundary_index in range(SECTOR_COUNT):
        angle_deg = boundary_index * SECTOR_WIDTH_DEG
        start = polar_to_xyz(params.inner_radius_mm, angle_deg, params.z_top_mm)
        finish = polar_to_xyz(params.inner_radius_mm, angle_deg, z_bottom)
        polylines.append(
            {
                "start": start,
                "finish": finish,
                "group": f"sector_boundary_{boundary_index}",
            }
        )
    return polylines


def _validate_tilt(label: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{label}: некорректное значение")
    if value <= -90.0 or value >= 90.0:
        raise ValueError(f"{label}: угол должен быть в диапазоне -89…89")


def build_trajectory(
    params: CylinderWallParams,
    start_point_mm: Point3 = (0.0, 0.0, 0.0),
    finish_point_mm: Point3 = (0.0, 0.0, 0.0),
) -> CylinderTrajectory:
    if params.inner_radius_mm <= 0:
        raise ValueError("inner_radius_mm must be positive")
    if params.wall_height_mm <= 0:
        raise ValueError("wall_height_mm must be positive")
    if not math.isfinite(params.z_top_mm):
        raise ValueError("z_top_mm: некорректное значение")
    if params.beam_width_mm <= 0:
        raise ValueError("beam_width_mm must be positive")
    if params.passes_per_sector < 1:
        raise ValueError("passes_per_sector must be >= 1")
    if params.start_sector < 0 or params.start_sector >= SECTOR_COUNT:
        raise ValueError(f"start_sector must be in 0…{SECTOR_COUNT - 1}")

    _validate_tilt("Наклон верхней точки", params.top_tilt_deg)
    _validate_tilt("Наклон нижней точки", params.bottom_tilt_deg)

    for index, waypoint in enumerate((params.profile_point_1, params.profile_point_2), start=1):
        _validate_tilt(f"Наклон промеж. точки {index}", waypoint.tilt_deg)
        if waypoint.z_down_mm > 0.0:
            if waypoint.z_down_mm >= params.wall_height_mm:
                raise ValueError(
                    f"Промеж. точка {index}: смещение вниз должно быть меньше "
                    f"высоты стенки ({params.wall_height_mm:.2f} мм)"
                )
            if not math.isfinite(waypoint.radial_inward_mm):
                raise ValueError(f"Промеж. точка {index}: некорректное смещение к оси")
            if waypoint.radial_inward_mm >= params.inner_radius_mm:
                raise ValueError(
                    f"Промеж. точка {index}: смещение к оси должно быть меньше "
                    f"внутр. радиуса ({params.inner_radius_mm:.2f} мм)"
                )

    ordered_passes = order_processing_passes(params)
    travel_segments = build_travel_segments(ordered_passes, start_point_mm, finish_point_mm)
    poses = build_poses(start_point_mm, finish_point_mm, ordered_passes)
    overlap_arc = overlap_per_sector(
        params.inner_radius_mm,
        params.beam_width_mm,
        params.passes_per_sector,
    )

    return CylinderTrajectory(
        inner_radius_mm=params.inner_radius_mm,
        wall_height_mm=params.wall_height_mm,
        z_top_mm=params.z_top_mm,
        beam_width_mm=params.beam_width_mm,
        passes_per_sector=params.passes_per_sector,
        overlap_arc_mm=overlap_arc,
        start_sector=params.start_sector,
        clockwise=params.clockwise,
        start_point_mm=start_point_mm,
        finish_point_mm=finish_point_mm,
        passes=ordered_passes,
        travel_segments=travel_segments,
        poses=poses,
    )
