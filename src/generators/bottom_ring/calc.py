from __future__ import annotations

import math

from typing import List, Optional, Tuple

from src.common.geometry import DEFAULT_TOOL_AXIS_Z, Point3, Pose6D, TravelSegment
from src.generators.bottom_ring.models import PassStop, RadialPass, RingTrajectory
from src.generators.bottom_ring.params import BottomRingParams, ProfileWaypoint

def recommended_sector_count(outer_radius_mm: float, beam_width_mm: float) -> int:

    if outer_radius_mm <= 0 or beam_width_mm <= 0:

        raise ValueError("outer_radius_mm and beam_width_mm must be positive")

    return max(1, math.floor(2 * math.pi * outer_radius_mm / beam_width_mm) + 1)

def overlap_mm(radius_mm: float, beam_width_mm: float, sector_count: int) -> float:

    if sector_count < 1:

        raise ValueError("sector_count must be >= 1")

    pitch = 2 * math.pi * radius_mm / sector_count

    return beam_width_mm - pitch

def overlap_percent(overlap_mm_value: float, beam_width_mm: float) -> float:

    if beam_width_mm <= 0:

        return 0.0

    return 100.0 * overlap_mm_value / beam_width_mm

def polar_to_xyz(radius_mm: float, angle_deg: float, z_mm: float = 0.0) -> Point3:

    angle_rad = math.radians(angle_deg)

    return (

        radius_mm * math.cos(angle_rad),

        radius_mm * math.sin(angle_rad),

        z_mm,

    )

def normalize_angle_deg(angle_deg: float) -> float:

    return angle_deg % 360.0

def angle_in_sector(angle_deg: float, start_deg: float, end_deg: float) -> bool:

    angle = normalize_angle_deg(angle_deg)

    start = normalize_angle_deg(start_deg)

    end = normalize_angle_deg(end_deg)

    if math.isclose(start, end, abs_tol=1e-9):

        return False

    if start < end:

        return start <= angle <= end

    return angle >= start or angle <= end

def ccw_distance(from_deg: float, to_deg: float) -> float:

    return (normalize_angle_deg(to_deg) - normalize_angle_deg(from_deg)) % 360.0

def angle_distance_to_positive_x(angle_deg: float) -> float:

    normalized = normalize_angle_deg(angle_deg)

    return min(normalized, 360.0 - normalized)

def active_profile_waypoints(params: BottomRingParams) -> List[ProfileWaypoint]:

    waypoints = [

        params.profile_point_1,

        params.profile_point_2,

    ]

    return [waypoint for waypoint in waypoints if waypoint.distance_from_outer_mm > 0.0]

def build_pass_stops(

    angle_deg: float,

    params: BottomRingParams,

    outer_to_inner: bool,

) -> List[PassStop]:

    inner_radius = params.inner_radius_mm

    outer_radius = params.outer_radius_mm

    inner_tilt = params.inner_tilt_deg

    outer_tilt = params.outer_tilt_deg

    profile_waypoints = sorted(

        active_profile_waypoints(params),

        key=lambda waypoint: waypoint.distance_from_outer_mm,

        reverse=True,

    )

    stops: List[PassStop] = [

        PassStop(

            position=polar_to_xyz(inner_radius, angle_deg, 0.0),

            pose_type="work_inner",

            tilt_deg=inner_tilt,

        )

    ]

    for index, waypoint in enumerate(profile_waypoints, start=1):

        radius_mm = outer_radius - waypoint.distance_from_outer_mm

        if radius_mm <= inner_radius:

            raise ValueError(

                f"Промеж. точка {index}: расстояние от внеш. радиуса слишком большое "

                f"({waypoint.distance_from_outer_mm:.2f} мм ≥ ширина кольца {params.ring_width_mm:.2f} мм)"

            )

        stops.append(

            PassStop(

                position=polar_to_xyz(radius_mm, angle_deg, waypoint.z_offset_mm),

                pose_type=f"work_profile_{index}",

                tilt_deg=waypoint.tilt_deg,

            )

        )

    stops.append(

        PassStop(

            position=polar_to_xyz(outer_radius, angle_deg, 0.0),

            pose_type="work_outer",

            tilt_deg=outer_tilt,

        )

    )

    if outer_to_inner:

        return list(reversed(stops))

    return stops

def build_radial_pass(

    index: int,

    angle_deg: float,

    params: BottomRingParams,

    outer_to_inner: bool,

) -> RadialPass:

    stops = build_pass_stops(angle_deg, params, outer_to_inner)

    return RadialPass(

        index=index,

        angle_deg=angle_deg,

        stops=stops,

        outer_to_inner=outer_to_inner,

    )

def order_processing_passes(

    angles_deg: List[float],

    params: BottomRingParams,

    entry_start_deg: Optional[float],

    entry_end_deg: Optional[float],

) -> List[RadialPass]:

    if entry_start_deg is None or entry_end_deg is None:

        start_index = min(

            range(len(angles_deg)),

            key=lambda index: angle_distance_to_positive_x(angles_deg[index]),

        )

    else:

        in_sector = [

            index

            for index, angle_deg in enumerate(angles_deg)

            if angle_in_sector(angle_deg, entry_start_deg, entry_end_deg)

        ]

        if not in_sector:

            raise ValueError(

                "В сектор O→I не попадает ни один луч сетки — измените границы или N секторов"

            )

        start_index = min(

            in_sector, key=lambda index: ccw_distance(entry_start_deg, angles_deg[index])

        )

    rotated = angles_deg[start_index:] + angles_deg[:start_index]

    has_entry = entry_start_deg is not None and entry_end_deg is not None

    return [

        build_radial_pass(

            index,

            angle_deg,

            params,

            outer_to_inner=(

                has_entry

                and angle_in_sector(angle_deg, entry_start_deg, entry_end_deg)

            ),

        )

        for index, angle_deg in enumerate(rotated)

    ]

def build_travel_segments(

    ordered_passes: List[RadialPass],

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

def radial_direction_xy(position: Point3) -> Point3:

    x, y, _ = position

    length = math.hypot(x, y)

    if length < 1e-9:

        return (1.0, 0.0, 0.0)

    return (x / length, y / length, 0.0)

def pass_direction_xy(start: Point3, finish: Point3) -> Point3:

    dx = finish[0] - start[0]

    dy = finish[1] - start[1]

    length = math.hypot(dx, dy)

    if length < 1e-9:

        return radial_direction_xy(start)

    return (dx / length, dy / length, 0.0)

def _normalize_vector(vector: Point3) -> Point3:

    length = math.hypot(vector[0], vector[1], vector[2])

    if length < 1e-9:

        return vector

    return (vector[0] / length, vector[1] / length, vector[2] / length)

def _cross(a: Point3, b: Point3) -> Point3:

    return (

        a[1] * b[2] - a[2] * b[1],

        a[2] * b[0] - a[0] * b[2],

        a[0] * b[1] - a[1] * b[0],

    )

def _dot(a: Point3, b: Point3) -> float:

    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def _rotate_vector(vector: Point3, axis: Point3, angle_deg: float) -> Point3:

    if abs(angle_deg) < 1e-9:

        return vector

    unit_axis = _normalize_vector(axis)

    angle_rad = math.radians(angle_deg)

    cos_a = math.cos(angle_rad)

    sin_a = math.sin(angle_rad)

    cross = _cross(unit_axis, vector)

    dot = _dot(unit_axis, vector)

    return (

        vector[0] * cos_a + cross[0] * sin_a + unit_axis[0] * dot * (1.0 - cos_a),

        vector[1] * cos_a + cross[1] * sin_a + unit_axis[1] * dot * (1.0 - cos_a),

        vector[2] * cos_a + cross[2] * sin_a + unit_axis[2] * dot * (1.0 - cos_a),

    )

def tilt_tool_axis_z(pass_dir_xy: Point3, tilt_deg: float) -> Point3:

    if abs(tilt_deg) < 1e-9:

        return DEFAULT_TOOL_AXIS_Z

    dx, dy, _ = pass_dir_xy

    axis_len = math.hypot(dx, dy)

    if axis_len < 1e-9:

        return DEFAULT_TOOL_AXIS_Z

    rot_axis = (-dy / axis_len, dx / axis_len, 0.0)

    return _normalize_vector(_rotate_vector(DEFAULT_TOOL_AXIS_Z, rot_axis, -tilt_deg))

def orthogonal_tool_axis_x(preferred_xy: Point3, tool_axis_z: Point3) -> Point3:

    preferred = (preferred_xy[0], preferred_xy[1], 0.0)

    projection = _dot(preferred, tool_axis_z)

    projected = (

        preferred[0] - projection * tool_axis_z[0],

        preferred[1] - projection * tool_axis_z[1],

        preferred[2] - projection * tool_axis_z[2],

    )

    length = math.hypot(projected[0], projected[1], projected[2])

    if length < 1e-9:

        return radial_direction_xy(preferred)

    return (projected[0] / length, projected[1] / length, projected[2] / length)

def build_work_pose(

    index: int,

    pose_type: str,

    position: Point3,

    pass_dir_xy: Point3,

    tilt_deg: float,

) -> Pose6D:

    tool_axis_z = tilt_tool_axis_z(pass_dir_xy, tilt_deg)

    tool_axis_x = orthogonal_tool_axis_x(pass_dir_xy, tool_axis_z)

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

    tool_axis_x: Optional[Point3] = None,

) -> Pose6D:

    return Pose6D(

        index=index,

        pose_type=pose_type,

        position=position,

        tool_axis_z=tool_axis_z,

        tool_axis_x=tool_axis_x if tool_axis_x is not None else radial_direction_xy(position),

    )

def build_poses(

    start_point_mm: Point3,

    finish_point_mm: Point3,

    ordered_passes: List[RadialPass],

) -> List[Pose6D]:

    poses: List[Pose6D] = [

        build_pose(0, "start", start_point_mm, DEFAULT_TOOL_AXIS_Z)

    ]

    for radial_pass in ordered_passes:

        pass_axis = pass_direction_xy(radial_pass.start, radial_pass.finish)

        for stop in radial_pass.stops:

            tilt_deg = -stop.tilt_deg if radial_pass.outer_to_inner else stop.tilt_deg

            poses.append(

                build_work_pose(

                    len(poses),

                    stop.pose_type,

                    stop.position,

                    pass_axis,

                    tilt_deg,

                )

            )

    poses.append(build_pose(len(poses), "finish", finish_point_mm, DEFAULT_TOOL_AXIS_Z))

    return poses

def _validate_tilt(label: str, value: float) -> None:

    if not math.isfinite(value):

        raise ValueError(f"{label}: некорректное значение")

    if value <= -90.0 or value >= 90.0:

        raise ValueError(f"{label}: угол должен быть в диапазоне -89…89")

def build_trajectory(

    params: BottomRingParams,

    start_point_mm: Point3 = (0.0, 0.0, 0.0),

    finish_point_mm: Point3 = (0.0, 0.0, 0.0),

    start_angle_deg: float = 0.0,

) -> RingTrajectory:

    if params.inner_radius_mm <= 0:

        raise ValueError("inner_radius_mm must be positive")

    if params.ring_width_mm <= 0:

        raise ValueError("ring_width_mm must be positive")

    if params.beam_width_mm <= 0:

        raise ValueError("beam_width_mm must be positive")

    if params.sector_count < 1:

        raise ValueError("sector_count must be >= 1")

    if (params.entry_sector_start_deg is None) ^ (params.entry_sector_end_deg is None):

        raise ValueError("Укажите начало и конец сектора O→I, или оставьте оба поля пустыми")

    if params.has_entry_sector:

        for label, value in (

            ("Начало сектора O→I", params.entry_sector_start_deg),

            ("Конец сектора O→I", params.entry_sector_end_deg),

        ):

            if value is None or not math.isfinite(value):

                raise ValueError(f"{label}: некорректное значение")

            if value < 0.0 or value >= 360.0:

                raise ValueError(f"{label}: угол должен быть в диапазоне 0–360")

    _validate_tilt("Наклон Z на внутр. радиусе", params.inner_tilt_deg)

    _validate_tilt("Наклон Z на внеш. радиусе", params.outer_tilt_deg)

    for index, waypoint in enumerate(

        (params.profile_point_1, params.profile_point_2), start=1

    ):

        _validate_tilt(f"Наклон промеж. точки {index}", waypoint.tilt_deg)

        if waypoint.distance_from_outer_mm > 0.0:

            if waypoint.distance_from_outer_mm >= params.ring_width_mm:

                raise ValueError(

                    f"Промеж. точка {index}: расстояние от внеш. радиуса "

                    f"должно быть меньше ширины кольца ({params.ring_width_mm:.2f} мм)"

                )

            if not math.isfinite(waypoint.z_offset_mm):

                raise ValueError(f"Промеж. точка {index}: некорректное значение Z")

    outer_radius = params.outer_radius_mm

    angle_step = 360.0 / params.sector_count

    angles_deg = [start_angle_deg + index * angle_step for index in range(params.sector_count)]

    ordered_passes = order_processing_passes(

        angles_deg,

        params,

        params.entry_sector_start_deg,

        params.entry_sector_end_deg,

    )

    travel_segments = build_travel_segments(ordered_passes, start_point_mm, finish_point_mm)

    poses = build_poses(start_point_mm, finish_point_mm, ordered_passes)

    overlap_outer = overlap_mm(outer_radius, params.beam_width_mm, params.sector_count)

    overlap_inner = overlap_mm(params.inner_radius_mm, params.beam_width_mm, params.sector_count)

    return RingTrajectory(

        inner_radius_mm=params.inner_radius_mm,

        outer_radius_mm=outer_radius,

        beam_width_mm=params.beam_width_mm,

        sector_count=params.sector_count,

        overlap_outer_mm=overlap_outer,

        overlap_inner_mm=overlap_inner,

        start_point_mm=start_point_mm,

        finish_point_mm=finish_point_mm,

        entry_sector_start_deg=params.entry_sector_start_deg,

        entry_sector_end_deg=params.entry_sector_end_deg,

        passes=ordered_passes,

        travel_segments=travel_segments,

        poses=poses,

    )

