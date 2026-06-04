from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import List, Optional, Tuple

from src.common.frame_pose import (
    FramePose6D,
    rotation_matrix_sxyz,
    transform_point,
    transform_vector,
)
from src.common.geometry import DEFAULT_TOOL_AXIS_Z, Point3, Pose6D
from src.common.tool_orientation import orthogonal_tool_axis_x
from src.common.trajectory import WorkTrajectory

RingCenterXY = Tuple[float, float]


@dataclass(frozen=True)
class RingArcGeometry:
    """Ring arc parameters: local ring in fixture frame, world via frame_pose."""

    frame_pose: FramePose6D
    center_xy: RingCenterXY
    z_mm: float
    track_radius_mm: float
    inner_radius_mm: float
    ring_width_mm: float
    beam_width_mm: float
    generator_id: str
    local_z_mm: float = 0.0


TOP_RING_TOOL_AXIS_Z_LOCAL: Point3 = (0.0, 0.0, 1.0)


def polar_to_xy(radius_mm: float, angle_deg: float) -> Tuple[float, float]:
    angle_rad = math.radians(angle_deg)
    return radius_mm * math.cos(angle_rad), radius_mm * math.sin(angle_rad)


def _local_arc_point(geometry: RingArcGeometry, radius_mm: float, angle_deg: float) -> Point3:
    local_x, local_y = polar_to_xy(radius_mm, angle_deg)
    return (local_x, local_y, geometry.local_z_mm)


def _default_tool_axis_z_local(geometry: RingArcGeometry) -> Point3:
    if geometry.generator_id == "top_ring":
        return TOP_RING_TOOL_AXIS_Z_LOCAL
    return DEFAULT_TOOL_AXIS_Z


def _geometry_rotation(geometry: RingArcGeometry):
    pose = geometry.frame_pose
    return rotation_matrix_sxyz(pose.rx_deg, pose.ry_deg, pose.rz_deg)


def _world_point(geometry: RingArcGeometry, local_point: Point3) -> Point3:
    rotation = _geometry_rotation(geometry)
    return transform_point(local_point, rotation, geometry.frame_pose.translation())


def _world_vector(geometry: RingArcGeometry, local_vector: Point3) -> Point3:
    return transform_vector(local_vector, _geometry_rotation(geometry))


def tangent_cw_xy(angle_deg: float) -> Tuple[float, float]:
    """Unit tangent in XY for clockwise motion (angle decreasing 180° → 0°)."""
    angle_rad = math.radians(angle_deg)
    return math.sin(angle_rad), -math.cos(angle_rad)


def tangent_ccw_xy(angle_deg: float) -> Tuple[float, float]:
    """Unit tangent in XY for counter-clockwise motion (angle increasing)."""
    angle_rad = math.radians(angle_deg)
    return -math.sin(angle_rad), math.cos(angle_rad)


def build_ring_arc_geometry(
    pose: FramePose6D,
    inner_radius_mm: float,
    ring_width_mm: float,
    beam_width_mm: float,
    generator_id: str = "bottom_ring",
    *,
    local_z_mm: float = 0.0,
) -> RingArcGeometry:
    if inner_radius_mm <= 0.0 or ring_width_mm <= 0.0:
        raise ValueError("inner_radius_mm and ring_width_mm must be positive")

    rotation = rotation_matrix_sxyz(pose.rx_deg, pose.ry_deg, pose.rz_deg)
    origin_world = transform_point((0.0, 0.0, 0.0), rotation, pose.translation())
    ring_plane_world = transform_point(
        (0.0, 0.0, local_z_mm), rotation, pose.translation()
    )
    return RingArcGeometry(
        frame_pose=pose,
        center_xy=(origin_world[0], origin_world[1]),
        z_mm=ring_plane_world[2],
        track_radius_mm=track_radius_mm(inner_radius_mm, ring_width_mm),
        inner_radius_mm=inner_radius_mm,
        ring_width_mm=ring_width_mm,
        beam_width_mm=beam_width_mm,
        generator_id=generator_id,
        local_z_mm=local_z_mm,
    )


def _frame_pose_from_trajectory(trajectory: WorkTrajectory) -> FramePose6D:
    frame = trajectory.metadata.get("frame_pose_applied")
    if isinstance(frame, dict):
        return FramePose6D.from_dict(frame)
    return FramePose6D(
        x_mm=trajectory.start_point_mm[0],
        y_mm=trajectory.start_point_mm[1],
        z_mm=trajectory.start_point_mm[2],
    )


def _params_from_trajectory(trajectory: WorkTrajectory) -> dict:
    snapshot = trajectory.params_snapshot or {}
    metadata = trajectory.metadata
    inner = float(snapshot.get("inner_radius_mm", metadata.get("inner_radius_mm", 0.0)))
    ring_width = float(snapshot.get("ring_width_mm", 0.0))
    if ring_width <= 0.0:
        outer = float(metadata.get("outer_radius_mm", inner))
        ring_width = outer - inner
    beam = float(snapshot.get("beam_width_mm", metadata.get("beam_width_mm", 0.0)))
    z_to_top = float(snapshot.get("z_to_top_mm", metadata.get("z_to_top_mm", 0.0)))
    return {
        "inner_radius_mm": inner,
        "ring_width_mm": ring_width,
        "beam_width_mm": beam,
        "z_to_top_mm": z_to_top,
    }


def track_radius_mm(inner_radius_mm: float, ring_width_mm: float) -> float:
    return inner_radius_mm + ring_width_mm / 2.0


@dataclass(frozen=True)
class ArcPassPlan:
    pass_count: int
    radii_mm: List[float]
    overlap_mm: float
    single_pass: bool


def plan_arc_passes(
    inner_radius_mm: float,
    ring_width_mm: float,
    beam_width_mm: float,
) -> ArcPassPlan:
    if inner_radius_mm <= 0.0 or ring_width_mm <= 0.0:
        raise ValueError("inner_radius_mm and ring_width_mm must be positive")
    if beam_width_mm <= 0.0:
        raise ValueError("beam_width_mm must be positive")

    radii = arc_track_radii_mm(inner_radius_mm, ring_width_mm, beam_width_mm)
    single_pass = len(radii) == 1
    if single_pass:
        overlap_mm = 0.0
    else:
        outer = inner_radius_mm + ring_width_mm
        start = inner_radius_mm + beam_width_mm / 2.0
        end = outer - beam_width_mm / 2.0
        pitch = (end - start) / (len(radii) - 1)
        overlap_mm = beam_width_mm - pitch
    return ArcPassPlan(
        pass_count=len(radii),
        radii_mm=radii,
        overlap_mm=overlap_mm,
        single_pass=single_pass,
    )


def plan_arc_passes_from_trajectory(trajectory: WorkTrajectory) -> ArcPassPlan:
    params = _params_from_trajectory(trajectory)
    return plan_arc_passes(
        params["inner_radius_mm"],
        params["ring_width_mm"],
        params["beam_width_mm"],
    )


def arc_pass_summary_text(plan: ArcPassPlan, beam_width_mm: float) -> str:
    from src.generators.bottom_ring.calc import overlap_percent

    if plan.single_pass:
        return "Дуги: 1 проход — ширина кольца достаточна для одного трека"
    percent = overlap_percent(plan.overlap_mm, beam_width_mm)
    return (
        f"Дуги: {plan.pass_count} прохода — перекрытие "
        f"{plan.overlap_mm:.2f} мм ({percent:.1f}%)"
    )


def geometry_with_track_radius(
    geometry: RingArcGeometry,
    track_radius_mm: float,
) -> RingArcGeometry:
    return replace(geometry, track_radius_mm=track_radius_mm)


def arc_track_radii_mm(
    inner_radius_mm: float,
    ring_width_mm: float,
    beam_width_mm: float,
) -> List[float]:
    """
    Concentric arc radii for ring_width > beam_width (phase 2).

    Single track at ring midline when ring fits one beam; otherwise evenly spaced
    tracks from inner+beam/2 to outer-beam/2.
    """
    if ring_width_mm <= beam_width_mm or beam_width_mm <= 0.0:
        return [track_radius_mm(inner_radius_mm, ring_width_mm)]
    outer = inner_radius_mm + ring_width_mm
    count = max(2, math.floor(ring_width_mm / beam_width_mm) + 1)
    start = inner_radius_mm + beam_width_mm / 2.0
    end = outer - beam_width_mm / 2.0
    if count == 1:
        return [track_radius_mm(inner_radius_mm, ring_width_mm)]
    step = (end - start) / (count - 1)
    return [start + index * step for index in range(count)]


def ring_arc_geometry_from_trajectory(trajectory: WorkTrajectory) -> RingArcGeometry:
    if trajectory.generator_id not in ("bottom_ring", "top_ring"):
        raise ValueError(
            f"Ring arc export expects bottom_ring or top_ring, got {trajectory.generator_id!r}"
        )
    params = _params_from_trajectory(trajectory)
    local_z_mm = (
        params["z_to_top_mm"] if trajectory.generator_id == "top_ring" else 0.0
    )
    return build_ring_arc_geometry(
        _frame_pose_from_trajectory(trajectory),
        params["inner_radius_mm"],
        params["ring_width_mm"],
        params["beam_width_mm"],
        generator_id=trajectory.generator_id,
        local_z_mm=local_z_mm,
    )


def build_arc_pose(
    geometry: RingArcGeometry,
    angle_deg: float,
    *,
    clockwise: bool,
    tool_axis_z: Optional[Point3] = None,
) -> Pose6D:
    local_position = _local_arc_point(geometry, geometry.track_radius_mm, angle_deg)
    position = _world_point(geometry, local_position)
    tangent_xy = tangent_cw_xy(angle_deg) if clockwise else tangent_ccw_xy(angle_deg)
    local_tangent: Point3 = (tangent_xy[0], tangent_xy[1], 0.0)
    world_tangent = _world_vector(geometry, local_tangent)
    axis_z = (
        tool_axis_z
        if tool_axis_z is not None
        else _world_vector(geometry, _default_tool_axis_z_local(geometry))
    )
    tool_axis_x = orthogonal_tool_axis_x(world_tangent, axis_z)
    return Pose6D(
        index=0,
        pose_type="arc",
        position=position,
        tool_axis_z=axis_z,
        tool_axis_x=tool_axis_x,
    )


def build_cw_semicircle_poses(
    geometry: RingArcGeometry,
    start_deg: float,
    via_deg: float,
    end_deg: float,
) -> Tuple[Pose6D, Pose6D, Pose6D]:
    """Start, via, end poses for one clockwise semicircle (angle decreasing)."""
    start = build_arc_pose(geometry, start_deg, clockwise=True)
    via = build_arc_pose(geometry, via_deg, clockwise=True)
    end = build_arc_pose(geometry, end_deg, clockwise=True)
    return start, via, end


def build_ccw_semicircle_poses(
    geometry: RingArcGeometry,
    start_deg: float,
    via_deg: float,
    end_deg: float,
) -> Tuple[Pose6D, Pose6D, Pose6D]:
    """Start, via, end poses for one counter-clockwise semicircle (angle increasing)."""
    start = build_arc_pose(geometry, start_deg, clockwise=False)
    via = build_arc_pose(geometry, via_deg, clockwise=False)
    end = build_arc_pose(geometry, end_deg, clockwise=False)
    return start, via, end
