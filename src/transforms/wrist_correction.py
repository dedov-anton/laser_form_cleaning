from __future__ import annotations

import math
from dataclasses import replace
from typing import Optional, Tuple

from src.common.geometry import Point3, Pose6D
from src.common.tool_orientation import orthogonal_tool_axis_x
from src.common.trajectory import WorkTrajectory

RollPitchYaw = Tuple[float, float, float]

VERTICAL_UPWARD: Point3 = (0.0, 0.0, 1.0)

WRIST_CORRECTION_SOURCE_RPY: RollPitchYaw = (math.pi / 2, 0.0, -math.pi / 2)
WRIST_CORRECTION_TARGET_RPY: RollPitchYaw = (math.pi, 0.0, math.pi / 2)


def _normalize(vector: Point3) -> Point3:
    length = math.hypot(vector[0], vector[1], vector[2])
    if length < 1e-9:
        raise ValueError("Cannot normalize zero vector")
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Point3, b: Point3) -> Point3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def rotate_vector_between_directions(vector: Point3, from_dir: Point3, to_dir: Point3) -> Point3:
    """Rotate vector from one direction alignment to another (Rodrigues, like welding_path_generator_bk)."""
    source = _normalize(from_dir)
    target = _normalize(to_dir)
    dot_product = _dot(source, target)

    if dot_product > 0.9999:
        return vector
    if dot_product < -0.9999:
        if abs(source[0]) < 0.9:
            axis = (1.0, 0.0, 0.0)
        else:
            axis = (0.0, 1.0, 0.0)
        axis = _normalize(_cross(source, axis))
        angle = math.pi
    else:
        axis = _normalize(_cross(source, target))
        angle = math.acos(max(-1.0, min(1.0, dot_product)))

    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    cross_product = _cross(axis, vector)
    axis_dot = _dot(axis, vector)
    return (
        vector[0] * cos_a + cross_product[0] * sin_a + axis[0] * axis_dot * (1.0 - cos_a),
        vector[1] * cos_a + cross_product[1] * sin_a + axis[1] * axis_dot * (1.0 - cos_a),
        vector[2] * cos_a + cross_product[2] * sin_a + axis[2] * axis_dot * (1.0 - cos_a),
    )


def cylinder_axis_xy(trajectory: WorkTrajectory) -> Tuple[float, float]:
    """Cylinder axis in XY: world fixture translation or local origin."""
    if trajectory.metadata.get("coordinate_frame") == "world":
        frame = trajectory.metadata.get("frame_pose_applied")
        if isinstance(frame, dict):
            return float(frame.get("x_mm", 0.0)), float(frame.get("y_mm", 0.0))
    return 0.0, 0.0


def inward_direction_to_axis(position: Point3, axis_xy: Tuple[float, float]) -> Optional[Point3]:
    dx = axis_xy[0] - position[0]
    dy = axis_xy[1] - position[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return None
    return (dx / length, dy / length, 0.0)


def outward_direction_from_axis(position: Point3, axis_xy: Tuple[float, float]) -> Optional[Point3]:
    """Horizontal beam direction from cylinder axis toward the wall point (tilt=0 base)."""
    inward = inward_direction_to_axis(position, axis_xy)
    if inward is None:
        return None
    return (-inward[0], -inward[1], 0.0)


def expected_wrist_corrected_tool_axis_z(
    original_tool_axis_z: Point3,
    position: Point3,
    axis_xy: Tuple[float, float],
) -> Optional[Point3]:
    outward = outward_direction_from_axis(position, axis_xy)
    if outward is None:
        return None
    return rotate_vector_between_directions(original_tool_axis_z, outward, VERTICAL_UPWARD)


def is_wrist_correction_work_pose(pose_type: str) -> bool:
    return pose_type not in ("start", "finish")


def offset_position_for_lcorr(
    position: Point3,
    inward: Point3,
    lcorr_mm: float,
) -> Point3:
    return (
        position[0] + lcorr_mm * inward[0],
        position[1] + lcorr_mm * inward[1],
        position[2] + lcorr_mm,
    )


def apply_wrist_correction_to_pose(
    pose: Pose6D,
    *,
    axis_xy: Tuple[float, float],
    lcorr_mm: float = 0.0,
) -> Pose6D:
    """
    Cylinder wrist fix: rotate the tool frame so horizontal outward becomes vertical upward.

    The same outward→vertical rotation is applied to tool_axis_z and tool_axis_x, preserving
    tilt in the radial–Z plane as forward/back tilt relative to vertical after the 90° turn.
    """
    inward = inward_direction_to_axis(pose.position, axis_xy)
    outward = outward_direction_from_axis(pose.position, axis_xy)
    if inward is None or outward is None:
        return pose

    new_tool_axis_z = rotate_vector_between_directions(
        pose.tool_axis_z, outward, VERTICAL_UPWARD
    )
    rotated_x = rotate_vector_between_directions(
        pose.tool_axis_x, outward, VERTICAL_UPWARD
    )
    new_tool_axis_x = orthogonal_tool_axis_x(rotated_x, new_tool_axis_z)
    new_position = pose.position
    if lcorr_mm > 0.0 and is_wrist_correction_work_pose(pose.pose_type):
        new_position = offset_position_for_lcorr(pose.position, inward, lcorr_mm)
    return replace(
        pose,
        position=new_position,
        tool_axis_z=new_tool_axis_z,
        tool_axis_x=new_tool_axis_x,
    )


def apply_wrist_correction(
    trajectory: WorkTrajectory,
    lcorr_mm: float = 0.0,
) -> WorkTrajectory:
    axis_xy = cylinder_axis_xy(trajectory)
    corrected_poses = [
        apply_wrist_correction_to_pose(pose, axis_xy=axis_xy, lcorr_mm=lcorr_mm)
        for pose in trajectory.poses
    ]
    metadata = dict(trajectory.metadata)
    metadata["wrist_correction_applied"] = True
    metadata["wrist_correction_mode"] = "radial_to_vertical_upward"
    metadata["wrist_correction_axis_xy"] = [axis_xy[0], axis_xy[1]]
    metadata["wrist_correction_target_tool_axis_z"] = list(VERTICAL_UPWARD)
    metadata["wrist_correction_lcorr_mm"] = lcorr_mm
    return replace(
        trajectory,
        poses=corrected_poses,
        metadata=metadata,
    )


def verify_radial_to_vertical_correction(
    trajectory: WorkTrajectory,
    tolerance: float = 1e-3,
) -> bool:
    corrected = apply_wrist_correction(trajectory)
    axis_xy = cylinder_axis_xy(trajectory)
    for original, pose in zip(trajectory.poses, corrected.poses):
        expected_z = expected_wrist_corrected_tool_axis_z(
            original.tool_axis_z, original.position, axis_xy
        )
        if expected_z is None:
            continue
        for index in range(3):
            if abs(pose.tool_axis_z[index] - expected_z[index]) > tolerance:
                return False
    return True
