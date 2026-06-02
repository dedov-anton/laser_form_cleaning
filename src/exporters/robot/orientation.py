from __future__ import annotations

import math
from typing import Tuple

from src.common.geometry import Point3

RollPitchYaw = Tuple[float, float, float]


def _normalize(vector: Point3) -> Point3:
    length = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
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


def _project_on_plane(vector: Point3, plane_normal: Point3) -> Point3:
    normal = _normalize(plane_normal)
    projection = _dot(vector, normal)
    return (
        vector[0] - projection * normal[0],
        vector[1] - projection * normal[1],
        vector[2] - projection * normal[2],
    )


def rotation_matrix_to_rpy(rotation: Tuple[Tuple[float, float, float], ...]) -> RollPitchYaw:
    """Extract roll, pitch, yaw (radians); rotation rows are basis columns like bk column_stack."""
    x_vec, y_vec, z_vec = rotation[0], rotation[1], rotation[2]
    sy = math.sqrt(x_vec[0] ** 2 + x_vec[1] ** 2)
    if sy > 1e-6:
        roll = math.atan2(y_vec[2], z_vec[2])
        pitch = math.atan2(-x_vec[2], sy)
        yaw = math.atan2(x_vec[1], x_vec[0])
    else:
        roll = math.atan2(-z_vec[1], y_vec[1])
        pitch = math.atan2(-x_vec[2], sy)
        yaw = 0.0
    return roll, pitch, yaw


def calculate_orientation(
    tool_vector: Point3,
    segment_direction: Point3,
    mount_rotation_rad: float = 0.0,
) -> RollPitchYaw:
    """
    Build tool orientation (roll, pitch, yaw in radians) from TCP +Z and motion direction.

    tool_vector — TCP +Z axis in world frame (not beam direction).
    mount_rotation_rad — rotation around tool Z before RPY extraction (head mount).
    """
    z_axis = _normalize(tool_vector)
    x_proj = _project_on_plane(segment_direction, z_axis)
    if math.sqrt(x_proj[0] ** 2 + x_proj[1] ** 2 + x_proj[2] ** 2) < 1e-6:
        x_proj = (1.0, 0.0, 0.0)
    x_axis = _normalize(x_proj)
    y_axis = _normalize(_cross(z_axis, x_axis))

    cos_r = math.cos(mount_rotation_rad)
    sin_r = math.sin(mount_rotation_rad)
    x_axis_rot = (
        x_axis[0] * cos_r + y_axis[0] * sin_r,
        x_axis[1] * cos_r + y_axis[1] * sin_r,
        x_axis[2] * cos_r + y_axis[2] * sin_r,
    )
    y_axis_rot = (
        -x_axis[0] * sin_r + y_axis[0] * cos_r,
        -x_axis[1] * sin_r + y_axis[1] * cos_r,
        -x_axis[2] * sin_r + y_axis[2] * cos_r,
    )
    rotation = (
        x_axis_rot,
        y_axis_rot,
        z_axis,
    )
    return rotation_matrix_to_rpy(rotation)


def pose_to_elite_rpy(
    tool_axis_z: Point3,
    tool_axis_x: Point3,
    mount_rotation_rad: float,
) -> RollPitchYaw:
    """
    Convert Pose6D tool axes to Elite rx, ry, rz.

    tool_axis_z — laser beam into the surface (trajectory / STEP convention).
    tool_axis_x — X axis of the 6D frame stored in the pose (STEP radial arrow).
    mount_rotation_rad — physical head mount rotation around tool Z.
    """
    tcp_z = (-tool_axis_z[0], -tool_axis_z[1], -tool_axis_z[2])
    roll, pitch, yaw = calculate_orientation(tcp_z, tool_axis_x, mount_rotation_rad)
    return roll, pitch, yaw
