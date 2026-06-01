from __future__ import annotations

import math
from typing import Tuple

from src.common.geometry import Point3
from src.exporters.robot.settings import TOOL_ROTATION_CORRECTION_RAD

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


def rotation_matrix_to_rpy(rotation: Tuple[Tuple[float, float, float], ...]) -> RollPitchYaw:
    """Extract roll, pitch, yaw (radians) from rotation matrix columns X, Y, Z."""
    r00, r10, r20 = rotation[0][0], rotation[1][0], rotation[2][0]
    r21, r22 = rotation[2][1], rotation[2][2]
    r11, r12 = rotation[1][1], rotation[1][2]

    sy = math.sqrt(r00 ** 2 + r10 ** 2)
    if sy > 1e-6:
        roll = math.atan2(r21, r22)
        pitch = math.atan2(-r20, sy)
        yaw = math.atan2(r10, r00)
    else:
        roll = math.atan2(-r12, r11)
        pitch = math.atan2(-r20, sy)
        yaw = 0.0
    return roll, pitch, yaw


def calculate_orientation(
    tool_vector: Point3,
    segment_direction: Point3,
    additional_rotation: float = 0.0,
) -> RollPitchYaw:
    """
    Build tool orientation (roll, pitch, yaw in radians) from tool Z axis and motion direction.

    Ported from welding_path_generator_bk.py — Static XYZ convention with -90° roll correction.
    """
    z_axis = _normalize(tool_vector)
    x_proj = (
        segment_direction[0] - _dot(segment_direction, z_axis) * z_axis[0],
        segment_direction[1] - _dot(segment_direction, z_axis) * z_axis[1],
        segment_direction[2] - _dot(segment_direction, z_axis) * z_axis[2],
    )
    if math.sqrt(x_proj[0] ** 2 + x_proj[1] ** 2 + x_proj[2] ** 2) < 1e-6:
        x_proj = (1.0, 0.0, 0.0)
    x_axis = _normalize(x_proj)
    y_axis = _normalize(_cross(z_axis, x_axis))

    cos_r = math.cos(additional_rotation + TOOL_ROTATION_CORRECTION_RAD)
    sin_r = math.sin(additional_rotation + TOOL_ROTATION_CORRECTION_RAD)
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
