from __future__ import annotations

import math

from src.common.geometry import Point3


def normalize_vector(vector: Point3) -> Point3:
    length = math.hypot(vector[0], vector[1], vector[2])
    if length < 1e-9:
        return vector
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def cross(a: Point3, b: Point3) -> Point3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def rotate_vector(vector: Point3, axis: Point3, angle_deg: float) -> Point3:
    if abs(angle_deg) < 1e-9:
        return vector

    unit_axis = normalize_vector(axis)
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    cross_product = cross(unit_axis, vector)
    dot_product = dot(unit_axis, vector)

    return (
        vector[0] * cos_a + cross_product[0] * sin_a + unit_axis[0] * dot_product * (1.0 - cos_a),
        vector[1] * cos_a + cross_product[1] * sin_a + unit_axis[1] * dot_product * (1.0 - cos_a),
        vector[2] * cos_a + cross_product[2] * sin_a + unit_axis[2] * dot_product * (1.0 - cos_a),
    )


def radial_direction_xy(position: Point3) -> Point3:
    x, y, _ = position
    length = math.hypot(x, y)
    if length < 1e-9:
        return (1.0, 0.0, 0.0)
    return (x / length, y / length, 0.0)


def inward_direction_xy(position: Point3) -> Point3:
    outward = radial_direction_xy(position)
    return (-outward[0], -outward[1], 0.0)


def tangent_direction_xy(position: Point3) -> Point3:
    x, y, _ = position
    length = math.hypot(x, y)
    if length < 1e-9:
        return (0.0, 1.0, 0.0)
    return (-y / length, x / length, 0.0)


def orthogonal_tool_axis_x(preferred: Point3, tool_axis_z: Point3) -> Point3:
    projection = dot(preferred, tool_axis_z)
    projected = (
        preferred[0] - projection * tool_axis_z[0],
        preferred[1] - projection * tool_axis_z[1],
        preferred[2] - projection * tool_axis_z[2],
    )
    length = math.hypot(projected[0], projected[1], projected[2])
    if length < 1e-9:
        return radial_direction_xy(preferred)
    return (projected[0] / length, projected[1] / length, projected[2] / length)


def tilt_tool_axis_radial(position: Point3, tilt_deg: float) -> Point3:
    """Ось Z: от оси цилиндра к точке на внутренней стенке (инструмент внутри)."""
    outward = radial_direction_xy(position)
    if abs(tilt_deg) < 1e-9:
        return outward
    tangent = tangent_direction_xy(position)
    return normalize_vector(rotate_vector(outward, tangent, -tilt_deg))
