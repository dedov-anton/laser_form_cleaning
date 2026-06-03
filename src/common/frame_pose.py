from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Tuple

from src.common.geometry import Point3

Matrix3 = Tuple[
    Tuple[float, float, float],
    Tuple[float, float, float],
    Tuple[float, float, float],
]


@dataclass(frozen=True)
class FramePose6D:
    """Поза оснастки: XYZ (мм) + Rx Ry Rz (°, Static XYZ / Elite sxyz)."""

    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    rx_deg: float = 0.0
    ry_deg: float = 0.0
    rz_deg: float = 0.0

    def translation(self) -> Point3:
        return (self.x_mm, self.y_mm, self.z_mm)

    def to_dict(self) -> dict[str, float]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> FramePose6D:
        return cls(
            x_mm=float(data.get("x_mm", 0.0)),
            y_mm=float(data.get("y_mm", 0.0)),
            z_mm=float(data.get("z_mm", 0.0)),
            rx_deg=float(data.get("rx_deg", 0.0)),
            ry_deg=float(data.get("ry_deg", 0.0)),
            rz_deg=float(data.get("rz_deg", 0.0)),
        )


def _rotation_x(angle_rad: float) -> Matrix3:
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return (
        (1.0, 0.0, 0.0),
        (0.0, cos_a, -sin_a),
        (0.0, sin_a, cos_a),
    )


def _rotation_y(angle_rad: float) -> Matrix3:
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return (
        (cos_a, 0.0, sin_a),
        (0.0, 1.0, 0.0),
        (-sin_a, 0.0, cos_a),
    )


def _rotation_z(angle_rad: float) -> Matrix3:
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return (
        (cos_a, -sin_a, 0.0),
        (sin_a, cos_a, 0.0),
        (0.0, 0.0, 1.0),
    )


def _multiply_matrix(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def rotation_matrix_sxyz(rx_deg: float, ry_deg: float, rz_deg: float) -> Matrix3:
    """R = Rz(rz) · Ry(ry) · Rx(rx) — фиксированные оси (Static XYZ / Elite)."""
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    return _multiply_matrix(_rotation_z(rz), _multiply_matrix(_rotation_y(ry), _rotation_x(rx)))


def rpy_from_rotation_matrix_sxyz(rotation: Matrix3) -> tuple[float, float, float]:
    """Inverse of rotation_matrix_sxyz; returns (rx, ry, rz) in radians."""
    sy = math.sqrt(rotation[0][0] ** 2 + rotation[1][0] ** 2)
    if sy > 1e-6:
        rx = math.atan2(rotation[2][1], rotation[2][2])
        ry = math.atan2(-rotation[2][0], sy)
        rz = math.atan2(rotation[1][0], rotation[0][0])
    else:
        rx = math.atan2(-rotation[1][2], rotation[1][1])
        ry = math.atan2(-rotation[2][0], sy)
        rz = 0.0
    return rx, ry, rz


def rotation_matrix_sxyz_rad(rx_rad: float, ry_rad: float, rz_rad: float) -> Matrix3:
    return rotation_matrix_sxyz(
        math.degrees(rx_rad),
        math.degrees(ry_rad),
        math.degrees(rz_rad),
    )


def transform_point(point: Point3, rotation: Matrix3, translation: Point3) -> Point3:
    return (
        rotation[0][0] * point[0] + rotation[0][1] * point[1] + rotation[0][2] * point[2] + translation[0],
        rotation[1][0] * point[0] + rotation[1][1] * point[1] + rotation[1][2] * point[2] + translation[1],
        rotation[2][0] * point[0] + rotation[2][1] * point[1] + rotation[2][2] * point[2] + translation[2],
    )


def transform_vector(vector: Point3, rotation: Matrix3) -> Point3:
    transformed = (
        rotation[0][0] * vector[0] + rotation[0][1] * vector[1] + rotation[0][2] * vector[2],
        rotation[1][0] * vector[0] + rotation[1][1] * vector[1] + rotation[1][2] * vector[2],
        rotation[2][0] * vector[0] + rotation[2][1] * vector[1] + rotation[2][2] * vector[2],
    )
    length = math.hypot(transformed[0], transformed[1], transformed[2])
    if length < 1e-12:
        return vector
    return (
        transformed[0] / length,
        transformed[1] / length,
        transformed[2] / length,
    )
