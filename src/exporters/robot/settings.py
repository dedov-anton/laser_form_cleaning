from __future__ import annotations

from dataclasses import dataclass
import math

DEFAULT_TOOL_MOUNT_ROTATION_DEG = 0.0

DEFAULT_BLEND_RADIUS_MM = 0.001
DEFAULT_VELOCITY = 0.03
DEFAULT_ACCELERATION = 0.05


@dataclass(frozen=True)
class RobotExportSettings:
    blend_radius_mm: float = DEFAULT_BLEND_RADIUS_MM
    velocity: float = DEFAULT_VELOCITY
    acceleration: float = DEFAULT_ACCELERATION
    tool_mount_rotation_deg: float = DEFAULT_TOOL_MOUNT_ROTATION_DEG

    @property
    def blend_radius_m(self) -> float:
        return self.blend_radius_mm / 1000.0

    @property
    def tool_mount_rotation_rad(self) -> float:
        return math.radians(self.tool_mount_rotation_deg)
