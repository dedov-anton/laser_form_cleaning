from __future__ import annotations

from dataclasses import dataclass
import math

TOOL_ROTATION_CORRECTION_DEG = -90.0
TOOL_ROTATION_CORRECTION_RAD = math.radians(TOOL_ROTATION_CORRECTION_DEG)

DEFAULT_BLEND_RADIUS_MM = 0.001
DEFAULT_VELOCITY = 0.03
DEFAULT_ACCELERATION = 0.05


@dataclass(frozen=True)
class RobotExportSettings:
    blend_radius_mm: float = DEFAULT_BLEND_RADIUS_MM
    velocity: float = DEFAULT_VELOCITY
    acceleration: float = DEFAULT_ACCELERATION

    @property
    def blend_radius_m(self) -> float:
        return self.blend_radius_mm / 1000.0
