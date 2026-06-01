from __future__ import annotations

from src.common.geometry import Point3
from src.common.trajectory import WorkTrajectory
from src.generators.cylinder_wall.params import CylinderWallParams


class CylinderWallGenerator:
    generator_id = "cylinder_wall"
    generator_version = "0.0"

    def validate_params(self, params: CylinderWallParams) -> None:
        raise NotImplementedError("Генератор поверхности качения — в разработке")

    def build(
        self,
        params: CylinderWallParams,
        *,
        start_mm: Point3,
        finish_mm: Point3,
    ) -> WorkTrajectory:
        raise NotImplementedError("Генератор поверхности качения — в разработке")
