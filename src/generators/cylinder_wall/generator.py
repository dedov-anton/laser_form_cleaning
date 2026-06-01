from __future__ import annotations

from src.common.geometry import Point3
from src.common.trajectory import WorkTrajectory
from src.generators.cylinder_wall.adapter import GENERATOR_ID, GENERATOR_VERSION, to_work_trajectory
from src.generators.cylinder_wall.calc import build_trajectory
from src.generators.cylinder_wall.params import CylinderWallParams


class CylinderWallGenerator:
    generator_id = GENERATOR_ID
    generator_version = GENERATOR_VERSION

    def validate_params(self, params: CylinderWallParams) -> None:
        build_trajectory(
            params,
            start_point_mm=(0.0, 0.0, 0.0),
            finish_point_mm=(0.0, 0.0, 0.0),
        )

    def build(
        self,
        params: CylinderWallParams,
        *,
        start_mm: Point3,
        finish_mm: Point3,
    ) -> WorkTrajectory:
        cylinder_trajectory = build_trajectory(
            params,
            start_point_mm=start_mm,
            finish_point_mm=finish_mm,
        )
        return to_work_trajectory(cylinder_trajectory, params)
