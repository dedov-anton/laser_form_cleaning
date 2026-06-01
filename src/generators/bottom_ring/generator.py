from __future__ import annotations

from src.common.geometry import Point3
from src.common.trajectory import WorkTrajectory
from src.generators.bottom_ring.adapter import GENERATOR_ID, GENERATOR_VERSION, to_work_trajectory
from src.generators.bottom_ring.calc import build_trajectory
from src.generators.bottom_ring.params import BottomRingParams


class BottomRingGenerator:
    generator_id = GENERATOR_ID
    generator_version = GENERATOR_VERSION

    def validate_params(self, params: BottomRingParams) -> None:
        build_trajectory(
            params,
            start_point_mm=(0.0, 0.0, 0.0),
            finish_point_mm=(0.0, 0.0, 0.0),
        )

    def build(
        self,
        params: BottomRingParams,
        *,
        start_mm: Point3,
        finish_mm: Point3,
    ) -> WorkTrajectory:
        ring_trajectory = build_trajectory(
            params,
            start_point_mm=start_mm,
            finish_point_mm=finish_mm,
        )
        return to_work_trajectory(ring_trajectory, params)
