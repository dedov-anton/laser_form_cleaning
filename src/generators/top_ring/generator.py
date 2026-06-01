from __future__ import annotations

from src.common.geometry import Point3
from src.common.trajectory import WorkTrajectory
from src.generators.top_ring.params import TopRingParams


class TopRingGenerator:
    generator_id = "top_ring"
    generator_version = "0.0"

    def validate_params(self, params: TopRingParams) -> None:
        raise NotImplementedError("Генератор верхнего кольца — в разработке")

    def build(
        self,
        params: TopRingParams,
        *,
        start_mm: Point3,
        finish_mm: Point3,
    ) -> WorkTrajectory:
        raise NotImplementedError("Гenerator верхнего кольца — в разработке")
