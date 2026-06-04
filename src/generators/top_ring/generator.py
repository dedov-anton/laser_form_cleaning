from __future__ import annotations

from src.common.geometry import Point3
from src.common.trajectory import WorkTrajectory
from src.generators.top_ring.params import TopRingParams


class TopRingGenerator:
    generator_id = "top_ring"
    generator_version = "0.1"

    def validate_params(self, params: TopRingParams) -> None:
        if params.inner_radius_mm <= 0.0:
            raise ValueError("inner_radius_mm must be positive")
        if params.ring_width_mm <= 0.0:
            raise ValueError("ring_width_mm must be positive")
        if params.beam_width_mm <= 0.0:
            raise ValueError("beam_width_mm must be positive")

    def build(
        self,
        params: TopRingParams,
        *,
        start_mm: Point3,
        finish_mm: Point3,
    ) -> WorkTrajectory:
        raise NotImplementedError(
            "Генератор верхнего кольца: используйте «Создать УП дугами» (радиальная траектория не реализована)"
        )
