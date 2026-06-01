from __future__ import annotations

from src.common.generator import TrajectoryGenerator
from src.generators.bottom_ring.generator import BottomRingGenerator
from src.generators.cylinder_wall.generator import CylinderWallGenerator
from src.generators.top_ring.generator import TopRingGenerator

GENERATORS: dict[str, TrajectoryGenerator] = {
    "bottom_ring": BottomRingGenerator(),
    "top_ring": TopRingGenerator(),
    "cylinder_wall": CylinderWallGenerator(),
}


def get_generator(generator_id: str) -> TrajectoryGenerator:
    try:
        return GENERATORS[generator_id]
    except KeyError as error:
        raise KeyError(f"Unknown generator: {generator_id}") from error
