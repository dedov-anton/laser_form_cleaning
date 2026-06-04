from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass
class TopRingParams:
    inner_radius_mm: float
    ring_width_mm: float
    z_to_top_mm: float
    beam_width_mm: float

    @property
    def outer_radius_mm(self) -> float:
        return self.inner_radius_mm + self.ring_width_mm

    def to_dict(self) -> dict:
        return asdict(self)
