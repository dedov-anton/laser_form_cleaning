from __future__ import annotations

from typing import Any, Protocol

from src.common.geometry import Point3
from src.common.trajectory import WorkTrajectory


class TrajectoryGenerator(Protocol):
    generator_id: str
    generator_version: str

    def validate_params(self, params: Any) -> None: ...

    def build(
        self,
        params: Any,
        *,
        start_mm: Point3,
        finish_mm: Point3,
    ) -> WorkTrajectory: ...
