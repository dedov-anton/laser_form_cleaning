from __future__ import annotations

from pathlib import Path

from src.common.frame_pose import FramePose6D
from src.common.geometry import Point3
from src.common.trajectory import WorkTrajectory
from src.exporters.robot.elite_program import (
    RobotExportError,
    build_elite_program,
    build_ring_arc_program,
)
from src.exporters.robot.ring_arc import build_ring_arc_geometry, plan_arc_passes
from src.exporters.robot.settings import RobotExportSettings


class RobotExportNotImplementedError(NotImplementedError):
    pass


def export_robot_program(
    trajectory: WorkTrajectory,
    filepath: Path,
    settings: RobotExportSettings | None = None,
) -> None:
    if settings is None:
        settings = RobotExportSettings()

    frame = trajectory.metadata.get("coordinate_frame", "local")
    if frame != "world":
        raise RobotExportError(
            "Траектория в локальной СК. Сначала нажмите «Переместить по 6D позе»."
        )

    program = build_elite_program(trajectory, settings)
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)
    filepath.write_text(program, encoding="utf-8")


def export_ring_arc_program(
    *,
    frame_pose: FramePose6D,
    inner_radius_mm: float,
    ring_width_mm: float,
    beam_width_mm: float,
    filepath: Path,
    settings: RobotExportSettings | None = None,
    generator_id: str = "bottom_ring",
    local_z_mm: float = 0.0,
) -> None:
    if settings is None:
        settings = RobotExportSettings()

    geometry = build_ring_arc_geometry(
        frame_pose,
        inner_radius_mm,
        ring_width_mm,
        beam_width_mm,
        generator_id=generator_id,
        local_z_mm=local_z_mm,
    )
    plan = plan_arc_passes(inner_radius_mm, ring_width_mm, beam_width_mm)
    program = build_ring_arc_program(
        settings,
        geometry=geometry,
        pass_plan=plan,
    )
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)
    filepath.write_text(program, encoding="utf-8")
