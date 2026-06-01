from __future__ import annotations

from pathlib import Path

from src.common.trajectory import WorkTrajectory
from src.exporters.robot.elite_program import RobotExportError, build_elite_program
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
