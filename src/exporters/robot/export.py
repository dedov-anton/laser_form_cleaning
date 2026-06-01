from __future__ import annotations

from pathlib import Path

from src.common.trajectory import WorkTrajectory


class RobotExportNotImplementedError(NotImplementedError):
    pass


def export_robot_program(trajectory: WorkTrajectory, filepath: Path) -> None:
    raise RobotExportNotImplementedError(
        f"Экспорт управляющей программы для «{trajectory.generator_id}» — следующий этап"
    )
