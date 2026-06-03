from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from src.common.trajectory import WorkTrajectory
from src.exporters.robot.settings import RobotExportSettings
from src.exporters.robot.elite_program import build_elite_program
from src.exporters.step.from_elite_program import export_step_from_elite_program
from src.storage.project_store import load_project, load_trajectory_file, save_trajectory_file
from src.transforms.wrist_correction import (
    VERTICAL_UPWARD,
    apply_wrist_correction,
    cylinder_axis_xy,
    verify_radial_to_vertical_correction,
)


@dataclass(frozen=True)
class WristCorrectionOutputs:
    json_path: Path
    step_path: Path
    up_path: Path
    pose_count: int
    axis_xy: tuple[float, float]
    lcorr_mm: float


def default_cylinder_input_path(project_root: Path) -> Path:
    return project_root / "config" / "trajectories" / "cylinder_wall.json"


def default_cylinder_output_paths(project_root: Path) -> tuple[Path, Path, Path]:
    json_path = project_root / "config" / "trajectories" / "cylinder_wall_wrist_corrected.json"
    step_path = project_root / "output" / "cylinder_wall_wrist_corrected.step"
    up_path = project_root / "УП" / "cylinder_wall_wrist_corrected.txt"
    return json_path, step_path, up_path


def run_cylinder_wrist_correction(
    project_root: Path,
    *,
    trajectory: WorkTrajectory | None = None,
    input_path: Path | None = None,
    output_json_path: Path | None = None,
    output_step_path: Path | None = None,
    output_up_path: Path | None = None,
    robot_settings: RobotExportSettings | None = None,
    lcorr_mm: float | None = None,
) -> WristCorrectionOutputs:
    source = trajectory
    if source is None:
        source_path = input_path or default_cylinder_input_path(project_root)
        if not source_path.exists():
            raise FileNotFoundError(f"Trajectory not found: {source_path}")
        source = load_trajectory_file(source_path)

    if source.generator_id != "cylinder_wall":
        raise ValueError(f"Expected generator_id=cylinder_wall, got {source.generator_id!r}")

    if not verify_radial_to_vertical_correction(source):
        raise RuntimeError("Radial-to-vertical correction verification failed")

    if lcorr_mm is None:
        project = load_project(project_root)
        lcorr_mm = float(project.cylinder_wall.get("lcorr_mm", 0.0))

    corrected = apply_wrist_correction(source, lcorr_mm=lcorr_mm)
    json_path, step_path, up_path = default_cylinder_output_paths(project_root)
    json_path = output_json_path or json_path
    step_path = output_step_path or step_path
    up_path = output_up_path or up_path

    save_trajectory_file(json_path, corrected)

    if robot_settings is None:
        project = load_project(project_root)
        robot_settings = RobotExportSettings(
            blend_radius_mm=project.robot_blend_radius_mm,
            velocity=project.robot_velocity,
            acceleration=project.robot_acceleration,
            tool_mount_rotation_deg=project.tool_mount_rotation_deg,
        )

    program = build_elite_program(corrected, robot_settings)
    up_path.parent.mkdir(parents=True, exist_ok=True)
    up_path.write_text(program, encoding="utf-8")

    step_path.parent.mkdir(parents=True, exist_ok=True)
    export_step_from_elite_program(program, step_path)

    return WristCorrectionOutputs(
        json_path=json_path,
        step_path=step_path,
        up_path=up_path,
        pose_count=len(corrected.poses),
        axis_xy=cylinder_axis_xy(source),
        lcorr_mm=lcorr_mm,
    )


def format_wrist_correction_summary(outputs: WristCorrectionOutputs) -> str:
    return (
        "Коррекция запястья выполнена.\n\n"
        f"Ось цилиндра XY: {outputs.axis_xy[0]:.1f}, {outputs.axis_xy[1]:.1f} мм\n"
        f"Lcorr: {outputs.lcorr_mm:.1f} мм\n"
        f"tool_axis_z: {VERTICAL_UPWARD}\n"
        f"Поз: {outputs.pose_count}\n\n"
        f"JSON:\n{outputs.json_path}\n\n"
        f"STEP:\n{outputs.step_path}\n\n"
        f"УП:\n{outputs.up_path}"
    )
