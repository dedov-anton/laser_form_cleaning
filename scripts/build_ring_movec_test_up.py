#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.exporters.robot.elite_program import build_ring_movec_test_program
from src.exporters.robot.settings import RobotExportSettings
from src.storage.project_store import load_project, load_trajectory_file


def main() -> int:
    trajectory_path = PROJECT_ROOT / "config" / "trajectories" / "bottom_ring.json"
    output_path = PROJECT_ROOT / "УП" / "test_ring_movec_180_to_0.txt"

    if not trajectory_path.exists():
        print(f"Trajectory not found: {trajectory_path}", file=sys.stderr)
        return 1

    trajectory = load_trajectory_file(trajectory_path)
    project = load_project(PROJECT_ROOT)
    settings = RobotExportSettings(
        blend_radius_mm=project.robot_blend_radius_mm,
        velocity=project.robot_velocity,
        acceleration=project.robot_acceleration,
        tool_mount_rotation_deg=project.tool_mount_rotation_deg,
    )

    program = build_ring_movec_test_program(trajectory, settings)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(program, encoding="utf-8")
    print(f"Written: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
