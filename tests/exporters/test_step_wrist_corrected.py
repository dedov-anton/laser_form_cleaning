import math
import tempfile
import unittest
from pathlib import Path

from src.common.geometry import TOOL_AXIS_VIS_LENGTH_MM, approach_line
from src.exporters.robot.elite_program import build_elite_program
from src.exporters.robot.orientation import pose_to_elite_rpy, rpy_to_rotation_rows
from src.exporters.robot.settings import RobotExportSettings
from src.exporters.step.export import export_trajectory
from src.exporters.step.from_elite_program import (
    _fill_writer_from_elite_work_poses,
    export_step_from_elite_program,
    parse_elite_program_work_poses,
)
from src.exporters.step.writer import (
    MM_TO_M,
    REFERENCE_COLOR,
    TOOL_VIS_COLOR,
    TRAVEL_COLOR,
    WORK_COLOR,
    StepWriter,
)
from src.storage.project_store import load_trajectory_file

ROOT = Path(__file__).resolve().parents[2]
CORRECTED_FIXTURE = ROOT / "config" / "trajectories" / "cylinder_wall_wrist_corrected.json"
UNCORRECTED_FIXTURE = ROOT / "config" / "trajectories" / "cylinder_wall.json"
UP_FIXTURE = ROOT / "УП" / "cylinder_wall_wrist_corrected.txt"


def _lines_by_color(writer: StepWriter, color):
    return [line for line in writer._lines if line.color == color]


def _scale_point(point) -> tuple[float, float, float]:
    return (point[0] * MM_TO_M, point[1] * MM_TO_M, point[2] * MM_TO_M)


def _line_length(start, finish) -> float:
    return math.hypot(
        finish[0] - start[0], finish[1] - start[1], finish[2] - start[2]
    )


def _points_close(a, b, tolerance: float = 1e-5) -> bool:
    return (
        abs(a[0] - b[0]) <= tolerance
        and abs(a[1] - b[1]) <= tolerance
        and abs(a[2] - b[2]) <= tolerance
    )


class TestStepFromEliteProgram(unittest.TestCase):
    def test_parse_up_fixture(self) -> None:
        program = UP_FIXTURE.read_text(encoding="utf-8")
        poses = parse_elite_program_work_poses(program)
        self.assertEqual(len(poses), 48)
        position_mm = (
            poses[0].position_m[0] * 1000.0,
            poses[0].position_m[1] * 1000.0,
            poses[0].position_m[2] * 1000.0,
        )
        self.assertAlmostEqual(position_mm[0], 452.9, delta=0.2)
        self.assertAlmostEqual(position_mm[1], 443.8, delta=0.2)
        self.assertAlmostEqual(position_mm[2], 1593.3, delta=0.2)

    def test_rpy_round_trip(self) -> None:
        beam = (0.9763825861650424, 0.12854320606946865, 0.1736481776669303)
        axis_x = (0.17216259343480456, 0.022665635416815422, -0.984807753012208)
        rpy = pose_to_elite_rpy(beam, axis_x, 0.0)
        _, _, tcp_z = rpy_to_rotation_rows(*rpy)
        self.assertAlmostEqual(tcp_z[0], -beam[0], places=4)
        self.assertAlmostEqual(tcp_z[1], -beam[1], places=4)
        self.assertAlmostEqual(tcp_z[2], -beam[2], places=4)

    def test_step_from_program_matches_up(self) -> None:
        trajectory = load_trajectory_file(CORRECTED_FIXTURE)
        program = build_elite_program(trajectory, RobotExportSettings())
        work_poses = parse_elite_program_work_poses(program)

        writer = StepWriter()
        _fill_writer_from_elite_work_poses(writer, work_poses)

        self.assertEqual(len(_lines_by_color(writer, TOOL_VIS_COLOR)), len(work_poses))
        self.assertEqual(_lines_by_color(writer, TRAVEL_COLOR), [])
        self.assertEqual(_lines_by_color(writer, WORK_COLOR), [])
        self.assertEqual(_lines_by_color(writer, REFERENCE_COLOR), [])

        position_mm = (
            work_poses[0].position_m[0] * 1000.0,
            work_poses[0].position_m[1] * 1000.0,
            work_poses[0].position_m[2] * 1000.0,
        )
        _, _, tcp_z = rpy_to_rotation_rows(*work_poses[0].rpy)
        scaled_tip = _scale_point(position_mm)
        expected_len = TOOL_AXIS_VIS_LENGTH_MM * MM_TO_M
        tool_lines = _lines_by_color(writer, TOOL_VIS_COLOR)
        self.assertTrue(
            any(
                _points_close(line.finish, scaled_tip)
                and abs(_line_length(line.start, line.finish) - expected_len)
                < expected_len * 0.02
                for line in tool_lines
            )
        )
        _, axis_tip = approach_line(position_mm, tcp_z, TOOL_AXIS_VIS_LENGTH_MM)
        scaled_axis_tip = _scale_point(axis_tip)
        self.assertTrue(
            any(_points_close(line.finish, scaled_axis_tip) for line in tool_lines)
        )

    def test_corrected_trajectory_export_raises(self) -> None:
        trajectory = load_trajectory_file(CORRECTED_FIXTURE)
        with self.assertRaises(ValueError):
            export_trajectory(trajectory, Path("unused.step"))

    def test_uncorrected_trajectory_full_export(self) -> None:
        trajectory = load_trajectory_file(UNCORRECTED_FIXTURE)
        writer = StepWriter()
        from src.exporters.step.export import _fill_writer

        _fill_writer(writer, trajectory)
        self.assertGreater(len(_lines_by_color(writer, TRAVEL_COLOR)), 0)
        self.assertGreater(len(_lines_by_color(writer, WORK_COLOR)), 0)

    def test_export_smoke_from_program(self) -> None:
        program = UP_FIXTURE.read_text(encoding="utf-8")
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "from_up.step"
            export_step_from_elite_program(program, path)
            self.assertTrue(path.exists())
            self.assertGreater(path.stat().st_size, 0)
