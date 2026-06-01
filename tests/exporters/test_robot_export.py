import math
import tempfile
import unittest
from pathlib import Path

from src.common.geometry import Pose6D
from src.common.project import FormProject
from src.common.trajectory import WorkTrajectory
from src.exporters.robot.elite_program import RobotExportError, build_elite_program
from src.exporters.robot.export import export_robot_program
from src.exporters.robot.orientation import calculate_orientation
from src.exporters.robot.settings import RobotExportSettings
from src.storage.project_store import load_project, save_project


def _minimal_world_trajectory() -> WorkTrajectory:
    return WorkTrajectory(
        generator_id="bottom_ring",
        generator_version="1.0",
        params_snapshot={},
        start_point_mm=(0.0, 0.0, 300.0),
        finish_point_mm=(0.0, 0.0, 300.0),
        poses=[
            Pose6D(
                index=0,
                pose_type="start",
                position=(0.0, 0.0, 300.0),
                tool_axis_z=(0.0, 0.0, -1.0),
                tool_axis_x=(1.0, 0.0, 0.0),
            ),
            Pose6D(
                index=1,
                pose_type="work_inner",
                position=(500.0, 0.0, 0.0),
                tool_axis_z=(0.0, 0.0, -1.0),
                tool_axis_x=(0.0, 1.0, 0.0),
            ),
            Pose6D(
                index=2,
                pose_type="work_inner",
                position=(500.0, 100.0, 0.0),
                tool_axis_z=(0.0, 0.0, -1.0),
                tool_axis_x=(0.0, 1.0, 0.0),
            ),
            Pose6D(
                index=3,
                pose_type="work_inner",
                position=(500.0, 200.0, 0.0),
                tool_axis_z=(0.0, 0.0, -1.0),
                tool_axis_x=(0.0, 1.0, 0.0),
            ),
            Pose6D(
                index=4,
                pose_type="finish",
                position=(0.0, 0.0, 300.0),
                tool_axis_z=(0.0, 0.0, -1.0),
                tool_axis_x=(1.0, 0.0, 0.0),
            ),
        ],
        travel_segments=[],
        work_segments=[],
        metadata={"coordinate_frame": "world"},
    )


class OrientationTests(unittest.TestCase):
    def test_tool_down_motion_along_x(self) -> None:
        roll, pitch, yaw = calculate_orientation(
            (0.0, 0.0, -1.0),
            (1.0, 0.0, 0.0),
        )
        self.assertTrue(math.isfinite(roll))
        self.assertTrue(math.isfinite(pitch))
        self.assertTrue(math.isfinite(yaw))


class EliteProgramTests(unittest.TestCase):
    def test_build_program_contains_movel_and_units(self) -> None:
        settings = RobotExportSettings(blend_radius_mm=1.0)
        program = build_elite_program(_minimal_world_trajectory(), settings)

        self.assertIn("# TRAJECTORY_LAYER: bottom_ring", program)
        self.assertIn("def welding_program():", program)
        self.assertIn("movel(pose_start", program)
        self.assertIn("movel(pose_work_1", program)
        self.assertNotIn("pose_safe", program)
        self.assertIn("r=0.0010", program)
        self.assertIn("pose_work_0 = [0.5000, 0.0000, 0.0000", program)
        self.assertIn("end", program)

    def test_no_work_poses_raises(self) -> None:
        trajectory = WorkTrajectory(
            generator_id="test",
            generator_version="1.0",
            params_snapshot={},
            start_point_mm=(0.0, 0.0, 0.0),
            finish_point_mm=(0.0, 0.0, 0.0),
            poses=[
                Pose6D(
                    index=0,
                    pose_type="start",
                    position=(0.0, 0.0, 0.0),
                    tool_axis_z=(0.0, 0.0, -1.0),
                    tool_axis_x=(1.0, 0.0, 0.0),
                )
            ],
            travel_segments=[],
            work_segments=[],
            metadata={"coordinate_frame": "world"},
        )
        with self.assertRaises(RobotExportError):
            build_elite_program(trajectory, RobotExportSettings())

    def test_local_frame_rejected_on_export(self) -> None:
        trajectory = _minimal_world_trajectory()
        trajectory.metadata["coordinate_frame"] = "local"
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "prog.txt"
            with self.assertRaises(RobotExportError):
                export_robot_program(trajectory, path, RobotExportSettings())

    def test_export_writes_file(self) -> None:
        trajectory = _minimal_world_trajectory()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "prog.txt"
            export_robot_program(trajectory, path, RobotExportSettings())
            text = path.read_text(encoding="utf-8")
            self.assertIn("def welding_program():", text)


class RobotProjectPersistenceTests(unittest.TestCase):
    def test_robot_settings_roundtrip(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            project = FormProject(
                robot_blend_radius_mm=0.002,
                robot_velocity=0.04,
                robot_acceleration=0.06,
            )
            save_project(project, root)
            loaded = load_project(root)
            self.assertAlmostEqual(loaded.robot_blend_radius_mm, 0.002)
            self.assertAlmostEqual(loaded.robot_velocity, 0.04)
            self.assertAlmostEqual(loaded.robot_acceleration, 0.06)

    def test_missing_robot_fields_use_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            config = root / "config"
            config.mkdir()
            (config / "project.json").write_text('{"beam_width_mm": 100.0}', encoding="utf-8")
            loaded = load_project(root)
            self.assertAlmostEqual(loaded.robot_blend_radius_mm, 0.001)
            self.assertAlmostEqual(loaded.robot_velocity, 0.03)


if __name__ == "__main__":
    unittest.main()
