import math
import re
import tempfile
import unittest
from pathlib import Path

from src.common.geometry import Pose6D
from src.common.project import FormProject
from src.common.trajectory import WorkTrajectory
from src.exporters.robot.elite_program import RobotExportError, _work_poses, build_elite_program
from src.exporters.robot.export import export_robot_program
from src.exporters.robot.orientation import pose_to_elite_rpy
from src.generators.bottom_ring.adapter import to_work_trajectory
from src.generators.bottom_ring.calc import build_trajectory
from src.generators.bottom_ring.params import BottomRingParams
from src.exporters.robot.settings import RobotExportSettings
from src.storage.project_store import load_project, save_project

CALIBRATION_POSE_PATTERN = re.compile(
    r"pose_\w+\s*=\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*"
    r"([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]"
)


def _angles_equal(left: float, right: float, places: int = 4) -> None:
    delta = math.atan2(math.sin(left - right), math.cos(left - right))
    assert abs(delta) < 10 ** (-places), f"{left} != {right}"


def _calibration_trajectory() -> WorkTrajectory:
    return WorkTrajectory(
        generator_id="calibration",
        generator_version="1.0",
        params_snapshot={},
        start_point_mm=(300.0, 300.0, 200.0),
        finish_point_mm=(400.0, 300.0, 200.0),
        poses=[
            Pose6D(
                index=0,
                pose_type="start",
                position=(300.0, 300.0, 200.0),
                tool_axis_z=(0.0, 0.0, -1.0),
                tool_axis_x=(1.0, 0.0, 0.0),
            ),
            Pose6D(
                index=1,
                pose_type="work",
                position=(300.0, 300.0, 200.0),
                tool_axis_z=(0.0, 0.0, -1.0),
                tool_axis_x=(1.0, 0.0, 0.0),
            ),
            Pose6D(
                index=2,
                pose_type="work",
                position=(400.0, 300.0, 200.0),
                tool_axis_z=(0.0, 0.0, -1.0),
                tool_axis_x=(1.0, 0.0, 0.0),
            ),
            Pose6D(
                index=3,
                pose_type="finish",
                position=(400.0, 300.0, 200.0),
                tool_axis_z=(0.0, 0.0, -1.0),
                tool_axis_x=(1.0, 0.0, 0.0),
            ),
        ],
        travel_segments=[],
        work_segments=[],
        metadata={"coordinate_frame": "world"},
    )


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
    def test_pose_to_elite_rpy_vertical_down(self) -> None:
        roll, pitch, yaw = pose_to_elite_rpy(
            (0.0, 0.0, -1.0),
            (1.0, 0.0, 0.0),
            0.0,
        )
        self.assertAlmostEqual(roll, 0.0, places=4)
        self.assertAlmostEqual(pitch, 0.0, places=4)
        self.assertAlmostEqual(yaw, 0.0, places=4)

    def test_rz_from_pose_tool_axis_x(self) -> None:
        cases = (
            ((1.0, 0.0, 0.0), 0.0),
            ((0.0, 1.0, 0.0), math.pi / 2),
            ((-1.0, 0.0, 0.0), math.pi),
            ((0.0, -1.0, 0.0), -math.pi / 2),
        )
        beam = (0.0, 0.0, -1.0)
        for tool_axis_x, expected_rz in cases:
            with self.subTest(tool_axis_x=tool_axis_x):
                roll, pitch, yaw = pose_to_elite_rpy(beam, tool_axis_x, 0.0)
                self.assertAlmostEqual(roll, 0.0, places=4)
                self.assertAlmostEqual(pitch, 0.0, places=4)
                _angles_equal(yaw, expected_rz)

    def test_same_pass_constant_rz_in_export(self) -> None:
        params = BottomRingParams(
            inner_radius_mm=250.0,
            ring_width_mm=150.0,
            beam_width_mm=100.0,
            sector_count=8,
        )
        work = to_work_trajectory(build_trajectory(params), params)
        work_poses = _work_poses(work)
        outer = next(p for p in work_poses if p.pose_type == "work_outer")
        inner = next(
            p
            for p in work_poses
            if p.pose_type == "work_inner"
            and math.isclose(p.tool_axis_x[0], outer.tool_axis_x[0], abs_tol=1e-6)
            and math.isclose(p.tool_axis_x[1], outer.tool_axis_x[1], abs_tol=1e-6)
        )
        settings = RobotExportSettings(tool_mount_rotation_deg=0.0)
        program = build_elite_program(work, settings)
        poses = CALIBRATION_POSE_PATTERN.findall(program)
        outer_rz = None
        inner_rz = None
        for pose in poses:
            x_m, y_m, roll, pitch, yaw = (
                float(pose[0]),
                float(pose[1]),
                float(pose[3]),
                float(pose[4]),
                float(pose[5]),
            )
            if math.isclose(x_m, outer.position[0] / 1000.0, abs_tol=1e-4) and math.isclose(
                y_m, outer.position[1] / 1000.0, abs_tol=1e-4
            ):
                outer_rz = yaw
            if math.isclose(x_m, inner.position[0] / 1000.0, abs_tol=1e-4) and math.isclose(
                y_m, inner.position[1] / 1000.0, abs_tol=1e-4
            ):
                inner_rz = yaw
        self.assertIsNotNone(outer_rz)
        self.assertIsNotNone(inner_rz)
        _angles_equal(outer_rz, inner_rz)

    def test_mount_affects_rz_not_rx(self) -> None:
        zero = pose_to_elite_rpy((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), 0.0)
        mounted = pose_to_elite_rpy(
            (0.0, 0.0, -1.0),
            (0.0, 1.0, 0.0),
            math.radians(90.0),
        )
        self.assertAlmostEqual(zero[0], 0.0, places=4)
        self.assertAlmostEqual(zero[1], 0.0, places=4)
        self.assertAlmostEqual(mounted[0], 0.0, places=4)
        self.assertAlmostEqual(mounted[1], 0.0, places=4)
        self.assertAlmostEqual(zero[2], math.pi / 2, places=4)
        self.assertNotAlmostEqual(zero[2], mounted[2], places=3)
        self.assertNotAlmostEqual(mounted[0], math.radians(90.0), places=3)

    def test_cylinder_radial_pose_rpy_bk_parity(self) -> None:
        """Horizontal radial beam + vertical path tangent; mount -90 matches bk column_stack RPY."""
        x_mm, y_mm = 297.4334584121431, 39.157857666015474
        radius = math.hypot(x_mm, y_mm)
        tool_axis_z = (x_mm / radius, y_mm / radius, 0.0)
        tool_axis_x = (0.0, 0.0, -1.0)
        mount = math.radians(-90.0)

        roll, pitch, yaw = pose_to_elite_rpy(tool_axis_z, tool_axis_x, mount)

        self.assertAlmostEqual(roll, -math.pi / 2, places=3)
        self.assertAlmostEqual(pitch, 0.0, places=3)
        _angles_equal(yaw, math.atan2(y_mm, x_mm) + math.pi / 2)


class EliteProgramTests(unittest.TestCase):
    def test_calibration_trajectory_exports_zero_rpy(self) -> None:
        settings = RobotExportSettings(tool_mount_rotation_deg=0.0)
        program = build_elite_program(_calibration_trajectory(), settings)
        poses = CALIBRATION_POSE_PATTERN.findall(program)
        self.assertGreaterEqual(len(poses), 2)
        for pose in poses:
            roll, pitch, yaw = (float(pose[3]), float(pose[4]), float(pose[5]))
            self.assertAlmostEqual(roll, 0.0, places=4)
            self.assertAlmostEqual(pitch, 0.0, places=4)
            self.assertAlmostEqual(yaw, 0.0, places=4)

    def test_build_program_contains_movel_and_units(self) -> None:
        settings = RobotExportSettings(blend_radius_mm=1.0, tool_mount_rotation_deg=0.0)
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
                tool_mount_rotation_deg=90.0,
            )
            save_project(project, root)
            loaded = load_project(root)
            self.assertAlmostEqual(loaded.robot_blend_radius_mm, 0.002)
            self.assertAlmostEqual(loaded.robot_velocity, 0.04)
            self.assertAlmostEqual(loaded.robot_acceleration, 0.06)
            self.assertAlmostEqual(loaded.tool_mount_rotation_deg, 90.0)

    def test_missing_robot_fields_use_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            config = root / "config"
            config.mkdir()
            (config / "project.json").write_text('{"beam_width_mm": 100.0}', encoding="utf-8")
            loaded = load_project(root)
            self.assertAlmostEqual(loaded.robot_blend_radius_mm, 0.001)
            self.assertAlmostEqual(loaded.robot_velocity, 0.03)
            self.assertAlmostEqual(loaded.tool_mount_rotation_deg, 0.0)


if __name__ == "__main__":
    unittest.main()
