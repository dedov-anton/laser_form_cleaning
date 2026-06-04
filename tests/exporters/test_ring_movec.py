import math
import unittest
from pathlib import Path

from src.common.frame_pose import FramePose6D
from src.exporters.robot.elite_program import (
    build_ring_arc_program,
    build_ring_arc_program_from_trajectory,
    build_ring_movec_test_program,
)
from src.exporters.robot.ring_arc import (
    build_ccw_semicircle_poses,
    build_cw_semicircle_poses,
    build_ring_arc_geometry,
    geometry_with_track_radius,
    plan_arc_passes,
    ring_arc_geometry_from_trajectory,
    track_radius_mm,
)
from src.exporters.robot.settings import RobotExportSettings
from src.storage.project_store import load_trajectory_file


class RingMovecTests(unittest.TestCase):
    def setUp(self) -> None:
        root = Path(__file__).resolve().parents[2]
        self.trajectory_path = root / "config" / "trajectories" / "bottom_ring.json"
        if not self.trajectory_path.exists():
            self.skipTest("bottom_ring.json fixture missing")
        self.trajectory = load_trajectory_file(self.trajectory_path)
        self.settings = RobotExportSettings()

    def test_geometry_from_bottom_ring_json(self) -> None:
        geometry = ring_arc_geometry_from_trajectory(self.trajectory)
        self.assertEqual(geometry.generator_id, "bottom_ring")
        self.assertAlmostEqual(geometry.center_xy[0], 600.0, places=3)
        self.assertAlmostEqual(geometry.center_xy[1], 500.0, places=3)
        self.assertAlmostEqual(geometry.inner_radius_mm, 200.0, places=3)
        self.assertAlmostEqual(geometry.ring_width_mm, 100.0, places=3)
        self.assertAlmostEqual(geometry.track_radius_mm, 250.0, places=3)
        self.assertAlmostEqual(geometry.z_mm, 200.0, places=3)

    def test_build_ring_arc_geometry_from_pose(self) -> None:
        pose = FramePose6D(x_mm=600.0, y_mm=500.0, z_mm=200.0)
        geometry = build_ring_arc_geometry(pose, 200.0, 100.0, 100.0)
        self.assertAlmostEqual(geometry.center_xy[0], 600.0, places=3)
        self.assertAlmostEqual(geometry.z_mm, 200.0, places=3)

    def test_arc_geometry_with_fixture_tilt(self) -> None:
        pose = FramePose6D(x_mm=100.0, y_mm=0.0, z_mm=50.0, ry_deg=90.0)
        geometry = geometry_with_track_radius(
            build_ring_arc_geometry(pose, 200.0, 100.0, 100.0),
            250.0,
        )
        start, _, _ = build_cw_semicircle_poses(geometry, 180.0, 90.0, 0.0)
        self.assertNotAlmostEqual(start.position[2], 50.0, places=1)

    def test_arc_points_cw_semicircle(self) -> None:
        geometry = build_ring_arc_geometry(
            FramePose6D(x_mm=600.0, y_mm=500.0, z_mm=200.0),
            200.0,
            100.0,
            100.0,
        )
        start, via, end = build_cw_semicircle_poses(
            geometry, start_deg=180.0, via_deg=90.0, end_deg=0.0
        )
        self.assertAlmostEqual(start.position[0], 350.0, places=2)
        self.assertAlmostEqual(start.position[1], 500.0, places=2)
        self.assertAlmostEqual(via.position[0], 600.0, places=2)
        self.assertAlmostEqual(via.position[1], 750.0, places=2)
        self.assertAlmostEqual(end.position[0], 850.0, places=2)
        self.assertAlmostEqual(end.position[1], 500.0, places=2)
        for pose in (start, via, end):
            self.assertAlmostEqual(pose.position[2], 200.0, places=2)

    def test_track_radius_midline(self) -> None:
        self.assertAlmostEqual(track_radius_mm(200.0, 100.0), 250.0, places=6)

    def test_program_contains_movec_and_center_comment(self) -> None:
        program = build_ring_movec_test_program(self.trajectory, self.settings)
        self.assertIn("movec(pose_arc_via_90, pose_arc_end_0", program)
        self.assertIn("mode=0", program)
        movec_line = next(line for line in program.splitlines() if "movec(" in line)
        self.assertNotIn("t=", movec_line)
        self.assertIn("movel(pose_arc_start_180", program)
        self.assertIn("center_xy_mm: 600.0000, 500.0000", program)
        self.assertIn("track_radius_mm: 250.0000", program)

    def test_tangent_at_180_points_cw(self) -> None:
        geometry = ring_arc_geometry_from_trajectory(self.trajectory)
        start, _, _ = build_cw_semicircle_poses(geometry, 180.0, 90.0, 0.0)
        length = math.hypot(start.tool_axis_x[0], start.tool_axis_x[1])
        self.assertAlmostEqual(length, 1.0, places=5)
        self.assertGreater(start.tool_axis_x[1], 0.0)

    def test_plan_arc_passes_single_track(self) -> None:
        plan = plan_arc_passes(200.0, 100.0, 100.0)
        self.assertTrue(plan.single_pass)
        self.assertEqual(plan.pass_count, 1)
        self.assertAlmostEqual(plan.radii_mm[0], 250.0, places=3)
        self.assertAlmostEqual(plan.overlap_mm, 0.0, places=6)

    def test_plan_arc_passes_multiple_tracks(self) -> None:
        plan = plan_arc_passes(200.0, 100.0, 50.0)
        self.assertFalse(plan.single_pass)
        self.assertEqual(plan.pass_count, 3)
        self.assertGreater(plan.overlap_mm, 0.0)
        self.assertEqual(len(plan.radii_mm), 3)

    def test_build_ring_arc_program_start_finish_match_first_last_move(self) -> None:
        geometry = ring_arc_geometry_from_trajectory(self.trajectory)
        plan = plan_arc_passes(200.0, 100.0, 100.0)
        program = build_ring_arc_program(
            self.settings,
            geometry=geometry,
            pass_plan=plan,
        )
        first_start, _, _ = build_cw_semicircle_poses(
            geometry_with_track_radius(geometry, plan.radii_mm[0]),
            180.0,
            90.0,
            0.0,
        )
        _, _, last_end = build_ccw_semicircle_poses(
            geometry_with_track_radius(geometry, plan.radii_mm[-1]),
            180.0,
            270.0,
            0.0,
        )
        start_m = tuple(value / 1000.0 for value in first_start.position)
        finish_m = tuple(value / 1000.0 for value in last_end.position)
        self.assertIn(
            f"pose_start = [{start_m[0]:.4f}, {start_m[1]:.4f}, {start_m[2]:.4f}",
            program,
        )
        self.assertIn(
            f"pose_finish = [{finish_m[0]:.4f}, {finish_m[1]:.4f}, {finish_m[2]:.4f}",
            program,
        )
        self.assertIn("pose_track_0_cw_180 = [", program)
        start_line = next(line for line in program.splitlines() if line.startswith(" pose_start"))
        first_work_line = next(
            line for line in program.splitlines() if line.startswith(" pose_track_0_cw_180")
        )
        finish_line = next(line for line in program.splitlines() if line.startswith(" pose_finish"))
        last_track = plan.pass_count - 1
        last_work_line = next(
            line
            for line in program.splitlines()
            if line.startswith(f" pose_track_{last_track}_ccw_end_0")
        )
        self.assertEqual(start_line.split(" = ", 1)[1], first_work_line.split(" = ", 1)[1])
        self.assertEqual(finish_line.split(" = ", 1)[1], last_work_line.split(" = ", 1)[1])

    def test_build_ring_arc_program_two_movec_per_track(self) -> None:
        plan = plan_arc_passes(200.0, 100.0, 50.0)
        program = build_ring_arc_program_from_trajectory(
            self.trajectory, self.settings, pass_plan=plan
        )
        movec_lines = [line for line in program.splitlines() if "movec(" in line]
        self.assertEqual(len(movec_lines), 2 * plan.pass_count)
        for line in movec_lines:
            self.assertIn("mode=0", line)
            self.assertNotIn("t=", line)
        self.assertIn("RING_ARC: 3 pass(es)", program)
        self.assertIn("track_0_cw_via_90", program)
        self.assertIn("track_0_ccw_via_270", program)


if __name__ == "__main__":
    unittest.main()
