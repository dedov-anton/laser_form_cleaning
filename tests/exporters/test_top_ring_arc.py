import tempfile
import unittest
from pathlib import Path

from src.common.frame_pose import FramePose6D
from src.exporters.robot.export import export_ring_arc_program
from src.exporters.robot.ring_arc import (
    TOP_RING_TOOL_AXIS_Z_LOCAL,
    build_arc_pose,
    build_cw_semicircle_poses,
    build_ring_arc_geometry,
    plan_arc_passes,
)
from src.exporters.robot.settings import RobotExportSettings


class TopRingArcTests(unittest.TestCase):
    def test_build_ring_arc_geometry_local_z(self) -> None:
        pose = FramePose6D(x_mm=600.0, y_mm=500.0, z_mm=0.0)
        geometry = build_ring_arc_geometry(
            pose,
            200.0,
            100.0,
            100.0,
            generator_id="top_ring",
            local_z_mm=200.0,
        )
        self.assertAlmostEqual(geometry.local_z_mm, 200.0, places=6)
        self.assertAlmostEqual(geometry.z_mm, 200.0, places=3)
        start, _, _ = build_cw_semicircle_poses(geometry, 180.0, 90.0, 0.0)
        self.assertAlmostEqual(start.position[2], 200.0, places=2)

    def test_build_arc_pose_tool_axis_points_up(self) -> None:
        geometry = build_ring_arc_geometry(
            FramePose6D(),
            200.0,
            100.0,
            100.0,
            generator_id="top_ring",
            local_z_mm=150.0,
        )
        pose = build_arc_pose(geometry, 0.0, clockwise=True)
        self.assertAlmostEqual(pose.tool_axis_z[0], 0.0, places=5)
        self.assertAlmostEqual(pose.tool_axis_z[1], 0.0, places=5)
        self.assertAlmostEqual(pose.tool_axis_z[2], 1.0, places=5)

    def test_top_ring_tool_axis_local_constant(self) -> None:
        self.assertEqual(TOP_RING_TOOL_AXIS_Z_LOCAL, (0.0, 0.0, 1.0))

    def test_export_ring_arc_program_top_ring(self) -> None:
        settings = RobotExportSettings()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "top_ring_arc.txt"
            export_ring_arc_program(
                frame_pose=FramePose6D(x_mm=100.0, y_mm=200.0, z_mm=0.0),
                inner_radius_mm=200.0,
                ring_width_mm=100.0,
                beam_width_mm=100.0,
                filepath=path,
                settings=settings,
                generator_id="top_ring",
                local_z_mm=300.0,
            )
            text = path.read_text(encoding="utf-8")
        self.assertIn("# TRAJECTORY_LAYER: top_ring", text)
        self.assertIn("movec(", text)
        self.assertIn("# z_mm: 300.0000", text)
        plan = plan_arc_passes(200.0, 100.0, 100.0)
        movec_lines = [line for line in text.splitlines() if "movec(" in line]
        self.assertEqual(len(movec_lines), 2 * plan.pass_count)


if __name__ == "__main__":
    unittest.main()
