import math
import tempfile
import unittest
from pathlib import Path

from src.core.models import ProfileWaypoint, RingParams
from src.core.ring_calc import (
    angle_in_sector,
    angle_distance_to_positive_x,
    build_radial_pass,
    build_trajectory,
    build_work_pose,
    ccw_distance,
    overlap_mm,
    pass_direction_xy,
    recommended_sector_count,
)
from src.export.step_export import export_ring_trajectory


class RingCalcTests(unittest.TestCase):
    def test_reference_case_sector_count(self) -> None:
        outer_radius = 700.0
        beam_width = 100.0
        self.assertEqual(recommended_sector_count(outer_radius, beam_width), 44)

    def test_reference_case_overlap(self) -> None:
        outer_radius = 700.0
        inner_radius = 500.0
        beam_width = 100.0
        sector_count = 44

        overlap_outer = overlap_mm(outer_radius, beam_width, sector_count)
        overlap_inner = overlap_mm(inner_radius, beam_width, sector_count)

        self.assertGreater(overlap_outer, 0.0)
        self.assertLess(overlap_outer, 1.0)
        self.assertAlmostEqual(overlap_outer, 0.04, places=1)
        self.assertGreater(overlap_inner, overlap_outer)

    def test_angle_in_sector_wraps_through_zero(self) -> None:
        self.assertTrue(angle_in_sector(10.0, 315.0, 45.0))
        self.assertTrue(angle_in_sector(330.0, 315.0, 45.0))
        self.assertFalse(angle_in_sector(90.0, 315.0, 45.0))

    def test_first_pass_is_closest_to_positive_x_without_entry_sector(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
        )
        trajectory = build_trajectory(params)

        self.assertEqual(len(trajectory.passes), 4)
        self.assertEqual(
            angle_distance_to_positive_x(trajectory.passes[0].angle_deg),
            0.0,
        )
        self.assertAlmostEqual(trajectory.passes[0].start[0], 500.0)
        self.assertAlmostEqual(trajectory.passes[0].start[1], 0.0)

    def test_entry_sector_one_pass_per_ray(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
            entry_sector_start_deg=45.0,
            entry_sector_end_deg=135.0,
        )
        trajectory = build_trajectory(params)

        self.assertEqual(len(trajectory.passes), 4)
        angles = [pass_.angle_deg for pass_ in trajectory.passes]
        self.assertEqual(len(set(round(a, 6) for a in angles)), 4)

        first = trajectory.passes[0]
        self.assertAlmostEqual(first.angle_deg, 90.0)
        self.assertAlmostEqual(first.start[0], 0.0, places=0)
        self.assertAlmostEqual(first.start[1], 700.0)
        self.assertAlmostEqual(first.finish[0], 0.0, places=0)
        self.assertAlmostEqual(first.finish[1], 500.0)
        self.assertEqual(trajectory.poses[1].pose_type, "work_outer")
        self.assertEqual(trajectory.poses[2].pose_type, "work_inner")

        self.assertAlmostEqual(trajectory.poses[1].tool_axis_x[0], 0.0, places=4)
        self.assertAlmostEqual(trajectory.poses[1].tool_axis_x[1], -1.0, places=4)

        second = trajectory.passes[1]
        self.assertAlmostEqual(second.angle_deg, 180.0)
        self.assertAlmostEqual(second.start[0], -500.0, places=0)
        self.assertAlmostEqual(second.finish[0], -700.0, places=0)

    def test_entry_sector_last_params_no_overlap_at_start(self) -> None:
        params = RingParams(
            inner_radius_mm=200.0,
            ring_width_mm=100.0,
            beam_width_mm=100.0,
            sector_count=19,
            entry_sector_start_deg=135.0,
            entry_sector_end_deg=225.0,
        )
        trajectory = build_trajectory(params)
        step = 360.0 / 19

        self.assertEqual(len(trajectory.passes), 19)
        angles = [pass_.angle_deg % 360.0 for pass_ in trajectory.passes]
        self.assertEqual(len(set(round(a, 4) for a in angles)), 19)

        first_angle = trajectory.passes[0].angle_deg
        last_angle = trajectory.passes[-1].angle_deg
        gap = ccw_distance(last_angle, first_angle)
        self.assertAlmostEqual(gap, step, delta=0.01)

        for pass_ in trajectory.passes:
            in_sector = angle_in_sector(pass_.angle_deg, 135.0, 225.0)
            r0 = math.hypot(pass_.start[0], pass_.start[1])
            r1 = math.hypot(pass_.finish[0], pass_.finish[1])
            if in_sector:
                self.assertGreater(r0, r1)
            else:
                self.assertLess(r0, r1)

    def test_processing_order_and_travel_segments(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
        )
        start = (0.0, 0.0, 0.0)
        finish = (10.0, 20.0, 0.0)
        trajectory = build_trajectory(
            params,
            start_point_mm=start,
            finish_point_mm=finish,
        )

        self.assertEqual(len(trajectory.passes), 4)
        self.assertEqual(len(trajectory.travel_segments), 5)
        self.assertEqual(
            len(trajectory.poses),
            sum(len(pass_.stops) for pass_ in trajectory.passes) + 2,
        )
        self.assertEqual(trajectory.poses[0].tool_axis_z, (0.0, 0.0, -1.0))
        self.assertEqual(trajectory.poses[0].tool_axis_x, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(trajectory.poses[1].tool_axis_x[0], 1.0)
        self.assertAlmostEqual(trajectory.poses[1].tool_axis_x[1], 0.0)
        self.assertEqual(trajectory.poses[0].pose_type, "start")
        self.assertEqual(trajectory.poses[1].pose_type, "work_inner")
        self.assertEqual(trajectory.poses[2].pose_type, "work_outer")
        self.assertEqual(trajectory.travel_segments[0].segment_type, "approach")
        self.assertEqual(trajectory.travel_segments[0].start, start)
        self.assertEqual(trajectory.travel_segments[0].finish, trajectory.passes[0].start)
        self.assertEqual(trajectory.travel_segments[-1].segment_type, "departure")
        self.assertEqual(trajectory.travel_segments[-1].finish, finish)

        angle_step_rad = math.radians(90.0)
        second = trajectory.passes[1]
        self.assertAlmostEqual(second.start[0], 500.0 * math.cos(angle_step_rad))
        self.assertAlmostEqual(second.start[1], 500.0 * math.sin(angle_step_rad))

    def test_entry_sector_requires_both_angles(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
            entry_sector_start_deg=10.0,
            entry_sector_end_deg=None,
        )
        with self.assertRaises(ValueError):
            build_trajectory(params)

    def test_tilt_zero_keeps_vertical_z_on_work_poses(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
        )
        trajectory = build_trajectory(params)

        for pose in trajectory.poses:
            if pose.pose_type.startswith("work_"):
                self.assertAlmostEqual(pose.tool_axis_z[0], 0.0, places=4)
                self.assertAlmostEqual(pose.tool_axis_z[1], 0.0, places=4)
                self.assertAlmostEqual(pose.tool_axis_z[2], -1.0, places=4)

    def test_outer_tilt_forward_on_x_aligned_pass(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
            outer_tilt_deg=10.0,
        )
        trajectory = build_trajectory(params)
        outer_pose = trajectory.poses[2]

        self.assertEqual(outer_pose.pose_type, "work_outer")
        self.assertGreater(outer_pose.tool_axis_z[0], 0.0)
        self.assertAlmostEqual(
            outer_pose.tool_axis_z[2], -math.cos(math.radians(10.0)), places=4
        )

    def test_oi_pass_tool_axis_matches_io_at_same_angle(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
            inner_tilt_deg=10.0,
            outer_tilt_deg=-5.0,
            profile_point_1=ProfileWaypoint(30.0, -2.0, 7.0),
        )
        angle_deg = 90.0
        io_pass = build_radial_pass(0, angle_deg, params, outer_to_inner=False)
        oi_pass = build_radial_pass(0, angle_deg, params, outer_to_inner=True)
        io_axis = pass_direction_xy(io_pass.start, io_pass.finish)
        oi_axis = pass_direction_xy(oi_pass.start, oi_pass.finish)

        for io_stop, oi_stop in zip(io_pass.stops, reversed(oi_pass.stops)):
            self.assertEqual(io_stop.position, oi_stop.position)
            io_pose = build_work_pose(0, io_stop.pose_type, io_stop.position, io_axis, io_stop.tilt_deg)
            oi_pose = build_work_pose(0, oi_stop.pose_type, oi_stop.position, oi_axis, -oi_stop.tilt_deg)
            for component in range(3):
                self.assertAlmostEqual(
                    io_pose.tool_axis_z[component],
                    oi_pose.tool_axis_z[component],
                    places=4,
                )

    def test_start_finish_unaffected_by_tilt(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
            inner_tilt_deg=10.0,
            outer_tilt_deg=10.0,
        )
        trajectory = build_trajectory(params)

        self.assertEqual(trajectory.poses[0].tool_axis_z, (0.0, 0.0, -1.0))
        self.assertEqual(trajectory.poses[-1].tool_axis_z, (0.0, 0.0, -1.0))

    def test_profile_adds_four_stops_per_pass(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
            profile_point_1=ProfileWaypoint(30.0, 5.0, 3.0),
            profile_point_2=ProfileWaypoint(80.0, -2.0, -4.0),
        )
        trajectory = build_trajectory(params)

        self.assertEqual(len(trajectory.passes[0].stops), 4)
        self.assertEqual(len(trajectory.poses), 4 * 4 + 2)

    def test_profile_point_radius_and_z(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
            profile_point_1=ProfileWaypoint(30.0, 5.0, 0.0),
        )
        trajectory = build_trajectory(params)
        profile_pose = next(
            pose for pose in trajectory.poses if pose.pose_type == "work_profile_1"
        )

        radius = math.hypot(profile_pose.position[0], profile_pose.position[1])
        self.assertAlmostEqual(radius, 670.0)
        self.assertAlmostEqual(profile_pose.position[2], 5.0)

    def test_profile_io_ordering(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
            profile_point_1=ProfileWaypoint(30.0, 0.0, 0.0),
            profile_point_2=ProfileWaypoint(80.0, 0.0, 0.0),
        )
        io_pass = build_trajectory(params).passes[0]
        oi_pass = build_trajectory(
            RingParams(
                inner_radius_mm=500.0,
                ring_width_mm=200.0,
                beam_width_mm=100.0,
                sector_count=4,
                entry_sector_start_deg=45.0,
                entry_sector_end_deg=135.0,
                profile_point_1=ProfileWaypoint(30.0, 0.0, 0.0),
                profile_point_2=ProfileWaypoint(80.0, 0.0, 0.0),
            )
        ).passes[0]

        io_radii = [math.hypot(*stop.position[:2]) for stop in io_pass.stops]
        oi_radii = [math.hypot(*stop.position[:2]) for stop in oi_pass.stops]
        self.assertEqual(io_radii, list(reversed(oi_radii)))


class StepExportTests(unittest.TestCase):
    def test_export_contains_trajectory_colors(self) -> None:
        params = RingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
        )
        trajectory = build_trajectory(params)

        with tempfile.TemporaryDirectory() as temp_dir:
            filepath = Path(temp_dir) / "ring.step"
            export_ring_trajectory(trajectory, filepath)
            content = filepath.read_text(encoding="utf-8")

            self.assertTrue(filepath.exists())
            self.assertGreater(filepath.stat().st_size, 0)
            self.assertIn("STYLED_ITEM", content)
            self.assertIn("CURVE_STYLE", content)
            self.assertIn("ring_trajectory", content)
            if "Open CASCADE STEP processor" in content:
                self.assertIn("SI_UNIT(.MILLI.", content)
                self.assertNotIn("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION", content)
                self.assertIn("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION", content)
            else:
                self.assertIn("COLOUR_RGB", content)
                self.assertIn("PRESENTATION_STYLE_ASSIGNMENT", content)
                self.assertRegex(content, r"STYLED_ITEM\('color',\(#\d+\),#\d+\)")


if __name__ == "__main__":
    unittest.main()
