import math
import unittest

from src.common.tool_orientation import dot, radial_direction_xy
from src.generators.cylinder_wall.adapter import to_work_trajectory
from src.generators.cylinder_wall.calc import (
    build_trajectory,
    ordered_sector_indices,
    overlap_per_sector,
    recommended_passes_per_sector,
    sector_pass_angles,
)
from src.generators.cylinder_wall.params import SECTOR_COUNT, CylinderWallParams


class CylinderWallCalcTests(unittest.TestCase):
    def _params(self, **overrides) -> CylinderWallParams:
        defaults = {
            "inner_radius_mm": 400.0,
            "wall_height_mm": 200.0,
            "z_top_mm": 500.0,
            "beam_width_mm": 100.0,
            "passes_per_sector": 3,
            "start_sector": 0,
            "clockwise": True,
            "top_tilt_deg": 5.0,
            "bottom_tilt_deg": -3.0,
        }
        defaults.update(overrides)
        return CylinderWallParams(**defaults)

    def test_recommended_passes_per_sector(self) -> None:
        passes = recommended_passes_per_sector(400.0, 100.0)
        arc = 2 * math.pi * 400.0 / SECTOR_COUNT
        expected = max(1, math.floor(arc / 100.0) + 1)
        self.assertEqual(passes, expected)

    def test_overlap_per_sector(self) -> None:
        overlap = overlap_per_sector(400.0, 100.0, 3)
        arc = 2 * math.pi * 400.0 / SECTOR_COUNT
        self.assertAlmostEqual(overlap, 100.0 - arc / 3)

    def test_sector_pass_angles(self) -> None:
        angles = sector_pass_angles(1, 2)
        self.assertAlmostEqual(angles[0], 56.25)
        self.assertAlmostEqual(angles[1], 78.75)

    def test_ordered_sectors_clockwise(self) -> None:
        self.assertEqual(ordered_sector_indices(2, True), [2, 1, 0, 7, 6, 5, 4, 3])

    def test_ordered_sectors_counterclockwise(self) -> None:
        self.assertEqual(ordered_sector_indices(2, False), [2, 3, 4, 5, 6, 7, 0, 1])

    def test_build_trajectory_geometry(self) -> None:
        trajectory = build_trajectory(self._params())
        self.assertEqual(len(trajectory.passes), 3 * SECTOR_COUNT)

        first_pass = trajectory.passes[0]
        top = first_pass.start
        bottom = first_pass.finish
        self.assertAlmostEqual(top[2], 500.0)
        self.assertAlmostEqual(bottom[2], 300.0)
        radius = math.hypot(top[0], top[1])
        self.assertAlmostEqual(radius, 400.0)

    def test_tool_axis_points_toward_wall(self) -> None:
        trajectory = build_trajectory(self._params(top_tilt_deg=0.0, bottom_tilt_deg=0.0))
        work_pose = trajectory.poses[1]
        outward = radial_direction_xy(work_pose.position)
        self.assertGreater(dot(work_pose.tool_axis_z, outward), 0.0)

    def test_top_and_bottom_tilt_differ(self) -> None:
        trajectory = build_trajectory(self._params(top_tilt_deg=10.0, bottom_tilt_deg=-8.0))
        top_pose = trajectory.poses[1]
        bottom_pose = trajectory.poses[2]
        self.assertFalse(
            all(
                math.isclose(top_pose.tool_axis_z[index], bottom_pose.tool_axis_z[index], abs_tol=1e-6)
                for index in range(3)
            )
        )

    def test_travel_and_work_segments(self) -> None:
        trajectory = build_trajectory(
            self._params(passes_per_sector=2, start_sector=0, clockwise=False),
            start_point_mm=(0.0, 0.0, 0.0),
            finish_point_mm=(10.0, 10.0, 10.0),
        )
        self.assertEqual(len(trajectory.passes), 2 * SECTOR_COUNT)
        self.assertEqual(len(trajectory.travel_segments), len(trajectory.passes) + 1)
        self.assertEqual(len(trajectory.poses), 1 + 2 * len(trajectory.passes) + 1)

    def test_start_sector_changes_first_angle(self) -> None:
        first_ccw = build_trajectory(
            self._params(start_sector=0, clockwise=False, passes_per_sector=1)
        ).passes[0].angle_deg
        first_cw = build_trajectory(
            self._params(start_sector=1, clockwise=True, passes_per_sector=1)
        ).passes[0].angle_deg
        self.assertAlmostEqual(first_ccw, 22.5)
        self.assertAlmostEqual(first_cw, 67.5)

    def test_reference_rings_at_top_and_bottom(self) -> None:
        params = self._params(z_top_mm=500.0, wall_height_mm=200.0)
        work = to_work_trajectory(build_trajectory(params), params)
        circles = work.reference_geometry["circles"]
        self.assertEqual(len(circles), 2)
        z_values = sorted(float(circle["z_mm"]) for circle in circles)
        self.assertAlmostEqual(z_values[0], 300.0)
        self.assertAlmostEqual(z_values[1], 500.0)
        for circle in circles:
            self.assertAlmostEqual(float(circle["radius_mm"]), 400.0)
