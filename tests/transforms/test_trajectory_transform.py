import math
import unittest

from src.common.frame_pose import FramePose6D, rotation_matrix_sxyz, transform_point, transform_vector
from src.common.geometry import Pose6D, TravelSegment
from src.common.trajectory import WorkSegment, WorkTrajectory
from src.generators.bottom_ring.adapter import to_work_trajectory
from src.generators.bottom_ring.calc import build_trajectory
from src.generators.bottom_ring.params import BottomRingParams
from src.transforms.trajectory_transform import transform_work_trajectory


def _minimal_trajectory() -> WorkTrajectory:
    params = BottomRingParams(
        inner_radius_mm=500.0,
        ring_width_mm=200.0,
        beam_width_mm=100.0,
        sector_count=4,
    )
    return to_work_trajectory(build_trajectory(params), params)


class FramePoseTests(unittest.TestCase):
    def test_translation_only(self) -> None:
        rotation = rotation_matrix_sxyz(0.0, 0.0, 0.0)
        result = transform_point((100.0, 0.0, 0.0), rotation, (50.0, 0.0, 0.0))
        self.assertAlmostEqual(result[0], 150.0)
        self.assertAlmostEqual(result[1], 0.0)
        self.assertAlmostEqual(result[2], 0.0)

    def test_rotation_90_deg_around_z(self) -> None:
        rotation = rotation_matrix_sxyz(0.0, 0.0, 90.0)
        result = transform_point((1.0, 0.0, 0.0), rotation, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(result[0], 0.0, places=4)
        self.assertAlmostEqual(result[1], 1.0, places=4)

    def test_vector_length_preserved(self) -> None:
        rotation = rotation_matrix_sxyz(10.0, -20.0, 30.0)
        vector = transform_vector((0.2, -0.3, -0.9), rotation)
        self.assertAlmostEqual(math.hypot(*vector), 1.0, places=6)


class TrajectoryTransformTests(unittest.TestCase):
    def test_identity_pose_unchanged(self) -> None:
        local = _minimal_trajectory()
        world = transform_work_trajectory(local, FramePose6D())
        self.assertAlmostEqual(world.poses[1].position[0], local.poses[1].position[0], places=4)
        self.assertAlmostEqual(world.poses[1].tool_axis_z[2], local.poses[1].tool_axis_z[2], places=4)

    def test_translation_updates_start_finish(self) -> None:
        local = _minimal_trajectory()
        pose = FramePose6D(x_mm=10.0, y_mm=20.0, z_mm=30.0)
        world = transform_work_trajectory(local, pose)
        self.assertAlmostEqual(world.start_point_mm[0], local.start_point_mm[0] + 10.0)
        self.assertAlmostEqual(world.finish_point_mm[1], local.finish_point_mm[1] + 20.0)
        self.assertEqual(world.metadata.get("coordinate_frame"), "world")

    def test_reapply_from_local_not_cumulative(self) -> None:
        local = WorkTrajectory(
            generator_id="test",
            generator_version="1.0",
            params_snapshot={},
            start_point_mm=(100.0, 0.0, 0.0),
            finish_point_mm=(100.0, 0.0, 0.0),
            poses=[
                Pose6D(
                    index=0,
                    pose_type="work_inner",
                    position=(100.0, 0.0, 0.0),
                    tool_axis_z=(0.0, 0.0, -1.0),
                    tool_axis_x=(1.0, 0.0, 0.0),
                )
            ],
            travel_segments=[
                TravelSegment(index=0, segment_type="approach", start=(0.0, 0.0, 0.0), finish=(100.0, 0.0, 0.0))
            ],
            work_segments=[
                WorkSegment(pass_index=0, start=(100.0, 0.0, 0.0), finish=(200.0, 0.0, 0.0), group="work")
            ],
            reference_geometry={"circles": [{"radius_mm": 300.0, "label": "reference_inner"}]},
            metadata={},
        )

        first = transform_work_trajectory(local, FramePose6D(x_mm=10.0))
        second = transform_work_trajectory(local, FramePose6D(x_mm=50.0))
        self.assertAlmostEqual(first.poses[0].position[0], 110.0)
        self.assertAlmostEqual(second.poses[0].position[0], 150.0)

    def test_reference_circles_become_polylines(self) -> None:
        local = _minimal_trajectory()
        world = transform_work_trajectory(local, FramePose6D(z_mm=5.0))
        self.assertNotIn("circles", world.reference_geometry)
        self.assertGreater(len(world.reference_geometry.get("polylines", [])), 0)


if __name__ == "__main__":
    unittest.main()
