import math
import tempfile
import unittest
from pathlib import Path

from src.common.geometry import Pose6D
from src.generators.bottom_ring.adapter import to_work_trajectory
from src.generators.bottom_ring.calc import build_trajectory
from src.generators.bottom_ring.params import BottomRingParams
from src.generators.cylinder_wall.adapter import to_work_trajectory as cylinder_to_work
from src.generators.cylinder_wall.calc import build_trajectory as build_cylinder
from src.generators.cylinder_wall.params import CylinderWallParams, WallProfileWaypoint
from src.storage.project_store import (
    load_trajectory,
    load_trajectory_file,
    save_project,
    save_trajectory_file,
    store_trajectory,
)
from src.storage.trajectory_codec import trajectory_from_dict, trajectory_to_dict
from src.transforms.wrist_correction import (
    VERTICAL_UPWARD,
    _dot,
    apply_wrist_correction,
    apply_wrist_correction_to_pose,
    cylinder_axis_xy,
    expected_wrist_corrected_tool_axis_z,
    inward_direction_to_axis,
    lcorr_perpendicular_up,
    tcp_position_after_lcorr,
    verify_radial_to_vertical_correction,
)
from src.transforms.wrist_correction_export import run_cylinder_wrist_correction


class WristCorrectionTests(unittest.TestCase):
    def _assert_corrected_z_matches_formula(
        self,
        original: Pose6D,
        corrected: Pose6D,
        axis_xy: tuple[float, float],
        places: int = 6,
    ) -> None:
        expected_z = expected_wrist_corrected_tool_axis_z(
            original.tool_axis_z, original.position, axis_xy
        )
        self.assertIsNotNone(expected_z)
        for index in range(3):
            self.assertAlmostEqual(corrected.tool_axis_z[index], expected_z[index], places=places)

    def test_tilt_zero_becomes_vertical_upward(self) -> None:
        pose = Pose6D(
            index=1,
            pose_type="top",
            position=(897.4334584121432, 239.15785766601547, 700.0),
            tool_axis_z=(0.9914448613738104, 0.13052619222005157, 0.0),
            tool_axis_x=(0.0, 0.0, -1.0),
        )
        corrected = apply_wrist_correction_to_pose(pose, axis_xy=(600.0, 200.0))
        for index in range(3):
            self.assertAlmostEqual(corrected.tool_axis_z[index], VERTICAL_UPWARD[index], places=6)
        inward = inward_direction_to_axis(pose.position, (600.0, 200.0))
        self.assertIsNotNone(inward)
        self.assertAlmostEqual(corrected.tool_axis_z[0], 0.0, places=3)
        self.assertAlmostEqual(corrected.tool_axis_z[1], 0.0, places=3)
        self.assertAlmostEqual(corrected.tool_axis_z[2], 1.0, places=3)

    def test_tilt_preserved_after_correction(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=300.0,
            beam_width_mm=50.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
            top_tilt_deg=10.0,
            bottom_tilt_deg=-8.0,
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        corrected = apply_wrist_correction(trajectory)
        axis_xy = cylinder_axis_xy(trajectory)

        top_before = next(pose for pose in trajectory.poses if pose.pose_type == "top")
        top_after = next(pose for pose in corrected.poses if pose.pose_type == "top")
        bottom_before = next(pose for pose in trajectory.poses if pose.pose_type == "bottom")
        bottom_after = next(pose for pose in corrected.poses if pose.pose_type == "bottom")

        self._assert_corrected_z_matches_formula(top_before, top_after, axis_xy)
        self._assert_corrected_z_matches_formula(bottom_before, bottom_after, axis_xy)
        self.assertFalse(
            all(
                math.isclose(top_after.tool_axis_z[index], VERTICAL_UPWARD[index], abs_tol=1e-3)
                for index in range(3)
            )
        )
        self.assertTrue(verify_radial_to_vertical_correction(trajectory))

    def test_profile_tilt_preserved(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=300.0,
            beam_width_mm=50.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
            top_tilt_deg=0.0,
            bottom_tilt_deg=0.0,
            profile_point_1=WallProfileWaypoint(80.0, 10.0, 3.0),
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        corrected = apply_wrist_correction(trajectory)
        axis_xy = cylinder_axis_xy(trajectory)

        profile_before = next(
            pose for pose in trajectory.poses if pose.pose_type == "work_profile_1"
        )
        profile_after = next(
            pose for pose in corrected.poses if pose.pose_type == "work_profile_1"
        )
        self._assert_corrected_z_matches_formula(profile_before, profile_after, axis_xy)
        self.assertFalse(
            all(
                math.isclose(
                    profile_after.tool_axis_z[index], VERTICAL_UPWARD[index], abs_tol=1e-3
                )
                for index in range(3)
            )
        )

    def test_profile_poses_in_corrected_trajectory(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=300.0,
            beam_width_mm=50.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
            profile_point_1=WallProfileWaypoint(50.0, 10.0, 3.0),
            profile_point_2=WallProfileWaypoint(150.0, 30.0, -4.0),
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        corrected = apply_wrist_correction(trajectory)
        cylinder = build_cylinder(params)

        profile_types = {
            pose.pose_type for pose in corrected.poses if pose.pose_type.startswith("work_profile_")
        }
        self.assertEqual(profile_types, {"work_profile_1", "work_profile_2"})
        self.assertEqual(len(cylinder.passes[0].stops), 4)
        self.assertEqual(
            sum(1 for pose in corrected.poses if pose.pose_type == "work_profile_1"),
            len(cylinder.passes),
        )

    def test_pose_position_is_preserved(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=300.0,
            beam_width_mm=50.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        corrected = apply_wrist_correction(trajectory, lcorr_mm=0.0)
        for before, after in zip(trajectory.poses, corrected.poses):
            self.assertEqual(before.position, after.position)

    def test_lcorr_shifts_position_inward_and_up(self) -> None:
        pose = Pose6D(
            index=1,
            pose_type="top",
            position=(897.4334584121432, 239.15785766601547, 700.0),
            tool_axis_z=(0.9914448613738104, 0.13052619222005157, 0.0),
            tool_axis_x=(0.0, 0.0, -1.0),
        )
        axis_xy = (600.0, 200.0)
        lcorr_mm = 50.0
        expected = tcp_position_after_lcorr(pose.position, pose.tool_axis_z, lcorr_mm)
        corrected = apply_wrist_correction_to_pose(pose, axis_xy=axis_xy, lcorr_mm=lcorr_mm)
        for index in range(3):
            self.assertAlmostEqual(corrected.position[index], expected[index], places=3)
        self.assertAlmostEqual(corrected.position[2], pose.position[2] + lcorr_mm, places=3)
        beam = (
            pose.tool_axis_z[0],
            pose.tool_axis_z[1],
            pose.tool_axis_z[2],
        )
        beam_len = math.hypot(*beam)
        beam = (beam[0] / beam_len, beam[1] / beam_len, beam[2] / beam_len)
        up = lcorr_perpendicular_up(beam)
        self.assertAlmostEqual(_dot(beam, up), 0.0, places=6)
        self.assertAlmostEqual(up[2], 1.0, places=6)

    def test_lcorr_tilt_top_bottom_different_xy(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=500.0,
            beam_width_mm=100.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
            top_tilt_deg=15.0,
            bottom_tilt_deg=-15.0,
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        corrected = apply_wrist_correction(trajectory, lcorr_mm=50.0)
        top = next(pose for pose in corrected.poses if pose.pose_type == "top")
        bottom = next(pose for pose in corrected.poses if pose.pose_type == "bottom")
        self.assertNotAlmostEqual(top.position[0], bottom.position[0], places=1)
        self.assertNotAlmostEqual(top.position[1], bottom.position[1], places=1)

    def test_lcorr_perpendicular_up_orthogonal_to_tilted_beam(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=500.0,
            beam_width_mm=100.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
            top_tilt_deg=15.0,
            bottom_tilt_deg=-15.0,
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        for pose in trajectory.poses:
            if pose.pose_type not in ("top", "bottom", "work_profile_1", "work_profile_2"):
                continue
            if pose.pose_type.startswith("work_profile_"):
                continue
            length = math.hypot(*pose.tool_axis_z)
            beam = tuple(component / length for component in pose.tool_axis_z)
            up = lcorr_perpendicular_up(beam)
            self.assertAlmostEqual(_dot(beam, up), 0.0, places=6)
            self.assertAlmostEqual(math.hypot(*up), 1.0, places=6)

    def test_lcorr_zero_preserves_position(self) -> None:
        pose = Pose6D(
            index=1,
            pose_type="top",
            position=(897.4334584121432, 239.15785766601547, 700.0),
            tool_axis_z=(0.9914448613738104, 0.13052619222005157, 0.0),
            tool_axis_x=(0.0, 0.0, -1.0),
        )
        corrected = apply_wrist_correction_to_pose(pose, axis_xy=(600.0, 200.0), lcorr_mm=0.0)
        self.assertEqual(corrected.position, pose.position)

    def test_lcorr_does_not_change_orientation_logic(self) -> None:
        pose = Pose6D(
            index=1,
            pose_type="top",
            position=(897.4334584121432, 239.15785766601547, 700.0),
            tool_axis_z=(0.9914448613738104, 0.13052619222005157, 0.0),
            tool_axis_x=(0.0, 0.0, -1.0),
        )
        without_lcorr = apply_wrist_correction_to_pose(pose, axis_xy=(600.0, 200.0))
        with_lcorr = apply_wrist_correction_to_pose(pose, axis_xy=(600.0, 200.0), lcorr_mm=50.0)
        self.assertEqual(without_lcorr.tool_axis_z, with_lcorr.tool_axis_z)
        self.assertEqual(without_lcorr.tool_axis_x, with_lcorr.tool_axis_x)
        for index in range(3):
            self.assertAlmostEqual(with_lcorr.tool_axis_z[index], VERTICAL_UPWARD[index], places=6)

    def test_lcorr_skips_start_and_finish(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=300.0,
            beam_width_mm=50.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        corrected = apply_wrist_correction(trajectory, lcorr_mm=50.0)
        for before, after in zip(trajectory.poses, corrected.poses):
            if before.pose_type in ("start", "finish"):
                self.assertEqual(before.position, after.position)

    def test_lcorr_metadata(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=300.0,
            beam_width_mm=50.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        corrected = apply_wrist_correction(trajectory, lcorr_mm=42.0)
        self.assertEqual(corrected.metadata.get("wrist_correction_lcorr_mm"), 42.0)

    def test_world_trajectory_uses_fixture_axis(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=300.0,
            beam_width_mm=50.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
        )
        local = cylinder_to_work(build_cylinder(params), params)
        world = local
        world.metadata = dict(local.metadata)
        world.metadata["coordinate_frame"] = "world"
        world.metadata["frame_pose_applied"] = {
            "x_mm": 600.0,
            "y_mm": 200.0,
            "z_mm": 400.0,
            "rx_deg": 0.0,
            "ry_deg": 0.0,
            "rz_deg": 0.0,
        }
        for pose in world.poses:
            if pose.pose_type == "top":
                pose.position = (897.4334584121432, 239.15785766601547, 700.0)
                pose.tool_axis_z = (0.9914448613738104, 0.13052619222005157, 0.0)
                pose.tool_axis_x = (0.0, 0.0, -1.0)
                break
        self.assertEqual(cylinder_axis_xy(world), (600.0, 200.0))
        self.assertTrue(verify_radial_to_vertical_correction(world))

    def test_json_roundtrip(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=300.0,
            beam_width_mm=50.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "cylinder_wall.json"
            save_trajectory_file(path, trajectory)
            loaded = load_trajectory_file(path)
            corrected = apply_wrist_correction(loaded)
            out_path = Path(temp_dir) / "cylinder_wall_wrist_corrected.json"
            save_trajectory_file(out_path, corrected)
            restored = trajectory_from_dict(trajectory_to_dict(load_trajectory_file(out_path)))
            self.assertTrue(restored.metadata.get("wrist_correction_applied"))
            self.assertEqual(
                restored.metadata.get("wrist_correction_mode"),
                "radial_to_vertical_upward",
            )
            self.assertEqual(len(restored.poses), len(corrected.poses))

    def test_pipeline_writes_json_step_and_up(self) -> None:
        params = CylinderWallParams(
            inner_radius_mm=300.0,
            wall_height_mm=200.0,
            z_top_mm=300.0,
            beam_width_mm=50.0,
            passes_per_sector=1,
            start_sector=0,
            clockwise=True,
        )
        trajectory = cylinder_to_work(build_cylinder(params), params)
        trajectory.metadata = dict(trajectory.metadata)
        trajectory.metadata["coordinate_frame"] = "world"
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            input_path = root / "config" / "trajectories" / "cylinder_wall.json"
            input_path.parent.mkdir(parents=True)
            save_trajectory_file(input_path, trajectory)
            outputs = run_cylinder_wrist_correction(root, input_path=input_path)
            self.assertTrue(outputs.json_path.exists())
            self.assertTrue(outputs.step_path.exists())
            self.assertTrue(outputs.up_path.exists())
            self.assertIn("def welding_program():", outputs.up_path.read_text(encoding="utf-8"))


class TrajectoryFileStorageTests(unittest.TestCase):
    def test_store_trajectory_writes_external_json(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            from src.common.project import FormProject

            project = FormProject()
            params = BottomRingParams(
                inner_radius_mm=500.0,
                ring_width_mm=200.0,
                beam_width_mm=100.0,
                sector_count=4,
            )
            work = to_work_trajectory(build_trajectory(params), params)
            store_trajectory(project, "bottom_ring", work, project_root=root)
            save_project(project, root)

            file_path = root / "config" / "trajectories" / "bottom_ring.json"
            self.assertTrue(file_path.exists())
            self.assertIsNone(project.trajectories["bottom_ring"])
            self.assertEqual(
                project.trajectory_files["bottom_ring"],
                "trajectories/bottom_ring.json",
            )

            loaded_project = __import__(
                "src.storage.project_store", fromlist=["load_project"]
            ).load_project(root)
            restored = load_trajectory(loaded_project, "bottom_ring", project_root=root)
            self.assertIsNotNone(restored)
            self.assertEqual(restored.generator_id, "bottom_ring")

    def test_migrate_embedded_trajectory_on_save(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            from src.common.project import FormProject

            params = BottomRingParams(
                inner_radius_mm=500.0,
                ring_width_mm=200.0,
                beam_width_mm=100.0,
                sector_count=4,
            )
            work = to_work_trajectory(build_trajectory(params), params)
            project = FormProject(
                trajectories={"bottom_ring": trajectory_to_dict(work), "top_ring": None, "cylinder_wall": None}
            )
            save_project(project, root)

            file_path = root / "config" / "trajectories" / "bottom_ring.json"
            self.assertTrue(file_path.exists())
            self.assertIsNone(project.trajectories["bottom_ring"])


if __name__ == "__main__":
    unittest.main()
