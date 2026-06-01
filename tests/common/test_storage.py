import json
import tempfile
import unittest
from pathlib import Path

from src.common.project import FormProject
from src.generators.bottom_ring.params import BottomRingParams
from src.generators.bottom_ring.adapter import to_work_trajectory
from src.generators.bottom_ring.calc import build_trajectory
from src.storage.project_store import load_project, save_project, store_trajectory
from src.storage.trajectory_codec import trajectory_from_dict, trajectory_to_dict


class TrajectoryCodecTests(unittest.TestCase):
    def test_roundtrip(self) -> None:
        params = BottomRingParams(
            inner_radius_mm=500.0,
            ring_width_mm=200.0,
            beam_width_mm=100.0,
            sector_count=4,
        )
        work = to_work_trajectory(build_trajectory(params), params)
        restored = trajectory_from_dict(trajectory_to_dict(work))

        self.assertEqual(restored.generator_id, work.generator_id)
        self.assertEqual(len(restored.poses), len(work.poses))
        self.assertEqual(restored.poses[0].position, work.poses[0].position)


class ProjectStoreTests(unittest.TestCase):
    def test_save_and_load_project_with_trajectory(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            project = FormProject(
                beam_width_mm=100.0,
                bottom_ring={"inner_radius_mm": 500.0, "ring_width_mm": 200.0},
            )
            params = BottomRingParams(
                inner_radius_mm=500.0,
                ring_width_mm=200.0,
                beam_width_mm=100.0,
                sector_count=4,
            )
            store_trajectory(project, "bottom_ring", to_work_trajectory(build_trajectory(params), params))
            save_project(project, root)

            loaded = load_project(root)
            self.assertEqual(loaded.beam_width_mm, 100.0)
            self.assertIsNotNone(loaded.trajectories["bottom_ring"])

    def test_migrate_legacy_last_params(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            config = root / "config"
            config.mkdir()
            legacy = {
                "inner_radius_mm": 200.0,
                "ring_width_mm": 100.0,
                "beam_width_mm": 100.0,
                "sector_count": 19,
            }
            (config / "last_params.json").write_text(json.dumps(legacy), encoding="utf-8")

            project = load_project(root)
            self.assertEqual(project.bottom_ring.get("inner_radius_mm"), 200.0)
            self.assertEqual(project.beam_width_mm, 100.0)
