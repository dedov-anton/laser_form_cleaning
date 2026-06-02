import re
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CALIBRATION_UP = PROJECT_ROOT / "УП" / "test_vertical_down.txt"

POSE_PATTERN = re.compile(
    r"pose_\w+\s*=\s*\["
    r"([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*"
    r"([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]"
)


class CalibrationUpTests(unittest.TestCase):
    def test_calibration_file_exists(self) -> None:
        self.assertTrue(CALIBRATION_UP.is_file(), f"Нет файла {CALIBRATION_UP}")

    def test_calibration_poses(self) -> None:
        content = CALIBRATION_UP.read_text(encoding="utf-8")
        poses = POSE_PATTERN.findall(content)
        self.assertEqual(len(poses), 2, "Ожидаются ровно 2 pose: start и end")

        start = tuple(float(value) for value in poses[0])
        end = tuple(float(value) for value in poses[1])

        self.assertAlmostEqual(start[0], 0.3000, places=4)
        self.assertAlmostEqual(start[1], 0.3000, places=4)
        self.assertAlmostEqual(start[2], 0.2000, places=4)
        self.assertAlmostEqual(start[3], 0.0000, places=4)
        self.assertAlmostEqual(start[4], 0.0000, places=4)
        self.assertAlmostEqual(start[5], 0.0000, places=4)

        self.assertAlmostEqual(end[0], 0.4000, places=4)
        self.assertAlmostEqual(end[1], 0.3000, places=4)
        self.assertAlmostEqual(end[2], 0.2000, places=4)
        self.assertAlmostEqual(end[3], 0.0000, places=4)
        self.assertAlmostEqual(end[4], 0.0000, places=4)
        self.assertAlmostEqual(end[5], 0.0000, places=4)

    def test_calibration_program_structure(self) -> None:
        content = CALIBRATION_UP.read_text(encoding="utf-8")
        self.assertIn("def welding_program():", content)
        self.assertIn("movel(pose_start", content)
        self.assertIn("movel(pose_end", content)
        self.assertIn("end", content)


if __name__ == "__main__":
    unittest.main()
