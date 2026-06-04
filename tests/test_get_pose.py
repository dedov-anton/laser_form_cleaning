import unittest

from src.get_pose import DEFAULT_POSE_6D_MM_DEG, get6dpose


class TestGetPose(unittest.TestCase):
    def test_get6dpose_returns_default_stub(self) -> None:
        self.assertEqual(get6dpose(), DEFAULT_POSE_6D_MM_DEG)
        self.assertEqual(get6dpose(), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
