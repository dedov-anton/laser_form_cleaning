import socket
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from src.common.project import DEFAULT_ROBOT_IP, FormProject
from src.exporters.robot.upload import (
    DEFAULT_ROBOT_UPLOAD_PORT,
    RobotUploadError,
    prepare_script_for_upload,
    send_program_file_to_robot,
    send_program_to_robot,
)
from src.storage.project_store import load_project, save_project


class PrepareScriptForUploadTests(unittest.TestCase):
    def test_removes_block_markers(self) -> None:
        content = "\n".join(
            [
                "def welding_program():",
                "#block1 type0",
                " movel(pose_1)",
                "#end block1",
                "end",
            ]
        )
        cleaned = prepare_script_for_upload(content)
        self.assertIn("def welding_program():", cleaned)
        self.assertIn(" movel(pose_1)", cleaned)
        self.assertNotIn("#block", cleaned)
        self.assertNotIn("#end block", cleaned)

    def test_keeps_plain_program_unchanged(self) -> None:
        content = "def welding_program():\n movel(pose_1)\nend\n"
        self.assertEqual(prepare_script_for_upload(content), content.rstrip("\n"))


class SendProgramToRobotTests(unittest.TestCase):
    @patch("src.exporters.robot.upload.socket.create_connection")
    def test_sends_clean_script_to_robot(self, create_connection: MagicMock) -> None:
        connection = MagicMock()
        create_connection.return_value.__enter__.return_value = connection

        send_program_to_robot(
            "#block1 type0\n movel(pose_1)\n#end block1\n",
            "192.168.1.12",
        )

        create_connection.assert_called_once_with(
            ("192.168.1.12", DEFAULT_ROBOT_UPLOAD_PORT),
            timeout=3.0,
        )
        connection.sendall.assert_called_once_with(b" movel(pose_1)")

    @patch("src.exporters.robot.upload.socket.create_connection")
    def test_connection_error_raises_robot_upload_error(
        self, create_connection: MagicMock
    ) -> None:
        create_connection.side_effect = socket.timeout("timed out")

        with self.assertRaises(RobotUploadError) as context:
            send_program_to_robot("def welding_program():\nend\n", "192.168.1.12")

        self.assertIn("192.168.1.12", str(context.exception))

    def test_empty_ip_raises(self) -> None:
        with self.assertRaises(RobotUploadError):
            send_program_to_robot("def welding_program():\nend\n", "  ")

    def test_empty_script_raises(self) -> None:
        with self.assertRaises(RobotUploadError):
            send_program_to_robot("#block1 type0\n#end block1\n", "192.168.1.12")


class SendProgramFileToRobotTests(unittest.TestCase):
    @patch("src.exporters.robot.upload.send_program_to_robot")
    def test_reads_file_and_delegates(self, send_program: MagicMock) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            filepath = Path(temp_dir) / "program.txt"
            filepath.write_text("def welding_program():\nend\n", encoding="utf-8")

            send_program_file_to_robot(filepath, "10.0.0.5")

        send_program.assert_called_once_with(
            "def welding_program():\nend\n",
            "10.0.0.5",
            port=DEFAULT_ROBOT_UPLOAD_PORT,
            timeout=3.0,
        )

    def test_missing_file_raises(self) -> None:
        with self.assertRaises(RobotUploadError):
            send_program_file_to_robot(Path("missing_program.txt"), "192.168.1.12")


class RobotIpPersistenceTests(unittest.TestCase):
    def test_robot_ip_saved_in_project(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            project = FormProject(robot_ip="192.168.1.99")
            save_project(project, root)
            loaded = load_project(root)
            self.assertEqual(loaded.robot_ip, "192.168.1.99")

    def test_robot_ip_defaults_when_missing(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            config = root / "config"
            config.mkdir()
            (config / "project.json").write_text('{"beam_width_mm": 100.0}', encoding="utf-8")
            loaded = load_project(root)
            self.assertEqual(loaded.robot_ip, DEFAULT_ROBOT_IP)


if __name__ == "__main__":
    unittest.main()
