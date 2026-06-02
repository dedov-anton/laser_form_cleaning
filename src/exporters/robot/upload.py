from __future__ import annotations

import socket
from pathlib import Path

DEFAULT_ROBOT_IP = "192.168.1.12"
DEFAULT_ROBOT_UPLOAD_PORT = 30001
DEFAULT_UPLOAD_TIMEOUT_S = 3.0


class RobotUploadError(OSError):
    pass


def prepare_script_for_upload(content: str) -> str:
    clean_lines = [
        line
        for line in content.splitlines()
        if not line.strip().startswith(("#block", "#end block"))
    ]
    return "\n".join(clean_lines)


def send_program_to_robot(
    content: str,
    ip: str,
    *,
    port: int = DEFAULT_ROBOT_UPLOAD_PORT,
    timeout: float = DEFAULT_UPLOAD_TIMEOUT_S,
) -> None:
    ip = ip.strip()
    if not ip:
        raise RobotUploadError("IP робота не задан")

    script = prepare_script_for_upload(content)
    if not script.strip():
        raise RobotUploadError("Файл УП пуст или не содержит команд")

    try:
        with socket.create_connection((ip, port), timeout=timeout) as connection:
            connection.sendall(script.encode("utf-8"))
    except OSError as error:
        raise RobotUploadError(f"Не удалось отправить УП на {ip}:{port}: {error}") from error


def send_program_file_to_robot(
    filepath: Path | str,
    ip: str,
    *,
    port: int = DEFAULT_ROBOT_UPLOAD_PORT,
    timeout: float = DEFAULT_UPLOAD_TIMEOUT_S,
) -> None:
    path = Path(filepath)
    if not path.is_file():
        raise RobotUploadError(f"Файл не найден: {path}")

    try:
        content = path.read_text(encoding="utf-8")
    except OSError as error:
        raise RobotUploadError(f"Не удалось прочитать файл {path}: {error}") from error

    send_program_to_robot(content, ip, port=port, timeout=timeout)
