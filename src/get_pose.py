from __future__ import annotations

DEFAULT_POSE_6D_MM_DEG = (600.0, 600.0, 155.0, 0.0, 0.0, 0.0)


def get6dpose() -> tuple[float, float, float, float, float, float]:
    """Фасад: поза с внешнего устройства/скрипта (x, y, z мм; rx, ry, rz °)."""


    return DEFAULT_POSE_6D_MM_DEG
