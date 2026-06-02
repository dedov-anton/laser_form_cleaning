from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

DEFAULT_BEAM_WIDTH_MM = 100.0
DEFAULT_POINT_MM = 0.0
DEFAULT_ROBOT_BLEND_RADIUS_MM = 0.001
DEFAULT_ROBOT_VELOCITY = 0.03
DEFAULT_ROBOT_ACCELERATION = 0.05
DEFAULT_ROBOT_IP = "192.168.1.12"
DEFAULT_TOOL_MOUNT_ROTATION_DEG = 0.0
PROJECT_VERSION = "1"


@dataclass
class FormProject:
    version: str = PROJECT_VERSION
    beam_width_mm: float = DEFAULT_BEAM_WIDTH_MM
    start_x_mm: float = DEFAULT_POINT_MM
    start_y_mm: float = DEFAULT_POINT_MM
    start_z_mm: float = DEFAULT_POINT_MM
    finish_x_mm: float = DEFAULT_POINT_MM
    finish_y_mm: float = DEFAULT_POINT_MM
    finish_z_mm: float = DEFAULT_POINT_MM
    fixture_x_mm: float = DEFAULT_POINT_MM
    fixture_y_mm: float = DEFAULT_POINT_MM
    fixture_z_mm: float = DEFAULT_POINT_MM
    fixture_rx_deg: float = DEFAULT_POINT_MM
    fixture_ry_deg: float = DEFAULT_POINT_MM
    fixture_rz_deg: float = DEFAULT_POINT_MM
    robot_blend_radius_mm: float = DEFAULT_ROBOT_BLEND_RADIUS_MM
    robot_velocity: float = DEFAULT_ROBOT_VELOCITY
    robot_acceleration: float = DEFAULT_ROBOT_ACCELERATION
    robot_ip: str = DEFAULT_ROBOT_IP
    tool_mount_rotation_deg: float = DEFAULT_TOOL_MOUNT_ROTATION_DEG
    bottom_ring: Dict[str, Any] = field(default_factory=dict)
    top_ring: Dict[str, Any] = field(default_factory=dict)
    cylinder_wall: Dict[str, Any] = field(default_factory=dict)
    trajectories: Dict[str, Optional[dict]] = field(
        default_factory=lambda: {
            "bottom_ring": None,
            "top_ring": None,
            "cylinder_wall": None,
        }
    )
    trajectories_local: Dict[str, Optional[dict]] = field(
        default_factory=lambda: {
            "bottom_ring": None,
            "top_ring": None,
            "cylinder_wall": None,
        }
    )
    trajectory_files: Dict[str, Optional[str]] = field(
        default_factory=lambda: {
            "bottom_ring": None,
            "top_ring": None,
            "cylinder_wall": None,
        }
    )
    trajectory_files_local: Dict[str, Optional[str]] = field(
        default_factory=lambda: {
            "bottom_ring": None,
            "top_ring": None,
            "cylinder_wall": None,
        }
    )
