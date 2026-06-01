from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass
from typing import List, Tuple

from src.common.tool_orientation import tilt_tool_axis_radial
from src.generators.cylinder_wall.params import WallProfileWaypoint

Point2 = Tuple[float, float]


@dataclass(frozen=True)
class WallProfilePoint:
    radial_inward_mm: float
    z_down_mm: float
    tilt_deg: float
    label: str


class WallProfileCanvas(tk.Canvas):
    """Сечение стенки: X — к оси от стенки, Y — вниз от верха."""

    _PROFILE_COLOR = "#2563eb"
    _TILT_COLOR = "#9333ea"

    def __init__(self, master, **kwargs) -> None:
        super().__init__(
            master,
            width=260,
            height=200,
            bg="#fafafa",
            highlightthickness=1,
            highlightbackground="#cccccc",
            **kwargs,
        )

    def draw_profile(
        self,
        wall_height_mm: float,
        inner_radius_mm: float,
        top_tilt_deg: float,
        bottom_tilt_deg: float,
        profile_point_1: WallProfileWaypoint,
        profile_point_2: WallProfileWaypoint,
    ) -> None:
        self.delete("all")
        if wall_height_mm <= 0:
            self._draw_message("Укажите высоту стенки")
            return

        points = self._profile_points(
            wall_height_mm,
            top_tilt_deg,
            bottom_tilt_deg,
            profile_point_1,
            profile_point_2,
        )
        if len(points) < 2:
            self._draw_message("Нет точек профиля")
            return

        radial_values = [point.radial_inward_mm for point in points]
        radial_min = min(radial_values + [0.0])
        radial_max = max(radial_values + [0.0])
        radial_range = max(radial_max - radial_min, inner_radius_mm * 0.1, 20.0)
        radial_range *= 1.15

        width = int(self.winfo_width() or 260)
        height = int(self.winfo_height() or 200)
        margin_left, margin_right, margin_top, margin_bottom = 36, 16, 16, 32
        plot_w = max(1, width - margin_left - margin_right)
        plot_h = max(1, height - margin_top - margin_bottom)
        px_per_mm_x = plot_w / radial_range
        px_per_mm_y = plot_h / wall_height_mm

        def to_canvas(radial_inward_mm: float, z_down_mm: float) -> Point2:
            x = margin_left + ((radial_inward_mm - radial_min) / radial_range) * plot_w
            y = margin_top + (z_down_mm / wall_height_mm) * plot_h
            return x, y

        self._draw_axes(
            margin_left,
            margin_top,
            plot_w,
            plot_h,
            width,
            wall_height_mm,
            to_canvas(0.0, 0.0)[0],
        )

        canvas_points = [to_canvas(point.radial_inward_mm, point.z_down_mm) for point in points]

        for index in range(len(canvas_points) - 1):
            x0, y0 = canvas_points[index]
            x1, y1 = canvas_points[index + 1]
            self.create_line(x0, y0, x1, y1, fill=self._PROFILE_COLOR, width=2)

        for point, (cx, cy) in zip(points, canvas_points):
            self.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill=self._PROFILE_COLOR, outline="")
            self.create_text(
                cx + 8,
                cy - 8,
                text=f"{point.label}\n↓={point.z_down_mm:g}",
                anchor=tk.W,
                fill="#333333",
                font=("Segoe UI", 8),
            )
            self._draw_tilt_vector(
                point,
                inner_radius_mm,
                to_canvas,
                px_per_mm_x,
                px_per_mm_y,
            )

    def _draw_tilt_vector(
        self,
        point: WallProfilePoint,
        inner_radius_mm: float,
        to_canvas,
        px_per_mm_x: float,
        px_per_mm_y: float,
    ) -> None:
        position = (
            inner_radius_mm - point.radial_inward_mm,
            0.0,
            0.0,
        )
        axis_x, axis_y, axis_z = tilt_tool_axis_radial(position, point.tilt_deg)
        arrow_len_mm = 28.0

        tip_x, tip_y = to_canvas(point.radial_inward_mm, point.z_down_mm)
        # Ось Z смотрит к стенке (изнутри); на схеме хвост — со стороны оси (вправо).
        tail_x = tip_x + axis_x * arrow_len_mm * px_per_mm_x
        tail_y = tip_y + axis_z * arrow_len_mm * px_per_mm_y

        self.create_line(
            tail_x, tail_y, tip_x, tip_y, fill=self._TILT_COLOR, width=2, arrow=tk.LAST
        )

    def _draw_axes(
        self,
        margin_left: float,
        margin_top: float,
        plot_w: float,
        plot_h: float,
        width: int,
        wall_height_mm: float,
        wall_x: float,
    ) -> None:
        height = int(self.winfo_height() or 200)
        base_y = margin_top + plot_h
        self.create_line(margin_left, margin_top, margin_left, base_y, fill="#888888")
        self.create_line(margin_left, margin_top, margin_left + plot_w, margin_top, fill="#888888")
        self.create_line(margin_left, base_y, margin_left + plot_w, base_y, fill="#888888")
        self.create_line(wall_x, margin_top, wall_x, base_y, fill="#cccccc", dash=(4, 3))
        self.create_text(margin_left + plot_w / 2, height - 8, text="→ к оси", fill="#666666")
        self.create_text(12, margin_top + plot_h / 2, text="↓", fill="#666666", angle=90)
        self.create_text(
            margin_left,
            margin_top - 6,
            text="верх",
            anchor=tk.SW,
            fill="#666666",
            font=("Segoe UI", 8),
        )
        self.create_text(
            margin_left,
            base_y + 14,
            text=f"низ ({wall_height_mm:g} мм)",
            anchor=tk.N,
            fill="#666666",
            font=("Segoe UI", 8),
        )
        self.create_text(
            wall_x,
            margin_top - 6,
            text="стенка",
            anchor=tk.S,
            fill="#666666",
            font=("Segoe UI", 8),
        )
        if wall_x > margin_left + 8:
            self.create_text(
                margin_left + 4,
                margin_top - 6,
                text="наружу",
                anchor=tk.SW,
                fill="#666666",
                font=("Segoe UI", 8),
            )

    def _profile_points(
        self,
        wall_height_mm: float,
        top_tilt_deg: float,
        bottom_tilt_deg: float,
        profile_point_1: WallProfileWaypoint,
        profile_point_2: WallProfileWaypoint,
    ) -> List[WallProfilePoint]:
        points: List[WallProfilePoint] = [
            WallProfilePoint(0.0, 0.0, top_tilt_deg, "верх")
        ]

        for index, waypoint in enumerate((profile_point_1, profile_point_2), start=1):
            if waypoint.z_down_mm <= 0.0:
                continue
            if waypoint.z_down_mm >= wall_height_mm:
                continue
            points.append(
                WallProfilePoint(
                    waypoint.radial_inward_mm,
                    waypoint.z_down_mm,
                    waypoint.tilt_deg,
                    f"P{index}",
                )
            )

        points.append(WallProfilePoint(0.0, wall_height_mm, bottom_tilt_deg, "низ"))
        points.sort(key=lambda point: point.z_down_mm)
        return points

    def _draw_message(self, text: str) -> None:
        width = int(self.winfo_width() or 260)
        height = int(self.winfo_height() or 200)
        self.create_text(width / 2, height / 2, text=text, fill="#888888")
