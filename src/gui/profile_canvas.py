from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass
from typing import List, Tuple

from src.core.models import ProfileWaypoint
from src.core.ring_calc import tilt_tool_axis_z

Point2 = Tuple[float, float]


@dataclass(frozen=True)
class ProfilePoint:
    radial_mm: float
    z_mm: float
    tilt_deg: float
    label: str


class ProfileCanvas(tk.Canvas):
    """Поперечный профиль: X — от внутреннего радиуса к внешнему, Y — Z."""

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
        ring_width_mm: float,
        inner_tilt_deg: float,
        outer_tilt_deg: float,
        profile_point_1: ProfileWaypoint,
        profile_point_2: ProfileWaypoint,
    ) -> None:
        self.delete("all")
        if ring_width_mm <= 0:
            self._draw_message("Укажите ширину кольца")
            return

        points = self._profile_points(
            ring_width_mm,
            inner_tilt_deg,
            outer_tilt_deg,
            profile_point_1,
            profile_point_2,
        )
        if len(points) < 2:
            self._draw_message("Нет точек профиля")
            return

        z_values = [point.z_mm for point in points]
        z_min = min(z_values + [0.0])
        z_max = max(z_values + [0.0])
        z_pad = max(5.0, (z_max - z_min) * 0.2) if z_max != z_min else 10.0
        z_min -= z_pad
        z_max += z_pad

        width = int(self.winfo_width() or 260)
        height = int(self.winfo_height() or 200)
        margin_left, margin_right, margin_top, margin_bottom = 36, 16, 16, 32
        plot_w = max(1, width - margin_left - margin_right)
        plot_h = max(1, height - margin_top - margin_bottom)
        px_per_mm = plot_w / ring_width_mm

        def to_canvas(radial_mm: float, z_mm: float) -> Point2:
            x = margin_left + (radial_mm / ring_width_mm) * plot_w
            y = margin_top + (z_max - z_mm) / (z_max - z_min) * plot_h
            return x, y

        self._draw_axes(margin_left, margin_top, plot_w, plot_h, width)

        canvas_points = [to_canvas(point.radial_mm, point.z_mm) for point in points]

        for index in range(len(canvas_points) - 1):
            x0, y0 = canvas_points[index]
            x1, y1 = canvas_points[index + 1]
            self.create_line(x0, y0, x1, y1, fill=self._PROFILE_COLOR, width=2)

        for point, (cx, cy) in zip(points, canvas_points):
            self.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill=self._PROFILE_COLOR, outline="")
            self.create_text(
                cx + 8,
                cy - 8,
                text=f"{point.label}\nZ={point.z_mm:g}",
                anchor=tk.W,
                fill="#333333",
                font=("Segoe UI", 8),
            )
            self._draw_tilt_vector(
                point.radial_mm,
                point.z_mm,
                point.tilt_deg,
                to_canvas,
                ring_width_mm,
                px_per_mm,
            )

    def _draw_tilt_vector(
        self,
        radial_mm: float,
        z_mm: float,
        tilt_deg: float,
        to_canvas,
        ring_width_mm: float,
        px_per_mm: float,
    ) -> None:
        # Ось Z инструмента в плоскости R–Z (луч наружу = +R), как в ring_calc / STEP.
        axis_dr, _, axis_dz = tilt_tool_axis_z((1.0, 0.0, 0.0), tilt_deg)
        arrow_len_mm = min(ring_width_mm * 0.15, 28.0)

        tip_x, tip_y = to_canvas(radial_mm, z_mm)
        # Один масштаб для обеих осей — иначе наклон на экране «сплющивается» в вертикаль.
        vec_scale = px_per_mm
        tail_x = tip_x - axis_dr * arrow_len_mm * vec_scale
        tail_y = tip_y + axis_dz * arrow_len_mm * vec_scale

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
    ) -> None:
        height = int(self.winfo_height() or 200)
        base_y = margin_top + plot_h
        self.create_line(margin_left, base_y, margin_left + plot_w, base_y, fill="#888888")
        self.create_line(margin_left, margin_top, margin_left, base_y, fill="#888888")
        self.create_text(margin_left + plot_w / 2, height - 8, text="R →", fill="#666666")
        self.create_text(12, margin_top + plot_h / 2, text="Z", fill="#666666", angle=90)
        self.create_text(
            margin_left,
            base_y + 14,
            text="внутр.",
            anchor=tk.N,
            fill="#666666",
            font=("Segoe UI", 8),
        )
        self.create_text(
            margin_left + plot_w,
            base_y + 14,
            text="внеш.",
            anchor=tk.N,
            fill="#666666",
            font=("Segoe UI", 8),
        )

    def _profile_points(
        self,
        ring_width_mm: float,
        inner_tilt_deg: float,
        outer_tilt_deg: float,
        profile_point_1: ProfileWaypoint,
        profile_point_2: ProfileWaypoint,
    ) -> List[ProfilePoint]:
        points: List[ProfilePoint] = [
            ProfilePoint(0.0, 0.0, inner_tilt_deg, "внутр.")
        ]

        for index, waypoint in enumerate((profile_point_1, profile_point_2), start=1):
            if waypoint.distance_from_outer_mm <= 0:
                continue
            radial_mm = ring_width_mm - waypoint.distance_from_outer_mm
            if radial_mm <= 0 or radial_mm >= ring_width_mm:
                continue
            points.append(
                ProfilePoint(radial_mm, waypoint.z_offset_mm, waypoint.tilt_deg, f"P{index}")
            )

        points.append(ProfilePoint(ring_width_mm, 0.0, outer_tilt_deg, "внеш."))
        points.sort(key=lambda point: point.radial_mm)
        return points

    def _draw_message(self, text: str) -> None:
        width = int(self.winfo_width() or 260)
        height = int(self.winfo_height() or 200)
        self.create_text(width / 2, height / 2, text=text, fill="#888888")
