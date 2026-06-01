from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import TYPE_CHECKING, Callable

from src.common.frame_pose import FramePose6D
from src.common.geometry import Point3
from src.gui.parsing import format_number, parse_float

if TYPE_CHECKING:
    from src.gui.app import TrajectoryApp


class FormCommonSection:
    def __init__(self, parent: ttk.Frame, app: TrajectoryApp) -> None:
        self.app = app
        self.beam_width_var = tk.StringVar(value="100")
        self.fixture_x_var = tk.StringVar(value="0")
        self.fixture_y_var = tk.StringVar(value="0")
        self.fixture_z_var = tk.StringVar(value="0")
        self.fixture_rx_var = tk.StringVar(value="0")
        self.fixture_ry_var = tk.StringVar(value="0")
        self.fixture_rz_var = tk.StringVar(value="0")
        self.start_x_var = tk.StringVar(value="0")
        self.start_y_var = tk.StringVar(value="0")
        self.start_z_var = tk.StringVar(value="0")
        self.finish_x_var = tk.StringVar(value="0")
        self.finish_y_var = tk.StringVar(value="0")
        self.finish_z_var = tk.StringVar(value="0")
        self._build(parent)

    def _build(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Общие параметры формы", padding=8)
        frame.pack(fill=tk.X, pady=(0, 8))

        row = ttk.Frame(frame)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="Ширина луча (мм):").pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.beam_width_var, width=10).pack(side=tk.LEFT, padx=(8, 24))
        ttk.Label(row, text="(общая для всех поверхностей)").pack(side=tk.LEFT)

        fixture = ttk.LabelFrame(frame, text="Поза матрицы / базы (мм, °)", padding=6)
        fixture.pack(fill=tk.X, pady=(8, 0))
        pose_row = ttk.Frame(fixture)
        pose_row.pack(fill=tk.X, pady=2)
        self._inline(pose_row, "X:", self.fixture_x_var, 0)
        self._inline(pose_row, "Y:", self.fixture_y_var, 2)
        self._inline(pose_row, "Z:", self.fixture_z_var, 4)
        self._inline(pose_row, "Rx:", self.fixture_rx_var, 6)
        self._inline(pose_row, "Ry:", self.fixture_ry_var, 8)
        self._inline(pose_row, "Rz:", self.fixture_rz_var, 10)
        ttk.Label(
            fixture,
            text="Static XYZ (Elite): Rx→Ry→Rz вокруг фикс. осей. «Переместить» — перенос локальной траектории.",
            wraplength=760,
        ).pack(anchor=tk.W, pady=(4, 0))

        points = ttk.LabelFrame(frame, text="Старт / финиш (мм)", padding=6)
        points.pack(fill=tk.X, pady=(8, 0))

        start_row = ttk.Frame(points)
        start_row.pack(fill=tk.X, pady=2)
        self._inline(start_row, "Старт X:", self.start_x_var, 0)
        self._inline(start_row, "Y:", self.start_y_var, 2)
        self._inline(start_row, "Z:", self.start_z_var, 4)

        finish_row = ttk.Frame(points)
        finish_row.pack(fill=tk.X, pady=2)
        self._inline(finish_row, "Финиш X:", self.finish_x_var, 0)
        self._inline(finish_row, "Y:", self.finish_y_var, 2)
        self._inline(finish_row, "Z:", self.finish_z_var, 4)

    def _inline(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.StringVar,
        column: int,
        width: int = 8,
    ) -> None:
        ttk.Label(parent, text=label).grid(
            row=0, column=column, sticky=tk.W, padx=(0 if column == 0 else 8, 4)
        )
        ttk.Entry(parent, textvariable=variable, width=width).grid(
            row=0, column=column + 1, sticky=tk.W
        )

    def bind_updates(self, callback: Callable[[], None]) -> None:
        for variable in (
            self.beam_width_var,
            self.fixture_x_var,
            self.fixture_y_var,
            self.fixture_z_var,
            self.fixture_rx_var,
            self.fixture_ry_var,
            self.fixture_rz_var,
            self.start_x_var,
            self.start_y_var,
            self.start_z_var,
            self.finish_x_var,
            self.finish_y_var,
            self.finish_z_var,
        ):
            variable.trace_add("write", lambda *_args: callback())

    def beam_width_mm(self) -> float:
        return parse_float(self.beam_width_var.get(), "Ширина луча")

    def fixture_pose(self) -> FramePose6D:
        return FramePose6D(
            x_mm=parse_float(self.fixture_x_var.get(), "Поза X"),
            y_mm=parse_float(self.fixture_y_var.get(), "Поза Y"),
            z_mm=parse_float(self.fixture_z_var.get(), "Поза Z"),
            rx_deg=parse_float(self.fixture_rx_var.get(), "Поза Rx"),
            ry_deg=parse_float(self.fixture_ry_var.get(), "Поза Ry"),
            rz_deg=parse_float(self.fixture_rz_var.get(), "Поза Rz"),
        )

    def trajectory_points(self) -> tuple[Point3, Point3]:
        start = (
            parse_float(self.start_x_var.get(), "Старт X"),
            parse_float(self.start_y_var.get(), "Старт Y"),
            parse_float(self.start_z_var.get(), "Старт Z"),
        )
        finish = (
            parse_float(self.finish_x_var.get(), "Финиш X"),
            parse_float(self.finish_y_var.get(), "Финиш Y"),
            parse_float(self.finish_z_var.get(), "Финиш Z"),
        )
        return start, finish

    def load_from_project(self) -> None:
        project = self.app.project
        self.beam_width_var.set(format_number(project.beam_width_mm))
        self.fixture_x_var.set(format_number(project.fixture_x_mm))
        self.fixture_y_var.set(format_number(project.fixture_y_mm))
        self.fixture_z_var.set(format_number(project.fixture_z_mm))
        self.fixture_rx_var.set(format_number(project.fixture_rx_deg))
        self.fixture_ry_var.set(format_number(project.fixture_ry_deg))
        self.fixture_rz_var.set(format_number(project.fixture_rz_deg))
        self.start_x_var.set(format_number(project.start_x_mm))
        self.start_y_var.set(format_number(project.start_y_mm))
        self.start_z_var.set(format_number(project.start_z_mm))
        self.finish_x_var.set(format_number(project.finish_x_mm))
        self.finish_y_var.set(format_number(project.finish_y_mm))
        self.finish_z_var.set(format_number(project.finish_z_mm))

    def save_to_project(self) -> None:
        project = self.app.project
        project.beam_width_mm = self.beam_width_mm()
        pose = self.fixture_pose()
        project.fixture_x_mm = pose.x_mm
        project.fixture_y_mm = pose.y_mm
        project.fixture_z_mm = pose.z_mm
        project.fixture_rx_deg = pose.rx_deg
        project.fixture_ry_deg = pose.ry_deg
        project.fixture_rz_deg = pose.rz_deg
        start, finish = self.trajectory_points()
        project.start_x_mm, project.start_y_mm, project.start_z_mm = start
        project.finish_x_mm, project.finish_y_mm, project.finish_z_mm = finish

    def save_fixture_pose_to_project(self) -> None:
        pose = self.fixture_pose()
        project = self.app.project
        project.fixture_x_mm = pose.x_mm
        project.fixture_y_mm = pose.y_mm
        project.fixture_z_mm = pose.z_mm
        project.fixture_rx_deg = pose.rx_deg
        project.fixture_ry_deg = pose.ry_deg
        project.fixture_rz_deg = pose.rz_deg
