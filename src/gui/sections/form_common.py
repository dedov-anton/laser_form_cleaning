from __future__ import annotations

import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import TYPE_CHECKING, Callable

from src.common.frame_pose import FramePose6D
from src.get_pose import get6dpose
from src.common.project import DEFAULT_ROBOT_IP, DEFAULT_TOOL_MOUNT_ROTATION_DEG
from src.exporters.robot.settings import RobotExportSettings
from src.exporters.robot.upload import send_program_file_to_robot
from src.gui.parsing import format_number, parse_float
from src.gui.section_styles import SectionUI

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
        self.robot_blend_var = tk.StringVar(value="0.001")
        self.robot_velocity_var = tk.StringVar(value="0.03")
        self.robot_acceleration_var = tk.StringVar(value="0.05")
        self.robot_ip_var = tk.StringVar(value=DEFAULT_ROBOT_IP)
        self.tool_mount_rotation_var = tk.StringVar(value=str(DEFAULT_TOOL_MOUNT_ROTATION_DEG))
        self.ui = SectionUI("form_common")
        self._build(parent)

    def _build(self, parent: ttk.Frame) -> None:
        ui = self.ui
        frame = ui.lf(parent, "Общие параметры формы")
        frame.pack(fill=tk.X, pady=(0, 8))

        row = ui.fr(frame)
        row.pack(fill=tk.X, pady=2)
        ui.lb(row, text="Ширина луча (мм):").pack(side=tk.LEFT)
        ui.en(row, textvariable=self.beam_width_var, width=10).pack(side=tk.LEFT, padx=(8, 24))
        ui.lb(row, text="(общая для всех поверхностей)").pack(side=tk.LEFT)

        robot = ui.lf(frame, "Параметры УП (Elite)", padding=6)
        robot.pack(fill=tk.X, pady=(8, 0))
        robot_row = ui.fr(robot)
        robot_row.pack(fill=tk.X, pady=2)
        self._inline(robot_row, "Blend (м):", self.robot_blend_var, 0, width=8)
        self._inline(robot_row, "v (м/с):", self.robot_velocity_var, 2, width=8)
        self._inline(robot_row, "a (м/с²):", self.robot_acceleration_var, 4, width=8)
        self._inline(robot_row, "Крепление (°):", self.tool_mount_rotation_var, 6, width=8)

        robot_send_row = ui.fr(robot)
        robot_send_row.pack(fill=tk.X, pady=(6, 2))
        self._inline(robot_send_row, "IP робота:", self.robot_ip_var, 0, width=14)
        ui.bn(
            robot_send_row,
            text="Отправить в робот УП",
            command=self._send_program_to_robot,
        ).grid(row=0, column=2, sticky=tk.W, padx=(16, 0))

        fixture = ui.lf(frame, "Поза матрицы / базы (мм, °)", padding=6)
        fixture.pack(fill=tk.X, pady=(8, 0))
        pose_row = ui.fr(fixture)
        pose_row.pack(fill=tk.X, pady=2)
        self._inline(pose_row, "X:", self.fixture_x_var, 0)
        self._inline(pose_row, "Y:", self.fixture_y_var, 2)
        self._inline(pose_row, "Z:", self.fixture_z_var, 4)
        self._inline(pose_row, "Rx:", self.fixture_rx_var, 6)
        self._inline(pose_row, "Ry:", self.fixture_ry_var, 8)
        self._inline(pose_row, "Rz:", self.fixture_rz_var, 10)
        ui.bn(
            pose_row,
            text="Получить позу",
            command=self._apply_pose_from_device,
        ).grid(row=0, column=12, sticky=tk.W, padx=(12, 0))
        ui.lb(
            fixture,
            text="Static XYZ (Elite): Rx→Ry→Rz вокруг фикс. осей. «Переместить» — перенос локальной траектории.",
            wraplength=760,
        ).pack(anchor=tk.W, pady=(4, 0))

    def _inline(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.StringVar,
        column: int,
        width: int = 8,
    ) -> None:
        self.ui.lb(parent, text=label).grid(
            row=0, column=column, sticky=tk.W, padx=(0 if column == 0 else 8, 4)
        )
        self.ui.en(parent, textvariable=variable, width=width).grid(
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
            self.robot_blend_var,
            self.robot_velocity_var,
            self.robot_acceleration_var,
            self.robot_ip_var,
            self.tool_mount_rotation_var,
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

    def robot_export_settings(self) -> RobotExportSettings:
        return RobotExportSettings(
            blend_radius_mm=parse_float(self.robot_blend_var.get(), "Радиус скругления"),
            velocity=parse_float(self.robot_velocity_var.get(), "Скорость v"),
            acceleration=parse_float(self.robot_acceleration_var.get(), "Ускорение a"),
            tool_mount_rotation_deg=parse_float(
                self.tool_mount_rotation_var.get(), "Поворот крепления"
            ),
        )

    def robot_ip(self) -> str:
        return self.robot_ip_var.get().strip()

    def _apply_pose_from_device(self) -> None:
        try:
            x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = get6dpose()
            self.fixture_x_var.set(format_number(x_mm))
            self.fixture_y_var.set(format_number(y_mm))
            self.fixture_z_var.set(format_number(z_mm))
            self.fixture_rx_var.set(format_number(rx_deg))
            self.fixture_ry_var.set(format_number(ry_deg))
            self.fixture_rz_var.set(format_number(rz_deg))
        except Exception as error:
            messagebox.showerror("Получить позу", str(error))

    def _send_program_to_robot(self) -> None:
        filepath = filedialog.askopenfilename(
            title="Отправить УП на робота",
            filetypes=[("УП Elite", "*.txt"), ("All files", "*.*")],
        )
        if not filepath:
            return

        ip = self.robot_ip()
        try:
            self.save_to_project()
            self.app.save_project()
            send_program_file_to_robot(filepath, ip)
        except Exception as error:
            messagebox.showerror("Отправка УП", str(error))
            return

        messagebox.showinfo("Отправка УП", f"УП отправлена на робота {ip}:\n{filepath}")

    def load_from_project(self) -> None:
        project = self.app.project
        self.beam_width_var.set(format_number(project.beam_width_mm))
        self.fixture_x_var.set(format_number(project.fixture_x_mm))
        self.fixture_y_var.set(format_number(project.fixture_y_mm))
        self.fixture_z_var.set(format_number(project.fixture_z_mm))
        self.fixture_rx_var.set(format_number(project.fixture_rx_deg))
        self.fixture_ry_var.set(format_number(project.fixture_ry_deg))
        self.fixture_rz_var.set(format_number(project.fixture_rz_deg))
        self.robot_blend_var.set(format_number(project.robot_blend_radius_mm))
        self.robot_velocity_var.set(format_number(project.robot_velocity))
        self.robot_acceleration_var.set(format_number(project.robot_acceleration))
        self.robot_ip_var.set(project.robot_ip)
        self.tool_mount_rotation_var.set(format_number(project.tool_mount_rotation_deg))

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
        robot = self.robot_export_settings()
        project.robot_blend_radius_mm = robot.blend_radius_mm
        project.robot_velocity = robot.velocity
        project.robot_acceleration = robot.acceleration
        project.robot_ip = self.robot_ip()
        project.tool_mount_rotation_deg = robot.tool_mount_rotation_deg

    def save_fixture_pose_to_project(self) -> None:
        pose = self.fixture_pose()
        project = self.app.project
        project.fixture_x_mm = pose.x_mm
        project.fixture_y_mm = pose.y_mm
        project.fixture_z_mm = pose.z_mm
        project.fixture_rx_deg = pose.rx_deg
        project.fixture_ry_deg = pose.ry_deg
        project.fixture_rz_deg = pose.rz_deg
