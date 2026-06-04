from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import TYPE_CHECKING, Callable

from src.exporters.robot.export import export_ring_arc_program
from src.exporters.robot.ring_arc import arc_pass_summary_text, plan_arc_passes
from src.generators.top_ring.params import TopRingParams
from src.gui.parsing import format_number, parse_float, parse_positive_float
from src.gui.section_styles import SectionUI

if TYPE_CHECKING:
    from src.gui.app import TrajectoryApp


class TopRingSection:
    generator_id = "top_ring"

    def __init__(self, parent: ttk.Frame, app: TrajectoryApp) -> None:
        self.app = app
        self.inner_radius_var = tk.StringVar()
        self.ring_width_var = tk.StringVar()
        self.outer_radius_var = tk.StringVar(value="—")
        self.z_to_top_var = tk.StringVar()
        self.arc_pass_var = tk.StringVar(value="Дуги: —")
        self.start_x_var = tk.StringVar(value="0")
        self.start_y_var = tk.StringVar(value="0")
        self.start_z_var = tk.StringVar(value="0")
        self.finish_x_var = tk.StringVar(value="0")
        self.finish_y_var = tk.StringVar(value="0")
        self.finish_z_var = tk.StringVar(value="0")
        self.ui = SectionUI("top_ring")
        self._build(parent)

    def _build(self, parent: ttk.Frame) -> None:
        ui = self.ui
        frame = ui.lf(parent, "Верх формы (кольцо)")
        frame.pack(fill=tk.X, pady=(0, 8))

        top_row = ui.fr(frame)
        top_row.pack(fill=tk.X, pady=(0, 8))
        top_row.columnconfigure(0, weight=1)
        top_row.columnconfigure(1, weight=1)

        ring_frame = ui.lf(top_row, "Параметры кольца")
        ring_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 4))
        self._add_labeled_entry(ring_frame, 0, "Внутренний радиус (мм):", self.inner_radius_var)
        self._add_labeled_entry(ring_frame, 1, "Ширина кольца (мм):", self.ring_width_var)
        self._add_readonly_row(ring_frame, 2, "Внешний радиус (мм):", self.outer_radius_var)
        self._add_labeled_entry(
            ring_frame, 3, "Расстояние до верха (мм):", self.z_to_top_var
        )

        clean_frame = ui.lf(top_row, "Параметры чистки (дуги)")
        clean_frame.grid(row=0, column=1, sticky=tk.NSEW, padx=(4, 0))
        self.arc_pass_label = ui.lb(clean_frame, textvariable=self.arc_pass_var)
        self.arc_pass_label.grid(row=0, column=0, sticky=tk.W, pady=2)

        points_frame = ui.lf(
            frame,
            "Старт / финиш (мм) — координаты УП, без переноса по 6D позе",
        )
        points_frame.pack(fill=tk.X, pady=(0, 8))
        start_row = ui.fr(points_frame)
        start_row.pack(fill=tk.X, pady=2)
        self._add_inline_field(start_row, "Старт X:", self.start_x_var, 0, width=8)
        self._add_inline_field(start_row, "Y:", self.start_y_var, 2, width=8)
        self._add_inline_field(start_row, "Z:", self.start_z_var, 4, width=8)
        finish_row = ui.fr(points_frame)
        finish_row.pack(fill=tk.X, pady=2)
        self._add_inline_field(finish_row, "Финиш X:", self.finish_x_var, 0, width=8)
        self._add_inline_field(finish_row, "Y:", self.finish_y_var, 2, width=8)
        self._add_inline_field(finish_row, "Z:", self.finish_z_var, 4, width=8)

        buttons = ui.fr(frame)
        buttons.pack(fill=tk.X, pady=(0, 4))
        self.export_robot_arc_button = ui.bn(
            buttons,
            text="Создать УП дугами",
            command=self._export_robot_arc,
            state=tk.DISABLED,
        )
        self.export_robot_arc_button.pack(side=tk.LEFT)

    def _add_inline_field(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.StringVar,
        column: int,
        width: int = 10,
    ) -> None:
        self.ui.lb(parent, text=label).grid(
            row=0, column=column, sticky=tk.W, padx=(0 if column == 0 else 8, 4)
        )
        self.ui.en(parent, textvariable=variable, width=width).grid(
            row=0, column=column + 1, sticky=tk.W
        )

    def _add_labeled_entry(
        self,
        parent: ttk.Frame,
        row: int,
        label: str,
        variable: tk.StringVar,
        entry_width: int = 12,
    ) -> None:
        self.ui.lb(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
        self.ui.en(parent, textvariable=variable, width=entry_width).grid(
            row=row, column=1, sticky=tk.W, padx=(8, 0), pady=2
        )

    def _add_readonly_row(
        self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar
    ) -> None:
        self.ui.lb(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
        self.ui.lb(parent, textvariable=variable).grid(
            row=row, column=1, sticky=tk.W, padx=(8, 0), pady=2
        )

    def bind_updates(self, callback: Callable[[], None]) -> None:
        for variable in (
            self.inner_radius_var,
            self.ring_width_var,
            self.z_to_top_var,
            self.start_x_var,
            self.start_y_var,
            self.start_z_var,
            self.finish_x_var,
            self.finish_y_var,
            self.finish_z_var,
        ):
            variable.trace_add("write", lambda *_args: callback())

    def load_from_project(self) -> None:
        data = self.app.project.top_ring
        if data.get("inner_radius_mm") is not None:
            self.inner_radius_var.set(format_number(float(data["inner_radius_mm"])))
        if data.get("ring_width_mm") is not None:
            self.ring_width_var.set(format_number(float(data["ring_width_mm"])))
        if data.get("z_to_top_mm") is not None:
            self.z_to_top_var.set(format_number(float(data["z_to_top_mm"])))

        self.start_x_var.set(format_number(float(data.get("start_x_mm", self.app.project.start_x_mm))))
        self.start_y_var.set(format_number(float(data.get("start_y_mm", self.app.project.start_y_mm))))
        self.start_z_var.set(format_number(float(data.get("start_z_mm", self.app.project.start_z_mm))))
        self.finish_x_var.set(format_number(float(data.get("finish_x_mm", self.app.project.finish_x_mm))))
        self.finish_y_var.set(format_number(float(data.get("finish_y_mm", self.app.project.finish_y_mm))))
        self.finish_z_var.set(format_number(float(data.get("finish_z_mm", self.app.project.finish_z_mm)))
        )
        self._update_derived_fields()

    def save_params_to_project(self) -> None:
        params = self._read_params()
        data = params.to_dict()
        data.update(self._read_trajectory_points_dict())
        self.app.project.top_ring = data

    def _read_trajectory_points(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
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

    def _read_trajectory_points_dict(self) -> dict[str, float]:
        start, finish = self._read_trajectory_points()
        return {
            "start_x_mm": start[0],
            "start_y_mm": start[1],
            "start_z_mm": start[2],
            "finish_x_mm": finish[0],
            "finish_y_mm": finish[1],
            "finish_z_mm": finish[2],
        }

    def _read_params(self) -> TopRingParams:
        inner_radius = parse_positive_float(self.inner_radius_var.get(), "Внутренний радиус")
        ring_width = parse_positive_float(self.ring_width_var.get(), "Ширина кольца")
        z_to_top = parse_float(self.z_to_top_var.get(), "Расстояние до верха")
        beam_width = self.app.form_common.beam_width_mm()
        return TopRingParams(
            inner_radius_mm=inner_radius,
            ring_width_mm=ring_width,
            z_to_top_mm=z_to_top,
            beam_width_mm=beam_width,
        )

    def _update_derived_fields(self) -> None:
        try:
            params = self._read_params()
        except ValueError:
            self.outer_radius_var.set("—")
            self.arc_pass_var.set("Дуги: —")
            self.arc_pass_label.configure(foreground="")
            self._update_arc_export_state()
            return

        self.outer_radius_var.set(format_number(params.outer_radius_mm))
        self._update_arc_pass_fields(params)
        self._update_arc_export_state()

    def _update_arc_pass_fields(self, params: TopRingParams) -> None:
        try:
            plan = plan_arc_passes(
                params.inner_radius_mm,
                params.ring_width_mm,
                params.beam_width_mm,
            )
        except ValueError:
            self.arc_pass_var.set("Дуги: —")
            self.arc_pass_label.configure(foreground="")
            self._update_arc_export_state()
            return

        self.arc_pass_var.set(arc_pass_summary_text(plan, params.beam_width_mm))
        if plan.single_pass:
            self.arc_pass_label.configure(foreground="green")
        else:
            self._set_arc_overlap_color(plan.overlap_mm, params.beam_width_mm)

    def _set_arc_overlap_color(self, overlap_mm: float, beam_width: float) -> None:
        if overlap_mm < 0:
            color = "red"
        elif overlap_mm < 0.1 * beam_width:
            color = "#b8860b"
        else:
            color = "green"
        self.arc_pass_label.configure(foreground=color)

    def _update_arc_export_state(self) -> None:
        try:
            params = self._read_params()
            plan_arc_passes(
                params.inner_radius_mm,
                params.ring_width_mm,
                params.beam_width_mm,
            )
        except ValueError:
            self.export_robot_arc_button.configure(state=tk.DISABLED)
            return
        self.export_robot_arc_button.configure(state=tk.NORMAL)

    def _export_robot_arc(self) -> None:
        filepath = filedialog.asksaveasfilename(
            title="Сохранить УП дугами — верх формы",
            defaultextension=".txt",
            initialfile="top_ring_arc.txt",
            filetypes=[("All files", "*.*")],
        )
        if not filepath:
            return

        try:
            params = self._read_params()
            plan = plan_arc_passes(
                params.inner_radius_mm,
                params.ring_width_mm,
                params.beam_width_mm,
            )
            frame_pose = self.app.form_common.fixture_pose()
            settings = self.app.form_common.robot_export_settings()
            export_ring_arc_program(
                frame_pose=frame_pose,
                inner_radius_mm=params.inner_radius_mm,
                ring_width_mm=params.ring_width_mm,
                beam_width_mm=params.beam_width_mm,
                filepath=Path(filepath),
                settings=settings,
                generator_id=self.generator_id,
                local_z_mm=params.z_to_top_mm,
            )
            self.app.form_common.save_fixture_pose_to_project()
            self.save_params_to_project()
            self.app.save_project()
        except Exception as error:
            messagebox.showerror("Создать УП дугами", str(error))
            return

        summary = arc_pass_summary_text(plan, params.beam_width_mm)
        messagebox.showinfo(
            "Создать УП дугами",
            f"Файл сохранён:\n{filepath}\n\n{summary}",
        )
