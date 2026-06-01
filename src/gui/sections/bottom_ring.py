from __future__ import annotations

import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import TYPE_CHECKING, Callable, Optional

from src.exporters.robot.export import RobotExportNotImplementedError, export_robot_program
from src.exporters.step.export import export_trajectory
from src.generators.bottom_ring.params import BottomRingParams, ProfileWaypoint
from src.generators.bottom_ring.calc import (
    overlap_mm,
    overlap_percent,
    recommended_sector_count,
)
from src.generators.registry import get_generator
from src.gui.parsing import (
    format_number,
    parse_float,
    parse_optional_angle,
    parse_optional_float,
    parse_optional_non_negative_float,
    parse_positive_float,
    parse_positive_int,
    parse_tilt_deg,
)
from src.gui.widgets.profile_canvas import ProfileCanvas
from src.storage.project_store import store_local_trajectory, store_trajectory
from src.transforms.trajectory_transform import transform_work_trajectory

if TYPE_CHECKING:
    from src.gui.app import TrajectoryApp


class BottomRingSection:
    generator_id = "bottom_ring"

    def __init__(self, parent: ttk.Frame, app: TrajectoryApp) -> None:
        self.app = app
        self.inner_radius_var = tk.StringVar()
        self.ring_width_var = tk.StringVar()
        self.outer_radius_var = tk.StringVar(value="—")
        self.sector_count_var = tk.StringVar()
        self.overlap_outer_var = tk.StringVar(value="—")
        self.overlap_inner_var = tk.StringVar(value="—")
        self.status_var = tk.StringVar(value="Траектория: не создана")
        self.entry_sector_start_var = tk.StringVar()
        self.entry_sector_end_var = tk.StringVar()
        self.inner_tilt_var = tk.StringVar(value="0")
        self.outer_tilt_var = tk.StringVar(value="0")
        self.profile1_dist_var = tk.StringVar(value="0")
        self.profile1_z_var = tk.StringVar(value="0")
        self.profile1_tilt_var = tk.StringVar(value="0")
        self.profile2_dist_var = tk.StringVar(value="0")
        self.profile2_z_var = tk.StringVar(value="0")
        self.profile2_tilt_var = tk.StringVar(value="0")
        self.start_x_var = tk.StringVar(value="0")
        self.start_y_var = tk.StringVar(value="0")
        self.start_z_var = tk.StringVar(value="0")
        self.finish_x_var = tk.StringVar(value="0")
        self.finish_y_var = tk.StringVar(value="0")
        self.finish_z_var = tk.StringVar(value="0")
        self._build(parent)

    def _build(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Низ формы (кольцо)", padding=8)
        frame.pack(fill=tk.X, pady=(0, 8))

        top_row = ttk.Frame(frame)
        top_row.pack(fill=tk.X, pady=(0, 8))
        top_row.columnconfigure(0, weight=1)
        top_row.columnconfigure(1, weight=1)

        ring_frame = ttk.LabelFrame(top_row, text="Параметры кольца", padding=8)
        ring_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 4))
        self._add_labeled_entry(ring_frame, 0, "Внутренний радиус (мм):", self.inner_radius_var)
        self._add_labeled_entry(ring_frame, 1, "Ширина кольца (мм):", self.ring_width_var)
        self._add_readonly_row(ring_frame, 2, "Внешний радиус (мм):", self.outer_radius_var)

        clean_frame = ttk.LabelFrame(top_row, text="Параметры чистки", padding=8)
        clean_frame.grid(row=0, column=1, sticky=tk.NSEW, padx=(4, 0))
        sector_row = ttk.Frame(clean_frame)
        sector_row.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=4)
        ttk.Label(sector_row, text="Кол-во секторов:").pack(side=tk.LEFT)
        ttk.Entry(sector_row, textvariable=self.sector_count_var, width=10).pack(
            side=tk.LEFT, padx=(8, 4)
        )
        ttk.Button(sector_row, text="↻ авто", command=self._apply_recommended_sectors).pack(
            side=tk.LEFT
        )
        self.overlap_outer_label = ttk.Label(clean_frame, textvariable=self.overlap_outer_var)
        self.overlap_outer_label.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=2)
        ttk.Label(clean_frame, textvariable=self.overlap_inner_var).grid(
            row=2, column=0, columnspan=2, sticky=tk.W, pady=2
        )

        entry_frame = ttk.LabelFrame(frame, text="Сектор O→I (° 0–360)", padding=8)
        entry_frame.pack(fill=tk.X, pady=(0, 8))
        sector_fields = ttk.Frame(entry_frame)
        sector_fields.pack(fill=tk.X)
        ttk.Label(sector_fields, text="Начало:").pack(side=tk.LEFT)
        ttk.Entry(sector_fields, textvariable=self.entry_sector_start_var, width=10).pack(
            side=tk.LEFT, padx=(6, 16)
        )
        ttk.Label(sector_fields, text="Конец:").pack(side=tk.LEFT)
        ttk.Entry(sector_fields, textvariable=self.entry_sector_end_var, width=10).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        profile_frame = ttk.LabelFrame(frame, text="Профиль / наклон Z", padding=8)
        profile_frame.pack(fill=tk.X, pady=(0, 8))
        profile_frame.columnconfigure(0, weight=1)
        profile_frame.columnconfigure(1, weight=0)

        fields = ttk.Frame(profile_frame)
        fields.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 8))

        edges = ttk.Frame(fields)
        edges.pack(fill=tk.X, pady=(0, 6))
        self._add_inline_field(edges, "Внутр. наклон (°):", self.inner_tilt_var, 0)
        self._add_inline_field(edges, "Внеш. наклон (°):", self.outer_tilt_var, 2)

        mid_row = ttk.Frame(fields)
        mid_row.pack(fill=tk.X)
        mid_row.columnconfigure(0, weight=1)
        mid_row.columnconfigure(1, weight=1)
        self._build_midpoint_block(
            mid_row, 0, "Промеж. 1", self.profile1_dist_var, self.profile1_z_var, self.profile1_tilt_var
        )
        self._build_midpoint_block(
            mid_row, 1, "Промеж. 2", self.profile2_dist_var, self.profile2_z_var, self.profile2_tilt_var
        )
        ttk.Label(
            fields,
            text="+ — к внутр. радиусу, − — наружу. Dist=0 — точка выкл.",
            wraplength=420,
        ).pack(anchor=tk.W, pady=(6, 0))

        preview_frame = ttk.Frame(profile_frame)
        preview_frame.grid(row=0, column=1, sticky=tk.N)
        ttk.Label(preview_frame, text="Сечение профиля").pack(anchor=tk.W)
        self.profile_canvas = ProfileCanvas(preview_frame)
        self.profile_canvas.pack()
        self.profile_canvas.bind("<Configure>", lambda _e: self._update_profile_preview())

        points_frame = ttk.LabelFrame(frame, text="Старт / финиш (мм)", padding=8)
        points_frame.pack(fill=tk.X, pady=(0, 8))
        start_row = ttk.Frame(points_frame)
        start_row.pack(fill=tk.X, pady=2)
        self._add_inline_field(start_row, "Старт X:", self.start_x_var, 0, width=8)
        self._add_inline_field(start_row, "Y:", self.start_y_var, 2, width=8)
        self._add_inline_field(start_row, "Z:", self.start_z_var, 4, width=8)
        finish_row = ttk.Frame(points_frame)
        finish_row.pack(fill=tk.X, pady=2)
        self._add_inline_field(finish_row, "Финиш X:", self.finish_x_var, 0, width=8)
        self._add_inline_field(finish_row, "Y:", self.finish_y_var, 2, width=8)
        self._add_inline_field(finish_row, "Z:", self.finish_z_var, 4, width=8)

        buttons = ttk.Frame(frame)
        buttons.pack(fill=tk.X, pady=(0, 4))
        ttk.Button(buttons, text="Создать траекторию", command=self._create_trajectory).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        self.move_pose_button = ttk.Button(
            buttons,
            text="Переместить по 6D позе",
            command=self._move_by_fixture_pose,
            state=tk.DISABLED,
        )
        self.move_pose_button.pack(side=tk.LEFT, padx=(0, 8))
        self.export_step_button = ttk.Button(
            buttons, text="Экспорт STEP", command=self._export_step, state=tk.DISABLED
        )
        self.export_step_button.pack(side=tk.LEFT, padx=(0, 8))
        self.export_robot_button = ttk.Button(
            buttons, text="Создать УП", command=self._export_robot, state=tk.DISABLED
        )
        self.export_robot_button.pack(side=tk.LEFT)
        ttk.Label(frame, textvariable=self.status_var).pack(anchor=tk.W)

    def _build_midpoint_block(
        self,
        parent: ttk.Frame,
        column: int,
        title: str,
        dist_var: tk.StringVar,
        z_var: tk.StringVar,
        tilt_var: tk.StringVar,
    ) -> None:
        block = ttk.LabelFrame(parent, text=title, padding=6)
        block.grid(row=0, column=column, sticky=tk.NSEW, padx=(0 if column == 0 else 4, 0))
        self._add_labeled_entry(block, 0, "От внеш. (мм):", dist_var, entry_width=8)
        self._add_labeled_entry(block, 1, "Z (мм):", z_var, entry_width=8)
        self._add_labeled_entry(block, 2, "Наклон (°):", tilt_var, entry_width=8)

    def _add_inline_field(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.StringVar,
        column: int,
        width: int = 10,
    ) -> None:
        ttk.Label(parent, text=label).grid(
            row=0, column=column, sticky=tk.W, padx=(0 if column == 0 else 8, 4)
        )
        ttk.Entry(parent, textvariable=variable, width=width).grid(
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
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(parent, textvariable=variable, width=entry_width).grid(
            row=row, column=1, sticky=tk.W, padx=(8, 0), pady=2
        )

    def _add_readonly_row(
        self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Label(parent, textvariable=variable).grid(
            row=row, column=1, sticky=tk.W, padx=(8, 0), pady=2
        )

    def bind_updates(self, callback: Callable[[], None]) -> None:
        for variable in (
            self.inner_radius_var,
            self.ring_width_var,
            self.sector_count_var,
            self.inner_tilt_var,
            self.outer_tilt_var,
            self.profile1_dist_var,
            self.profile1_z_var,
            self.profile1_tilt_var,
            self.profile2_dist_var,
            self.profile2_z_var,
            self.profile2_tilt_var,
            self.start_x_var,
            self.start_y_var,
            self.start_z_var,
            self.finish_x_var,
            self.finish_y_var,
            self.finish_z_var,
        ):
            variable.trace_add("write", lambda *_args: callback())

    def load_from_project(self) -> None:
        data = self.app.project.bottom_ring
        if data.get("inner_radius_mm") is not None:
            self.inner_radius_var.set(format_number(float(data["inner_radius_mm"])))
        if data.get("ring_width_mm") is not None:
            self.ring_width_var.set(format_number(float(data["ring_width_mm"])))
        if data.get("entry_sector_start_deg") is not None:
            self.entry_sector_start_var.set(format_number(float(data["entry_sector_start_deg"])))
        if data.get("entry_sector_end_deg") is not None:
            self.entry_sector_end_var.set(format_number(float(data["entry_sector_end_deg"])))
        self.inner_tilt_var.set(format_number(float(data.get("inner_tilt_deg", 0.0))))
        self.outer_tilt_var.set(format_number(float(data.get("outer_tilt_deg", 0.0))))
        self._load_profile_waypoint(data.get("profile_point_1", {}), 1)
        self._load_profile_waypoint(data.get("profile_point_2", {}), 2)
        if data.get("sector_count") is not None:
            self.sector_count_var.set(str(int(data["sector_count"])))
        else:
            self._apply_recommended_sectors(silent=True)

        self.start_x_var.set(format_number(float(data.get("start_x_mm", self.app.project.start_x_mm))))
        self.start_y_var.set(format_number(float(data.get("start_y_mm", self.app.project.start_y_mm))))
        self.start_z_var.set(format_number(float(data.get("start_z_mm", self.app.project.start_z_mm))))
        self.finish_x_var.set(format_number(float(data.get("finish_x_mm", self.app.project.finish_x_mm))))
        self.finish_y_var.set(format_number(float(data.get("finish_y_mm", self.app.project.finish_y_mm))))
        self.finish_z_var.set(format_number(float(data.get("finish_z_mm", self.app.project.finish_z_mm))))

        trajectory = self.app.get_trajectory(self.generator_id)
        local = self.app.get_local_trajectory(self.generator_id)
        if trajectory is None:
            self._set_export_state(False)
            self.move_pose_button.configure(state=tk.DISABLED)
            self.status_var.set("Траектория: не создана")
        else:
            self._set_export_state(True)
            self.move_pose_button.configure(state=tk.NORMAL if local is not None else tk.DISABLED)
            suffix = " (мировая)" if trajectory.metadata.get("coordinate_frame") == "world" else " (локальная)"
            self.status_var.set(self._format_status(trajectory) + suffix)

        self._update_derived_fields()
        self._update_profile_preview()

    def save_params_to_project(self) -> None:
        params = self._read_params(require_sector_count=True)
        data = params.to_dict()
        data.update(self._read_trajectory_points_dict())
        self.app.project.bottom_ring = data

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

    def _load_profile_waypoint(self, data: dict, index: int) -> None:
        vars_ = (
            (self.profile1_dist_var, self.profile1_z_var, self.profile1_tilt_var)
            if index == 1
            else (self.profile2_dist_var, self.profile2_z_var, self.profile2_tilt_var)
        )
        vars_[0].set(format_number(float(data.get("distance_from_outer_mm", 0.0))))
        vars_[1].set(format_number(float(data.get("z_offset_mm", 0.0))))
        vars_[2].set(format_number(float(data.get("tilt_deg", 0.0))))

    def _read_profile_waypoint(self, index: int) -> ProfileWaypoint:
        prefix = f"Промеж. точка {index}"
        dist_var = self.profile1_dist_var if index == 1 else self.profile2_dist_var
        z_var = self.profile1_z_var if index == 1 else self.profile2_z_var
        tilt_var = self.profile1_tilt_var if index == 1 else self.profile2_tilt_var
        return ProfileWaypoint(
            distance_from_outer_mm=parse_optional_non_negative_float(
                dist_var.get(), f"{prefix}, от внеш. радиуса"
            ),
            z_offset_mm=parse_optional_float(z_var.get(), f"{prefix}, Z"),
            tilt_deg=parse_tilt_deg(tilt_var.get(), f"{prefix}, наклон"),
        )

    def _read_params(self, require_sector_count: bool) -> BottomRingParams:
        inner_radius = parse_positive_float(self.inner_radius_var.get(), "Внутренний радиус")
        ring_width = parse_positive_float(self.ring_width_var.get(), "Ширина кольца")
        beam_width = self.app.form_common.beam_width_mm()

        sector_text = self.sector_count_var.get().strip()
        if not sector_text:
            if require_sector_count:
                raise ValueError("Укажите количество секторов")
            sector_count = recommended_sector_count(inner_radius + ring_width, beam_width)
        else:
            sector_count = parse_positive_int(sector_text, "Кол-во секторов")

        return BottomRingParams(
            inner_radius_mm=inner_radius,
            ring_width_mm=ring_width,
            beam_width_mm=beam_width,
            sector_count=sector_count,
            entry_sector_start_deg=parse_optional_angle(
                self.entry_sector_start_var.get(), "Начало сектора O→I"
            ),
            entry_sector_end_deg=parse_optional_angle(
                self.entry_sector_end_var.get(), "Конец сектора O→I"
            ),
            inner_tilt_deg=parse_tilt_deg(
                self.inner_tilt_var.get(), "Наклон Z на внутр. радиусе"
            ),
            outer_tilt_deg=parse_tilt_deg(
                self.outer_tilt_var.get(), "Наклон Z на внеш. радиусе"
            ),
            profile_point_1=self._read_profile_waypoint(1),
            profile_point_2=self._read_profile_waypoint(2),
        )

    def _apply_recommended_sectors(self, silent: bool = False) -> None:
        try:
            params = self._read_params(require_sector_count=False)
        except ValueError as error:
            if not silent:
                messagebox.showerror("Ошибка", str(error))
            return

        recommended = recommended_sector_count(params.outer_radius_mm, params.beam_width_mm)
        self.sector_count_var.set(str(recommended))
        self._update_derived_fields()
        self._update_profile_preview()

    def _update_derived_fields(self) -> None:
        try:
            params = self._read_params(require_sector_count=False)
        except ValueError:
            self.outer_radius_var.set("—")
            self.overlap_outer_var.set("Перекрытие на внешнем: —")
            self.overlap_inner_var.set("Перекрытие на внутреннем: —")
            self.overlap_outer_label.configure(foreground="")
            return

        self.outer_radius_var.set(format_number(params.outer_radius_mm))

        if not self.sector_count_var.get().strip():
            return

        try:
            params_with_sectors = self._read_params(require_sector_count=True)
        except ValueError:
            self.overlap_outer_var.set("Перекрытие на внешнем: —")
            self.overlap_inner_var.set("Перекрытие на внутреннем: —")
            self.overlap_outer_label.configure(foreground="")
            return

        overlap_outer = overlap_mm(
            params_with_sectors.outer_radius_mm,
            params_with_sectors.beam_width_mm,
            params_with_sectors.sector_count,
        )
        overlap_inner = overlap_mm(
            params_with_sectors.inner_radius_mm,
            params_with_sectors.beam_width_mm,
            params_with_sectors.sector_count,
        )
        outer_pct = overlap_percent(overlap_outer, params_with_sectors.beam_width_mm)

        self.overlap_outer_var.set(
            f"Перекрытие на внешнем: {overlap_outer:.2f} мм ({outer_pct:.1f}%)"
        )
        self.overlap_inner_var.set(f"Перекрытие на внутреннем: {overlap_inner:.2f} мм")
        self._set_overlap_color(overlap_outer, params_with_sectors.beam_width_mm)

    def _set_overlap_color(self, overlap_outer: float, beam_width: float) -> None:
        if overlap_outer < 0:
            color = "red"
        elif overlap_outer < 0.1 * beam_width:
            color = "#b8860b"
        else:
            color = "green"
        self.overlap_outer_label.configure(foreground=color)

    def _update_profile_preview(self) -> None:
        try:
            ring_width = parse_positive_float(self.ring_width_var.get(), "Ширина кольца")
        except ValueError:
            self.profile_canvas.draw_profile(0.0, 0.0, 0.0, ProfileWaypoint(), ProfileWaypoint())
            return

        try:
            inner_tilt = parse_tilt_deg(self.inner_tilt_var.get(), "Наклон Z на внутр. радиусе")
            outer_tilt = parse_tilt_deg(self.outer_tilt_var.get(), "Наклон Z на внеш. радиусе")
            profile1 = self._read_profile_waypoint(1)
            profile2 = self._read_profile_waypoint(2)
        except ValueError:
            return

        self.profile_canvas.draw_profile(
            ring_width, inner_tilt, outer_tilt, profile1, profile2
        )

    def _create_trajectory(self) -> None:
        try:
            params = self._read_params(require_sector_count=True)
            start_point, finish_point = self._read_trajectory_points()
            trajectory = get_generator(self.generator_id).build(
                params,
                start_mm=start_point,
                finish_mm=finish_point,
            )
        except ValueError as error:
            messagebox.showerror("Ошибка", str(error))
            return
        except NotImplementedError as error:
            messagebox.showinfo("В разработке", str(error))
            return

        self.app.set_trajectory(self.generator_id, trajectory)
        self.app.set_local_trajectory(self.generator_id, trajectory)
        store_trajectory(self.app.project, self.generator_id, trajectory)
        store_local_trajectory(self.app.project, self.generator_id, trajectory)
        self.app.form_common.save_to_project()
        self.save_params_to_project()
        self.app.save_project()

        self.status_var.set(self._format_status(trajectory) + " (локальная)")
        self._set_export_state(True)
        self.move_pose_button.configure(state=tk.NORMAL)
        self._update_derived_fields()
        self._update_profile_preview()

    def _move_by_fixture_pose(self) -> None:
        local = self.app.get_local_trajectory(self.generator_id)
        if local is None:
            messagebox.showwarning("Перенос", "Сначала создайте траекторию.")
            return

        try:
            pose = self.app.form_common.fixture_pose()
            world = transform_work_trajectory(local, pose)
        except ValueError as error:
            messagebox.showerror("Ошибка", str(error))
            return

        self.app.set_trajectory(self.generator_id, world)
        store_trajectory(self.app.project, self.generator_id, world)
        self.app.form_common.save_fixture_pose_to_project()
        self.app.save_project()

        self.status_var.set(self._format_status(world) + " (мировая)")
        self._set_export_state(True)
        self.move_pose_button.configure(state=tk.NORMAL)

    def _format_status(self, trajectory) -> str:
        return (
            f"Траектория: {trajectory.metadata.get('pass_count', '?')} проходов, "
            f"{len(trajectory.travel_segments)} перемещений, "
            f"{trajectory.pose_count} поз"
        )

    def _set_export_state(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        self.export_step_button.configure(state=state)
        self.export_robot_button.configure(state=state)

    def _export_step(self) -> None:
        trajectory = self.app.get_trajectory(self.generator_id)
        if trajectory is None:
            messagebox.showwarning("Экспорт", "Сначала создайте траекторию.")
            return

        filepath = filedialog.asksaveasfilename(
            title="Сохранить STEP — низ формы",
            defaultextension=".step",
            filetypes=[("STEP files", "*.step *.stp"), ("All files", "*.*")],
        )
        if not filepath:
            return

        try:
            export_trajectory(trajectory, filepath)
        except Exception as error:
            messagebox.showerror("Экспорт STEP", str(error))
            return

        messagebox.showinfo("Экспорт STEP", f"Файл сохранён:\n{filepath}")

    def _export_robot(self) -> None:
        trajectory = self.app.get_trajectory(self.generator_id)
        if trajectory is None:
            messagebox.showwarning("Экспорт", "Сначала создайте траекторию.")
            return

        filepath = filedialog.asksaveasfilename(
            title="Сохранить УП — низ формы",
            defaultextension=".txt",
            filetypes=[("All files", "*.*")],
        )
        if not filepath:
            return

        try:
            export_robot_program(trajectory, filepath)
        except RobotExportNotImplementedError:
            messagebox.showinfo(
                "Создать УП",
                "Экспорт управляющей программы — следующий этап разработки.",
            )
        except Exception as error:
            messagebox.showerror("Создать УП", str(error))
