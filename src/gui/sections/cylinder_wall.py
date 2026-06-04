from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import TYPE_CHECKING, Callable

from src.exporters.robot.export import export_robot_program
from src.exporters.robot.elite_program import build_elite_program
from src.exporters.step.export import export_trajectory
from src.exporters.step.from_elite_program import export_step_from_elite_program
from src.generators.cylinder_wall.calc import (
    overlap_per_sector,
    overlap_percent,
    recommended_passes_per_sector,
)
from src.generators.cylinder_wall.params import (
    SECTOR_COUNT,
    CylinderWallParams,
    WallProfileWaypoint,
)
from src.generators.registry import get_generator
from src.gui.parsing import (
    format_number,
    parse_float,
    parse_optional_float,
    parse_optional_non_negative_float,
    parse_positive_float,
    parse_positive_int,
    parse_tilt_deg,
)
from src.gui.section_styles import SectionUI
from src.gui.widgets.wall_profile_canvas import WallProfileCanvas
from src.storage.project_store import store_local_trajectory, store_trajectory
from src.transforms.trajectory_transform import (
    apply_program_start_finish,
    transform_work_trajectory,
)
from src.transforms.wrist_correction_export import (
    format_wrist_correction_summary,
    run_cylinder_wrist_correction,
)

if TYPE_CHECKING:
    from src.gui.app import TrajectoryApp

SECTOR_LABELS = [
    "0 (0–45°)",
    "1 (45–90°)",
    "2 (90–135°)",
    "3 (135–180°)",
    "4 (180–225°)",
    "5 (225–270°)",
    "6 (270–315°)",
    "7 (315–360°)",
]


class CylinderWallSection:
    generator_id = "cylinder_wall"

    def __init__(self, parent: ttk.Frame, app: TrajectoryApp) -> None:
        self.app = app
        self.inner_radius_var = tk.StringVar()
        self.wall_height_var = tk.StringVar()
        self.z_top_var = tk.StringVar()
        self.passes_per_sector_var = tk.StringVar()
        self.overlap_var = tk.StringVar(value="—")
        self.start_sector_var = tk.StringVar(value=SECTOR_LABELS[0])
        self.clockwise_var = tk.BooleanVar(value=True)
        self.top_tilt_var = tk.StringVar(value="0")
        self.bottom_tilt_var = tk.StringVar(value="0")
        self.profile1_down_var = tk.StringVar(value="0")
        self.profile1_radial_var = tk.StringVar(value="0")
        self.profile1_tilt_var = tk.StringVar(value="0")
        self.profile2_down_var = tk.StringVar(value="0")
        self.profile2_radial_var = tk.StringVar(value="0")
        self.profile2_tilt_var = tk.StringVar(value="0")
        self.start_x_var = tk.StringVar(value="0")
        self.start_y_var = tk.StringVar(value="0")
        self.start_z_var = tk.StringVar(value="0")
        self.finish_x_var = tk.StringVar(value="0")
        self.finish_y_var = tk.StringVar(value="0")
        self.finish_z_var = tk.StringVar(value="0")
        self.lcorr_var = tk.StringVar(value="0")
        self.status_var = tk.StringVar(value="Траектория: не создана")
        self.ui = SectionUI("cylinder_wall")
        self._build(parent)

    def _build(self, parent: ttk.Frame) -> None:
        ui = self.ui
        frame = ui.lf(parent, "Поверхность качения (цилиндр)")
        frame.pack(fill=tk.X, pady=(0, 8))

        top_row = ui.fr(frame)
        top_row.pack(fill=tk.X, pady=(0, 8))
        top_row.columnconfigure(0, weight=1)
        top_row.columnconfigure(1, weight=1)

        cylinder_frame = ui.lf(top_row, "Параметры цилиндра")
        cylinder_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 4))
        self._add_labeled_entry(cylinder_frame, 0, "Внутренний радиус (мм):", self.inner_radius_var)
        self._add_labeled_entry(cylinder_frame, 1, "Высота стенки (мм):", self.wall_height_var)
        self._add_labeled_entry(cylinder_frame, 2, "Z до верхней точки (мм):", self.z_top_var)

        clean_frame = ui.lf(top_row, "Параметры чистки")
        clean_frame.grid(row=0, column=1, sticky=tk.NSEW, padx=(4, 0))
        passes_row = ui.fr(clean_frame)
        passes_row.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=4)
        ui.lb(passes_row, text="Проходов в секторе:").pack(side=tk.LEFT)
        ui.en(passes_row, textvariable=self.passes_per_sector_var, width=10).pack(
            side=tk.LEFT, padx=(8, 4)
        )
        ui.bn(passes_row, text="↻ авто", command=self._apply_recommended_passes).pack(side=tk.LEFT)
        self.overlap_label = ui.lb(clean_frame, textvariable=self.overlap_var)
        self.overlap_label.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=2)
        ui.lb(
            clean_frame,
            text=f"Секторов: {SECTOR_COUNT} (по 45°), обработка изнутри, сверху вниз",
            wraplength=360,
        ).grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(4, 0))

        sector_frame = ui.lf(frame, "Стартовый сектор")
        sector_frame.pack(fill=tk.X, pady=(0, 8))
        sector_row = ui.fr(sector_frame)
        sector_row.pack(fill=tk.X)
        ui.lb(sector_row, text="Сектор:").pack(side=tk.LEFT)
        ui.cmb(
            sector_row,
            textvariable=self.start_sector_var,
            values=SECTOR_LABELS,
            state="readonly",
            width=16,
        ).pack(side=tk.LEFT, padx=(8, 16))
        ui.cb(
            sector_row,
            text="По часовой стрелке",
            variable=self.clockwise_var,
        ).pack(side=tk.LEFT)

        profile_frame = ui.lf(frame, "Профиль / наклон")
        profile_frame.pack(fill=tk.X, pady=(0, 8))
        profile_frame.columnconfigure(0, weight=1)
        profile_frame.columnconfigure(1, weight=0)

        fields = ui.fr(profile_frame)
        fields.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 8))

        edges = ui.fr(fields)
        edges.pack(fill=tk.X, pady=(0, 6))
        self._add_inline_field(edges, "Верх. наклон (°):", self.top_tilt_var, 0)
        self._add_inline_field(edges, "Низ. наклон (°):", self.bottom_tilt_var, 2)

        mid_row = ui.fr(fields)
        mid_row.pack(fill=tk.X)
        mid_row.columnconfigure(0, weight=1)
        mid_row.columnconfigure(1, weight=1)
        self._build_midpoint_block(
            mid_row,
            0,
            "Промеж. 1",
            self.profile1_down_var,
            self.profile1_radial_var,
            self.profile1_tilt_var,
        )
        self._build_midpoint_block(
            mid_row,
            1,
            "Промеж. 2",
            self.profile2_down_var,
            self.profile2_radial_var,
            self.profile2_tilt_var,
        )
        ui.lb(
            fields,
            text="+ — к оси, − — наружу от стенки. Вниз=0 — точка выкл.",
            wraplength=420,
        ).pack(anchor=tk.W, pady=(6, 0))

        preview_frame = ui.fr(profile_frame)
        preview_frame.grid(row=0, column=1, sticky=tk.N)
        ui.lb(preview_frame, text="Сечение стенки").pack(anchor=tk.W)
        self.profile_canvas = WallProfileCanvas(preview_frame)
        self.profile_canvas.pack()
        self.profile_canvas.bind("<Configure>", lambda _e: self._update_profile_preview())

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
        ui.bn(buttons, text="Создать траекторию", command=self._create_trajectory).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        self.move_pose_button = ui.bn(
            buttons,
            text="Переместить по 6D позе",
            command=self._move_by_fixture_pose,
            state=tk.DISABLED,
        )
        self.move_pose_button.pack(side=tk.LEFT, padx=(0, 8))
        self.export_step_button = ui.bn(
            buttons, text="Экспорт STEP", command=self._export_step, state=tk.DISABLED
        )
        self.export_step_button.pack(side=tk.LEFT, padx=(0, 8))
        self.export_robot_button = ui.bn(
            buttons, text="Создать УП", command=self._export_robot, state=tk.DISABLED
        )
        self.export_robot_button.pack(side=tk.LEFT, padx=(0, 8))
        ui.lb(buttons, text="Lcorr (мм):").pack(side=tk.LEFT, padx=(0, 4))
        ui.en(buttons, textvariable=self.lcorr_var, width=6).pack(side=tk.LEFT, padx=(0, 8))
        self.wrist_correction_button = ui.bn(
            buttons,
            text="Коррекция запястья",
            command=self._apply_wrist_correction,
            state=tk.DISABLED,
        )
        self.wrist_correction_button.pack(side=tk.LEFT)
        ui.lb(frame, textvariable=self.status_var).pack(anchor=tk.W)

    def _build_midpoint_block(
        self,
        parent: ttk.Frame,
        column: int,
        title: str,
        down_var: tk.StringVar,
        radial_var: tk.StringVar,
        tilt_var: tk.StringVar,
    ) -> None:
        block = self.ui.lf(parent, title, padding=6)
        block.grid(row=0, column=column, sticky=tk.NSEW, padx=(0 if column == 0 else 4, 0))
        self._add_labeled_entry(block, 0, "Вниз (мм):", down_var, entry_width=8)
        self._add_labeled_entry(block, 1, "К оси (мм):", radial_var, entry_width=8)
        self._add_labeled_entry(block, 2, "Наклон (°):", tilt_var, entry_width=8)

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

    def bind_updates(self, callback: Callable[[], None]) -> None:
        for variable in (
            self.inner_radius_var,
            self.wall_height_var,
            self.z_top_var,
            self.passes_per_sector_var,
            self.top_tilt_var,
            self.bottom_tilt_var,
            self.profile1_down_var,
            self.profile1_radial_var,
            self.profile1_tilt_var,
            self.profile2_down_var,
            self.profile2_radial_var,
            self.profile2_tilt_var,
            self.start_x_var,
            self.start_y_var,
            self.start_z_var,
            self.finish_x_var,
            self.finish_y_var,
            self.finish_z_var,
        ):
            variable.trace_add("write", lambda *_args: callback())
        self.start_sector_var.trace_add("write", lambda *_args: callback())
        self.clockwise_var.trace_add("write", lambda *_args: callback())

    def load_from_project(self) -> None:
        data = self.app.project.cylinder_wall
        if data.get("inner_radius_mm") is not None:
            self.inner_radius_var.set(format_number(float(data["inner_radius_mm"])))
        if data.get("wall_height_mm") is not None:
            self.wall_height_var.set(format_number(float(data["wall_height_mm"])))
        if data.get("z_top_mm") is not None:
            self.z_top_var.set(format_number(float(data["z_top_mm"])))
        self.top_tilt_var.set(format_number(float(data.get("top_tilt_deg", 0.0))))
        self.bottom_tilt_var.set(format_number(float(data.get("bottom_tilt_deg", 0.0))))
        self._load_profile_waypoint(data.get("profile_point_1", {}), 1)
        self._load_profile_waypoint(data.get("profile_point_2", {}), 2)
        self.clockwise_var.set(bool(data.get("clockwise", True)))

        start_sector = int(data.get("start_sector", 0))
        if 0 <= start_sector < len(SECTOR_LABELS):
            self.start_sector_var.set(SECTOR_LABELS[start_sector])

        if data.get("passes_per_sector") is not None:
            self.passes_per_sector_var.set(str(int(data["passes_per_sector"])))
        else:
            self._apply_recommended_passes(silent=True)

        self.start_x_var.set(format_number(float(data.get("start_x_mm", 0.0))))
        self.start_y_var.set(format_number(float(data.get("start_y_mm", 0.0))))
        self.start_z_var.set(format_number(float(data.get("start_z_mm", 0.0))))
        self.finish_x_var.set(format_number(float(data.get("finish_x_mm", 0.0))))
        self.finish_y_var.set(format_number(float(data.get("finish_y_mm", 0.0))))
        self.finish_z_var.set(format_number(float(data.get("finish_z_mm", 0.0))))
        self.lcorr_var.set(format_number(float(data.get("lcorr_mm", 0.0))))

        trajectory = self.app.get_trajectory(self.generator_id)
        local = self.app.get_local_trajectory(self.generator_id)
        if trajectory is None:
            self._set_export_state(False)
            self.move_pose_button.configure(state=tk.DISABLED)
            self.status_var.set("Траектория: не создана")
        else:
            self._set_export_state(True)
            self.move_pose_button.configure(state=tk.NORMAL if local is not None else tk.DISABLED)
            suffix = (
                " (мировая)"
                if trajectory.metadata.get("coordinate_frame") == "world"
                else " (локальная)"
            )
            self.status_var.set(self._format_status(trajectory) + suffix)

        self._update_derived_fields()
        self._update_profile_preview()

    def _load_profile_waypoint(self, data: dict, index: int) -> None:
        vars_ = (
            (self.profile1_down_var, self.profile1_radial_var, self.profile1_tilt_var)
            if index == 1
            else (self.profile2_down_var, self.profile2_radial_var, self.profile2_tilt_var)
        )
        vars_[0].set(format_number(float(data.get("z_down_mm", 0.0))))
        vars_[1].set(format_number(float(data.get("radial_inward_mm", 0.0))))
        vars_[2].set(format_number(float(data.get("tilt_deg", 0.0))))

    def _read_profile_waypoint(self, index: int) -> WallProfileWaypoint:
        prefix = f"Промеж. точка {index}"
        if index == 1:
            down_var, radial_var, tilt_var = (
                self.profile1_down_var,
                self.profile1_radial_var,
                self.profile1_tilt_var,
            )
        else:
            down_var, radial_var, tilt_var = (
                self.profile2_down_var,
                self.profile2_radial_var,
                self.profile2_tilt_var,
            )
        return WallProfileWaypoint(
            z_down_mm=parse_optional_non_negative_float(down_var.get(), f"{prefix}, вниз"),
            radial_inward_mm=parse_optional_float(radial_var.get(), f"{prefix}, к оси"),
            tilt_deg=parse_tilt_deg(tilt_var.get(), f"{prefix}, наклон"),
        )

    def _update_profile_preview(self) -> None:
        try:
            wall_height = parse_positive_float(self.wall_height_var.get(), "Высота стенки")
            inner_radius = parse_positive_float(self.inner_radius_var.get(), "Внутренний радиус")
        except ValueError:
            self.profile_canvas.draw_profile(
                0.0,
                0.0,
                0.0,
                0.0,
                WallProfileWaypoint(),
                WallProfileWaypoint(),
            )
            return

        try:
            top_tilt = parse_tilt_deg(self.top_tilt_var.get(), "Наклон верхней точки")
            bottom_tilt = parse_tilt_deg(self.bottom_tilt_var.get(), "Наклон нижней точки")
            profile1 = self._read_profile_waypoint(1)
            profile2 = self._read_profile_waypoint(2)
        except ValueError:
            return

        self.profile_canvas.draw_profile(
            wall_height,
            inner_radius,
            top_tilt,
            bottom_tilt,
            profile1,
            profile2,
        )

    def save_params_to_project(self) -> None:
        params = self._read_params(require_passes=True)
        data = params.to_dict()
        data.update(self._read_trajectory_points_dict())
        data["lcorr_mm"] = parse_float(self.lcorr_var.get(), "Lcorr")
        self.app.project.cylinder_wall = data

    def _parse_start_sector(self) -> int:
        label = self.start_sector_var.get().strip()
        if label in SECTOR_LABELS:
            return SECTOR_LABELS.index(label)
        return int(label.split()[0])

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

    def _read_params(self, require_passes: bool) -> CylinderWallParams:
        inner_radius = parse_positive_float(self.inner_radius_var.get(), "Внутренний радиус")
        wall_height = parse_positive_float(self.wall_height_var.get(), "Высота стенки")
        z_top = parse_float(self.z_top_var.get(), "Z до верхней точки")
        beam_width = self.app.form_common.beam_width_mm()

        passes_text = self.passes_per_sector_var.get().strip()
        if not passes_text:
            if require_passes:
                raise ValueError("Укажите число проходов в секторе")
            passes_per_sector = recommended_passes_per_sector(inner_radius, beam_width)
        else:
            passes_per_sector = parse_positive_int(passes_text, "Проходов в секторе")

        return CylinderWallParams(
            inner_radius_mm=inner_radius,
            wall_height_mm=wall_height,
            z_top_mm=z_top,
            beam_width_mm=beam_width,
            passes_per_sector=passes_per_sector,
            start_sector=self._parse_start_sector(),
            clockwise=bool(self.clockwise_var.get()),
            top_tilt_deg=parse_tilt_deg(self.top_tilt_var.get(), "Наклон верхней точки"),
            bottom_tilt_deg=parse_tilt_deg(self.bottom_tilt_var.get(), "Наклон нижней точки"),
            profile_point_1=self._read_profile_waypoint(1),
            profile_point_2=self._read_profile_waypoint(2),
        )

    def _apply_recommended_passes(self, silent: bool = False) -> None:
        try:
            params = self._read_params(require_passes=False)
        except ValueError as error:
            if not silent:
                messagebox.showerror("Ошибка", str(error))
            return

        recommended = recommended_passes_per_sector(
            params.inner_radius_mm,
            params.beam_width_mm,
        )
        self.passes_per_sector_var.set(str(recommended))
        self._update_derived_fields()

    def _update_derived_fields(self) -> None:
        try:
            params = self._read_params(require_passes=False)
        except ValueError:
            self.overlap_var.set("Перекрытие на дуге сектора: —")
            self.overlap_label.configure(foreground="")
            return

        if not self.passes_per_sector_var.get().strip():
            return

        try:
            params_with_passes = self._read_params(require_passes=True)
        except ValueError:
            self.overlap_var.set("Перекрытие на дуге сектора: —")
            self.overlap_label.configure(foreground="")
            return

        overlap = overlap_per_sector(
            params_with_passes.inner_radius_mm,
            params_with_passes.beam_width_mm,
            params_with_passes.passes_per_sector,
        )
        overlap_pct = overlap_percent(overlap, params_with_passes.beam_width_mm)
        self.overlap_var.set(f"Перекрытие на дуге сектора: {overlap:.2f} мм ({overlap_pct:.1f}%)")
        if overlap < 0:
            color = "red"
        elif overlap < 0.1 * params_with_passes.beam_width_mm:
            color = "#b8860b"
        else:
            color = "green"
        self.overlap_label.configure(foreground=color)

    def _create_trajectory(self) -> None:
        try:
            params = self._read_params(require_passes=True)
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
        self.wrist_correction_button.configure(state=state)

    def _export_step(self) -> None:
        trajectory = self.app.get_trajectory(self.generator_id)
        if trajectory is None:
            messagebox.showwarning("Экспорт", "Сначала создайте траекторию.")
            return

        filepath = filedialog.asksaveasfilename(
            title="Сохранить STEP — поверхность качения",
            defaultextension=".step",
            filetypes=[("STEP files", "*.step *.stp"), ("All files", "*.*")],
        )
        if not filepath:
            return

        try:
            if trajectory.metadata.get("wrist_correction_applied"):
                settings = self.app.form_common.robot_export_settings()
                program = build_elite_program(trajectory, settings)
                export_step_from_elite_program(program, filepath)
            else:
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
            title="Сохранить УП — поверхность качения",
            defaultextension=".txt",
            filetypes=[("All files", "*.*")],
        )
        if not filepath:
            return

        try:
            start_point, finish_point = self._read_trajectory_points()
            trajectory = apply_program_start_finish(trajectory, start_point, finish_point)
            settings = self.app.form_common.robot_export_settings()
            export_robot_program(trajectory, Path(filepath), settings)
        except Exception as error:
            messagebox.showerror("Создать УП", str(error))
            return

        messagebox.showinfo("Создать УП", f"Файл сохранён:\n{filepath}")

    def _apply_wrist_correction(self) -> None:
        trajectory = self.app.get_trajectory(self.generator_id)
        if trajectory is None:
            messagebox.showwarning("Коррекция запястья", "Сначала создайте траекторию.")
            return

        project_root = Path(__file__).resolve().parents[3]
        default_up_dir = project_root / "УП"
        up_path = filedialog.asksaveasfilename(
            title="Сохранить УП — коррекция запястья",
            defaultextension=".txt",
            filetypes=[("УП Elite", "*.txt"), ("All files", "*.*")],
            initialdir=str(default_up_dir) if default_up_dir.is_dir() else str(project_root),
            initialfile="cylinder_wall_wrist_corrected.txt",
        )
        if not up_path:
            return

        try:
            lcorr_mm = parse_float(self.lcorr_var.get(), "Lcorr")
            outputs = run_cylinder_wrist_correction(
                project_root,
                trajectory=trajectory,
                robot_settings=self.app.form_common.robot_export_settings(),
                lcorr_mm=lcorr_mm,
                output_up_path=Path(up_path),
            )
        except Exception as error:
            messagebox.showerror("Коррекция запястья", str(error))
            return

        messagebox.showinfo("Коррекция запястья", format_wrist_correction_summary(outputs))
