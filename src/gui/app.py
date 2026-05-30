from __future__ import annotations

import math
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Optional, Tuple

from src.core.config_store import SavedParams, load_params, save_params
from src.core.models import ProfileWaypoint, RingParams, RingTrajectory
from src.core.ring_calc import (
    overlap_mm,
    overlap_percent,
    recommended_sector_count,
    build_trajectory,
)
from src.export.step_export import export_ring_trajectory
from src.gui.profile_canvas import ProfileCanvas

PROJECT_ROOT = Path(__file__).resolve().parents[2]


class TrajectoryApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Pirelli Trajectory Builder — нижнее кольцо")
        self.geometry("920x640")
        self.minsize(820, 520)

        self._trajectory: Optional[RingTrajectory] = None
        self._build_variables()
        self._build_ui()
        self._load_saved_params()
        self._bind_updates()

    def _build_variables(self) -> None:
        self.inner_radius_var = tk.StringVar()
        self.ring_width_var = tk.StringVar()
        self.outer_radius_var = tk.StringVar(value="—")
        self.beam_width_var = tk.StringVar(value="100")
        self.sector_count_var = tk.StringVar()
        self.overlap_outer_var = tk.StringVar(value="—")
        self.overlap_inner_var = tk.StringVar(value="—")
        self.status_var = tk.StringVar(value="Траектория: не создана")
        self.start_x_var = tk.StringVar(value="0")
        self.start_y_var = tk.StringVar(value="0")
        self.start_z_var = tk.StringVar(value="0")
        self.finish_x_var = tk.StringVar(value="0")
        self.finish_y_var = tk.StringVar(value="0")
        self.finish_z_var = tk.StringVar(value="0")
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

    def _build_ui(self) -> None:
        bottom = ttk.Frame(self, padding=(10, 0, 10, 10))
        bottom.pack(side=tk.BOTTOM, fill=tk.X)

        buttons = ttk.Frame(bottom)
        buttons.pack(fill=tk.X, pady=(0, 4))
        ttk.Button(buttons, text="Создать траекторию", command=self._create_trajectory).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        self.export_button = ttk.Button(
            buttons, text="Экспорт STEP", command=self._export_step, state=tk.DISABLED
        )
        self.export_button.pack(side=tk.LEFT)
        ttk.Label(bottom, textvariable=self.status_var).pack(anchor=tk.W)

        scroll_container = ttk.Frame(self, padding=(10, 10, 10, 0))
        scroll_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(scroll_container, highlightthickness=0)
        self._scroll_canvas = canvas
        scrollbar = ttk.Scrollbar(scroll_container, orient=tk.VERTICAL, command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        main = ttk.Frame(canvas)
        canvas_window = canvas.create_window((0, 0), window=main, anchor=tk.NW)

        def _on_configure(_event=None) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfigure(canvas_window, width=canvas.winfo_width())

        main.bind("<Configure>", _on_configure)
        canvas.bind("<Configure>", _on_configure)
        canvas.bind_all("<MouseWheel>", self._on_mousewheel, add="+")

        top_row = ttk.Frame(main)
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
        self._add_labeled_entry(clean_frame, 0, "Ширина луча (мм):", self.beam_width_var)
        sector_row = ttk.Frame(clean_frame)
        sector_row.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=4)
        ttk.Label(sector_row, text="Кол-во секторов:").pack(side=tk.LEFT)
        ttk.Entry(sector_row, textvariable=self.sector_count_var, width=10).pack(
            side=tk.LEFT, padx=(8, 4)
        )
        ttk.Button(sector_row, text="↻ авто", command=self._apply_recommended_sectors).pack(
            side=tk.LEFT
        )
        self.overlap_outer_label = ttk.Label(clean_frame, textvariable=self.overlap_outer_var)
        self.overlap_outer_label.grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=2)
        ttk.Label(clean_frame, textvariable=self.overlap_inner_var).grid(
            row=3, column=0, columnspan=2, sticky=tk.W, pady=2
        )

        entry_frame = ttk.LabelFrame(main, text="Сектор O→I (° 0–360)", padding=8)
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

        profile_frame = ttk.LabelFrame(main, text="Профиль / наклон Z", padding=8)
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

        self._build_midpoint_block(mid_row, 0, "Промеж. 1", self.profile1_dist_var, self.profile1_z_var, self.profile1_tilt_var)
        self._build_midpoint_block(mid_row, 1, "Промеж. 2", self.profile2_dist_var, self.profile2_z_var, self.profile2_tilt_var)

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

        points_frame = ttk.LabelFrame(main, text="Старт / финиш (мм)", padding=8)
        points_frame.pack(fill=tk.X, pady=(0, 4))
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
        ttk.Label(parent, text=label).grid(row=0, column=column, sticky=tk.W, padx=(0 if column == 0 else 8, 4))
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

    def _on_mousewheel(self, event: tk.Event) -> None:
        if event.delta and self._scroll_canvas.winfo_exists():
            self._scroll_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _bind_updates(self) -> None:
        for variable in (
            self.inner_radius_var,
            self.ring_width_var,
            self.beam_width_var,
            self.sector_count_var,
            self.inner_tilt_var,
            self.outer_tilt_var,
            self.profile1_dist_var,
            self.profile1_z_var,
            self.profile1_tilt_var,
            self.profile2_dist_var,
            self.profile2_z_var,
            self.profile2_tilt_var,
        ):
            variable.trace_add("write", self._on_params_changed)

    def _on_params_changed(self, *_args) -> None:
        self._update_derived_fields()
        self._update_profile_preview()

    def _update_profile_preview(self) -> None:
        try:
            ring_width = self._parse_positive_float(
                self.ring_width_var.get(), "Ширина кольца"
            )
        except ValueError:
            self.profile_canvas.draw_profile(0.0, 0.0, 0.0, ProfileWaypoint(), ProfileWaypoint())
            return

        try:
            inner_tilt = self._parse_tilt_deg(
                self.inner_tilt_var.get(), "Наклон Z на внутр. радиусе"
            )
            outer_tilt = self._parse_tilt_deg(
                self.outer_tilt_var.get(), "Наклон Z на внеш. радиусе"
            )
            profile1 = self._read_profile_waypoint(1)
            profile2 = self._read_profile_waypoint(2)
        except ValueError:
            return

        self.profile_canvas.draw_profile(
            ring_width, inner_tilt, outer_tilt, profile1, profile2
        )

    def _load_saved_params(self) -> None:
        saved = load_params(PROJECT_ROOT)
        if saved.inner_radius_mm is not None:
            self.inner_radius_var.set(self._format_number(saved.inner_radius_mm))
        if saved.ring_width_mm is not None:
            self.ring_width_var.set(self._format_number(saved.ring_width_mm))
        self.beam_width_var.set(self._format_number(saved.beam_width_mm))
        self.start_x_var.set(self._format_number(saved.start_x_mm))
        self.start_y_var.set(self._format_number(saved.start_y_mm))
        self.start_z_var.set(self._format_number(saved.start_z_mm))
        self.finish_x_var.set(self._format_number(saved.finish_x_mm))
        self.finish_y_var.set(self._format_number(saved.finish_y_mm))
        self.finish_z_var.set(self._format_number(saved.finish_z_mm))
        if saved.entry_sector_start_deg is not None:
            self.entry_sector_start_var.set(
                self._format_number(saved.entry_sector_start_deg)
            )
        if saved.entry_sector_end_deg is not None:
            self.entry_sector_end_var.set(
                self._format_number(saved.entry_sector_end_deg)
            )
        self.inner_tilt_var.set(self._format_number(saved.inner_tilt_deg))
        self.outer_tilt_var.set(self._format_number(saved.outer_tilt_deg))
        self._load_profile_waypoint(saved.profile_point_1, 1)
        self._load_profile_waypoint(saved.profile_point_2, 2)

        if saved.sector_count is not None:
            self.sector_count_var.set(str(saved.sector_count))
        else:
            self._apply_recommended_sectors(silent=True)

        self._update_derived_fields()
        self._update_profile_preview()

    def _apply_recommended_sectors(self, silent: bool = False) -> None:
        try:
            params = self._read_params(require_sector_count=False)
        except ValueError as error:
            if not silent:
                messagebox.showerror("Ошибка", str(error))
            return

        recommended = recommended_sector_count(
            params.outer_radius_mm, params.beam_width_mm
        )
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

        self.outer_radius_var.set(self._format_number(params.outer_radius_mm))

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

    def _read_params(self, require_sector_count: bool) -> RingParams:
        inner_radius = self._parse_positive_float(
            self.inner_radius_var.get(), "Внутренний радиус"
        )
        ring_width = self._parse_positive_float(
            self.ring_width_var.get(), "Ширина кольца"
        )
        beam_width = self._parse_positive_float(
            self.beam_width_var.get(), "Ширина луча"
        )

        sector_text = self.sector_count_var.get().strip()
        if not sector_text:
            if require_sector_count:
                raise ValueError("Укажите количество секторов")
            sector_count = recommended_sector_count(inner_radius + ring_width, beam_width)
        else:
            sector_count = self._parse_positive_int(sector_text, "Кол-во секторов")

        return RingParams(
            inner_radius_mm=inner_radius,
            ring_width_mm=ring_width,
            beam_width_mm=beam_width,
            sector_count=sector_count,
            entry_sector_start_deg=self._read_optional_angle(
                self.entry_sector_start_var.get(), "Начало сектора O→I"
            ),
            entry_sector_end_deg=self._read_optional_angle(
                self.entry_sector_end_var.get(), "Конец сектора O→I"
            ),
            inner_tilt_deg=self._parse_tilt_deg(
                self.inner_tilt_var.get(), "Наклон Z на внутр. радиусе"
            ),
            outer_tilt_deg=self._parse_tilt_deg(
                self.outer_tilt_var.get(), "Наклон Z на внеш. радиусе"
            ),
            profile_point_1=self._read_profile_waypoint(1),
            profile_point_2=self._read_profile_waypoint(2),
        )

    def _load_profile_waypoint(self, waypoint: ProfileWaypoint, index: int) -> None:
        vars_ = (
            (self.profile1_dist_var, self.profile1_z_var, self.profile1_tilt_var)
            if index == 1
            else (self.profile2_dist_var, self.profile2_z_var, self.profile2_tilt_var)
        )
        vars_[0].set(self._format_number(waypoint.distance_from_outer_mm))
        vars_[1].set(self._format_number(waypoint.z_offset_mm))
        vars_[2].set(self._format_number(waypoint.tilt_deg))

    def _read_profile_waypoint(self, index: int) -> ProfileWaypoint:
        prefix = f"Промеж. точка {index}"
        dist_var = self.profile1_dist_var if index == 1 else self.profile2_dist_var
        z_var = self.profile1_z_var if index == 1 else self.profile2_z_var
        tilt_var = self.profile1_tilt_var if index == 1 else self.profile2_tilt_var
        return ProfileWaypoint(
            distance_from_outer_mm=self._parse_optional_non_negative_float(
                dist_var.get(), f"{prefix}, от внеш. радиуса"
            ),
            z_offset_mm=self._parse_optional_float(z_var.get(), f"{prefix}, Z"),
            tilt_deg=self._parse_tilt_deg(tilt_var.get(), f"{prefix}, наклон"),
        )

    def _read_optional_angle(self, value: str, field_name: str) -> Optional[float]:
        text = value.strip().replace(",", ".")
        if not text:
            return None
        number = self._parse_float(text, field_name)
        if number < 0.0 or number >= 360.0:
            raise ValueError(f"{field_name}: угол должен быть в диапазоне 0–360")
        return number

    def _read_trajectory_points(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        start_point = (
            self._parse_float(self.start_x_var.get(), "Старт X"),
            self._parse_float(self.start_y_var.get(), "Старт Y"),
            self._parse_float(self.start_z_var.get(), "Старт Z"),
        )
        finish_point = (
            self._parse_float(self.finish_x_var.get(), "Финиш X"),
            self._parse_float(self.finish_y_var.get(), "Финиш Y"),
            self._parse_float(self.finish_z_var.get(), "Финиш Z"),
        )
        return start_point, finish_point

    def _create_trajectory(self) -> None:
        try:
            params = self._read_params(require_sector_count=True)
            start_point, finish_point = self._read_trajectory_points()
            trajectory = build_trajectory(
                params,
                start_point_mm=start_point,
                finish_point_mm=finish_point,
            )
        except ValueError as error:
            messagebox.showerror("Ошибка", str(error))
            return

        self._trajectory = trajectory
        self.status_var.set(
            f"Траектория: {len(trajectory.passes)} проходов, "
            f"{len(trajectory.travel_segments)} перемещений, "
            f"{len(trajectory.poses)} поз"
        )
        self.export_button.configure(state=tk.NORMAL)

        save_params(
            SavedParams(
                inner_radius_mm=params.inner_radius_mm,
                ring_width_mm=params.ring_width_mm,
                beam_width_mm=params.beam_width_mm,
                sector_count=params.sector_count,
                start_x_mm=start_point[0],
                start_y_mm=start_point[1],
                start_z_mm=start_point[2],
                finish_x_mm=finish_point[0],
                finish_y_mm=finish_point[1],
                finish_z_mm=finish_point[2],
                entry_sector_start_deg=params.entry_sector_start_deg,
                entry_sector_end_deg=params.entry_sector_end_deg,
                inner_tilt_deg=params.inner_tilt_deg,
                outer_tilt_deg=params.outer_tilt_deg,
                profile_point_1=params.profile_point_1,
                profile_point_2=params.profile_point_2,
            ),
            PROJECT_ROOT,
        )
        self._update_derived_fields()
        self._update_profile_preview()

    def _export_step(self) -> None:
        if self._trajectory is None:
            messagebox.showwarning("Экспорт", "Сначала создайте траекторию.")
            return

        filepath = filedialog.asksaveasfilename(
            title="Сохранить STEP",
            defaultextension=".step",
            filetypes=[("STEP files", "*.step *.stp"), ("All files", "*.*")],
        )
        if not filepath:
            return

        try:
            export_ring_trajectory(self._trajectory, Path(filepath))
        except Exception as error:
            messagebox.showerror("Экспорт STEP", str(error))
            return

        messagebox.showinfo("Экспорт STEP", f"Файл сохранён:\n{filepath}")

    @staticmethod
    def _parse_float(value: str, field_name: str) -> float:
        text = value.strip().replace(",", ".")
        if not text:
            raise ValueError(f"{field_name}: введите число")
        try:
            number = float(text)
        except ValueError as error:
            raise ValueError(f"{field_name}: некорректное число") from error
        if not math.isfinite(number):
            raise ValueError(f"{field_name}: некорректное число")
        return number

    @staticmethod
    def _parse_tilt_deg(value: str, field_name: str) -> float:
        text = value.strip().replace(",", ".")
        if not text:
            return 0.0
        number = TrajectoryApp._parse_float(text, field_name)
        if number <= -90.0 or number >= 90.0:
            raise ValueError(f"{field_name}: угол должен быть в диапазоне -89…89")
        return number

    @staticmethod
    def _parse_optional_float(value: str, field_name: str) -> float:
        text = value.strip().replace(",", ".")
        if not text:
            return 0.0
        return TrajectoryApp._parse_float(text, field_name)

    @staticmethod
    def _parse_optional_non_negative_float(value: str, field_name: str) -> float:
        number = TrajectoryApp._parse_optional_float(value, field_name)
        if number < 0.0:
            raise ValueError(f"{field_name}: должно быть >= 0")
        return number

    @staticmethod
    def _parse_positive_float(value: str, field_name: str) -> float:
        number = TrajectoryApp._parse_float(value, field_name)
        if number <= 0:
            raise ValueError(f"{field_name}: должно быть больше 0")
        return number

    @staticmethod
    def _parse_positive_int(value: str, field_name: str) -> int:
        text = value.strip()
        try:
            number = int(text)
        except ValueError as error:
            raise ValueError(f"{field_name}: введите целое число") from error
        if number < 1:
            raise ValueError(f"{field_name}: должно быть >= 1")
        return number

    @staticmethod
    def _format_number(value: float) -> str:
        if float(value).is_integer():
            return str(int(value))
        return f"{value:g}"


def run_app() -> None:
    app = TrajectoryApp()
    app.mainloop()
