from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Optional

from src.common.project import FormProject
from src.common.trajectory import WorkTrajectory
from src.gui.sections.bottom_ring import BottomRingSection
from src.gui.sections.cylinder_wall import CylinderWallSection
from src.gui.sections.form_common import FormCommonSection
from src.gui.section_styles import setup_section_styles
from src.gui.sections.top_ring import TopRingSection
from src.storage.project_store import (
    load_local_trajectory,
    load_project,
    load_trajectory,
    save_project,
)

PROJECT_ROOT = Path(__file__).resolve().parents[2]


class TrajectoryApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Pirelli — генератор траекторий лазерной очистки")
        self.geometry("920x760")
        self.minsize(820, 600)

        self.project: FormProject = load_project(PROJECT_ROOT)
        self._trajectories: dict[str, WorkTrajectory] = {}
        self._local_trajectories: dict[str, WorkTrajectory] = {}
        self._load_stored_trajectories()

        setup_section_styles(self)
        self._build_ui()
        self._load_sections()
        self._bind_updates()

    def _load_stored_trajectories(self) -> None:
        for generator_id in ("bottom_ring", "top_ring", "cylinder_wall"):
            trajectory = load_trajectory(self.project, generator_id)
            if trajectory is not None:
                self._trajectories[generator_id] = trajectory
            local = load_local_trajectory(self.project, generator_id)
            if local is not None:
                self._local_trajectories[generator_id] = local
            elif trajectory is not None and trajectory.metadata.get("coordinate_frame") != "world":
                self._local_trajectories[generator_id] = trajectory

    def _build_ui(self) -> None:
        scroll_container = ttk.Frame(self, padding=(10, 10, 10, 10))
        scroll_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(scroll_container, highlightthickness=0, bg="#F5F5F5")
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

        self.form_common = FormCommonSection(main, self)
        self.bottom_ring = BottomRingSection(main, self)
        self.top_ring = TopRingSection(main, self)
        self.cylinder_wall = CylinderWallSection(main, self)

    def _load_sections(self) -> None:
        self.form_common.load_from_project()
        self.bottom_ring.load_from_project()
        self.top_ring.load_from_project()
        self.cylinder_wall.load_from_project()

    def _bind_updates(self) -> None:
        def on_change() -> None:
            self.bottom_ring._update_derived_fields()
            self.bottom_ring._update_profile_preview()
            self.top_ring._update_derived_fields()
            self.cylinder_wall._update_derived_fields()
            self.cylinder_wall._update_profile_preview()

        self.form_common.bind_updates(on_change)
        self.bottom_ring.bind_updates(on_change)
        self.top_ring.bind_updates(on_change)
        self.cylinder_wall.bind_updates(on_change)

    def _on_mousewheel(self, event: tk.Event) -> None:
        if event.delta and self._scroll_canvas.winfo_exists():
            self._scroll_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def get_trajectory(self, generator_id: str) -> Optional[WorkTrajectory]:
        return self._trajectories.get(generator_id)

    def get_local_trajectory(self, generator_id: str) -> Optional[WorkTrajectory]:
        return self._local_trajectories.get(generator_id)

    def set_trajectory(self, generator_id: str, trajectory: WorkTrajectory) -> None:
        self._trajectories[generator_id] = trajectory

    def set_local_trajectory(self, generator_id: str, trajectory: WorkTrajectory) -> None:
        self._local_trajectories[generator_id] = trajectory

    def save_project(self) -> None:
        save_project(self.project, PROJECT_ROOT)


def run_app() -> None:
    app = TrajectoryApp()
    app.mainloop()
