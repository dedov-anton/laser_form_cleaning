from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import TYPE_CHECKING, Callable, Optional

if TYPE_CHECKING:
    from src.gui.app import TrajectoryApp


class StubSurfaceSection:
    """Секция-заглушка для будущих генераторов."""

    def __init__(
        self,
        parent: ttk.Frame,
        app: TrajectoryApp,
        *,
        title: str,
        generator_id: str,
        description: str,
    ) -> None:
        self.app = app
        self.generator_id = generator_id
        self.status_var = tk.StringVar(value="Траектория: не создана")
        self._build(parent, title, description)

    def _build(self, parent: ttk.Frame, title: str, description: str) -> None:
        frame = ttk.LabelFrame(parent, text=title, padding=8)
        frame.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(frame, text=description, wraplength=760).pack(anchor=tk.W, pady=(0, 8))

        buttons = ttk.Frame(frame)
        buttons.pack(fill=tk.X, pady=(0, 4))
        ttk.Button(
            buttons,
            text="Создать траекторию",
            state=tk.DISABLED,
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(buttons, text="Экспорт STEP", state=tk.DISABLED).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(buttons, text="Создать УП", state=tk.DISABLED).pack(side=tk.LEFT)
        ttk.Label(frame, textvariable=self.status_var).pack(anchor=tk.W)

    def bind_updates(self, _callback: Callable[[], None]) -> None:
        return

    def load_from_project(self) -> None:
        trajectory = self.app.get_trajectory(self.generator_id)
        if trajectory is None:
            self.status_var.set("Траектория: не создана")
            return
        self.status_var.set(
            f"Траектория: {trajectory.metadata.get('pass_count', '?')} проходов, "
            f"{len(trajectory.travel_segments)} перемещений, "
            f"{trajectory.pose_count} поз (из проекта)"
        )
