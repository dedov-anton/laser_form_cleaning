from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Dict, Tuple

# Подложки основных блоков GUI (clam theme на Windows даёт видимый background).
SECTION_COLORS: Dict[str, Tuple[str, str]] = {
    "form_common": ("#ECEFF1", "#37474F"),
    "bottom_ring": ("#E3F2FD", "#1565C0"),
    "top_ring": ("#E8F5E9", "#2E7D32"),
    "cylinder_wall": ("#F3E5F5", "#6A1B9A"),
}

_STYLES_INITIALIZED = False


def _style_prefix(section_id: str) -> str:
    return "".join(part.capitalize() for part in section_id.split("_"))


def setup_section_styles(root: tk.Misc) -> None:
    global _STYLES_INITIALIZED
    if _STYLES_INITIALIZED:
        return

    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass

    for section_id, (background, foreground) in SECTION_COLORS.items():
        prefix = _style_prefix(section_id)
        for widget in (
            "TLabelframe",
            "TFrame",
            "TLabel",
            "TButton",
            "TCheckbutton",
            "TCombobox",
            "TEntry",
        ):
            name = f"{prefix}.{widget}"
            if widget == "TEntry":
                style.configure(name, fieldbackground="#FFFFFF", background=background)
            else:
                style.configure(name, background=background)
        style.configure(f"{prefix}.TLabelframe.Label", background=background, foreground=foreground)
        style.configure(f"{prefix}.TLabel", background=background, foreground=foreground)

    _STYLES_INITIALIZED = True


def section_label_frame(
    parent: tk.Misc,
    section_id: str,
    text: str,
    *,
    padding: int | Tuple[int, ...] = 8,
) -> ttk.LabelFrame:
    prefix = _style_prefix(section_id)
    return ttk.LabelFrame(
        parent,
        text=text,
        style=f"{prefix}.TLabelframe",
        padding=padding,
    )


def section_frame(parent: tk.Misc, section_id: str) -> ttk.Frame:
    prefix = _style_prefix(section_id)
    return ttk.Frame(parent, style=f"{prefix}.TFrame")


def section_label(parent: tk.Misc, section_id: str, **kwargs) -> ttk.Label:
    prefix = _style_prefix(section_id)
    return ttk.Label(parent, style=f"{prefix}.TLabel", **kwargs)


def section_button(parent: tk.Misc, section_id: str, **kwargs) -> ttk.Button:
    prefix = _style_prefix(section_id)
    return ttk.Button(parent, style=f"{prefix}.TButton", **kwargs)


def section_entry(parent: tk.Misc, section_id: str, **kwargs) -> ttk.Entry:
    prefix = _style_prefix(section_id)
    return ttk.Entry(parent, style=f"{prefix}.TEntry", **kwargs)


def section_checkbutton(parent: tk.Misc, section_id: str, **kwargs) -> ttk.Checkbutton:
    prefix = _style_prefix(section_id)
    return ttk.Checkbutton(parent, style=f"{prefix}.TCheckbutton", **kwargs)


def section_combobox(parent: tk.Misc, section_id: str, **kwargs) -> ttk.Combobox:
    prefix = _style_prefix(section_id)
    return ttk.Combobox(parent, style=f"{prefix}.TCombobox", **kwargs)


class SectionUI:
    """Хелпер виджетов с единой подложкой секции."""

    __slots__ = ("section_id",)

    def __init__(self, section_id: str) -> None:
        self.section_id = section_id

    def lf(self, parent: tk.Misc, text: str, *, padding: int | Tuple[int, ...] = 8) -> ttk.LabelFrame:
        return section_label_frame(parent, self.section_id, text, padding=padding)

    def fr(self, parent: tk.Misc) -> ttk.Frame:
        return section_frame(parent, self.section_id)

    def lb(self, parent: tk.Misc, **kwargs) -> ttk.Label:
        return section_label(parent, self.section_id, **kwargs)

    def en(self, parent: tk.Misc, **kwargs) -> ttk.Entry:
        return section_entry(parent, self.section_id, **kwargs)

    def bn(self, parent: tk.Misc, **kwargs) -> ttk.Button:
        return section_button(parent, self.section_id, **kwargs)

    def cb(self, parent: tk.Misc, **kwargs) -> ttk.Checkbutton:
        return section_checkbutton(parent, self.section_id, **kwargs)

    def cmb(self, parent: tk.Misc, **kwargs) -> ttk.Combobox:
        return section_combobox(parent, self.section_id, **kwargs)
