# -*- coding: utf-8 -*-
"""
Welding Path Generator for Elite CS612 Robot v2.0
+ Уникальные имена слоёв
+ Старт с безопасной высоты

import runpy

path = r"D:\Program Files\FreeCAD 1.0\Mod\EliteRobot\scripts\welding_path_generator_bk.py"
runpy.run_path(path, run_name="__main__")



"""

import FreeCAD as App
import FreeCADGui as Guiвот
from PySide import QtGui, QtCore
import numpy as np
import math
import Part
import sys
import os

# Импортируем функции сохранения
sys.path.insert(0, os.path.dirname(__file__))
from trajectory_storage import save_trajectory_with_prefix

from config_manager import get_config



DEBUG_WPG = True
DEBUG_TAG = "WPG_DEBUG_2026-01-13"

if DEBUG_WPG:
    try:
        App.Console.PrintMessage(f"\n[{DEBUG_TAG}] LOADED FILE: {__file__}\n")
    except Exception:
        App.Console.PrintMessage(f"\n[{DEBUG_TAG}] LOADED FILE: __file__ is not defined\n")



# === КОНСТАНТЫ ===
TOOL_ROTATION_CORRECTION_DEG = -90.0
TOOL_ROTATION_CORRECTION_RAD = math.radians(TOOL_ROTATION_CORRECTION_DEG)
DEFAULT_DISCRETIZATION_MM = 0.2
DEFAULT_SAFE_HEIGHT_MM = 200.0
DEFAULT_BLEND_RADIUS_MM = 0.001
DEFAULT_TOOL_ROTATION_DEG = 0.0
DEFAULT_TOOL_OFFSET_MM = 5.0
DEFAULT_VELOCITY = 0.03
DEFAULT_ACCELERATION = 0.05


class ParametersDialog(QtGui.QDialog):
    def __init__(self):
        super(ParametersDialog, self).__init__(Gui.getMainWindow())
        self.setWindowTitle("Параметры сварочной траектории")
        self.setup_ui()
        # --- Интерактивная корректировка ориентации на выходе ---
        self.is_closed_trajectory = False
        self.base_tool_vectors = []
        self.base_tangents = []
        self.base_orientations = []
        # Параметры выхода (для незамкнутых):
        self.finish_tool_rotation_deg = None  # если None, будет равен self.tool_rotation
        self.finish_tilt_along_deg = 0.0
        self.finish_tilt_across_deg = 0.0
        self._vis_group_obj = None



    def setup_ui(self):
        layout = QtGui.QFormLayout()
        self.discr_spin = QtGui.QDoubleSpinBox()
        self.discr_spin.setRange(0.01, 10.0)
        self.discr_spin.setDecimals(2)
        self.discr_spin.setValue(DEFAULT_DISCRETIZATION_MM)
        layout.addRow("Дискретизация (мм):", self.discr_spin)
        self.safe_spin = QtGui.QDoubleSpinBox()
        self.safe_spin.setRange(0.0, 1500.0)
        self.safe_spin.setDecimals(1)
        self.safe_spin.setValue(DEFAULT_SAFE_HEIGHT_MM)
        layout.addRow("Безопасная высота (мм):", self.safe_spin)
        self.blend_spin = QtGui.QDoubleSpinBox()
        self.blend_spin.setRange(0.0, 10.0)
        self.blend_spin.setDecimals(3)
        self.blend_spin.setValue(DEFAULT_BLEND_RADIUS_MM)
        layout.addRow("Радиус скругления (мм):", self.blend_spin)
        self.offset_spin = QtGui.QDoubleSpinBox()
        self.offset_spin.setRange(0.0, 100.0)
        self.offset_spin.setDecimals(1)
        self.offset_spin.setValue(DEFAULT_TOOL_OFFSET_MM)
        layout.addRow("Смещение инструмента (мм):", self.offset_spin)
        self.rot_spin = QtGui.QDoubleSpinBox()
        self.rot_spin.setRange(-180.0, 180.0)
        self.rot_spin.setDecimals(1)
        self.rot_spin.setValue(DEFAULT_TOOL_ROTATION_DEG)
        layout.addRow("Поворот инструмента (°):", self.rot_spin)
        self.vel_spin = QtGui.QDoubleSpinBox()
        self.vel_spin.setRange(0.001, 1.0)
        self.vel_spin.setDecimals(3)
        self.vel_spin.setValue(DEFAULT_VELOCITY)
        layout.addRow("Скорость (м/с):", self.vel_spin)
        self.acc_spin = QtGui.QDoubleSpinBox()
        self.acc_spin.setRange(0.001, 2.0)
        self.acc_spin.setDecimals(3)
        self.acc_spin.setValue(DEFAULT_ACCELERATION)
        layout.addRow("Ускорение (м/с²):", self.acc_spin)

        # ✅ НОВЫЙ ЧЕКБОКС
        self.use_movej_checkbox = QtGui.QCheckBox()
        self.use_movej_checkbox.setChecked(False)  # По умолчанию movel
        layout.addRow("Использовать movej для подхода:", self.use_movej_checkbox)


        buttons = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)
        self.setLayout(layout)

    def get_values(self):
        return {
            'discretization': self.discr_spin.value(),
            'safe_height': self.safe_spin.value(),
            'blend_radius': self.blend_spin.value(),
            'tool_offset': self.offset_spin.value(),
            'tool_rotation': self.rot_spin.value(),
            'velocity': self.vel_spin.value(),
            'acceleration': self.acc_spin.value(),
            'use_movej': self.use_movej_checkbox.isChecked()  # ✅ НОВОЕ
        }




class FinishOrientationDialog(QtGui.QDialog):
    """Диалог интерактивной настройки траектории (геометрия) + ориентации на выходе (для незамкнутых)."""

    def __init__(self, parent=None,
                 discretization_mm=0.2,
                 safe_height_mm=200.0,
                 blend_radius_mm=0.001,
                 tool_offset_mm=5.0,
                 start_roll_deg=0.0,
                 end_roll_deg=0.0,
                 end_tilt_along_deg=0.0,
                 end_tilt_across_deg=0.0):
        super(FinishOrientationDialog, self).__init__(parent if parent else Gui.getMainWindow())
        self.setWindowTitle("Параметры траектории и ориентации")
        self.resize(560, 320)

        self._pending_recalc = False
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._on_timer)

        layout = QtGui.QVBoxLayout()

        info = QtGui.QLabel(
            "Меняйте параметры траектории и ориентацию инструмента.\n"
            "Пересчёт применяется к текущей выбранной геометрии и перерисовывает слой."
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        # --- Параметры траектории ---
        grp_path = QtGui.QGroupBox("Параметры траектории (геометрия)")
        form_path = QtGui.QFormLayout()

        self.discr_spin = QtGui.QDoubleSpinBox()
        self.discr_spin.setRange(0.01, 10.0)
        self.discr_spin.setDecimals(2)
        self.discr_spin.setValue(float(discretization_mm))
        form_path.addRow("Дискретизация (мм):", self.discr_spin)

        self.safe_spin = QtGui.QDoubleSpinBox()
        self.safe_spin.setRange(0.0, 1500.0)
        self.safe_spin.setDecimals(1)
        self.safe_spin.setValue(float(safe_height_mm))
        form_path.addRow("Безопасная высота (мм):", self.safe_spin)

        self.blend_spin = QtGui.QDoubleSpinBox()
        self.blend_spin.setRange(0.0, 10.0)
        self.blend_spin.setDecimals(3)
        self.blend_spin.setValue(float(blend_radius_mm))
        form_path.addRow("Радиус скругления (мм):", self.blend_spin)

        self.offset_spin = QtGui.QDoubleSpinBox()
        self.offset_spin.setRange(0.0, 100.0)
        self.offset_spin.setDecimals(1)
        self.offset_spin.setValue(float(tool_offset_mm))
        form_path.addRow("Смещение инструмента (мм):", self.offset_spin)

        self.roll_start_spin = QtGui.QDoubleSpinBox()
        self.roll_start_spin.setRange(-180.0, 180.0)
        self.roll_start_spin.setDecimals(1)
        self.roll_start_spin.setValue(float(start_roll_deg))
        form_path.addRow("Поворот инструмента (начальный, °):", self.roll_start_spin)

        grp_path.setLayout(form_path)
        layout.addWidget(grp_path)

        # --- Ориентация выхода ---
        grp_orient = QtGui.QGroupBox("Ориентация на выходе")
        form_or = QtGui.QFormLayout()

        self.roll_end_spin = QtGui.QDoubleSpinBox()
        self.roll_end_spin.setRange(-180.0, 180.0)
        self.roll_end_spin.setDecimals(1)
        self.roll_end_spin.setValue(float(end_roll_deg))
        form_or.addRow("Поворот инструмента на выходе (°):", self.roll_end_spin)

        self.tilt_along_spin = QtGui.QDoubleSpinBox()
        self.tilt_along_spin.setRange(-180, 180) # этьо углы вектора выхода
        self.tilt_along_spin.setDecimals(1)
        self.tilt_along_spin.setValue(float(end_tilt_along_deg))
        form_or.addRow("Наклон вдоль траектории на выходе (°):", self.tilt_along_spin)

        self.tilt_across_spin = QtGui.QDoubleSpinBox()
        self.tilt_across_spin.setRange(-180, 180) # этьо углы вектора выхода
        self.tilt_across_spin.setDecimals(1)
        self.tilt_across_spin.setValue(float(end_tilt_across_deg))
        form_or.addRow("Наклон поперёк траектории на выходе (°):", self.tilt_across_spin)

        grp_orient.setLayout(form_or)
        layout.addWidget(grp_orient)

        layout.addStretch()

        # Кнопки
        btns = QtGui.QHBoxLayout()

        self.ok_btn = QtGui.QPushButton("✅ ОК")
        self.ok_btn.setMinimumHeight(36)
        self.ok_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")

        self.recalc_btn = QtGui.QPushButton("🔄 Пересчитать")
        self.recalc_btn.setMinimumHeight(36)

        self.cancel_btn = QtGui.QPushButton("❌ Выход")
        self.cancel_btn.setMinimumHeight(36)
        self.cancel_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")

        btns.addWidget(self.ok_btn)
        btns.addWidget(self.recalc_btn)
        btns.addWidget(self.cancel_btn)
        layout.addLayout(btns)

        self.setLayout(layout)

        # Авто‑пересчёт при изменениях
        self.discr_spin.valueChanged.connect(self.request_recalc)
        self.safe_spin.valueChanged.connect(self.request_recalc)
        self.blend_spin.valueChanged.connect(self.request_recalc)
        self.offset_spin.valueChanged.connect(self.request_recalc)
        self.roll_start_spin.valueChanged.connect(self.request_recalc)

        self.roll_end_spin.valueChanged.connect(self.request_recalc)
        self.tilt_along_spin.valueChanged.connect(self.request_recalc)
        self.tilt_across_spin.valueChanged.connect(self.request_recalc)

    def request_recalc(self):
        self._pending_recalc = True
        self._timer.start(200)

    def _on_timer(self):
        if self._pending_recalc:
            self._pending_recalc = False
            self.emit(QtCore.SIGNAL("recalcRequested()"))

    def get_values(self):
        return {
            'discretization_mm': self.discr_spin.value(),
            'safe_height_mm': self.safe_spin.value(),
            'blend_radius_mm': self.blend_spin.value(),
            'tool_offset_mm': self.offset_spin.value(),
            'roll_start_deg': self.roll_start_spin.value(),

            'roll_end_deg': self.roll_end_spin.value(),
            'tilt_along_end_deg': self.tilt_along_spin.value(),
            'tilt_across_end_deg': self.tilt_across_spin.value(),
        }


class WeldingPathGenerator:
    def __init__(self):
        self.weld_wire = None
        self.tool_wire = None
        self.tool_vector_start = None
        self.tool_vector_end = None
        self.discretization = DEFAULT_DISCRETIZATION_MM
        self.safe_height = DEFAULT_SAFE_HEIGHT_MM
        self.blend_radius = DEFAULT_BLEND_RADIUS_MM
        self.tool_offset = DEFAULT_TOOL_OFFSET_MM
        self.tool_rotation = DEFAULT_TOOL_ROTATION_DEG
        self.velocity = DEFAULT_VELOCITY
        self.acceleration = DEFAULT_ACCELERATION
        self.weld_points = []
        self.offset_points = []
        self.tool_vectors = []
        self.orientations = []
        self.layer_name = "welding_trajectory"  # По умолчанию
        self.layer_name = "welding_trajectory"

        # --- Параметры выхода (для НЕзамкнутых) ---
        self.finish_tool_rotation_deg = None    # угол на выходе (вокруг оси инструмента), °

        self.finish_tilt_along_deg = 0.0        # наклон вдоль траектории на выходе, °
        self.finish_tilt_across_deg = 0.0       # наклон поперёк траектории на выходе, °

        # Флаг замкнутости (можно выставлять в process_trajectory)
        self.is_closed_trajectory = False

        # Для интерактива/перерисовки (если вы это уже делали)
        self._vis_group_obj = None

        # --- CONFIG: load defaults from [Laser_welding] ---
        self.cfg = get_config()
        sec = self.cfg.section("Laser_welding")

        # Все ключи с weld_ префиксом
        self.discretization = sec.get_float("weld_discretization_mm", self.discretization)
        self.safe_height = sec.get_float("weld_safe_height_mm", self.safe_height)
        self.blend_radius = sec.get_float("weld_blend_radius_mm", self.blend_radius)
        self.tool_offset = sec.get_float("weld_tool_offset_mm", self.tool_offset)

        self.tool_rotation = sec.get_float("weld_roll_start_deg", self.tool_rotation)

        # Параметры выхода
        self.finish_tool_rotation_deg = sec.get_float("weld_roll_end_deg", float(self.tool_rotation))
        self.finish_tilt_along_deg = sec.get_float("weld_tilt_along_end_deg", self.finish_tilt_along_deg)
        self.finish_tilt_across_deg = sec.get_float("weld_tilt_across_end_deg", self.finish_tilt_across_deg)

        # Скорость/ускорение и подход
        self.velocity = sec.get_float("weld_speed", self.velocity)
        self.acceleration = sec.get_float("weld_accel", self.acceleration)
        self.use_movej = sec.get_bool("weld_use_movej", False)


    def start(self):
        App.Console.PrintMessage(f"\n{'=' * 60}\n")
        App.Console.PrintMessage("🔥 ГЕНЕРАТОР СВАРОЧНОЙ ТРАЕКТОРИИ v2.0\n")
        App.Console.PrintMessage(f"{'=' * 60}\n\n")

        dialog = ParametersDialog()

        # --- CONFIG -> UI defaults ---
        dialog.discr_spin.setValue(float(self.discretization))
        dialog.safe_spin.setValue(float(self.safe_height))
        dialog.blend_spin.setValue(float(self.blend_radius))
        dialog.offset_spin.setValue(float(self.tool_offset))
        dialog.rot_spin.setValue(float(self.tool_rotation))
        dialog.vel_spin.setValue(float(self.velocity))
        dialog.acc_spin.setValue(float(self.acceleration))
        dialog.use_movej_checkbox.setChecked(bool(self.use_movej))



        if dialog.exec_():
            params = dialog.get_values()
            self.discretization = params['discretization']
            self.safe_height = params['safe_height']
            self.blend_radius = params['blend_radius']
            self.tool_offset = params['tool_offset']
            self.tool_rotation = params['tool_rotation']

            self.finish_tool_rotation_deg = float(self.tool_rotation)
            self.finish_tilt_along_deg = 0.0
            self.finish_tilt_across_deg = 0.0



            self.velocity = params['velocity']
            self.acceleration = params['acceleration']
            self.use_movej = params['use_movej']  # ✅ ДОБАВЬ ЭТУ СТРОКУ

            App.Console.PrintMessage(f"✓ Параметры:\n")
            App.Console.PrintMessage(f"  Дискретизация: {self.discretization} мм\n")
            App.Console.PrintMessage(f"  Безопасная высота: {self.safe_height} мм\n")
            App.Console.PrintMessage(f"  Радиус: {self.blend_radius} мм\n")
            App.Console.PrintMessage(f"  Смещение: {self.tool_offset} мм\n")
            App.Console.PrintMessage(f"  Поворот: {self.tool_rotation}°\n")
            App.Console.PrintMessage(f"  v={self.velocity} м/с, a={self.acceleration} м/с²\n\n")

            App.Console.PrintMessage(f"  Подход: {'movej' if self.use_movej else 'movel'}\n\n")  # ✅ НОВОЕ

            # --- SAVE to [Laser_welding] after initial dialog ---
            sec = self.cfg.section("Laser_welding")
            sec.set("weld_discretization_mm", float(self.discretization))
            sec.set("weld_safe_height_mm", float(self.safe_height))
            sec.set("weld_blend_radius_mm", float(self.blend_radius))
            sec.set("weld_tool_offset_mm", float(self.tool_offset))

            sec.set("weld_roll_start_deg", float(self.tool_rotation))
            sec.set("weld_roll_end_deg", float(
                self.finish_tool_rotation_deg if self.finish_tool_rotation_deg is not None else self.tool_rotation))
            sec.set("weld_tilt_along_end_deg", float(self.finish_tilt_along_deg))
            sec.set("weld_tilt_across_end_deg", float(self.finish_tilt_across_deg))

            sec.set("weld_speed", float(self.velocity))
            sec.set("weld_accel", float(self.acceleration))
            sec.set("weld_use_movej", bool(self.use_movej))

            self.cfg.save()

            self.select_geometry()




        else:
            App.Console.PrintMessage("❌ Отменено\n")

    def select_geometry(self):
        App.Console.PrintMessage("📋 Шаг 1: Выбор сварочной линии\n")
        self.show_weld_line_dialog()

    def show_weld_line_dialog(self):
        dialog = QtGui.QDialog(Gui.getMainWindow())
        dialog.setWindowTitle("📋 Выбор сварочной линии")
        dialog.resize(400, 180)

        layout = QtGui.QVBoxLayout()
        info = QtGui.QLabel(
            "<h3>📍 Шаг 1: Выберите сварочную линию</h3>"
            "<p>Выберите Wire или Edge в модели (кликом на 3D)</p>"
        )
        info.setWordWrap(True)
        layout.addWidget(info)
        layout.addStretch()

        continue_btn = QtGui.QPushButton("✅ Линия выбрана")
        continue_btn.setMinimumHeight(40)
        continue_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")

        cancel_btn = QtGui.QPushButton("❌ Отмена")
        cancel_btn.setMinimumHeight(40)
        cancel_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")

        button_layout = QtGui.QHBoxLayout()
        button_layout.addWidget(continue_btn)
        button_layout.addWidget(cancel_btn)
        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        def on_continue():
            sel = Gui.Selection.getSelectionEx()
            if len(sel) == 0:
                QtGui.QMessageBox.warning(None, "Ошибка", "Выберите сварочную линию!")
                return

            weld_obj = sel[0]

            # ✅ ПРОВЕРКА 1: Есть SubObjects (клик в 3D на edge/wire)
            if hasattr(weld_obj, 'SubObjects') and len(weld_obj.SubObjects) > 0:
                shape = weld_obj.SubObjects[0]
                if hasattr(shape, 'Wires') and len(shape.Wires) > 0:
                    self.weld_wire = shape.Wires[0]
                elif hasattr(shape, 'Edges') and len(shape.Edges) > 0:
                    self.weld_wire = Part.Wire(shape.Edges)
                else:
                    self.weld_wire = shape

            # ✅ ПРОВЕРКА 2: Выбран целый объект (клик в дереве)
            elif hasattr(weld_obj.Object, 'Shape'):
                obj_shape = weld_obj.Object.Shape
                if hasattr(obj_shape, 'Wires') and len(obj_shape.Wires) > 0:
                    self.weld_wire = obj_shape.Wires[0]
                elif hasattr(obj_shape, 'Edges') and len(obj_shape.Edges) > 0:
                    self.weld_wire = Part.Wire(obj_shape.Edges)
                else:
                    self.weld_wire = obj_shape

            if not self.weld_wire:
                QtGui.QMessageBox.warning(None, "Ошибка", "Не удалось получить линию!")
                return

            App.Console.PrintMessage(f"✓ Сварочная линия выбрана\n\n")
            Gui.Selection.clearSelection()
            dialog.close()
            self.show_tool_vector_dialog()

        def on_cancel():
            dialog.close()
            App.Console.PrintMessage("❌ Отменено\n")

        continue_btn.clicked.connect(on_continue)
        cancel_btn.clicked.connect(on_cancel)
        dialog.setWindowModality(QtCore.Qt.NonModal)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def show_tool_vector_dialog(self):
        dialog = QtGui.QDialog(Gui.getMainWindow())
        dialog.setWindowTitle("📍 Выбор вектора инструмента")
        dialog.resize(400, 180)

        layout = QtGui.QVBoxLayout()
        info = QtGui.QLabel(
            "<h3>📍 Шаг 2: Выберите вектор инструмента</h3>"
            "<p>Выберите Wire или Edge в модели (кликом на 3D)</p>"
        )
        info.setWordWrap(True)
        layout.addWidget(info)
        layout.addStretch()

        continue_btn = QtGui.QPushButton("✅ Вектор выбран")
        continue_btn.setMinimumHeight(40)
        continue_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")

        cancel_btn = QtGui.QPushButton("❌ Отмена")
        cancel_btn.setMinimumHeight(40)
        cancel_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")

        button_layout = QtGui.QHBoxLayout()
        button_layout.addWidget(continue_btn)
        button_layout.addWidget(cancel_btn)
        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        def on_continue():
            App.Console.PrintMessage("[A] Кнопка нажата\n")

            sel = Gui.Selection.getSelectionEx()
            if len(sel) == 0:
                QtGui.QMessageBox.warning(None, "Ошибка", "Выберите вектор инструмента!")
                return

            App.Console.PrintMessage("[B] Получен выбор\n")

            tool_obj = sel[0]

            # ✅ ПРОВЕРКА 1: Есть SubObjects (клик в 3D)
            if hasattr(tool_obj, 'SubObjects') and len(tool_obj.SubObjects) > 0:
                App.Console.PrintMessage("[C1] SubObjects найдены\n")
                shape = tool_obj.SubObjects[0]
                if hasattr(shape, 'Wires') and len(shape.Wires) > 0:
                    self.tool_wire = shape.Wires[0]
                elif hasattr(shape, 'Edges') and len(shape.Edges) > 0:
                    self.tool_wire = Part.Wire(shape.Edges)
                else:
                    self.tool_wire = shape

            # ✅ ПРОВЕРКА 2: Выбран целый объект (клик в дереве)
            elif hasattr(tool_obj.Object, 'Shape'):
                App.Console.PrintMessage("[C2] Целый объект выбран\n")
                obj_shape = tool_obj.Object.Shape
                if hasattr(obj_shape, 'Wires') and len(obj_shape.Wires) > 0:
                    self.tool_wire = obj_shape.Wires[0]
                elif hasattr(obj_shape, 'Edges') and len(obj_shape.Edges) > 0:
                    self.tool_wire = Part.Wire(obj_shape.Edges)
                else:
                    self.tool_wire = obj_shape

            App.Console.PrintMessage("[D] tool_wire обработан\n")

            if not self.tool_wire:
                QtGui.QMessageBox.warning(None, "Ошибка", "Не удалось получить вектор!")
                return

            App.Console.PrintMessage(f"✓ Вектор инструмента выбран\n\n")

            App.Console.PrintMessage("[E] Очистка выбора\n")
            Gui.Selection.clearSelection()

            App.Console.PrintMessage("[F] Закрытие диалога\n")
            dialog.close()

            App.Console.PrintMessage("[G] Вызов determine_tool_vector_ends\n")
            self.determine_tool_vector_ends()

            App.Console.PrintMessage("[H] Вызов process_trajectory\n")
            self.process_trajectory()

            App.Console.PrintMessage("[I] Завершение\n")

        def on_cancel():
            dialog.close()
            App.Console.PrintMessage("❌ Отменено\n")

        continue_btn.clicked.connect(on_continue)
        cancel_btn.clicked.connect(on_cancel)
        dialog.setWindowModality(QtCore.Qt.NonModal)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def determine_tool_vector_ends(self):
        tool_start_point = self.tool_wire.Vertexes[0].Point
        tool_end_point = self.tool_wire.Vertexes[-1].Point
        weld_start_point = self.weld_wire.Vertexes[0].Point

        dist_start = tool_start_point.distanceToPoint(weld_start_point)
        dist_end = tool_end_point.distanceToPoint(weld_start_point)

        if dist_start < dist_end:
            self.tool_vector_end = np.array([tool_start_point.x, tool_start_point.y, tool_start_point.z])
            self.tool_vector_start = np.array([tool_end_point.x, tool_end_point.y, tool_end_point.z])
        else:
            self.tool_vector_end = np.array([tool_end_point.x, tool_end_point.y, tool_end_point.z])
            self.tool_vector_start = np.array([tool_start_point.x, tool_start_point.y, tool_start_point.z])

        App.Console.PrintMessage(f"✓ Дальний конец: [{self.tool_vector_start[0]:.1f}, "
                                 f"{self.tool_vector_start[1]:.1f}, {self.tool_vector_start[2]:.1f}]\n")
        App.Console.PrintMessage(f"✓ Точка касания: [{self.tool_vector_end[0]:.1f}, "
                                 f"{self.tool_vector_end[1]:.1f}, {self.tool_vector_end[2]:.1f}]\n\n")

    def process_trajectory(self):
        App.Console.PrintMessage("🔄 Обработка траектории...\n")
        try:
            App.Console.PrintMessage(">>> Вызов discretize_wire\n")
            self.discretize_wire()

            App.Console.PrintMessage(">>> Вызов check_weld_direction\n")
            self.check_weld_direction()

            App.Console.PrintMessage(">>> Вызов reorder_closed_trajectory\n")
            self.reorder_closed_trajectory()

            App.Console.PrintMessage(">>> Вызов calculate_orientations\n")
            self.calculate_orientations()

            App.Console.PrintMessage(">>> Вызов apply_tool_offset\n")
            self.apply_tool_offset()

            App.Console.PrintMessage(">>> Вызов visualize_trajectory\n")
            self.visualize_trajectory()

            # --- НОВОЕ: определяем замкнутость и выбираем следующий шаг ---
            # (не трогаем замкнутые — пропускаем интерактивные настройки)
            try:
                if len(self.weld_points) >= 2:
                    dist_closure = np.linalg.norm(self.weld_points[-1] - self.weld_points[0])
                    is_closed = dist_closure < 1.0
                else:
                    is_closed = False
            except Exception as e:
                App.Console.PrintMessage(f"⚠️ Не удалось проверить замкнутость: {e}\n")
                is_closed = False

            # Если вы добавляли self.is_closed_trajectory в calculate_orientations() — можно синхронизировать:
            try:
                self.is_closed_trajectory = bool(is_closed)
            except Exception:
                pass

            App.Console.PrintMessage(f">>> Trajectory closed = {is_closed}\n")

            if is_closed:
                QtGui.QMessageBox.information(
                    None,
                    "Замкнутая траектория",
                    "Траектория замкнута — настройки поворота инструмента недоступны."
                )
                App.Console.PrintMessage(">>> Вызов show_confirmation_dialog (closed)\n")
                self.show_confirmation_dialog()
                return

            # Незамкнутая: вместо show_confirmation_dialog вызываем окно настройки выхода
            App.Console.PrintMessage(">>> Вызов show_finish_orientation_dialog (open)\n")
            self.show_finish_orientation_dialog()
            return

        except Exception as e:
            App.Console.PrintError(f"❌ Ошибка: {e}\n")
            import traceback
            traceback.print_exc()

    def discretize_wire(self):
        try:
            points = self.weld_wire.discretize(Deflection=self.discretization)
            self.weld_points = []
            for point in points:
                self.weld_points.append(np.array([point.x, point.y, point.z]))
            wire_length = self.weld_wire.Length
            avg_segment = wire_length / (len(self.weld_points) - 1) if len(self.weld_points) > 1 else 0
            App.Console.PrintMessage(f"✓ Дискретизация: {len(self.weld_points)} точек\n")
            App.Console.PrintMessage(f"  Длина кривой: {wire_length:.1f} мм\n")
            App.Console.PrintMessage(f"  Средний сегмент: {avg_segment:.2f} мм\n")
        except Exception as e:
            App.Console.PrintError(f"Ошибка дискретизации: {e}\n")
            raise

    def check_weld_direction(self):
        weld_start = self.weld_points[0]
        weld_end = self.weld_points[-1]
        tool_touch = self.tool_vector_end
        dist_to_start = np.linalg.norm(tool_touch - weld_start)
        dist_to_end = np.linalg.norm(tool_touch - weld_end)
        App.Console.PrintMessage(f"  Расстояние до начала: {dist_to_start:.2f} мм\n")
        App.Console.PrintMessage(f"  Расстояние до конца: {dist_to_end:.2f} мм\n")
        if dist_to_end < dist_to_start:
            App.Console.PrintMessage("  ⚠️ Линия перевернута - начинаем с противоположного конца\n")
            self.weld_points.reverse()
        else:
            App.Console.PrintMessage("  ✓ Направление правильное\n")

    def reorder_closed_trajectory(self):
        """Переупорядочивает замкнутую траекторию (с защитой от дубликатов)"""
        if len(self.weld_points) < 3:
            return

        start_pt = self.weld_points[0]
        end_pt = self.weld_points[-1]

        # 1. Проверяем, замкнута ли траектория геометрически
        dist_closure = np.linalg.norm(end_pt - start_pt)
        is_closed_geo = dist_closure < 1.0

        if not is_closed_geo:
            App.Console.PrintMessage("  Траектория не замкнута (геометрически)\n")
            return

        # ✅ 2. УДАЛЯЕМ ПОСЛЕДНЮЮ ТОЧКУ, если она дублирует первую
        # Это критически важно, чтобы не получить [A, B, ... A, A, B]
        if dist_closure < 0.001:
            App.Console.PrintMessage("  Удаляем дублирующую конечную точку перед сдвигом\n")
            self.weld_points.pop()

            # 3. Ищем индекс ближайшей точки
        closest_idx = 0
        min_dist = float('inf')

        for i, pt in enumerate(self.weld_points):
            dist = np.linalg.norm(pt - self.tool_vector_end)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        if closest_idx == 0:
            App.Console.PrintMessage("  Точка касания уже первая\n")
            # Возвращаем удаленную точку, чтобы замкнуть обратно
            self.weld_points.append(self.weld_points[0].copy())
            return

        App.Console.PrintMessage(f"  Сдвиг массива с индекса {closest_idx}...\n")

        # 4. Сдвигаем массив (Cyclic roll)
        # Было: [A, B, C, D, E] (ближайшая C, idx=2)
        # Стало: [C, D, E, A, B]
        new_order = self.weld_points[closest_idx:] + self.weld_points[:closest_idx]
        self.weld_points = new_order

        # 5. Замыкаем траекторию (добавляем копию новой первой точки в конец)
        self.weld_points.append(self.weld_points[0].copy())

        App.Console.PrintMessage(f"  ✓ Траектория переупорядочена: {len(self.weld_points)} точек\n")

    def calculate_orientations(self):
        initial_tool_vec = self.tool_vector_end - self.tool_vector_start
        self.tool_vectors = []
        self.orientations = []

        # Check if closed
        start_point = self.weld_points[0]
        end_point = self.weld_points[-1]
        is_closed = np.linalg.norm(end_point - start_point) < 1.0
        self.is_closed_trajectory = bool(is_closed)

        # Базовые данные (без дополнительного изгиба выхода)
        self.base_tool_vectors = []
        self.base_tangents = []
        self.base_orientations = []

        App.Console.PrintMessage(f"\n{'=' * 60}\n")
        App.Console.PrintMessage(f"DEBUG: calculate_orientations()\n")
        App.Console.PrintMessage(f"{'=' * 60}\n")
        App.Console.PrintMessage(f"Total weld points: {len(self.weld_points)}\n")
        App.Console.PrintMessage(f"Closed: {is_closed} (distance: {np.linalg.norm(end_point - start_point):.2f} mm)\n")
        App.Console.PrintMessage(
            f"Initial tool vector: [{initial_tool_vec[0]:.2f}, {initial_tool_vec[1]:.2f}, {initial_tool_vec[2]:.2f}]\n")
        App.Console.PrintMessage(f"{'=' * 60}\n\n")

        # ✅ Find closest point index to tool_vector_end (where tool touches the curve)
        min_dist = float('inf')
        tool_touch_idx = 0
        for i, pt in enumerate(self.weld_points):
            dist = np.linalg.norm(pt - self.tool_vector_end)
            if dist < min_dist:
                min_dist = dist
                tool_touch_idx = i

        App.Console.PrintMessage(f"Tool touches curve at index {tool_touch_idx} (distance: {min_dist:.2f} mm)\n")

        # ✅ Calculate tangent at tool touch point (REFERENCE tangent)
        if tool_touch_idx == 0:
            reference_tangent = self.weld_points[1] - self.weld_points[0]
        elif tool_touch_idx == len(self.weld_points) - 1:
            reference_tangent = self.weld_points[-1] - self.weld_points[-2]
        else:
            # Average of adjacent segments for smoothness
            prev_seg = self.weld_points[tool_touch_idx] - self.weld_points[tool_touch_idx - 1]
            next_seg = self.weld_points[tool_touch_idx + 1] - self.weld_points[tool_touch_idx]
            reference_tangent = (prev_seg / np.linalg.norm(prev_seg) + next_seg / np.linalg.norm(next_seg))

        reference_tangent = reference_tangent / np.linalg.norm(reference_tangent)
        App.Console.PrintMessage(
            f"Reference tangent (at tool touch): [{reference_tangent[0]:.4f}, {reference_tangent[1]:.4f}, {reference_tangent[2]:.4f}]\n")

        # First segment direction (for debug)
        first_segment_dir = self.weld_points[1] - self.weld_points[0]
        first_segment_dir = first_segment_dir / np.linalg.norm(first_segment_dir)
        App.Console.PrintMessage(
            f"First segment direction: [{first_segment_dir[0]:.4f}, {first_segment_dir[1]:.4f}, {first_segment_dir[2]:.4f}]\n")
        App.Console.PrintMessage(
            f"First weld point: [{self.weld_points[0][0]:.2f}, {self.weld_points[0][1]:.2f}, {self.weld_points[0][2]:.2f}]\n\n")

        # ✅ Process ALL points (including last)
        for i in range(len(self.weld_points)):
            # Determine current tangent
            if i == 0:
                curr_tangent = self.weld_points[1] - self.weld_points[0]
            elif i == len(self.weld_points) - 1:
                curr_tangent = self.weld_points[-1] - self.weld_points[-2]
                if is_closed:
                    # For closed, average with first tangent
                    first_tang = self.weld_points[1] - self.weld_points[0]
                    first_tang = first_tang / np.linalg.norm(first_tang)
                    curr_tangent = curr_tangent / np.linalg.norm(curr_tangent)
                    curr_tangent = (curr_tangent + first_tang) / 2.0
            else:
                # Middle points - average of adjacent segments
                prev_seg = self.weld_points[i] - self.weld_points[i - 1]
                next_seg = self.weld_points[i + 1] - self.weld_points[i]
                curr_tangent = (prev_seg / np.linalg.norm(prev_seg) + next_seg / np.linalg.norm(next_seg))

            curr_tangent = curr_tangent / np.linalg.norm(curr_tangent)

            # ✅ Rotate from REFERENCE tangent (where tool was specified), not from first segment!
            rotated_tool_vec = self.rotate_vector_between_directions(
                initial_tool_vec,
                reference_tangent,
                curr_tangent
            )

            self.base_tool_vectors.append(rotated_tool_vec)
            self.base_tangents.append(curr_tangent)

            # Calculate base orientation (вращение на входе одинаковое по всей траектории)
            roll, pitch, yaw = self.calculate_orientation(
                rotated_tool_vec, curr_tangent, math.radians(self.tool_rotation)
            )
            self.base_orientations.append((roll, pitch, yaw))

            # Debug output for first/last points
            if i <= 2 or i >= len(self.weld_points) - 3:
                App.Console.PrintMessage(f"Point {i}: tool_vector added, orientation calculated\n")
                App.Console.PrintMessage(
                    f"  Vector: [{rotated_tool_vec[0]:.2f}, {rotated_tool_vec[1]:.2f}, {rotated_tool_vec[2]:.2f}]\n")

        App.Console.PrintMessage(f"\n{'=' * 60}\n")
        App.Console.PrintMessage(f"FINAL COUNTS:\n")
        App.Console.PrintMessage(f"  Weld points: {len(self.weld_points)}\n")
        App.Console.PrintMessage(f"  Tool vectors: {len(self.tool_vectors)}\n")
        App.Console.PrintMessage(f"  Orientations: {len(self.orientations)}\n")
        App.Console.PrintMessage(f"{'=' * 60}\n\n")

        # Переносим базовые данные в рабочие поля
        self.tool_vectors = list(self.base_tool_vectors)
        self.orientations = list(self.base_orientations)

        if is_closed:
            first_vec = self.base_tool_vectors[0]
            last_vec = self.base_tool_vectors[-1]
            vec_diff = np.linalg.norm(last_vec - first_vec)
            App.Console.PrintMessage(f"✅ Closure check: first-last vector difference = {vec_diff:.2f} mm\n")

    def rotate_vector_between_directions(self, vector, from_dir, to_dir):
        from_dir = from_dir / np.linalg.norm(from_dir)
        to_dir = to_dir / np.linalg.norm(to_dir)
        dot = np.dot(from_dir, to_dir)
        if dot > 0.9999:
            return vector.copy()
        if dot < -0.9999:
            if abs(from_dir[0]) < 0.9:
                axis = np.array([1, 0, 0])
            else:
                axis = np.array([0, 1, 0])
            axis = np.cross(from_dir, axis)
            axis = axis / np.linalg.norm(axis)
            angle = np.pi
        else:
            axis = np.cross(from_dir, to_dir)
            axis = axis / np.linalg.norm(axis)
            angle = np.arccos(np.clip(dot, -1.0, 1.0))
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        rotated = (vector * cos_a + np.cross(axis, vector) * sin_a +
                   axis * np.dot(axis, vector) * (1 - cos_a))
        return rotated

    def calculate_orientation(self, tool_vector, segment_direction, additional_rotation):
        z_axis = tool_vector / np.linalg.norm(tool_vector)
        x_proj = segment_direction - np.dot(segment_direction, z_axis) * z_axis
        if np.linalg.norm(x_proj) < 1e-6:
            x_proj = np.array([1, 0, 0])
        x_axis = x_proj / np.linalg.norm(x_proj)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        cos_r = math.cos(additional_rotation + TOOL_ROTATION_CORRECTION_RAD)
        sin_r = math.sin(additional_rotation + TOOL_ROTATION_CORRECTION_RAD)
        x_axis_rot = x_axis * cos_r + y_axis * sin_r
        y_axis_rot = -x_axis * sin_r + y_axis * cos_r
        R = np.column_stack((x_axis_rot, y_axis_rot, z_axis))
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        if sy > 1e-6:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            roll = math.atan2(-R[1, 2], R[1, 1])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = 0
        return roll, pitch, yaw



    def rotate_vector_axis_angle(self, vector, axis, angle_rad):
        """Поворот вектора вокруг оси (формула Родрига)"""
        axis = np.array(axis, dtype=float)
        if np.linalg.norm(axis) < 1e-9:
            return vector.copy()
        axis = axis / np.linalg.norm(axis)
        v = np.array(vector, dtype=float)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        return v * cos_a + np.cross(axis, v) * sin_a + axis * np.dot(axis, v) * (1 - cos_a)

    def apply_finish_adjustments(self):
        """Применяет плавную корректировку ориентации от входа (s=0) к выходу (s=1)."""
        if self.is_closed_trajectory:
            return
        if not self.base_tool_vectors or not self.base_tangents:
            return
        n = len(self.base_tool_vectors)
        if n < 2:
            self.tool_vectors = list(self.base_tool_vectors)
            self.orientations = list(self.base_orientations)
            return

        start_roll_deg = float(self.tool_rotation)
        end_roll_deg = float(self.finish_tool_rotation_deg if self.finish_tool_rotation_deg is not None else self.tool_rotation)
        end_tilt_along_deg = float(self.finish_tilt_along_deg)
        end_tilt_across_deg = float(self.finish_tilt_across_deg)

        tool_vectors = []
        orientations = []

        for i in range(n):
            s = i / float(n - 1)

            base_tool_vec = np.array(self.base_tool_vectors[i], dtype=float)
            t = np.array(self.base_tangents[i], dtype=float)

            # Нормализация
            if np.linalg.norm(base_tool_vec) < 1e-9:
                base_tool_vec = np.array([0.0, 0.0, 1.0])
            if np.linalg.norm(t) < 1e-9:
                t = np.array([1.0, 0.0, 0.0])

            z = base_tool_vec / np.linalg.norm(base_tool_vec)
            t = t / np.linalg.norm(t)

            # Оси для "наклона":
            # U = поперёк траектории, V = вдоль (в плоскости (Z, T))
            u = np.cross(z, t)
            if np.linalg.norm(u) < 1e-9:
                # если почти параллельно, выбираем любую ось, ортогональную Z
                tmp = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
                u = np.cross(z, tmp)
            u = u / np.linalg.norm(u)
            v = np.cross(u, z)
            if np.linalg.norm(v) < 1e-9:
                v = t
            else:
                v = v / np.linalg.norm(v)

            # Плавные углы к выходу
            ang_along = math.radians(end_tilt_along_deg) * s
            ang_across = math.radians(end_tilt_across_deg) * s

            z1 = self.rotate_vector_axis_angle(z, u, ang_along)
            z2 = self.rotate_vector_axis_angle(z1, v, ang_across)

            tool_vec = z2 * np.linalg.norm(base_tool_vec)
            tool_vectors.append(tool_vec)

            roll_deg = start_roll_deg + (end_roll_deg - start_roll_deg) * s
            roll, pitch, yaw = self.calculate_orientation(tool_vec, t, math.radians(roll_deg))
            orientations.append((roll, pitch, yaw))

        self.tool_vectors = tool_vectors
        self.orientations = orientations

    def recalc_and_redraw(self):
        """Полный пересчёт траектории + ориентаций + оффсета и обновление визуализации."""
        App.Console.PrintMessage("🔄 Пересчёт траектории по новым параметрам...\n")

        # Полная цепочка как в process_trajectory(), только без открытия новых окон
        self.discretize_wire()
        self.check_weld_direction()
        self.reorder_closed_trajectory()
        self.calculate_orientations()

        # Применяем плавные изменения на выход (для незамкнутых)
        self.apply_finish_adjustments()

        self.apply_tool_offset()
        self.visualize_trajectory()

    def show_finish_orientation_dialog(self):
        """Показывает диалог интерактивной настройки траектории+выхода и затем продолжает сохранение."""
        # Инициализация значений по умолчанию
        if self.finish_tool_rotation_deg is None:
            self.finish_tool_rotation_deg = float(self.tool_rotation)

        dialog = FinishOrientationDialog(
            parent=Gui.getMainWindow(),

            discretization_mm=float(self.discretization),
            safe_height_mm=float(self.safe_height),
            blend_radius_mm=float(self.blend_radius),
            tool_offset_mm=float(self.tool_offset),

            start_roll_deg=float(self.tool_rotation),
            end_roll_deg=float(self.finish_tool_rotation_deg),
            end_tilt_along_deg=float(self.finish_tilt_along_deg),
            end_tilt_across_deg=float(self.finish_tilt_across_deg),
        )

        def apply_values_from_dialog():
            vals = dialog.get_values()

            # Параметры траектории (геометрия)
            self.discretization = float(vals['discretization_mm'])
            self.safe_height = float(vals['safe_height_mm'])
            self.blend_radius = float(vals['blend_radius_mm'])
            self.tool_offset = float(vals['tool_offset_mm'])

            # Начальный поворот инструмента
            self.tool_rotation = float(vals['roll_start_deg'])

            # Параметры выхода
            self.finish_tool_rotation_deg = float(vals['roll_end_deg'])
            self.finish_tilt_along_deg = float(vals['tilt_along_end_deg'])
            self.finish_tilt_across_deg = float(vals['tilt_across_end_deg'])

        def on_recalc():
            apply_values_from_dialog()

            # --- SAVE on every recalc ---
            sec = self.cfg.section("Laser_welding")
            sec.set("weld_discretization_mm", float(self.discretization))
            sec.set("weld_safe_height_mm", float(self.safe_height))
            sec.set("weld_blend_radius_mm", float(self.blend_radius))
            sec.set("weld_tool_offset_mm", float(self.tool_offset))

            sec.set("weld_roll_start_deg", float(self.tool_rotation))
            sec.set("weld_roll_end_deg", float(
                self.finish_tool_rotation_deg if self.finish_tool_rotation_deg is not None else self.tool_rotation))
            sec.set("weld_tilt_along_end_deg", float(self.finish_tilt_along_deg))
            sec.set("weld_tilt_across_end_deg", float(self.finish_tilt_across_deg))

            sec.set("weld_speed", float(self.velocity))
            sec.set("weld_accel", float(self.acceleration))
            sec.set("weld_use_movej", bool(self.use_movej))

            self.cfg.save()



            self.recalc_and_redraw()

        def on_ok():
            apply_values_from_dialog()

            # --- SAVE on every recalc ---
            sec = self.cfg.section("Laser_welding")
            sec.set("weld_discretization_mm", float(self.discretization))
            sec.set("weld_safe_height_mm", float(self.safe_height))
            sec.set("weld_blend_radius_mm", float(self.blend_radius))
            sec.set("weld_tool_offset_mm", float(self.tool_offset))

            sec.set("weld_roll_start_deg", float(self.tool_rotation))
            sec.set("weld_roll_end_deg", float(
                self.finish_tool_rotation_deg if self.finish_tool_rotation_deg is not None else self.tool_rotation))
            sec.set("weld_tilt_along_end_deg", float(self.finish_tilt_along_deg))
            sec.set("weld_tilt_across_end_deg", float(self.finish_tilt_across_deg))

            sec.set("weld_speed", float(self.velocity))
            sec.set("weld_accel", float(self.acceleration))
            sec.set("weld_use_movej", bool(self.use_movej))

            self.cfg.save()



            self.recalc_and_redraw()
            dialog.close()
            self.show_confirmation_dialog()

        def on_cancel():
            App.Console.PrintMessage("❌ Отменено (настройка параметров траектории/выхода)\n")
            dialog.close()

        dialog.ok_btn.clicked.connect(on_ok)
        dialog.recalc_btn.clicked.connect(on_recalc)
        dialog.cancel_btn.clicked.connect(on_cancel)

        # Авто‑пересчёт по таймеру
        QtCore.QObject.connect(dialog, QtCore.SIGNAL("recalcRequested()"), on_recalc)

        dialog.setWindowModality(QtCore.Qt.NonModal)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def apply_tool_offset(self):
        self.offset_points = []
        for i in range(len(self.weld_points) - 1):
            tool_vec = self.tool_vectors[i]
            offset_vec = (tool_vec / np.linalg.norm(tool_vec)) * (-self.tool_offset)
            offset_point = self.weld_points[i] + offset_vec
            self.offset_points.append(offset_point)
        last_tool_vec = self.tool_vectors[-1]
        last_offset_vec = (last_tool_vec / np.linalg.norm(last_tool_vec)) * (-self.tool_offset)
        last_offset_point = self.weld_points[-1] + last_offset_vec
        self.offset_points.append(last_offset_point)
        App.Console.PrintMessage(f"✓ Применено смещение: {self.tool_offset} мм\n")

    def get_unique_layer_name(self, base_name="welding_trajectory"):
        """✅ Генерирует уникальное имя слоя с индексом"""
        doc = App.ActiveDocument
        if not doc:
            return base_name

        # Ищем существующие слои с таким базовым именем
        existing_indices = []
        for obj in doc.Objects:
            if obj.Name.startswith(base_name):
                # Извлекаем индекс (если есть)
                suffix = obj.Name[len(base_name):]
                if suffix == "":
                    existing_indices.append(0)  # Первый без индекса
                elif suffix.startswith("_"):
                    try:
                        idx = int(suffix[1:])
                        existing_indices.append(idx)
                    except:
                        pass

        # Находим первый свободный индекс
        if not existing_indices:
            return base_name  # Первый раз - без индекса

        next_index = max(existing_indices) + 1
        return f"{base_name}_{next_index}"

    def visualize_trajectory(self):
        doc = App.ActiveDocument
        if not doc:
            return

        # --- ВАЖНО: не плодим слои, а очищаем и перерисовываем текущий ---
        vis_group = None
        if self._vis_group_obj is not None:
            try:
                vis_group = doc.getObject(self._vis_group_obj.Name)
            except Exception:
                vis_group = None

        if vis_group is not None:
            # удаляем все объекты внутри группы
            try:
                for obj in list(vis_group.Group):
                    try:
                        doc.removeObject(obj.Name)
                    except Exception:
                        pass
                doc.recompute()
            except Exception:
                pass
        else:
            # первый раз создаём слой
            if not self.layer_name or self.layer_name == "welding_trajectory":
                self.layer_name = self.get_unique_layer_name("welding_trajectory")

            vis_group = doc.addObject("App::DocumentObjectGroup", self.layer_name)
            self._vis_group_obj = vis_group
            App.Console.PrintMessage(f"✓ Создан слой: {self.layer_name}\n")

        from FreeCAD import Vector

        # Точки исходной траектории (оранжевые)
        for i, pt in enumerate(self.weld_points):
            sphere = Part.makeSphere(2.0, Vector(*pt))
            pt_obj = doc.addObject("Part::Feature", f"weld_pt_{i}")
            pt_obj.Shape = sphere
            if hasattr(pt_obj, "ViewObject"):
                pt_obj.ViewObject.ShapeColor = (1.0, 0.5, 0.0)
            vis_group.addObject(pt_obj)

        # Точки смещённой траектории (красные)
        for i, pt in enumerate(self.offset_points):
            sphere = Part.makeSphere(2.5, Vector(*pt))
            pt_obj = doc.addObject("Part::Feature", f"offset_pt_{i}")
            pt_obj.Shape = sphere
            if hasattr(pt_obj, "ViewObject"):
                pt_obj.ViewObject.ShapeColor = (1.0, 0.0, 0.0)
            vis_group.addObject(pt_obj)

        # Векторы инструмента (голубые)
        for i, pt in enumerate(self.weld_points):
            if i < len(self.tool_vectors):
                tool_vec = self.tool_vectors[i]
                vec_start = Vector(*pt)
                vec_end = Vector(*(pt - tool_vec))
                tool_line = Part.makeLine(vec_start, vec_end)
                tool_obj = doc.addObject("Part::Feature", f"tool_vec_{i}")
                tool_obj.Shape = tool_line
                if hasattr(tool_obj, "ViewObject"):
                    tool_obj.ViewObject.LineColor = (0.0, 0.5, 1.0)
                    tool_obj.ViewObject.LineWidth = 2.0
                vis_group.addObject(tool_obj)

        # --- НОВОЕ: маркеры "поворота инструмента" (roll) для 1-го и последнего вектора ---
        try:
            if len(self.weld_points) > 0 and len(self.tool_vectors) > 0 and len(self.orientations) > 0:
                i0 = 0
                i1 = min(len(self.weld_points) - 1, len(self.tool_vectors) - 1, len(self.orientations) - 1)

                self._draw_roll_marker(
                    vis_group=vis_group,
                    pt_np=self.weld_points[i0],
                    tool_vec_np=self.tool_vectors[i0],
                    rpy=self.orientations[i0],
                    name_prefix="roll_start",
                    size_mm=12.0
                )

                self._draw_roll_marker(
                    vis_group=vis_group,
                    pt_np=self.weld_points[i1],
                    tool_vec_np=self.tool_vectors[i1],
                    rpy=self.orientations[i1],
                    name_prefix="roll_end",
                    size_mm=12.0
                )
        except Exception as e:
            App.Console.PrintMessage(f"⚠️ Не удалось нарисовать маркеры roll: {e}\n")

        # Безопасная плоскость
        safe_plane = Part.makePlane(500, 500, Vector(-250, -250, self.safe_height))
        safe_obj = doc.addObject("Part::Feature", "safe_plane")
        safe_obj.Shape = safe_plane
        if hasattr(safe_obj, "ViewObject"):
            safe_obj.ViewObject.ShapeColor = (0.0, 1.0, 0.0)
            safe_obj.ViewObject.Transparency = 70
        vis_group.addObject(safe_obj)

        doc.recompute()
        App.Console.PrintMessage("✓ Визуализация готова\n")
        App.Console.PrintMessage(" 🔵 Голубые линии = векторы инструмента\n")
        App.Console.PrintMessage(" 🟠 Оранжевые точки = оригинальная траектория\n")
        App.Console.PrintMessage(" 🔴 Красные точки = смещенная траектория (реальная)\n\n")

    def _draw_roll_marker(self, vis_group, pt_np, tool_vec_np, rpy, name_prefix="roll_mark", size_mm=12.0):
        """
        Рисует маркер "поворота инструмента" (roll вокруг оси инструмента) в виде стрелки
        у конца (tip) вектора инструмента.
        pt_np        - точка траектории (numpy [x,y,z] в мм)
        tool_vec_np  - вектор инструмента (numpy [x,y,z] в мм), как в tool_vectors
        rpy          - (roll, pitch, yaw) в радианах, как в self.orientations
        """
        try:
            from FreeCAD import Vector

            roll, pitch, yaw = rpy

            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)

            # R = Rz(yaw) * Ry(pitch) * Rx(roll)
            Rz = np.array([[cy, -sy, 0.0],
                           [sy, cy, 0.0],
                           [0.0, 0.0, 1.0]], dtype=float)
            Ry = np.array([[cp, 0.0, sp],
                           [0.0, 1.0, 0.0],
                           [-sp, 0.0, cp]], dtype=float)
            Rx = np.array([[1.0, 0.0, 0.0],
                           [0.0, cr, -sr],
                           [0.0, sr, cr]], dtype=float)

            R = Rz.dot(Ry).dot(Rx)

            # "стрелка" будет показывать направление оси X инструмента (колонка 0 матрицы)
            x_axis = R[:, 0]
            if np.linalg.norm(x_axis) < 1e-9:
                return
            x_axis = x_axis / np.linalg.norm(x_axis)

            # Конец (tip) вектора инструмента: как у вас рисуется линия vec_end = pt - tool_vec
            tip = pt_np - tool_vec_np

            tip_v = Vector(float(tip[0]), float(tip[1]), float(tip[2]))
            end_v = Vector(float(tip[0] - x_axis[0] * size_mm),
                           float(tip[1] - x_axis[1] * size_mm),
                           float(tip[2] - x_axis[2] * size_mm))

            # Основная "стрелка" (линия)
            main_line = Part.makeLine(tip_v, end_v)
            main_obj = App.ActiveDocument.addObject("Part::Feature", f"{name_prefix}_main")
            main_obj.Shape = main_line
            if hasattr(main_obj, "ViewObject"):
                main_obj.ViewObject.LineColor = (1.0, 0.0, 1.0)  # фиолетовый
                main_obj.ViewObject.LineWidth = 3.0
            vis_group.addObject(main_obj)

            # Небольшая "галочка" на конце (простая стрелка)
            head = size_mm * 0.35
            # направление для "усиков" — любая ось, перпендикулярная x_axis
            z = np.array(tool_vec_np, dtype=float)
            if np.linalg.norm(z) < 1e-9:
                z = np.array([0.0, 0.0, 1.0])
            z = z / np.linalg.norm(z)

            side = np.cross(x_axis, z)
            if np.linalg.norm(side) < 1e-9:
                # fallback
                side = np.array([0.0, 1.0, 0.0])
            side = side / np.linalg.norm(side)

            p1 = Vector(float(end_v.x - x_axis[0] * head + side[0] * head),
                        float(end_v.y - x_axis[1] * head + side[1] * head),
                        float(end_v.z - x_axis[2] * head + side[2] * head))
            p2 = Vector(float(end_v.x - x_axis[0] * head - side[0] * head),
                        float(end_v.y - x_axis[1] * head - side[1] * head),
                        float(end_v.z - x_axis[2] * head - side[2] * head))

            l1 = Part.makeLine(end_v, p1)
            l2 = Part.makeLine(end_v, p2)

            h1 = App.ActiveDocument.addObject("Part::Feature", f"{name_prefix}_head1")
            h1.Shape = l1
            h2 = App.ActiveDocument.addObject("Part::Feature", f"{name_prefix}_head2")
            h2.Shape = l2

            for o in (h1, h2):
                if hasattr(o, "ViewObject"):
                    o.ViewObject.LineColor = (1.0, 0.0, 1.0)
                    o.ViewObject.LineWidth = 3.0
                vis_group.addObject(o)

        except Exception as e:
            App.Console.PrintMessage(f"⚠️ Не удалось нарисовать маркер roll: {e}\n")

    def show_confirmation_dialog(self):
        dialog = QtGui.QDialog(Gui.getMainWindow())
        dialog.setWindowTitle("Подтверждение")
        dialog.resize(500, 300)
        layout = QtGui.QVBoxLayout()
        info_text = (
            f"**Параметры программы:**\n\n"
            f"Точек: {len(self.offset_points)}\n"
            f"Длина: {self.weld_wire.Length:.1f} мм\n"
            f"Смещение: {self.tool_offset} мм\n"
            f"Высота: {self.safe_height} мм\n"
            f"v={self.velocity} м/с, a={self.acceleration} м/с²\n"
        )
        info_label = QtGui.QLabel(info_text)
        info_label.setWordWrap(True)
        layout.addWidget(info_label)
        layout.addStretch()
        button_layout = QtGui.QHBoxLayout()
        send_button = QtGui.QPushButton("💾 Сохранить")
        send_button.setMinimumHeight(40)
        send_button.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")
        cancel_button = QtGui.QPushButton("❌ Отменить")
        cancel_button.setMinimumHeight(40)
        cancel_button.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")
        button_layout.addWidget(send_button)
        button_layout.addWidget(cancel_button)
        layout.addLayout(button_layout)
        dialog.setLayout(layout)

        def on_send():
            self.generate_and_save_program()
            dialog.close()

        def on_cancel():
            App.Console.PrintMessage("❌ Отменено\n")
            dialog.close()

        send_button.clicked.connect(on_send)
        cancel_button.clicked.connect(on_cancel)
        dialog.setWindowModality(QtCore.Qt.NonModal)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dialog.show()
        dialog.raise_()

    def generate_and_save_program(self):
        program = 'def welding_program():\n'

        # ✅ 1. НАЧАЛО НА БЕЗОПАСНОЙ ВЫСОТЕ (НОВОЕ!)
        approach_start = (self.tool_vector_start / 1000.0)
        r0, p0, y0 = self.orientations[0]

        safe_start = np.array([approach_start[0], approach_start[1], self.safe_height / 1000.0])
        program += f' pose_safe_start = [{safe_start[0]:.4f}, {safe_start[1]:.4f}, {safe_start[2]:.4f}, {r0:.4f}, {p0:.4f}, {y0:.4f}]\n'

        # ✅ ВЫБОР КОМАНДЫ В ЗАВИСИМОСТИ ОТ ГАЛКИ
        move_cmd = 'movej' if self.use_movej else 'movel'
        program += f' {move_cmd}(pose_safe_start, a={self.acceleration}, v={self.velocity}, t=0, r=0.0)\n\n'

        #program += f' movel(pose_safe_start, a={self.acceleration}, v={self.velocity}, t=0, r=0.0)\n\n'

        # ✅ 2. СПУСК К НАЧАЛУ ВЕКТОРА
        program += f' pose_approach = [{approach_start[0]:.4f}, {approach_start[1]:.4f}, {approach_start[2]:.4f}, {r0:.4f}, {p0:.4f}, {y0:.4f}]\n'
        program += f' movel(pose_approach, a={self.acceleration}, v={self.velocity}, t=0, r=0.0)\n\n'

        # ✅ 3. НАЧАЛО СВАРКИ
        weld_start = self.offset_points[0] / 1000.0
        program += f' pose_start = [{weld_start[0]:.4f}, {weld_start[1]:.4f}, {weld_start[2]:.4f}, {r0:.4f}, {p0:.4f}, {y0:.4f}]\n'
        program += f' movel(pose_start, a={self.acceleration}, v={self.velocity}, t=0, r=0.0)\n\n'

        # ✅ 4. ТРАЕКТОРИЯ
        for i in range(1, len(self.offset_points)):
            pt = self.offset_points[i] / 1000.0
            r, p, y = self.orientations[min(i, len(self.orientations) - 1)]
            radius = 0.0 if i == len(self.offset_points) - 1 else self.blend_radius / 1000.0
            program += f' pose_{i} = [{pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f}, {r:.4f}, {p:.4f}, {y:.4f}]\n'
            program += f' movel(pose_{i}, a={self.acceleration}, v={self.velocity}, t=0, r={radius})\n'

        # ✅ 5. ВЫХОД ИЗ СВАРКИ
        last_tool_vec = self.tool_vectors[-1] if self.tool_vectors else (self.tool_vector_end - self.tool_vector_start)
        exit_point = (self.offset_points[-1] - last_tool_vec) / 1000.0
        r_last, p_last, y_last = self.orientations[-1]
        program += f'\n pose_exit = [{exit_point[0]:.4f}, {exit_point[1]:.4f}, {exit_point[2]:.4f}, {r_last:.4f}, {p_last:.4f}, {y_last:.4f}]\n'
        program += f' movel(pose_exit, a={self.acceleration}, v={self.velocity}, t=0, r=0.0)\n\n'

        # ✅ 6. ВОЗВРАТ НА БЕЗОПАСНУЮ ВЫСОТУ
        safe_final = np.array([exit_point[0], exit_point[1], self.safe_height / 1000.0])
        program += f' pose_safe = [{safe_final[0]:.4f}, {safe_final[1]:.4f}, {safe_final[2]:.4f}, {r_last:.4f}, {p_last:.4f}, {y_last:.4f}]\n'
        program += f' movel(pose_safe, a={self.acceleration}, v={self.velocity}, t=0, r=0.0)\n'
        program += 'end\n'

        App.Console.PrintMessage(f"{'=' * 60}\n📄 ПРОГРАММА\n{'=' * 60}\n")
        App.Console.PrintMessage(program)
        App.Console.PrintMessage(f"{'=' * 60}\n")

        # ✅ ДОБАВЛЯЕМ МЕТАДАННЫЕ В ПЕРВУЮ СТРОКУ
        metadata = f"# TRAJECTORY_LAYER: {self.layer_name}\n"
        full_program = metadata + program

        # Используем функцию из trajectory_storage
        save_trajectory_with_prefix(full_program)


def run():
    generator = WeldingPathGenerator()
    generator.start()


if __name__ == "__main__":
    run()
