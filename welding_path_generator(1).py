# -*- coding: utf-8 -*-
"""
Welding Path Generator for Elite CS612 Robot v3.0
+ Использование скомпилированного модуля elite_core
+ Уникальные имена слоёв
+ Старт с безопасной высоты
"""

import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtGui, QtCore
import numpy as np
import math
import Part
import sys
import os

# Импортируем функции сохранения
sys.path.insert(0, os.path.dirname(__file__))
from trajectory_storage import save_trajectory_with_prefix

# === КОНСТАНТЫ ===
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

    def setup_ui(self):
        layout = QtGui.QFormLayout()
        self.discr_spin = QtGui.QDoubleSpinBox()
        self.discr_spin.setRange(0.01, 10.0)
        self.discr_spin.setDecimals(2)
        self.discr_spin.setValue(DEFAULT_DISCRETIZATION_MM)
        layout.addRow("Дискретизация (мм):", self.discr_spin)
        self.safe_spin = QtGui.QDoubleSpinBox()
        self.safe_spin.setRange(0.0, 1000.0)
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
            'acceleration': self.acc_spin.value()
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
        self.layer_name = "welding_trajectory"

    def start(self):
        App.Console.PrintMessage(f"\n{'=' * 60}\n")
        App.Console.PrintMessage("🔥 ГЕНЕРАТОР СВАРОЧНОЙ ТРАЕКТОРИИ v3.0\n")
        App.Console.PrintMessage(f"{'=' * 60}\n\n")

        dialog = ParametersDialog()
        if dialog.exec_():
            params = dialog.get_values()
            self.discretization = params['discretization']
            self.safe_height = params['safe_height']
            self.blend_radius = params['blend_radius']
            self.tool_offset = params['tool_offset']
            self.tool_rotation = params['tool_rotation']
            self.velocity = params['velocity']
            self.acceleration = params['acceleration']

            App.Console.PrintMessage(f"✓ Параметры:\n")
            App.Console.PrintMessage(f"  Дискретизация: {self.discretization} мм\n")
            App.Console.PrintMessage(f"  Безопасная высота: {self.safe_height} мм\n")
            App.Console.PrintMessage(f"  Радиус: {self.blend_radius} мм\n")
            App.Console.PrintMessage(f"  Смещение: {self.tool_offset} мм\n")
            App.Console.PrintMessage(f"  Поворот: {self.tool_rotation}°\n")
            App.Console.PrintMessage(f"  v={self.velocity} м/с, a={self.acceleration} м/с²\n\n")

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
            if hasattr(weld_obj, 'SubObjects') and len(weld_obj.SubObjects) > 0:
                shape = weld_obj.SubObjects[0]
                if hasattr(shape, 'Wires') and len(shape.Wires) > 0:
                    self.weld_wire = shape.Wires[0]
                elif hasattr(shape, 'Edges') and len(shape.Edges) > 0:
                    self.weld_wire = Part.Wire(shape.Edges)
                else:
                    self.weld_wire = shape
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
            sel = Gui.Selection.getSelectionEx()
            if len(sel) == 0:
                QtGui.QMessageBox.warning(None, "Ошибка", "Выберите вектор инструмента!")
                return
            tool_obj = sel[0]
            if hasattr(tool_obj, 'SubObjects') and len(tool_obj.SubObjects) > 0:
                shape = tool_obj.SubObjects[0]
                if hasattr(shape, 'Wires') and len(shape.Wires) > 0:
                    self.tool_wire = shape.Wires[0]
                elif hasattr(shape, 'Edges') and len(shape.Edges) > 0:
                    self.tool_wire = Part.Wire(shape.Edges)
                else:
                    self.tool_wire = shape
            if not self.tool_wire:
                QtGui.QMessageBox.warning(None, "Ошибка", "Не удалось получить вектор!")
                return
            App.Console.PrintMessage(f"✓ Вектор инструмента выбран\n\n")
            Gui.Selection.clearSelection()
            dialog.close()
            self.determine_tool_vector_ends()
            self.process_trajectory()

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
            self.discretize_wire()
            self.check_weld_direction()
            self.calculate_orientations()
            self.apply_tool_offset()
            self.visualize_trajectory()
            self.show_confirmation_dialog()
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

    def calculate_orientations(self):
        import elite_core

        initial_tool_vec = self.tool_vector_end - self.tool_vector_start
        self.tool_vectors = []
        self.orientations = []

        first_segment_dir = self.weld_points[1] - self.weld_points[0]
        first_segment_dir = first_segment_dir / np.linalg.norm(first_segment_dir)

        for i in range(len(self.weld_points) - 1):
            segment_dir = self.weld_points[i + 1] - self.weld_points[i]
            segment_dir = segment_dir / np.linalg.norm(segment_dir)

            rotated_tool_vec = self.rotate_vector_between_directions(
                initial_tool_vec, first_segment_dir, segment_dir
            )
            self.tool_vectors.append(rotated_tool_vec)

            roll, pitch, yaw = elite_core.calculate_orientation(
                rotated_tool_vec.tolist(),
                segment_dir.tolist(),
                math.radians(self.tool_rotation)
            )
            self.orientations.append((roll, pitch, yaw))

        App.Console.PrintMessage(f"✓ Рассчитано {len(self.orientations)} ориентаций\n")

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
        doc = App.ActiveDocument
        if not doc:
            return base_name

        existing_indices = []
        for obj in doc.Objects:
            if obj.Name.startswith(base_name):
                suffix = obj.Name[len(base_name):]
                if suffix == "":
                    existing_indices.append(0)
                elif suffix.startswith("_"):
                    try:
                        idx = int(suffix[1:])
                        existing_indices.append(idx)
                    except:
                        pass

        if not existing_indices:
            return base_name

        next_index = max(existing_indices) + 1
        return f"{base_name}_{next_index}"

    def visualize_trajectory(self):
        doc = App.ActiveDocument
        if not doc:
            return

        self.layer_name = self.get_unique_layer_name("welding_trajectory")
        vis_group = doc.addObject("App::DocumentObjectGroup", self.layer_name)

        App.Console.PrintMessage(f"✓ Создан слой: {self.layer_name}\n")

        from FreeCAD import Vector

        for i, pt in enumerate(self.weld_points):
            sphere = Part.makeSphere(2.0, Vector(*pt))
            pt_obj = doc.addObject("Part::Feature", f"weld_pt_{i}")
            pt_obj.Shape = sphere
            if hasattr(pt_obj, "ViewObject"):
                pt_obj.ViewObject.ShapeColor = (1.0, 0.5, 0.0)
            vis_group.addObject(pt_obj)

        for i, pt in enumerate(self.offset_points):
            sphere = Part.makeSphere(2.5, Vector(*pt))
            pt_obj = doc.addObject("Part::Feature", f"offset_pt_{i}")
            pt_obj.Shape = sphere
            if hasattr(pt_obj, "ViewObject"):
                pt_obj.ViewObject.ShapeColor = (1.0, 0.0, 0.0)
            vis_group.addObject(pt_obj)

        for i, pt in enumerate(self.weld_points[:-1]):
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

        safe_plane = Part.makePlane(500, 500, Vector(-250, -250, self.safe_height))
        safe_obj = doc.addObject("Part::Feature", "safe_plane")
        safe_obj.Shape = safe_plane
        if hasattr(safe_obj, "ViewObject"):
            safe_obj.ViewObject.ShapeColor = (0.0, 1.0, 0.0)
            safe_obj.ViewObject.Transparency = 70
        vis_group.addObject(safe_obj)

        doc.recompute()
        App.Console.PrintMessage("✓ Визуализация готова\n")
        App.Console.PrintMessage(f"  🔵 Голубые линии = векторы инструмента\n")
        App.Console.PrintMessage(f"  🟠 Оранжевые точки = оригинальная траектория\n")
        App.Console.PrintMessage(f"  🔴 Красные точки = смещенная траектория (реальная)\n\n")

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
        import elite_core

        # Вызываем функцию генерации из скомпилированного модуля
        program_text = elite_core.generate_program(
            offset_points=[p.tolist() for p in self.offset_points],
            orientations=self.orientations,
            tool_vector_start=self.tool_vector_start.tolist(),
            last_tool_vector=self.tool_vectors[-1].tolist(),  # ✅ ДОБАВЛЕНО!
            safe_height=self.safe_height,
            blend_radius=self.blend_radius,
            velocity=self.velocity,
            acceleration=self.acceleration
        )

        # Добавляем метаданные
        metadata = f"# TRAJECTORY_LAYER: {self.layer_name}\n"
        full_program = metadata + program_text

        App.Console.PrintMessage(f"\n{'=' * 60}\n📄 ПРОГРАММА\n{'=' * 60}\n")
        App.Console.PrintMessage(full_program)
        App.Console.PrintMessage(f"{'=' * 60}\n")

        # Сохранение
        save_trajectory_with_prefix(full_program)


def run():
    generator = WeldingPathGenerator()
    generator.start()


if __name__ == "__main__":
    run()
