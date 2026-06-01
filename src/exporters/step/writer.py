from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Tuple

Point3 = Tuple[float, float, float]
AddEntity = Callable[[str], int]
PointFn = Callable[[float, float, float], int]
DirectionFn = Callable[[float, float, float], int]

# GUI и расчёты в мм; в STEP пишем метры — так большинство CAD/CAM читает масштаб корректно.
MM_TO_M = 0.001


@dataclass(frozen=True)
class RgbColor:
    r: float
    g: float
    b: float


WORK_COLOR = RgbColor(0.0, 1.0, 0.0)
TRAVEL_COLOR = RgbColor(0.0, 0.4, 1.0)
FIRST_WORK_COLOR = RgbColor(1.0, 0.6, 0.0)
LAST_WORK_COLOR = RgbColor(1.0, 0.0, 0.0)
START_POINT_COLOR = RgbColor(1.0, 0.0, 1.0)
FINISH_POINT_COLOR = RgbColor(0.0, 0.8, 1.0)
REFERENCE_COLOR = RgbColor(0.5, 0.5, 0.5)
TOOL_VIS_COLOR = RgbColor(0.8, 0.0, 0.8)
POINT_MARKER_SIZE_MM = 10.0

# OCCT схлопывает «канонические» RGB в DRAUGHTING_PRE_DEFINED_COLOUR;
# FreeCAD рисует цвет линий только из COLOUR_RGB в CURVE_STYLE.
_STEP_RGB_NUDGE = 0.001


def step_export_rgb(color: RgbColor) -> RgbColor:
    def channel(value: float) -> float:
        if value <= 0.0:
            return _STEP_RGB_NUDGE
        if value >= 1.0:
            return 1.0 - _STEP_RGB_NUDGE
        return value

    return RgbColor(channel(color.r), channel(color.g), channel(color.b))


@dataclass(frozen=True)
class _LineSpec:
    start: Point3
    finish: Point3
    color: RgbColor


@dataclass(frozen=True)
class _CircleSpec:
    radius: float
    color: RgbColor
    z_mm: float = 0.0


class StepWriter:
    def __init__(self, length_scale: float = MM_TO_M) -> None:
        self._length_scale = length_scale
        self._lines: List[_LineSpec] = []
        self._circles: List[_CircleSpec] = []

    def add_line(
        self, start: Point3, finish: Point3, color: RgbColor, group: str = ""
    ) -> None:
        del group
        scaled_start = _scale_point(start, self._length_scale)
        scaled_finish = _scale_point(finish, self._length_scale)
        if _distance(scaled_start, scaled_finish) < 1e-12:
            return
        self._lines.append(
            _LineSpec(
                start=_round_point(scaled_start),
                finish=_round_point(scaled_finish),
                color=color,
            )
        )

    def add_circle(
        self, radius: float, color: RgbColor, group: str = "", *, z_mm: float = 0.0
    ) -> None:
        del group
        scaled_radius = radius * self._length_scale
        if scaled_radius <= 0:
            return
        self._circles.append(
            _CircleSpec(
                radius=scaled_radius,
                color=color,
                z_mm=z_mm * self._length_scale,
            )
        )

    def add_point_marker(
        self,
        center: Point3,
        color: RgbColor,
        group: str = "",
        size_mm: float = POINT_MARKER_SIZE_MM,
    ) -> None:
        del group
        half = size_mm / 2.0
        x, y, z = center
        self.add_line((x - half, y, z), (x + half, y, z), color)
        self.add_line((x, y - half, z), (x, y + half, z), color)

    def write(self, filepath: Path) -> None:
        entities: List[str] = []
        next_id = 0

        def add(content: str) -> int:
            nonlocal next_id
            next_id += 1
            entities.append(f"#{next_id} = {content};")
            return next_id

        timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        header = [
            "ISO-10303-21;",
            "HEADER;",
            "FILE_DESCRIPTION(('Pirelli Trajectory'),'2;1');",
            (
                f"FILE_NAME('ring_trajectory','{timestamp}',('Pirelli'),"
                "(''),'Pirelli STEP writer','Pirelli Trajectory Builder','');"
            ),
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));",
            "ENDSEC;",
            "DATA;",
        ]

        app_context_id = add(
            "APPLICATION_CONTEXT('core data for automotive mechanical design processes')"
        )
        add(
            "APPLICATION_PROTOCOL_DEFINITION('international standard',"
            f"'automotive_design',2000,#{app_context_id})"
        )

        length_unit_id = add(
            "( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) )"
        )
        angle_unit_id = add(
            "( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )"
        )
        solid_angle_unit_id = add(
            "( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )"
        )
        uncertainty_id = add(
            "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),"
            f"#{length_unit_id},'distance_accuracy_value','confusion accuracy')"
        )
        context_id = add(
            "( GEOMETRIC_REPRESENTATION_CONTEXT(3) "
            f"GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty_id})) "
            "GLOBAL_UNIT_ASSIGNED_CONTEXT("
            f"(#{length_unit_id},#{angle_unit_id},#{solid_angle_unit_id})) "
            "REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"
        )

        style_cache: Dict[RgbColor, int] = {}

        def curve_style(color: RgbColor) -> int:
            cached = style_cache.get(color)
            if cached is not None:
                return cached
            export_color = step_export_rgb(color)
            colour_id = add(
                f"COLOUR_RGB('',{_fmt(export_color.r)},{_fmt(export_color.g)},"
                f"{_fmt(export_color.b)})"
            )
            curve_font_id = add("DRAUGHTING_PRE_DEFINED_CURVE_FONT('continuous')")
            curve_style_id = add(
                f"CURVE_STYLE('',#{curve_font_id},POSITIVE_LENGTH_MEASURE(0.1),"
                f"#{colour_id})"
            )
            fill_colour_id = add(
                f"FILL_AREA_STYLE_COLOUR('',#{colour_id})"
            )
            fill_style_id = add(f"FILL_AREA_STYLE('',(#{fill_colour_id}))")
            surface_fill_id = add(f"SURFACE_STYLE_FILL_AREA(#{fill_style_id})")
            surface_side_id = add(f"SURFACE_SIDE_STYLE('',(#{surface_fill_id}))")
            surface_usage_id = add(f"SURFACE_STYLE_USAGE(.BOTH.,#{surface_side_id})")
            style_id = add(
                f"PRESENTATION_STYLE_ASSIGNMENT((#{surface_usage_id},#{curve_style_id}))"
            )
            style_cache[color] = style_id
            return style_id

        def point(x: float, y: float, z: float) -> int:
            return add(f"CARTESIAN_POINT('',({_fmt(x)},{_fmt(y)},{_fmt(z)}))")

        def direction(x: float, y: float, z: float) -> int:
            return add(f"DIRECTION('',({_fmt(x)},{_fmt(y)},{_fmt(z)}))")

        origin_id = point(0.0, 0.0, 0.0)
        z_dir_id = direction(0.0, 0.0, 1.0)
        x_dir_id = direction(1.0, 0.0, 0.0)
        axis_id = add(
            f"AXIS2_PLACEMENT_3D('',#{origin_id},#{z_dir_id},#{x_dir_id})"
        )

        curve_set_ids: List[int] = []
        styled_items: List[Tuple[int, int]] = []

        for line_spec in self._lines:
            curve_set_id = _add_line_entity(
                add, point, direction, line_spec.start, line_spec.finish
            )
            curve_set_ids.append(curve_set_id)
            styled_items.append((curve_set_id, curve_style(line_spec.color)))

        for circle_spec in self._circles:
            curve_set_id = _add_circle_entity(
                add, point, direction, circle_spec.radius, circle_spec.z_mm
            )
            curve_set_ids.append(curve_set_id)
            styled_items.append((curve_set_id, curve_style(circle_spec.color)))

        product_context_id = add(
            f"PRODUCT_CONTEXT('',#{app_context_id},'mechanical')"
        )
        product_id = add(
            "PRODUCT('ring_trajectory','ring_trajectory','',"
            f"(#{product_context_id}))"
        )
        pdf_id = add(f"PRODUCT_DEFINITION_FORMATION('','',#{product_id})")
        pdc_id = add(
            f"PRODUCT_DEFINITION_CONTEXT('part definition',#{app_context_id},'design')"
        )
        pd_id = add(f"PRODUCT_DEFINITION('design','',#{pdf_id},#{pdc_id})")
        pds_id = add(f"PRODUCT_DEFINITION_SHAPE('','',#{pd_id})")

        curve_refs = ",#".join(str(item_id) for item_id in [axis_id] + curve_set_ids)
        shape_rep_id = add(
            "GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('',"
            f"(#{curve_refs}),#{context_id})"
        )
        add(f"SHAPE_DEFINITION_REPRESENTATION(#{pds_id},#{shape_rep_id})")

        for curve_set_id, style_id in styled_items:
            add(f"STYLED_ITEM('color',(#{style_id}),#{curve_set_id})")

        footer = ["ENDSEC;", "END-ISO-10303-21;"]
        filepath.write_text(
            "\n".join(header + entities + footer) + "\n",
            encoding="utf-8",
        )


def _scale_point(point: Point3, scale: float) -> Point3:
    return (point[0] * scale, point[1] * scale, point[2] * scale)


def _add_line_entity(
    add: AddEntity,
    point: PointFn,
    direction: DirectionFn,
    start: Point3,
    finish: Point3,
) -> int:
    length = _distance(start, finish)
    direction_id = direction(
        (finish[0] - start[0]) / length,
        (finish[1] - start[1]) / length,
        (finish[2] - start[2]) / length,
    )
    start_id = point(*start)
    finish_id = point(*finish)
    vector_id = add(f"VECTOR('',#{direction_id},1.)")
    line_id = add(f"LINE('',#{start_id},#{vector_id})")
    trimmed_id = add(
        "TRIMMED_CURVE('',"
        f"#{line_id},(#{start_id},PARAMETER_VALUE(0.)),(#{finish_id},"
        f"PARAMETER_VALUE({_fmt(length)})),.T.,.PARAMETER.)"
    )
    return add(f"GEOMETRIC_CURVE_SET('',(#{trimmed_id}))")


def _add_circle_entity(
    add: AddEntity,
    point: PointFn,
    direction: DirectionFn,
    radius: float,
    z_mm: float = 0.0,
) -> int:
    origin_id = point(0.0, 0.0, z_mm)
    z_dir_id = direction(0.0, 0.0, 1.0)
    x_dir_id = direction(1.0, 0.0, 0.0)
    axis_id = add(
        f"AXIS2_PLACEMENT_3D('',#{origin_id},#{z_dir_id},#{x_dir_id})"
    )
    circle_id = add(f"CIRCLE('',#{axis_id},{_fmt(radius)})")
    start_id = point(radius, 0.0, z_mm)
    end_id = point(radius, 0.0, z_mm)
    two_pi = 2.0 * math.pi
    trimmed_id = add(
        "TRIMMED_CURVE('',"
        f"#{circle_id},(#{start_id},PARAMETER_VALUE(0.)),(#{end_id},"
        f"PARAMETER_VALUE({_fmt(two_pi)})),.T.,.PARAMETER.)"
    )
    return add(f"GEOMETRIC_CURVE_SET('',(#{trimmed_id}))")


def _round_point(point: Point3) -> Point3:
    return (round(point[0], 9), round(point[1], 9), round(point[2], 9))


def _distance(start: Point3, finish: Point3) -> float:
    return math.sqrt(
        (finish[0] - start[0]) ** 2
        + (finish[1] - start[1]) ** 2
        + (finish[2] - start[2]) ** 2
    )


def _fmt(value: float) -> str:
    value = float(value)
    if abs(value) < 1e-12:
        return "0."
    text = f"{value:.9f}".rstrip("0").rstrip(".")
    return text + "." if "." not in text else text
