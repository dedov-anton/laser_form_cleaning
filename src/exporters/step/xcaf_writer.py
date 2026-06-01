from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.gp import gp_Ax2, gp_Circ, gp_Dir, gp_Pnt
from OCP.IFSelect import IFSelect_ReturnStatus
from OCP.Interface import Interface_Static
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopoDS import TopoDS_Compound, TopoDS_Edge
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool
from OCP.XSControl import XSControl_WorkSession

from src.exporters.step.writer import POINT_MARKER_SIZE_MM, RgbColor, step_export_rgb

Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class _ColoredEdge:
    edge: TopoDS_Edge
    color: RgbColor


class XcafStepWriter:
    """STEP с цветами: один Part, цвет на каждую кривую — как FreeCAD пересохраняет."""

    def __init__(self) -> None:
        self._items: List[_ColoredEdge] = []

    def add_line(
        self,
        start: Point3,
        finish: Point3,
        color: RgbColor,
        group: str,
    ) -> None:
        del group
        if _distance(start, finish) < 1e-9:
            return
        edge = BRepBuilderAPI_MakeEdge(_gp_pnt(start), _gp_pnt(finish)).Edge()
        self._items.append(_ColoredEdge(edge=edge, color=color))

    def add_circle(self, radius: float, color: RgbColor, group: str) -> None:
        del group
        if radius <= 0:
            return
        circle = gp_Circ(
            gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
            radius,
        )
        edge = BRepBuilderAPI_MakeEdge(circle).Edge()
        self._items.append(_ColoredEdge(edge=edge, color=color))

    def add_point_marker(
        self,
        center: Point3,
        color: RgbColor,
        group: str,
        size_mm: float = POINT_MARKER_SIZE_MM,
    ) -> None:
        half = size_mm / 2.0
        x, y, z = center
        self.add_line((x - half, y, z), (x + half, y, z), color, group)
        self.add_line((x, y - half, z), (x, y + half, z), color, group)

    def write(self, filepath: Path) -> None:
        if not self._items:
            raise ValueError("No geometry to export")

        app = XCAFApp_Application.GetApplication_s()
        doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
        app.InitDocument(doc)

        shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
        shape_tool.SetAutoNaming_s(False)

        root = shape_tool.NewShape()
        TDataStd_Name.Set_s(
            root,
            TCollection_ExtendedString("ring_trajectory"),
        )

        builder = BRep_Builder()
        compound = TopoDS_Compound()
        builder.MakeCompound(compound)
        for item in self._items:
            builder.Add(compound, item.edge)

        shape_tool.SetShape(root, compound)

        for item in self._items:
            sub_label = shape_tool.AddSubShape(root, item.edge)
            _set_curve_color(color_tool, sub_label, item.color)

        shape_tool.UpdateAssemblies()

        session = XSControl_WorkSession()
        writer = STEPCAFControl_Writer(session, False)
        writer.SetColorMode(True)
        writer.SetNameMode(True)
        Interface_Static.SetCVal_s("write.step.unit", "MM")

        if not writer.Transfer(doc, STEPControl_AsIs):
            raise RuntimeError("STEPCAFControl_Writer.Transfer failed")

        status = writer.Write(str(filepath))
        if status != IFSelect_ReturnStatus.IFSelect_RetDone:
            raise RuntimeError(f"STEPCAFControl_Writer.Write failed: {status}")


def _set_curve_color(color_tool, label, color: RgbColor) -> None:
    export_color = step_export_rgb(color)
    quantity_color = Quantity_Color(
        export_color.r,
        export_color.g,
        export_color.b,
        Quantity_TOC_RGB,
    )
    color_tool.SetColor(label, quantity_color, XCAFDoc_ColorType.XCAFDoc_ColorCurv)
    color_tool.SetColor(label, quantity_color, XCAFDoc_ColorType.XCAFDoc_ColorGen)


def _gp_pnt(point: Point3) -> gp_Pnt:
    return gp_Pnt(point[0], point[1], point[2])


def _distance(start: Point3, finish: Point3) -> float:
    return math.sqrt(
        (finish[0] - start[0]) ** 2
        + (finish[1] - start[1]) ** 2
        + (finish[2] - start[2]) ** 2
    )
