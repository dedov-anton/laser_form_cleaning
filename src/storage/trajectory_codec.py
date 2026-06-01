from __future__ import annotations

from dataclasses import asdict
from typing import Any

from src.common.geometry import Pose6D, TravelSegment
from src.common.trajectory import WorkSegment, WorkTrajectory


def _point3_to_list(point: tuple[float, float, float]) -> list[float]:
    return [float(point[0]), float(point[1]), float(point[2])]


def _list_to_point3(values: list[float]) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


def _serialize_reference_geometry(reference_geometry: dict[str, Any]) -> dict[str, Any]:
    result = dict(reference_geometry)
    polylines = []
    for item in result.get("polylines", []):
        polylines.append(
            {
                **{key: value for key, value in item.items() if key not in ("start", "finish")},
                "start": _point3_to_list(item["start"]),
                "finish": _point3_to_list(item["finish"]),
            }
        )
    if polylines:
        result["polylines"] = polylines
    return result


def _deserialize_reference_geometry(reference_geometry: dict[str, Any]) -> dict[str, Any]:
    result = dict(reference_geometry)
    polylines = []
    for item in result.get("polylines", []):
        polylines.append(
            {
                **{key: value for key, value in item.items() if key not in ("start", "finish")},
                "start": _list_to_point3(item["start"]),
                "finish": _list_to_point3(item["finish"]),
            }
        )
    if polylines:
        result["polylines"] = polylines
    return result


def trajectory_to_dict(trajectory: WorkTrajectory) -> dict[str, Any]:
    return {
        "generator_id": trajectory.generator_id,
        "generator_version": trajectory.generator_version,
        "params_snapshot": trajectory.params_snapshot,
        "start_point_mm": _point3_to_list(trajectory.start_point_mm),
        "finish_point_mm": _point3_to_list(trajectory.finish_point_mm),
        "poses": [
            {
                "index": pose.index,
                "pose_type": pose.pose_type,
                "position": _point3_to_list(pose.position),
                "tool_axis_z": _point3_to_list(pose.tool_axis_z),
                "tool_axis_x": _point3_to_list(pose.tool_axis_x),
            }
            for pose in trajectory.poses
        ],
        "travel_segments": [
            {
                "index": segment.index,
                "segment_type": segment.segment_type,
                "start": _point3_to_list(segment.start),
                "finish": _point3_to_list(segment.finish),
            }
            for segment in trajectory.travel_segments
        ],
        "work_segments": [
            {
                "pass_index": segment.pass_index,
                "start": _point3_to_list(segment.start),
                "finish": _point3_to_list(segment.finish),
                "group": segment.group,
            }
            for segment in trajectory.work_segments
        ],
        "reference_geometry": _serialize_reference_geometry(trajectory.reference_geometry),
        "metadata": trajectory.metadata,
    }


def trajectory_from_dict(data: dict[str, Any]) -> WorkTrajectory:
    return WorkTrajectory(
        generator_id=str(data["generator_id"]),
        generator_version=str(data.get("generator_version", "1.0")),
        params_snapshot=dict(data.get("params_snapshot", {})),
        start_point_mm=_list_to_point3(data["start_point_mm"]),
        finish_point_mm=_list_to_point3(data["finish_point_mm"]),
        poses=[
            Pose6D(
                index=int(pose["index"]),
                pose_type=str(pose["pose_type"]),
                position=_list_to_point3(pose["position"]),
                tool_axis_z=_list_to_point3(pose["tool_axis_z"]),
                tool_axis_x=_list_to_point3(pose["tool_axis_x"]),
            )
            for pose in data.get("poses", [])
        ],
        travel_segments=[
            TravelSegment(
                index=int(segment["index"]),
                segment_type=str(segment["segment_type"]),
                start=_list_to_point3(segment["start"]),
                finish=_list_to_point3(segment["finish"]),
            )
            for segment in data.get("travel_segments", [])
        ],
        work_segments=[
            WorkSegment(
                pass_index=int(segment["pass_index"]),
                start=_list_to_point3(segment["start"]),
                finish=_list_to_point3(segment["finish"]),
                group=str(segment["group"]),
            )
            for segment in data.get("work_segments", [])
        ],
        reference_geometry=_deserialize_reference_geometry(dict(data.get("reference_geometry", {}))),
        metadata=dict(data.get("metadata", {})),
    )


def project_to_dict(project) -> dict[str, Any]:
    from src.common.project import FormProject

    if not isinstance(project, FormProject):
        raise TypeError("Expected FormProject")
    data = asdict(project)
    return data
