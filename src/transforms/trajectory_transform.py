from __future__ import annotations

import math
from typing import Any, List

import copy

from src.common.frame_pose import FramePose6D, rotation_matrix_sxyz, transform_point, transform_vector
from src.common.geometry import Point3, Pose6D, TravelSegment
from src.common.trajectory import WorkSegment, WorkTrajectory

_CIRCLE_SAMPLES = 72


def _sample_circle_polyline(radius_mm: float, z_mm: float = 0.0) -> List[Point3]:
    points: List[Point3] = []
    for index in range(_CIRCLE_SAMPLES + 1):
        angle = 2.0 * math.pi * index / _CIRCLE_SAMPLES
        points.append((radius_mm * math.cos(angle), radius_mm * math.sin(angle), z_mm))
    return points


def _transform_polyline(points: List[Point3], rotation, translation: Point3) -> List[Point3]:
    return [transform_point(point, rotation, translation) for point in points]


def _transform_reference_geometry(
    reference_geometry: dict[str, Any],
    rotation,
    translation: Point3,
    pose: FramePose6D,
) -> dict[str, Any]:
    result = copy.deepcopy(reference_geometry)
    polylines: List[dict[str, Any]] = list(result.get("polylines", []))

    for circle in result.get("circles", []):
        radius_mm = float(circle["radius_mm"])
        z_mm = float(circle.get("z_mm", 0.0))
        polyline_points = _transform_polyline(
            _sample_circle_polyline(radius_mm, z_mm), rotation, translation
        )
        for index in range(len(polyline_points) - 1):
            polylines.append(
                {
                    "start": polyline_points[index],
                    "finish": polyline_points[index + 1],
                    "label": str(circle.get("label", "reference")),
                    "group": "reference",
                }
            )

    result["polylines"] = polylines
    result.pop("circles", None)
    result["source_pose"] = pose.to_dict()
    return result


def apply_program_start_finish(
    trajectory: WorkTrajectory,
    start_point_mm: Point3,
    finish_point_mm: Point3,
) -> WorkTrajectory:
    """Override start/finish with GUI values without changing work poses."""
    result = copy.deepcopy(trajectory)
    result.start_point_mm = start_point_mm
    result.finish_point_mm = finish_point_mm
    result.poses = [
        Pose6D(
            index=pose.index,
            pose_type=pose.pose_type,
            position=start_point_mm if pose.pose_type == "start" else finish_point_mm
            if pose.pose_type == "finish"
            else pose.position,
            tool_axis_z=pose.tool_axis_z,
            tool_axis_x=pose.tool_axis_x,
        )
        for pose in result.poses
    ]
    if result.travel_segments:
        segments = list(result.travel_segments)
        approach = next((s for s in segments if s.segment_type == "approach"), None)
        departure = next((s for s in segments if s.segment_type == "departure"), None)
        updated: list[TravelSegment] = []
        for segment in segments:
            if segment is approach:
                updated.append(
                    TravelSegment(
                        index=segment.index,
                        segment_type=segment.segment_type,
                        start=start_point_mm,
                        finish=segment.finish,
                    )
                )
            elif segment is departure:
                updated.append(
                    TravelSegment(
                        index=segment.index,
                        segment_type=segment.segment_type,
                        start=segment.start,
                        finish=finish_point_mm,
                    )
                )
            else:
                updated.append(segment)
        result.travel_segments = updated
    return result


def _is_fixed_program_point(pose_type: str) -> bool:
    return pose_type in ("start", "finish")


def _transform_travel_endpoint(
    point: Point3,
    segment_type: str,
    *,
    is_start: bool,
    rotation,
    translation: Point3,
) -> Point3:
    if segment_type == "approach" and is_start:
        return point
    if segment_type == "departure" and not is_start:
        return point
    return transform_point(point, rotation, translation)


def transform_work_trajectory(trajectory: WorkTrajectory, pose: FramePose6D) -> WorkTrajectory:
    rotation = rotation_matrix_sxyz(pose.rx_deg, pose.ry_deg, pose.rz_deg)
    translation = pose.translation()

    poses = []
    for pose_item in trajectory.poses:
        if _is_fixed_program_point(pose_item.pose_type):
            poses.append(pose_item)
            continue
        poses.append(
            Pose6D(
                index=pose_item.index,
                pose_type=pose_item.pose_type,
                position=transform_point(pose_item.position, rotation, translation),
                tool_axis_z=transform_vector(pose_item.tool_axis_z, rotation),
                tool_axis_x=transform_vector(pose_item.tool_axis_x, rotation),
            )
        )

    travel_segments = [
        TravelSegment(
            index=segment.index,
            segment_type=segment.segment_type,
            start=_transform_travel_endpoint(
                segment.start,
                segment.segment_type,
                is_start=True,
                rotation=rotation,
                translation=translation,
            ),
            finish=_transform_travel_endpoint(
                segment.finish,
                segment.segment_type,
                is_start=False,
                rotation=rotation,
                translation=translation,
            ),
        )
        for segment in trajectory.travel_segments
    ]

    work_segments = [
        WorkSegment(
            pass_index=segment.pass_index,
            start=transform_point(segment.start, rotation, translation),
            finish=transform_point(segment.finish, rotation, translation),
            group=segment.group,
        )
        for segment in trajectory.work_segments
    ]

    metadata = copy.deepcopy(trajectory.metadata)
    metadata["frame_pose_applied"] = pose.to_dict()
    metadata["coordinate_frame"] = "world"

    return WorkTrajectory(
        generator_id=trajectory.generator_id,
        generator_version=trajectory.generator_version,
        params_snapshot=copy.deepcopy(trajectory.params_snapshot),
        start_point_mm=trajectory.start_point_mm,
        finish_point_mm=trajectory.finish_point_mm,
        poses=poses,
        travel_segments=travel_segments,
        work_segments=work_segments,
        reference_geometry=_transform_reference_geometry(
            trajectory.reference_geometry,
            rotation,
            translation,
            pose,
        ),
        metadata=metadata,
    )
