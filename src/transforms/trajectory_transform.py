from __future__ import annotations

import copy
import math
from typing import Any, List

from src.common.frame_pose import FramePose6D, rotation_matrix_sxyz, transform_point, transform_vector
from src.common.geometry import Point3, Pose6D, TravelSegment
from src.common.trajectory import WorkSegment, WorkTrajectory

_CIRCLE_SAMPLES = 72


def _sample_circle_polyline(radius_mm: float) -> List[Point3]:
    points: List[Point3] = []
    for index in range(_CIRCLE_SAMPLES + 1):
        angle = 2.0 * math.pi * index / _CIRCLE_SAMPLES
        points.append((radius_mm * math.cos(angle), radius_mm * math.sin(angle), 0.0))
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
        polyline_points = _transform_polyline(_sample_circle_polyline(radius_mm), rotation, translation)
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


def transform_work_trajectory(trajectory: WorkTrajectory, pose: FramePose6D) -> WorkTrajectory:
    rotation = rotation_matrix_sxyz(pose.rx_deg, pose.ry_deg, pose.rz_deg)
    translation = pose.translation()

    poses = [
        Pose6D(
            index=pose_item.index,
            pose_type=pose_item.pose_type,
            position=transform_point(pose_item.position, rotation, translation),
            tool_axis_z=transform_vector(pose_item.tool_axis_z, rotation),
            tool_axis_x=transform_vector(pose_item.tool_axis_x, rotation),
        )
        for pose_item in trajectory.poses
    ]

    travel_segments = [
        TravelSegment(
            index=segment.index,
            segment_type=segment.segment_type,
            start=transform_point(segment.start, rotation, translation),
            finish=transform_point(segment.finish, rotation, translation),
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
        start_point_mm=transform_point(trajectory.start_point_mm, rotation, translation),
        finish_point_mm=transform_point(trajectory.finish_point_mm, rotation, translation),
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
