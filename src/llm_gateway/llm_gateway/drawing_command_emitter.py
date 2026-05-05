"""Command emitter for drawing strokes — builds PTP/LIN/BLENDED_SEQUENCE commands.

This module depends on drawing_geometry for data classes and vector math helpers.
It is separate from drawing_geometry.py to keep the geometry compiler under the
file-size budget.
"""

from __future__ import annotations

from typing import Any, Iterable, Mapping, Sequence


def lift_points_to_poses(
    points_2d: Sequence[tuple[float, float]], workplane: Any
) -> tuple[dict[str, Any], ...]:
    from llm_gateway.drawing_geometry import _vector_add, _vector_scale

    poses: list[dict[str, Any]] = []
    for point_x, point_y in points_2d:
        position = _vector_add(
            workplane.origin,
            _vector_add(
                _vector_scale(workplane.x_axis, point_x),
                _vector_scale(workplane.y_axis, point_y),
            ),
        )
        poses.append(
            {
                "position": {
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "z": float(position[2]),
                },
                "orientation": dict(workplane.orientation),
            }
        )
    return tuple(poses)


def compile_strokes_to_commands(
    *,
    strokes: Sequence[Any],
    workplane: Any,
    reference_frame: str,
    approach_distance_m: float,
    retract_distance_m: float,
    drawing_speed_scale: float,
    travel_speed_scale: float,
    plan_only: bool,
    max_waypoints_per_chunk: int,
    use_blended_sequence: bool = True,
    blend_radius_m: float = 0.008,
) -> Any:
    from llm_gateway.drawing_geometry import DrawingCompileResult, DrawingGeometryError

    if approach_distance_m < 0.0 or retract_distance_m < 0.0:
        raise DrawingGeometryError(
            "invalid_size: approach/retract distance must be >= 0"
        )
    if drawing_speed_scale <= 0.0 or travel_speed_scale <= 0.0:
        raise DrawingGeometryError(
            "invalid_size: drawing/travel speed scales must be > 0"
        )
    if max_waypoints_per_chunk < 1:
        raise DrawingGeometryError("max_waypoints_per_chunk must be >= 1")

    commands: list[dict[str, Any]] = []
    chunk_count = 0
    draw_stroke_count = 0
    lifted_point_count = 0

    for stroke_index, stroke in enumerate(strokes):
        if stroke.kind != "draw":
            continue
        if len(stroke.points_2d) < 2:
            raise DrawingGeometryError(
                "invalid_size: draw stroke must contain at least 2 points"
            )

        draw_stroke_count += 1
        lifted_poses = list(lift_points_to_poses(stroke.points_2d, workplane))
        lifted_point_count += len(lifted_poses)

        start_pose = lifted_poses[0]
        end_pose = lifted_poses[-1]
        above_start_pose = _offset_pose_along_normal(
            start_pose, workplane.normal, approach_distance_m
        )
        above_end_pose = _offset_pose_along_normal(
            end_pose, workplane.normal, retract_distance_m
        )

        commands.append(
            _ptp_command(
                target_pose=above_start_pose,
                reference_frame=reference_frame,
                speed_scale=travel_speed_scale,
                plan_only=plan_only,
            )
        )
        commands.append(
            _lin_command(
                target_pose=start_pose,
                reference_frame=reference_frame,
                speed_scale=drawing_speed_scale,
                plan_only=plan_only,
            )
        )

        if use_blended_sequence and len(lifted_poses) >= 3:
            chunk_count += 1
            commands.append(
                _blended_sequence_command(
                    waypoints=lifted_poses[1:],
                    reference_frame=reference_frame,
                    speed_scale=drawing_speed_scale,
                    plan_only=plan_only,
                    blend_radius_m=blend_radius_m,
                    stroke_index=stroke_index + 1,
                )
            )
        else:
            draw_targets = lifted_poses[1:]
            for chunk in _chunk_sequence(draw_targets, max_waypoints_per_chunk):
                chunk_count += 1
                if len(chunk) == 1:
                    commands.append(
                        _lin_command(
                            target_pose=chunk[0],
                            reference_frame=reference_frame,
                            speed_scale=drawing_speed_scale,
                            plan_only=plan_only,
                        )
                    )
                else:
                    commands.append(
                        _cartesian_path_command(
                            waypoints=chunk,
                            reference_frame=reference_frame,
                            speed_scale=drawing_speed_scale,
                            plan_only=plan_only,
                            chunk_index=chunk_count,
                            stroke_index=stroke_index + 1,
                        )
                    )

        commands.append(
            _lin_command(
                target_pose=above_end_pose,
                reference_frame=reference_frame,
                speed_scale=travel_speed_scale,
                plan_only=plan_only,
            )
        )

    summary = {
        "draw_stroke_count": draw_stroke_count,
        "lifted_point_count": lifted_point_count,
        "chunk_count": chunk_count,
        "workplane": {
            "mode": workplane.mode,
            "frame_id": workplane.frame_id,
            "origin": {
                "x": workplane.origin[0],
                "y": workplane.origin[1],
                "z": workplane.origin[2],
            },
            "x_axis": {
                "x": workplane.x_axis[0],
                "y": workplane.x_axis[1],
                "z": workplane.x_axis[2],
            },
            "y_axis": {
                "x": workplane.y_axis[0],
                "y": workplane.y_axis[1],
                "z": workplane.y_axis[2],
            },
            "normal": {
                "x": workplane.normal[0],
                "y": workplane.normal[1],
                "z": workplane.normal[2],
            },
        },
    }
    return DrawingCompileResult(commands=tuple(commands), summary=summary)


# ---------------------------------------------------------------------------
# Internal command builders
# ---------------------------------------------------------------------------


def _ptp_command(
    *,
    target_pose: dict[str, Any],
    reference_frame: str,
    speed_scale: float,
    plan_only: bool,
) -> dict[str, Any]:
    return {
        "primitive_type": "PTP",
        "target_pose": target_pose,
        "reference_frame": reference_frame,
        "velocity_scale": float(speed_scale),
        "acceleration_scale": float(speed_scale),
        "planner_id": "PILZ_PTP",
        "plan_only": bool(plan_only),
    }


def _lin_command(
    *,
    target_pose: dict[str, Any],
    reference_frame: str,
    speed_scale: float,
    plan_only: bool,
) -> dict[str, Any]:
    return {
        "primitive_type": "LIN",
        "target_pose": target_pose,
        "reference_frame": reference_frame,
        "velocity_scale": float(speed_scale),
        "acceleration_scale": float(speed_scale),
        "planner_id": "PILZ_LIN",
        "plan_only": bool(plan_only),
    }


def _blended_sequence_command(
    *,
    waypoints: Sequence[dict[str, Any]],
    reference_frame: str,
    speed_scale: float,
    plan_only: bool,
    blend_radius_m: float,
    stroke_index: int,
) -> dict[str, Any]:
    steps = []
    for i, wp in enumerate(waypoints):
        br = 0.0 if (i == 0 or i == len(waypoints) - 1) else blend_radius_m
        steps.append(
            {
                "primitive_type": "LIN",
                "target_pose": wp,
                "blend_radius_m": br,
                "planner_id": "PILZ_LIN",
                "velocity_scale": float(speed_scale),
                "acceleration_scale": float(speed_scale),
            }
        )
    return {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": steps,
        "reference_frame": reference_frame,
        "velocity_scale": float(speed_scale),
        "acceleration_scale": float(speed_scale),
        "planner_id": "PILZ_LIN",
        "plan_only": bool(plan_only),
        "stroke_index": int(stroke_index),
    }


# DEPRECATED: removal_date=2026-06-01, reason=replaced_by_BLENDED_SEQUENCE_in_W2
def _cartesian_path_command(
    *,
    waypoints: Sequence[dict[str, Any]],
    reference_frame: str,
    speed_scale: float,
    plan_only: bool,
    chunk_index: int,
    stroke_index: int,
) -> dict[str, Any]:
    return {
        "primitive_type": "CARTESIAN_PATH",
        "waypoints": list(waypoints),
        "reference_frame": reference_frame,
        "velocity_scale": float(speed_scale),
        "acceleration_scale": float(speed_scale),
        "planner_id": "PILZ_LIN",
        "plan_only": bool(plan_only),
        "chunk_index": int(chunk_index),
        "stroke_index": int(stroke_index),
    }


def _offset_pose_along_normal(
    pose: Mapping[str, Any], normal: Any, distance_m: float
) -> dict[str, Any]:
    from llm_gateway.drawing_geometry import _vector_add, _vector_scale

    if distance_m == 0.0:
        return {
            "position": dict(pose["position"]),
            "orientation": dict(pose["orientation"]),
        }
    position = pose["position"]
    offset = _vector_add(
        (float(position["x"]), float(position["y"]), float(position["z"])),
        _vector_scale(normal, distance_m),
    )
    return {
        "position": {
            "x": offset[0],
            "y": offset[1],
            "z": offset[2],
        },
        "orientation": dict(pose["orientation"]),
    }


def _chunk_sequence(
    items: Sequence[dict[str, Any]], chunk_size: int
) -> Iterable[Sequence[dict[str, Any]]]:
    if not items:
        return []
    return [
        items[index : index + chunk_size] for index in range(0, len(items), chunk_size)
    ]


__all__ = [
    "compile_strokes_to_commands",
    "lift_points_to_poses",
]
