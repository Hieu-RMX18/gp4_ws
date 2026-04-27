"""Deterministic geometry compiler for DRAW_SHAPE and DRAW_TEXT intents.

This module is pure Python and intentionally independent from ROS runtime APIs.
It converts high-level drawing semantics into deterministic local 2D strokes, then
lifts them into 3D poses inside an explicitly defined workplane.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

from llm_gateway.stroke_font import GLYPHS, generate_text_strokes


Point2D = Tuple[float, float]
Vector3 = Tuple[float, float, float]


_SUPPORTED_UNITS_TO_M = {
    "m": 1.0,
    "meter": 1.0,
    "meters": 1.0,
    "cm": 0.01,
    "centimeter": 0.01,
    "centimeters": 0.01,
    "mm": 0.001,
    "millimeter": 0.001,
    "millimeters": 0.001,
}


@dataclass(frozen=True)
class StrokeSegment:
    kind: str
    points_2d: Tuple[Point2D, ...]
    closed: bool
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class WorkplaneAxes:
    mode: str
    frame_id: str
    origin: Vector3
    x_axis: Vector3
    y_axis: Vector3
    normal: Vector3
    orientation: Mapping[str, float]


@dataclass(frozen=True)
class DrawingCompileResult:
    commands: Tuple[Dict[str, Any], ...]
    summary: Mapping[str, Any]


class DrawingGeometryError(ValueError):
    """Raised when drawing geometry or workplane resolution fails."""


def to_meters(value: Any, units: str, *, field_name: str) -> float:
    """Convert a scalar from declared units to meters."""
    if value is None:
        raise DrawingGeometryError(f"{field_name} is required")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise DrawingGeometryError(f"{field_name} must be numeric") from exc

    scale = _SUPPORTED_UNITS_TO_M.get(str(units).strip().lower())
    if scale is None:
        raise DrawingGeometryError(f"invalid_units: unsupported units '{units}'")
    return numeric * scale


def to_radians(value: Any, *, field_name: str, units: str = "deg") -> float:
    if value is None:
        raise DrawingGeometryError(f"{field_name} is required")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise DrawingGeometryError(f"{field_name} must be numeric") from exc
    normalized_units = str(units).strip().lower()
    if normalized_units in {"rad", "radian", "radians"}:
        return numeric
    if normalized_units in {"deg", "degree", "degrees"}:
        return math.radians(numeric)
    raise DrawingGeometryError(f"invalid_angle_units: unsupported angle units '{units}'")


def parse_position_dict(value: Any, *, field_name: str) -> Vector3:
    if not isinstance(value, dict):
        raise DrawingGeometryError(f"{field_name} must be an object with x/y/z")
    try:
        return (float(value["x"]), float(value["y"]), float(value["z"]))
    except KeyError as exc:
        raise DrawingGeometryError(f"{field_name}.{exc.args[0]} is required") from exc
    except (TypeError, ValueError) as exc:
        raise DrawingGeometryError(f"{field_name} values must be numeric") from exc


def parse_vector_dict(value: Any, *, field_name: str) -> Vector3:
    return parse_position_dict(value, field_name=field_name)


def orientation_to_quaternion(orientation: Mapping[str, Any], *, field_name: str) -> Dict[str, float]:
    try:
        qx = float(orientation["x"])
        qy = float(orientation["y"])
        qz = float(orientation["z"])
        qw = float(orientation["w"])
    except KeyError as exc:
        raise DrawingGeometryError(f"{field_name}.{exc.args[0]} is required") from exc
    except (TypeError, ValueError) as exc:
        raise DrawingGeometryError(f"{field_name} quaternion values must be numeric") from exc

    norm = math.sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise DrawingGeometryError(f"{field_name} quaternion is invalid")
    return {
        "x": qx / norm,
        "y": qy / norm,
        "z": qz / norm,
        "w": qw / norm,
    }


def resolve_workplane(
    *,
    mode: str,
    frame_id: str,
    anchor_position: Vector3 | None,
    origin_pose: Mapping[str, Any] | None,
    default_orientation: Mapping[str, Any],
    normal: Vector3 | None = None,
    x_axis_hint: Vector3 | None = None,
) -> WorkplaneAxes:
    normalized_mode = str(mode).strip().lower()
    if normalized_mode not in {"base", "tool", "explicit_pose"}:
        raise DrawingGeometryError(f"missing_workplane: unsupported workplane mode '{mode}'")

    if origin_pose is not None:
        if not isinstance(origin_pose, dict):
            raise DrawingGeometryError("workplane.origin must be an object")
        if "position" not in origin_pose:
            raise DrawingGeometryError("workplane.origin.position is required")
        origin = parse_position_dict(origin_pose["position"], field_name="workplane.origin.position")
    elif anchor_position is not None:
        origin = anchor_position
    else:
        raise DrawingGeometryError("missing_workplane: origin is required")

    orientation = orientation_to_quaternion(default_orientation, field_name="workplane.default_orientation")
    if origin_pose is not None and isinstance(origin_pose.get("orientation"), dict):
        orientation = orientation_to_quaternion(
            origin_pose["orientation"], field_name="workplane.origin.orientation"
        )

    basis_x, basis_y, basis_z = _quaternion_basis(orientation)

    normal_axis = _normalize(normal if normal is not None else basis_z, field_name="workplane.normal")
    if normalized_mode == "base" and normal is None:
        normal_axis = (0.0, 0.0, 1.0)

    x_axis = _normalize(x_axis_hint if x_axis_hint is not None else basis_x, field_name="workplane.x_axis_hint")
    if normalized_mode == "base" and x_axis_hint is None:
        x_axis = (1.0, 0.0, 0.0)

    cross_mag = _norm(_cross(normal_axis, x_axis))
    if cross_mag <= 1e-9:
        raise DrawingGeometryError("missing_workplane: x_axis_hint is parallel to normal")

    y_axis = _normalize(_cross(normal_axis, x_axis), field_name="workplane.y_axis")
    x_axis = _normalize(_cross(y_axis, normal_axis), field_name="workplane.x_axis")

    return WorkplaneAxes(
        mode=normalized_mode,
        frame_id=str(frame_id),
        origin=origin,
        x_axis=x_axis,
        y_axis=y_axis,
        normal=normal_axis,
        orientation=orientation,
    )


def generate_square_path(*, side_m: float) -> Tuple[StrokeSegment, ...]:
    if side_m <= 0.0:
        raise DrawingGeometryError("invalid_size: square side must be > 0")
    return generate_rectangle_path(width_m=side_m, height_m=side_m)


def generate_rectangle_path(*, width_m: float, height_m: float) -> Tuple[StrokeSegment, ...]:
    if width_m <= 0.0 or height_m <= 0.0:
        raise DrawingGeometryError("invalid_size: rectangle width/height must be > 0")
    points = (
        (0.0, 0.0),
        (width_m, 0.0),
        (width_m, height_m),
        (0.0, height_m),
        (0.0, 0.0),
    )
    return (StrokeSegment(kind="draw", points_2d=points, closed=True, metadata={"shape": "rectangle"}),)


def generate_triangle_path(*, side_m: float, points_2d: Sequence[Point2D] | None = None) -> Tuple[StrokeSegment, ...]:
    if points_2d is not None:
        if len(points_2d) != 3:
            raise DrawingGeometryError("triangle explicit points require exactly 3 vertices")
        closed_points = tuple(points_2d) + (tuple(points_2d[0]),)
        return (StrokeSegment(kind="draw", points_2d=closed_points, closed=True, metadata={"shape": "triangle"}),)

    if side_m <= 0.0:
        raise DrawingGeometryError("invalid_size: triangle side must be > 0")
    height = side_m * math.sqrt(3.0) / 2.0
    points = (
        (0.0, 0.0),
        (side_m, 0.0),
        (side_m / 2.0, height),
        (0.0, 0.0),
    )
    return (StrokeSegment(kind="draw", points_2d=points, closed=True, metadata={"shape": "triangle"}),)


def generate_polygon_path(
    *,
    n_sides: int,
    radius_m: float | None = None,
    side_m: float | None = None,
) -> Tuple[StrokeSegment, ...]:
    if n_sides < 3:
        raise DrawingGeometryError("invalid_polygon_sides: n_sides must be >= 3")

    resolved_radius = radius_m
    if resolved_radius is None and side_m is not None:
        if side_m <= 0.0:
            raise DrawingGeometryError("invalid_size: polygon side must be > 0")
        resolved_radius = side_m / (2.0 * math.sin(math.pi / float(n_sides)))

    if resolved_radius is None or resolved_radius <= 0.0:
        raise DrawingGeometryError("invalid_size: polygon radius or side length is required")

    points: List[Point2D] = []
    center_x = resolved_radius
    center_y = 0.0
    for idx in range(n_sides + 1):
        theta = (2.0 * math.pi * float(idx)) / float(n_sides)
        points.append((center_x - resolved_radius * math.cos(theta), center_y + resolved_radius * math.sin(theta)))
    return (StrokeSegment(kind="draw", points_2d=tuple(points), closed=True, metadata={"shape": "polygon"}),)


def generate_circle_path(
    *,
    radius_m: float,
    max_chord_error_m: float,
    max_segment_angle_rad: float,
) -> Tuple[StrokeSegment, ...]:
    if radius_m <= 0.0:
        raise DrawingGeometryError("invalid_size: circle radius must be > 0")
    if max_chord_error_m <= 0.0:
        raise DrawingGeometryError("max_chord_error_m must be > 0")
    if max_segment_angle_rad <= 0.0:
        raise DrawingGeometryError("max_segment_angle_rad must be > 0")

    theta_error = _segment_angle_from_chord_error(radius_m, max_chord_error_m)
    segments_by_error = max(3, int(math.ceil((2.0 * math.pi) / theta_error)))
    segments_by_angle = max(3, int(math.ceil((2.0 * math.pi) / max_segment_angle_rad)))
    segments = max(12, segments_by_error, segments_by_angle)

    points: List[Point2D] = []
    center_x = radius_m
    center_y = 0.0
    for idx in range(segments + 1):
        theta = (2.0 * math.pi * float(idx)) / float(segments)
        points.append((center_x - radius_m * math.cos(theta), center_y + radius_m * math.sin(theta)))
    return (StrokeSegment(kind="draw", points_2d=tuple(points), closed=True, metadata={"shape": "circle"}),)


def generate_arc_path(
    *,
    radius_m: float,
    sweep_rad: float,
    max_chord_error_m: float,
    max_segment_angle_rad: float,
) -> Tuple[StrokeSegment, ...]:
    if radius_m <= 0.0:
        raise DrawingGeometryError("invalid_size: arc radius must be > 0")
    if math.isclose(sweep_rad, 0.0, abs_tol=1e-12):
        raise DrawingGeometryError("invalid_size: arc sweep must be non-zero")
    if max_chord_error_m <= 0.0:
        raise DrawingGeometryError("max_chord_error_m must be > 0")
    if max_segment_angle_rad <= 0.0:
        raise DrawingGeometryError("max_segment_angle_rad must be > 0")

    abs_sweep = abs(sweep_rad)
    theta_error = _segment_angle_from_chord_error(radius_m, max_chord_error_m)
    segments_by_error = max(2, int(math.ceil(abs_sweep / theta_error)))
    segments_by_angle = max(2, int(math.ceil(abs_sweep / max_segment_angle_rad)))
    segments = max(2, segments_by_error, segments_by_angle)

    points: List[Point2D] = []
    center_x = radius_m
    center_y = 0.0
    for idx in range(segments + 1):
        theta = sweep_rad * float(idx) / float(segments)
        points.append((center_x - radius_m * math.cos(theta), center_y + radius_m * math.sin(theta)))
    return (StrokeSegment(kind="draw", points_2d=tuple(points), closed=False, metadata={"shape": "arc"}),)


def generate_polyline_path(*, points_2d: Sequence[Point2D]) -> Tuple[StrokeSegment, ...]:
    if len(points_2d) < 2:
        raise DrawingGeometryError("invalid_size: polyline requires at least 2 points")
    return (StrokeSegment(kind="draw", points_2d=tuple(points_2d), closed=False, metadata={"shape": "polyline"}),)


def generate_text_stroke_segments(
    *,
    text: str,
    height_m: float,
    char_spacing_m: float,
    line_spacing_m: float,
    alignment: str,
) -> Tuple[StrokeSegment, ...]:
    if not isinstance(text, str) or not text:
        raise DrawingGeometryError("text must be non-empty")
    if height_m <= 0.0:
        raise DrawingGeometryError("invalid_size: font height must be > 0")
    if char_spacing_m < 0.0:
        raise DrawingGeometryError("invalid_size: char spacing must be >= 0")
    if line_spacing_m < 0.0:
        raise DrawingGeometryError("invalid_size: line spacing must be >= 0")

    normalized_align = str(alignment).strip().lower() or "left"
    if normalized_align not in {"left", "center", "right"}:
        raise DrawingGeometryError("invalid_alignment: alignment must be left|center|right")

    lines = text.split("\n")
    output: List[StrokeSegment] = []
    previous_end: Point2D | None = None

    for line_index, line in enumerate(lines):
        baseline_y = -float(line_index) * (height_m + line_spacing_m)
        line_width = _line_advance_width(line, height_m=height_m, char_spacing_m=char_spacing_m)

        if normalized_align == "left":
            x_shift = 0.0
        elif normalized_align == "center":
            x_shift = -0.5 * line_width
        else:
            x_shift = -line_width

        if not line:
            previous_end = None
            continue

        raw_segments = generate_text_strokes(line, height_m=height_m, char_spacing_m=char_spacing_m)
        for raw_segment in raw_segments:
            shifted_points = tuple((x + x_shift, y + baseline_y) for x, y in raw_segment.points_2d)

            if raw_segment.kind == "draw" and previous_end is not None:
                if _point_distance(previous_end, shifted_points[0]) > 1e-9:
                    output.append(
                        StrokeSegment(
                            kind="travel",
                            points_2d=(previous_end, shifted_points[0]),
                            closed=False,
                            metadata={"source": "text_line_transition"},
                        )
                    )

            output.append(
                StrokeSegment(
                    kind=raw_segment.kind,
                    points_2d=shifted_points,
                    closed=bool(raw_segment.closed),
                    metadata={"source": "text"},
                )
            )
            previous_end = shifted_points[-1]

    return tuple(output)


def lift_points_to_poses(points_2d: Sequence[Point2D], workplane: WorkplaneAxes) -> Tuple[Dict[str, Any], ...]:
    poses: List[Dict[str, Any]] = []
    for point_x, point_y in points_2d:
        position = _vector_add(
            workplane.origin,
            _vector_add(_vector_scale(workplane.x_axis, point_x), _vector_scale(workplane.y_axis, point_y)),
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
    strokes: Sequence[StrokeSegment],
    workplane: WorkplaneAxes,
    reference_frame: str,
    approach_distance_m: float,
    retract_distance_m: float,
    drawing_speed_scale: float,
    travel_speed_scale: float,
    plan_only: bool,
    max_waypoints_per_chunk: int,
) -> DrawingCompileResult:
    if approach_distance_m < 0.0 or retract_distance_m < 0.0:
        raise DrawingGeometryError("invalid_size: approach/retract distance must be >= 0")
    if drawing_speed_scale <= 0.0 or travel_speed_scale <= 0.0:
        raise DrawingGeometryError("invalid_size: drawing/travel speed scales must be > 0")
    if max_waypoints_per_chunk < 1:
        raise DrawingGeometryError("max_waypoints_per_chunk must be >= 1")

    commands: List[Dict[str, Any]] = []
    chunk_count = 0
    draw_stroke_count = 0
    lifted_point_count = 0

    for stroke_index, stroke in enumerate(strokes):
        if stroke.kind != "draw":
            continue
        if len(stroke.points_2d) < 2:
            raise DrawingGeometryError("invalid_size: draw stroke must contain at least 2 points")

        draw_stroke_count += 1
        lifted_poses = list(lift_points_to_poses(stroke.points_2d, workplane))
        lifted_point_count += len(lifted_poses)

        start_pose = lifted_poses[0]
        end_pose = lifted_poses[-1]
        above_start_pose = _offset_pose_along_normal(start_pose, workplane.normal, approach_distance_m)
        above_end_pose = _offset_pose_along_normal(end_pose, workplane.normal, retract_distance_m)

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


def supported_glyphs() -> Tuple[str, ...]:
    return tuple(sorted(GLYPHS.keys()))


def _ptp_command(
    *,
    target_pose: Dict[str, Any],
    reference_frame: str,
    speed_scale: float,
    plan_only: bool,
) -> Dict[str, Any]:
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
    target_pose: Dict[str, Any],
    reference_frame: str,
    speed_scale: float,
    plan_only: bool,
) -> Dict[str, Any]:
    return {
        "primitive_type": "LIN",
        "target_pose": target_pose,
        "reference_frame": reference_frame,
        "velocity_scale": float(speed_scale),
        "acceleration_scale": float(speed_scale),
        "planner_id": "PILZ_LIN",
        "plan_only": bool(plan_only),
    }


def _cartesian_path_command(
    *,
    waypoints: Sequence[Dict[str, Any]],
    reference_frame: str,
    speed_scale: float,
    plan_only: bool,
    chunk_index: int,
    stroke_index: int,
) -> Dict[str, Any]:
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


def _offset_pose_along_normal(pose: Mapping[str, Any], normal: Vector3, distance_m: float) -> Dict[str, Any]:
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


def _chunk_sequence(items: Sequence[Dict[str, Any]], chunk_size: int) -> Iterable[Sequence[Dict[str, Any]]]:
    if not items:
        return []
    return [items[index:index + chunk_size] for index in range(0, len(items), chunk_size)]


def _line_advance_width(text: str, *, height_m: float, char_spacing_m: float) -> float:
    if not text:
        return 0.0
    width = 0.0
    for index, character in enumerate(text):
        glyph = GLYPHS.get(character)
        if glyph is None:
            raise DrawingGeometryError(f"unsupported_font_glyph: '{character}'")
        width += glyph.advance_width * height_m
        if index != len(text) - 1:
            width += char_spacing_m
    return width


def _segment_angle_from_chord_error(radius_m: float, max_chord_error_m: float) -> float:
    ratio = 1.0 - min(max(max_chord_error_m / radius_m, 0.0), 1.999999)
    value = max(-1.0, min(1.0, ratio))
    theta = 2.0 * math.acos(value)
    if theta <= 1e-6:
        return math.radians(1.0)
    return theta


def _point_distance(a: Point2D, b: Point2D) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return math.sqrt((dx * dx) + (dy * dy))


def _vector_add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vector_scale(v: Vector3, scalar: float) -> Vector3:
    return (v[0] * scalar, v[1] * scalar, v[2] * scalar)


def _norm(v: Vector3) -> float:
    return math.sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]))


def _normalize(v: Vector3, *, field_name: str) -> Vector3:
    magnitude = _norm(v)
    if not math.isfinite(magnitude) or magnitude <= 1e-12:
        raise DrawingGeometryError(f"{field_name} must be a non-zero finite vector")
    return (v[0] / magnitude, v[1] / magnitude, v[2] / magnitude)


def _cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        (a[1] * b[2]) - (a[2] * b[1]),
        (a[2] * b[0]) - (a[0] * b[2]),
        (a[0] * b[1]) - (a[1] * b[0]),
    )


def _quaternion_basis(q: Mapping[str, float]) -> Tuple[Vector3, Vector3, Vector3]:
    qx = float(q["x"])
    qy = float(q["y"])
    qz = float(q["z"])
    qw = float(q["w"])

    # Rotation matrix columns (world vectors of local unit axes).
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    basis_x = (
        1.0 - 2.0 * (yy + zz),
        2.0 * (xy + wz),
        2.0 * (xz - wy),
    )
    basis_y = (
        2.0 * (xy - wz),
        1.0 - 2.0 * (xx + zz),
        2.0 * (yz + wx),
    )
    basis_z = (
        2.0 * (xz + wy),
        2.0 * (yz - wx),
        1.0 - 2.0 * (xx + yy),
    )
    return basis_x, basis_y, basis_z


__all__ = [
    "DrawingCompileResult",
    "DrawingGeometryError",
    "Point2D",
    "StrokeSegment",
    "WorkplaneAxes",
    "compile_strokes_to_commands",
    "generate_arc_path",
    "generate_circle_path",
    "generate_polygon_path",
    "generate_polyline_path",
    "generate_rectangle_path",
    "generate_square_path",
    "generate_text_stroke_segments",
    "generate_triangle_path",
    "lift_points_to_poses",
    "parse_position_dict",
    "parse_vector_dict",
    "resolve_workplane",
    "supported_glyphs",
    "to_meters",
    "to_radians",
]
