"""Draw shape and draw text routing extracted from intent_router."""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Any, Dict, List, Mapping, Sequence, Tuple

from safety.policy_loader import load_safety_rules as _load_safety_rules

from llm_gateway.drawing_geometry import (
    DrawingGeometryError,
    compile_strokes_to_commands,
    generate_arc_path,
    generate_circle_path,
    generate_polygon_path,
    generate_polyline_path,
    generate_rectangle_path,
    generate_square_path,
    generate_text_stroke_segments,
    generate_triangle_path,
    parse_position_dict,
    parse_vector_dict,
    resolve_workplane,
    supported_glyphs,
    to_meters,
    to_radians,
)


_FRAME_BASE_LINK = "base_link"


def _build_route_result(*args: Any, **kwargs: Any) -> Any:
    # Late import avoids circular dependency: intent_router imports this module.
    from llm_gateway.intent_router import RouteResult

    return RouteResult(*args, **kwargs)


class DrawRouterMixin:
    """Mixin providing draw shape/text routing for IntentRouter."""

    def _route_draw_shape(self, payload: Dict[str, Any]) -> RouteResult:
        policy = self._macro_policy["macros"].get("draw_shape")
        if not isinstance(policy, dict):
            raise ValueError("Macro policy missing draw_shape configuration.")

        availability = str(policy.get("availability", "all")).strip().lower()
        if availability == "sim_only" and self._runtime_mode != "sim":
            raise ValueError(
                "draw_shape is sim-only and is unavailable in hardware mode."
            )
        if availability == "disabled":
            raise ValueError("draw_shape capability_unavailable")

        shape = str(payload.get("shape_type", payload.get("shape", ""))).strip().lower()
        supported_shapes = {
            str(item).strip().lower() for item in policy.get("supported_shapes", [])
        }
        if shape not in supported_shapes:
            raise ValueError(
                f"unsupported_shape_type: '{shape}'; supported: {sorted(supported_shapes)}"
            )

        reference_frame = payload.get(
            "frame_id", payload.get("reference_frame", _FRAME_BASE_LINK)
        )
        self._validate_reference_frame(
            reference_frame,
            allowed_frames={str(item) for item in policy.get("supported_frames", [])},
        )

        units = (
            str(payload.get("units", policy.get("default_units", "m"))).strip().lower()
        )
        params = payload.get("params") or {}
        if not isinstance(params, dict):
            raise ValueError("draw_shape params must be an object.")

        try:
            workplane = self._resolve_draw_workplane(
                payload=payload,
                policy=policy,
                reference_frame=reference_frame,
            )
            execution_policy = self._resolve_execution_policy(payload, policy)
            stroke_config = self._resolve_stroke_config(payload, policy, units=units)
            max_chord_error_m = float(policy.get("max_chord_error_m", 0.0005))
            max_segment_angle_rad = math.radians(
                float(policy.get("max_segment_angle_deg", 10.0))
            )

            if shape == "square":
                side_m = self._extract_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("side_m", "side", "size_m", "size"),
                    error_label="square side",
                )
                strokes = generate_square_path(side_m=side_m)
            elif shape == "rectangle":
                width_m = self._extract_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("width_m", "width"),
                    error_label="rectangle width",
                )
                height_m = self._extract_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("height_m", "height"),
                    error_label="rectangle height",
                )
                strokes = generate_rectangle_path(width_m=width_m, height_m=height_m)
            elif shape == "triangle":
                explicit_points = self._extract_points_2d(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("points", "vertices"),
                    required=False,
                )
                if explicit_points is not None:
                    strokes = generate_triangle_path(
                        side_m=1.0, points_2d=explicit_points
                    )
                else:
                    side_m = self._extract_positive_length(
                        payload=payload,
                        params=params,
                        units=units,
                        candidates=("side_m", "side", "size_m", "size"),
                        error_label="triangle side",
                    )
                    strokes = generate_triangle_path(side_m=side_m)
            elif shape == "circle":
                radius_m = self._extract_circle_radius_m(
                    payload=payload, params=params, units=units
                )
                strokes = generate_circle_path(
                    radius_m=radius_m,
                    max_chord_error_m=max_chord_error_m,
                    max_segment_angle_rad=max_segment_angle_rad,
                )
            elif shape == "arc":
                radius_m = self._extract_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("radius_m", "radius"),
                    error_label="arc radius",
                )
                sweep_rad = self._extract_sweep_radians(payload=payload, params=params)
                strokes = generate_arc_path(
                    radius_m=radius_m,
                    sweep_rad=sweep_rad,
                    max_chord_error_m=max_chord_error_m,
                    max_segment_angle_rad=max_segment_angle_rad,
                )
            elif shape == "polygon":
                n_sides = self._extract_polygon_sides(
                    payload=payload, params=params, policy=policy
                )
                radius_candidate = self._extract_optional_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("radius_m", "radius"),
                )
                side_candidate = self._extract_optional_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("side_m", "side", "size_m", "size"),
                )
                strokes = generate_polygon_path(
                    n_sides=n_sides,
                    radius_m=radius_candidate,
                    side_m=side_candidate,
                )
            else:  # polyline
                points_2d = self._extract_points_2d(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("points", "vertices"),
                    required=True,
                )
                strokes = generate_polyline_path(points_2d=points_2d)

            drawing_cfg = _load_safety_rules().get("drawing", {})
            compiled = compile_strokes_to_commands(
                strokes=strokes,
                workplane=workplane,
                reference_frame=reference_frame,
                approach_distance_m=stroke_config["approach_distance_m"],
                retract_distance_m=stroke_config["retract_distance_m"],
                drawing_speed_scale=stroke_config["drawing_speed_scale"],
                travel_speed_scale=stroke_config["travel_speed_scale"],
                plan_only=execution_policy["plan_only"],
                max_waypoints_per_chunk=int(policy.get("max_waypoints_per_chunk", 80)),
                use_blended_sequence=bool(
                    drawing_cfg.get("use_blended_sequence", True)
                ),
                blend_radius_m=float(drawing_cfg.get("blend_radius_m", 0.008)),
            )
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

        return _build_route_result(
            route_type="sequence",
            commands=[deepcopy(command) for command in compiled.commands],
            metadata={
                "source": "semantic_ir",
                "macro_name": "draw_shape",
                "requires_current_pose": bool(
                    policy.get("requires_current_pose", False)
                ),
                "shape_type": shape,
                "execution_mode": execution_policy["execution_mode"],
                "summary": dict(compiled.summary),
            },
        )

    def _route_draw_text(self, payload: Dict[str, Any]) -> RouteResult:
        policy = self._macro_policy["macros"].get("draw_text")
        if not isinstance(policy, dict):
            raise ValueError("Macro policy missing draw_text configuration.")

        availability = str(policy.get("availability", "all")).strip().lower()
        if availability == "sim_only" and self._runtime_mode != "sim":
            raise ValueError(
                "draw_text is sim-only and is unavailable in hardware mode."
            )
        if availability == "disabled":
            raise ValueError("draw_text capability_unavailable")

        reference_frame = payload.get(
            "frame_id", payload.get("reference_frame", _FRAME_BASE_LINK)
        )
        self._validate_reference_frame(
            reference_frame,
            allowed_frames={str(item) for item in policy.get("supported_frames", [])},
        )

        text = payload.get("text")
        if not isinstance(text, str) or not text.strip():
            raise ValueError("draw_text requires a non-empty text string.")
        normalized_text = text.upper()

        supported_characters = {
            str(item) for item in policy.get("supported_characters", [])
        }
        if not supported_characters:
            supported_characters = set(supported_glyphs())
        unsupported_characters = sorted(
            {
                character
                for character in normalized_text
                if character not in supported_characters and character != "\n"
            }
        )
        if unsupported_characters:
            raise ValueError(
                "unsupported_font_glyph: "
                f"{unsupported_characters}; supported: {sorted(supported_characters)}"
            )

        units = (
            str(payload.get("units", policy.get("default_units", "m"))).strip().lower()
        )
        font = payload.get("font") or {}
        if not isinstance(font, dict):
            raise ValueError("draw_text font must be an object.")
        font_type = str(font.get("type", "single_stroke_builtin")).strip().lower()
        if font_type != "single_stroke_builtin":
            raise ValueError(
                "capability_unavailable: only single_stroke_builtin font is supported."
            )

        try:
            workplane = self._resolve_draw_workplane(
                payload=payload,
                policy=policy,
                reference_frame=reference_frame,
            )
            execution_policy = self._resolve_execution_policy(payload, policy)
            stroke_config = self._resolve_stroke_config(payload, policy, units=units)

            height_value = self._extract_any(
                payload=payload,
                params=font,
                candidates=("height_m", "height", "char_height_m"),
            )
            if height_value is None:
                raise DrawingGeometryError("missing_size: text height is required")
            height_m = to_meters(height_value, units, field_name="font.height")
            if height_m <= 0.0:
                raise DrawingGeometryError("invalid_size: text height must be > 0")

            default_char_spacing = (
                float(policy.get("default_char_spacing_ratio", 0.20)) * height_m
            )
            default_line_spacing = (
                float(policy.get("default_line_spacing_ratio", 0.50)) * height_m
            )

            char_spacing_raw = self._extract_any(
                payload=payload,
                params=font,
                candidates=("char_spacing_m", "char_spacing"),
            )
            line_spacing_raw = self._extract_any(
                payload=payload,
                params=font,
                candidates=("line_spacing_m", "line_spacing"),
            )
            char_spacing_m = (
                default_char_spacing
                if char_spacing_raw is None
                else to_meters(char_spacing_raw, units, field_name="font.char_spacing")
            )
            line_spacing_m = (
                default_line_spacing
                if line_spacing_raw is None
                else to_meters(line_spacing_raw, units, field_name="font.line_spacing")
            )

            alignment = (
                str(font.get("alignment", payload.get("alignment", "left")))
                .strip()
                .lower()
            )
            direction = (
                str(font.get("direction", payload.get("direction", "+x")))
                .strip()
                .lower()
            )
            if direction not in {"+x", "x", "local+x"}:
                raise DrawingGeometryError(
                    "capability_unavailable: only +X text direction is supported"
                )

            stroke_segments = generate_text_stroke_segments(
                text=normalized_text,
                height_m=height_m,
                char_spacing_m=char_spacing_m,
                line_spacing_m=line_spacing_m,
                alignment=alignment,
            )
            if not any(segment.kind == "draw" for segment in stroke_segments):
                raise DrawingGeometryError(
                    "unsupported_font_glyph: text has no drawable glyphs"
                )

            drawing_cfg = _load_safety_rules().get("drawing", {})
            compiled = compile_strokes_to_commands(
                strokes=stroke_segments,
                workplane=workplane,
                reference_frame=reference_frame,
                approach_distance_m=stroke_config["approach_distance_m"],
                retract_distance_m=stroke_config["retract_distance_m"],
                drawing_speed_scale=stroke_config["drawing_speed_scale"],
                travel_speed_scale=stroke_config["travel_speed_scale"],
                plan_only=execution_policy["plan_only"],
                max_waypoints_per_chunk=int(policy.get("max_waypoints_per_chunk", 80)),
                use_blended_sequence=bool(
                    drawing_cfg.get("use_blended_sequence", True)
                ),
                blend_radius_m=float(drawing_cfg.get("blend_radius_m", 0.008)),
            )
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

        return _build_route_result(
            route_type="sequence",
            commands=[deepcopy(command) for command in compiled.commands],
            metadata={
                "source": "semantic_ir",
                "macro_name": "draw_text",
                "requires_current_pose": bool(
                    policy.get("requires_current_pose", False)
                ),
                "text": normalized_text,
                "execution_mode": execution_policy["execution_mode"],
                "summary": dict(compiled.summary),
            },
        )

    def _resolve_draw_workplane(
        self,
        *,
        payload: Dict[str, Any],
        policy: Dict[str, Any],
        reference_frame: str,
    ):
        workplane_payload = payload.get("workplane") or {}
        if not isinstance(workplane_payload, dict):
            raise ValueError("workplane must be an object.")

        supported_modes = {
            str(item).strip().lower()
            for item in policy.get("supported_workplane_modes", ["base"])
        }
        mode = (
            str(
                workplane_payload.get(
                    "mode",
                    payload.get(
                        "plane_mode", policy.get("default_workplane_mode", "base")
                    ),
                )
            )
            .strip()
            .lower()
        )
        if mode not in supported_modes:
            raise ValueError(
                f"missing_workplane: unsupported mode '{mode}'; supported: {sorted(supported_modes)}"
            )

        start_pose = payload.get("start_pose")
        anchor_position: Tuple[float, float, float] | None = None
        if isinstance(start_pose, dict) and isinstance(
            start_pose.get("position"), dict
        ):
            try:
                anchor_position = parse_position_dict(
                    start_pose["position"], field_name="start_pose.position"
                )
            except DrawingGeometryError as exc:
                raise ValueError(str(exc)) from exc

        origin_pose = workplane_payload.get("origin")
        if origin_pose is None and mode == "tool" and isinstance(start_pose, dict):
            origin_pose = start_pose
        if origin_pose is None and mode == "explicit_pose":
            raise ValueError(
                "missing_workplane: explicit_pose requires workplane.origin"
            )

        normal = None
        x_axis_hint = None
        if isinstance(workplane_payload.get("normal"), dict):
            try:
                normal = parse_vector_dict(
                    workplane_payload["normal"], field_name="workplane.normal"
                )
            except DrawingGeometryError as exc:
                raise ValueError(str(exc)) from exc
        if isinstance(workplane_payload.get("x_axis_hint"), dict):
            try:
                x_axis_hint = parse_vector_dict(
                    workplane_payload["x_axis_hint"],
                    field_name="workplane.x_axis_hint",
                )
            except DrawingGeometryError as exc:
                raise ValueError(str(exc)) from exc

        default_orientation = self._orientation_from_preset(
            policy.get("default_orientation_preset", "tool-down")
        )
        try:
            return resolve_workplane(
                mode=mode,
                frame_id=reference_frame,
                anchor_position=anchor_position,
                origin_pose=origin_pose if isinstance(origin_pose, dict) else None,
                default_orientation=default_orientation,
                normal=normal,
                x_axis_hint=x_axis_hint,
            )
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

    def _resolve_execution_policy(
        self, payload: Dict[str, Any], policy: Dict[str, Any]
    ) -> Dict[str, Any]:
        execution_mode = (
            str(
                payload.get(
                    "execution_mode", policy.get("default_execution_mode", "execute")
                )
            )
            .strip()
            .lower()
        )
        if execution_mode not in {"execute", "plan_only"}:
            raise ValueError("execution_mode must be one of: execute, plan_only")

        return {
            "execution_mode": execution_mode,
            "plan_only": execution_mode == "plan_only",
        }

    def _resolve_stroke_config(
        self,
        payload: Dict[str, Any],
        policy: Dict[str, Any],
        *,
        units: str,
    ) -> Dict[str, float]:
        stroke_payload = payload.get("stroke") or {}
        if not isinstance(stroke_payload, dict):
            raise ValueError("stroke must be an object.")

        def _field(*names: str) -> Any:
            for name in names:
                if name in stroke_payload:
                    return stroke_payload[name]
                if name in payload:
                    return payload[name]
            return None

        try:
            approach_raw = _field("approach_distance_m", "approach_distance")
            retract_raw = _field("retract_distance_m", "retract_distance")
            approach_distance_m = to_meters(
                approach_raw
                if approach_raw is not None
                else policy.get("approach_distance_m", 0.01),
                units,
                field_name="stroke.approach_distance",
            )
            retract_distance_m = to_meters(
                retract_raw
                if retract_raw is not None
                else policy.get("retract_distance_m", 0.01),
                units,
                field_name="stroke.retract_distance",
            )
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

        drawing_speed_scale = float(
            _field("drawing_speed_scale")
            or policy.get("drawing_speed_scale", payload.get("velocity_scale", 0.06))
        )
        travel_speed_scale = float(
            _field("travel_speed_scale") or policy.get("travel_speed_scale", 0.06)
        )
        if drawing_speed_scale <= 0.0 or travel_speed_scale <= 0.0:
            raise ValueError(
                "invalid_size: drawing_speed_scale and travel_speed_scale must be > 0"
            )

        return {
            "approach_distance_m": approach_distance_m,
            "retract_distance_m": retract_distance_m,
            "drawing_speed_scale": drawing_speed_scale,
            "travel_speed_scale": travel_speed_scale,
        }

    def _extract_any(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        candidates: Sequence[str],
    ) -> Any:
        for candidate in candidates:
            if candidate in params:
                return params[candidate]
            if candidate in payload:
                return payload[candidate]
        return None

    def _extract_positive_length(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        units: str,
        candidates: Sequence[str],
        error_label: str,
    ) -> float:
        raw_value = self._extract_any(
            payload=payload, params=params, candidates=candidates
        )
        if raw_value is None:
            raise ValueError(f"missing_size: {error_label} is required")
        try:
            length_m = to_meters(raw_value, units, field_name=error_label)
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc
        if length_m <= 0.0:
            raise ValueError(f"invalid_size: {error_label} must be > 0")
        return length_m

    def _extract_optional_positive_length(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        units: str,
        candidates: Sequence[str],
    ) -> float | None:
        raw_value = self._extract_any(
            payload=payload, params=params, candidates=candidates
        )
        if raw_value is None:
            return None
        try:
            length_m = to_meters(raw_value, units, field_name=candidates[0])
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc
        if length_m <= 0.0:
            raise ValueError(f"invalid_size: {candidates[0]} must be > 0")
        return length_m

    def _extract_circle_radius_m(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        units: str,
    ) -> float:
        radius_m = self._extract_optional_positive_length(
            payload=payload,
            params=params,
            units=units,
            candidates=("radius_m", "radius"),
        )
        if radius_m is not None:
            return radius_m

        diameter_m = self._extract_optional_positive_length(
            payload=payload,
            params=params,
            units=units,
            candidates=("diameter_m", "diameter", "size_m", "size"),
        )
        if diameter_m is None:
            raise ValueError("missing_size: circle radius is required")
        return diameter_m * 0.5

    def _extract_sweep_radians(
        self, *, payload: Dict[str, Any], params: Mapping[str, Any]
    ) -> float:
        sweep_rad_raw = self._extract_any(
            payload=payload, params=params, candidates=("sweep_rad",)
        )
        sweep_deg_raw = self._extract_any(
            payload=payload, params=params, candidates=("sweep_deg", "sweep")
        )

        try:
            if sweep_rad_raw is not None:
                sweep_rad = to_radians(
                    sweep_rad_raw, field_name="arc sweep_rad", units="rad"
                )
            elif sweep_deg_raw is not None:
                sweep_rad = to_radians(
                    sweep_deg_raw, field_name="arc sweep_deg", units="deg"
                )
            else:
                sweep_rad = math.radians(180.0)
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

        if math.isclose(sweep_rad, 0.0, abs_tol=1e-12):
            raise ValueError("invalid_size: arc sweep must be non-zero")
        return sweep_rad

    def _extract_polygon_sides(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        policy: Mapping[str, Any],
    ) -> int:
        raw_value = self._extract_any(
            payload=payload, params=params, candidates=("n_sides", "sides")
        )
        if raw_value is None:
            raw_value = policy.get("polygon_default_sides", 6)
        try:
            n_sides = int(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "invalid_polygon_sides: n_sides must be an integer"
            ) from exc

        min_sides = int(policy.get("polygon_min_sides", 3))
        max_sides = int(policy.get("polygon_max_sides", 12))
        if n_sides < min_sides or n_sides > max_sides:
            raise ValueError(
                f"invalid_polygon_sides: n_sides must be in [{min_sides}, {max_sides}]"
            )
        return n_sides

    def _extract_points_2d(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        units: str,
        candidates: Sequence[str],
        required: bool,
    ) -> List[Tuple[float, float]] | None:
        raw_points = self._extract_any(
            payload=payload, params=params, candidates=candidates
        )
        if raw_points is None:
            if required:
                raise ValueError("invalid_size: points are required")
            return None
        if not isinstance(raw_points, list):
            raise ValueError("invalid_size: points must be an array")
        if required and len(raw_points) < 2:
            raise ValueError("invalid_size: polyline requires at least 2 points")

        points_2d: List[Tuple[float, float]] = []
        for index, point in enumerate(raw_points):
            if not isinstance(point, dict):
                raise ValueError(f"invalid_size: points[{index}] must be an object")
            if "x" not in point or "y" not in point:
                raise ValueError(f"invalid_size: points[{index}] requires x and y")
            try:
                x_m = to_meters(point["x"], units, field_name=f"points[{index}].x")
                y_m = to_meters(point["y"], units, field_name=f"points[{index}].y")
            except DrawingGeometryError as exc:
                raise ValueError(str(exc)) from exc
            points_2d.append((x_m, y_m))
        return points_2d

    def _build_square_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        size_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        points = [
            {"x": position["x"], "y": position["y"], "z": position["z"]},
            {"x": position["x"] + size_m, "y": position["y"], "z": position["z"]},
            {
                "x": position["x"] + size_m,
                "y": position["y"] + size_m,
                "z": position["z"],
            },
            {"x": position["x"], "y": position["y"] + size_m, "z": position["z"]},
            {"x": position["x"], "y": position["y"], "z": position["z"]},
        ]
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_triangle_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        size_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        """Equilateral triangle on XY plane. size_m = side length."""
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        sx, sy, sz = position["x"], position["y"], position["z"]
        points = [
            {"x": sx, "y": sy, "z": sz},
            {"x": sx + size_m, "y": sy, "z": sz},
            {"x": sx + size_m / 2.0, "y": sy + size_m * math.sqrt(3) / 2.0, "z": sz},
            {"x": sx, "y": sy, "z": sz},  # close path
        ]
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_circle_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        size_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
        segments: int = 32,
    ) -> List[Dict[str, Any]]:
        """Circle on XY plane approximated by N cartesian waypoints. size_m = diameter."""
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        radius = size_m / 2.0
        # Center offset so circle starts at start_pose
        center_x = position["x"] + radius
        center_y = position["y"]
        sz = position["z"]

        points: List[Dict[str, float]] = []
        for i in range(segments + 1):
            theta = 2.0 * math.pi * i / segments
            points.append(
                {
                    "x": center_x - radius * math.cos(theta),
                    "y": center_y + radius * math.sin(theta),
                    "z": sz,
                }
            )
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_polygon_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        size_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
        sides: int = 6,
    ) -> List[Dict[str, Any]]:
        """Regular polygon on XY plane. size_m = circumscribed diameter."""
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        radius = size_m / 2.0
        # Center offset so polygon starts at start_pose
        center_x = position["x"] + radius
        center_y = position["y"]
        sz = position["z"]

        points: List[Dict[str, float]] = []
        for i in range(sides + 1):  # +1 to close path
            theta = 2.0 * math.pi * i / sides
            points.append(
                {
                    "x": center_x - radius * math.cos(theta),
                    "y": center_y + radius * math.sin(theta),
                    "z": sz,
                }
            )
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_rectangle_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        width_m: float,
        height_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        points = [
            {"x": position["x"], "y": position["y"], "z": position["z"]},
            {"x": position["x"] + width_m, "y": position["y"], "z": position["z"]},
            {
                "x": position["x"] + width_m,
                "y": position["y"] + height_m,
                "z": position["z"],
            },
            {"x": position["x"], "y": position["y"] + height_m, "z": position["z"]},
            {"x": position["x"], "y": position["y"], "z": position["z"]},
        ]
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_arc_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        radius_m: float,
        sweep_deg: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
        segments: int,
    ) -> List[Dict[str, Any]]:
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        center_x = position["x"] + radius_m
        center_y = position["y"]
        sweep_rad = math.radians(sweep_deg)
        points: List[Dict[str, float]] = []
        for index in range(segments + 1):
            theta = sweep_rad * index / segments
            points.append(
                {
                    "x": center_x - radius_m * math.cos(theta),
                    "y": center_y + radius_m * math.sin(theta),
                    "z": position["z"],
                }
            )
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_polyline_sequence(
        self,
        *,
        points: List[Dict[str, float]],
        orientation: Dict[str, float] | None,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_cartesian_path_sequence(
        self,
        *,
        points: List[Dict[str, float]],
        orientation: Dict[str, float] | None,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        if len(points) < 2:
            raise ValueError("CARTESIAN_PATH sequences require at least 2 points.")
        return [
            self._build_ptp_command(
                position=points[0],
                orientation=orientation,
                reference_frame=reference_frame,
                motion_fields=motion_fields,
            ),
            self._build_cartesian_path_command(
                points=points[1:],
                orientation=orientation,
                reference_frame=reference_frame,
                motion_fields=motion_fields,
            ),
        ]

    def _build_ptp_command(
        self,
        *,
        position: Dict[str, float],
        orientation: Dict[str, float] | None,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> Dict[str, Any]:
        return {
            "primitive_type": "PTP",
            "reference_frame": reference_frame,
            "target_pose": self._pose_from_position(position, orientation),
            **deepcopy(motion_fields),
        }

    def _build_cartesian_path_command(
        self,
        *,
        points: List[Dict[str, float]],
        orientation: Dict[str, float] | None,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> Dict[str, Any]:
        if not points:
            raise ValueError("CARTESIAN_PATH requires at least one waypoint.")
        return {
            "primitive_type": "CARTESIAN_PATH",
            "reference_frame": reference_frame,
            "waypoints": [
                self._pose_from_position(point, orientation) for point in points
            ],
            **deepcopy(motion_fields),
        }

    def _pose_from_position(
        self, position: Dict[str, float], orientation: Dict[str, float] | None
    ) -> Dict[str, Any]:
        pose = {"position": deepcopy(position)}
        if orientation is not None:
            pose["orientation"] = deepcopy(orientation)
        return pose

    def _resolve_macro_start_pose(
        self,
        payload: Dict[str, Any],
        macro_name: str,
        policy: Dict[str, Any],
    ) -> Dict[str, Any]:
        start_pose = payload.get("start_pose")
        if not isinstance(start_pose, dict):
            raise ValueError(
                f"{macro_name} requires start_pose; current-pose-aware macros are not implemented."
            )
        return self._resolve_target_pose(
            target_pose=start_pose,
            orientation_preset=policy.get("default_orientation_preset"),
            keep_current_orientation=False,
            allow_orientation_default=True,
        )
