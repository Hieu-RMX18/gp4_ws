"""Route Semantic IR into public primitive commands without executing motion."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import math
import os
from pathlib import Path
from typing import Any, Dict, List

import yaml

try:  # pragma: no cover - optional in source-only test environments
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - exercised implicitly in tests
    get_package_share_directory = None

from llm_gateway.stroke_font import SUPPORTED_GLYPHS, generate_text_strokes


_ORIENTATION_PRESETS: Dict[str, Dict[str, float]] = {
    "tool-down": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
    "tool-forward": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
    "tool-up": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
}
_FRAME_BASE_LINK = "base_link"


def _default_macro_policy_path() -> str:
    """Resolve macro_policy.yaml from the installed package or local source tree."""
    if get_package_share_directory is not None:
        try:
            pkg_share = get_package_share_directory("llm_gateway")
            candidate = os.path.join(pkg_share, "config", "macro_policy.yaml")
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


def load_macro_policy(policy_path: str | None = None) -> Dict[str, Any]:
    resolved_path = policy_path or _default_macro_policy_path()
    with open(resolved_path, "r", encoding="utf-8") as policy_file:
        policy = yaml.safe_load(policy_file) or {}
    if not isinstance(policy, dict):
        raise ValueError("Macro policy root must be an object.")
    macros = policy.get("macros")
    if not isinstance(macros, dict):
        raise ValueError("Macro policy must define a macros object.")
    return policy


@dataclass(frozen=True)
class RouteResult:
    route_type: str
    commands: List[Dict[str, Any]]
    error_payload: Dict[str, Any] | None = None
    diagnostics: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)


class IntentRouter:
    """Translate Semantic IR into the frozen public primitive set."""

    def __init__(
        self,
        macro_policy_path: str | None = None,
        runtime_mode: str = "hardware",
    ) -> None:
        self._macro_policy = load_macro_policy(macro_policy_path)
        self._runtime_mode = str(runtime_mode).strip().lower() or "hardware"

    def route(self, payload: Dict[str, Any]) -> RouteResult:
        if not isinstance(payload, dict):
            raise ValueError("Semantic IR payload must be an object.")

        if "primitive_type" in payload:
            return RouteResult(
                route_type="primitive",
                commands=[deepcopy(payload)],
                metadata={"source": "primitive_passthrough"},
            )

        if "error" in payload:
            return RouteResult(
                route_type="error",
                commands=[],
                error_payload=deepcopy(payload),
                diagnostics=[str(payload["error"])],
                metadata={"source": "error_passthrough"},
            )

        intent = payload.get("intent")
        if not isinstance(intent, str) or not intent.strip():
            raise ValueError("Semantic IR requires a non-empty intent field.")

        normalized_intent = intent.strip()
        if normalized_intent == "sequence":
            return self._route_sequence(payload)
        if normalized_intent == "draw_shape":
            return self._route_draw_shape(payload)
        if normalized_intent == "draw_text":
            return self._route_draw_text(payload)

        return RouteResult(
            route_type="primitive",
            commands=[self._route_single_intent(payload)],
            metadata={"source": "semantic_ir", "intent": normalized_intent},
        )

    def _route_sequence(self, payload: Dict[str, Any]) -> RouteResult:
        steps = payload.get("steps")
        if not isinstance(steps, list) or not steps:
            raise ValueError("sequence intent requires a non-empty steps list.")

        commands: List[Dict[str, Any]] = []
        for index, step in enumerate(steps):
            if not isinstance(step, dict):
                raise ValueError(f"sequence step {index} must be an object.")
            routed_step = self.route(step)
            if routed_step.route_type == "error":
                error_code = routed_step.error_payload.get("error", "unknown_error")
                raise ValueError(f"sequence step {index} is an error payload: {error_code}")
            commands.extend(deepcopy(routed_step.commands))

        return RouteResult(
            route_type="sequence",
            commands=commands,
            metadata={"source": "semantic_ir", "intent": "sequence"},
        )

    def _route_single_intent(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        intent = str(payload["intent"])

        if intent == "go_home":
            return self._base_command("HOME", payload)
        if intent == "absolute_move_ptp":
            return self._route_absolute_move(payload, primitive_type="PTP")
        if intent == "absolute_move_lin":
            return self._route_absolute_move(payload, primitive_type="LIN")
        if intent == "move_relative":
            return self._route_move_relative(payload)
        if intent == "move_joint":
            return self._route_move_joint(payload)
        if intent == "move_joints":
            return self._route_move_joints(payload)
        if intent == "set_speed":
            return {"primitive_type": "SET_SPEED", "velocity_scale": float(payload["velocity_scale"])}
        if intent == "wait":
            return {"primitive_type": "WAIT", "wait_duration_sec": float(payload["wait_duration_sec"])}
        if intent == "stop":
            return {"primitive_type": "STOP"}
        if intent == "io_set":
            return {
                "primitive_type": "IO_SET",
                "io_address": int(payload["io_address"]),
                "io_value": int(payload["io_value"]),
            }
        if intent == "alarm_reset":
            return {"primitive_type": "ALARM_RESET"}
        if intent == "get_pose":
            command = {"primitive_type": "GET_POSE"}
            reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
            self._validate_reference_frame(reference_frame)
            command["reference_frame"] = reference_frame
            return command

        raise ValueError(f"unsupported semantic intent '{intent}'")

    def _route_absolute_move(self, payload: Dict[str, Any], *, primitive_type: str) -> Dict[str, Any]:
        target_pose = payload.get("target_pose")
        if not isinstance(target_pose, dict):
            raise ValueError(f"{payload['intent']} requires target_pose.")

        reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
        self._validate_reference_frame(reference_frame)

        command = self._base_command(primitive_type, payload)
        command["reference_frame"] = reference_frame
        command["target_pose"] = self._resolve_target_pose(
            target_pose=target_pose,
            orientation_preset=payload.get("orientation_preset"),
            keep_current_orientation=payload.get("keep_current_orientation"),
        )
        return command

    def _route_move_relative(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        delta = payload.get("delta")
        if not isinstance(delta, dict):
            raise ValueError("move_relative requires delta.")

        reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
        self._validate_reference_frame(reference_frame)
        return {
            "primitive_type": "MOVE_REL",
            "delta_x": float(delta.get("x", 0.0)),
            "delta_y": float(delta.get("y", 0.0)),
            "delta_z": float(delta.get("z", 0.0)),
            "reference_frame": reference_frame,
            **self._optional_motion_fields(payload),
        }

    def _route_move_joint(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "primitive_type": "MOVE_JOINT",
            "joint_index": int(payload["joint_index"]),
            "joint_angle": float(payload["joint_angle"]),
            **self._optional_motion_fields(payload),
        }

    def _route_move_joints(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        joint_target = payload.get("joint_target")
        if not isinstance(joint_target, list):
            raise ValueError("move_joints requires joint_target.")
        return {
            "primitive_type": "MOVE_JOINTS",
            "joint_target": [float(value) for value in joint_target],
            **self._optional_motion_fields(payload),
        }

    def _route_draw_shape(self, payload: Dict[str, Any]) -> RouteResult:
        policy = self._macro_policy["macros"].get("draw_shape")
        if not isinstance(policy, dict):
            raise ValueError("Macro policy missing draw_shape configuration.")

        if policy.get("availability") == "sim_only" and self._runtime_mode != "sim":
            raise ValueError("draw_shape is sim-only and is unavailable in hardware mode.")

        shape = str(payload.get("shape", "")).strip().lower()
        supported_shapes = {str(item).strip().lower() for item in policy.get("supported_shapes", [])}
        if shape not in supported_shapes:
            raise ValueError(
                f"draw_shape: unsupported shape '{shape}'; supported shapes: {sorted(supported_shapes)}"
            )

        plane = str(payload.get("plane", "")).strip().lower()
        supported_planes = {str(item).strip().lower() for item in policy.get("supported_planes", [])}
        if plane not in supported_planes:
            raise ValueError(
                f"draw_shape: unsupported plane '{plane}'; supported planes: {sorted(supported_planes)}"
            )

        reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
        self._validate_reference_frame(
            reference_frame,
            allowed_frames={str(item) for item in policy.get("supported_frames", [])},
        )

        motion_fields = self._optional_motion_fields(payload)

        if shape == "square":
            commands = self._build_square_sequence(
                start_pose=self._resolve_macro_start_pose(payload, "draw_shape", policy),
                size_m=self._require_positive_float(payload, "size_m", "draw_shape requires size_m > 0."),
                reference_frame=reference_frame,
                motion_fields=motion_fields,
            )
        elif shape == "triangle":
            commands = self._build_triangle_sequence(
                start_pose=self._resolve_macro_start_pose(payload, "draw_shape", policy),
                size_m=self._require_positive_float(payload, "size_m", "draw_shape requires size_m > 0."),
                reference_frame=reference_frame,
                motion_fields=motion_fields,
            )
        elif shape == "circle":
            segments = int(payload.get("segments", policy.get("circle_segments", 32)))
            if segments < 8:
                raise ValueError("draw_shape: circle segments must be >= 8.")
            commands = self._build_circle_sequence(
                start_pose=self._resolve_macro_start_pose(payload, "draw_shape", policy),
                size_m=self._require_positive_float(payload, "size_m", "draw_shape requires size_m > 0."),
                reference_frame=reference_frame,
                motion_fields=motion_fields,
                segments=segments,
            )
        elif shape == "polygon":
            default_sides = int(policy.get("polygon_default_sides", 6))
            min_sides = int(policy.get("polygon_min_sides", 3))
            max_sides = int(policy.get("polygon_max_sides", 12))
            sides = int(payload.get("sides", default_sides))
            if sides < min_sides or sides > max_sides:
                raise ValueError(
                    f"draw_shape: polygon sides must be in [{min_sides}, {max_sides}], got {sides}."
                )
            commands = self._build_polygon_sequence(
                start_pose=self._resolve_macro_start_pose(payload, "draw_shape", policy),
                size_m=self._require_positive_float(payload, "size_m", "draw_shape requires size_m > 0."),
                reference_frame=reference_frame,
                motion_fields=motion_fields,
                sides=sides,
            )
        elif shape == "rectangle":
            commands = self._build_rectangle_sequence(
                start_pose=self._resolve_macro_start_pose(payload, "draw_shape", policy),
                width_m=self._require_positive_float(
                    payload, "width_m", "draw_shape rectangle requires width_m > 0."
                ),
                height_m=self._require_positive_float(
                    payload, "height_m", "draw_shape rectangle requires height_m > 0."
                ),
                reference_frame=reference_frame,
                motion_fields=motion_fields,
            )
        elif shape == "arc":
            sweep_deg = float(payload.get("sweep_deg", 180.0))
            if math.isclose(sweep_deg, 0.0, abs_tol=1e-9):
                raise ValueError("draw_shape arc requires non-zero sweep_deg.")
            default_circle_segments = int(policy.get("circle_segments", 32))
            default_arc_segments = max(
                8,
                int(math.ceil(default_circle_segments * abs(sweep_deg) / 360.0)),
            )
            segments = int(payload.get("segments", default_arc_segments))
            if segments < 2:
                raise ValueError("draw_shape arc segments must be >= 2.")
            commands = self._build_arc_sequence(
                start_pose=self._resolve_macro_start_pose(payload, "draw_shape", policy),
                radius_m=self._require_positive_float(
                    payload, "radius_m", "draw_shape arc requires radius_m > 0."
                ),
                sweep_deg=sweep_deg,
                reference_frame=reference_frame,
                motion_fields=motion_fields,
                segments=segments,
            )
        elif shape == "polyline":
            raw_points = payload.get("points")
            if not isinstance(raw_points, list) or len(raw_points) < 2:
                raise ValueError("draw_shape polyline requires at least 2 points.")
            commands = self._build_polyline_sequence(
                points=[
                    self._resolve_position(point, field_prefix=f"points[{index}]")
                    for index, point in enumerate(raw_points)
                ],
                orientation=self._orientation_from_preset(policy.get("default_orientation_preset")),
                reference_frame=reference_frame,
                motion_fields=motion_fields,
            )
        else:
            raise ValueError(f"draw_shape: no builder for shape '{shape}'.")

        return RouteResult(
            route_type="sequence",
            commands=commands,
            metadata={
                "source": "semantic_ir",
                "macro_name": "draw_shape",
                "requires_current_pose": bool(policy.get("requires_current_pose", False)),
            },
        )

    def _route_draw_text(self, payload: Dict[str, Any]) -> RouteResult:
        policy = self._macro_policy["macros"].get("draw_text")
        if not isinstance(policy, dict):
            raise ValueError("Macro policy missing draw_text configuration.")

        if policy.get("availability") == "sim_only" and self._runtime_mode != "sim":
            raise ValueError("draw_text is sim-only and is unavailable in hardware mode.")

        plane = str(payload.get("plane", "")).strip().lower()
        supported_planes = {str(item).strip().lower() for item in policy.get("supported_planes", [])}
        if plane not in supported_planes:
            raise ValueError(
                f"draw_text: unsupported plane '{plane}'; supported planes: {sorted(supported_planes)}"
            )

        reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
        self._validate_reference_frame(
            reference_frame,
            allowed_frames={str(item) for item in policy.get("supported_frames", [])},
        )

        text = payload.get("text")
        if not isinstance(text, str) or not text:
            raise ValueError("draw_text requires a non-empty text string.")

        supported_characters = {str(item) for item in policy.get("supported_characters", [])}
        if not supported_characters:
            supported_characters = set(SUPPORTED_GLYPHS)
        unsupported_characters = sorted({character for character in text if character not in supported_characters})
        if unsupported_characters:
            raise ValueError(
                "draw_text: unsupported characters "
                f"{unsupported_characters}; supported characters: {sorted(supported_characters)}"
            )

        height_m = self._require_positive_float(payload, "height_m", "draw_text requires height_m > 0.")
        char_spacing_m = float(
            payload.get(
                "char_spacing_m",
                float(policy.get("default_char_spacing_ratio", 0.20)) * height_m,
            )
        )
        if char_spacing_m < 0.0:
            raise ValueError("draw_text requires char_spacing_m >= 0.")

        approach_distance_m = float(payload.get("approach_distance_m", policy.get("approach_distance_m", 0.01)))
        if approach_distance_m < 0.0:
            raise ValueError("draw_text requires approach_distance_m >= 0.")

        resolved_start_pose = self._resolve_macro_start_pose(payload, "draw_text", policy)
        stroke_segments = generate_text_strokes(
            text=text,
            height_m=height_m,
            char_spacing_m=char_spacing_m,
        )
        if not any(segment.kind == "draw" for segment in stroke_segments):
            raise ValueError("draw_text requires at least one drawable character.")

        origin = resolved_start_pose["position"]
        orientation = deepcopy(resolved_start_pose.get("orientation"))
        motion_fields = self._optional_motion_fields(payload)
        commands: List[Dict[str, Any]] = []
        next_draw_has_approach = False

        for segment in stroke_segments:
            segment_points = [
                {
                    "x": origin["x"] + offset_x,
                    "y": origin["y"] + offset_y,
                    "z": origin["z"],
                }
                for offset_x, offset_y in segment.points_2d
            ]

            if segment.kind == "travel":
                commands.append(
                    self._build_ptp_command(
                        position={
                            "x": segment_points[-1]["x"],
                            "y": segment_points[-1]["y"],
                            "z": segment_points[-1]["z"] + approach_distance_m,
                        },
                        orientation=orientation,
                        reference_frame=reference_frame,
                        motion_fields=motion_fields,
                    )
                )
                next_draw_has_approach = True
                continue

            if not next_draw_has_approach:
                commands.append(
                    self._build_ptp_command(
                        position={
                            "x": segment_points[0]["x"],
                            "y": segment_points[0]["y"],
                            "z": segment_points[0]["z"] + approach_distance_m,
                        },
                        orientation=orientation,
                        reference_frame=reference_frame,
                        motion_fields=motion_fields,
                    )
                )

            commands.append(
                self._build_cartesian_path_command(
                    points=segment_points,
                    orientation=orientation,
                    reference_frame=reference_frame,
                    motion_fields=motion_fields,
                )
            )
            next_draw_has_approach = False

        return RouteResult(
            route_type="sequence",
            commands=commands,
            metadata={
                "source": "semantic_ir",
                "macro_name": "draw_text",
                "requires_current_pose": bool(policy.get("requires_current_pose", False)),
                "text": text,
            },
        )

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
            {"x": position["x"] + size_m, "y": position["y"] + size_m, "z": position["z"]},
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
            points.append({
                "x": center_x - radius * math.cos(theta),
                "y": center_y + radius * math.sin(theta),
                "z": sz,
            })
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
            points.append({
                "x": center_x - radius * math.cos(theta),
                "y": center_y + radius * math.sin(theta),
                "z": sz,
            })
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
            {"x": position["x"] + width_m, "y": position["y"] + height_m, "z": position["z"]},
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
            "waypoints": [self._pose_from_position(point, orientation) for point in points],
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

    def _resolve_position(self, raw_position: Any, *, field_prefix: str) -> Dict[str, float]:
        if not isinstance(raw_position, dict):
            raise ValueError(f"{field_prefix} must be an object with x/y/z.")
        resolved_position: Dict[str, float] = {}
        for axis in ("x", "y", "z"):
            if axis not in raw_position:
                raise ValueError(f"{field_prefix}.{axis} is required.")
            resolved_position[axis] = float(raw_position[axis])
        return resolved_position

    def _require_positive_float(
        self,
        payload: Dict[str, Any],
        field_name: str,
        error_message: str,
    ) -> float:
        value = float(payload.get(field_name, 0.0))
        if value <= 0.0:
            raise ValueError(error_message)
        return value

    def _base_command(self, primitive_type: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        return {"primitive_type": primitive_type, **self._optional_motion_fields(payload)}

    def _optional_motion_fields(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command: Dict[str, Any] = {}
        for field in ("velocity_scale", "acceleration_scale", "planner_id", "require_approval"):
            if field in payload:
                command[field] = payload[field]
        return command

    def _resolve_target_pose(
        self,
        *,
        target_pose: Dict[str, Any],
        orientation_preset: Any,
        keep_current_orientation: Any,
        allow_orientation_default: bool = False,
    ) -> Dict[str, Any]:
        resolved_pose = deepcopy(target_pose)
        position = resolved_pose.get("position")
        if not isinstance(position, dict):
            raise ValueError("target_pose.position must be an object.")

        if "orientation" in resolved_pose and orientation_preset is not None:
            raise ValueError("Provide either target_pose.orientation or orientation_preset, not both.")

        if "orientation" in resolved_pose:
            return resolved_pose

        if orientation_preset is not None:
            resolved_pose["orientation"] = self._orientation_from_preset(orientation_preset)
            return resolved_pose

        if keep_current_orientation not in (None, True, False):
            raise ValueError("keep_current_orientation must be a boolean when provided.")

        if allow_orientation_default:
            return resolved_pose

        if keep_current_orientation is False:
            raise ValueError(
                "Explicit orientation is required when keep_current_orientation is false."
            )
        return resolved_pose

    def _orientation_from_preset(self, preset_name: Any) -> Dict[str, float]:
        normalized_name = str(preset_name).strip().lower().replace("_", "-")
        preset = _ORIENTATION_PRESETS.get(normalized_name)
        if preset is None:
            raise ValueError(
                f"unsupported orientation_preset '{preset_name}'; "
                f"supported presets: {sorted(_ORIENTATION_PRESETS)}"
            )
        return deepcopy(preset)

    def _validate_reference_frame(
        self,
        reference_frame: Any,
        *,
        allowed_frames: set[str] | None = None,
    ) -> None:
        normalized_frame = str(reference_frame).strip()
        frames = allowed_frames if allowed_frames is not None else {_FRAME_BASE_LINK}
        if normalized_frame not in frames:
            raise ValueError(
                f"unsupported reference_frame '{normalized_frame}'; "
                f"supported frames: {sorted(frames)}"
            )


__all__ = ["IntentRouter", "RouteResult", "load_macro_policy"]
