"""Route Semantic IR into public primitive commands without executing motion."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import os
from pathlib import Path
from typing import Any, Dict, List

import yaml

try:  # pragma: no cover - optional in source-only test environments
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - exercised implicitly in tests
    get_package_share_directory = None

from llm_gateway.draw_router import DrawRouterMixin


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


class IntentRouter(DrawRouterMixin):
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
                raise ValueError(
                    f"sequence step {index} is an error payload: {error_code}"
                )
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
            return {
                "primitive_type": "SET_SPEED",
                "velocity_scale": float(payload["velocity_scale"]),
            }
        if intent == "wait":
            return {
                "primitive_type": "WAIT",
                "wait_duration_sec": float(payload["wait_duration_sec"]),
            }
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
        if intent == "circular_move":
            return self._route_circular_move(payload)

        raise ValueError(f"unsupported semantic intent '{intent}'")

    def _route_absolute_move(
        self, payload: Dict[str, Any], *, primitive_type: str
    ) -> Dict[str, Any]:
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

    def _route_circular_move(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Convert circular_move semantic intent to CIRC primitive.

        CIRC requires:
          - target_pose: final arc endpoint
          - waypoints: [auxiliary_pose] — the arc via-point
        """
        target_pose = payload.get("target_pose")
        if not isinstance(target_pose, dict):
            raise ValueError("circular_move requires target_pose.")

        auxiliary_pose = payload.get("auxiliary_pose")
        if not isinstance(auxiliary_pose, dict):
            raise ValueError("circular_move requires auxiliary_pose (arc via-point).")

        reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
        self._validate_reference_frame(reference_frame)

        resolved_target = self._resolve_target_pose(
            target_pose=target_pose,
            orientation_preset=payload.get("orientation_preset"),
            keep_current_orientation=payload.get("keep_current_orientation"),
        )
        resolved_auxiliary = self._resolve_target_pose(
            target_pose=auxiliary_pose,
            orientation_preset=payload.get("orientation_preset"),
            keep_current_orientation=payload.get("keep_current_orientation"),
        )

        command = self._base_command("CIRC", payload)
        command["reference_frame"] = reference_frame
        command["target_pose"] = resolved_target
        command["waypoints"] = [resolved_auxiliary]
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

    def _resolve_position(
        self, raw_position: Any, *, field_prefix: str
    ) -> Dict[str, float]:
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

    def _base_command(
        self, primitive_type: str, payload: Dict[str, Any]
    ) -> Dict[str, Any]:
        return {
            "primitive_type": primitive_type,
            **self._optional_motion_fields(payload),
        }

    def _optional_motion_fields(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command: Dict[str, Any] = {}
        for field in (
            "velocity_scale",
            "acceleration_scale",
            "planner_id",
            "linear_unit",
            "angular_unit",
        ):
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
            raise ValueError(
                "Provide either target_pose.orientation or orientation_preset, not both."
            )

        if "orientation" in resolved_pose:
            return resolved_pose

        if orientation_preset is not None:
            resolved_pose["orientation"] = self._orientation_from_preset(
                orientation_preset
            )
            return resolved_pose

        if keep_current_orientation not in (None, True, False):
            raise ValueError(
                "keep_current_orientation must be a boolean when provided."
            )

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
