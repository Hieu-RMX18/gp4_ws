from __future__ import annotations

import json
import math
from pathlib import Path
import re
import sys
import unicodedata
from typing import Any, Callable

from ..domain.constants import GP4_JOINT_NAMES as JOINT_NAMES

_LLM_GATEWAY_SOURCE = Path(__file__).resolve().parents[3] / "src" / "llm_gateway"
if _LLM_GATEWAY_SOURCE.exists():
    llm_gateway_source = str(_LLM_GATEWAY_SOURCE)
    if llm_gateway_source not in sys.path:
        sys.path.append(llm_gateway_source)

_SAFETY_SOURCE = Path(__file__).resolve().parents[3] / "src" / "safety"
if _SAFETY_SOURCE.exists():
    safety_source = str(_SAFETY_SOURCE)
    if safety_source not in sys.path:
        sys.path.append(safety_source)

try:  # pragma: no cover - fallback logic is covered
    from llm_gateway.intent_router import IntentRouter
except Exception:  # pragma: no cover - depends on optional source path
    IntentRouter = None  # type: ignore[assignment]

try:  # pragma: no cover - fallback logic is covered
    from llm_gateway.sequence_validator import SequenceValidationError, SequenceValidator
except Exception:  # pragma: no cover - depends on optional source path
    SequenceValidationError = ValueError  # type: ignore[assignment]
    SequenceValidator = None  # type: ignore[assignment]

try:  # pragma: no cover - fallback logic is covered
    from llm_gateway.parser import parse_llm_output
except Exception:  # pragma: no cover - depends on optional source path
    parse_llm_output = None  # type: ignore[assignment]


UNIT_TO_METERS = {"mm": 0.001, "cm": 0.01, "m": 1.0}
DEFAULT_DRAW_TEXT_HEIGHT = 20.0
DEFAULT_DRAW_TEXT_UNITS = "mm"
ROUTED_DRAW_METADATA_FIELDS = {"plan_only", "chunk_index", "stroke_index"}
SUPPORTED_PRIMITIVES = {
    "HOME",
    "PTP",
    "LIN",
    "CIRC",
    "CARTESIAN_PATH",
    "MOVE_REL",
    "MOVE_JOINT",
    "MOVE_JOINTS",
    "WAIT",
    "STOP",
    "SET_SPEED",
    "IO_SET",
    "ALARM_RESET",
    "GET_POSE",
}
HARDWARE_WHITELIST = set(SUPPORTED_PRIMITIVES)
PLANNER_DEFAULTS = {
    "HOME": "PILZ_PTP",
    "PTP": "PILZ_PTP",
    "LIN": "PILZ_LIN",
    "CIRC": "PILZ_CIRC",
    "CARTESIAN_PATH": "PILZ_LIN",
    "MOVE_REL": "PILZ_LIN",
    "MOVE_JOINT": "PILZ_PTP",
    "MOVE_JOINTS": "PILZ_PTP",
}
MOTION_PRIMITIVES = {
    "HOME",
    "PTP",
    "LIN",
    "CIRC",
    "CARTESIAN_PATH",
    "MOVE_REL",
    "MOVE_JOINT",
    "MOVE_JOINTS",
}
_OLD_ACTIONS = {
    "move_home": "HOME",
    "home": "HOME",
    "stop": "STOP",
    "move_rel": "MOVE_REL",
    "move_cartesian_delta": "MOVE_REL",
    "move_joint": "MOVE_JOINT",
    "move_joint_delta": "MOVE_JOINT",
    "move_joints": "MOVE_JOINTS",
    "wait": "WAIT",
    "set_speed": "SET_SPEED",
    "io_set": "IO_SET",
    "alarm_reset": "ALARM_RESET",
    "get_pose": "GET_POSE",
    "ptp": "PTP",
    "lin": "LIN",
    "circ": "CIRC",
    "cartesian_path": "CARTESIAN_PATH",
}
_ALLOWED_FIELDS_BY_PRIMITIVE = {
    "HOME": {"velocity_scale", "acceleration_scale", "planner_id", "require_approval", "reference_frame"},
    "PTP": {
        "target_pose",
        "joint_target",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "require_approval",
        "reference_frame",
    },
    "LIN": {
        "target_pose",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "require_approval",
        "reference_frame",
    },
    "CIRC": {
        "target_pose",
        "waypoints",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "require_approval",
        "reference_frame",
    },
    "CARTESIAN_PATH": {
        "waypoints",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "require_approval",
        "reference_frame",
    },
    "MOVE_REL": {
        "delta_x",
        "delta_y",
        "delta_z",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "require_approval",
        "reference_frame",
    },
    "MOVE_JOINT": {
        "joint_index",
        "joint_angle",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "require_approval",
    },
    "MOVE_JOINTS": {
        "joint_target",
        "velocity_scale",
        "acceleration_scale",
        "planner_id",
        "require_approval",
    },
    "WAIT": {"wait_duration_sec", "reference_frame"},
    "STOP": {"reference_frame"},
    "SET_SPEED": {"velocity_scale"},
    "IO_SET": {"io_address", "io_value", "reference_frame"},
    "ALARM_RESET": {"reference_frame"},
    "GET_POSE": {"reference_frame"},
}
_CARTESIAN_DIRECTIONS = {
    "up": (0.0, 0.0, 1.0),
    "down": (0.0, 0.0, -1.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "forward": (1.0, 0.0, 0.0),
    "back": (-1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
}

_DRAW_TEXT_PREFIX_PATTERN = re.compile(r"^(?:write|ve\s+chu|vẽ\s+chữ|viet|viết)\s+(.+)$", re.IGNORECASE)


class IntentResolutionError(ValueError):
    def __init__(
        self,
        reason: str,
        *,
        missing_slots: list[str] | None = None,
        rejected_fields: list[str] | None = None,
    ) -> None:
        super().__init__(reason)
        self.reason = reason
        self.missing_slots = missing_slots or []
        self.rejected_fields = rejected_fields or []

    def operator_message(self) -> str:
        fragments = [self.reason]
        if self.missing_slots:
            fragments.append("missing fields: " + ", ".join(self.missing_slots))
        if self.rejected_fields:
            fragments.append("rejected fields: " + ", ".join(self.rejected_fields))
        return "; ".join(fragments)

    def to_dict(self) -> dict[str, Any]:
        return {
            "reason": self.reason,
            "missingSlots": list(self.missing_slots),
            "rejectedFields": list(self.rejected_fields),
        }


def _to_float(value: Any, field: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise IntentResolutionError(f"{field} must be numeric.") from exc


def _to_int(value: Any, field: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise IntentResolutionError(f"{field} must be an integer.") from exc


def _wrap_to_pi(angle_rad: float) -> float:
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    wrapped -= math.pi
    if wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def _normalize_angle(value: Any, field: str) -> float:
    angle = _to_float(value, field)
    if abs(angle) > (2.0 * math.pi):
        angle = math.radians(angle)
    return _wrap_to_pi(angle)


def _rpy_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> dict[str, float]:
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class IntentResolutionService:
    def __init__(
        self,
        *,
        default_velocity_scale: float = 0.06,
        default_acceleration_scale: float = 0.06,
        min_velocity_scale: float = 0.01,
        max_velocity_scale: float = 0.06,
        max_acceleration_scale: float = 0.06,
    ) -> None:
        self._default_velocity_scale = float(default_velocity_scale)
        self._default_acceleration_scale = float(default_acceleration_scale)
        self._min_velocity_scale = float(min_velocity_scale)
        self._max_velocity_scale = float(max_velocity_scale)
        self._max_acceleration_scale = float(max_acceleration_scale)

    def prepare_sequence_submission(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        runtime_mode: str,
        current_joints: list[Any],
        current_pose_loader: Callable[[], dict[str, Any] | None] | None = None,
    ) -> dict[str, Any] | None:
        intent_name = ""
        candidate_payload: dict[str, Any] | None = None
        force_sequence = False

        if structured_intent is not None:
            intent_name = str(structured_intent.get("intent") or "").strip().lower()
            if intent_name:
                candidate_payload = dict(structured_intent)
                force_sequence = intent_name in {"sequence", "draw_shape", "draw_text"}
        elif raw_text:
            parsed_draw = self._parse_draw_text_to_semantic(raw_text)
            if parsed_draw is not None:
                candidate_payload = parsed_draw
                intent_name = str(parsed_draw.get("intent") or "").strip().lower()
                force_sequence = True

        if candidate_payload is None:
            return None

        route_metadata = self._sequence_candidate_metadata(candidate_payload)
        diagnostics: list[str] = []
        working_payload = dict(candidate_payload)
        if intent_name in {"draw_shape", "draw_text"}:
            try:
                working_payload = self._hydrate_draw_workplane(
                    working_payload,
                    current_pose_loader=current_pose_loader,
                )
            except IntentResolutionError as exc:
                return {
                    "parsed_steps": None,
                    "diagnostics": diagnostics,
                    "parse_error": exc.operator_message(),
                    "route_metadata": route_metadata,
                    "structured_intent": working_payload,
                }

        if IntentRouter is None:
            if force_sequence:
                return {
                    "parsed_steps": None,
                    "diagnostics": diagnostics,
                    "parse_error": (
                        "semantic intent routing is unavailable because llm_gateway.intent_router is not importable."
                    ),
                    "route_metadata": route_metadata,
                    "structured_intent": working_payload,
                }
            return None

        try:
            routed = IntentRouter(runtime_mode=runtime_mode).route(working_payload)
        except ValueError as exc:
            if not force_sequence:
                return None
            return {
                "parsed_steps": None,
                "diagnostics": diagnostics,
                "parse_error": str(exc),
                "route_metadata": route_metadata,
                "structured_intent": working_payload,
            }

        route_metadata.update(dict(routed.metadata))
        is_sequence = routed.route_type == "sequence" or len(routed.commands) > 1
        if not is_sequence:
            return None

        if routed.metadata.get("macro_name") in {"draw_shape", "draw_text"}:
            for command in routed.commands:
                if bool(command.get("plan_only")):
                    return {
                        "parsed_steps": None,
                        "diagnostics": diagnostics,
                        "parse_error": (
                            "draw execution_mode=plan_only is not supported by the HMI; "
                            "resubmit with execution_mode='execute'."
                        ),
                        "route_metadata": route_metadata,
                        "structured_intent": working_payload,
                    }

        if SequenceValidator is None:
            return {
                "parsed_steps": None,
                "diagnostics": diagnostics,
                "parse_error": (
                    "sequence routing is unavailable because llm_gateway sequence helpers are not importable."
                ),
                "route_metadata": route_metadata,
                "structured_intent": working_payload,
            }

        try:
            validation = SequenceValidator().validate(routed.commands)
            diagnostics.extend(validation.diagnostics)
            parsed_steps = [
                self.resolve(
                    raw_text="",
                    structured_intent=command,
                    runtime_mode=runtime_mode,
                    current_joints=current_joints,
                    allow_routed_draw_metadata=bool(routed.metadata.get("macro_name") in {"draw_shape", "draw_text"}),
                )
                for command in routed.commands
            ]
        except (IntentResolutionError, SequenceValidationError, ValueError) as exc:
            message = exc.operator_message() if isinstance(exc, IntentResolutionError) else str(exc)
            return {
                "parsed_steps": None,
                "diagnostics": diagnostics,
                "parse_error": message,
                "route_metadata": route_metadata,
                "structured_intent": working_payload,
            }

        return {
            "parsed_steps": parsed_steps,
            "diagnostics": diagnostics,
            "parse_error": None,
            "route_metadata": route_metadata,
            "structured_intent": working_payload,
        }

    def resolve(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        runtime_mode: str,
        current_joints: list[Any],
        allow_routed_draw_metadata: bool = False,
    ) -> dict[str, Any]:
        mode = str(runtime_mode or "unknown").strip().lower() or "unknown"
        if structured_intent is not None:
            initial_command = self._structured_to_command(
                structured_intent=structured_intent,
                runtime_mode=mode,
                current_joints=current_joints,
            )
            normalized_text = json.dumps(structured_intent, separators=(",", ":"), ensure_ascii=True)
            source = "structured"
        else:
            initial_command = self._text_to_command(raw_text=raw_text, current_joints=current_joints)
            normalized_text = " ".join(raw_text.strip().split()).lower()
            source = "text"

        normalized_command, normalization_notes = self._normalize_command(
            command=initial_command,
            runtime_mode=mode,
            allow_routed_draw_metadata=allow_routed_draw_metadata,
        )
        primitive_type = str(normalized_command["primitive_type"])
        parameters = {
            key: value
            for key, value in normalized_command.items()
            if key != "primitive_type"
        }

        return {
            "source": source,
            "normalizedText": normalized_text,
            "action": primitive_type,
            "parameters": parameters,
            "targetSummary": self._target_summary(normalized_command),
            "normalizedCommand": normalized_command,
            "normalizationNotes": normalization_notes,
        }

    def _structured_to_command(
        self,
        *,
        structured_intent: dict[str, Any],
        runtime_mode: str,
        current_joints: list[Any],
    ) -> dict[str, Any]:
        payload = dict(structured_intent)

        if "primitive_type" in payload:
            return payload

        if "intent" in payload:
            if IntentRouter is None:
                raise IntentResolutionError(
                    "semantic intent routing is unavailable because llm_gateway.intent_router is not importable."
                )
            routed = IntentRouter(runtime_mode=runtime_mode).route(payload)
            if routed.route_type == "error":
                message = (routed.error_payload or {}).get("message") or (routed.error_payload or {}).get("error")
                raise IntentResolutionError(str(message or "intent router returned an error payload."))
            if len(routed.commands) != 1:
                raise IntentResolutionError(
                    f"semantic routing expected one command but produced {len(routed.commands)} commands."
                )
            return dict(routed.commands[0])

        if "action" not in payload:
            raise IntentResolutionError(
                "structuredIntent must include action, intent, or primitive_type.",
                missing_slots=["action|intent|primitive_type"],
            )

        return self._legacy_action_to_command(payload, current_joints=current_joints)

    def _legacy_action_to_command(
        self,
        payload: dict[str, Any],
        *,
        current_joints: list[Any],
    ) -> dict[str, Any]:
        action = str(payload.get("action") or "").strip().lower()
        parameters = dict(payload.get("parameters") or {})
        primitive = _OLD_ACTIONS.get(action, action.upper())

        if primitive == "HOME":
            return {"primitive_type": "HOME"}
        if primitive == "STOP":
            return {"primitive_type": "STOP"}
        if primitive == "WAIT":
            return {"primitive_type": "WAIT", "wait_duration_sec": parameters.get("wait_duration_sec")}
        if primitive == "SET_SPEED":
            return {"primitive_type": "SET_SPEED", "velocity_scale": parameters.get("velocity_scale")}
        if primitive == "IO_SET":
            return {
                "primitive_type": "IO_SET",
                "io_address": parameters.get("io_address"),
                "io_value": parameters.get("io_value"),
            }
        if primitive == "ALARM_RESET":
            return {"primitive_type": "ALARM_RESET"}
        if primitive == "GET_POSE":
            return {"primitive_type": "GET_POSE", "reference_frame": parameters.get("reference_frame", "base_link")}
        if primitive == "MOVE_REL":
            frame = str(parameters.get("frame") or parameters.get("reference_frame") or "base_link")
            return {
                "primitive_type": "MOVE_REL",
                "delta_x": _to_float(parameters.get("xMm", 0.0), "parameters.xMm") / 1000.0,
                "delta_y": _to_float(parameters.get("yMm", 0.0), "parameters.yMm") / 1000.0,
                "delta_z": _to_float(parameters.get("zMm", 0.0), "parameters.zMm") / 1000.0,
                "reference_frame": frame,
                "velocity_scale": parameters.get("velocity_scale"),
                "acceleration_scale": parameters.get("acceleration_scale"),
            }
        if primitive == "MOVE_JOINT":
            joint_index = self._resolve_joint_index(parameters)
            if joint_index is None:
                raise IntentResolutionError(
                    "MOVE_JOINT action did not resolve to a valid GP4 joint index.",
                    missing_slots=["joint_index"],
                )
            resolved_target = parameters.get("resolvedTargetDeg")
            if resolved_target is None:
                current_deg = self._read_joint_deg(current_joints=current_joints, joint_index=joint_index)
                if current_deg is None:
                    raise IntentResolutionError(
                        f"fresh joint position for {JOINT_NAMES[joint_index]} is unavailable.",
                        missing_slots=["fresh_joint_position"],
                    )
                resolved_target = float(current_deg) + _to_float(parameters.get("deltaDeg", 0.0), "parameters.deltaDeg")
            return {
                "primitive_type": "MOVE_JOINT",
                "joint_index": joint_index,
                "joint_angle": float(resolved_target),
                "velocity_scale": parameters.get("velocity_scale"),
                "acceleration_scale": parameters.get("acceleration_scale"),
            }
        if primitive == "MOVE_JOINTS":
            return {
                "primitive_type": "MOVE_JOINTS",
                "joint_target": parameters.get("joint_target"),
                "velocity_scale": parameters.get("velocity_scale"),
                "acceleration_scale": parameters.get("acceleration_scale"),
            }
        if primitive in {"PTP", "LIN", "CIRC", "CARTESIAN_PATH"}:
            command = {"primitive_type": primitive}
            command.update(parameters)
            return command

        raise IntentResolutionError(f"unsupported structured action: {payload.get('action')!r}")

    def _sequence_candidate_metadata(self, payload: dict[str, Any]) -> dict[str, Any]:
        intent = str(payload.get("intent") or "").strip().lower()
        metadata: dict[str, Any] = {"intent": intent}
        if intent == "draw_shape":
            metadata["macro_name"] = "draw_shape"
            metadata["shape_type"] = str(payload.get("shape_type", payload.get("shape", ""))).strip().lower()
        if intent == "draw_text":
            metadata["macro_name"] = "draw_text"
            metadata["text"] = str(payload.get("text") or "").strip().upper()
        return metadata

    def _hydrate_draw_workplane(
        self,
        payload: dict[str, Any],
        *,
        current_pose_loader: Callable[[], dict[str, Any] | None] | None,
    ) -> dict[str, Any]:
        if not isinstance(payload, dict):
            return payload
        intent = str(payload.get("intent", "")).strip().lower()
        if intent not in {"draw_shape", "draw_text"}:
            return payload

        working_payload = dict(payload)
        workplane = working_payload.get("workplane")
        if workplane is None:
            workplane = {"mode": "tool"}
            working_payload["workplane"] = workplane
        if not isinstance(workplane, dict):
            return working_payload

        mode = str(workplane.get("mode", "base")).strip().lower()
        if mode != "tool":
            return working_payload
        if isinstance(workplane.get("origin"), dict):
            return working_payload
        if isinstance(working_payload.get("start_pose"), dict):
            return working_payload

        current_pose = current_pose_loader() if callable(current_pose_loader) else None
        if current_pose is None:
            raise IntentResolutionError(
                "missing_workplane: tool mode requires current pose, but /get_current_pose is unavailable"
            )

        hydrated_workplane = dict(workplane)
        hydrated_workplane["origin"] = current_pose
        working_payload["workplane"] = hydrated_workplane
        return working_payload

    def _parse_draw_text_to_semantic(self, raw_text: str) -> dict[str, Any] | None:
        stripped_text = " ".join(raw_text.strip().split())
        if not stripped_text:
            return None
        folded = self._fold_text(stripped_text)

        if folded.startswith("draw circle") or folded.startswith("ve hinh tron"):
            payload = {
                "intent": "draw_shape",
                "shape_type": "circle",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "params": {},
            }
            match = re.fullmatch(
                r"(?:draw\s+circle|ve\s+hinh\s+tron)(?:\s+(?:radius|ban\s+kinh))?\s+([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m)",
                folded,
            )
            if match:
                payload["units"] = match.group(2)
                payload["params"] = {"radius": float(match.group(1))}
            return payload

        if folded.startswith("draw rectangle") or folded.startswith("ve hinh chu nhat"):
            payload = {
                "intent": "draw_shape",
                "shape_type": "rectangle",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "params": {},
            }
            match = re.fullmatch(
                r"(?:draw\s+rectangle|ve\s+hinh\s+chu\s+nhat)\s+([+-]?\d+(?:\.\d+)?)\s*(?:x|by)\s*([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m)",
                folded,
            )
            if match:
                payload["units"] = match.group(3)
                payload["params"] = {
                    "width": float(match.group(1)),
                    "height": float(match.group(2)),
                }
            return payload

        if folded.startswith("draw polygon") or folded.startswith("ve da giac"):
            payload = {
                "intent": "draw_shape",
                "shape_type": "polygon",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "params": {},
            }
            match = re.fullmatch(
                (
                    r"(?:draw\s+polygon|ve\s+da\s+giac)\s+(\d+)\s*(?:sides?|canh)"
                    r"(?:\s+(?:radius|ban\s+kinh)\s+([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m)"
                    r"|\s+(?:side|canh)\s+([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m))"
                ),
                folded,
            )
            if match:
                payload["params"] = {"n_sides": int(match.group(1))}
                if match.group(2) is not None:
                    payload["units"] = match.group(3)
                    payload["params"]["radius"] = float(match.group(2))
                elif match.group(4) is not None:
                    payload["units"] = match.group(5)
                    payload["params"]["side"] = float(match.group(4))
            return payload

        prefix_match = _DRAW_TEXT_PREFIX_PATTERN.fullmatch(stripped_text)
        if prefix_match is None:
            return None

        content = prefix_match.group(1).strip()
        if not content:
            return None

        height_match = re.fullmatch(r"(.+?)\s+([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m)\s+tall", content, re.IGNORECASE)
        if height_match is None:
            height_match = re.fullmatch(r"(.+?)\s+cao\s+([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m)", content, re.IGNORECASE)

        if height_match is not None:
            text = height_match.group(1).strip()
            height = float(height_match.group(2))
            units = str(height_match.group(3)).lower()
        else:
            text = content
            height = DEFAULT_DRAW_TEXT_HEIGHT
            units = DEFAULT_DRAW_TEXT_UNITS

        return {
            "intent": "draw_text",
            "text": text,
            "units": units,
            "frame_id": "base_link",
            "workplane": {"mode": "tool"},
            "font": {
                "type": "single_stroke_builtin",
                "height": height,
            },
        }

    def _fold_text(self, value: str) -> str:
        normalized = unicodedata.normalize("NFKD", value)
        return "".join(character for character in normalized if not unicodedata.combining(character)).lower()

    def _text_to_command(self, *, raw_text: str, current_joints: list[Any]) -> dict[str, Any]:
        normalized = " ".join(raw_text.strip().split()).lower()
        if not normalized:
            raise IntentResolutionError("empty command text is not allowed", missing_slots=["intentText"])

        if normalized.startswith("{") and parse_llm_output is not None:
            try:
                parsed_payload = parse_llm_output(raw_text)
            except Exception as exc:  # pragma: no cover - parser is optional
                raise IntentResolutionError(f"failed to parse JSON intent payload: {exc}") from exc
            if not isinstance(parsed_payload, dict):
                raise IntentResolutionError("JSON intent payload must decode to an object.")
            return self._structured_to_command(
                structured_intent=parsed_payload,
                runtime_mode="hardware",
                current_joints=current_joints,
            )

        if normalized in {"home", "go home", "move home", "return home"}:
            return {"primitive_type": "HOME"}

        if normalized in {"stop", "stop motion", "cancel motion", "halt"}:
            return {"primitive_type": "STOP"}

        if normalized in {"alarm reset", "reset alarm", "clear alarm"}:
            return {"primitive_type": "ALARM_RESET"}

        if normalized in {"get pose", "current pose", "where is robot", "where is tcp"}:
            return {"primitive_type": "GET_POSE", "reference_frame": "base_link"}

        wait_match = re.fullmatch(r"(?:wait|pause)\s+([+-]?\d+(?:\.\d+)?)\s*(?:s|sec|second|seconds)?", normalized)
        if wait_match:
            return {
                "primitive_type": "WAIT",
                "wait_duration_sec": float(wait_match.group(1)),
            }

        speed_match = re.fullmatch(
            r"(?:set\s+speed|speed)\s+([+-]?\d+(?:\.\d+)?)\s*(%|pct|percent)?",
            normalized,
        )
        if speed_match:
            speed_value = float(speed_match.group(1))
            if speed_match.group(2) is not None or speed_value > 1.0:
                speed_value /= 100.0
            return {"primitive_type": "SET_SPEED", "velocity_scale": speed_value}

        cartesian_match = re.fullmatch(
            r"move\s+(up|down|left|right|forward|back|backward)\s+([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m)",
            normalized,
        )
        if cartesian_match:
            direction, magnitude_text, unit = cartesian_match.groups()
            axis_scale = _CARTESIAN_DIRECTIONS[direction]
            magnitude_m = float(magnitude_text) * UNIT_TO_METERS[unit]
            return {
                "primitive_type": "MOVE_REL",
                "delta_x": axis_scale[0] * magnitude_m,
                "delta_y": axis_scale[1] * magnitude_m,
                "delta_z": axis_scale[2] * magnitude_m,
                "reference_frame": "base_link",
            }

        joint_match = re.fullmatch(
            r"(?:move\s+)?(?:joint|j)\s*([1-6])\s+([+-]?\d+(?:\.\d+)?)\s*(deg|degree|degrees)",
            normalized,
        )
        if joint_match:
            joint_one_based = int(joint_match.group(1))
            angle_text = joint_match.group(2)
            target_deg = float(angle_text)
            joint_index = joint_one_based - 1
            if angle_text.startswith(("+", "-")):
                current_deg = self._read_joint_deg(current_joints=current_joints, joint_index=joint_index)
                if current_deg is None:
                    raise IntentResolutionError(
                        f"fresh joint position for {JOINT_NAMES[joint_index]} is unavailable.",
                        missing_slots=["fresh_joint_position"],
                    )
                target_deg = current_deg + target_deg
            return {
                "primitive_type": "MOVE_JOINT",
                "joint_index": joint_index,
                "joint_angle": math.radians(target_deg),
            }

        joints_match = re.fullmatch(r"move\s+joints?\s+(.+)", normalized)
        if joints_match:
            numbers = re.findall(r"[+-]?\d+(?:\.\d+)?", joints_match.group(1))
            if len(numbers) != 6:
                raise IntentResolutionError(
                    "MOVE_JOINTS text command requires exactly 6 joint values.",
                    missing_slots=["joint_target[0..5]"],
                )
            return {
                "primitive_type": "MOVE_JOINTS",
                "joint_target": [float(value) for value in numbers],
            }

        raise IntentResolutionError(
            "intent is ambiguous or unsupported. Supported phrases: home, stop, wait, set speed, "
            "move <direction> <distance>, move joint <n> <deg>, move joints <6 values>.",
        )

    def _normalize_command(
        self,
        *,
        command: dict[str, Any],
        runtime_mode: str,
        allow_routed_draw_metadata: bool = False,
    ) -> tuple[dict[str, Any], list[str]]:
        if allow_routed_draw_metadata:
            command = {
                key: value
                for key, value in command.items()
                if key not in ROUTED_DRAW_METADATA_FIELDS
            }
        primitive = str(command.get("primitive_type") or "").strip().upper()
        if not primitive:
            raise IntentResolutionError("missing primitive_type", missing_slots=["primitive_type"])
        if primitive not in SUPPORTED_PRIMITIVES:
            raise IntentResolutionError(
                f"primitive_type {primitive!r} is outside the supported ExecuteMotion policy.",
                rejected_fields=["primitive_type"],
            )
        if runtime_mode == "hardware" and primitive not in HARDWARE_WHITELIST:
            raise IntentResolutionError(
                f"primitive_type {primitive!r} is blocked by the hardware primitive whitelist.",
                rejected_fields=["primitive_type"],
            )

        allowed_fields = _ALLOWED_FIELDS_BY_PRIMITIVE[primitive]
        rejected_fields = sorted(key for key in command if key not in allowed_fields and key != "primitive_type")
        if rejected_fields:
            raise IntentResolutionError(
                f"{primitive} payload contains unsupported fields.",
                rejected_fields=rejected_fields,
            )

        normalized: dict[str, Any] = {"primitive_type": primitive}
        notes: list[str] = []

        if primitive in MOTION_PRIMITIVES:
            velocity_scale = _to_float(
                command.get("velocity_scale", self._default_velocity_scale),
                "velocity_scale",
            )
            velocity_clamped = _clamp(velocity_scale, self._min_velocity_scale, self._max_velocity_scale)
            if velocity_clamped != velocity_scale:
                notes.append(
                    f"velocity_scale clamped from {velocity_scale:.4f} to {velocity_clamped:.4f}."
                )
            normalized["velocity_scale"] = velocity_clamped

            acceleration_scale = _to_float(
                command.get("acceleration_scale", self._default_acceleration_scale),
                "acceleration_scale",
            )
            acceleration_clamped = _clamp(acceleration_scale, self._min_velocity_scale, self._max_acceleration_scale)
            if acceleration_clamped != acceleration_scale:
                notes.append(
                    f"acceleration_scale clamped from {acceleration_scale:.4f} to {acceleration_clamped:.4f}."
                )
            normalized["acceleration_scale"] = acceleration_clamped
            normalized["planner_id"] = str(command.get("planner_id") or PLANNER_DEFAULTS.get(primitive, "PILZ_PTP"))
            normalized["require_approval"] = bool(command.get("require_approval", False))

        if primitive in {"HOME", "PTP", "LIN", "CIRC", "CARTESIAN_PATH", "MOVE_REL", "GET_POSE", "WAIT", "STOP", "IO_SET", "ALARM_RESET"}:
            normalized["reference_frame"] = str(command.get("reference_frame") or "base_link")
            if normalized["reference_frame"] != "base_link":
                raise IntentResolutionError(
                    f"{primitive} requires reference_frame='base_link'.",
                    rejected_fields=["reference_frame"],
                )

        if primitive in {"PTP", "LIN", "CIRC"}:
            if "target_pose" not in command:
                raise IntentResolutionError(
                    f"{primitive} requires target_pose.",
                    missing_slots=["target_pose"],
                )
            normalized["target_pose"] = self._normalize_pose(command["target_pose"], field_name="target_pose")

        if primitive == "PTP" and "joint_target" in command:
            normalized["joint_target"] = self._normalize_joint_target(command["joint_target"])
        if primitive == "PTP" and "target_pose" not in normalized and "joint_target" not in normalized:
            raise IntentResolutionError(
                "PTP requires target_pose or joint_target.",
                missing_slots=["target_pose|joint_target"],
            )

        if primitive == "CIRC":
            if "waypoints" not in command:
                raise IntentResolutionError("CIRC requires waypoints.", missing_slots=["waypoints[0]"])
            waypoints = command.get("waypoints")
            if not isinstance(waypoints, list) or len(waypoints) != 1:
                raise IntentResolutionError("CIRC requires exactly one auxiliary waypoint.", missing_slots=["waypoints[0]"])
            normalized["waypoints"] = [
                self._normalize_pose(waypoints[0], field_name="waypoints[0]")
            ]

        if primitive == "CARTESIAN_PATH":
            waypoints = command.get("waypoints")
            if not isinstance(waypoints, list) or not waypoints:
                raise IntentResolutionError(
                    "CARTESIAN_PATH requires non-empty waypoints.",
                    missing_slots=["waypoints[0..n]"],
                )
            normalized["waypoints"] = [
                self._normalize_pose(waypoint, field_name=f"waypoints[{index}]")
                for index, waypoint in enumerate(waypoints)
            ]

        if primitive == "MOVE_REL":
            missing_delta = [
                field
                for field in ("delta_x", "delta_y", "delta_z")
                if field not in command
            ]
            if missing_delta:
                raise IntentResolutionError(
                    "MOVE_REL requires delta_x, delta_y, and delta_z.",
                    missing_slots=missing_delta,
                )
            normalized["delta_x"] = _to_float(command.get("delta_x"), "delta_x")
            normalized["delta_y"] = _to_float(command.get("delta_y"), "delta_y")
            normalized["delta_z"] = _to_float(command.get("delta_z"), "delta_z")
            if (
                normalized["delta_x"] == 0.0
                and normalized["delta_y"] == 0.0
                and normalized["delta_z"] == 0.0
            ):
                raise IntentResolutionError("MOVE_REL requires at least one non-zero delta component.")

        if primitive == "MOVE_JOINT":
            if "joint_index" not in command:
                raise IntentResolutionError("MOVE_JOINT requires joint_index.", missing_slots=["joint_index"])
            if "joint_angle" not in command:
                raise IntentResolutionError("MOVE_JOINT requires joint_angle.", missing_slots=["joint_angle"])
            joint_index = _to_int(command.get("joint_index"), "joint_index")
            if joint_index < 0 or joint_index >= len(JOINT_NAMES):
                raise IntentResolutionError("MOVE_JOINT joint_index must be between 0 and 5.", rejected_fields=["joint_index"])
            normalized["joint_index"] = joint_index
            normalized["joint_angle"] = _normalize_angle(command.get("joint_angle"), "joint_angle")

        if primitive == "MOVE_JOINTS":
            if "joint_target" not in command:
                raise IntentResolutionError("MOVE_JOINTS requires joint_target.", missing_slots=["joint_target"])
            normalized["joint_target"] = self._normalize_joint_target(command.get("joint_target"))

        if primitive == "WAIT":
            if "wait_duration_sec" not in command:
                raise IntentResolutionError("WAIT requires wait_duration_sec.", missing_slots=["wait_duration_sec"])
            wait_duration_sec = _to_float(command.get("wait_duration_sec"), "wait_duration_sec")
            if wait_duration_sec < 0.0:
                raise IntentResolutionError("WAIT wait_duration_sec must be >= 0.", rejected_fields=["wait_duration_sec"])
            if wait_duration_sec > 60.0:
                raise IntentResolutionError("WAIT wait_duration_sec must be <= 60.", rejected_fields=["wait_duration_sec"])
            normalized["wait_duration_sec"] = wait_duration_sec

        if primitive == "SET_SPEED":
            if "velocity_scale" not in command:
                raise IntentResolutionError("SET_SPEED requires velocity_scale.", missing_slots=["velocity_scale"])
            speed_raw = _to_float(command.get("velocity_scale"), "velocity_scale")
            speed_clamped = _clamp(speed_raw, self._min_velocity_scale, self._max_velocity_scale)
            if speed_clamped != speed_raw:
                notes.append(f"SET_SPEED velocity_scale clamped from {speed_raw:.4f} to {speed_clamped:.4f}.")
            normalized["velocity_scale"] = speed_clamped

        if primitive == "IO_SET":
            missing_io = [field for field in ("io_address", "io_value") if field not in command]
            if missing_io:
                raise IntentResolutionError("IO_SET requires io_address and io_value.", missing_slots=missing_io)
            io_address = _to_int(command.get("io_address"), "io_address")
            io_value = _to_int(command.get("io_value"), "io_value")
            if io_address < 0:
                raise IntentResolutionError("IO_SET io_address must be >= 0.", rejected_fields=["io_address"])
            if io_value not in {0, 1}:
                raise IntentResolutionError("IO_SET io_value must be 0 or 1.", rejected_fields=["io_value"])
            normalized["io_address"] = io_address
            normalized["io_value"] = io_value

        return normalized, notes

    def _normalize_joint_target(self, value: Any) -> list[float]:
        if not isinstance(value, list):
            raise IntentResolutionError("joint_target must be an array of 6 values.", missing_slots=["joint_target"])
        if len(value) != len(JOINT_NAMES):
            raise IntentResolutionError(
                "joint_target must include exactly 6 values.",
                missing_slots=["joint_target[0..5]"],
            )
        return [_normalize_angle(entry, f"joint_target[{index}]") for index, entry in enumerate(value)]

    def _normalize_pose(self, value: Any, *, field_name: str) -> dict[str, Any]:
        if not isinstance(value, dict):
            raise IntentResolutionError(f"{field_name} must be an object.")
        position = value.get("position")
        if not isinstance(position, dict):
            raise IntentResolutionError(f"{field_name}.position must be an object.", missing_slots=[f"{field_name}.position"])

        x = _to_float(position.get("x"), f"{field_name}.position.x")
        y = _to_float(position.get("y"), f"{field_name}.position.y")
        z = _to_float(position.get("z"), f"{field_name}.position.z")
        if any(abs(component) > 10.0 for component in (x, y, z)):
            x /= 1000.0
            y /= 1000.0
            z /= 1000.0

        orientation_payload = value.get("orientation")
        if orientation_payload is None:
            orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
        elif not isinstance(orientation_payload, dict):
            raise IntentResolutionError(f"{field_name}.orientation must be an object.")
        elif {"x", "y", "z", "w"}.issubset(set(orientation_payload.keys())):
            orientation = {
                "x": _to_float(orientation_payload.get("x"), f"{field_name}.orientation.x"),
                "y": _to_float(orientation_payload.get("y"), f"{field_name}.orientation.y"),
                "z": _to_float(orientation_payload.get("z"), f"{field_name}.orientation.z"),
                "w": _to_float(orientation_payload.get("w"), f"{field_name}.orientation.w"),
            }
        elif {"roll", "pitch", "yaw"}.issubset(set(orientation_payload.keys())):
            roll = _to_float(orientation_payload.get("roll"), f"{field_name}.orientation.roll")
            pitch = _to_float(orientation_payload.get("pitch"), f"{field_name}.orientation.pitch")
            yaw = _to_float(orientation_payload.get("yaw"), f"{field_name}.orientation.yaw")
            if any(abs(component) > (2.0 * math.pi) for component in (roll, pitch, yaw)):
                roll = math.radians(roll)
                pitch = math.radians(pitch)
                yaw = math.radians(yaw)
            orientation = _rpy_to_quaternion(roll, pitch, yaw)
        else:
            raise IntentResolutionError(
                f"{field_name}.orientation must provide quaternion (x,y,z,w) or RPY (roll,pitch,yaw)."
            )

        return {
            "position": {"x": x, "y": y, "z": z},
            "orientation": orientation,
        }

    def _resolve_joint_index(self, parameters: dict[str, Any]) -> int | None:
        raw_zero_based = parameters.get("jointIndexZeroBased")
        if raw_zero_based is not None:
            index = _to_int(raw_zero_based, "jointIndexZeroBased")
            if 0 <= index < len(JOINT_NAMES):
                return index

        raw_joint_index = parameters.get("jointIndex")
        if raw_joint_index is not None:
            index = _to_int(raw_joint_index, "jointIndex")
            if 0 <= index < len(JOINT_NAMES):
                return index
            if 1 <= index <= len(JOINT_NAMES):
                return index - 1

        raw_joint_name = str(parameters.get("jointNameResolved") or parameters.get("joint") or "").strip().lower()
        if raw_joint_name:
            if raw_joint_name in JOINT_NAMES:
                return JOINT_NAMES.index(raw_joint_name)
            match = re.fullmatch(r"joint[_\s-]*([1-6])(?:[_\s-].+)?", raw_joint_name)
            if match:
                return int(match.group(1)) - 1

        return None

    def _read_joint_deg(self, *, current_joints: list[Any], joint_index: int) -> float | None:
        target_name = JOINT_NAMES[joint_index]
        for joint in current_joints:
            if getattr(joint, "name", None) == target_name:
                position_deg = getattr(joint, "position_deg", None)
                if position_deg is None:
                    return None
                return float(position_deg)
        return None

    def _target_summary(self, normalized_command: dict[str, Any]) -> str:
        primitive = normalized_command["primitive_type"]
        if primitive == "HOME":
            return "Return robot to configured home pose."
        if primitive == "STOP":
            return "Request supervised stop handling."
        if primitive == "MOVE_REL":
            return (
                "Relative translation in base_link: "
                f"dx={normalized_command.get('delta_x', 0.0):.4f} "
                f"dy={normalized_command.get('delta_y', 0.0):.4f} "
                f"dz={normalized_command.get('delta_z', 0.0):.4f} m."
            )
        if primitive == "MOVE_JOINT":
            return (
                "Move single joint target: "
                f"joint_index={normalized_command.get('joint_index')} "
                f"joint_angle={normalized_command.get('joint_angle'):.4f} rad."
            )
        if primitive == "MOVE_JOINTS":
            return "Move all six joints to absolute targets."
        if primitive == "WAIT":
            return f"Pause execution for {normalized_command.get('wait_duration_sec', 0.0):.2f} s."
        if primitive == "SET_SPEED":
            return f"Set default velocity scale to {normalized_command.get('velocity_scale', 0.0):.4f}."
        if primitive == "IO_SET":
            return (
                f"Set controller IO address {normalized_command.get('io_address')} "
                f"to {normalized_command.get('io_value')}."
            )
        if primitive == "ALARM_RESET":
            return "Request alarm reset at execution boundary."
        if primitive == "GET_POSE":
            return "Query current robot TCP pose."
        if primitive == "LIN":
            return "Linear motion to target pose."
        if primitive == "PTP":
            return "Point-to-point motion to target pose or joint target."
        if primitive == "CIRC":
            return "Circular motion using auxiliary waypoint."
        if primitive == "CARTESIAN_PATH":
            return "Cartesian multi-waypoint path execution."
        return f"Execute primitive {primitive}."
