from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Callable

from ..domain.constants import GP4_JOINT_NAMES as JOINT_NAMES
from .intent_constants import (
    _OLD_ACTIONS,
)
from .intent_normalization import (
    IntentNormalizationMixin,
    _to_float,
)

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
    from llm_gateway.intent_engine import IntentRouter
    from llm_gateway.intent_engine import prepare_semantic_ir_for_routing
except Exception:  # pragma: no cover - depends on optional source path
    IntentRouter = None  # type: ignore[assignment]
    prepare_semantic_ir_for_routing = None  # type: ignore[assignment]

try:  # pragma: no cover - fallback logic is covered
    from llm_gateway.intent_engine import (
        SequenceValidationError,
        SequenceValidator,
    )
except Exception:  # pragma: no cover - depends on optional source path
    SequenceValidationError = ValueError  # type: ignore[assignment]
    SequenceValidator = None  # type: ignore[assignment]


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


class IntentResolutionService(IntentNormalizationMixin):
    def __init__(
        self,
        *,
        default_velocity_scale: float = 0.06,
        default_acceleration_scale: float = 0.06,
        min_velocity_scale: float = 0.01,
        max_velocity_scale: float = 0.06,
        max_acceleration_scale: float = 0.06,
        ros_adapter: Any | None = None,
    ) -> None:
        self._default_velocity_scale = float(default_velocity_scale)
        self._default_acceleration_scale = float(default_acceleration_scale)
        self._min_velocity_scale = float(min_velocity_scale)
        self._max_velocity_scale = float(max_velocity_scale)
        self._max_acceleration_scale = float(max_acceleration_scale)
        self._ros_adapter = ros_adapter

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

        if candidate_payload is None:
            return None

        if intent_name == "factory_task_runtime":
            return self._prepare_factory_task_runtime_sequence_submission(
                candidate_payload,
                runtime_mode=runtime_mode,
                current_joints=current_joints,
                current_pose_loader=current_pose_loader,
            )

        route_metadata = self._sequence_candidate_metadata(candidate_payload)
        diagnostics: list[str] = []
        working_payload = dict(candidate_payload)
        try:
            working_payload = self._inject_return_to_start_joints(
                working_payload, current_joints
            )
            working_payload = self._prepare_semantic_ir_for_routing(
                working_payload, current_joints
            )
        except IntentResolutionError as exc:
            return {
                "parsed_steps": None,
                "diagnostics": diagnostics,
                "parse_error": exc.operator_message(),
                "route_metadata": route_metadata,
                "structured_intent": working_payload,
            }
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
                        "semantic intent routing is unavailable because llm_gateway.intent_engine is not importable."
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
                    allow_routed_draw_metadata=bool(
                        routed.metadata.get("macro_name") in {"draw_shape", "draw_text"}
                    ),
                    allow_primitive_structured=True,
                )
                for command in routed.commands
            ]
        except (IntentResolutionError, SequenceValidationError, ValueError) as exc:
            message = (
                exc.operator_message()
                if isinstance(exc, IntentResolutionError)
                else str(exc)
            )
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

    def _inject_return_to_start_joints(
        self, payload: dict[str, Any], current_joints: list[Any]
    ) -> dict[str, Any]:
        if not self._semantic_ir_contains_intent(payload, "return_to_start"):
            return dict(payload)
        joint_target = self._current_joint_target_rad(current_joints)
        return self._inject_return_to_start_joints_recursive(payload, joint_target)

    def _semantic_ir_contains_intent(self, payload: Any, target_intent: str) -> bool:
        if isinstance(payload, dict):
            if str(payload.get("intent") or "").strip().lower() == target_intent:
                return True
            return any(
                self._semantic_ir_contains_intent(value, target_intent)
                for value in payload.values()
            )
        if isinstance(payload, list):
            return any(
                self._semantic_ir_contains_intent(value, target_intent)
                for value in payload
            )
        return False

    def _current_joint_target_rad(self, current_joints: list[Any]) -> list[float]:
        if len(current_joints) != len(JOINT_NAMES):
            raise IntentResolutionError(
                "return_to_start requires a complete current joint snapshot.",
                missing_slots=["joint_positions"],
            )
        joint_target: list[float] = []
        for index, joint in enumerate(current_joints):
            position_deg = getattr(joint, "position_deg", None)
            if position_deg is None:
                raise IntentResolutionError(
                    "return_to_start requires a complete current joint snapshot.",
                    missing_slots=[f"joint_positions[{index}]"],
                )
            joint_target.append(math.radians(float(position_deg)))
        return joint_target

    def _prepare_semantic_ir_for_routing(
        self, payload: dict[str, Any], current_joints: list[Any]
    ) -> dict[str, Any]:
        if not self._semantic_ir_contains_intent(payload, "move_joint_delta"):
            return dict(payload)
        if prepare_semantic_ir_for_routing is None:
            raise IntentResolutionError(
                "semantic intent preparation is unavailable because llm_gateway.intent_engine is not importable."
            )
        try:
            if len(current_joints) != len(JOINT_NAMES):
                raise IntentResolutionError(
                    "move_joint_delta requires a complete current joint snapshot.",
                    missing_slots=["joint_positions"],
                )
            joint_positions_rad = []
            for index, joint in enumerate(current_joints):
                position_deg = getattr(joint, "position_deg", None)
                if position_deg is None:
                    raise IntentResolutionError(
                        "move_joint_delta requires a complete current joint snapshot.",
                        missing_slots=[f"joint_positions[{index}]"],
                    )
                joint_positions_rad.append(math.radians(float(position_deg)))
            return prepare_semantic_ir_for_routing(
                payload,
                current_joint_positions_rad=joint_positions_rad,
            )
        except IntentResolutionError:
            raise
        except ValueError as exc:
            raise IntentResolutionError(str(exc)) from exc

    def _inject_return_to_start_joints_recursive(
        self, payload: dict[str, Any], joint_target: list[float]
    ) -> dict[str, Any]:
        enriched = dict(payload)
        intent_name = str(enriched.get("intent") or "").strip().lower()
        if intent_name == "return_to_start" and "joint_target" not in enriched:
            enriched["joint_target"] = list(joint_target)
            return enriched
        if intent_name == "sequence":
            steps = enriched.get("steps")
            if isinstance(steps, list):
                enriched["steps"] = [
                    self._inject_return_to_start_joints_recursive(step, joint_target)
                    if isinstance(step, dict)
                    else step
                    for step in steps
                ]
        return enriched

    def resolve(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        runtime_mode: str,
        current_joints: list[Any],
        allow_routed_draw_metadata: bool = False,
        allow_primitive_structured: bool = False,
    ) -> dict[str, Any]:
        mode = str(runtime_mode or "unknown").strip().lower() or "unknown"
        if structured_intent is not None:
            initial_command = self._structured_to_command(
                structured_intent=structured_intent,
                runtime_mode=mode,
                current_joints=current_joints,
                allow_primitive_structured=allow_primitive_structured,
            )
            normalized_text = json.dumps(
                structured_intent, separators=(",", ":"), ensure_ascii=True
            )
            source = "structured"
        else:
            initial_command = self._text_to_command(
                raw_text=raw_text, current_joints=current_joints
            )
            normalized_text = " ".join(raw_text.strip().split()).lower()
            source = "text"
            # If local text parser produced semantic IR (intent), route it like structured_intent
            if "intent" in initial_command:
                initial_command = self._structured_to_command(
                    structured_intent=initial_command,
                    runtime_mode=mode,
                    current_joints=current_joints,
                    allow_primitive_structured=allow_primitive_structured,
                )

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
        allow_primitive_structured: bool = False,
    ) -> dict[str, Any]:
        payload = dict(structured_intent)

        if "primitive_type" in payload:
            if allow_primitive_structured:
                return payload
            raise IntentResolutionError(
                "structuredIntent primitive_type payloads are not accepted from the HMI API; "
                "submit natural-language text or Semantic IR with an intent field.",
                rejected_fields=["primitive_type"],
            )

        if str(payload.get("intent") or "").strip().lower() == "factory_task_runtime":
            payload = self._factory_task_runtime_single_semantic_ir(payload)

        if "intent" in payload:
            if IntentRouter is None:
                raise IntentResolutionError(
                    "semantic intent routing is unavailable because llm_gateway.intent_engine is not importable."
                )
            payload = self._prepare_semantic_ir_for_routing(payload, current_joints)
            routed = IntentRouter(runtime_mode=runtime_mode).route(payload)
            if routed.route_type == "error":
                message = (routed.error_payload or {}).get("message") or (
                    routed.error_payload or {}
                ).get("error")
                raise IntentResolutionError(
                    str(message or "intent router returned an error payload.")
                )
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

    def _prepare_factory_task_runtime_sequence_submission(
        self,
        payload: dict[str, Any],
        *,
        runtime_mode: str,
        current_joints: list[Any],
        current_pose_loader: Callable[[], dict[str, Any] | None] | None,
    ) -> dict[str, Any] | None:
        working_payload = deepcopy(payload)
        runtime_plan = self._factory_task_runtime_plan(working_payload)
        if not isinstance(runtime_plan, dict):
            return None

        route_metadata = self._sequence_candidate_metadata(working_payload)
        diagnostics: list[str] = []
        try:
            semantic_steps = self._runtime_plan_to_semantic_steps(
                runtime_plan,
                current_pose_loader=current_pose_loader,
            )
            if not semantic_steps:
                raise IntentResolutionError(
                    "FactoryTask runtime sequence has no executable steps."
                )
            is_runtime_sequence = (
                str(runtime_plan.get("type") or "").strip().lower() == "sequence"
            )
            has_draw_step = any(
                str(step.get("intent") or "").strip().lower()
                in {"draw_shape", "draw_text"}
                for step in semantic_steps
            )
            if not is_runtime_sequence and not has_draw_step:
                return None

            parsed_steps: list[dict[str, Any]] = []
            for step in semantic_steps:
                step_intent = str(step.get("intent") or "").strip().lower()
                if step_intent in {"draw_shape", "draw_text"}:
                    if IntentRouter is None:
                        raise IntentResolutionError(
                            "semantic intent routing is unavailable because llm_gateway.intent_engine is not importable."
                        )
                    routed = IntentRouter(runtime_mode=runtime_mode).route(step)
                    if routed.route_type == "error":
                        message = (routed.error_payload or {}).get("message") or (
                            routed.error_payload or {}
                        ).get("error")
                        raise IntentResolutionError(
                            str(message or "intent router returned an error payload.")
                        )
                    if len(semantic_steps) == 1:
                        route_metadata.update(dict(routed.metadata))
                    if routed.metadata.get("macro_name") in {"draw_shape", "draw_text"}:
                        for command in routed.commands:
                            if bool(command.get("plan_only")):
                                raise IntentResolutionError(
                                    "draw execution_mode=plan_only is not supported by the HMI; "
                                    "resubmit with execution_mode='execute'."
                                )
                    parsed_steps.extend(
                        self.resolve(
                            raw_text="",
                            structured_intent=command,
                            runtime_mode=runtime_mode,
                            current_joints=current_joints,
                            allow_routed_draw_metadata=True,
                            allow_primitive_structured=True,
                        )
                        for command in routed.commands
                    )
                    continue
                parsed_steps.append(
                    self.resolve(
                        raw_text="",
                        structured_intent=step,
                        runtime_mode=runtime_mode,
                        current_joints=current_joints,
                    )
                )
            if SequenceValidator is not None:
                validation = SequenceValidator().validate(
                    [step["normalizedCommand"] for step in parsed_steps]
                )
                diagnostics.extend(validation.diagnostics)
        except (IntentResolutionError, SequenceValidationError, ValueError) as exc:
            message = (
                exc.operator_message()
                if isinstance(exc, IntentResolutionError)
                else str(exc)
            )
            return {
                "parsed_steps": None,
                "diagnostics": diagnostics,
                "parse_error": message,
                "route_metadata": route_metadata,
                "structured_intent": payload,
            }

        return {
            "parsed_steps": parsed_steps,
            "diagnostics": diagnostics,
            "parse_error": None,
            "route_metadata": route_metadata,
            "structured_intent": working_payload,
        }

    def _factory_task_runtime_single_semantic_ir(
        self, payload: dict[str, Any]
    ) -> dict[str, Any]:
        runtime_plan = self._factory_task_runtime_plan(payload)
        if not isinstance(runtime_plan, dict):
            raise IntentResolutionError(
                "FactoryTask runtime payload is missing metadata.runtime_plan."
            )
        steps = self._runtime_plan_to_semantic_steps(runtime_plan)
        if len(steps) != 1:
            raise IntentResolutionError(
                "FactoryTask runtime plan must be a single executable skill for command submission."
            )
        return steps[0]

    @staticmethod
    def _factory_task_runtime_plan(payload: dict[str, Any]) -> dict[str, Any] | None:
        if payload.get("_factory_task_runtime") is not True:
            return None
        metadata = payload.get("metadata")
        if not isinstance(metadata, dict):
            return None
        runtime_plan = metadata.get("runtime_plan")
        return runtime_plan if isinstance(runtime_plan, dict) else None

    def _runtime_plan_to_semantic_steps(
        self,
        runtime_plan: dict[str, Any],
        *,
        current_pose_loader: Callable[[], dict[str, Any] | None] | None = None,
    ) -> list[dict[str, Any]]:
        node_type = str(runtime_plan.get("type") or "").strip().lower()
        if node_type == "sequence":
            children = runtime_plan.get("children")
            if not isinstance(children, list):
                raise IntentResolutionError(
                    "FactoryTask runtime sequence requires children."
                )
            steps: list[dict[str, Any]] = []
            for child in children:
                if not isinstance(child, dict):
                    raise IntentResolutionError(
                        "FactoryTask runtime sequence child must be an object."
                    )
                steps.extend(
                    self._runtime_plan_to_semantic_steps(
                        child,
                        current_pose_loader=current_pose_loader,
                    )
                )
            return steps
        if node_type == "skill":
            semantic_ir = self._runtime_skill_to_semantic_ir(runtime_plan)
            intent = str(semantic_ir.get("intent") or "").strip().lower()
            if intent in {"draw_shape", "draw_text"}:
                semantic_ir = self._hydrate_draw_workplane(
                    semantic_ir,
                    current_pose_loader=current_pose_loader,
                )
                args = (
                    runtime_plan.get("args")
                    if isinstance(runtime_plan.get("args"), dict)
                    else {}
                )
                runtime_plan["args"] = {
                    **dict(args),
                    **{
                        key: value
                        for key, value in semantic_ir.items()
                        if key != "intent"
                    },
                }
            return [semantic_ir]
        raise IntentResolutionError(
            f"FactoryTask runtime node '{node_type or '<empty>'}' is not supported by the HMI Phase 1 adapter."
        )

    def _runtime_skill_to_semantic_ir(self, node: dict[str, Any]) -> dict[str, Any]:
        name = str(node.get("name") or "").strip().lower()
        args = node.get("args") if isinstance(node.get("args"), dict) else {}
        args = dict(args)
        if name == "go_home":
            return {"intent": "go_home"}
        if name in {"stop", "alarm_reset"}:
            return {"intent": name}
        if name == "get_pose":
            return {
                "intent": "get_pose",
                "reference_frame": str(
                    args.get("reference_frame") or args.get("frame_id") or "base_link"
                ),
            }
        if name == "wait":
            return {
                "intent": "wait",
                "wait_duration_sec": float(args.get("wait_duration_sec", 2.0)),
            }
        if name == "set_speed":
            return self._runtime_skill_payload(
                "set_speed",
                args,
                required=("velocity_scale",),
                optional=("acceleration_scale",),
            )
        if name == "move_relative":
            payload = self._runtime_skill_payload(
                "move_relative",
                args,
                required=("delta",),
                optional=("linear_unit", "velocity_scale", "acceleration_scale"),
            )
            payload["reference_frame"] = str(
                args.get("reference_frame") or args.get("frame_id") or "base_link"
            )
            return payload
        if name == "move_cartesian":
            intent = (
                "absolute_move_lin"
                if args.get("motion_type") == "lin"
                else "absolute_move_ptp"
            )
            payload = self._runtime_skill_payload(
                intent,
                args,
                required=("target_pose",),
                optional=(
                    "linear_unit",
                    "orientation_preset",
                    "keep_current_orientation",
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                ),
            )
            payload["reference_frame"] = str(
                args.get("reference_frame") or args.get("frame_id") or "base_link"
            )
            return payload
        if name == "move_joint_delta":
            payload = {"intent": "move_joint_delta"}
            joint_deltas_deg = args.get("joint_deltas_deg")
            if isinstance(joint_deltas_deg, dict) and len(joint_deltas_deg) == 1:
                joint_name, delta_deg = next(iter(joint_deltas_deg.items()))
                payload["joint"] = joint_name
                payload["delta_angle"] = delta_deg
                payload["angular_unit"] = "deg"
            elif "joint_index" in args:
                payload["joint_index"] = args["joint_index"]
            elif "joint_name" in args:
                payload["joint_name"] = args["joint_name"]
            elif "joint" in args:
                payload["joint"] = args["joint"]
            else:
                raise IntentResolutionError(
                    "FactoryTask runtime skill 'move_joint_delta' requires joint_index or joint."
                )
            if "delta_angle" in args:
                payload["delta_angle"] = args["delta_angle"]
            elif "delta_deg" in args:
                payload["delta_angle"] = args["delta_deg"]
                payload["angular_unit"] = "deg"
            elif "delta_angle" not in payload:
                raise IntentResolutionError(
                    "FactoryTask runtime skill 'move_joint_delta' requires delta_angle or delta_deg."
                )
            for key in ("velocity_scale", "acceleration_scale"):
                if key in args:
                    payload[key] = args[key]
            return payload
        if name in {"move_joint", "move_joints"}:
            required = {
                "move_joint": ("joint_index", "joint_angle"),
                "move_joints": ("joint_target",),
            }[name]
            return self._runtime_skill_payload(
                name,
                args,
                required=required,
                optional=("angular_unit", "velocity_scale", "acceleration_scale"),
            )
        if name in {"move_named_pose", "move_to_region"}:
            pose_name = str(
                args.get("pose_name") or args.get("pose") or args.get("region") or ""
            ).strip()
            if not pose_name:
                raise IntentResolutionError(
                    f"FactoryTask runtime skill '{name}' requires pose_name or region."
                )
            return {"intent": "move_named_pose", "pose_name": pose_name}
        if name in {"draw_shape", "draw_text"}:
            payload = {"intent": name}
            payload.update(args)
            return payload
        raise IntentResolutionError(
            f"FactoryTask runtime skill '{name or '<empty>'}' is not supported by the HMI Phase 1 adapter."
        )

    @staticmethod
    def _runtime_skill_payload(
        intent: str,
        args: dict[str, Any],
        *,
        required: tuple[str, ...] = (),
        optional: tuple[str, ...] = (),
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {"intent": intent}
        for key in required:
            if key not in args:
                raise IntentResolutionError(
                    f"FactoryTask runtime skill '{intent}' requires {key}."
                )
            payload[key] = args[key]
        for key in optional:
            if key in args:
                payload[key] = args[key]
        return payload

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
            return {
                "primitive_type": "WAIT",
                "wait_duration_sec": parameters.get("wait_duration_sec"),
            }
        if primitive == "SET_SPEED":
            return {
                "primitive_type": "SET_SPEED",
                "velocity_scale": parameters.get("velocity_scale"),
            }
        if primitive == "IO_SET":
            return {
                "primitive_type": "IO_SET",
                "io_address": parameters.get("io_address"),
                "io_value": parameters.get("io_value"),
            }
        if primitive == "ALARM_RESET":
            return {"primitive_type": "ALARM_RESET"}
        if primitive == "GET_POSE":
            return {
                "primitive_type": "GET_POSE",
                "reference_frame": parameters.get("reference_frame", "base_link"),
            }
        if primitive == "MOVE_REL":
            frame = str(
                parameters.get("frame")
                or parameters.get("reference_frame")
                or "base_link"
            )
            return {
                "primitive_type": "MOVE_REL",
                "delta_x": _to_float(parameters.get("xMm", 0.0), "parameters.xMm")
                / 1000.0,
                "delta_y": _to_float(parameters.get("yMm", 0.0), "parameters.yMm")
                / 1000.0,
                "delta_z": _to_float(parameters.get("zMm", 0.0), "parameters.zMm")
                / 1000.0,
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
                current_deg = self._read_joint_deg(
                    current_joints=current_joints, joint_index=joint_index
                )
                if current_deg is None:
                    raise IntentResolutionError(
                        f"fresh joint position for {JOINT_NAMES[joint_index]} is unavailable.",
                        missing_slots=["fresh_joint_position"],
                    )
                resolved_target = float(current_deg) + _to_float(
                    parameters.get("deltaDeg", 0.0), "parameters.deltaDeg"
                )
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

        raise IntentResolutionError(
            f"unsupported structured action: {payload.get('action')!r}"
        )

    def _sequence_candidate_metadata(self, payload: dict[str, Any]) -> dict[str, Any]:
        intent = str(payload.get("intent") or "").strip().lower()
        metadata: dict[str, Any] = {"intent": intent}
        if intent == "draw_shape":
            metadata["macro_name"] = "draw_shape"
            metadata["shape_type"] = (
                str(payload.get("shape_type", payload.get("shape", ""))).strip().lower()
            )
        if intent == "draw_text":
            metadata["macro_name"] = "draw_text"
            metadata["text"] = str(payload.get("text") or "").strip().upper()
        return metadata

    # W5.T3 — replaced local logic with ROS service call to /llm_gateway/hydrate_workplane
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

        ros = self._ros_adapter
        if ros is not None and hasattr(ros, "hydrate_workplane"):
            import json

            result = ros.hydrate_workplane(
                payload_json=json.dumps(
                    payload, ensure_ascii=True, separators=(",", ":")
                )
            )
            if result.get("success"):
                import json

                hydrated = json.loads(result["hydrated_payload_json"])
                if isinstance(hydrated, dict):
                    return hydrated
            raise IntentResolutionError(
                f"workplane hydration failed: {result.get('error', 'unknown error')}"
            )

        # Fallback: local hydration when ROS adapter is unavailable (sim/test mode)
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

    def _text_to_command(
        self, *, raw_text: str, current_joints: list[Any]
    ) -> dict[str, Any]:
        normalized = " ".join(raw_text.strip().split()).lower()
        if not normalized:
            raise IntentResolutionError(
                "empty command text is not allowed", missing_slots=["intentText"]
            )

        if normalized in {
            "home",
            "go home",
            "move home",
            "return home",
            "move to home",
        }:
            return {"intent": "go_home"}

        if normalized in {"stop", "stop motion", "cancel motion", "halt"}:
            return {"intent": "stop"}

        if normalized in {"get pose", "current pose", "where is robot", "where is tcp"}:
            return {"intent": "get_pose", "reference_frame": "base_link"}

        wait_match = re.fullmatch(
            r"(?:wait|pause)\s+([+-]?\d+(?:\.\d+)?)\s*(?:s|sec|second|seconds)?",
            normalized,
        )
        if wait_match:
            return {
                "intent": "wait",
                "wait_duration_sec": float(wait_match.group(1)),
            }

        # Directional relative moves (local fallback)
        rel_match = re.fullmatch(
            r"(forward|back|backward|left|right|up|down)\s+([+-]?\d+(?:\.\d+)?)\s*(m|cm|mm)?",
            normalized,
        )
        if rel_match:
            direction = rel_match.group(1)
            magnitude = float(rel_match.group(2))
            unit = rel_match.group(3) or "cm"
            scale = {"m": 1.0, "cm": 0.01, "mm": 0.001}[unit]
            distance = magnitude * scale
            axis_delta = {
                "forward": (distance, 0.0, 0.0),
                "back": (-distance, 0.0, 0.0),
                "backward": (-distance, 0.0, 0.0),
                "right": (0.0, -distance, 0.0),
                "left": (0.0, distance, 0.0),
                "up": (0.0, 0.0, distance),
                "down": (0.0, 0.0, -distance),
            }
            dx, dy, dz = axis_delta[direction]
            return {
                "intent": "move_relative",
                "delta": {"x": dx, "y": dy, "z": dz},
                "reference_frame": "base_link",
            }

        # Single-joint move (local fallback)
        joint_match = re.fullmatch(
            r"(rotate|move)\s+joint\s+(\d)\s+by\s+([+-]?\d+(?:\.\d+)?)\s*(deg|degree|degrees|rad|radian|radians)?",
            normalized,
        )
        if joint_match:
            joint_index = int(joint_match.group(2)) - 1
            angle = float(joint_match.group(3))
            unit = (joint_match.group(4) or "deg").lower()
            if unit in {"deg", "degree", "degrees"}:
                angular_unit = "deg"
            elif unit in {"rad", "radian", "radians"}:
                angular_unit = "rad"
            else:
                raise IntentResolutionError("joint angle unit must be deg or rad")
            return {
                "intent": "move_joint_delta",
                "joint_index": joint_index,
                "delta_angle": angle,
                "angular_unit": angular_unit,
            }

        # Multi-joint move (local fallback) - e.g., "rotate joints 1 2 by 10 deg"
        multi_joint_match = re.fullmatch(
            r"(rotate|move)\s+joints?\s+([\d\s]+)\s+by\s+([+-]?\d+(?:\.\d+)?)\s*(deg|degree|degrees|rad|radian|radians)?",
            normalized,
        )
        if multi_joint_match:
            joint_indices = [int(j.strip()) - 1 for j in multi_joint_match.group(2).split()]
            angle = float(multi_joint_match.group(3))
            unit = (multi_joint_match.group(4) or "deg").lower()
            if unit in {"deg", "degree", "degrees"}:
                angular_unit = "deg"
            elif unit in {"rad", "radian", "radians"}:
                angular_unit = "rad"
            else:
                raise IntentResolutionError("joint angle unit must be deg or rad")
            # Return semantic IR for sequence of joint moves
            steps = []
            for idx in joint_indices:
                steps.append({
                    "intent": "move_joint_delta",
                    "joint_index": idx,
                    "delta_angle": angle,
                    "angular_unit": angular_unit,
                })
            return {"intent": "sequence", "steps": steps}

        # Named pose shorthand (local fallback) — returns semantic IR so it routes through IntentRouter
        pose_match = re.fullmatch(
            r"(?:move\s+to\s+pose|go\s+to|move\s+to)\s+([a-zA-Z_][a-zA-Z0-9_]*)",
            normalized,
        )
        if pose_match:
            pose_name = pose_match.group(1)
            if pose_name:
                return {"intent": "move_named_pose", "pose_name": pose_name}

        # Move to XYZ coordinates (local fallback) — returns semantic IR for PTP
        xyz_match = re.fullmatch(
            r"move\s+to\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m)?",
            normalized,
        )
        if xyz_match:
            unit = xyz_match.group(4) or "mm"
            scale = {"m": 1.0, "cm": 0.01, "mm": 0.001}[unit]
            x = float(xyz_match.group(1)) * scale
            y = float(xyz_match.group(2)) * scale
            z = float(xyz_match.group(3)) * scale
            return {
                "intent": "absolute_move_lin",
                "target_pose": {"x": x, "y": y, "z": z},
                "reference_frame": "base_link",
            }

        # Draw circle (local fallback) — returns semantic IR for draw_shape
        circle_match = re.fullmatch(
            r"draw\s+(?:a\s+)?circle(?:\s+(?:with\s+)?radius)?\s+([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m)?",
            normalized,
        )
        if circle_match:
            unit = circle_match.group(2) or "mm"
            scale = {"m": 1.0, "cm": 0.01, "mm": 0.001}[unit]
            radius = float(circle_match.group(1)) * scale
            return {
                "intent": "draw_shape",
                "shape_type": "circle",
                "params": {"radius": radius},
                "frame_id": "base_link",
                "units": unit,
            }

        raise IntentResolutionError(
            "intent is ambiguous or unsupported. HMI local text fallback supports "
            "home, stop, wait <seconds>, get pose, forward/back/left/right/up/down <distance>, "
            "rotate joint <n> by <angle>, and move to pose <name>; complex commands "
            "must come from llm_gateway semantic review.",
        )
