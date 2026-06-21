"""Sequence-handling mixin for :class:`SupervisorService`.

Groups the sequence-specific parsing, routing, summarising, and current-pose
bootstrapping methods. Peer of :class:`SupervisorViewsMixin`,
:class:`SupervisorValidationMixin`, :class:`SupervisorExecutionMixin`.

This module is **not** a standalone service — it must be mixed into
:class:`SupervisorService` because several methods depend on the host class
for ``self._intent_resolution``, ``self._ros``, ``self._parse_intent``, etc.
"""

from __future__ import annotations

import json
import re
import math
from typing import Any

from ..domain.models import RuntimeMode
from .supervisor_validation import IntentResolutionError
from llm_gateway.factory_task import (
    IntentRouter,
    SequenceValidationError,
    SequenceValidator,
)


# Strong sequence separators only. Commas are handled separately so coordinate
# lists like "x 0.3, y 0.0, z 0.3" do not get misclassified as multi-step
# sequences.
_SEQUENCE_SPLIT_PATTERN = re.compile(
    r"(?:\s*;\s*|\s*\n+\s*|\s+(?:and\s+then|then)\s+)",
    re.IGNORECASE,
)

# W5: default blend radius for intermediate items in a blended sequence.
_DEFAULT_BLEND_RADIUS_M = 0.01

# Primitives that can participate in a BLENDED_SEQUENCE.
# W7: Restrict to pose-based LIN/PTP only. Named/home/joint targets
# must execute step-by-step through safety gate.
_BLENDED_SEQUENCE_ELIGIBLE = {"PTP", "LIN"}

# Maximum number of iterations the HMI Phase 1 adapter will expand for a
# FactoryTask `repeat` runtime node. Prevents a malicious or buggy LLM
# payload from generating thousands of motion steps.
_REPEAT_MAX_COUNT = 100

_COMMA_SEQUENCE_PREFIXES = {
    "alarm",
    "bật",
    "bat",
    "clear",
    "close",
    "cho",
    "di",
    "dich",
    "doi",
    "dong",
    "dừng",
    "dung",
    "draw",
    "go",
    "ha",
    "home",
    "lift",
    "lower",
    "mở",
    "mo",
    "move",
    "open",
    "pause",
    "quay",
    "reset",
    "rotate",
    "set",
    "stop",
    "tat",
    "tắt",
    "về",
    "vẽ",
    "ve",
    "wait",
    "write",
    "xoay",
    "đi",
    "đợi",
    "đóng",
    "dịch",
    "nâng",
    "hạ",
    "chờ",
}


def _factory_task_summary_fields(
    structured_intent: dict[str, Any] | None,
) -> dict[str, Any]:
    if not isinstance(structured_intent, dict):
        return {}
    metadata = structured_intent.get("metadata")
    if not isinstance(metadata, dict):
        return {}
    summary: dict[str, Any] = {}
    factory_task = metadata.get("factory_task")
    if isinstance(factory_task, dict):
        summary["factoryTask"] = dict(factory_task)
    runtime_plan = metadata.get("runtime_plan")
    if isinstance(runtime_plan, dict):
        summary["factoryTaskRuntimePlan"] = dict(runtime_plan)
    policy_decisions = metadata.get("policy_decisions")
    if isinstance(policy_decisions, list):
        summary["factoryTaskPolicyDecisions"] = [
            dict(item) if isinstance(item, dict) else item
            for item in policy_decisions
        ]
    return summary

class SupervisorSequenceMixin:
    """Sequence parsing, routing, and summary helpers for SupervisorService."""

    # ── Sequence intake ──────────────────────────────────────────────────

    @staticmethod
    def _is_sequence_request(
        structured_intent: dict[str, Any] | None,
        sequence_segments: list[str],
    ) -> bool:
        """Return True when the input should be handled as a multi-step sequence."""
        if (
            structured_intent is not None
            and str(structured_intent.get("intent") or "").strip().lower() == "sequence"
        ):
            return True
        return len(sequence_segments) > 1

    @staticmethod
    def _split_sequence_text(raw_text: str) -> list[str]:
        """Split a free-form instruction into cleaned sequence segments."""
        if not raw_text:
            return []
        segments: list[str] = []
        for segment in _SEQUENCE_SPLIT_PATTERN.split(raw_text):
            cleaned = re.sub(
                r"^(?:and\s+then|then)\s+",
                "",
                segment.strip(" ,;"),
                flags=re.IGNORECASE,
            )
            if not cleaned:
                continue

            comma_parts = [part.strip(" ,;") for part in cleaned.split(",")]
            current = comma_parts[0]
            for part in comma_parts[1:]:
                if not part:
                    continue
                if SupervisorSequenceMixin._starts_new_comma_clause(part):
                    if current:
                        segments.append(current)
                    current = part
                else:
                    current = f"{current}, {part}" if current else part
            if current:
                segments.append(current)
        return segments

    @staticmethod
    def _starts_new_comma_clause(fragment: str) -> bool:
        """Return True when a comma-separated fragment starts a new command."""
        words = fragment.strip().split()
        if not words:
            return False
        return words[0].lower() in _COMMA_SEQUENCE_PREFIXES

    # ── Sequence planning (impure — host state needed) ───────────────────

    def _prepare_sequence_request(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        requested_mode: RuntimeMode,
    ) -> dict[str, Any] | None:
        if structured_intent is None:
            return None

        intent_name = str(structured_intent.get("intent") or "").strip().lower()
        if intent_name not in {"sequence", "draw_shape", "draw_text", "factory_task_runtime"}:
            return None

        route_metadata = {
            "macro_name": structured_intent.get("intent"),
            "text": structured_intent.get("text"),
            "shape_type": structured_intent.get("shape_type"),
        }
        if intent_name == "factory_task_runtime":
            metadata = structured_intent.get("metadata") or {}
            task_info = metadata.get("factory_task") or {}
            plan_info = metadata.get("runtime_plan") or {}
            route_metadata = {
                "factory_task": task_info,
                "factory_task_runtime_plan": plan_info,
                "factory_task_policy_decisions": metadata.get("policy_decisions") or [],
            }

        diagnostics: list[str] = []
        working_payload = dict(structured_intent)
        current_joints = self._current_joints()

        try:
            start_joints_rad = self._current_joints_rad()
            working_payload = self._inject_return_to_start_joints(
                working_payload, start_joints_rad
            )
            working_payload = self._prepare_semantic_ir_for_routing(
                working_payload, current_joints
            )
        except IntentResolutionError as exc:
            return {
                "parsed_steps": None,
                "diagnostics": diagnostics,
                "parse_error": str(exc),
                "route_metadata": route_metadata,
                "structured_intent": working_payload,
            }

        if intent_name == "factory_task_runtime":
            runtime_plan = working_payload.get("metadata", {}).get("runtime_plan", {})
            if not isinstance(runtime_plan, dict):
                return None
            try:
                semantic_steps = self._runtime_plan_to_semantic_steps(
                    runtime_plan,
                    current_pose_loader=self._current_pose_snapshot,
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

                parsed_steps = []
                for step in semantic_steps:
                    step_intent = str(step.get("intent") or "").strip().lower()
                    if step_intent in {"draw_shape", "draw_text"}:
                        routed = IntentRouter(runtime_mode=requested_mode.value).route(step)
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
                        for command in routed.commands:
                            parsed_step, parse_err = self._parse_intent(
                                raw_text="",
                                structured_intent=command,
                                mode=requested_mode,
                            )
                            if parsed_step is None:
                                raise IntentResolutionError(parse_err or "failed to parse routed command")
                            parsed_steps.append(parsed_step)
                        continue
                    
                    parsed_step, parse_err = self._parse_intent(
                        raw_text="",
                        structured_intent=step,
                        mode=requested_mode,
                    )
                    if parsed_step is None:
                        raise IntentResolutionError(parse_err or "failed to parse step")
                    parsed_steps.append(parsed_step)

                validation = SequenceValidator().validate(
                    [step.get("rawCommand", step["normalizedCommand"]) for step in parsed_steps]
                )
                diagnostics.extend(validation.diagnostics)
            except (IntentResolutionError, SequenceValidationError, ValueError) as exc:
                return {
                    "parsed_steps": None,
                    "diagnostics": diagnostics,
                    "parse_error": str(exc),
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

        if intent_name in {"draw_shape", "draw_text"}:
            try:
                working_payload = self._hydrate_draw_workplane(
                    working_payload,
                    current_pose_loader=self._current_pose_snapshot,
                )
            except IntentResolutionError as exc:
                return {
                    "parsed_steps": None,
                    "diagnostics": diagnostics,
                    "parse_error": str(exc),
                    "route_metadata": route_metadata,
                    "structured_intent": working_payload,
                }

        try:
            routed = IntentRouter(runtime_mode=requested_mode.value).route(working_payload)
            if routed.route_type == "error":
                message = (routed.error_payload or {}).get("message") or (
                    routed.error_payload or {}
                ).get("error")
                raise IntentResolutionError(
                    str(message or "intent router returned an error payload.")
                )
            route_metadata.update(dict(routed.metadata))
            if routed.metadata.get("macro_name") in {"draw_shape", "draw_text"}:
                for command in routed.commands:
                    if bool(command.get("plan_only")):
                        raise IntentResolutionError(
                            "draw execution_mode=plan_only is not supported by the HMI; "
                            "resubmit with execution_mode='execute'."
                        )
            
            parsed_steps = []
            for command in routed.commands:
                parsed_step, parse_err = self._parse_intent(
                    raw_text="",
                    structured_intent=command,
                    mode=requested_mode,
                )
                if parsed_step is None:
                    raise IntentResolutionError(parse_err or "failed to parse command")
                parsed_steps.append(parsed_step)

            validation = SequenceValidator().validate(routed.commands)
            diagnostics.extend(validation.diagnostics)
        except (IntentResolutionError, SequenceValidationError, ValueError) as exc:
            return {
                "parsed_steps": None,
                "diagnostics": diagnostics,
                "parse_error": str(exc),
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

        ros = self._ros
        if ros is not None and hasattr(ros, "hydrate_workplane"):
            result = ros.hydrate_workplane(
                payload_json=json.dumps(
                    payload, ensure_ascii=True, separators=(",", ":")
                )
            )
            if result.get("success"):
                hydrated = json.loads(result["hydrated_payload_json"])
                if isinstance(hydrated, dict):
                    return hydrated
            raise IntentResolutionError(
                f"workplane hydration failed: {result.get('error', 'unknown error')}"
            )

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
        if node_type == "repeat":
            body = runtime_plan.get("body")
            if not isinstance(body, dict):
                raise IntentResolutionError(
                    "FactoryTask runtime repeat node requires a 'body' object."
                )
            count = runtime_plan.get("count")
            if not isinstance(count, int) or isinstance(count, bool):
                raise IntentResolutionError(
                    "FactoryTask runtime repeat node requires an integer 'count'."
                )
            if count < 1:
                raise IntentResolutionError(
                    "FactoryTask runtime repeat count must be >= 1; got "
                    f"{count}."
                )
            if count > _REPEAT_MAX_COUNT:
                raise IntentResolutionError(
                    f"FactoryTask runtime repeat count {count} exceeds the "
                    f"HMI Phase 1 limit of {_REPEAT_MAX_COUNT}."
                )
            inner_steps = self._runtime_plan_to_semantic_steps(
                body,
                current_pose_loader=current_pose_loader,
            )
            return list(inner_steps) * count
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

    def _runtime_skill_payload(
        self,
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

    def _current_pose_snapshot(self) -> dict[str, Any] | None:
        reader = getattr(self._ros, "get_current_pose", None)
        if not callable(reader):
            return None
        return reader(reference_frame="base_link")

    @staticmethod
    def _inject_return_to_start_joints(
        payload: dict[str, Any],
        start_joints_rad: list[float] | None,
    ) -> dict[str, Any]:
        """If any step is return_to_start, inject captured joint_target."""
        enriched = dict(payload)
        intent_name = str(enriched.get("intent") or "").strip().lower()
        if intent_name == "return_to_start":
            if not start_joints_rad or len(start_joints_rad) != 6:
                return enriched
            enriched["joint_target"] = [float(v) for v in start_joints_rad]
            return enriched
        if intent_name == "sequence":
            steps = enriched.get("steps")
            if isinstance(steps, list):
                enriched["steps"] = [
                    SupervisorSequenceMixin._inject_return_to_start_joints(
                        step, start_joints_rad
                    )
                    if isinstance(step, dict)
                    else step
                    for step in steps
                ]
        return enriched

    def _prepare_semantic_ir_for_routing(
        self, payload: dict[str, Any], current_joints: list[Any]
    ) -> dict[str, Any]:
        def contains_intent(val: Any, target: str) -> bool:
            if isinstance(val, dict):
                if val.get("intent") == target:
                    return True
                for child in val.values():
                    if contains_intent(child, target):
                        return True
            elif isinstance(val, list):
                for child in val:
                    if contains_intent(child, target):
                        return True
            return False

        if not contains_intent(payload, "move_joint_delta"):
            return dict(payload)

        from llm_gateway.factory_task import prepare_semantic_ir_for_routing
        if len(current_joints) != 6:
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
        try:
            return prepare_semantic_ir_for_routing(
                payload,
                current_joint_positions_rad=joint_positions_rad,
            )
        except ValueError as exc:
            raise IntentResolutionError(str(exc)) from exc

    def _parse_sequence_steps(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        mode: RuntimeMode,
        sequence_segments: list[str],
    ) -> tuple[
        list[dict[str, Any]] | None, list[str], str | None, dict[str, Any] | None
    ]:
        diagnostics: list[str] = []
        parsed_steps: list[dict[str, Any]] = []
        start_joints_rad = self._current_joints_rad()
        if (
            structured_intent is not None
            and str(structured_intent.get("intent") or "").strip().lower() == "sequence"
        ):
            try:
                routed_payload = self._inject_return_to_start_joints(
                    structured_intent, start_joints_rad
                )
                routed_payload = self._prepare_semantic_ir_for_routing(
                    routed_payload, self._current_joints()
                )
                routed = IntentRouter(runtime_mode=mode.value).route(routed_payload)
                if routed.route_type != "sequence":
                    return (
                        None,
                        diagnostics,
                        "structured sequence did not resolve to a sequence route.",
                        {"intent": "sequence"},
                    )
                validation = SequenceValidator().validate(routed.commands)
                diagnostics.extend(validation.diagnostics)
                for command in routed.commands:
                    parsed_step, parse_err = self._parse_intent(
                        raw_text="",
                        structured_intent=command,
                        mode=mode,
                    )
                    if parsed_step is None:
                        return None, diagnostics, parse_err, {"intent": "sequence"}
                    parsed_steps.append(parsed_step)
            except (IntentResolutionError, SequenceValidationError, ValueError) as exc:
                return None, diagnostics, str(exc), {"intent": "sequence"}
            return parsed_steps, diagnostics, None, dict(routed.metadata)

        for segment in sequence_segments:
            parsed_step, parse_error = self._parse_intent(segment, None, mode=mode)
            if parsed_step is None:
                return None, diagnostics, parse_error, None
            parsed_steps.append(parsed_step)
        try:
            validation = SequenceValidator().validate(
                [parsed_step.get("rawCommand", parsed_step["normalizedCommand"]) for parsed_step in parsed_steps]
            )
            diagnostics.extend(validation.diagnostics)
        except SequenceValidationError as exc:
            return None, diagnostics, str(exc), None
        return parsed_steps, diagnostics, None, None

    # ── Sequence summarisation (pure) ────────────────────────────────────

    @staticmethod
    def _sequence_summary_label(
        *,
        parsed_steps: list[dict[str, Any]],
        route_metadata: dict[str, Any] | None,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
    ) -> str:
        macro_name = str((route_metadata or {}).get("macro_name") or "").strip().lower()
        if macro_name == "draw_shape":
            shape_type = (
                str((route_metadata or {}).get("shape_type") or "shape").strip().lower()
                or "shape"
            )
            return f"Draw {shape_type}"[:120]
        if macro_name == "draw_text":
            text = str((route_metadata or {}).get("text") or "").strip().upper()
            return f"Draw text '{text}'"[:120] if text else "Draw text"
        if parsed_steps:
            return " -> ".join(step["targetSummary"] for step in parsed_steps)[:120]
        if raw_text:
            return raw_text[:120]
        if structured_intent is not None:
            return json.dumps(
                structured_intent, separators=(",", ":"), ensure_ascii=True
            )[:120]
        return "structured sequence"

    def _sequence_plan_summary(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        parsed_steps: list[dict[str, Any]],
        diagnostics: list[str],
        route_metadata: dict[str, Any] | None,
        requires_confirmation: bool,
    ) -> dict[str, Any]:
        summary = {
            "normalizedIntent": (
                raw_text
                or (
                    json.dumps(
                        structured_intent, separators=(",", ":"), ensure_ascii=True
                    )
                    if structured_intent is not None
                    else ""
                )
            ),
            "parsedAction": "SEQUENCE",
            "targetSummary": self._sequence_summary_label(
                parsed_steps=parsed_steps,
                route_metadata=route_metadata,
                raw_text=raw_text,
                structured_intent=structured_intent,
            ),
            "requiresConfirmation": requires_confirmation,
            "stepCount": len(parsed_steps),
        }
        summary.update(_factory_task_summary_fields(structured_intent))
        if diagnostics:
            summary["diagnostics"] = list(diagnostics)
        macro_name = str((route_metadata or {}).get("macro_name") or "").strip().lower()
        if macro_name:
            summary["macroName"] = macro_name
        if macro_name == "draw_shape":
            summary["shapeType"] = (route_metadata or {}).get("shape_type")
        if macro_name == "draw_text":
            summary["text"] = (route_metadata or {}).get("text")
        if isinstance((route_metadata or {}).get("summary"), dict):
            summary["macroSummary"] = dict((route_metadata or {})["summary"])
        if (route_metadata or {}).get("execution_mode") is not None:
            summary["executionMode"] = (route_metadata or {}).get("execution_mode")
        return summary

    # ── BLENDED_SEQUENCE emission (W5) ─────────────────────────────────

    @staticmethod
    def _should_emit_blended_sequence(
        parsed_steps: list[dict[str, Any]], route_metadata: dict[str, Any] | None
    ) -> bool:
        """Return True when all steps are motion primitives eligible for blending."""
        if not parsed_steps or len(parsed_steps) < 2:
            return False
        macro = str((route_metadata or {}).get("macro_name") or "").strip().lower()
        if macro in {"draw_shape", "draw_text"}:
            return False
        for step in parsed_steps:
            action = str(step.get("action") or "").strip().upper()
            if action not in _BLENDED_SEQUENCE_ELIGIBLE:
                return False
            norm_cmd = step.get("normalizedCommand") or {}
            primitive = norm_cmd.get("primitive_type", action)
            # execution_gate rejects GOAL_JOINTS and GOAL_NAMED in blended
            # sequences; named poses and HOME resolve to those goal types.
            if primitive == "HOME":
                return False
            if norm_cmd.get("joint_target"):
                return False
            if norm_cmd.get("named_target"):
                return False
            params = step.get("parameters") or {}
            if params.get("joint_target"):
                return False
        return True

    @staticmethod
    def _build_blended_sequence_step(
        parsed_steps: list[dict[str, Any]],
        raw_text: str,
        structured_intent: dict[str, Any] | None,
    ) -> dict[str, Any]:
        """Collapse multiple motion-primitive parsed_steps into one BLENDED_SEQUENCE step."""
        sequence_steps: list[dict[str, Any]] = []
        for idx, step in enumerate(parsed_steps):
            norm_cmd = step.get("normalizedCommand") or {}
            action = str(step.get("action") or "").strip().upper()
            primitive = norm_cmd.get("primitive_type", action)

            # Determine goal type and payload
            goal_type = 0  # GOAL_POSE default
            step_payload: dict[str, Any] = {
                "primitive_type": primitive,
                "goal_type": goal_type,
            }

            # Target: pose, joints, or named
            if primitive == "HOME":
                goal_type = 2  # GOAL_NAMED
                step_payload["goal_type"] = goal_type
                step_payload["named_target"] = "home"
            elif norm_cmd.get("joint_target"):
                goal_type = 1  # GOAL_JOINTS
                step_payload["goal_type"] = goal_type
                step_payload["joint_target"] = list(norm_cmd["joint_target"])
            elif norm_cmd.get("target_pose"):
                step_payload["target_pose"] = dict(norm_cmd["target_pose"])
            else:
                # Fallback: extract from parameters
                params = step.get("parameters") or {}
                if params.get("target_pose"):
                    step_payload["target_pose"] = dict(params["target_pose"])
                elif params.get("joint_target"):
                    goal_type = 1
                    step_payload["goal_type"] = goal_type
                    step_payload["joint_target"] = list(params["joint_target"])

            # Blend radius: last must be 0.0; intermediates use default.
            if idx == len(parsed_steps) - 1:
                step_payload["blend_radius_m"] = 0.0
            else:
                step_payload["blend_radius_m"] = _DEFAULT_BLEND_RADIUS_M

            # Planner / scales
            step_payload["planner_id"] = norm_cmd.get("planner_id", "")
            if norm_cmd.get("velocity_scale") is not None:
                step_payload["velocity_scale"] = float(norm_cmd["velocity_scale"])
            if norm_cmd.get("acceleration_scale") is not None:
                step_payload["acceleration_scale"] = float(
                    norm_cmd["acceleration_scale"]
                )

            sequence_steps.append(step_payload)

        # Build the single collapsed parsed step
        target_summary = " -> ".join(
            step.get("targetSummary", "") for step in parsed_steps
        )[:120]

        normalized_command = {
            "primitive_type": "BLENDED_SEQUENCE",
            "sequence_steps": sequence_steps,
        }

        return {
            "source": "blended",
            "normalizedText": raw_text
            or (
                json.dumps(
                    structured_intent, separators=(",", ":"), ensure_ascii=True
                )
                if structured_intent is not None
                else ""
            ),
            "action": "BLENDED_SEQUENCE",
            "parameters": {"sequence_steps": sequence_steps},
            "targetSummary": target_summary,
            "normalizedCommand": normalized_command,
            "normalizationNotes": ["collapsed_into_blended_sequence"],
        }
