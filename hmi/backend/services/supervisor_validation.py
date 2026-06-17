from __future__ import annotations

import hashlib
import json
import math
from typing import Any

from ..domain.models import (
    CommandRiskLevel,
    RuntimeMode,
    RuntimeSnapshot,
    TelemetryFreshnessState,
)
from ..domain.state_machine import is_blocking_runtime_state
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


EVENT_DRIVEN_SOURCE_NAMES = {
    "llm_debug",
    "llm_command",
    "review_intent_service",
}

POST_PARSE_INFO_ONLY_SOURCE_NAMES = {
    "gateway_status",
    "supervisor_alerts",
}

EXECUTION_BOUNDARY_SOURCE_NAMES = {
    "validate_command_service",
    "execute_motion_action",
}


class SupervisorValidationMixin:
    def _target_summary(self, normalized_command: dict[str, Any]) -> str:
        primitive = normalized_command["primitive_type"]
        if primitive == "HOME":
            return "Return robot to configured home pose."
        if primitive == "STOP":
            return "Request supervised stop handling."
        if primitive == "MOVE_REL":
            return (
                "Relative translation in base_link: "
                f"dx={float(normalized_command.get('delta_x', 0.0)) * 1000.0:.1f} mm "
                f"dy={float(normalized_command.get('delta_y', 0.0)) * 1000.0:.1f} mm "
                f"dz={float(normalized_command.get('delta_z', 0.0)) * 1000.0:.1f} mm."
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
        if primitive == "FACTORY_TASK_RUNTIME":
            metadata = normalized_command.get("metadata") or {}
            return metadata.get("operator_summary") or "Execute consolidated FactoryTask runtime plan."
        return f"Execute primitive {primitive}."

    def _to_jsonable(self, val: Any) -> Any:
        if isinstance(val, dict):
            return {k: self._to_jsonable(v) for k, v in val.items()}
        if isinstance(val, list):
            return [self._to_jsonable(v) for v in val]
        if hasattr(val, "__slots__") and not isinstance(val, type):
            return {slot: self._to_jsonable(getattr(val, slot)) for slot in val.__slots__}
        return val

    def _parse_intent(
        self,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        mode: RuntimeMode,
    ) -> tuple[dict[str, Any] | None, str | None]:
        try:
            runtime_mode_str = str(mode.value or "hardware").strip().lower()

            if structured_intent is None and raw_text:
                normalized_text = raw_text.strip().lower()
                if normalized_text in {"home", "go home", "move home", "return home", "move to home"}:
                    structured_intent = {"intent": "go_home"}
                elif normalized_text in {"stop", "stop motion", "cancel motion", "halt"}:
                    structured_intent = {"intent": "stop"}
                elif normalized_text in {"get pose", "current pose", "where is robot", "where is tcp"}:
                    structured_intent = {"intent": "get_pose"}
                elif normalized_text in {"alarm reset", "reset alarm", "clear alarm"}:
                    structured_intent = {"intent": "alarm_reset"}

            if structured_intent is None:
                raise IntentResolutionError(
                    "structuredIntent is required for semantic intent parsing."
                )

            from llm_gateway.semantic_ir_contract import is_factory_task_runtime_sentinel
            if is_factory_task_runtime_sentinel(structured_intent):
                metadata = structured_intent.get("metadata") or {}
                operator_summary = metadata.get("operator_summary") or "Execute consolidated FactoryTask runtime plan."
                normalized_command = self._to_jsonable({
                    "primitive_type": "FACTORY_TASK_RUNTIME",
                    "metadata": metadata,
                    "_factory_task_runtime": True,
                    "intent": "factory_task_runtime",
                })
                parsed = {
                    "source": "structured",
                    "normalizedText": json.dumps(
                        structured_intent, separators=(",", ":"), ensure_ascii=True
                    ),
                    "action": "FACTORY_TASK_RUNTIME",
                    "intent": "factory_task_runtime",
                    "_factory_task_runtime": True,
                    "metadata": metadata,
                    "parameters": {
                        "metadata": metadata,
                        "_factory_task_runtime": True,
                        "intent": "factory_task_runtime",
                    },
                    "targetSummary": operator_summary,
                    "normalizedCommand": normalized_command,
                    "rawCommand": normalized_command,
                    "normalizationNotes": [],
                }
                return parsed, None

            from llm_gateway.factory_task import IntentRouter, Normalizer
            from .intent_constants import ROUTED_DRAW_METADATA_FIELDS

            if "primitive_type" in structured_intent:
                normalized_command = self._to_jsonable(Normalizer().normalize(structured_intent))
                if isinstance(normalized_command, dict):
                    normalized_command = {
                        k: v for k, v in normalized_command.items()
                        if k not in ROUTED_DRAW_METADATA_FIELDS
                    }
                primitive_type = str(normalized_command["primitive_type"])
                parameters = {
                    key: value
                    for key, value in normalized_command.items()
                    if key != "primitive_type"
                }
                parsed = {
                    "source": "structured",
                    "normalizedText": json.dumps(
                        structured_intent, separators=(",", ":"), ensure_ascii=True
                    ),
                    "action": primitive_type,
                    "parameters": parameters,
                    "targetSummary": self._target_summary(normalized_command),
                    "normalizedCommand": normalized_command,
                    "rawCommand": structured_intent,
                    "normalizationNotes": [],
                }
                return parsed, None

            if "intent" not in structured_intent:
                raise IntentResolutionError(
                    "structuredIntent must include action, intent, or primitive_type.",
                    missing_slots=["action|intent|primitive_type"],
                )

            if hasattr(self, "_prepare_semantic_ir_for_routing"):
                try:
                    current_joints = self._current_joints()
                    structured_intent = self._prepare_semantic_ir_for_routing(
                        structured_intent, current_joints
                    )
                except Exception as exc:
                    raise IntentResolutionError(str(exc))

            routed = IntentRouter(runtime_mode=runtime_mode_str).route(structured_intent)
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

            raw_cmd = routed.commands[0]
            normalized_command = self._to_jsonable(Normalizer().normalize(raw_cmd))
            if isinstance(normalized_command, dict):
                normalized_command = {
                    k: v for k, v in normalized_command.items()
                    if k not in ROUTED_DRAW_METADATA_FIELDS
                }
            primitive_type = str(normalized_command["primitive_type"])
            parameters = {
                key: value
                for key, value in normalized_command.items()
                if key != "primitive_type"
            }

            parsed = {
                "source": "structured",
                "normalizedText": json.dumps(
                    structured_intent, separators=(",", ":"), ensure_ascii=True
                ),
                "action": primitive_type,
                "parameters": parameters,
                "targetSummary": self._target_summary(normalized_command),
                "normalizedCommand": normalized_command,
                "rawCommand": raw_cmd,
                "normalizationNotes": [],
            }
            return parsed, None

        except IntentResolutionError as exc:
            return None, exc.operator_message()
        except Exception as exc:
            return None, str(exc)

    def _planner_for_intent(self, parsed_intent: dict[str, Any]) -> str | None:
        normalized_command = parsed_intent.get("normalizedCommand") or {}
        planner = normalized_command.get("planner_id")
        if planner is None:
            return None
        return str(planner)

    def _frame_for_intent(self, parsed_intent: dict[str, Any]) -> str | None:
        action = str(parsed_intent.get("action") or "").upper()
        normalized_command = parsed_intent.get("normalizedCommand") or {}
        if action in {"MOVE_JOINT", "MOVE_JOINTS"}:
            return "joint_space"
        reference_frame = normalized_command.get("reference_frame")
        if reference_frame is None:
            return None
        return str(reference_frame)

    def _validate_command(
        self,
        *,
        runtime: RuntimeSnapshot,
        lease: Any,
        parsed_intent: dict[str, Any] | None,
        requested_mode: RuntimeMode,
        enforce_execution_readiness: bool = False,
    ) -> dict[str, Any]:
        source_statuses = self._read_source_statuses()
        nonblocking_source_names = self._nonblocking_source_names(
            enforce_execution_readiness=enforce_execution_readiness
        )
        critical_sources = [
            source
            for source in source_statuses
            if getattr(source, "active", False)
            and source.name not in nonblocking_source_names
        ]
        optional_sources = [
            source
            for source in source_statuses
            if not getattr(source, "active", False)
            and source.name not in EVENT_DRIVEN_SOURCE_NAMES
        ]
        event_driven_sources = [
            source
            for source in source_statuses
            if source.name in EVENT_DRIVEN_SOURCE_NAMES
        ]
        blocking_reasons: list[str] = []
        confirmation_reasons = [
            "HMI v2 requires explicit operator confirmation before a validated plan may cross the execution boundary."
        ]
        hardware_gate = self._hardware_gate_evaluator.evaluate()
        preflight = self._execution_preflight(requested_mode=requested_mode)

        if parsed_intent is None:
            blocking_reasons.append("parsed intent is unavailable")

        if requested_mode not in {RuntimeMode.SIM, RuntimeMode.HARDWARE}:
            blocking_reasons.append(
                f"runtime mode {requested_mode.value} is not command-capable for HMI v2."
            )

        if runtime.mode != requested_mode:
            blocking_reasons.append(
                f"requested mode {requested_mode.value} does not match runtime mode {runtime.mode.value}."
            )

        if is_blocking_runtime_state(runtime.system_state):
            blocking_reasons.append(
                f"runtime state {runtime.system_state.value} is hard-blocking for command-capable actions"
            )

        stale_sources = [
            source.name
            for source in critical_sources
            if source.freshness_state != TelemetryFreshnessState.FRESH
        ]
        if stale_sources:
            blocking_reasons.append(
                "freshness-critical telemetry is stale or unavailable: "
                + ", ".join(stale_sources)
            )

        if not preflight.get("accepted", True):
            preflight_reasons = self._filtered_preflight_reasons(
                preflight.get("reasons") or [],
                enforce_execution_readiness=enforce_execution_readiness,
            )
            if preflight_reasons:
                blocking_reasons.extend(preflight_reasons)

        risk_level = self._assess_risk(parsed_intent)
        if risk_level in {CommandRiskLevel.HIGH, CommandRiskLevel.CRITICAL}:
            confirmation_reasons.append(
                f"Risk assessment is {risk_level.value}; high-risk actions must stay behind confirmation."
            )

        action = str(
            parsed_intent.get("action") if parsed_intent is not None else ""
        ).upper()
        if action in {
            "MOVE_REL",
            "MOVE_JOINT",
            "MOVE_JOINTS",
            "PTP",
            "LIN",
            "CIRC",
            "CARTESIAN_PATH",
            "BLENDED_SEQUENCE",
        }:
            confirmation_reasons.append(
                "Motion primitives always require explicit confirmation in v2."
            )

        plan_fingerprint = (
            self._plan_fingerprint(parsed_intent, lease.lease_id, requested_mode.value)
            if parsed_intent is not None and not blocking_reasons
            else None
        )
        return {
            "accepted": not blocking_reasons,
            "leaseValid": True,
            "runtimeAllowed": not is_blocking_runtime_state(runtime.system_state),
            "telemetryFresh": not stale_sources,
            "requiresConfirmation": True,
            "riskLevel": risk_level.value if risk_level else None,
            "blockingReasons": list(dict.fromkeys(blocking_reasons)),
            "confirmationReasons": confirmation_reasons,
            "planFingerprint": plan_fingerprint,
            "executionAllowedNow": False,
            "criticalSources": [
                self._source_status_view(source) for source in critical_sources
            ],
            "optionalSources": [
                self._source_status_view(source) for source in optional_sources
            ],
            "eventDrivenSources": [
                self._source_status_view(source) for source in event_driven_sources
            ],
            "hardwareGate": hardware_gate.to_dict(),
            "preflight": preflight,
        }

    def _nonblocking_source_names(
        self, *, enforce_execution_readiness: bool
    ) -> set[str]:
        nonblocking = set(EVENT_DRIVEN_SOURCE_NAMES)
        nonblocking.update(POST_PARSE_INFO_ONLY_SOURCE_NAMES)
        if not enforce_execution_readiness:
            nonblocking.update(EXECUTION_BOUNDARY_SOURCE_NAMES)
        return nonblocking

    def _filtered_preflight_reasons(
        self,
        reasons: list[Any],
        *,
        enforce_execution_readiness: bool,
    ) -> list[str]:
        if enforce_execution_readiness:
            return [str(reason) for reason in reasons]

        ignored_source_names = (
            POST_PARSE_INFO_ONLY_SOURCE_NAMES | EXECUTION_BOUNDARY_SOURCE_NAMES
        )
        filtered: list[str] = []
        for reason in reasons:
            text = str(reason)
            if any(source_name in text for source_name in ignored_source_names):
                continue
            filtered.append(text)
        return filtered

    def _assess_risk(
        self, parsed_intent: dict[str, Any] | None
    ) -> CommandRiskLevel | None:
        if parsed_intent is None:
            return None
        action = str(parsed_intent.get("action") or "").upper()
        parameters = (
            parsed_intent.get("normalizedCommand")
            or parsed_intent.get("parameters")
            or {}
        )
        if action in {"STOP", "WAIT", "GET_POSE", "ALARM_RESET", "IO_SET", "SET_SPEED"}:
            return CommandRiskLevel.LOW
        if action == "HOME":
            return CommandRiskLevel.MEDIUM
        if action == "MOVE_JOINT":
            magnitude = abs(float(parameters.get("joint_angle", 0.0)))
            if magnitude <= math.radians(5.0):
                return CommandRiskLevel.LOW
            if magnitude <= math.radians(20.0):
                return CommandRiskLevel.MEDIUM
            return CommandRiskLevel.HIGH
        if action == "MOVE_JOINTS":
            return CommandRiskLevel.HIGH
        if action == "MOVE_REL":
            magnitude = math.sqrt(
                float(parameters.get("delta_x", 0.0)) ** 2
                + float(parameters.get("delta_y", 0.0)) ** 2
                + float(parameters.get("delta_z", 0.0)) ** 2
            )
            if magnitude <= 0.02:
                return CommandRiskLevel.MEDIUM
            if magnitude <= 0.10:
                return CommandRiskLevel.HIGH
            return CommandRiskLevel.CRITICAL
        if action in {"PTP", "LIN", "CIRC", "CARTESIAN_PATH"}:
            velocity_scale = float(parameters.get("velocity_scale", 0.0))
            if velocity_scale <= 0.03:
                return CommandRiskLevel.MEDIUM
            return CommandRiskLevel.HIGH
        return CommandRiskLevel.HIGH

    def _plan_fingerprint(
        self, parsed_intent: dict[str, Any], lease_id: str, runtime_mode: str
    ) -> str:
        stable_blob = json.dumps(
            {
                "parsedIntent": parsed_intent,
                "leaseId": lease_id,
                "runtimeMode": runtime_mode,
            },
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        )
        return hashlib.sha256(stable_blob.encode("ascii")).hexdigest()

    def _build_validation_result(
        self,
        *,
        runtime: RuntimeSnapshot,
        lease: Any,
        parsed_intent: dict[str, Any] | None,
        blocking_reasons: list[str],
        risk_level: CommandRiskLevel | None,
        requires_confirmation: bool,
    ) -> dict[str, Any]:
        hardware_gate = self._hardware_gate_evaluator.evaluate()
        preflight = self._execution_preflight(requested_mode=runtime.mode)
        return {
            "accepted": not blocking_reasons,
            "leaseValid": lease is not None,
            "runtimeAllowed": not is_blocking_runtime_state(runtime.system_state),
            "telemetryFresh": False,
            "requiresConfirmation": requires_confirmation,
            "riskLevel": risk_level.value if risk_level else None,
            "blockingReasons": list(dict.fromkeys(blocking_reasons)),
            "confirmationReasons": [],
            "planFingerprint": None,
            "executionAllowedNow": False,
            "criticalSources": [],
            "optionalSources": [],
            "eventDrivenSources": [],
            "hardwareGate": hardware_gate.to_dict(),
            "preflight": preflight,
        }

    def _execution_preflight(self, *, requested_mode: RuntimeMode) -> dict[str, Any]:
        evaluator = getattr(self._ros, "evaluate_execution_preflight", None)
        if callable(evaluator):
            payload = evaluator(target_mode=requested_mode.value)
            if isinstance(payload, dict):
                accepted = bool(payload.get("accepted", False))
                reasons = payload.get("reasons") or []
                if not isinstance(reasons, list):
                    reasons = [str(reasons)]
                return {
                    "accepted": accepted,
                    "mode": str(payload.get("mode") or requested_mode.value),
                    "reasons": [str(reason) for reason in reasons],
                    "requiredSources": payload.get("requiredSources") or [],
                    "sourceStatuses": payload.get("sourceStatuses") or [],
                    "runtimeState": payload.get("runtimeState"),
                }
        return {
            "accepted": True,
            "mode": requested_mode.value,
            "reasons": [],
            "requiredSources": [],
            "sourceStatuses": [],
            "runtimeState": None,
        }
