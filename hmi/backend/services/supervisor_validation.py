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
from .intent_resolution import IntentResolutionError

EVENT_DRIVEN_SOURCE_NAMES = {"llm_debug", "llm_command"}


class SupervisorValidationMixin:
    def _parse_intent(
        self,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        mode: RuntimeMode,
    ) -> tuple[dict[str, Any] | None, str | None]:
        try:
            parsed = self._intent_resolution.resolve(
                raw_text=raw_text,
                structured_intent=structured_intent,
                runtime_mode=mode.value,
                current_joints=self._current_joints(),
            )
            return parsed, None
        except IntentResolutionError as exc:
            return None, exc.operator_message()
        except ValueError as exc:
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
    ) -> dict[str, Any]:
        source_statuses = self._read_source_statuses()
        critical_sources = [source for source in source_statuses if getattr(source, "active", False)]
        optional_sources = [
            source
            for source in source_statuses
            if not getattr(source, "active", False) and source.name not in EVENT_DRIVEN_SOURCE_NAMES
        ]
        event_driven_sources = [source for source in source_statuses if source.name in EVENT_DRIVEN_SOURCE_NAMES]
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

        if requested_mode == RuntimeMode.HARDWARE and not hardware_gate.unlocked:
            blocking_reasons.extend(hardware_gate.reasons)

        stale_sources = [
            source.name
            for source in critical_sources
            if source.freshness_state != TelemetryFreshnessState.FRESH
        ]
        if stale_sources:
            blocking_reasons.append(
                "freshness-critical telemetry is stale or unavailable: " + ", ".join(stale_sources)
            )

        if not preflight.get("accepted", True):
            preflight_reasons = list(preflight.get("reasons") or [])
            if preflight_reasons:
                blocking_reasons.extend(preflight_reasons)

        risk_level = self._assess_risk(parsed_intent)
        if risk_level in {CommandRiskLevel.HIGH, CommandRiskLevel.CRITICAL}:
            confirmation_reasons.append(
                f"Risk assessment is {risk_level.value}; high-risk actions must stay behind confirmation."
            )

        action = str(parsed_intent.get("action") if parsed_intent is not None else "").upper()
        if action in {"MOVE_REL", "MOVE_JOINT", "MOVE_JOINTS", "PTP", "LIN", "CIRC", "CARTESIAN_PATH"}:
            confirmation_reasons.append("Motion primitives always require explicit confirmation in v2.")

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
            "criticalSources": [self._source_status_view(source) for source in critical_sources],
            "optionalSources": [self._source_status_view(source) for source in optional_sources],
            "eventDrivenSources": [self._source_status_view(source) for source in event_driven_sources],
            "hardwareGate": hardware_gate.to_dict(),
            "preflight": preflight,
        }

    def _assess_risk(self, parsed_intent: dict[str, Any] | None) -> CommandRiskLevel | None:
        if parsed_intent is None:
            return None
        action = str(parsed_intent.get("action") or "").upper()
        parameters = parsed_intent.get("normalizedCommand") or parsed_intent.get("parameters") or {}
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

    def _plan_fingerprint(self, parsed_intent: dict[str, Any], lease_id: str, runtime_mode: str) -> str:
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
