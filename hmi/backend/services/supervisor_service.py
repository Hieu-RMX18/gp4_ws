from __future__ import annotations

import json
import logging
from collections import deque
from datetime import datetime, timedelta, timezone
from threading import Lock
from typing import Any
from uuid import uuid4

from ..domain.models import (
    ChatMessage,
    CommandKind,
    CommandLifecycleState,
    CommandRecord,
    CommandRiskLevel,
    JointPosition,
    RuntimeMode,
    RuntimeSnapshot,
    SystemRuntimeState,
)
from ..domain.state_machine import ensure_command_transition, is_terminal_command_state
from .audit_service import AuditService
from .hardware_gate import HardwareGateEvaluator
from .intent_resolution import IntentResolutionService
from .session_lock_service import LeaseNotOwnedError, LeaseRejectedError, SessionLockService
from .supervisor_execution import SupervisorExecutionMixin
from .supervisor_sequence import SupervisorSequenceMixin
from .supervisor_validation import SupervisorValidationMixin
from .supervisor_views import SupervisorViewsMixin
from .telemetry_bridge_service import TelemetryBridgeService


LOGGER = logging.getLogger("uvicorn.error")


def utcnow() -> datetime:
    return datetime.now(timezone.utc)


class SupervisorServiceError(RuntimeError):
    status_code = 400


class ForbiddenActionError(SupervisorServiceError):
    status_code = 403


class NotFoundError(SupervisorServiceError):
    status_code = 404


class ConflictError(SupervisorServiceError):
    status_code = 409


class SupervisorService(
    SupervisorViewsMixin,
    SupervisorValidationMixin,
    SupervisorExecutionMixin,
    SupervisorSequenceMixin,
):
    def __init__(
        self,
        *,
        audit_service: AuditService,
        session_lock_service: SessionLockService,
        ros_adapter: Any,
        confirmation_window_sec: float = 30.0,
        hardware_gate_evaluator: HardwareGateEvaluator | None = None,
        intent_resolution_service: IntentResolutionService | None = None,
        sim_auto_confirm: bool = False,
    ) -> None:
        self._audit = audit_service
        self._session_lock = session_lock_service
        self._ros = ros_adapter
        self._hardware_gate_evaluator = hardware_gate_evaluator or HardwareGateEvaluator()
        self._intent_resolution = intent_resolution_service or IntentResolutionService()
        self._sim_auto_confirm = bool(sim_auto_confirm)
        self._telemetry: TelemetryBridgeService | None = None
        self._confirmation_window = timedelta(seconds=confirmation_window_sec)
        self._commands: dict[str, CommandRecord] = {}
        self._active_command_id: str | None = None
        self._active_sequence_id: str | None = None
        self._messages: deque[ChatMessage] = deque(maxlen=200)
        self._lock = Lock()

    def _trace(self, stage: str, **fields: Any) -> None:
        rendered_fields: list[str] = []
        for key, value in fields.items():
            if value is None:
                continue
            if isinstance(value, (CommandLifecycleState, RuntimeMode, CommandRiskLevel)):
                rendered = value.value
            elif isinstance(value, datetime):
                rendered = value.isoformat()
            elif isinstance(value, (dict, list, tuple)):
                rendered = json.dumps(value, ensure_ascii=True, separators=(",", ":"))
            else:
                rendered = str(value)
            rendered_fields.append(f"{key}={rendered}")
        if rendered_fields:
            LOGGER.info("[HMI CMD] %s | %s", stage, " | ".join(rendered_fields))
            return
        LOGGER.info("[HMI CMD] %s", stage)

    def bind_telemetry_service(self, telemetry_service: TelemetryBridgeService) -> None:
        self._telemetry = telemetry_service
        telemetry_service.set_snapshot_overlay_provider(self.snapshot_overlay)

    def snapshot_overlay(self, session_id: str, operator_id: str) -> dict[str, Any]:
        self._expire_pending_confirmations()
        active_command = self._commands.get(self._active_command_id) if self._active_command_id else None
        active_sequence = self._commands.get(self._active_sequence_id) if self._active_sequence_id else None
        replay_items = self._audit.list_commands(limit=25, top_level_only=True)
        return {
            "capabilities": self._bridge_capabilities().to_dict(),
            "lease": self._serialize_lease_view(session_id, operator_id),
            "messages": [message.to_dict() for message in self._messages],
            "activeCommand": self._serialize_command(active_command),
            "activeSequence": self._serialize_sequence(active_sequence),
            "planMetrics": self._serialize_metrics(active_command.metrics if active_command else None),
            "replayItems": [self._serialize_replay_item(item) for item in replay_items],
        }

    def acquire_lease(
        self,
        *,
        session_id: str,
        operator_id: str,
        force_takeover: bool = False,
        takeover_reason: str | None = None,
    ) -> dict[str, Any]:
        capabilities = self._bridge_capabilities()
        if not capabilities.can_acquire_lease:
            raise ConflictError(
                "Supervisor-owned command ingress is sim-only. Control lease acquisition is disabled outside sim mode."
            )

        try:
            self._session_lock.acquire_controller(
                session_id,
                operator_id,
                force_takeover=force_takeover,
                takeover_reason=takeover_reason,
            )
        except LeaseRejectedError as exc:
            return {
                "accepted": False,
                "lease": self._serialize_lease_view(session_id, operator_id),
                "reason": str(exc),
            }

        self._audit.record_runtime_event(
            system_state=self._current_runtime().system_state,
            session_id=session_id,
            operator_id=operator_id,
            message="controller lease acquired",
            payload={"force_takeover": force_takeover, "takeover_reason": takeover_reason},
        )
        self._broadcast_lease_state()
        return {
            "accepted": True,
            "lease": self._serialize_lease_view(session_id, operator_id),
            "reason": None,
        }

    def renew_lease(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str,
    ) -> dict[str, Any]:
        try:
            self._session_lock.renew(session_id, operator_id, lease_token)
        except LeaseNotOwnedError as exc:
            raise ForbiddenActionError(str(exc)) from exc
        self._broadcast_lease_state()
        return {
            "accepted": True,
            "lease": self._serialize_lease_view(session_id, operator_id),
            "reason": None,
        }

    def release_lease(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str,
    ) -> dict[str, Any]:
        try:
            self._session_lock.release(session_id, operator_id, lease_token)
        except LeaseNotOwnedError as exc:
            raise ForbiddenActionError(str(exc)) from exc

        self._audit.record_runtime_event(
            system_state=self._current_runtime().system_state,
            session_id=session_id,
            operator_id=operator_id,
            message="controller lease released",
        )
        self._broadcast_lease_state()
        return {
            "accepted": True,
            "lease": self._serialize_lease_view(session_id, operator_id),
            "reason": None,
        }

    def submit_intent(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        raw_text: str | None = None,
        structured_intent: dict[str, Any] | None = None,
        mode: str,
    ) -> dict[str, Any]:
        self._expire_pending_confirmations()
        raw_text = (raw_text or "").strip()
        if not raw_text and structured_intent is None:
            raise ConflictError("intentText or structuredIntent is required")

        lease = self._assert_controller(session_id, operator_id, lease_token)
        runtime = self._current_runtime()
        requested_mode = RuntimeMode(mode) if mode in RuntimeMode._value2member_map_ else runtime.mode
        self._assert_no_active_job()
        sequence_segments = self._split_sequence_text(raw_text) if raw_text else []
        prepared_sequence = self._prepare_sequence_request(
            raw_text=raw_text,
            structured_intent=structured_intent,
            requested_mode=requested_mode,
        )
        if prepared_sequence is not None or self._is_sequence_request(structured_intent, sequence_segments):
            return self._submit_sequence(
                session_id=session_id,
                operator_id=operator_id,
                lease=lease,
                runtime=runtime,
                requested_mode=requested_mode,
                raw_text=raw_text,
                structured_intent=structured_intent,
                sequence_segments=sequence_segments,
                prepared_sequence=prepared_sequence,
            )
        return self._submit_single_command(
            session_id=session_id,
            operator_id=operator_id,
            lease=lease,
            runtime=runtime,
            requested_mode=requested_mode,
            raw_text=raw_text,
            structured_intent=structured_intent,
        )

    def _submit_single_command(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease: Any,
        runtime: RuntimeSnapshot,
        requested_mode: RuntimeMode,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
    ) -> dict[str, Any]:
        command_id = str(uuid4())
        correlation_id = str(uuid4())
        command = CommandRecord(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            raw_text=raw_text,
            lifecycle_state=CommandLifecycleState.RECEIVED,
            summary_label=(raw_text or "structured command")[:80],
            mode=requested_mode,
            created_at=utcnow(),
            intent_source="structured" if structured_intent is not None else "text",
            correlation_id=correlation_id,
            structured_intent=structured_intent,
        )
        self._trace(
            "request.received",
            command_id=command_id,
            correlation_id=correlation_id,
            session_id=session_id,
            operator_id=operator_id,
            mode=command.mode.value,
            has_structured_intent=structured_intent is not None,
            raw_text=raw_text or "<structured-intent>",
        )
        self._ros.submit_text_for_review(
            raw_text=raw_text or json.dumps(structured_intent, separators=(",", ":"), ensure_ascii=True),
            session_id=session_id,
            operator_id=operator_id,
            command_id=command_id,
        )

        with self._lock:
            self._commands[command_id] = command
            self._active_command_id = command_id
            self._active_sequence_id = None

        self._append_message(origin="operator", text=raw_text or "Submitted structured command.", command_id=command_id)
        self._audit.upsert_command(command)
        self._audit.record_transition(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            from_state=None,
            to_state=CommandLifecycleState.RECEIVED,
            runtime_state=runtime.system_state,
            reason="command received from HMI v2 intent endpoint",
            payload={"mode": command.mode.value, "intentSource": command.intent_source},
        )

        self._transition_command(
            command,
            next_state=CommandLifecycleState.PARSING,
            reason="supervisor parsing operator intent",
            runtime_state=runtime.system_state,
            message_text=f"Step 1/6 PARSING: received intent '{raw_text or 'structured command'}'.",
            message_tag=CommandLifecycleState.PARSING.value,
        )

        parsed_intent, parse_error = self._parse_intent(
            raw_text,
            structured_intent,
            mode=command.mode,
        )
        if parsed_intent is None:
            self._trace(
                "parse.rejected",
                command_id=command_id,
                correlation_id=correlation_id,
                reason=parse_error,
            )
            self._audit.record_runtime_event(
                system_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
                command_id=command.command_id,
                message="intent resolution failed",
                payload={"reason": parse_error},
            )
            command.reject_reason = parse_error
            command.validation_result = self._build_validation_result(
                runtime=runtime,
                lease=lease,
                parsed_intent=None,
                blocking_reasons=[parse_error],
                risk_level=None,
                requires_confirmation=False,
            )
            self._reject_command(
                command,
                reason=parse_error,
                runtime_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
            )
            return self._command_response(session_id, operator_id, command, accepted=False, reason=parse_error)

        command.parsed_intent = parsed_intent
        self._audit.record_runtime_event(
            system_state=runtime.system_state,
            session_id=session_id,
            operator_id=operator_id,
            command_id=command.command_id,
            message="intent resolution accepted",
            payload={
                "action": parsed_intent.get("action"),
                "normalizedCommand": parsed_intent.get("normalizedCommand"),
                "normalizationNotes": parsed_intent.get("normalizationNotes") or [],
            },
        )

        command.summary_label = parsed_intent["targetSummary"]
        command.planner_used = self._planner_for_intent(parsed_intent)
        command.frame_used = self._frame_for_intent(parsed_intent)
        self._trace(
            "parse.accepted",
            command_id=command_id,
            correlation_id=correlation_id,
            action=parsed_intent.get("action"),
            targetSummary=parsed_intent.get("targetSummary"),
            parameters=parsed_intent.get("parameters"),
            planner=command.planner_used,
            frame=command.frame_used,
        )
        self._transition_command(
            command,
            next_state=CommandLifecycleState.VALIDATING,
            reason="supervisor validating parsed intent",
            runtime_state=runtime.system_state,
            payload={"parsedIntent": parsed_intent},
            message_text=(
                f"Step 2/6 VALIDATING: action={parsed_intent.get('action')} "
                f"planner={command.planner_used or '--'} frame={command.frame_used or '--'}."
            ),
            message_tag=CommandLifecycleState.VALIDATING.value,
        )

        validation = self._validate_command(
            runtime=runtime,
            lease=lease,
            parsed_intent=parsed_intent,
            requested_mode=command.mode,
        )
        command.validation_result = validation
        command.plan_summary = {
            "normalizedIntent": parsed_intent["normalizedText"],
            "parsedAction": parsed_intent["action"],
            "targetSummary": parsed_intent["targetSummary"],
            "requiresConfirmation": validation["requiresConfirmation"],
            "normalizedCommand": parsed_intent.get("normalizedCommand"),
        }
        command.plan_fingerprint = validation["planFingerprint"]
        command.risk_level = (
            CommandRiskLevel(validation["riskLevel"])
            if validation["riskLevel"] is not None
            else None
        )
        self._audit.upsert_command(command)
        self._audit.record_runtime_event(
            system_state=runtime.system_state,
            session_id=session_id,
            operator_id=operator_id,
            command_id=command.command_id,
            message="validation result recorded",
            payload=validation,
        )
        if validation["accepted"]:
            self._trace(
                "validation.accepted",
                command_id=command_id,
                correlation_id=correlation_id,
                risk_level=validation.get("riskLevel"),
                confirmation_reasons=validation.get("confirmationReasons"),
                plan_fingerprint=validation.get("planFingerprint"),
            )
        else:
            self._trace(
                "validation.rejected",
                command_id=command_id,
                correlation_id=correlation_id,
                blocking_reasons=validation.get("blockingReasons"),
                risk_level=validation.get("riskLevel"),
            )
        if not validation["accepted"]:
            reject_reason = "; ".join(validation["blockingReasons"]) or "validation failed"
            command.reject_reason = reject_reason
            self._reject_command(
                command,
                reason=reject_reason,
                runtime_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
            )
            return self._command_response(session_id, operator_id, command, accepted=False, reason=reject_reason)

        if self._is_get_pose_command(command):
            return self._execute_get_pose_query(
                command=command,
                session_id=session_id,
                operator_id=operator_id,
                runtime=runtime,
            )

        command.confirmation_expires_at = utcnow() + self._confirmation_window
        self._transition_command(
            command,
            next_state=CommandLifecycleState.NEEDS_CONFIRMATION,
            reason="validated plan now requires operator confirmation",
            runtime_state=runtime.system_state,
            payload={
                "planFingerprint": command.plan_fingerprint,
                "reviewExpiresAt": command.confirmation_expires_at.isoformat(),
                "riskLevel": validation["riskLevel"],
                "confirmationReasons": validation["confirmationReasons"],
            },
            message_text=(
                "Step 3/6 NEEDS_CONFIRMATION: validation passed. "
                "Review the normalized plan and confirm before execution boundary handoff."
            ),
            message_tag=CommandLifecycleState.NEEDS_CONFIRMATION.value,
        )
        if (
            self._sim_auto_confirm
            and command.mode == RuntimeMode.SIM
            and command.plan_fingerprint is not None
        ):
            return self._confirm_command_internal(
                command=command,
                lease=lease,
                session_id=session_id,
                operator_id=operator_id,
                plan_fingerprint=command.plan_fingerprint,
            )
        return self._command_response(session_id, operator_id, command, accepted=True, reason=None)

    def _submit_sequence(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease: Any,
        runtime: RuntimeSnapshot,
        requested_mode: RuntimeMode,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        sequence_segments: list[str],
        prepared_sequence: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        sequence_id = str(uuid4())
        correlation_id = str(uuid4())
        sequence = CommandRecord(
            command_id=sequence_id,
            command_kind=CommandKind.SEQUENCE,
            session_id=session_id,
            operator_id=operator_id,
            raw_text=raw_text,
            lifecycle_state=CommandLifecycleState.RECEIVED,
            summary_label=(raw_text or "structured sequence")[:80],
            mode=requested_mode,
            created_at=utcnow(),
            intent_source="structured" if structured_intent is not None else "text",
            correlation_id=correlation_id,
            structured_intent=structured_intent,
        )
        with self._lock:
            self._commands[sequence_id] = sequence
            self._active_sequence_id = sequence_id
            self._active_command_id = None
        self._ros.submit_text_for_review(
            raw_text=raw_text or json.dumps(structured_intent, separators=(",", ":"), ensure_ascii=True),
            session_id=session_id,
            operator_id=operator_id,
            command_id=sequence_id,
        )
        self._append_message(origin="operator", text=raw_text or "Submitted structured sequence.", command_id=sequence_id)
        self._audit.upsert_command(sequence)
        self._audit.record_transition(
            command_id=sequence_id,
            session_id=session_id,
            operator_id=operator_id,
            from_state=None,
            to_state=CommandLifecycleState.RECEIVED,
            runtime_state=runtime.system_state,
            reason="sequence received from HMI v2 intent endpoint",
            payload={"mode": sequence.mode.value, "intentSource": sequence.intent_source, "kind": sequence.command_kind.value},
        )
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.PARSING,
            reason="supervisor parsing sequence intent",
            runtime_state=runtime.system_state,
            message_text="Step 1/6 PARSING: received multi-step sequence intent.",
            message_tag=CommandLifecycleState.PARSING.value,
        )
        if prepared_sequence is None:
            parsed_steps, diagnostics, parse_error, route_metadata = self._parse_sequence_steps(
                raw_text=raw_text,
                structured_intent=structured_intent,
                mode=sequence.mode,
                sequence_segments=sequence_segments,
            )
        else:
            parsed_steps = prepared_sequence.get("parsed_steps")
            diagnostics = list(prepared_sequence.get("diagnostics") or [])
            parse_error = prepared_sequence.get("parse_error")
            route_metadata = prepared_sequence.get("route_metadata")
            if prepared_sequence.get("structured_intent") is not None:
                sequence.structured_intent = prepared_sequence["structured_intent"]

        sequence.summary_label = self._sequence_summary_label(
            parsed_steps=parsed_steps or [],
            route_metadata=route_metadata,
            raw_text=raw_text,
            structured_intent=sequence.structured_intent,
        )
        sequence.plan_summary = self._sequence_plan_summary(
            raw_text=raw_text,
            structured_intent=sequence.structured_intent,
            parsed_steps=parsed_steps or [],
            diagnostics=diagnostics,
            route_metadata=route_metadata,
            requires_confirmation=parse_error is None,
        )
        if parsed_steps is None:
            sequence.reject_reason = parse_error
            sequence.validation_result = self._build_validation_result(
                runtime=runtime,
                lease=lease,
                parsed_intent=None,
                blocking_reasons=[parse_error or "sequence parsing failed"],
                risk_level=None,
                requires_confirmation=False,
            )
            self._reject_top_level_record(
                sequence,
                reason=parse_error or "sequence parsing failed",
                runtime_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
            )
            return self._sequence_response(session_id, operator_id, sequence, accepted=False, reason=parse_error)

        sequence.sequence_step_count = len(parsed_steps)
        sequence.current_step_index = 0
        sequence.sequence_diagnostics = list(diagnostics)
        child_commands: list[CommandRecord] = []
        for step_index, parsed_step in enumerate(parsed_steps):
            child_command = CommandRecord(
                command_id=str(uuid4()),
                session_id=session_id,
                operator_id=operator_id,
                raw_text=(sequence_segments[step_index] if step_index < len(sequence_segments) else parsed_step["targetSummary"]),
                lifecycle_state=CommandLifecycleState.RECEIVED,
                summary_label=parsed_step["targetSummary"],
                mode=sequence.mode,
                created_at=utcnow(),
                intent_source=sequence.intent_source,
                correlation_id=f"{correlation_id}:{step_index + 1}",
                parsed_intent=parsed_step,
                planner_used=self._planner_for_intent(parsed_step),
                frame_used=self._frame_for_intent(parsed_step),
                parent_sequence_id=sequence_id,
                sequence_step_index=step_index,
                sequence_step_count=len(parsed_steps),
            )
            child_commands.append(child_command)
            sequence.child_command_ids.append(child_command.command_id)
            self._commands[child_command.command_id] = child_command
            self._audit.upsert_command(child_command)
            self._audit.record_transition(
                command_id=child_command.command_id,
                session_id=session_id,
                operator_id=operator_id,
                from_state=None,
                to_state=CommandLifecycleState.RECEIVED,
                runtime_state=runtime.system_state,
                reason="sequence step received from HMI v2 intent endpoint",
                payload={"parentSequenceId": sequence_id, "stepIndex": step_index, "stepCount": len(parsed_steps)},
            )
            self._transition_command(
                child_command,
                next_state=CommandLifecycleState.PARSING,
                reason="supervisor parsing sequence step intent",
                runtime_state=runtime.system_state,
            )
            self._transition_command(
                child_command,
                next_state=CommandLifecycleState.VALIDATING,
                reason="supervisor validating parsed sequence step",
                runtime_state=runtime.system_state,
            )

        self._active_command_id = child_commands[0].command_id if child_commands else None
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.VALIDATING,
            reason="supervisor validating parsed sequence",
            runtime_state=runtime.system_state,
            payload={"stepCount": len(parsed_steps)},
            message_text=f"Step 2/6 VALIDATING: {len(parsed_steps)} ordered steps accepted for sequence validation.",
            message_tag=CommandLifecycleState.VALIDATING.value,
        )
        validation = self._validate_sequence(
            runtime=runtime,
            lease=lease,
            parsed_steps=parsed_steps,
            requested_mode=sequence.mode,
            diagnostics=diagnostics,
        )
        sequence.validation_result = validation
        sequence.plan_fingerprint = validation["planFingerprint"]
        if validation["riskLevel"] is not None:
            sequence.risk_level = CommandRiskLevel(validation["riskLevel"])
        for child_command in child_commands:
            child_validation = self._validate_command(
                runtime=runtime,
                lease=lease,
                parsed_intent=child_command.parsed_intent,
                requested_mode=child_command.mode,
            )
            child_command.validation_result = child_validation
            child_command.plan_fingerprint = child_validation["planFingerprint"]
            if child_validation["riskLevel"] is not None:
                child_command.risk_level = CommandRiskLevel(child_validation["riskLevel"])
            if not child_validation["accepted"]:
                validation["accepted"] = False
                validation["blockingReasons"] = list(dict.fromkeys(validation["blockingReasons"] + child_validation["blockingReasons"]))
            self._audit.upsert_command(child_command)
        self._audit.upsert_command(sequence)
        if not validation["accepted"]:
            reject_reason = "; ".join(validation["blockingReasons"]) or "sequence validation failed"
            sequence.reject_reason = reject_reason
            self._reject_top_level_record(
                sequence,
                reason=reject_reason,
                runtime_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
            )
            return self._sequence_response(session_id, operator_id, sequence, accepted=False, reason=reject_reason)

        sequence.confirmation_expires_at = utcnow() + self._confirmation_window
        for child_command in child_commands:
            child_command.confirmation_expires_at = sequence.confirmation_expires_at
            self._transition_command(
                child_command,
                next_state=CommandLifecycleState.NEEDS_CONFIRMATION,
                reason="sequence-level validation passed; waiting for parent confirmation",
                runtime_state=runtime.system_state,
            )
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.NEEDS_CONFIRMATION,
            reason="validated sequence now requires operator confirmation",
            runtime_state=runtime.system_state,
            payload={
                "planFingerprint": sequence.plan_fingerprint,
                "reviewExpiresAt": sequence.confirmation_expires_at.isoformat(),
                "riskLevel": validation["riskLevel"],
                "confirmationReasons": validation["confirmationReasons"],
            },
            message_text=(
                "Step 3/6 NEEDS_CONFIRMATION: sequence validation passed. "
                "Review the ordered steps and confirm once before execution."
            ),
            message_tag=CommandLifecycleState.NEEDS_CONFIRMATION.value,
        )
        if self._sim_auto_confirm and sequence.mode == RuntimeMode.SIM and sequence.plan_fingerprint is not None:
            return self.confirm_sequence(
                session_id=session_id,
                operator_id=operator_id,
                lease_token=lease.lease_token,
                sequence_id=sequence.command_id,
                plan_fingerprint=sequence.plan_fingerprint,
            )
        return self._sequence_response(session_id, operator_id, sequence, accepted=True, reason=None)

    def get_command(self, command_id: str) -> dict[str, Any]:
        self._expire_pending_confirmations()
        command = self._commands.get(command_id)
        if command is not None:
            if command.command_kind != CommandKind.COMMAND:
                raise NotFoundError("command not found")
            return self._serialize_command(command)

        detail = self._audit.get_command_detail(command_id)
        if detail is None:
            raise NotFoundError("command not found")
        if (detail["command"].get("command_kind") or CommandKind.COMMAND.value) != CommandKind.COMMAND.value:
            raise NotFoundError("command not found")
        return self._serialize_audited_command(detail["command"])

    def get_sequence(self, sequence_id: str) -> dict[str, Any]:
        self._expire_pending_confirmations()
        sequence = self._commands.get(sequence_id)
        if sequence is not None:
            if sequence.command_kind != CommandKind.SEQUENCE:
                raise NotFoundError("sequence not found")
            return self._serialize_sequence(sequence)

        detail = self._audit.get_command_detail(sequence_id)
        if detail is None:
            raise NotFoundError("sequence not found")
        if (detail["command"].get("command_kind") or CommandKind.COMMAND.value) != CommandKind.SEQUENCE.value:
            raise NotFoundError("sequence not found")
        steps = [
            self._serialize_audited_command(row)
            for row in self._audit.list_sequence_children(sequence_id)
        ]
        return self._serialize_audited_sequence(detail["command"], steps)

    def list_commands(
        self,
        *,
        session_id: str | None = None,
        operator_id: str | None = None,
        final_state: str | None = None,
        created_from: str | None = None,
        created_to: str | None = None,
        limit: int = 50,
    ) -> dict[str, Any]:
        self._expire_pending_confirmations()
        items = self._audit.list_commands(
            session_id=session_id,
            operator_id=operator_id,
            final_state=final_state,
            created_from=created_from,
            created_to=created_to,
            command_kind=CommandKind.COMMAND.value,
            parent_sequence_id="",
            limit=limit,
        )
        return {"items": [self._serialize_replay_item(item) for item in items]}

    def confirm_command(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        command_id: str,
        plan_fingerprint: str,
    ) -> dict[str, Any]:
        self._expire_pending_confirmations()
        lease = self._assert_controller(session_id, operator_id, lease_token)
        command = self._require_owned_command(command_id, session_id, operator_id)
        if command.command_kind != CommandKind.COMMAND or command.parent_sequence_id is not None:
            raise ConflictError("sequence steps are controlled by the parent sequence")
        self._trace(
            "confirmation.requested",
            command_id=command.command_id,
            correlation_id=command.correlation_id,
            operator_id=operator_id,
            plan_fingerprint=plan_fingerprint,
        )
        if command.lifecycle_state != CommandLifecycleState.NEEDS_CONFIRMATION:
            raise ConflictError("command is not waiting for confirmation")
        if command.confirmation_expires_at is None or command.confirmation_expires_at <= utcnow():
            self._expire_command(command, reason="confirmation window expired before operator confirmation")
            raise ConflictError("confirmation window expired")
        if plan_fingerprint != command.plan_fingerprint:
            raise ConflictError("plan fingerprint mismatch")

        return self._confirm_command_internal(
            command=command,
            lease=lease,
            session_id=session_id,
            operator_id=operator_id,
            plan_fingerprint=plan_fingerprint,
        )

    def confirm_sequence(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        sequence_id: str,
        plan_fingerprint: str,
    ) -> dict[str, Any]:
        self._expire_pending_confirmations()
        lease = self._assert_controller(session_id, operator_id, lease_token)
        sequence = self._require_owned_sequence(sequence_id, session_id, operator_id)
        if sequence.lifecycle_state != CommandLifecycleState.NEEDS_CONFIRMATION:
            raise ConflictError("sequence is not waiting for confirmation")
        if sequence.confirmation_expires_at is None or sequence.confirmation_expires_at <= utcnow():
            self._expire_top_level_record(sequence, reason="confirmation window expired before operator confirmation")
            raise ConflictError("confirmation window expired")
        if plan_fingerprint != sequence.plan_fingerprint:
            raise ConflictError("plan fingerprint mismatch")

        runtime = self._current_runtime()
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.CONFIRMED,
            reason="operator confirmed validated sequence plan",
            runtime_state=runtime.system_state,
            payload={"planFingerprint": plan_fingerprint},
            message_text=(
                "Step 4/6 CONFIRMED: operator confirmed the validated sequence fingerprint. "
                "Ordered execution is now allowed."
            ),
            message_tag=CommandLifecycleState.CONFIRMED.value,
        )
        sequence.confirm_at = utcnow()
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.EXECUTION_REQUESTED,
            reason="supervisor forwarding confirmed sequence to execution boundary",
            runtime_state=runtime.system_state,
            payload={
                "correlationId": sequence.correlation_id,
                "leaseId": lease.lease_id,
                "operatorId": operator_id,
                "stepCount": sequence.sequence_step_count,
            },
            message_text="Step 5/6 EXECUTION_REQUESTED: confirmed sequence queued for ordered execution.",
            message_tag=CommandLifecycleState.EXECUTION_REQUESTED.value,
        )
        sequence.execute_at = utcnow()
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.EXECUTING,
            reason="sequence execution started",
            runtime_state=runtime.system_state,
            payload={"currentStepIndex": sequence.current_step_index},
            message_text="Step 5/6 EXECUTING: sequence steps are being dispatched in order.",
            message_tag=CommandLifecycleState.EXECUTING.value,
        )

        for child_id in sequence.child_command_ids:
            child_command = self._commands[child_id]
            sequence.current_step_index = child_command.sequence_step_index
            self._active_command_id = child_id
            self._audit.upsert_command(sequence)
            self._emit_sequence_event(sequence, None)
            self._confirm_command_internal(
                command=child_command,
                lease=lease,
                session_id=session_id,
                operator_id=operator_id,
                plan_fingerprint=child_command.plan_fingerprint or "",
            )
            if child_command.final_state != CommandLifecycleState.SUCCEEDED:
                sequence.final_state = child_command.final_state
                sequence.reject_reason = child_command.reject_reason
                sequence.manual_recovery_required = any(
                    self._commands[step_id].parsed_intent
                    and str(self._commands[step_id].parsed_intent.get("action") or "").upper() == "IO_SET"
                    for step_id in sequence.child_command_ids[: (child_command.sequence_step_index or 0) + 1]
                )
                sequence.execution_result = {
                    "accepted": False,
                    "adapter": "workspace_ros_adapter",
                    "status": (child_command.final_state.value.lower() if child_command.final_state else "failed"),
                    "summary": child_command.reject_reason or "sequence step failed",
                    "dispatchedToRos": bool(
                        child_command.execution_result and child_command.execution_result.get("dispatchedToRos")
                    ),
                }
                self._transition_top_level_record(
                    sequence,
                    next_state=child_command.final_state or CommandLifecycleState.FAILED,
                    reason=child_command.reject_reason or "sequence step failed",
                    runtime_state=self._current_runtime().system_state,
                    payload={"failedCommandId": child_command.command_id, "failedStepIndex": child_command.sequence_step_index},
                    message_text=f"Step 6/6 RESULT: {child_command.reject_reason or 'Sequence step failed.'}",
                    message_tag=(child_command.final_state.value if child_command.final_state else CommandLifecycleState.FAILED.value),
                )
                return self._sequence_response(session_id, operator_id, sequence, accepted=False, reason=sequence.reject_reason)

        if sequence.sequence_step_count:
            sequence.current_step_index = sequence.sequence_step_count - 1
        sequence.final_state = CommandLifecycleState.SUCCEEDED
        sequence.reject_reason = None
        sequence.execution_result = {
            "accepted": True,
            "adapter": "workspace_ros_adapter",
            "status": "succeeded",
            "summary": "Sequence executed successfully.",
            "dispatchedToRos": True,
        }
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.SUCCEEDED,
            reason="all sequence steps completed successfully",
            runtime_state=self._current_runtime().system_state,
            payload={"stepCount": sequence.sequence_step_count},
            message_text="Step 6/6 RESULT: Sequence executed successfully.",
            message_tag=CommandLifecycleState.SUCCEEDED.value,
        )
        return self._sequence_response(session_id, operator_id, sequence, accepted=True, reason=None)

    def cancel_command(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        command_id: str,
        reason: str | None = None,
    ) -> dict[str, Any]:
        self._expire_pending_confirmations()
        self._assert_controller(session_id, operator_id, lease_token)
        command = self._require_owned_command(command_id, session_id, operator_id)
        if command.parent_sequence_id is not None:
            raise ConflictError("sequence steps are controlled by the parent sequence")
        if is_terminal_command_state(command.lifecycle_state):
            return self._command_response(
                session_id,
                operator_id,
                command,
                accepted=True,
                reason="command already terminal",
            )

        if command.lifecycle_state in {
            CommandLifecycleState.EXECUTION_REQUESTED,
            CommandLifecycleState.EXECUTING,
        }:
            ok, adapter_reason = self._ros.abort_command(command_id=command.command_id)
            if not ok:
                raise ConflictError(adapter_reason)

        runtime = self._current_runtime()
        command.final_state = CommandLifecycleState.CANCELLED
        command.reject_reason = reason
        self._transition_command(
            command,
            next_state=CommandLifecycleState.CANCELLED,
            reason=reason or "operator cancelled command",
            runtime_state=runtime.system_state,
            message_text=f"Step 6/6 RESULT: {reason or 'Command cancelled.'}",
            message_tag=CommandLifecycleState.CANCELLED.value,
        )
        self._trace(
            "terminal.cancelled",
            command_id=command.command_id,
            correlation_id=command.correlation_id,
            reason=reason or "operator cancelled command",
        )
        self._audit.upsert_command(command)
        self._broadcast_replay_update()
        return self._command_response(session_id, operator_id, command, accepted=True, reason=reason)

    def cancel_sequence(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        sequence_id: str,
        reason: str | None = None,
    ) -> dict[str, Any]:
        self._expire_pending_confirmations()
        self._assert_controller(session_id, operator_id, lease_token)
        sequence = self._require_owned_sequence(sequence_id, session_id, operator_id)
        if is_terminal_command_state(sequence.lifecycle_state):
            return self._sequence_response(session_id, operator_id, sequence, accepted=True, reason="sequence already terminal")

        current_child = self._commands.get(self._active_command_id) if self._active_command_id else None
        if current_child is not None and current_child.parent_sequence_id == sequence.command_id and current_child.lifecycle_state in {
            CommandLifecycleState.EXECUTION_REQUESTED,
            CommandLifecycleState.EXECUTING,
        }:
            ok, adapter_reason = self._ros.abort_command(command_id=current_child.command_id)
            if not ok:
                raise ConflictError(adapter_reason)

        runtime = self._current_runtime()
        for child_id in sequence.child_command_ids:
            child = self._commands[child_id]
            if is_terminal_command_state(child.lifecycle_state):
                continue
            child.final_state = CommandLifecycleState.CANCELLED
            child.reject_reason = reason
            self._transition_command(
                child,
                next_state=CommandLifecycleState.CANCELLED,
                reason=reason or "operator cancelled parent sequence",
                runtime_state=runtime.system_state,
            )
        sequence.final_state = CommandLifecycleState.CANCELLED
        sequence.reject_reason = reason
        self._transition_top_level_record(
            sequence,
            next_state=CommandLifecycleState.CANCELLED,
            reason=reason or "operator cancelled parent sequence",
            runtime_state=runtime.system_state,
            message_text=f"Step 6/6 RESULT: {reason or 'Sequence cancelled.'}",
            message_tag=CommandLifecycleState.CANCELLED.value,
        )
        return self._sequence_response(session_id, operator_id, sequence, accepted=True, reason=reason)

    def list_replay(self, **filters: Any) -> dict[str, Any]:
        self._expire_pending_confirmations()
        items = self._audit.list_commands(top_level_only=True, **filters)
        return {"items": [self._serialize_replay_item(item) for item in items]}

    def replay_detail(self, command_id: str) -> dict[str, Any]:
        detail = self._audit.get_command_detail(command_id)
        if detail is None:
            raise NotFoundError("command not found")
        kind = detail["command"].get("command_kind") or CommandKind.COMMAND.value
        if kind == CommandKind.SEQUENCE.value:
            steps = [
                self._serialize_audited_command(row)
                for row in self._audit.list_sequence_children(command_id)
            ]
            return {
                "jobType": CommandKind.SEQUENCE.value,
                "command": None,
                "sequence": self._serialize_audited_sequence(detail["command"], steps),
                "timeline": [self._serialize_timeline_row(row) for row in detail["timeline"]],
                "runtimeEvents": [self._serialize_runtime_row(row) for row in detail["runtime_events"]],
            }
        return {
            "jobType": CommandKind.COMMAND.value,
            "command": self._serialize_audited_command(detail["command"]),
            "sequence": None,
            "timeline": [self._serialize_timeline_row(row) for row in detail["timeline"]],
            "runtimeEvents": [self._serialize_runtime_row(row) for row in detail["runtime_events"]],
        }

    def _assert_controller(
        self,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
    ) -> Any:
        try:
            return self._session_lock.assert_controller(session_id, operator_id, lease_token)
        except LeaseNotOwnedError as exc:
            raise ForbiddenActionError(str(exc)) from exc

    def _assert_no_active_job(self) -> None:
        active_sequence = self._commands.get(self._active_sequence_id) if self._active_sequence_id else None
        if active_sequence is not None and not is_terminal_command_state(active_sequence.lifecycle_state):
            raise ConflictError("another sequence is already pending or executing")
        active_command = self._commands.get(self._active_command_id) if self._active_command_id else None
        if (
            active_command is not None
            and active_command.command_kind == CommandKind.COMMAND
            and active_command.parent_sequence_id is None
            and not is_terminal_command_state(active_command.lifecycle_state)
        ):
            raise ConflictError("another command is already pending or executing")

    def _require_owned_command(
        self,
        command_id: str,
        session_id: str,
        operator_id: str,
    ) -> CommandRecord:
        command = self._commands.get(command_id)
        if command is None:
            raise NotFoundError("command not found")
        if command.session_id != session_id or command.operator_id != operator_id:
            raise ForbiddenActionError("command belongs to another session")
        return command

    def _require_owned_sequence(
        self,
        sequence_id: str,
        session_id: str,
        operator_id: str,
    ) -> CommandRecord:
        sequence = self._require_owned_command(sequence_id, session_id, operator_id)
        if sequence.command_kind != CommandKind.SEQUENCE:
            raise NotFoundError("sequence not found")
        return sequence

    def _current_runtime(self) -> RuntimeSnapshot:
        return self._ros.read_runtime_snapshot()

    def _current_joints(self) -> list[JointPosition]:
        return self._ros.read_joint_positions()

    def _read_source_statuses(self) -> list[Any]:
        reader = getattr(self._ros, "read_source_statuses", None)
        if callable(reader):
            return reader()
        return []

    def _transition_command(
        self,
        command: CommandRecord,
        *,
        next_state: CommandLifecycleState,
        reason: str,
        runtime_state: SystemRuntimeState,
        payload: dict[str, Any] | None = None,
        message_text: str | None = None,
        message_tag: str | None = None,
    ) -> None:
        previous = command.lifecycle_state
        ensure_command_transition(previous, next_state)
        command.lifecycle_state = next_state
        self._audit.upsert_command(command)
        self._audit.record_transition(
            command_id=command.command_id,
            session_id=command.session_id,
            operator_id=command.operator_id,
            from_state=previous,
            to_state=next_state,
            runtime_state=runtime_state,
            reason=reason,
            payload=payload,
        )
        messages: list[ChatMessage] | None = None
        if message_text is not None:
            messages = [
                self._append_message(
                    origin="system",
                    text=message_text,
                    command_id=command.command_id,
                    tag=message_tag,
                )
            ]
        self._emit_command_event(command, messages)

    def _transition_top_level_record(
        self,
        command: CommandRecord,
        *,
        next_state: CommandLifecycleState,
        reason: str,
        runtime_state: SystemRuntimeState,
        payload: dict[str, Any] | None = None,
        message_text: str | None = None,
        message_tag: str | None = None,
    ) -> None:
        previous = command.lifecycle_state
        ensure_command_transition(previous, next_state)
        command.lifecycle_state = next_state
        self._audit.upsert_command(command)
        self._audit.record_transition(
            command_id=command.command_id,
            session_id=command.session_id,
            operator_id=command.operator_id,
            from_state=previous,
            to_state=next_state,
            runtime_state=runtime_state,
            reason=reason,
            payload=payload,
        )
        messages: list[ChatMessage] | None = None
        if message_text is not None:
            messages = [
                self._append_message(
                    origin="system",
                    text=message_text,
                    command_id=command.command_id,
                    tag=message_tag,
                )
            ]
        if command.command_kind == CommandKind.SEQUENCE:
            self._emit_sequence_event(command, messages)
            return
        self._emit_command_event(command, messages)

    def _validate_sequence(
        self,
        *,
        runtime: RuntimeSnapshot,
        lease: Any,
        parsed_steps: list[dict[str, Any]],
        requested_mode: RuntimeMode,
        diagnostics: list[str],
    ) -> dict[str, Any]:
        source_statuses = self._read_source_statuses()
        critical_sources = [source for source in source_statuses if getattr(source, "active", False)]
        optional_sources = [
            source
            for source in source_statuses
            if not getattr(source, "active", False) and source.name not in {"llm_debug", "llm_command"}
        ]
        event_driven_sources = [source for source in source_statuses if source.name in {"llm_debug", "llm_command"}]
        blocking_reasons: list[str] = []
        confirmation_reasons = [
            "HMI v2 requires explicit operator confirmation before a validated sequence may cross the execution boundary."
        ]
        hardware_gate = self._hardware_gate_evaluator.evaluate()
        preflight = self._execution_preflight(requested_mode=requested_mode)
        if requested_mode not in {RuntimeMode.SIM, RuntimeMode.HARDWARE}:
            blocking_reasons.append(f"runtime mode {requested_mode.value} is not command-capable for HMI v2.")
        if runtime.mode != requested_mode:
            blocking_reasons.append(
                f"requested mode {requested_mode.value} does not match runtime mode {runtime.mode.value}."
            )
        if runtime.system_state in {
            SystemRuntimeState.FAULT,
            SystemRuntimeState.ESTOP,
            SystemRuntimeState.LOST_CONN,
            SystemRuntimeState.SAFETY_BLOCKED,
        }:
            blocking_reasons.append(
                f"runtime state {runtime.system_state.value} is hard-blocking for command-capable actions"
            )
        if requested_mode == RuntimeMode.HARDWARE and not hardware_gate.unlocked:
            blocking_reasons.extend(hardware_gate.reasons)
        stale_sources = [
            source.name
            for source in critical_sources
            if source.freshness_state.value != "fresh"
        ]
        if stale_sources:
            blocking_reasons.append(
                "freshness-critical telemetry is stale or unavailable: " + ", ".join(stale_sources)
            )
        if not preflight.get("accepted", True):
            blocking_reasons.extend([str(reason) for reason in (preflight.get("reasons") or [])])

        risk_order = {
            CommandRiskLevel.LOW: 0,
            CommandRiskLevel.MEDIUM: 1,
            CommandRiskLevel.HIGH: 2,
            CommandRiskLevel.CRITICAL: 3,
        }
        risk_level: CommandRiskLevel | None = None
        for parsed_step in parsed_steps:
            step_risk = self._assess_risk(parsed_step)
            if step_risk is None:
                continue
            if risk_level is None or risk_order[step_risk] > risk_order[risk_level]:
                risk_level = step_risk
        if risk_level in {CommandRiskLevel.HIGH, CommandRiskLevel.CRITICAL}:
            confirmation_reasons.append(
                f"Sequence risk assessment is {risk_level.value}; high-risk sequences must stay behind confirmation."
            )
        confirmation_reasons.append(f"Sequence contains {len(parsed_steps)} ordered steps.")
        confirmation_reasons.extend(list(diagnostics))
        plan_fingerprint = None
        if not blocking_reasons:
            stable_blob = json.dumps(
                {
                    "parsedSteps": parsed_steps,
                    "leaseId": lease.lease_id,
                    "runtimeMode": requested_mode.value,
                },
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=True,
            )
            import hashlib

            plan_fingerprint = hashlib.sha256(stable_blob.encode("ascii")).hexdigest()
        return {
            "accepted": not blocking_reasons,
            "leaseValid": True,
            "runtimeAllowed": runtime.system_state
            not in {
                SystemRuntimeState.FAULT,
                SystemRuntimeState.ESTOP,
                SystemRuntimeState.LOST_CONN,
                SystemRuntimeState.SAFETY_BLOCKED,
            },
            "telemetryFresh": not stale_sources,
            "requiresConfirmation": True,
            "riskLevel": risk_level.value if risk_level else None,
            "blockingReasons": list(dict.fromkeys(blocking_reasons)),
            "confirmationReasons": list(dict.fromkeys(confirmation_reasons)),
            "planFingerprint": plan_fingerprint,
            "executionAllowedNow": False,
            "criticalSources": [self._source_status_view(source) for source in critical_sources],
            "optionalSources": [self._source_status_view(source) for source in optional_sources],
            "eventDrivenSources": [self._source_status_view(source) for source in event_driven_sources],
            "hardwareGate": hardware_gate.to_dict(),
            "preflight": preflight,
        }

    def _reject_command(
        self,
        command: CommandRecord,
        *,
        reason: str,
        runtime_state: SystemRuntimeState,
        session_id: str,
        operator_id: str,
    ) -> None:
        command.final_state = CommandLifecycleState.REJECTED
        command.reject_reason = reason
        self._transition_command(
            command,
            next_state=CommandLifecycleState.REJECTED,
            reason=reason,
            runtime_state=runtime_state,
            payload=command.validation_result,
            message_text=f"Step 6/6 RESULT: {reason}",
            message_tag=CommandLifecycleState.REJECTED.value,
        )
        self._trace(
            "terminal.rejected",
            command_id=command.command_id,
            correlation_id=command.correlation_id,
            reason=reason,
        )
        self._audit.upsert_command(command)
        self._audit.record_runtime_event(
            system_state=runtime_state,
            session_id=session_id,
            operator_id=operator_id,
            command_id=command.command_id,
            message="command rejected",
            payload={"reason": reason},
        )
        self._broadcast_replay_update()

    def _reject_top_level_record(
        self,
        command: CommandRecord,
        *,
        reason: str,
        runtime_state: SystemRuntimeState,
        session_id: str,
        operator_id: str,
    ) -> None:
        command.final_state = CommandLifecycleState.REJECTED
        command.reject_reason = reason
        self._transition_top_level_record(
            command,
            next_state=CommandLifecycleState.REJECTED,
            reason=reason,
            runtime_state=runtime_state,
            payload=command.validation_result,
            message_text=f"Step 6/6 RESULT: {reason}",
            message_tag=CommandLifecycleState.REJECTED.value,
        )
        self._audit.upsert_command(command)
        self._audit.record_runtime_event(
            system_state=runtime_state,
            session_id=session_id,
            operator_id=operator_id,
            command_id=command.command_id,
            message="sequence rejected" if command.command_kind == CommandKind.SEQUENCE else "command rejected",
            payload={"reason": reason},
        )
        self._broadcast_replay_update()

    def _expire_pending_confirmations(self) -> None:
        now = utcnow()
        with self._lock:
            expiring = [
                command
                for command in self._commands.values()
                if command.lifecycle_state == CommandLifecycleState.NEEDS_CONFIRMATION
                and command.parent_sequence_id is None
                and command.confirmation_expires_at is not None
                and command.confirmation_expires_at <= now
            ]
        for command in expiring:
            if command.command_kind == CommandKind.SEQUENCE:
                self._expire_top_level_record(command, reason="confirmation window expired")
            else:
                self._expire_command(command, reason="confirmation window expired")

    def _expire_command(self, command: CommandRecord, *, reason: str) -> None:
        runtime_state = self._current_runtime().system_state
        command.final_state = CommandLifecycleState.EXPIRED
        command.reject_reason = reason
        self._transition_command(
            command,
            next_state=CommandLifecycleState.EXPIRED,
            reason=reason,
            runtime_state=runtime_state,
            message_text=f"Step 6/6 RESULT: {reason}.",
            message_tag=CommandLifecycleState.EXPIRED.value,
        )
        self._trace(
            "terminal.expired",
            command_id=command.command_id,
            correlation_id=command.correlation_id,
            reason=reason,
        )
        self._audit.upsert_command(command)
        self._broadcast_replay_update()

    def _expire_top_level_record(self, command: CommandRecord, *, reason: str) -> None:
        runtime_state = self._current_runtime().system_state
        command.final_state = CommandLifecycleState.EXPIRED
        command.reject_reason = reason
        self._transition_top_level_record(
            command,
            next_state=CommandLifecycleState.EXPIRED,
            reason=reason,
            runtime_state=runtime_state,
            message_text=f"Step 6/6 RESULT: {reason}.",
            message_tag=CommandLifecycleState.EXPIRED.value,
        )
        self._audit.upsert_command(command)
        self._broadcast_replay_update()
