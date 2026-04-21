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


class SupervisorService(SupervisorViewsMixin, SupervisorValidationMixin):
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
        replay_items = self._audit.list_commands(limit=25)
        return {
            "capabilities": self._bridge_capabilities().to_dict(),
            "lease": self._serialize_lease_view(session_id, operator_id),
            "messages": [message.to_dict() for message in self._messages],
            "activeCommand": self._serialize_command(active_command),
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
        command_id = str(uuid4())
        correlation_id = str(uuid4())
        command = CommandRecord(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            raw_text=raw_text,
            lifecycle_state=CommandLifecycleState.RECEIVED,
            summary_label=(raw_text or "structured command")[:80],
            mode=RuntimeMode(mode) if mode in RuntimeMode._value2member_map_ else runtime.mode,
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
        if command.mode == RuntimeMode.HARDWARE:
            self._audit.record_runtime_event(
                system_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
                command_id=command.command_id,
                message="hardware gate evaluation",
                payload=validation.get("hardwareGate"),
            )
            self._audit.record_runtime_event(
                system_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
                command_id=command.command_id,
                message="hardware preflight result",
                payload=validation.get("preflight"),
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
        self._trace(
            "confirmation.required",
            command_id=command_id,
            correlation_id=correlation_id,
            plan_fingerprint=command.plan_fingerprint,
            review_expires_at=command.confirmation_expires_at,
        )
        if (
            self._sim_auto_confirm
            and command.mode == RuntimeMode.SIM
            and command.plan_fingerprint is not None
        ):
            self._trace(
                "confirmation.autorun_requested",
                command_id=command_id,
                correlation_id=correlation_id,
                plan_fingerprint=command.plan_fingerprint,
            )
            return self._confirm_command_internal(
                command=command,
                lease=lease,
                session_id=session_id,
                operator_id=operator_id,
                plan_fingerprint=command.plan_fingerprint,
            )
        return self._command_response(session_id, operator_id, command, accepted=True, reason=None)

    def get_command(self, command_id: str) -> dict[str, Any]:
        self._expire_pending_confirmations()
        command = self._commands.get(command_id)
        if command is not None:
            return self._serialize_command(command)

        detail = self._audit.get_command_detail(command_id)
        if detail is None:
            raise NotFoundError("command not found")
        return self._serialize_audited_command(detail["command"])

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

    def _confirm_command_internal(
        self,
        *,
        command: CommandRecord,
        lease: Any,
        session_id: str,
        operator_id: str,
        plan_fingerprint: str,
    ) -> dict[str, Any]:
        runtime = self._current_runtime()
        confirm_gate = self._validate_command(
            runtime=runtime,
            lease=lease,
            parsed_intent=command.parsed_intent,
            requested_mode=command.mode,
        )
        if command.mode == RuntimeMode.HARDWARE:
            self._audit.record_runtime_event(
                system_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
                command_id=command.command_id,
                message="hardware gate evaluation",
                payload=confirm_gate.get("hardwareGate"),
            )
            self._audit.record_runtime_event(
                system_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
                command_id=command.command_id,
                message="hardware preflight result",
                payload=confirm_gate.get("preflight"),
            )
        if not confirm_gate["accepted"]:
            reason = "; ".join(confirm_gate["blockingReasons"]) or "confirmation gate rejected command"
            self._trace(
                "confirmation.rejected",
                command_id=command.command_id,
                correlation_id=command.correlation_id,
                reason=reason,
                blocking_reasons=confirm_gate["blockingReasons"],
            )
            command.validation_result = confirm_gate
            self._audit.upsert_command(command)
            self._emit_command_event(command, messages=None)
            return self._command_response(
                session_id,
                operator_id,
                command,
                accepted=False,
                reason=reason,
            )

        return self._execute_confirmed_command(
            command=command,
            lease=lease,
            session_id=session_id,
            operator_id=operator_id,
            plan_fingerprint=plan_fingerprint,
            runtime=runtime,
        )

    def _execute_confirmed_command(
        self,
        *,
        command: CommandRecord,
        lease: Any,
        session_id: str,
        operator_id: str,
        plan_fingerprint: str,
        runtime: RuntimeSnapshot,
    ) -> dict[str, Any]:
        self._transition_command(
            command,
            next_state=CommandLifecycleState.CONFIRMED,
            reason="operator confirmed validated plan",
            runtime_state=runtime.system_state,
            payload={"planFingerprint": plan_fingerprint},
            message_text=(
                "Step 4/6 CONFIRMED: operator confirmed the validated plan fingerprint. "
                "Execution handoff is now allowed."
            ),
            message_tag=CommandLifecycleState.CONFIRMED.value,
        )
        self._trace(
            "confirmation.accepted",
            command_id=command.command_id,
            correlation_id=command.correlation_id,
            plan_fingerprint=plan_fingerprint,
        )
        command.confirm_at = utcnow()
        self._audit.upsert_command(command)

        self._transition_command(
            command,
            next_state=CommandLifecycleState.EXECUTION_REQUESTED,
            reason="supervisor forwarding confirmed command to execution boundary",
            runtime_state=runtime.system_state,
            payload={
                "correlationId": command.correlation_id,
                "leaseId": lease.lease_id,
                "operatorId": operator_id,
            },
            message_text=(
                "Step 5/6 EXECUTION_REQUESTED: confirmed plan forwarded to the "
                "supervisor-owned execution boundary."
            ),
            message_tag=CommandLifecycleState.EXECUTION_REQUESTED.value,
        )
        self._trace(
            "execution.requested",
            command_id=command.command_id,
            correlation_id=command.correlation_id,
            lease_id=lease.lease_id,
            operator_id=operator_id,
        )
        command.execute_at = utcnow()

        execution_result = self._ros.confirm_command(
            command_id=command.command_id,
            plan_fingerprint=plan_fingerprint,
            operator_id=operator_id,
            session_id=session_id,
            lease_id=lease.lease_id,
            correlation_id=command.correlation_id or str(uuid4()),
            parsed_intent=command.parsed_intent,
            requested_mode=command.mode.value,
        )
        command.execution_result = execution_result
        self._audit.record_runtime_event(
            system_state=runtime.system_state,
            session_id=session_id,
            operator_id=operator_id,
            command_id=command.command_id,
            message="execution boundary response recorded",
            payload=execution_result,
        )
        self._trace(
            "execution.response",
            command_id=command.command_id,
            correlation_id=command.correlation_id,
            status=execution_result.get("status"),
            dispatched_to_ros=execution_result.get("dispatchedToRos"),
            summary=execution_result.get("summary"),
        )

        execution_status = str(execution_result.get("status") or "").lower()
        dispatched_to_ros = bool(execution_result.get("dispatchedToRos"))
        if dispatched_to_ros:
            self._trace(
                "execution.dispatched",
                command_id=command.command_id,
                correlation_id=command.correlation_id,
                summary=execution_result.get("summary"),
            )
            self._transition_command(
                command,
                next_state=CommandLifecycleState.EXECUTING,
                reason="execution adapter dispatched the validated request to ROS",
                runtime_state=runtime.system_state,
                payload=execution_result,
                message_text=(
                    "Step 5/6 EXECUTING: execution adapter dispatched the validated "
                    "request to ROS."
                ),
                message_tag=CommandLifecycleState.EXECUTING.value,
            )
            command.final_state = None

        if execution_status == "succeeded":
            command.final_state = CommandLifecycleState.SUCCEEDED
            command.reject_reason = None
            self._transition_command(
                command,
                next_state=CommandLifecycleState.SUCCEEDED,
                reason=execution_result.get("summary") or "execution completed successfully",
                runtime_state=runtime.system_state,
                payload=execution_result,
                message_text=(
                    f"Step 6/6 RESULT: "
                    f"{execution_result.get('summary') or 'Execution completed successfully.'}"
                ),
                message_tag=CommandLifecycleState.SUCCEEDED.value,
            )
            self._trace(
                "terminal.succeeded",
                command_id=command.command_id,
                correlation_id=command.correlation_id,
                reason=execution_result.get("summary") or "execution completed successfully",
            )
        elif execution_status == "cancelled":
            if not dispatched_to_ros:
                self._transition_command(
                    command,
                    next_state=CommandLifecycleState.EXECUTING,
                    reason="execution adapter reported a cancellation outcome",
                    runtime_state=runtime.system_state,
                    payload=execution_result,
                    message_text=(
                        "Step 5/6 EXECUTING: execution adapter reported a cancellation outcome."
                    ),
                    message_tag=CommandLifecycleState.EXECUTING.value,
                )
            command.final_state = CommandLifecycleState.CANCELLED
            command.reject_reason = execution_result.get("summary")
            self._transition_command(
                command,
                next_state=CommandLifecycleState.CANCELLED,
                reason=execution_result.get("summary") or "execution was cancelled",
                runtime_state=runtime.system_state,
                payload=execution_result,
                message_text=(
                    f"Step 6/6 RESULT: "
                    f"{execution_result.get('summary') or 'Execution was cancelled.'}"
                ),
                message_tag=CommandLifecycleState.CANCELLED.value,
            )
            self._trace(
                "terminal.cancelled",
                command_id=command.command_id,
                correlation_id=command.correlation_id,
                reason=execution_result.get("summary") or "execution was cancelled",
            )
        else:
            command.final_state = CommandLifecycleState.FAILED
            command.reject_reason = execution_result.get("summary")
            self._transition_command(
                command,
                next_state=CommandLifecycleState.FAILED,
                reason=execution_result.get("summary") or "execution boundary rejected request",
                runtime_state=runtime.system_state,
                payload=execution_result,
                message_text=(
                    f"Step 6/6 RESULT: "
                    f"{execution_result.get('summary') or 'Execution boundary rejected the request.'}"
                ),
                message_tag=CommandLifecycleState.FAILED.value,
            )
            self._trace(
                "terminal.failed",
                command_id=command.command_id,
                correlation_id=command.correlation_id,
                reason=execution_result.get("summary") or "execution boundary rejected request",
            )

        self._audit.upsert_command(command)
        self._broadcast_replay_update()
        accepted = command.final_state == CommandLifecycleState.SUCCEEDED
        return self._command_response(
            session_id,
            operator_id,
            command,
            accepted=accepted,
            reason=execution_result.get("summary") if not accepted else None,
        )

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

    def list_replay(self, **filters: Any) -> dict[str, Any]:
        return self.list_commands(**filters)

    def replay_detail(self, command_id: str) -> dict[str, Any]:
        detail = self._audit.get_command_detail(command_id)
        if detail is None:
            raise NotFoundError("command not found")
        return {
            "command": self._serialize_audited_command(detail["command"]),
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

    def _expire_pending_confirmations(self) -> None:
        now = utcnow()
        with self._lock:
            expiring = [
                command
                for command in self._commands.values()
                if command.lifecycle_state == CommandLifecycleState.NEEDS_CONFIRMATION
                and command.confirmation_expires_at is not None
                and command.confirmation_expires_at <= now
            ]
        for command in expiring:
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
