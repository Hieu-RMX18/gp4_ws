from __future__ import annotations

import hashlib
import json
import re
from collections import deque
from datetime import datetime, timedelta, timezone
from threading import Lock
from typing import Any
from uuid import uuid4

from ..domain.models import (
    BridgeCapabilities,
    ChatMessage,
    CommandLifecycleState,
    CommandRecord,
    CommandRiskLevel,
    JointPosition,
    LeaseRole,
    PlanMetrics,
    RuntimeMode,
    RuntimeSnapshot,
    SystemRuntimeState,
    TelemetryFreshnessState,
)
from ..domain.state_machine import (
    ensure_command_transition,
    is_blocking_runtime_state,
    is_terminal_command_state,
)
from .audit_service import AuditService
from .session_lock_service import LeaseNotOwnedError, LeaseRejectedError, SessionLockService
from .telemetry_bridge_service import TelemetryBridgeService


CARTESIAN_DIRECTIONS_MM = {
    "up": {"zMm": 1.0},
    "down": {"zMm": -1.0},
    "left": {"yMm": 1.0},
    "right": {"yMm": -1.0},
    "forward": {"xMm": 1.0},
    "back": {"xMm": -1.0},
    "backward": {"xMm": -1.0},
}
UNIT_TO_MM = {"mm": 1.0, "cm": 10.0, "m": 1000.0}
SUPPORTED_ACTIONS = {"move_home", "move_cartesian_delta", "move_joint_delta", "stop"}
EVENT_DRIVEN_SOURCE_NAMES = {"llm_debug", "llm_command"}
GP4_JOINT_NAMES = (
    "joint_1_s",
    "joint_2_l",
    "joint_3_u",
    "joint_4_r",
    "joint_5_b",
    "joint_6_t",
)


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


class SupervisorService:
    def __init__(
        self,
        *,
        audit_service: AuditService,
        session_lock_service: SessionLockService,
        ros_adapter: Any,
        confirmation_window_sec: float = 30.0,
    ) -> None:
        self._audit = audit_service
        self._session_lock = session_lock_service
        self._ros = ros_adapter
        self._telemetry: TelemetryBridgeService | None = None
        self._confirmation_window = timedelta(seconds=confirmation_window_sec)
        self._commands: dict[str, CommandRecord] = {}
        self._active_command_id: str | None = None
        self._messages: deque[ChatMessage] = deque(maxlen=200)
        self._lock = Lock()

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
        )

        parsed_intent, parse_error = self._parse_intent(raw_text, structured_intent)
        if parsed_intent is None:
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

        parsed_intent, enrich_error = self._enrich_parsed_intent(parsed_intent)
        command.parsed_intent = parsed_intent
        if parsed_intent is None:
            command.reject_reason = enrich_error
            command.validation_result = self._build_validation_result(
                runtime=runtime,
                lease=lease,
                parsed_intent=None,
                blocking_reasons=[enrich_error],
                risk_level=None,
                requires_confirmation=False,
            )
            self._reject_command(
                command,
                reason=enrich_error,
                runtime_state=runtime.system_state,
                session_id=session_id,
                operator_id=operator_id,
            )
            return self._command_response(session_id, operator_id, command, accepted=False, reason=enrich_error)

        command.summary_label = parsed_intent["targetSummary"]
        command.planner_used = self._planner_for_intent(parsed_intent)
        command.frame_used = self._frame_for_intent(parsed_intent)
        self._transition_command(
            command,
            next_state=CommandLifecycleState.VALIDATING,
            reason="supervisor validating parsed intent",
            runtime_state=runtime.system_state,
            payload={"parsedIntent": parsed_intent},
        )

        validation = self._validate_command(runtime=runtime, lease=lease, parsed_intent=parsed_intent)
        command.validation_result = validation
        command.plan_summary = {
            "normalizedIntent": parsed_intent["normalizedText"],
            "parsedAction": parsed_intent["action"],
            "targetSummary": parsed_intent["targetSummary"],
            "requiresConfirmation": validation["requiresConfirmation"],
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
            message_text="Validation passed. Review the normalized plan and confirm before execution boundary handoff.",
            message_tag=CommandLifecycleState.NEEDS_CONFIRMATION.value,
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
        if command.lifecycle_state != CommandLifecycleState.NEEDS_CONFIRMATION:
            raise ConflictError("command is not waiting for confirmation")
        if command.confirmation_expires_at is None or command.confirmation_expires_at <= utcnow():
            self._expire_command(command, reason="confirmation window expired before operator confirmation")
            raise ConflictError("confirmation window expired")
        if plan_fingerprint != command.plan_fingerprint:
            raise ConflictError("plan fingerprint mismatch")

        runtime = self._current_runtime()
        confirm_gate = self._validate_command(
            runtime=runtime,
            lease=lease,
            parsed_intent=command.parsed_intent,
        )
        if not confirm_gate["accepted"]:
            command.validation_result = confirm_gate
            self._audit.upsert_command(command)
            self._emit_command_event(command, messages=None)
            return self._command_response(
                session_id,
                operator_id,
                command,
                accepted=False,
                reason="; ".join(confirm_gate["blockingReasons"]),
            )

        self._transition_command(
            command,
            next_state=CommandLifecycleState.CONFIRMED,
            reason="operator confirmed validated plan",
            runtime_state=runtime.system_state,
            payload={"planFingerprint": plan_fingerprint},
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
            message_text="Confirmed plan forwarded to the supervisor-owned execution boundary.",
            message_tag=CommandLifecycleState.EXECUTION_REQUESTED.value,
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

        execution_status = str(execution_result.get("status") or "").lower()
        dispatched_to_ros = bool(execution_result.get("dispatchedToRos"))
        if dispatched_to_ros:
            self._transition_command(
                command,
                next_state=CommandLifecycleState.EXECUTING,
                reason="execution adapter dispatched the validated request to ROS",
                runtime_state=runtime.system_state,
                payload=execution_result,
                message_text="Execution adapter dispatched the validated request to ROS.",
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
                message_text=execution_result.get("summary") or "Execution completed successfully.",
                message_tag=CommandLifecycleState.SUCCEEDED.value,
            )
        elif execution_status == "cancelled":
            if not dispatched_to_ros:
                self._transition_command(
                    command,
                    next_state=CommandLifecycleState.EXECUTING,
                    reason="execution adapter reported a cancellation outcome",
                    runtime_state=runtime.system_state,
                    payload=execution_result,
                    message_text="Execution adapter reported a cancellation outcome.",
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
                message_text=execution_result.get("summary") or "Execution was cancelled.",
                message_tag=CommandLifecycleState.CANCELLED.value,
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
                message_text=execution_result.get("summary") or "Execution boundary rejected the request.",
                message_tag=CommandLifecycleState.FAILED.value,
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
            message_text=reason or "Command cancelled.",
            message_tag=CommandLifecycleState.CANCELLED.value,
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

    def _bridge_capabilities(self) -> BridgeCapabilities:
        runtime = self._current_runtime()
        sim_mode = runtime.mode == RuntimeMode.SIM
        command_ingress_available = sim_mode
        confirmation_available = sim_mode
        return BridgeCapabilities(
            read_only=not command_ingress_available,
            can_acquire_lease=command_ingress_available,
            can_submit_commands=command_ingress_available,
            can_confirm_commands=confirmation_available,
            can_cancel_commands=confirmation_available,
            can_abort_commands=confirmation_available,
            command_ingress_available=command_ingress_available,
            confirmation_available=confirmation_available,
            execution_allowed=sim_mode,
            replay_available=True,
            sim_only=True,
        )

    def _serialize_lease_view(self, session_id: str, operator_id: str) -> dict[str, Any]:
        capabilities = self._bridge_capabilities()
        lease = self._session_lock.current_controller()
        if lease is None:
            return {
                "leaseId": None,
                "leaseToken": None,
                "role": LeaseRole.OBSERVER.value,
                "ownsControl": False,
                "holderOperatorId": None,
                "holderSessionId": None,
                "acquiredAt": None,
                "expiresAt": None,
                "statusText": (
                    "Observer mode — request the supervisor lease before submitting commands."
                    if capabilities.can_acquire_lease
                    else "Read-only telemetry mode — command lease acquisition is disabled outside sim mode."
                ),
                "canForceTakeover": capabilities.can_acquire_lease,
            }

        owns_control = lease.session_id == session_id and lease.operator_id == operator_id
        if capabilities.read_only:
            role = LeaseRole.OBSERVER.value
            lease_token: str | None = None
            owns_control = False
            status_text = (
                "Hardware and unknown runtime modes remain read-only until MotoROS2 freshness is explicitly verified."
            )
        else:
            role = LeaseRole.CONTROLLER.value if owns_control else LeaseRole.OBSERVER.value
            lease_token = lease.lease_token if owns_control else None
            status_text = (
                "Controller lease active for this session"
                if owns_control
                else f"Observer mode — controller lease held by {lease.operator_id}"
            )

        return {
            "leaseId": lease.lease_id,
            "leaseToken": lease_token,
            "role": role,
            "ownsControl": owns_control,
            "holderOperatorId": lease.operator_id,
            "holderSessionId": lease.session_id,
            "acquiredAt": lease.acquired_at.isoformat(),
            "expiresAt": lease.expires_at.isoformat(),
            "statusText": status_text,
            "canForceTakeover": capabilities.can_acquire_lease and not owns_control,
        }

    def _serialize_metrics(self, metrics: PlanMetrics | None) -> dict[str, Any] | None:
        if metrics is None:
            return None
        return {
            "score": metrics.score,
            "pathLengthRad": metrics.path_length_rad,
            "smoothness": metrics.smoothness,
            "clearanceM": metrics.clearance_m,
            "cartesianCompletionPct": metrics.cartesian_completion_pct,
            "replanCount": metrics.replan_count,
        }

    def _serialize_command(self, command: CommandRecord | None) -> dict[str, Any] | None:
        if command is None:
            return None
        return {
            "commandId": command.command_id,
            "sessionId": command.session_id,
            "operatorId": command.operator_id,
            "rawText": command.raw_text,
            "intentSource": command.intent_source,
            "structuredIntent": command.structured_intent,
            "lifecycleState": command.lifecycle_state.value,
            "summaryLabel": command.summary_label,
            "plannerUsed": command.planner_used,
            "frameUsed": command.frame_used,
            "mode": command.mode.value,
            "riskLevel": command.risk_level.value if command.risk_level else None,
            "planFingerprint": command.plan_fingerprint,
            "correlationId": command.correlation_id,
            "rejectReason": command.reject_reason,
            "parsedIntent": command.parsed_intent,
            "validationResult": command.validation_result,
            "planSummary": command.plan_summary,
            "metrics": self._serialize_metrics(command.metrics),
            "confirmationExpiresAt": (
                command.confirmation_expires_at.isoformat() if command.confirmation_expires_at else None
            ),
            "createdAt": command.created_at.isoformat(),
            "confirmAt": command.confirm_at.isoformat() if command.confirm_at else None,
            "executeAt": command.execute_at.isoformat() if command.execute_at else None,
            "executionResult": command.execution_result,
            "finalState": command.final_state.value if command.final_state else None,
        }

    def _serialize_audited_command(self, row: dict[str, Any]) -> dict[str, Any]:
        parsed_intent = self._decode_json(row.get("parsed_intent_json"))
        validation_result = self._decode_json(row.get("validation_result_json"))
        structured_intent = self._decode_json(row.get("structured_intent_json"))
        plan_summary = self._decode_json(row.get("plan_summary_json"))
        execution_result = self._decode_json(row.get("execution_result_json"))
        return {
            "commandId": row["command_id"],
            "sessionId": row["session_id"],
            "operatorId": row["operator_id"],
            "rawText": row["raw_text"],
            "intentSource": "structured" if structured_intent else "text",
            "structuredIntent": structured_intent,
            "lifecycleState": row.get("lifecycle_state") or row.get("final_state") or CommandLifecycleState.RECEIVED.value,
            "summaryLabel": row.get("summary_label") or row["raw_text"][:80],
            "plannerUsed": row.get("planner_used"),
            "frameUsed": row.get("frame_used"),
            "mode": row["mode"],
            "riskLevel": row.get("risk_level"),
            "planFingerprint": row.get("plan_fingerprint"),
            "correlationId": row.get("correlation_id"),
            "rejectReason": row.get("reject_reason"),
            "parsedIntent": parsed_intent,
            "validationResult": validation_result,
            "planSummary": plan_summary,
            "metrics": None,
            "confirmationExpiresAt": row.get("review_expires_at"),
            "createdAt": row["created_at"],
            "confirmAt": row.get("confirm_at"),
            "executeAt": row.get("execute_at"),
            "executionResult": execution_result,
            "finalState": row.get("final_state"),
        }

    def _serialize_replay_item(self, row: dict[str, Any]) -> dict[str, Any]:
        lifecycle_state = row.get("lifecycle_state") or row.get("final_state") or CommandLifecycleState.RECEIVED.value
        return {
            "commandId": row["command_id"],
            "sessionId": row["session_id"],
            "operatorId": row["operator_id"],
            "summaryLabel": row.get("summary_label") or row["raw_text"][:80],
            "lifecycleState": lifecycle_state,
            "finalState": row.get("final_state"),
            "plannerUsed": row.get("planner_used"),
            "frameUsed": row.get("frame_used"),
            "mode": row["mode"],
            "createdAt": row["created_at"],
            "executeAt": row.get("execute_at"),
            "riskLevel": row.get("risk_level"),
        }

    def _serialize_timeline_row(self, row: dict[str, Any]) -> dict[str, Any]:
        return {
            "id": row["event_id"],
            "commandId": row["command_id"],
            "timestamp": row["created_at"],
            "fromState": row["from_state"],
            "toState": row["to_state"],
            "runtimeState": row["runtime_state"],
            "message": row["reason"] or "",
            "payload": self._decode_json(row.get("payload_json")),
        }

    def _serialize_runtime_row(self, row: dict[str, Any]) -> dict[str, Any]:
        return {
            "id": row["event_id"],
            "commandId": row.get("command_id"),
            "timestamp": row["created_at"],
            "fromState": None,
            "toState": None,
            "runtimeState": row["system_state"],
            "message": row["message"],
            "payload": self._decode_json(row.get("payload_json")),
        }

    def _parse_intent(
        self,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
    ) -> tuple[dict[str, Any] | None, str | None]:
        if structured_intent is not None:
            action = structured_intent.get("action")
            if action not in SUPPORTED_ACTIONS:
                return None, f"unsupported structured action: {action!r}"
            target_summary = structured_intent.get("targetSummary") or action.replace("_", " ")
            return {
                "source": "structured",
                "normalizedText": json.dumps(structured_intent, separators=(",", ":"), ensure_ascii=True),
                "action": action,
                "parameters": structured_intent.get("parameters") or {},
                "targetSummary": target_summary,
            }, None

        normalized = " ".join(raw_text.lower().split())
        if not normalized:
            return None, "empty command text is not allowed"

        if normalized in {"home", "go home", "move home", "return home"}:
            return {
                "source": "text",
                "normalizedText": normalized,
                "action": "move_home",
                "parameters": {"frame": "world"},
                "targetSummary": "Return robot to the named home position.",
            }, None

        if normalized in {"stop", "stop motion", "cancel motion", "halt"}:
            return {
                "source": "text",
                "normalizedText": normalized,
                "action": "stop",
                "parameters": {},
                "targetSummary": "Request supervised stop handling.",
            }, None

        cartesian_match = re.fullmatch(
            r"move\s+(up|down|left|right|forward|back|backward)\s+([+-]?\d+(?:\.\d+)?)\s*(mm|cm|m)",
            normalized,
        )
        if cartesian_match:
            direction, magnitude_text, unit = cartesian_match.groups()
            magnitude_mm = float(magnitude_text) * UNIT_TO_MM[unit]
            delta = {
                axis: scale * magnitude_mm
                for axis, scale in CARTESIAN_DIRECTIONS_MM[direction].items()
            }
            return {
                "source": "text",
                "normalizedText": normalized,
                "action": "move_cartesian_delta",
                "parameters": {"frame": "base_link", **delta},
                "targetSummary": f"Move TCP {direction} by {magnitude_mm:.1f} mm in base_link.",
            }, None

        joint_match = re.fullmatch(
            r"(?:move\s+)?(?:joint|j)\s*([1-6])\s+([+-]?\d+(?:\.\d+)?)\s*(deg|degree|degrees)",
            normalized,
        )
        if joint_match:
            joint_index, delta_text, _unit = joint_match.groups()
            delta_deg = float(delta_text)
            return {
                "source": "text",
                "normalizedText": normalized,
                "action": "move_joint_delta",
                "parameters": {"joint": f"joint_{joint_index}", "deltaDeg": delta_deg},
                "targetSummary": f"Move joint {joint_index} by {delta_deg:.1f} deg.",
            }, None

        return None, (
            "intent is ambiguous or unsupported; submit a structured intent or use one of the supported phrases: "
            "home, stop, move <direction> <distance>, or move joint <n> <deg>."
        )

    def _enrich_parsed_intent(
        self,
        parsed_intent: dict[str, Any],
    ) -> tuple[dict[str, Any] | None, str | None]:
        action = parsed_intent.get("action")
        if action != "move_joint_delta":
            return parsed_intent, None

        parameters = dict(parsed_intent.get("parameters") or {})
        joint_index, joint_name = self._resolve_joint_target(parameters)
        if joint_index is None or joint_name is None:
            return None, "joint delta command did not resolve to a valid GP4 joint"

        current_position_deg = self._read_joint_position_deg(joint_name)
        if current_position_deg is None:
            return None, f"fresh joint position for {joint_name} is unavailable"

        delta_deg = float(parameters.get("deltaDeg", 0.0))
        resolved_target_deg = current_position_deg + delta_deg
        parameters.update(
            {
                "jointIndexZeroBased": joint_index,
                "jointNameResolved": joint_name,
                "currentPositionDeg": current_position_deg,
                "resolvedTargetDeg": resolved_target_deg,
            }
        )
        enriched = dict(parsed_intent)
        enriched["parameters"] = parameters
        enriched["targetSummary"] = (
            f"Move joint {joint_index + 1} by {delta_deg:.1f} deg "
            f"to {resolved_target_deg:.1f} deg absolute."
        )
        return enriched, None

    def _resolve_joint_target(self, parameters: dict[str, Any]) -> tuple[int | None, str | None]:
        raw_joint_index = parameters.get("jointIndex")
        if raw_joint_index is not None:
            try:
                candidate = int(raw_joint_index)
            except (TypeError, ValueError):
                candidate = None
            if candidate is not None:
                if 0 <= candidate < len(GP4_JOINT_NAMES):
                    return candidate, GP4_JOINT_NAMES[candidate]
                if 1 <= candidate <= len(GP4_JOINT_NAMES):
                    zero_based = candidate - 1
                    return zero_based, GP4_JOINT_NAMES[zero_based]

        raw_joint_name = str(parameters.get("joint") or parameters.get("jointName") or "").strip().lower()
        if raw_joint_name:
            if raw_joint_name in {name.lower() for name in GP4_JOINT_NAMES}:
                for index, joint_name in enumerate(GP4_JOINT_NAMES):
                    if joint_name.lower() == raw_joint_name:
                        return index, joint_name
            joint_match = re.fullmatch(r"joint[_\s-]*([1-6])(?:[_\s-].+)?", raw_joint_name)
            if joint_match:
                zero_based = int(joint_match.group(1)) - 1
                return zero_based, GP4_JOINT_NAMES[zero_based]

        return None, None

    def _read_joint_position_deg(self, joint_name: str) -> float | None:
        for joint in self._current_joints():
            if joint.name == joint_name:
                return joint.position_deg
        return None

    def _planner_for_intent(self, parsed_intent: dict[str, Any]) -> str | None:
        action = parsed_intent.get("action")
        if action in {"move_home", "move_joint_delta"}:
            return "PILZ_PTP"
        if action == "move_cartesian_delta":
            return "PILZ_LIN"
        return None

    def _frame_for_intent(self, parsed_intent: dict[str, Any]) -> str | None:
        action = parsed_intent.get("action")
        parameters = parsed_intent.get("parameters") or {}
        if action == "move_cartesian_delta":
            return str(parameters.get("frame") or "base_link")
        if action == "move_home":
            return "base_link"
        if action == "move_joint_delta":
            return "joint_space"
        return None

    def _validate_command(
        self,
        *,
        runtime: RuntimeSnapshot,
        lease: Any,
        parsed_intent: dict[str, Any] | None,
    ) -> dict[str, Any]:
        source_statuses = self._read_source_statuses()
        critical_sources = [source for source in source_statuses if getattr(source, "active", False)]
        optional_sources = [
            source for source in source_statuses if not getattr(source, "active", False) and source.name not in EVENT_DRIVEN_SOURCE_NAMES
        ]
        event_driven_sources = [source for source in source_statuses if source.name in EVENT_DRIVEN_SOURCE_NAMES]
        blocking_reasons: list[str] = []
        confirmation_reasons = [
            "HMI v2 requires explicit operator confirmation before a validated plan may cross the execution boundary."
        ]

        if parsed_intent is None:
            blocking_reasons.append("parsed intent is unavailable")

        if runtime.mode != RuntimeMode.SIM:
            blocking_reasons.append(
                "HMI v2 command ingress is sim-only until hardware-side MotoROS2 freshness and execution semantics are verified."
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
                "freshness-critical telemetry is stale or unavailable: " + ", ".join(stale_sources)
            )

        risk_level = self._assess_risk(parsed_intent)
        if risk_level in {CommandRiskLevel.HIGH, CommandRiskLevel.CRITICAL}:
            confirmation_reasons.append(
                f"Risk assessment is {risk_level.value}; high-risk actions must stay behind confirmation."
            )

        if parsed_intent and parsed_intent["action"] == "move_cartesian_delta":
            confirmation_reasons.append("Cartesian motion requests always require confirmation in v2.")
        if parsed_intent and parsed_intent["action"] == "move_joint_delta":
            confirmation_reasons.append("Joint delta motion requests always require confirmation in v2.")

        plan_fingerprint = (
            self._plan_fingerprint(parsed_intent, lease.lease_id, runtime.mode.value)
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
            "blockingReasons": blocking_reasons,
            "confirmationReasons": confirmation_reasons,
            "planFingerprint": plan_fingerprint,
            "executionAllowedNow": False,
            "criticalSources": [self._source_status_view(source) for source in critical_sources],
            "optionalSources": [self._source_status_view(source) for source in optional_sources],
            "eventDrivenSources": [self._source_status_view(source) for source in event_driven_sources],
        }

    def _assess_risk(self, parsed_intent: dict[str, Any] | None) -> CommandRiskLevel | None:
        if parsed_intent is None:
            return None
        action = parsed_intent["action"]
        parameters = parsed_intent.get("parameters") or {}
        if action == "stop":
            return CommandRiskLevel.LOW
        if action == "move_home":
            return CommandRiskLevel.MEDIUM
        if action == "move_joint_delta":
            magnitude = abs(float(parameters.get("deltaDeg", 0.0)))
            if magnitude <= 5.0:
                return CommandRiskLevel.LOW
            if magnitude <= 20.0:
                return CommandRiskLevel.MEDIUM
            return CommandRiskLevel.HIGH
        if action == "move_cartesian_delta":
            magnitude = max(abs(float(value)) for value in parameters.values() if isinstance(value, (int, float)))
            if magnitude <= 20.0:
                return CommandRiskLevel.MEDIUM
            if magnitude <= 100.0:
                return CommandRiskLevel.HIGH
            return CommandRiskLevel.CRITICAL
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
        return {
            "accepted": not blocking_reasons,
            "leaseValid": lease is not None,
            "runtimeAllowed": not is_blocking_runtime_state(runtime.system_state),
            "telemetryFresh": False,
            "requiresConfirmation": requires_confirmation,
            "riskLevel": risk_level.value if risk_level else None,
            "blockingReasons": blocking_reasons,
            "confirmationReasons": [],
            "planFingerprint": None,
            "executionAllowedNow": False,
            "criticalSources": [],
            "optionalSources": [],
            "eventDrivenSources": [],
        }

    def _source_status_view(self, source: Any) -> dict[str, Any]:
        return {
            "name": source.name,
            "label": source.label,
            "topic": source.topic,
            "freshnessState": source.freshness_state.value,
            "active": source.active,
            "preferred": source.preferred,
            "detail": source.detail,
        }

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
            message_text=reason,
            message_tag=CommandLifecycleState.REJECTED.value,
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
            message_text="Command expired before confirmation.",
            message_tag=CommandLifecycleState.EXPIRED.value,
        )
        self._audit.upsert_command(command)
        self._broadcast_replay_update()

    def _command_response(
        self,
        session_id: str,
        operator_id: str,
        command: CommandRecord,
        *,
        accepted: bool,
        reason: str | None,
    ) -> dict[str, Any]:
        return {
            "accepted": accepted,
            "commandId": command.command_id,
            "reason": reason,
            "snapshot": self._telemetry.get_snapshot(session_id, operator_id) if self._telemetry else None,
            "command": self._serialize_command(command),
        }

    def _emit_command_event(
        self,
        command: CommandRecord,
        messages: list[ChatMessage] | None,
    ) -> None:
        if self._telemetry is None:
            return
        self._telemetry.broadcast_event(
            lambda _session_id, _operator_id: {
                "type": "command_lifecycle",
                "command": self._serialize_command(command),
                "messages": [message.to_dict() for message in messages] if messages else [],
                "planMetrics": self._serialize_metrics(command.metrics),
            }
        )
        self._broadcast_replay_update()

    def _broadcast_replay_update(self) -> None:
        if self._telemetry is None:
            return
        self._telemetry.broadcast_event(
            lambda _session_id, _operator_id: {
                "type": "replay_updated",
                "replayItems": [
                    self._serialize_replay_item(item)
                    for item in self._audit.list_commands(limit=25)
                ],
            }
        )

    def _broadcast_lease_state(self) -> None:
        if self._telemetry is None:
            return
        self._telemetry.broadcast_event(
            lambda session_id, operator_id: {
                "type": "lease_state",
                "lease": self._serialize_lease_view(session_id, operator_id),
                "capabilities": self._bridge_capabilities().to_dict(),
            }
        )

    def _append_message(
        self,
        *,
        origin: str,
        text: str,
        command_id: str | None = None,
        tag: str | None = None,
    ) -> ChatMessage:
        message = ChatMessage(
            message_id=str(uuid4()),
            command_id=command_id,
            origin=origin,
            timestamp=utcnow().strftime("%H:%M:%S"),
            text=text,
            tag=tag,
        )
        self._messages.append(message)
        return message

    def _decode_json(self, payload: str | None) -> Any:
        if not payload:
            return None
        return json.loads(payload)
