from __future__ import annotations

import hashlib
import json
from typing import Any

from ..domain.models import (
    ChatMessage,
    CommandKind,
    CommandLifecycleState,
    CommandRecord,
    CommandRiskLevel,
    RuntimeMode,
    RuntimeSnapshot,
    SystemRuntimeState,
)
from ..domain.state_machine import ensure_command_transition, is_terminal_command_state
from .supervisor_validation import EVENT_DRIVEN_SOURCE_NAMES


def _utcnow():
    from datetime import datetime, timezone

    return datetime.now(timezone.utc)


class SupervisorLifecycleMixin:
    """Mixin providing confirm, cancel, transition, validation, reject, and expire methods."""

    def confirm_command(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        command_id: str,
        plan_fingerprint: str,
    ) -> dict[str, Any]:
        from .supervisor_service import ConflictError

        self._expire_pending_confirmations()
        lease = self._assert_controller(session_id, operator_id, lease_token)
        command = self._require_owned_command(command_id, session_id, operator_id)
        if (
            command.command_kind != CommandKind.COMMAND
            or command.parent_sequence_id is not None
        ):
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
        if (
            command.confirmation_expires_at is None
            or command.confirmation_expires_at <= _utcnow()
        ):
            self._expire_command(
                command,
                reason="confirmation window expired before operator confirmation",
            )
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
        from .supervisor_service import ConflictError

        self._expire_pending_confirmations()
        lease = self._assert_controller(session_id, operator_id, lease_token)
        sequence = self._require_owned_sequence(sequence_id, session_id, operator_id)
        if sequence.lifecycle_state != CommandLifecycleState.NEEDS_CONFIRMATION:
            raise ConflictError("sequence is not waiting for confirmation")
        if (
            sequence.confirmation_expires_at is None
            or sequence.confirmation_expires_at <= _utcnow()
        ):
            self._expire_top_level_record(
                sequence,
                reason="confirmation window expired before operator confirmation",
            )
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
        sequence.confirm_at = _utcnow()
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
        sequence.execute_at = _utcnow()
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
                    and str(
                        self._commands[step_id].parsed_intent.get("action") or ""
                    ).upper()
                    == "IO_SET"
                    for step_id in sequence.child_command_ids[
                        : (child_command.sequence_step_index or 0) + 1
                    ]
                )
                sequence.execution_result = {
                    "accepted": False,
                    "adapter": "workspace_ros_adapter",
                    "status": (
                        child_command.final_state.value.lower()
                        if child_command.final_state
                        else "failed"
                    ),
                    "summary": child_command.reject_reason or "sequence step failed",
                    "dispatchedToRos": bool(
                        child_command.execution_result
                        and child_command.execution_result.get("dispatchedToRos")
                    ),
                }
                self._transition_top_level_record(
                    sequence,
                    next_state=child_command.final_state
                    or CommandLifecycleState.FAILED,
                    reason=child_command.reject_reason or "sequence step failed",
                    runtime_state=self._current_runtime().system_state,
                    payload={
                        "failedCommandId": child_command.command_id,
                        "failedStepIndex": child_command.sequence_step_index,
                    },
                    message_text=f"Step 6/6 RESULT: {child_command.reject_reason or 'Sequence step failed.'}",
                    message_tag=(
                        child_command.final_state.value
                        if child_command.final_state
                        else CommandLifecycleState.FAILED.value
                    ),
                )
                return self._sequence_response(
                    session_id,
                    operator_id,
                    sequence,
                    accepted=False,
                    reason=sequence.reject_reason,
                )

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
        return self._sequence_response(
            session_id, operator_id, sequence, accepted=True, reason=None
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
        from .supervisor_service import ConflictError

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
        return self._command_response(
            session_id, operator_id, command, accepted=True, reason=reason
        )

    def cancel_sequence(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        sequence_id: str,
        reason: str | None = None,
    ) -> dict[str, Any]:
        from .supervisor_service import ConflictError

        self._expire_pending_confirmations()
        self._assert_controller(session_id, operator_id, lease_token)
        sequence = self._require_owned_sequence(sequence_id, session_id, operator_id)
        if is_terminal_command_state(sequence.lifecycle_state):
            return self._sequence_response(
                session_id,
                operator_id,
                sequence,
                accepted=True,
                reason="sequence already terminal",
            )

        current_child = (
            self._commands.get(self._active_command_id)
            if self._active_command_id
            else None
        )
        if (
            current_child is not None
            and current_child.parent_sequence_id == sequence.command_id
            and current_child.lifecycle_state
            in {
                CommandLifecycleState.EXECUTION_REQUESTED,
                CommandLifecycleState.EXECUTING,
            }
        ):
            ok, adapter_reason = self._ros.abort_command(
                command_id=current_child.command_id
            )
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
        return self._sequence_response(
            session_id, operator_id, sequence, accepted=True, reason=reason
        )

    # ── Transition helpers ─────────────────────────────────────────────────

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

    # ── Sequence validation ────────────────────────────────────────────────

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
        critical_sources = [
            source
            for source in source_statuses
            if getattr(source, "active", False)
            and source.name not in EVENT_DRIVEN_SOURCE_NAMES
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
            "HMI v2 requires explicit operator confirmation before a validated sequence may cross the execution boundary."
        ]
        hardware_gate = self._hardware_gate_evaluator.evaluate()
        preflight = self._execution_preflight(requested_mode=requested_mode)
        if requested_mode not in {RuntimeMode.SIM, RuntimeMode.HARDWARE}:
            blocking_reasons.append(
                f"runtime mode {requested_mode.value} is not command-capable for HMI v2."
            )
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
        stale_sources = [
            source.name
            for source in critical_sources
            if source.freshness_state.value != "fresh"
        ]
        if stale_sources:
            blocking_reasons.append(
                "freshness-critical telemetry is stale or unavailable: "
                + ", ".join(stale_sources)
            )
        if not preflight.get("accepted", True):
            blocking_reasons.extend(
                [str(reason) for reason in (preflight.get("reasons") or [])]
            )

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
        confirmation_reasons.append(
            f"Sequence contains {len(parsed_steps)} ordered steps."
        )
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

    # ── Reject / Expire ────────────────────────────────────────────────────

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
            message="sequence rejected"
            if command.command_kind == CommandKind.SEQUENCE
            else "command rejected",
            payload={"reason": reason},
        )
        self._broadcast_replay_update()

    def _expire_pending_confirmations(self) -> None:
        now = _utcnow()
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
                self._expire_top_level_record(
                    command, reason="confirmation window expired"
                )
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
