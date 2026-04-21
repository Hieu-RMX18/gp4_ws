from __future__ import annotations

from datetime import datetime, timezone
from typing import Any
from uuid import uuid4

from ..domain.models import CommandLifecycleState, CommandRecord, RuntimeMode, RuntimeSnapshot


class SupervisorExecutionMixin:
    @staticmethod
    def _utcnow() -> datetime:
        return datetime.now(timezone.utc)

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
        command.confirm_at = self._utcnow()
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
        command.execute_at = self._utcnow()

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
