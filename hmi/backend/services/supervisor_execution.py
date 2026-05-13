from __future__ import annotations

from datetime import datetime, timezone
from typing import Any
from uuid import uuid4

from ..domain.models import (
    CommandLifecycleState,
    CommandRecord,
    RuntimeMode,
    RuntimeSnapshot,
)


class SupervisorExecutionMixin:
    @staticmethod
    def _utcnow() -> datetime:
        return datetime.now(timezone.utc)

    def start_servo(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
    ) -> dict[str, Any]:
        return self._run_servo_control(
            session_id=session_id,
            operator_id=operator_id,
            lease_token=lease_token,
            action_label="Servo START",
            adapter_method_name="start_traj_mode",
            require_hardware_gate=True,
        )

    def hold_servo(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
    ) -> dict[str, Any]:
        return self._run_servo_control(
            session_id=session_id,
            operator_id=operator_id,
            lease_token=lease_token,
            action_label="Servo HOLD",
            adapter_method_name="stop_motion",
            require_hardware_gate=False,
        )

    def _run_servo_control(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        action_label: str,
        adapter_method_name: str,
        require_hardware_gate: bool,
    ) -> dict[str, Any]:
        lease = self._assert_controller(session_id, operator_id, lease_token)
        runtime = self._current_runtime()
        if runtime.mode != RuntimeMode.HARDWARE:
            return {
                "accepted": False,
                "message": f"{action_label} is allowed only in hardware mode.",
            }

        hardware_gate = self._hardware_gate_evaluator.evaluate()
        preflight = self._execution_preflight(requested_mode=RuntimeMode.HARDWARE)
        if require_hardware_gate and not preflight["accepted"]:
            return {
                "accepted": False,
                "message": "; ".join(preflight["reasons"])
                or "hardware preflight failed.",
            }

        adapter_method = getattr(self._ros, adapter_method_name, None)
        if not callable(adapter_method):
            return {
                "accepted": False,
                "message": f"{action_label} adapter method is unavailable.",
            }

        self._audit.record_runtime_event(
            system_state=runtime.system_state,
            session_id=session_id,
            operator_id=operator_id,
            command_id=None,
            message=f"{action_label} requested through supervised hardware preflight",
            payload={
                "leaseId": lease.lease_id,
                "hardwareGate": hardware_gate.to_dict(),
                "preflight": preflight,
                "requiresHardwareGate": require_hardware_gate,
            },
        )
        result = adapter_method()
        if not isinstance(result, dict):
            return {
                "accepted": False,
                "message": f"{action_label} returned an invalid response.",
            }
        return {
            "accepted": bool(result.get("accepted", False)),
            "message": str(result.get("message", "")),
        }

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
            reason = (
                "; ".join(confirm_gate["blockingReasons"])
                or "confirmation gate rejected command"
            )
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

        if self._is_get_pose_command(command):
            return self._execute_get_pose_query(
                command=command,
                session_id=session_id,
                operator_id=operator_id,
                runtime=runtime,
            )

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
                reason=execution_result.get("summary")
                or "execution completed successfully",
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
                reason=execution_result.get("summary")
                or "execution completed successfully",
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
                reason=execution_result.get("summary")
                or "execution boundary rejected request",
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
                reason=execution_result.get("summary")
                or "execution boundary rejected request",
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

    @staticmethod
    def _is_get_pose_command(command: CommandRecord) -> bool:
        parsed_intent = command.parsed_intent or {}
        normalized_command = parsed_intent.get("normalizedCommand") or {}
        primitive = str(
            normalized_command.get("primitive_type")
            or parsed_intent.get("action")
            or ""
        ).upper()
        return primitive == "GET_POSE"

    def _execute_get_pose_query(
        self,
        *,
        command: CommandRecord,
        session_id: str,
        operator_id: str,
        runtime: RuntimeSnapshot,
    ) -> dict[str, Any]:
        parsed_intent = command.parsed_intent or {}
        normalized_command = parsed_intent.get("normalizedCommand") or {}
        reference_frame = str(normalized_command.get("reference_frame") or "base_link")
        reader = getattr(self._ros, "get_current_pose", None)
        pose = reader(reference_frame=reference_frame) if callable(reader) else None

        if pose is None:
            execution_result = {
                "accepted": False,
                "adapter": "workspace_ros_adapter",
                "status": "failed",
                "summary": "GET_POSE query failed: /get_current_pose is unavailable or returned no pose.",
                "dispatchedToRos": False,
                "queryOnly": True,
                "referenceFrame": reference_frame,
            }
        else:
            position = pose.get("position", {}) if isinstance(pose, dict) else {}
            orientation = pose.get("orientation", {}) if isinstance(pose, dict) else {}
            pose_mm = {
                "position": {
                    "x": float(position.get("x", 0.0)) * 1000.0,
                    "y": float(position.get("y", 0.0)) * 1000.0,
                    "z": float(position.get("z", 0.0)) * 1000.0,
                },
                "orientation": {
                    "x": float(orientation.get("x", 0.0)),
                    "y": float(orientation.get("y", 0.0)),
                    "z": float(orientation.get("z", 0.0)),
                    "w": float(orientation.get("w", 1.0)),
                },
            }
            summary = (
                "GET_POSE result: "
                f"x={pose_mm['position']['x']:.1f} mm, "
                f"y={pose_mm['position']['y']:.1f} mm, "
                f"z={pose_mm['position']['z']:.1f} mm in {reference_frame}."
            )
            execution_result = {
                "accepted": True,
                "adapter": "workspace_ros_adapter",
                "status": "succeeded",
                "summary": summary,
                "dispatchedToRos": False,
                "queryOnly": True,
                "referenceFrame": reference_frame,
                "pose": pose,
                "poseMm": pose_mm,
            }

        command.execution_result = execution_result
        self._audit.record_runtime_event(
            system_state=runtime.system_state,
            session_id=session_id,
            operator_id=operator_id,
            command_id=command.command_id,
            message="pose query response recorded",
            payload=execution_result,
        )
        self._trace(
            "query.get_pose.response",
            command_id=command.command_id,
            correlation_id=command.correlation_id,
            status=execution_result.get("status"),
            summary=execution_result.get("summary"),
        )
        should_emit_querying_state = (
            command.lifecycle_state != CommandLifecycleState.VALIDATING
        )
        if should_emit_querying_state:
            self._transition_command(
                command,
                next_state=CommandLifecycleState.EXECUTING,
                reason="supervisor queried current pose via dedicated ROS service",
                runtime_state=runtime.system_state,
                payload=execution_result,
                message_text="Step 5/6 QUERYING: current TCP pose requested from /get_current_pose.",
                message_tag=CommandLifecycleState.EXECUTING.value,
            )

        if execution_result["status"] == "succeeded":
            command.final_state = CommandLifecycleState.SUCCEEDED
            command.reject_reason = None
            self._transition_command(
                command,
                next_state=CommandLifecycleState.SUCCEEDED,
                reason=execution_result["summary"],
                runtime_state=runtime.system_state,
                payload=execution_result,
                message_text=f"Step 6/6 RESULT: {execution_result['summary']}",
                message_tag=CommandLifecycleState.SUCCEEDED.value,
            )
        else:
            command.final_state = CommandLifecycleState.FAILED
            command.reject_reason = execution_result["summary"]
            self._transition_command(
                command,
                next_state=CommandLifecycleState.FAILED,
                reason=execution_result["summary"],
                runtime_state=runtime.system_state,
                payload=execution_result,
                message_text=f"Step 6/6 RESULT: {execution_result['summary']}",
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
            reason=execution_result["summary"] if not accepted else None,
        )
