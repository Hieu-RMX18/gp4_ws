from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
from threading import Lock
from typing import Any
from uuid import uuid4

from ..domain.models import (
    BridgeConnection,
    ChatMessage,
    CommandLifecycleState,
    CommandRecord,
    ConnectionHealth,
    JointPosition,
    LeaseRecord,
    LeaseRole,
    PlanMetrics,
    RuntimeMode,
    RuntimeSnapshot,
    SystemRuntimeState,
)
from ..domain.state_machine import ensure_command_transition, is_terminal_command_state
from .audit_service import AuditService
from .session_lock_service import LeaseNotOwnedError, LeaseRejectedError, SessionLockService


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
    ) -> None:
        self._audit = audit_service
        self._session_lock = session_lock_service
        self._ros = ros_adapter
        self._commands: dict[str, CommandRecord] = {}
        self._active_command_id: str | None = None
        self._messages: deque[ChatMessage] = deque(maxlen=200)
        self._subscribers: dict[asyncio.Queue[dict[str, Any]], tuple[str, str]] = {}
        self._lock = Lock()

    def subscribe(self, session_id: str, operator_id: str) -> asyncio.Queue[dict[str, Any]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=4)
        with self._lock:
            self._subscribers[queue] = (session_id, operator_id)
        return queue

    def unsubscribe(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        with self._lock:
            self._subscribers.pop(queue, None)

    def _broadcast_snapshots(self) -> None:
        with self._lock:
            subscribers = list(self._subscribers.items())
        for queue, (session_id, operator_id) in subscribers:
            event = {
                "type": "snapshot",
                "snapshot": self.build_snapshot(session_id, operator_id),
            }
            try:
                if queue.full():
                    queue.get_nowait()
                queue.put_nowait(event)
            except asyncio.QueueFull:
                continue

    def _append_message(
        self,
        *,
        origin: str,
        text: str,
        command_id: str | None = None,
        tag: str | None = None,
    ) -> None:
        self._messages.append(
            ChatMessage(
                message_id=str(uuid4()),
                command_id=command_id,
                origin=origin,
                timestamp=utcnow().strftime("%H:%M:%S"),
                text=text,
                tag=tag,
            )
        )

    def _current_runtime(self) -> RuntimeSnapshot:
        return self._ros.read_runtime_snapshot()

    def _current_connections(self) -> list[BridgeConnection]:
        return self._ros.read_connections()

    def _current_joints(self) -> list[JointPosition]:
        return self._ros.read_joint_positions()

    def _transport_state(self, connections: list[BridgeConnection]) -> str:
        if all(connection.health == ConnectionHealth.HEALTHY for connection in connections):
            return "connected"
        if any(connection.health != ConnectionHealth.DOWN for connection in connections):
            return "connecting"
        return "disconnected"

    def _serialize_lease_view(self, session_id: str, operator_id: str) -> dict[str, Any]:
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
                "statusText": "Observer mode — no active controller lease",
                "canForceTakeover": False,
            }

        owns_control = lease.session_id == session_id and lease.operator_id == operator_id
        return {
            "leaseId": lease.lease_id,
            "leaseToken": lease.lease_token if owns_control else None,
            "role": LeaseRole.CONTROLLER.value if owns_control else LeaseRole.OBSERVER.value,
            "ownsControl": owns_control,
            "holderOperatorId": lease.operator_id,
            "holderSessionId": lease.session_id,
            "acquiredAt": lease.acquired_at.isoformat(),
            "expiresAt": lease.expires_at.isoformat(),
            "statusText": (
                "Controller lease active for this session"
                if owns_control
                else f"Observer mode — controller lease held by {lease.operator_id}"
            ),
            "canForceTakeover": not owns_control,
        }

    def _serialize_connections(self, connections: list[BridgeConnection]) -> list[dict[str, Any]]:
        return [
            {
                "name": connection.name,
                "label": connection.label,
                "health": connection.health.value,
            }
            for connection in connections
        ]

    def _serialize_runtime(self, runtime: RuntimeSnapshot) -> dict[str, Any]:
        return {
            "systemState": runtime.system_state.value,
            "blocking": runtime.blocking,
            "statusText": runtime.status_text,
            "mode": runtime.mode.value,
            "robotStatus": {
                "servoState": runtime.robot_status.servo_state,
                "eStop": runtime.robot_status.e_stop,
                "alarmState": runtime.robot_status.alarm_state,
                "motionMode": runtime.robot_status.motion_mode,
                "trajectoryPointsUsed": runtime.robot_status.trajectory_points_used,
                "trajectoryPointsCapacity": runtime.robot_status.trajectory_points_capacity,
                "readinessMessage": runtime.robot_status.readiness_message,
            },
        }

    def _serialize_joint(self, joint: JointPosition) -> dict[str, Any]:
        return {
            "name": joint.name,
            "positionDeg": joint.position_deg,
            "minDeg": joint.min_deg,
            "maxDeg": joint.max_deg,
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
            "lifecycleState": command.lifecycle_state.value,
            "summaryLabel": command.summary_label,
            "plannerUsed": command.planner_used,
            "frameUsed": command.frame_used,
            "mode": command.mode.value,
            "rejectReason": command.reject_reason,
            "parsedIntent": command.parsed_intent,
            "validationResult": command.validation_result,
            "planSummary": command.plan_summary,
            "metrics": self._serialize_metrics(command.metrics),
            "createdAt": command.created_at.isoformat(),
            "confirmAt": command.confirm_at.isoformat() if command.confirm_at else None,
            "executeAt": command.execute_at.isoformat() if command.execute_at else None,
            "finalState": command.final_state.value if command.final_state else None,
        }

    def _serialize_replay_item(self, row: dict[str, Any]) -> dict[str, Any]:
        return {
            "commandId": row["command_id"],
            "sessionId": row["session_id"],
            "operatorId": row["operator_id"],
            "summaryLabel": row["raw_text"][:80],
            "lifecycleState": row["final_state"] or CommandLifecycleState.RECEIVED.value,
            "finalState": row["final_state"],
            "plannerUsed": row["planner_used"],
            "frameUsed": row["frame_used"],
            "mode": row["mode"],
            "createdAt": row["created_at"],
            "executeAt": row["execute_at"],
        }

    def build_snapshot(self, session_id: str, operator_id: str) -> dict[str, Any]:
        connections = self._current_connections()
        runtime = self._current_runtime()
        active_command = self._commands.get(self._active_command_id) if self._active_command_id else None
        replay_items = self._audit.list_commands(limit=25)

        return {
            "generatedAt": utcnow().isoformat(),
            "transportState": self._transport_state(connections),
            "mode": runtime.mode.value,
            "connections": self._serialize_connections(connections),
            "lease": self._serialize_lease_view(session_id, operator_id),
            "runtime": self._serialize_runtime(runtime),
            "messages": [message.to_dict() for message in self._messages],
            "activeCommand": self._serialize_command(active_command),
            "jointPositions": [self._serialize_joint(joint) for joint in self._current_joints()],
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
        try:
            self._session_lock.acquire_controller(
                session_id,
                operator_id,
                force_takeover=force_takeover,
                takeover_reason=takeover_reason,
            )
        except LeaseRejectedError as exc:
            lease_view = self._serialize_lease_view(session_id, operator_id)
            return {
                "accepted": False,
                "lease": lease_view,
                "reason": str(exc),
            }

        self._audit.record_runtime_event(
            system_state=self._current_runtime().system_state,
            session_id=session_id,
            operator_id=operator_id,
            message="controller lease acquired",
            payload={"force_takeover": force_takeover, "takeover_reason": takeover_reason},
        )
        self._broadcast_snapshots()
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
        self._broadcast_snapshots()
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
        self._broadcast_snapshots()
        return {
            "accepted": True,
            "lease": self._serialize_lease_view(session_id, operator_id),
            "reason": None,
        }

    def submit_command(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        raw_text: str,
        mode: str,
    ) -> dict[str, Any]:
        if not raw_text.strip():
            raise ConflictError("rawText must not be empty")

        try:
            self._session_lock.assert_controller(session_id, operator_id, lease_token)
        except LeaseNotOwnedError as exc:
            raise ForbiddenActionError(str(exc)) from exc

        runtime = self._current_runtime()
        command_id = str(uuid4())
        command = CommandRecord(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            raw_text=raw_text.strip(),
            lifecycle_state=CommandLifecycleState.RECEIVED,
            summary_label=raw_text.strip()[:80],
            mode=RuntimeMode(mode) if mode in RuntimeMode._value2member_map_ else RuntimeMode.UNKNOWN,
            created_at=utcnow(),
        )

        with self._lock:
            self._commands[command_id] = command
            self._active_command_id = command_id

        self._append_message(origin="operator", text=command.raw_text, command_id=command_id)
        self._audit.upsert_command(command)
        self._audit.record_transition(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            from_state=None,
            to_state=CommandLifecycleState.RECEIVED,
            runtime_state=runtime.system_state,
            reason="command submitted from HMI",
            payload={"mode": command.mode.value},
        )

        try:
            self._ros.submit_text_for_review(
                raw_text=command.raw_text,
                session_id=session_id,
                operator_id=operator_id,
                command_id=command_id,
            )
        except NotImplementedError as exc:
            previous = command.lifecycle_state
            ensure_command_transition(previous, CommandLifecycleState.REJECTED)
            command.lifecycle_state = CommandLifecycleState.REJECTED
            command.final_state = CommandLifecycleState.REJECTED
            command.reject_reason = str(exc)
            self._append_message(
                origin="system",
                text=str(exc),
                command_id=command_id,
                tag="REJECTED",
            )
            self._audit.upsert_command(command)
            self._audit.record_transition(
                command_id=command_id,
                session_id=session_id,
                operator_id=operator_id,
                from_state=previous,
                to_state=CommandLifecycleState.REJECTED,
                runtime_state=runtime.system_state,
                reason=str(exc),
                payload=None,
            )
            self._broadcast_snapshots()
            return {
                "accepted": False,
                "commandId": command_id,
                "reason": str(exc),
                "snapshot": self.build_snapshot(session_id, operator_id),
            }

        self._broadcast_snapshots()
        return {
            "accepted": True,
            "commandId": command_id,
            "reason": None,
            "snapshot": self.build_snapshot(session_id, operator_id),
        }

    def confirm_command(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        command_id: str,
    ) -> dict[str, Any]:
        try:
            self._session_lock.assert_controller(session_id, operator_id, lease_token)
        except LeaseNotOwnedError as exc:
            raise ForbiddenActionError(str(exc)) from exc

        command = self._commands.get(command_id)
        if command is None:
            raise NotFoundError("command not found")
        if command.lifecycle_state != CommandLifecycleState.READY_FOR_CONFIRM:
            raise ConflictError("command is not in READY_FOR_CONFIRM")

        try:
            self._ros.confirm_command(command_id=command_id)
        except NotImplementedError as exc:
            raise ConflictError(str(exc)) from exc

        previous = command.lifecycle_state
        ensure_command_transition(previous, CommandLifecycleState.EXECUTING)
        command.lifecycle_state = CommandLifecycleState.EXECUTING
        command.confirm_at = utcnow()
        command.execute_at = command.confirm_at
        self._append_message(
            origin="system",
            text="Command confirmed and handed off to backend execution.",
            command_id=command_id,
            tag="EXECUTING",
        )
        self._audit.upsert_command(command)
        self._audit.record_transition(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            from_state=previous,
            to_state=CommandLifecycleState.EXECUTING,
            runtime_state=self._current_runtime().system_state,
            reason="operator confirmed command",
            payload=None,
        )
        self._broadcast_snapshots()
        return {
            "accepted": True,
            "commandId": command_id,
            "reason": None,
            "snapshot": self.build_snapshot(session_id, operator_id),
        }

    def abort_command(
        self,
        *,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
        command_id: str,
        reason: str | None = None,
    ) -> dict[str, Any]:
        try:
            self._session_lock.assert_controller(session_id, operator_id, lease_token)
        except LeaseNotOwnedError as exc:
            raise ForbiddenActionError(str(exc)) from exc

        command = self._commands.get(command_id)
        if command is None:
            raise NotFoundError("command not found")
        if is_terminal_command_state(command.lifecycle_state):
            return {
                "accepted": True,
                "commandId": command_id,
                "reason": "command already terminal",
                "snapshot": self.build_snapshot(session_id, operator_id),
            }

        if command.lifecycle_state == CommandLifecycleState.EXECUTING:
            ok, adapter_reason = self._ros.abort_command(command_id=command_id)
            if not ok:
                raise ConflictError(adapter_reason)

        previous = command.lifecycle_state
        ensure_command_transition(previous, CommandLifecycleState.ABORTED)
        command.lifecycle_state = CommandLifecycleState.ABORTED
        command.final_state = CommandLifecycleState.ABORTED
        command.reject_reason = reason
        self._append_message(
            origin="system",
            text=reason or "Command aborted from HMI.",
            command_id=command_id,
            tag="REJECTED",
        )
        self._audit.upsert_command(command)
        self._audit.record_transition(
            command_id=command_id,
            session_id=session_id,
            operator_id=operator_id,
            from_state=previous,
            to_state=CommandLifecycleState.ABORTED,
            runtime_state=self._current_runtime().system_state,
            reason=reason or "operator aborted command",
            payload=None,
        )
        self._broadcast_snapshots()
        return {
            "accepted": True,
            "commandId": command_id,
            "reason": reason,
            "snapshot": self.build_snapshot(session_id, operator_id),
        }

    def list_replay(self, **filters: Any) -> dict[str, Any]:
        items = self._audit.list_commands(**filters)
        return {
            "items": [self._serialize_replay_item(item) for item in items],
        }

    def replay_detail(self, command_id: str) -> dict[str, Any]:
        detail = self._audit.get_command_detail(command_id)
        if detail is None:
            raise NotFoundError("command not found")

        command = detail["command"]
        return {
            "command": {
                "commandId": command["command_id"],
                "sessionId": command["session_id"],
                "operatorId": command["operator_id"],
                "rawText": command["raw_text"],
                "lifecycleState": command["final_state"] or CommandLifecycleState.RECEIVED.value,
                "summaryLabel": command["raw_text"][:80],
                "plannerUsed": command["planner_used"],
                "frameUsed": command["frame_used"],
                "mode": command["mode"],
                "rejectReason": command["reject_reason"],
                "parsedIntent": None,
                "validationResult": None,
                "planSummary": None,
                "metrics": None,
                "createdAt": command["created_at"],
                "confirmAt": command["confirm_at"],
                "executeAt": command["execute_at"],
                "finalState": command["final_state"],
            },
            "timeline": [
                {
                    "id": row["event_id"],
                    "commandId": row["command_id"],
                    "timestamp": row["created_at"],
                    "fromState": row["from_state"],
                    "toState": row["to_state"],
                    "runtimeState": row["runtime_state"],
                    "message": row["reason"] or "",
                    "payload": None,
                }
                for row in detail["timeline"]
            ],
            "runtimeEvents": [
                {
                    "id": row["event_id"],
                    "commandId": row["command_id"],
                    "timestamp": row["created_at"],
                    "fromState": None,
                    "toState": None,
                    "runtimeState": row["system_state"],
                    "message": row["message"],
                    "payload": None,
                }
                for row in detail["runtime_events"]
            ],
        }

