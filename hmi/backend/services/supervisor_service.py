from __future__ import annotations

import json
import logging
from collections import deque
from datetime import datetime, timedelta, timezone
from threading import Lock
from typing import Any

from ..domain.models import (
    ChatMessage,
    CommandKind,
    CommandLifecycleState,
    CommandRecord,
    CommandRiskLevel,
    JointPosition,
    RuntimeMode,
    RuntimeSnapshot,
)
from ..domain.state_machine import is_terminal_command_state
from .audit_service import AuditService
from .hardware_gate import HardwareGateEvaluator
from .intent_resolution import IntentResolutionService
from .session_lock_service import (
    LeaseNotOwnedError,
    LeaseRejectedError,
    SessionLockService,
)
from .supervisor_execution import SupervisorExecutionMixin
from .supervisor_lifecycle import SupervisorLifecycleMixin
from .supervisor_sequence import SupervisorSequenceMixin
from .supervisor_submission import SupervisorSubmissionMixin
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
    SupervisorSubmissionMixin,
    SupervisorLifecycleMixin,
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
        self._hardware_gate_evaluator = (
            hardware_gate_evaluator or HardwareGateEvaluator()
        )
        self._intent_resolution = intent_resolution_service or IntentResolutionService(
            ros_adapter=ros_adapter
        )
        self._sim_auto_confirm = bool(sim_auto_confirm)
        self._telemetry: TelemetryBridgeService | None = None
        self._confirmation_window = timedelta(seconds=confirmation_window_sec)
        self._commands: dict[str, CommandRecord] = {}
        self._active_command_id: str | None = None
        self._active_sequence_id: str | None = None
        self._messages: deque[ChatMessage] = deque(maxlen=200)
        self._lock = Lock()

    @property
    def ros_adapter(self) -> Any:
        return self._ros

    def _trace(self, stage: str, **fields: Any) -> None:
        rendered_fields: list[str] = []
        for key, value in fields.items():
            if value is None:
                continue
            if isinstance(
                value, (CommandLifecycleState, RuntimeMode, CommandRiskLevel)
            ):
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
        active_command = (
            self._commands.get(self._active_command_id)
            if self._active_command_id
            else None
        )
        active_sequence = (
            self._commands.get(self._active_sequence_id)
            if self._active_sequence_id
            else None
        )
        replay_items = self._audit.list_commands(limit=25, top_level_only=True)
        return {
            "capabilities": self._bridge_capabilities().to_dict(),
            "lease": self._serialize_lease_view(session_id, operator_id),
            "messages": [message.to_dict() for message in self._messages],
            "activeCommand": self._serialize_command(active_command),
            "activeSequence": self._serialize_sequence(active_sequence),
            "planMetrics": self._serialize_metrics(
                active_command.metrics if active_command else None
            ),
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
            payload={
                "force_takeover": force_takeover,
                "takeover_reason": takeover_reason,
            },
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
        if (
            detail["command"].get("command_kind") or CommandKind.COMMAND.value
        ) != CommandKind.COMMAND.value:
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
        if (
            detail["command"].get("command_kind") or CommandKind.COMMAND.value
        ) != CommandKind.SEQUENCE.value:
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
                "timeline": [
                    self._serialize_timeline_row(row) for row in detail["timeline"]
                ],
                "runtimeEvents": [
                    self._serialize_runtime_row(row) for row in detail["runtime_events"]
                ],
            }
        return {
            "jobType": CommandKind.COMMAND.value,
            "command": self._serialize_audited_command(detail["command"]),
            "sequence": None,
            "timeline": [
                self._serialize_timeline_row(row) for row in detail["timeline"]
            ],
            "runtimeEvents": [
                self._serialize_runtime_row(row) for row in detail["runtime_events"]
            ],
        }

    def _assert_controller(
        self,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
    ) -> Any:
        try:
            return self._session_lock.assert_controller(
                session_id, operator_id, lease_token
            )
        except LeaseNotOwnedError as exc:
            raise ForbiddenActionError(str(exc)) from exc

    def _assert_no_active_job(self) -> None:
        active_sequence = (
            self._commands.get(self._active_sequence_id)
            if self._active_sequence_id
            else None
        )
        if active_sequence is not None and not is_terminal_command_state(
            active_sequence.lifecycle_state
        ):
            raise ConflictError("another sequence is already pending or executing")
        active_command = (
            self._commands.get(self._active_command_id)
            if self._active_command_id
            else None
        )
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
