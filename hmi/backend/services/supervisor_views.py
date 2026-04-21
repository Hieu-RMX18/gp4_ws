from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any
from uuid import uuid4

from ..domain.models import (
    BridgeCapabilities,
    ChatMessage,
    CommandLifecycleState,
    CommandRecord,
    LeaseRole,
    PlanMetrics,
    RuntimeMode,
)


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class SupervisorViewsMixin:
    def _bridge_capabilities(self) -> BridgeCapabilities:
        runtime = self._current_runtime()
        hardware_gate = self._hardware_gate_evaluator.evaluate()
        if runtime.mode == RuntimeMode.SIM:
            command_ingress_available = True
            confirmation_available = True
            sim_only = True
        elif runtime.mode == RuntimeMode.HARDWARE:
            command_ingress_available = hardware_gate.unlocked
            confirmation_available = hardware_gate.unlocked
            sim_only = not hardware_gate.unlocked
        else:
            command_ingress_available = False
            confirmation_available = False
            sim_only = True
        return BridgeCapabilities(
            read_only=not command_ingress_available,
            can_acquire_lease=command_ingress_available,
            can_submit_commands=command_ingress_available,
            can_confirm_commands=confirmation_available,
            can_cancel_commands=confirmation_available,
            can_abort_commands=confirmation_available,
            command_ingress_available=command_ingress_available,
            confirmation_available=confirmation_available,
            execution_allowed=command_ingress_available,
            replay_available=True,
            sim_only=sim_only,
            hardware_gate=hardware_gate,
        )

    def _serialize_lease_view(self, session_id: str, operator_id: str) -> dict[str, Any]:
        capabilities = self._bridge_capabilities()
        runtime = self._current_runtime()
        lease = self._session_lock.current_controller()
        if lease is None:
            if runtime.mode == RuntimeMode.HARDWARE:
                status_text = (
                    "Hardware command ingress locked — "
                    + (
                        capabilities.hardware_gate.reasons[0]
                        if capabilities.hardware_gate.reasons
                        else "dual hardware gate is not satisfied."
                    )
                )
            else:
                status_text = (
                    "Observer mode — request the supervisor lease before submitting commands."
                    if capabilities.can_acquire_lease
                    else "Read-only telemetry mode — command lease acquisition is disabled."
                )
            return {
                "leaseId": None,
                "leaseToken": None,
                "role": LeaseRole.OBSERVER.value,
                "ownsControl": False,
                "holderOperatorId": None,
                "holderSessionId": None,
                "acquiredAt": None,
                "expiresAt": None,
                "statusText": status_text,
                "canForceTakeover": capabilities.can_acquire_lease,
            }

        owns_control = lease.session_id == session_id and lease.operator_id == operator_id
        if capabilities.read_only:
            role = LeaseRole.OBSERVER.value
            lease_token: str | None = None
            owns_control = False
            if runtime.mode == RuntimeMode.HARDWARE:
                status_text = (
                    "Hardware command ingress locked — "
                    + (
                        capabilities.hardware_gate.reasons[0]
                        if capabilities.hardware_gate.reasons
                        else "dual hardware gate is not satisfied."
                    )
                )
            else:
                status_text = "Read-only telemetry mode — command lease is disabled."
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
            timestamp=_utcnow().strftime("%H:%M:%S"),
            text=text,
            tag=tag,
        )
        self._messages.append(message)
        return message

    def _decode_json(self, payload: str | None) -> Any:
        if not payload:
            return None
        return json.loads(payload)
