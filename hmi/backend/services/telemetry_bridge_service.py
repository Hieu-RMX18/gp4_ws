from __future__ import annotations

import asyncio
import json
from contextlib import suppress
from copy import deepcopy
from datetime import datetime, timezone
from threading import Lock
from typing import Any, Callable

from ..domain.models import (
    BridgeCapabilities,
    BridgeConnection,
    JointPosition,
    RuntimeSnapshot,
    TelemetryFreshnessState,
    TelemetrySourceSnapshot,
)
from .audit_service import AuditService
from .session_lock_service import SessionLockService


def utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


SNAPSHOT_SCHEMA_VERSION = 'telemetry.v1'


class TelemetryBridgeService:
    """Read-only HMI bridge service.

    This service owns no control-capable ROS path. It only exposes aggregated
    telemetry snapshots from the ROS adapter and audit-logs state changes.
    """

    def __init__(
        self,
        *,
        audit_service: AuditService,
        session_lock_service: SessionLockService,
        ros_adapter: Any,
        poll_interval_sec: float = 0.5,
        heartbeat_interval_sec: float = 5.0,
    ) -> None:
        self._audit = audit_service
        self._session_lock = session_lock_service
        self._ros = ros_adapter
        self._poll_interval_sec = poll_interval_sec
        self._heartbeat_interval_sec = heartbeat_interval_sec
        self._capabilities = BridgeCapabilities()
        self._snapshot_overlay_provider: Callable[[str, str], dict[str, Any]] | None = None
        self._subscribers: dict[asyncio.Queue[dict[str, Any]], tuple[str, str]] = {}
        self._lock = Lock()
        self._stop_event = asyncio.Event()
        self._task: asyncio.Task[None] | None = None
        self._latest_base_snapshot: dict[str, Any] = self._build_base_snapshot()
        self._latest_fingerprint = self._fingerprint(self._latest_base_snapshot)

    async def start(self) -> None:
        self._ros.start()
        self._latest_base_snapshot = self._build_base_snapshot()
        self._latest_fingerprint = self._fingerprint(self._latest_base_snapshot)
        self._audit.record_telemetry_snapshot(
            transport_state=self._latest_base_snapshot["transportState"],
            runtime_state=self._ros.read_runtime_snapshot().system_state,
            payload=self._latest_base_snapshot,
        )
        self._stop_event = asyncio.Event()
        self._task = asyncio.create_task(self._poll_loop())

    async def stop(self) -> None:
        self._stop_event.set()
        if self._task is not None:
            with suppress(asyncio.CancelledError):
                await self._task
            self._task = None
        self._ros.stop()

    @property
    def heartbeat_interval_sec(self) -> float:
        return self._heartbeat_interval_sec

    def set_capabilities(self, capabilities: BridgeCapabilities) -> None:
        self._capabilities = capabilities

    def set_snapshot_overlay_provider(
        self,
        provider: Callable[[str, str], dict[str, Any]],
    ) -> None:
        self._snapshot_overlay_provider = provider

    def subscribe(self, session_id: str, operator_id: str) -> asyncio.Queue[dict[str, Any]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=8)
        with self._lock:
            self._subscribers[queue] = (session_id, operator_id)
        return queue

    def unsubscribe(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        with self._lock:
            self._subscribers.pop(queue, None)

    def subscriber_count(self) -> int:
        with self._lock:
            return len(self._subscribers)

    def get_snapshot(self, session_id: str, operator_id: str) -> dict[str, Any]:
        base_snapshot, _ = self._refresh_if_needed()
        return self._build_hmi_snapshot(base_snapshot, session_id, operator_id)

    def get_runtime_state(self, session_id: str, operator_id: str) -> dict[str, Any]:
        snapshot = self.get_snapshot(session_id, operator_id)
        return {
            "schemaVersion": snapshot["schemaVersion"],
            "generatedAt": snapshot["generatedAt"],
            "telemetryState": snapshot["telemetryState"],
            "telemetrySources": deepcopy(snapshot["telemetrySources"]),
            "runtime": snapshot["runtime"],
            "jointPositions": snapshot["jointPositions"],
        }

    def get_connection_state(self) -> dict[str, Any]:
        base_snapshot, _ = self._refresh_if_needed()
        return {
            "schemaVersion": base_snapshot["schemaVersion"],
            "generatedAt": base_snapshot["generatedAt"],
            "transportState": base_snapshot["transportState"],
            "telemetryState": base_snapshot["telemetryState"],
            "telemetrySources": deepcopy(base_snapshot["telemetrySources"]),
            "connections": deepcopy(base_snapshot["connections"]),
        }

    def get_lease_state(self, session_id: str, operator_id: str) -> dict[str, Any]:
        snapshot = self.get_snapshot(session_id, operator_id)
        return {
            "schemaVersion": snapshot["schemaVersion"],
            "generatedAt": snapshot["generatedAt"],
            "capabilities": deepcopy(snapshot["capabilities"]),
            "lease": deepcopy(snapshot["lease"]),
        }

    async def _poll_loop(self) -> None:
        while not self._stop_event.is_set():
            snapshot, changed = self._refresh_if_needed()
            if changed:
                await self._broadcast_snapshot(snapshot)
            try:
                await asyncio.wait_for(
                    self._stop_event.wait(),
                    timeout=self._poll_interval_sec,
                )
            except asyncio.TimeoutError:
                continue

    def _refresh_if_needed(self) -> tuple[dict[str, Any], bool]:
        next_snapshot = self._build_base_snapshot()
        next_fingerprint = self._fingerprint(next_snapshot)
        if next_fingerprint == self._latest_fingerprint:
            return deepcopy(next_snapshot), False

        previous = self._latest_base_snapshot
        self._record_state_changes(previous, next_snapshot)
        self._latest_base_snapshot = next_snapshot
        self._latest_fingerprint = next_fingerprint
        self._audit.record_telemetry_snapshot(
            transport_state=next_snapshot["transportState"],
            runtime_state=self._ros.read_runtime_snapshot().system_state,
            payload=next_snapshot,
        )
        return deepcopy(next_snapshot), True

    def _build_base_snapshot(self) -> dict[str, Any]:
        runtime = self._ros.read_runtime_snapshot()
        connections = self._ros.read_connections()
        joints = self._ros.read_joint_positions()
        read_source_statuses = getattr(self._ros, "read_source_statuses", None)
        source_statuses = read_source_statuses() if callable(read_source_statuses) else []
        transport_state = self._transport_state(connections)
        return {
            "schemaVersion": SNAPSHOT_SCHEMA_VERSION,
            "generatedAt": utcnow_iso(),
            "transportState": transport_state,
            "telemetryState": self._telemetry_state(transport_state, source_statuses),
            "mode": runtime.mode.value,
            "connections": [self._serialize_connection(connection) for connection in connections],
            "runtime": self._serialize_runtime(runtime),
            "jointPositions": [self._serialize_joint(joint) for joint in joints],
            "telemetrySources": [self._serialize_source_status(source_status) for source_status in source_statuses],
        }

    def _build_hmi_snapshot(
        self,
        base_snapshot: dict[str, Any],
        session_id: str,
        operator_id: str,
    ) -> dict[str, Any]:
        overlay = (
            deepcopy(self._snapshot_overlay_provider(session_id, operator_id))
            if self._snapshot_overlay_provider is not None
            else {}
        )
        snapshot = {
            **deepcopy(base_snapshot),
            "capabilities": self._capabilities.to_dict(),
            "lease": self._serialize_lease_view(session_id, operator_id),
            "messages": [],
            "activeCommand": None,
            "planMetrics": None,
            "replayItems": [],
        }
        snapshot.update(overlay)
        return snapshot

    async def _broadcast_snapshot(self, base_snapshot: dict[str, Any]) -> None:
        with self._lock:
            subscribers = list(self._subscribers.items())
        if not subscribers:
            return

        for queue, (session_id, operator_id) in subscribers:
            payload = {
                "type": "snapshot",
                "snapshot": self._build_hmi_snapshot(base_snapshot, session_id, operator_id),
            }
            try:
                if queue.full():
                    queue.get_nowait()
                queue.put_nowait(payload)
            except asyncio.QueueFull:
                continue

    def broadcast_event(
        self,
        event_factory: Callable[[str, str], dict[str, Any] | None],
    ) -> None:
        with self._lock:
            subscribers = list(self._subscribers.items())
        if not subscribers:
            return

        for queue, (session_id, operator_id) in subscribers:
            payload = event_factory(session_id, operator_id)
            if payload is None:
                continue
            try:
                if queue.full():
                    queue.get_nowait()
                queue.put_nowait(payload)
            except asyncio.QueueFull:
                continue

    def get_heartbeat_event(self) -> dict[str, Any]:
        base_snapshot, _ = self._refresh_if_needed()
        return {
            "type": "heartbeat",
            "schemaVersion": base_snapshot["schemaVersion"],
            "generatedAt": utcnow_iso(),
            "transportState": base_snapshot["transportState"],
            "telemetryState": base_snapshot["telemetryState"],
        }

    def _serialize_lease_view(self, session_id: str, operator_id: str) -> dict[str, Any]:
        lease = self._session_lock.current_controller()
        if lease is None:
            return {
                "leaseId": None,
                "leaseToken": None,
                "role": "observer",
                "ownsControl": False,
                "holderOperatorId": None,
                "holderSessionId": None,
                "acquiredAt": None,
                "expiresAt": None,
                "statusText": (
                    "Telemetry bridge v1 is read-only. "
                    "Controller lease acquisition is intentionally unavailable."
                ),
                "canForceTakeover": False,
            }

        owns_control = lease.session_id == session_id and lease.operator_id == operator_id
        return {
            "leaseId": lease.lease_id,
            "leaseToken": None,
            "role": "observer",
            "ownsControl": False,
            "holderOperatorId": lease.operator_id,
            "holderSessionId": lease.session_id,
            "acquiredAt": lease.acquired_at.isoformat(),
            "expiresAt": lease.expires_at.isoformat(),
            "statusText": (
                "Telemetry bridge v1 is read-only. "
                f"An external controller lease is currently held by {lease.operator_id}."
                if not owns_control
                else "Telemetry bridge v1 is read-only. Control is not available from this client."
            ),
            "canForceTakeover": False,
        }

    def _serialize_connection(self, connection: BridgeConnection) -> dict[str, Any]:
        return {
            "name": connection.name,
            "label": connection.label,
            "health": connection.health.value,
        }

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

    def _serialize_source_status(self, source_status: TelemetrySourceSnapshot) -> dict[str, Any]:
        return source_status.to_dict()

    def _telemetry_state(
        self,
        transport_state: str,
        source_statuses: list[TelemetrySourceSnapshot],
    ) -> str:
        if transport_state == "disconnected":
            return TelemetryFreshnessState.UNAVAILABLE.value

        active_sources = [source for source in source_statuses if source.active]
        if any(source.freshness_state != TelemetryFreshnessState.FRESH for source in active_sources):
            return TelemetryFreshnessState.STALE.value

        return TelemetryFreshnessState.FRESH.value

    def _serialize_joint(self, joint: JointPosition) -> dict[str, Any]:
        return {
            "name": joint.name,
            "positionDeg": joint.position_deg,
            "minDeg": joint.min_deg,
            "maxDeg": joint.max_deg,
        }

    def _transport_state(self, connections: list[BridgeConnection]) -> str:
        if not connections:
            return "disconnected"
        if connections[0].health.value == "healthy":
            return "connected"
        if any(connection.health.value != "down" for connection in connections):
            return "connecting"
        return "disconnected"

    def _fingerprint(self, snapshot: dict[str, Any]) -> str:
        stable_snapshot = {
            "transportState": snapshot["transportState"],
            "telemetryState": snapshot["telemetryState"],
            "mode": snapshot["mode"],
            "connections": snapshot["connections"],
            "runtime": snapshot["runtime"],
            "jointPositions": snapshot["jointPositions"],
            "telemetrySources": [
                {
                    "name": source["name"],
                    "freshnessState": source["freshnessState"],
                    "active": source["active"],
                }
                for source in snapshot["telemetrySources"]
            ],
        }
        return json.dumps(stable_snapshot, sort_keys=True, separators=(",", ":"))

    def _record_state_changes(
        self,
        previous: dict[str, Any],
        current: dict[str, Any],
    ) -> None:
        if previous["transportState"] != current["transportState"]:
            self._audit.record_state_transition(
                channel="transportState",
                previous_value=previous["transportState"],
                next_value=current["transportState"],
                payload={"connections": current["connections"]},
            )

        if previous["telemetryState"] != current["telemetryState"]:
            self._audit.record_state_transition(
                channel="telemetryState",
                previous_value=previous["telemetryState"],
                next_value=current["telemetryState"],
                payload={"telemetrySources": current["telemetrySources"]},
            )

        previous_runtime = previous["runtime"]["systemState"]
        current_runtime = current["runtime"]["systemState"]
        if previous_runtime != current_runtime:
            self._audit.record_state_transition(
                channel="runtime.systemState",
                previous_value=previous_runtime,
                next_value=current_runtime,
                payload={"statusText": current["runtime"]["statusText"]},
            )

        previous_connections = {item["name"]: item["health"] for item in previous["connections"]}
        for connection in current["connections"]:
            name = connection["name"]
            previous_health = previous_connections.get(name)
            if previous_health != connection["health"]:
                self._audit.record_state_transition(
                    channel=f"connections.{name}",
                    previous_value=previous_health,
                    next_value=connection["health"],
                    payload={"label": connection["label"]},
                )

        previous_sources = {
            item["name"]: (item["freshnessState"], item["active"])
            for item in previous["telemetrySources"]
        }
        for source in current["telemetrySources"]:
            previous_source = previous_sources.get(source["name"])
            current_source = (source["freshnessState"], source["active"])
            if previous_source != current_source:
                self._audit.record_state_transition(
                    channel=f"telemetrySources.{source['name']}",
                    previous_value=previous_source[0] if previous_source else None,
                    next_value=source["freshnessState"],
                    payload={
                        "topic": source["topic"],
                        "lastSeenAt": source["lastSeenAt"],
                        "active": source["active"],
                    },
                )
