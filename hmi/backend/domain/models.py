from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any


class RuntimeMode(str, Enum):
    SIM = "sim"
    HARDWARE = "hardware"
    UNKNOWN = "unknown"


class LeaseRole(str, Enum):
    CONTROLLER = "controller"
    OBSERVER = "observer"


class CommandLifecycleState(str, Enum):
    RECEIVED = "RECEIVED"
    PARSING = "PARSING"
    VALIDATING = "VALIDATING"
    NEEDS_CONFIRMATION = "NEEDS_CONFIRMATION"
    CONFIRMED = "CONFIRMED"
    EXECUTION_REQUESTED = "EXECUTION_REQUESTED"
    EXECUTING = "EXECUTING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    REJECTED = "REJECTED"
    CANCELLED = "CANCELLED"
    EXPIRED = "EXPIRED"


class CommandRiskLevel(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class CommandKind(str, Enum):
    COMMAND = "command"
    SEQUENCE = "sequence"


class SystemRuntimeState(str, Enum):
    NORMAL = "NORMAL"
    FAULT = "FAULT"
    ESTOP = "ESTOP"
    HOLD = "HOLD"
    TIMEOUT = "TIMEOUT"
    LOST_CONN = "LOST_CONN"
    SAFETY_BLOCKED = "SAFETY_BLOCKED"


class ConnectionHealth(str, Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    DOWN = "down"


class TelemetryFreshnessState(str, Enum):
    FRESH = "fresh"
    STALE = "stale"
    UNAVAILABLE = "unavailable"


@dataclass(slots=True)
class BridgeConnection:
    name: str
    label: str
    health: ConnectionHealth


@dataclass(slots=True)
class BridgeCapabilities:
    read_only: bool = True
    can_acquire_lease: bool = False
    can_submit_commands: bool = False
    can_confirm_commands: bool = False
    can_cancel_commands: bool = False
    can_abort_commands: bool = False
    command_ingress_available: bool = False
    confirmation_available: bool = False
    execution_allowed: bool = False
    replay_available: bool = False
    sim_only: bool = False
    hardware_gate: dict[str, Any] = field(
        default_factory=lambda: {"unlocked": True, "reasons": [], "flagEnabled": True}
    )

    def to_dict(self) -> dict[str, Any]:
        return {
            "readOnly": self.read_only,
            "canAcquireLease": self.can_acquire_lease,
            "canSubmitCommands": self.can_submit_commands,
            "canConfirmCommands": self.can_confirm_commands,
            "canCancelCommands": self.can_cancel_commands,
            "canAbortCommands": self.can_abort_commands,
            "commandIngressAvailable": self.command_ingress_available,
            "confirmationAvailable": self.confirmation_available,
            "executionAllowed": self.execution_allowed,
            "replayAvailable": self.replay_available,
            "simOnly": self.sim_only,
            "hardwareGate": dict(self.hardware_gate),
        }


@dataclass(slots=True)
class LeaseRecord:
    lease_id: str
    lease_token: str
    role: LeaseRole
    session_id: str
    operator_id: str
    acquired_at: datetime
    expires_at: datetime
    force_takeover: bool = False
    takeover_reason: str | None = None

    def is_expired(self, now: datetime) -> bool:
        return self.expires_at <= now


@dataclass(slots=True)
class PlanMetrics:
    score: float | None = None
    path_length_rad: float | None = None
    smoothness: float | None = None
    clearance_m: float | None = None
    cartesian_completion_pct: float | None = None
    replan_count: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class RobotStatusSnapshot:
    servo_state: str = "UNKNOWN"
    e_stop: str = "UNKNOWN"
    alarm_state: str = "UNKNOWN"
    motion_mode: str | None = None
    trajectory_points_used: int | None = None
    trajectory_points_capacity: int | None = None
    readiness_message: str = "No backend runtime snapshot available."

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class RuntimeSnapshot:
    system_state: SystemRuntimeState = SystemRuntimeState.LOST_CONN
    blocking: bool = True
    status_text: str = "Telemetry bridge disconnected."
    mode: RuntimeMode = RuntimeMode.UNKNOWN
    robot_status: RobotStatusSnapshot = field(default_factory=RobotStatusSnapshot)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["system_state"] = self.system_state.value
        payload["mode"] = self.mode.value
        return payload


@dataclass(slots=True)
class JointPosition:
    name: str
    position_deg: float | None
    min_deg: float = -180.0
    max_deg: float = 180.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class TelemetrySourceSnapshot:
    name: str
    label: str
    topic: str
    last_seen_at: datetime | None
    freshness_threshold_sec: float
    freshness_state: TelemetryFreshnessState
    preferred: bool = False
    active: bool = True
    detail: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "label": self.label,
            "topic": self.topic,
            "lastSeenAt": self.last_seen_at.isoformat() if self.last_seen_at else None,
            "freshnessThresholdSec": self.freshness_threshold_sec,
            "freshnessState": self.freshness_state.value,
            "preferred": self.preferred,
            "active": self.active,
            "detail": self.detail,
        }


@dataclass(slots=True)
class ChatMessage:
    message_id: str
    command_id: str | None
    origin: str
    timestamp: str
    text: str
    tag: str | None = None
    source: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.message_id,
            "commandId": self.command_id,
            "origin": self.origin,
            "timestamp": self.timestamp,
            "text": self.text,
            "tag": self.tag,
            "source": self.source,
        }


@dataclass(slots=True)
class CommandRecord:
    command_id: str
    session_id: str
    operator_id: str
    raw_text: str
    lifecycle_state: CommandLifecycleState
    summary_label: str
    mode: RuntimeMode
    created_at: datetime
    command_kind: CommandKind = CommandKind.COMMAND
    intent_source: str = "text"
    correlation_id: str | None = None
    planner_used: str | None = None
    frame_used: str | None = None
    risk_level: CommandRiskLevel | None = None
    plan_fingerprint: str | None = None
    reject_reason: str | None = None
    structured_intent: dict[str, Any] | None = None
    parsed_intent: dict[str, Any] | None = None
    validation_result: dict[str, Any] | None = None
    plan_summary: dict[str, Any] | None = None
    metrics: PlanMetrics | None = None
    confirmation_expires_at: datetime | None = None
    confirm_at: datetime | None = None
    execute_at: datetime | None = None
    execution_result: dict[str, Any] | None = None
    final_state: CommandLifecycleState | None = None
    parent_sequence_id: str | None = None
    sequence_step_index: int | None = None
    sequence_step_count: int | None = None
    current_step_index: int | None = None
    sequence_diagnostics: list[str] = field(default_factory=list)
    child_command_ids: list[str] = field(default_factory=list)
    manual_recovery_required: bool = False
    pipeline_traces: list[dict[str, Any]] = field(default_factory=list)


@dataclass(slots=True)
class TimelineEvent:
    event_id: str
    command_id: str | None
    timestamp: datetime
    from_state: CommandLifecycleState | None
    to_state: CommandLifecycleState | None
    runtime_state: SystemRuntimeState | None
    message: str
    payload: dict[str, Any] | None = None
