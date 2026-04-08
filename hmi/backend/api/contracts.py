from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, TypeAdapter

from ..services.telemetry_bridge_service import SNAPSHOT_SCHEMA_VERSION


class StrictModel(BaseModel):
    model_config = ConfigDict(extra='forbid')


class BridgeConnectionModel(StrictModel):
    name: Literal['ros2', 'moveit2', 'llm', 'motoros2']
    label: str
    health: Literal['healthy', 'degraded', 'down']


class LeaseViewModel(StrictModel):
    leaseId: str | None
    leaseToken: str | None
    role: Literal['controller', 'observer']
    ownsControl: bool
    holderOperatorId: str | None
    holderSessionId: str | None
    acquiredAt: str | None
    expiresAt: str | None
    statusText: str
    canForceTakeover: bool


class BridgeCapabilitiesModel(StrictModel):
    readOnly: bool
    canAcquireLease: bool
    canSubmitCommands: bool
    canConfirmCommands: bool
    canAbortCommands: bool


class JointPositionModel(StrictModel):
    name: str
    positionDeg: float | None
    minDeg: float
    maxDeg: float


class TelemetrySourceStatusModel(StrictModel):
    name: str
    label: str
    topic: str
    lastSeenAt: str | None
    freshnessThresholdSec: float
    freshnessState: Literal['fresh', 'stale', 'unavailable']
    preferred: bool
    active: bool
    detail: str | None


class RobotStatusSnapshotModel(StrictModel):
    servoState: Literal['ON', 'OFF', 'UNKNOWN']
    eStop: Literal['CLEAR', 'ACTIVE', 'UNKNOWN']
    alarmState: Literal['NONE', 'ACTIVE', 'UNKNOWN']
    motionMode: str | None
    trajectoryPointsUsed: int | None
    trajectoryPointsCapacity: int | None
    readinessMessage: str


class RuntimeSnapshotModel(StrictModel):
    systemState: Literal['NORMAL', 'FAULT', 'ESTOP', 'HOLD', 'TIMEOUT', 'LOST_CONN', 'SAFETY_BLOCKED']
    blocking: bool
    statusText: str
    mode: Literal['sim', 'hardware', 'unknown']
    robotStatus: RobotStatusSnapshotModel


class HmiStateSnapshotModel(StrictModel):
    schemaVersion: Literal[SNAPSHOT_SCHEMA_VERSION]
    generatedAt: str
    transportState: Literal['connected', 'connecting', 'disconnected']
    telemetryState: Literal['fresh', 'stale', 'unavailable']
    telemetrySources: list[TelemetrySourceStatusModel]
    mode: Literal['sim', 'hardware', 'unknown']
    connections: list[BridgeConnectionModel]
    capabilities: BridgeCapabilitiesModel
    lease: LeaseViewModel
    runtime: RuntimeSnapshotModel
    messages: list[dict[str, Any]]
    activeCommand: dict[str, Any] | None
    jointPositions: list[JointPositionModel]
    planMetrics: dict[str, Any] | None
    replayItems: list[dict[str, Any]]


class RuntimeStateResponseModel(StrictModel):
    schemaVersion: Literal[SNAPSHOT_SCHEMA_VERSION]
    generatedAt: str
    telemetryState: Literal['fresh', 'stale', 'unavailable']
    telemetrySources: list[TelemetrySourceStatusModel]
    runtime: RuntimeSnapshotModel
    jointPositions: list[JointPositionModel]


class ConnectionStateResponseModel(StrictModel):
    schemaVersion: Literal[SNAPSHOT_SCHEMA_VERSION]
    generatedAt: str
    transportState: Literal['connected', 'connecting', 'disconnected']
    telemetryState: Literal['fresh', 'stale', 'unavailable']
    telemetrySources: list[TelemetrySourceStatusModel]
    connections: list[BridgeConnectionModel]


class LeaseStateResponseModel(StrictModel):
    schemaVersion: Literal[SNAPSHOT_SCHEMA_VERSION]
    generatedAt: str
    capabilities: BridgeCapabilitiesModel
    lease: LeaseViewModel


class SnapshotStreamEventModel(StrictModel):
    type: Literal['snapshot']
    snapshot: HmiStateSnapshotModel


class HeartbeatStreamEventModel(StrictModel):
    type: Literal['heartbeat']
    schemaVersion: Literal[SNAPSHOT_SCHEMA_VERSION]
    generatedAt: str
    transportState: Literal['connected', 'connecting', 'disconnected']
    telemetryState: Literal['fresh', 'stale', 'unavailable']


READ_ONLY_STREAM_EVENT_ADAPTER = TypeAdapter(SnapshotStreamEventModel | HeartbeatStreamEventModel)
