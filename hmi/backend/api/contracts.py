from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, model_validator

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
    canCancelCommands: bool = False
    canAbortCommands: bool
    commandIngressAvailable: bool = False
    confirmationAvailable: bool = False
    executionAllowed: bool = False
    replayAvailable: bool = False
    simOnly: bool = False
    hardwareGate: HardwareGateStatusModel


class HardwareGateChecklistModel(StrictModel):
    timingJitter: bool
    disconnectReconnect: bool
    robotStatusSemantics: bool
    jointSourcePrecedence: bool
    auditVisibility: bool


class HardwareGateStatusModel(StrictModel):
    unlocked: bool
    reasons: list[str]
    flagEnabled: bool
    evidencePath: str
    approvedBy: str | None
    approvedAt: str | None
    reportPath: str | None
    reportSha256: str | None
    reportSha256Match: bool
    checklist: HardwareGateChecklistModel | None


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


class ChatMessageModel(StrictModel):
    id: str
    commandId: str | None
    origin: Literal['system', 'operator', 'assistant']
    timestamp: str
    text: str
    tag: str | None = None


class PlanMetricsModel(StrictModel):
    score: float | None
    pathLengthRad: float | None
    smoothness: float | None
    clearanceM: float | None
    cartesianCompletionPct: float | None
    replanCount: int | None


class ValidationSourceStatusModel(StrictModel):
    name: str
    label: str
    topic: str
    freshnessState: Literal['fresh', 'stale', 'unavailable']
    active: bool
    preferred: bool
    detail: str | None


class CommandValidationResultModel(StrictModel):
    accepted: bool
    leaseValid: bool
    runtimeAllowed: bool
    telemetryFresh: bool
    requiresConfirmation: bool
    riskLevel: Literal['low', 'medium', 'high', 'critical'] | None
    blockingReasons: list[str]
    confirmationReasons: list[str]
    planFingerprint: str | None
    executionAllowedNow: bool
    criticalSources: list[ValidationSourceStatusModel]
    optionalSources: list[ValidationSourceStatusModel]
    eventDrivenSources: list[ValidationSourceStatusModel]
    hardwareGate: HardwareGateStatusModel
    preflight: dict[str, Any]


class CommandExecutionResultModel(StrictModel):
    accepted: bool
    adapter: str
    status: str
    summary: str
    dispatchedToRos: bool
    commandId: str | None = None
    planFingerprint: str | None = None
    operatorId: str | None = None
    sessionId: str | None = None
    leaseId: str | None = None
    correlationId: str | None = None


class CommandViewModel(StrictModel):
    commandId: str
    commandKind: Literal['command']
    sessionId: str
    operatorId: str
    rawText: str
    intentSource: Literal['text', 'structured']
    structuredIntent: dict[str, Any] | None = None
    lifecycleState: Literal[
        'RECEIVED',
        'PARSING',
        'VALIDATING',
        'NEEDS_CONFIRMATION',
        'CONFIRMED',
        'EXECUTION_REQUESTED',
        'EXECUTING',
        'SUCCEEDED',
        'FAILED',
        'REJECTED',
        'CANCELLED',
        'EXPIRED',
    ]
    summaryLabel: str
    plannerUsed: str | None
    frameUsed: str | None
    mode: Literal['sim', 'hardware', 'unknown']
    riskLevel: Literal['low', 'medium', 'high', 'critical'] | None = None
    planFingerprint: str | None = None
    correlationId: str | None = None
    rejectReason: str | None
    parsedIntent: dict[str, Any] | None = None
    validationResult: CommandValidationResultModel | None = None
    planSummary: dict[str, Any] | None = None
    metrics: PlanMetricsModel | None = None
    confirmationExpiresAt: str | None = None
    createdAt: str
    confirmAt: str | None
    executeAt: str | None
    executionResult: CommandExecutionResultModel | None = None
    finalState: Literal['SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELLED', 'EXPIRED'] | None = None
    parentSequenceId: str | None = None
    sequenceStepIndex: int | None = None
    sequenceStepCount: int | None = None


class SequenceViewModel(StrictModel):
    sequenceId: str
    commandKind: Literal['sequence']
    sessionId: str
    operatorId: str
    rawText: str
    intentSource: Literal['text', 'structured']
    structuredIntent: dict[str, Any] | None = None
    lifecycleState: Literal[
        'RECEIVED',
        'PARSING',
        'VALIDATING',
        'NEEDS_CONFIRMATION',
        'CONFIRMED',
        'EXECUTION_REQUESTED',
        'EXECUTING',
        'SUCCEEDED',
        'FAILED',
        'REJECTED',
        'CANCELLED',
        'EXPIRED',
    ]
    summaryLabel: str
    plannerUsed: str | None
    frameUsed: str | None
    mode: Literal['sim', 'hardware', 'unknown']
    riskLevel: Literal['low', 'medium', 'high', 'critical'] | None = None
    planFingerprint: str | None = None
    correlationId: str | None = None
    rejectReason: str | None
    validationResult: CommandValidationResultModel | None = None
    planSummary: dict[str, Any] | None = None
    metrics: PlanMetricsModel | None = None
    confirmationExpiresAt: str | None = None
    createdAt: str
    confirmAt: str | None
    executeAt: str | None
    executionResult: CommandExecutionResultModel | None = None
    finalState: Literal['SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELLED', 'EXPIRED'] | None = None
    stepCount: int
    currentStepIndex: int | None = None
    diagnostics: list[str] = Field(default_factory=list)
    manualRecoveryRequired: bool = False
    steps: list[CommandViewModel] = Field(default_factory=list)


class ReplayListItemModel(StrictModel):
    commandId: str
    kind: Literal['command', 'sequence']
    sessionId: str
    operatorId: str
    summaryLabel: str
    lifecycleState: str
    finalState: str | None
    plannerUsed: str | None
    frameUsed: str | None
    mode: Literal['sim', 'hardware', 'unknown']
    createdAt: str
    executeAt: str | None
    riskLevel: Literal['low', 'medium', 'high', 'critical'] | None = None
    stepCount: int | None = None
    currentStepIndex: int | None = None
    manualRecoveryRequired: bool = False


class TimelineEventModel(StrictModel):
    id: str
    commandId: str | None
    timestamp: str
    fromState: str | None
    toState: str | None
    runtimeState: str | None
    message: str
    payload: dict[str, Any] | None = None


class ReplayDetailModel(StrictModel):
    jobType: Literal['command', 'sequence']
    command: CommandViewModel | None = None
    sequence: SequenceViewModel | None = None
    timeline: list[TimelineEventModel]
    runtimeEvents: list[TimelineEventModel]


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
    messages: list[ChatMessageModel]
    activeCommand: CommandViewModel | None
    activeSequence: SequenceViewModel | None = None
    jointPositions: list[JointPositionModel]
    planMetrics: PlanMetricsModel | None
    replayItems: list[ReplayListItemModel]


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


class LeaseAcquireRequestModel(StrictModel):
    sessionId: str
    operatorId: str
    requestedRole: Literal['controller', 'observer']
    forceTakeover: bool = False
    takeoverReason: str | None = None


class LeaseRenewRequestModel(StrictModel):
    sessionId: str
    operatorId: str
    leaseToken: str


class LeaseReleaseRequestModel(StrictModel):
    sessionId: str
    operatorId: str
    leaseToken: str


class LeaseMutationResponseModel(StrictModel):
    accepted: bool
    lease: LeaseViewModel
    reason: str | None


class CommandIntentRequestModel(StrictModel):
    sessionId: str
    operatorId: str
    leaseToken: str | None
    intentText: str | None = None
    structuredIntent: dict[str, Any] | None = None
    mode: Literal['sim', 'hardware', 'unknown']

    @model_validator(mode='after')
    def validate_intent_payload(self) -> 'CommandIntentRequestModel':
        if not (self.intentText and self.intentText.strip()) and self.structuredIntent is None:
            raise ValueError('intentText or structuredIntent is required')
        return self


class CommandConfirmRequestModel(StrictModel):
    sessionId: str
    operatorId: str
    leaseToken: str | None
    planFingerprint: str


class CommandCancelRequestModel(StrictModel):
    sessionId: str
    operatorId: str
    leaseToken: str | None
    reason: str | None = None


class CommandMutationResponseModel(StrictModel):
    accepted: bool
    jobType: Literal['command', 'sequence']
    commandId: str | None
    sequenceId: str | None = None
    reason: str | None
    snapshot: HmiStateSnapshotModel | None = None
    command: CommandViewModel | None = None
    sequence: SequenceViewModel | None = None


class CommandListResponseModel(StrictModel):
    items: list[ReplayListItemModel]


class SnapshotStreamEventModel(StrictModel):
    type: Literal['snapshot']
    snapshot: HmiStateSnapshotModel


class HeartbeatStreamEventModel(StrictModel):
    type: Literal['heartbeat']
    schemaVersion: Literal[SNAPSHOT_SCHEMA_VERSION]
    generatedAt: str
    transportState: Literal['connected', 'connecting', 'disconnected']
    telemetryState: Literal['fresh', 'stale', 'unavailable']


class LeaseStateStreamEventModel(StrictModel):
    type: Literal['lease_state']
    lease: LeaseViewModel
    capabilities: BridgeCapabilitiesModel


class CommandLifecycleStreamEventModel(StrictModel):
    type: Literal['command_lifecycle']
    command: CommandViewModel
    messages: list[ChatMessageModel] = Field(default_factory=list)
    planMetrics: PlanMetricsModel | None = None


class SequenceLifecycleStreamEventModel(StrictModel):
    type: Literal['sequence_lifecycle']
    sequence: SequenceViewModel
    messages: list[ChatMessageModel] = Field(default_factory=list)


class ReplayUpdatedStreamEventModel(StrictModel):
    type: Literal['replay_updated']
    replayItems: list[ReplayListItemModel]


# ── Jog Pendant Models ──────────────────────────────────────────────────────

class JogBridgeStatusModel(StrictModel):
    state: str
    pointsQueued: int
    effectiveHz: float
    robotReady: bool
    servoActive: bool
    bridgeActive: bool
    lastError: str
    rejectionReason: str


class JogBridgeStatusStreamEventModel(StrictModel):
    type: Literal['jog_bridge_status']
    jogBridgeStatus: JogBridgeStatusModel


class JogCommandRequestModel(StrictModel):
    jointIndex: int = Field(..., ge=0, le=5)
    direction: Literal[-1, 1]
    mode: Literal['continuous', 'discrete']
    velocityScale: float = Field(..., ge=0.0, le=0.3)
    stepDegrees: float = Field(0.0, ge=0.0, le=10.0)


HMI_STREAM_EVENT_ADAPTER = TypeAdapter(
    SnapshotStreamEventModel
    | HeartbeatStreamEventModel
    | LeaseStateStreamEventModel
    | CommandLifecycleStreamEventModel
    | SequenceLifecycleStreamEventModel
    | ReplayUpdatedStreamEventModel
    | JogBridgeStatusStreamEventModel
)
