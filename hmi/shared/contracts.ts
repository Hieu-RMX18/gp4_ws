export type RuntimeMode = 'sim' | 'hardware' | 'unknown';
export type TransportState = 'connected' | 'connecting' | 'disconnected';
export type ConnectionHealth = 'healthy' | 'degraded' | 'down';
export type TelemetryState = 'fresh' | 'stale' | 'unavailable';
export type LeaseRole = 'controller' | 'observer';

export type CommandLifecycleState =
  | 'IDLE'
  | 'RECEIVED'
  | 'PARSED'
  | 'VALIDATED'
  | 'PLANNED'
  | 'QUALITY_CHECKED'
  | 'READY_FOR_CONFIRM'
  | 'EXECUTING'
  | 'SUCCEEDED'
  | 'FAILED'
  | 'REJECTED'
  | 'CANCELLED'
  | 'ABORTED';

export type SystemRuntimeState =
  | 'NORMAL'
  | 'FAULT'
  | 'ESTOP'
  | 'HOLD'
  | 'TIMEOUT'
  | 'LOST_CONN'
  | 'SAFETY_BLOCKED';

export type MessageOrigin = 'system' | 'operator' | 'assistant';
export type MessageTag =
  | 'PARSED'
  | 'VALIDATED'
  | 'PLANNED'
  | 'QUALITY_CHECKED'
  | 'READY_FOR_CONFIRM'
  | 'EXECUTING'
  | 'SUCCEEDED'
  | 'FAILED'
  | 'REJECTED'
  | 'DEBUG_REQUIRED';

export interface BridgeConnection {
  name: 'ros2' | 'moveit2' | 'llm' | 'motoros2';
  label: string;
  health: ConnectionHealth;
}

export interface LeaseView {
  leaseId: string | null;
  leaseToken: string | null;
  role: LeaseRole;
  ownsControl: boolean;
  holderOperatorId: string | null;
  holderSessionId: string | null;
  acquiredAt: string | null;
  expiresAt: string | null;
  statusText: string;
  canForceTakeover: boolean;
}

export interface BridgeCapabilities {
  readOnly: boolean;
  canAcquireLease: boolean;
  canSubmitCommands: boolean;
  canConfirmCommands: boolean;
  canAbortCommands: boolean;
}

export interface TelemetrySourceStatus {
  name: string;
  label: string;
  topic: string;
  lastSeenAt: string | null;
  freshnessThresholdSec: number;
  freshnessState: TelemetryState;
  preferred: boolean;
  active: boolean;
  detail: string | null;
}

export interface JointPosition {
  name: string;
  positionDeg: number | null;
  minDeg: number;
  maxDeg: number;
}

export interface PlanMetrics {
  score: number | null;
  pathLengthRad: number | null;
  smoothness: number | null;
  clearanceM: number | null;
  cartesianCompletionPct: number | null;
  replanCount: number | null;
}

export interface RobotStatusSnapshot {
  servoState: 'ON' | 'OFF' | 'UNKNOWN';
  eStop: 'CLEAR' | 'ACTIVE' | 'UNKNOWN';
  alarmState: 'NONE' | 'ACTIVE' | 'UNKNOWN';
  motionMode: string | null;
  trajectoryPointsUsed: number | null;
  trajectoryPointsCapacity: number | null;
  readinessMessage: string;
}

export interface RuntimeSnapshot {
  systemState: SystemRuntimeState;
  blocking: boolean;
  statusText: string;
  mode: RuntimeMode;
  robotStatus: RobotStatusSnapshot;
}

export interface ChatMessage {
  id: string;
  commandId: string | null;
  origin: MessageOrigin;
  timestamp: string;
  text: string;
  tag?: MessageTag;
}

export interface TimelineEvent {
  id: string;
  commandId: string | null;
  timestamp: string;
  fromState: CommandLifecycleState | null;
  toState: CommandLifecycleState | null;
  runtimeState: SystemRuntimeState | null;
  message: string;
  payload?: Record<string, unknown>;
}

export interface CommandView {
  commandId: string;
  sessionId: string;
  operatorId: string;
  rawText: string;
  lifecycleState: CommandLifecycleState;
  summaryLabel: string;
  plannerUsed: string | null;
  frameUsed: string | null;
  mode: RuntimeMode;
  rejectReason: string | null;
  parsedIntent?: Record<string, unknown> | null;
  validationResult?: Record<string, unknown> | null;
  planSummary?: Record<string, unknown> | null;
  metrics?: PlanMetrics | null;
  createdAt: string;
  confirmAt: string | null;
  executeAt: string | null;
  finalState: CommandLifecycleState | null;
}

export interface ReplayListItem {
  commandId: string;
  sessionId: string;
  operatorId: string;
  summaryLabel: string;
  lifecycleState: CommandLifecycleState;
  finalState: CommandLifecycleState | null;
  plannerUsed: string | null;
  frameUsed: string | null;
  mode: RuntimeMode;
  createdAt: string;
  executeAt: string | null;
}

export interface ReplayDetail {
  command: CommandView;
  timeline: TimelineEvent[];
  runtimeEvents: TimelineEvent[];
}

export interface HmiStateSnapshot {
  schemaVersion: string;
  generatedAt: string;
  transportState: TransportState;
  telemetryState: TelemetryState;
  telemetrySources: TelemetrySourceStatus[];
  mode: RuntimeMode;
  connections: BridgeConnection[];
  capabilities: BridgeCapabilities;
  lease: LeaseView;
  runtime: RuntimeSnapshot;
  messages: ChatMessage[];
  activeCommand: CommandView | null;
  jointPositions: JointPosition[];
  planMetrics: PlanMetrics | null;
  replayItems: ReplayListItem[];
}

export interface RuntimeStateResponse {
  schemaVersion: string;
  generatedAt: string;
  telemetryState: TelemetryState;
  telemetrySources: TelemetrySourceStatus[];
  runtime: RuntimeSnapshot;
  jointPositions: JointPosition[];
}

export interface ConnectionStateResponse {
  schemaVersion: string;
  generatedAt: string;
  transportState: TransportState;
  telemetryState: TelemetryState;
  telemetrySources: TelemetrySourceStatus[];
  connections: BridgeConnection[];
}

export interface LeaseStateResponse {
  schemaVersion: string;
  generatedAt: string;
  capabilities: BridgeCapabilities;
  lease: LeaseView;
}

export interface LeaseAcquireRequest {
  sessionId: string;
  operatorId: string;
  requestedRole: LeaseRole;
  forceTakeover?: boolean;
  takeoverReason?: string;
}

export interface LeaseRenewRequest {
  sessionId: string;
  operatorId: string;
  leaseToken: string;
}

export interface LeaseReleaseRequest {
  sessionId: string;
  operatorId: string;
  leaseToken: string;
}

export interface SubmitCommandRequest {
  sessionId: string;
  operatorId: string;
  leaseToken: string | null;
  rawText: string;
  mode: RuntimeMode;
}

export interface CommandActionRequest {
  sessionId: string;
  operatorId: string;
  leaseToken: string | null;
  reason?: string;
}

export interface LeaseMutationResponse {
  accepted: boolean;
  lease: LeaseView;
  reason: string | null;
}

export interface SubmitCommandResponse {
  accepted: boolean;
  commandId: string | null;
  reason: string | null;
  snapshot?: HmiStateSnapshot;
}

export interface CommandActionResponse {
  accepted: boolean;
  commandId: string;
  reason: string | null;
  snapshot?: HmiStateSnapshot;
}

export interface ReplayListQuery {
  sessionId?: string;
  operatorId?: string;
  finalState?: CommandLifecycleState;
  from?: string;
  to?: string;
  limit?: number;
}

export interface ReplayListResponse {
  items: ReplayListItem[];
}

export type HmiStreamEvent =
  | { type: 'snapshot'; snapshot: HmiStateSnapshot }
  | { type: 'heartbeat'; schemaVersion: string; generatedAt: string; transportState: TransportState; telemetryState: TelemetryState }
  | { type: 'lease_state'; lease: LeaseView }
  | { type: 'command_lifecycle'; command: CommandView; messages?: ChatMessage[]; planMetrics?: PlanMetrics | null }
  | { type: 'runtime_state'; runtime: RuntimeSnapshot; jointPositions?: JointPosition[] }
  | { type: 'replay_updated'; replayItems: ReplayListItem[] }
  | { type: 'connection_state'; transportState: TransportState; connections?: BridgeConnection[] };

export interface GP4BridgeClient {
  connect(params: {
    sessionId: string;
    operatorId: string;
    onEvent: (event: HmiStreamEvent) => void;
    onTransportStateChange?: (state: TransportState) => void;
  }): () => void;
  acquireLease(request: LeaseAcquireRequest): Promise<LeaseMutationResponse>;
  renewLease(request: LeaseRenewRequest): Promise<LeaseMutationResponse>;
  releaseLease(request: LeaseReleaseRequest): Promise<LeaseMutationResponse>;
  submitCommand(request: SubmitCommandRequest): Promise<SubmitCommandResponse>;
  confirmCommand(commandId: string, request: CommandActionRequest): Promise<CommandActionResponse>;
  abortCommand(commandId: string, request: CommandActionRequest): Promise<CommandActionResponse>;
  getRuntimeState(sessionId: string, operatorId: string): Promise<RuntimeStateResponse>;
  getConnectionState(): Promise<ConnectionStateResponse>;
  getLeaseState(sessionId: string, operatorId: string): Promise<LeaseStateResponse>;
  listReplay(query?: ReplayListQuery): Promise<ReplayListResponse>;
  getReplayDetail(commandId: string): Promise<ReplayDetail>;
}
