export type RuntimeMode = 'sim' | 'hardware' | 'unknown';
export type TransportState = 'connected' | 'connecting' | 'disconnected';
export type ConnectionHealth = 'healthy' | 'degraded' | 'down';
export type TelemetryState = 'fresh' | 'stale' | 'unavailable';
export type LeaseRole = 'controller' | 'observer';
export type CommandRiskLevel = 'low' | 'medium' | 'high' | 'critical';

export type CommandLifecycleState =
  | 'RECEIVED'
  | 'PARSING'
  | 'VALIDATING'
  | 'NEEDS_CONFIRMATION'
  | 'CONFIRMED'
  | 'EXECUTION_REQUESTED'
  | 'EXECUTING'
  | 'SUCCEEDED'
  | 'FAILED'
  | 'REJECTED'
  | 'CANCELLED'
  | 'EXPIRED';

export type TerminalCommandLifecycleState = 'SUCCEEDED' | 'FAILED' | 'REJECTED' | 'CANCELLED' | 'EXPIRED';

export type SystemRuntimeState =
  | 'NORMAL'
  | 'FAULT'
  | 'ESTOP'
  | 'HOLD'
  | 'TIMEOUT'
  | 'LOST_CONN'
  | 'SAFETY_BLOCKED';

export type MessageOrigin = 'system' | 'operator' | 'assistant';

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
  canCancelCommands: boolean;
  canAbortCommands: boolean;
  commandIngressAvailable: boolean;
  confirmationAvailable: boolean;
  executionAllowed: boolean;
  replayAvailable: boolean;
  simOnly: boolean;
  hardwareGate: HardwareGateStatus;
}

export interface HardwareGateChecklist {
  timingJitter: boolean;
  disconnectReconnect: boolean;
  robotStatusSemantics: boolean;
  jointSourcePrecedence: boolean;
  auditVisibility: boolean;
}

export interface HardwareGateStatus {
  unlocked: boolean;
  reasons: string[];
  flagEnabled: boolean;
  evidencePath: string;
  approvedBy: string | null;
  approvedAt: string | null;
  reportPath: string | null;
  reportSha256: string | null;
  reportSha256Match: boolean;
  checklist: HardwareGateChecklist | null;
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

export interface ValidationSourceStatus {
  name: string;
  label: string;
  topic: string;
  freshnessState: TelemetryState;
  active: boolean;
  preferred: boolean;
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
  tag?: string | null;
}

export interface CommandValidationResult {
  accepted: boolean;
  leaseValid: boolean;
  runtimeAllowed: boolean;
  telemetryFresh: boolean;
  requiresConfirmation: boolean;
  riskLevel: CommandRiskLevel | null;
  blockingReasons: string[];
  confirmationReasons: string[];
  planFingerprint: string | null;
  executionAllowedNow: boolean;
  criticalSources: ValidationSourceStatus[];
  optionalSources: ValidationSourceStatus[];
  eventDrivenSources: ValidationSourceStatus[];
  hardwareGate: HardwareGateStatus;
  preflight: Record<string, unknown>;
}

export interface CommandExecutionResult {
  accepted: boolean;
  adapter: string;
  status: string;
  summary: string;
  dispatchedToRos: boolean;
  queryOnly?: boolean;
  referenceFrame?: string | null;
  pose?: Record<string, unknown> | null;
  poseMm?: Record<string, unknown> | null;
  commandId?: string | null;
  planFingerprint?: string | null;
  operatorId?: string | null;
  sessionId?: string | null;
  leaseId?: string | null;
  correlationId?: string | null;
}

export interface CommandView {
  commandId: string;
  commandKind: 'command';
  sessionId: string;
  operatorId: string;
  rawText: string;
  intentSource: 'text' | 'structured';
  structuredIntent?: Record<string, unknown> | null;
  lifecycleState: CommandLifecycleState;
  summaryLabel: string;
  plannerUsed: string | null;
  frameUsed: string | null;
  mode: RuntimeMode;
  riskLevel: CommandRiskLevel | null;
  planFingerprint: string | null;
  correlationId: string | null;
  rejectReason: string | null;
  parsedIntent?: Record<string, unknown> | null;
  validationResult?: CommandValidationResult | null;
  planSummary?: Record<string, unknown> | null;
  metrics?: PlanMetrics | null;
  confirmationExpiresAt: string | null;
  createdAt: string;
  confirmAt: string | null;
  executeAt: string | null;
  executionResult?: CommandExecutionResult | null;
  finalState: TerminalCommandLifecycleState | null;
  parentSequenceId?: string | null;
  sequenceStepIndex?: number | null;
  sequenceStepCount?: number | null;
}

export interface SequenceView {
  sequenceId: string;
  commandKind: 'sequence';
  sessionId: string;
  operatorId: string;
  rawText: string;
  intentSource: 'text' | 'structured';
  structuredIntent?: Record<string, unknown> | null;
  lifecycleState: CommandLifecycleState;
  summaryLabel: string;
  plannerUsed: string | null;
  frameUsed: string | null;
  mode: RuntimeMode;
  riskLevel: CommandRiskLevel | null;
  planFingerprint: string | null;
  correlationId: string | null;
  rejectReason: string | null;
  validationResult?: CommandValidationResult | null;
  planSummary?: Record<string, unknown> | null;
  metrics?: PlanMetrics | null;
  confirmationExpiresAt: string | null;
  createdAt: string;
  confirmAt: string | null;
  executeAt: string | null;
  executionResult?: CommandExecutionResult | null;
  finalState: TerminalCommandLifecycleState | null;
  stepCount: number;
  currentStepIndex?: number | null;
  diagnostics: string[];
  manualRecoveryRequired: boolean;
  steps: CommandView[];
}

export interface ReplayListItem {
  commandId: string;
  kind: 'command' | 'sequence';
  sessionId: string;
  operatorId: string;
  summaryLabel: string;
  lifecycleState: string;
  finalState: string | null;
  plannerUsed: string | null;
  frameUsed: string | null;
  mode: RuntimeMode;
  createdAt: string;
  executeAt: string | null;
  riskLevel: CommandRiskLevel | null;
  stepCount?: number | null;
  currentStepIndex?: number | null;
  manualRecoveryRequired: boolean;
}

export interface TimelineEvent {
  id: string;
  commandId: string | null;
  timestamp: string;
  fromState: string | null;
  toState: string | null;
  runtimeState: string | null;
  message: string;
  payload?: Record<string, unknown> | null;
}

export interface ReplayDetail {
  jobType: 'command' | 'sequence';
  command: CommandView | null;
  sequence: SequenceView | null;
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
  activeSequence: SequenceView | null;
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

export interface CommandIntentRequest {
  sessionId: string;
  operatorId: string;
  leaseToken: string | null;
  intentText?: string | null;
  mode: 'sim' | 'hardware';
}

export interface CommandConfirmRequest {
  sessionId: string;
  operatorId: string;
  leaseToken: string | null;
  planFingerprint: string;
}

export interface CommandCancelRequest {
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

export interface CommandMutationResponse {
  accepted: boolean;
  jobType: 'command' | 'sequence';
  commandId: string | null;
  sequenceId?: string | null;
  reason: string | null;
  snapshot?: HmiStateSnapshot | null;
  command: CommandView | null;
  sequence?: SequenceView | null;
}

export interface ReplayListQuery {
  sessionId?: string;
  operatorId?: string;
  finalState?: string;
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
  | { type: 'lease_state'; lease: LeaseView; capabilities: BridgeCapabilities }
  | { type: 'command_lifecycle'; command: CommandView; messages?: ChatMessage[]; planMetrics?: PlanMetrics | null }
  | { type: 'sequence_lifecycle'; sequence: SequenceView; messages?: ChatMessage[] }
  | { type: 'replay_updated'; replayItems: ReplayListItem[] }
  | { type: 'connection_state'; transportState: TransportState; connections?: BridgeConnection[] }
  | { type: 'jog_bridge_status'; jogBridgeStatus: JogBridgeStatusSnapshot };

// ── Jog Pendant Types ──────────────────────────────────────────────────────

export type JogMode = 'continuous' | 'discrete';

export type JogBridgeState =
  | 'IDLE'
  | 'STARTING'
  | 'READY'
  | 'ACTIVE'
  | 'HALTING'
  | 'HALTED'
  | 'ERROR'
  | 'REJECTED_NOT_READY'
  | 'REJECTED_FJT_ACTIVE'
  | 'TIMEOUT'
  | 'BUSY_RETRY';

export interface JogBridgeStatusSnapshot {
  state: JogBridgeState;
  pointsQueued: number;
  effectiveHz: number;
  robotReady: boolean;
  servoActive: boolean;
  bridgeActive: boolean;
  lastError: string;
  rejectionReason: string;
}

export interface JogCommandRequest {
  jointIndex: number;       // 0-5
  direction: 1 | -1;        // +1 = positive, -1 = negative
  mode: JogMode;
  velocityScale: number;     // 0.0-0.3
  stepDegrees: number;       // for discrete mode
}

export interface JogBridgeCapabilities {
  jogAvailable: boolean;
  bridgeServiceAvailable: boolean;
  canActivateBridge: boolean;
  bridgeState: JogBridgeState;
  isExclusiveMode: boolean;  // true when jog bridge is active (blocks FJT path)
}

export interface ServoControlResponse {
  accepted: boolean;
  message: string;
}

export interface ServoControlRequest {
  sessionId: string;
  operatorId: string;
  leaseToken: string | null;
}

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
  submitCommand(request: CommandIntentRequest): Promise<CommandMutationResponse>;
  confirmCommand(commandId: string, request: CommandConfirmRequest): Promise<CommandMutationResponse>;
  confirmSequence(sequenceId: string, request: CommandConfirmRequest): Promise<CommandMutationResponse>;
  abortCommand(commandId: string, request: CommandCancelRequest): Promise<CommandMutationResponse>;
  abortSequence(sequenceId: string, request: CommandCancelRequest): Promise<CommandMutationResponse>;
  getRuntimeState(sessionId: string, operatorId: string): Promise<RuntimeStateResponse>;
  getConnectionState(): Promise<ConnectionStateResponse>;
  getLeaseState(sessionId: string, operatorId: string): Promise<LeaseStateResponse>;
  listReplay(query?: ReplayListQuery): Promise<ReplayListResponse>;
  getReplayDetail(commandId: string): Promise<ReplayDetail>;
  getSequence(sequenceId: string): Promise<SequenceView>;
  // Jog pendant
  activateJogBridge(): Promise<{ accepted: boolean; message: string }>;
  deactivateJogBridge(): Promise<{ accepted: boolean; message: string }>;
  sendJogCommand(cmd: JogCommandRequest): Promise<{ accepted: boolean; message: string }>;
  // Servo control
  startServo(request: ServoControlRequest): Promise<ServoControlResponse>;
  stopServo(request: ServoControlRequest): Promise<ServoControlResponse>;
}
