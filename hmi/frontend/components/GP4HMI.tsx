import { useEffect, useMemo, useState, type KeyboardEvent } from 'react';

import type {
  BridgeConnection,
  ChatMessage,
  CommandLifecycleState,
  CommandMutationResponse,
  CommandView,
  GP4BridgeClient,
  JointPosition,
  MessageOrigin,
  TelemetrySourceStatus,
} from '../../shared/contracts';
import { useGP4Bridge } from '../hooks/useGP4Bridge';
import { RuntimeStateBanner } from './RuntimeStateBanner';

const JOINT_ORDER = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];

interface IntentTemplate {
  id: string;
  intent: string;
  title: string;
  subtitle: string;
}

const INTENT_TEMPLATES: IntentTemplate[] = [
  {
    id: 'home',
    intent: 'home',
    title: 'Home / Về home',
    subtitle: 'Return to home pose · Đưa robot về vị trí home',
  },
  {
    id: 'stop',
    intent: 'stop',
    title: 'Stop / Dừng',
    subtitle: 'Immediate supervised stop request · Yêu cầu dừng có giám sát',
  },
  {
    id: 'up_5cm',
    intent: 'move up 5 cm',
    title: 'Move up 5 cm / Nâng lên 5 cm',
    subtitle: 'Small vertical lift in base_link · Tịnh tiến đứng nhỏ trong base_link',
  },
  {
    id: 'down_2cm',
    intent: 'move down 2 cm',
    title: 'Move down 2 cm / Hạ xuống 2 cm',
    subtitle: 'Small downward move in base_link · Tịnh tiến xuống nhỏ trong base_link',
  },
  {
    id: 'joint_1_plus_5',
    intent: 'move joint 1 +5 deg',
    title: 'Joint 1 +5° / Khớp 1 +5°',
    subtitle: 'Conservative joint adjustment · Điều chỉnh khớp bảo thủ',
  },
  {
    id: 'wait_2s',
    intent: 'wait 2 s',
    title: 'Wait 2 s / Chờ 2 giây',
    subtitle: 'Pause sequence safely · Tạm dừng chuỗi an toàn',
  },
  {
    id: 'get_pose',
    intent: 'get pose',
    title: 'Get pose / Lấy pose',
    subtitle: 'Query current TCP pose · Truy vấn pose TCP hiện tại',
  },
];

type PillTone = 'green' | 'blue' | 'amber' | 'red' | 'cyan' | 'gray';
type MessageRole = 'user' | 'robot' | 'system';
type LogLevel = 'info' | 'ok' | 'warn' | 'err';
type PipelineStatus = 'done' | 'active' | 'pending' | 'error';

interface PipelineStepView {
  key: string;
  label: string;
  status: PipelineStatus;
  marker: string;
}

interface TopicView {
  key: string;
  name: string;
  rateLabel: string;
  fillWidth: number;
}

interface LogEntryView {
  id: string;
  time: string;
  level: LogLevel;
  message: string;
}

interface ActionFeedbackView {
  id: string;
  timestamp: string;
  level: LogLevel;
  message: string;
}

interface TraceStepView {
  key: string;
  label: string;
  status: PipelineStatus;
  summary: string;
  details: Record<string, unknown> | null;
}

interface StatusPillView {
  key: string;
  label: string;
  tone: PillTone;
}

const VALIDATED_OR_BEYOND: ReadonlySet<CommandLifecycleState> = new Set([
  'NEEDS_CONFIRMATION',
  'CONFIRMED',
  'EXECUTION_REQUESTED',
  'EXECUTING',
  'SUCCEEDED',
  'FAILED',
  'REJECTED',
  'CANCELLED',
  'EXPIRED',
]);

const CONFIRMED_OR_BEYOND: ReadonlySet<CommandLifecycleState> = new Set([
  'CONFIRMED',
  'EXECUTION_REQUESTED',
  'EXECUTING',
  'SUCCEEDED',
  'FAILED',
  'REJECTED',
  'CANCELLED',
  'EXPIRED',
]);

const EXECUTION_REQUESTED_OR_BEYOND: ReadonlySet<CommandLifecycleState> = new Set([
  'EXECUTION_REQUESTED',
  'EXECUTING',
  'SUCCEEDED',
  'FAILED',
  'REJECTED',
  'CANCELLED',
  'EXPIRED',
]);

const EXECUTING_OR_BEYOND: ReadonlySet<CommandLifecycleState> = new Set([
  'EXECUTING',
  'SUCCEEDED',
  'FAILED',
  'REJECTED',
  'CANCELLED',
  'EXPIRED',
]);

const TERMINAL_FAILURE_STATES: ReadonlySet<CommandLifecycleState> = new Set([
  'FAILED',
  'REJECTED',
  'CANCELLED',
  'EXPIRED',
]);

function isInStateSet(
  state: CommandLifecycleState | null | undefined,
  allowed: ReadonlySet<CommandLifecycleState>,
): boolean {
  return state !== null && state !== undefined && allowed.has(state);
}

function toPercent(joint: JointPosition): number {
  if (joint.positionDeg === null) {
    return 0;
  }
  const span = joint.maxDeg - joint.minDeg;
  if (span <= 0) {
    return 0;
  }
  return Math.max(0, Math.min(100, ((joint.positionDeg - joint.minDeg) / span) * 100));
}

function formatAngle(value: number | null): string {
  if (value === null) {
    return '--';
  }
  return `${value >= 0 ? '+' : ''}${value.toFixed(1)}°`;
}

function formatMetric(value: number | null, digits = 2, suffix = ''): string {
  if (value === null) {
    return '--';
  }
  return `${value.toFixed(digits)}${suffix}`;
}

function formatTimestamp(value: string): string {
  const parsed = new Date(value);
  if (Number.isNaN(parsed.getTime())) {
    return value;
  }
  return parsed.toLocaleTimeString('en-GB', { hour12: false });
}

function formatClock(value: Date): string {
  const year = value.getFullYear();
  const month = String(value.getMonth() + 1).padStart(2, '0');
  const day = String(value.getDate()).padStart(2, '0');
  const hour = String(value.getHours()).padStart(2, '0');
  const minute = String(value.getMinutes()).padStart(2, '0');
  const second = String(value.getSeconds()).padStart(2, '0');
  return `${year}-${month}-${day} ${hour}:${minute}:${second}`;
}

function humanizeLabel(value: string): string {
  return value
    .toLowerCase()
    .split('_')
    .map((token) => token.charAt(0).toUpperCase() + token.slice(1))
    .join(' ');
}

function toneFromConnectionHealth(health: BridgeConnection['health'] | undefined): PillTone {
  if (health === 'healthy') {
    return 'green';
  }
  if (health === 'degraded') {
    return 'amber';
  }
  return 'red';
}

function toneFromServoState(servo: 'ON' | 'OFF' | 'UNKNOWN'): PillTone {
  if (servo === 'ON') {
    return 'green';
  }
  if (servo === 'OFF') {
    return 'red';
  }
  return 'amber';
}

function toneFromMode(mode: 'sim' | 'hardware' | 'unknown'): PillTone {
  if (mode === 'sim') {
    return 'blue';
  }
  if (mode === 'hardware') {
    return 'amber';
  }
  return 'gray';
}

function toneFromLifecycle(state: CommandLifecycleState | null | undefined): PillTone {
  if (state === null || state === undefined) {
    return 'gray';
  }
  if (state === 'SUCCEEDED') {
    return 'green';
  }
  if (TERMINAL_FAILURE_STATES.has(state)) {
    return 'red';
  }
  if (state === 'EXECUTING' || state === 'EXECUTION_REQUESTED') {
    return 'cyan';
  }
  if (state === 'NEEDS_CONFIRMATION') {
    return 'amber';
  }
  return 'blue';
}

function tagClassName(tag?: string | null): string {
  if (!tag) {
    return 'tag';
  }

  const normalized = tag.toLowerCase();
  if (normalized === 'needs_confirmation') {
    return 'tag confirm';
  }
  if (normalized === 'confirmed' || normalized === 'execution_requested') {
    return 'tag planned';
  }
  if (normalized === 'executing') {
    return 'tag executing';
  }
  if (normalized === 'succeeded') {
    return 'tag success';
  }
  if (
    normalized === 'failed' ||
    normalized === 'rejected' ||
    normalized === 'cancelled' ||
    normalized === 'expired'
  ) {
    return 'tag error';
  }
  if (normalized === 'validating' || normalized === 'validated') {
    return 'tag validated';
  }
  if (normalized === 'parsing' || normalized === 'received') {
    return 'tag parsed';
  }
  return 'tag';
}

function toMessageRole(origin: MessageOrigin): MessageRole {
  if (origin === 'operator') {
    return 'user';
  }
  if (origin === 'assistant') {
    return 'robot';
  }
  return 'system';
}

function avatarForRole(role: MessageRole): string {
  if (role === 'user') {
    return 'OP';
  }
  if (role === 'robot') {
    return 'GP4';
  }
  return 'SYS';
}

function nameForRole(role: MessageRole): string {
  if (role === 'user') {
    return 'Operator';
  }
  if (role === 'robot') {
    return 'GP4 Agent';
  }
  return 'System';
}

function toTopicFillWidth(freshness: TelemetrySourceStatus['freshnessState']): number {
  if (freshness === 'fresh') {
    return 100;
  }
  if (freshness === 'stale') {
    return 40;
  }
  return 10;
}

function toTopicRateLabel(freshness: TelemetrySourceStatus['freshnessState']): string {
  if (freshness === 'fresh') {
    return 'FRESH';
  }
  if (freshness === 'stale') {
    return 'STALE';
  }
  return 'DOWN';
}

function toLogLevel(message: ChatMessage): LogLevel {
  const tag = (message.tag ?? '').toLowerCase();
  if (tag === 'succeeded' || tag === 'confirmed' || tag === 'execution_requested') {
    return 'ok';
  }
  if (tag === 'needs_confirmation' || tag === 'validating') {
    return 'warn';
  }
  if (tag === 'failed' || tag === 'rejected' || tag === 'cancelled' || tag === 'expired') {
    return 'err';
  }
  if (message.origin === 'assistant') {
    return 'ok';
  }
  return 'info';
}

function buildPipeline(state: CommandLifecycleState | null | undefined): PipelineStepView[] {
  const steps: Array<Omit<PipelineStepView, 'marker'>> = [
    {
      key: 'parsed',
      label: 'Parsed',
      status: state === 'PARSING' ? 'active' : state !== undefined && state !== null && state !== 'RECEIVED' ? 'done' : 'pending',
    },
    {
      key: 'validated',
      label: 'Validated',
      status: state === 'VALIDATING' ? 'active' : isInStateSet(state, VALIDATED_OR_BEYOND) ? 'done' : 'pending',
    },
    {
      key: 'confirmation',
      label: 'Confirmation',
      status: state === 'NEEDS_CONFIRMATION' ? 'active' : isInStateSet(state, CONFIRMED_OR_BEYOND) ? 'done' : 'pending',
    },
    {
      key: 'dispatch',
      label: 'Dispatch Requested',
      status:
        state === 'EXECUTION_REQUESTED'
          ? 'active'
          : isInStateSet(state, EXECUTION_REQUESTED_OR_BEYOND)
            ? 'done'
            : 'pending',
    },
    {
      key: 'executing',
      label: 'Executing',
      status: state === 'EXECUTING' ? 'active' : isInStateSet(state, EXECUTING_OR_BEYOND) ? 'done' : 'pending',
    },
    {
      key: 'completed',
      label: 'Completed',
      status: state === 'SUCCEEDED' ? 'done' : isInStateSet(state, TERMINAL_FAILURE_STATES) ? 'error' : 'pending',
    },
  ];

  return steps.map((step, index) => ({
    ...step,
    marker: step.status === 'done' ? '✓' : step.status === 'error' ? '!' : String(index + 1),
  }));
}

function durationSeconds(command: CommandView | null): number | null {
  if (!command) {
    return null;
  }
  const startIso = command.confirmAt ?? command.createdAt;
  const endIso = command.executeAt;
  if (!startIso || !endIso) {
    return null;
  }
  const startTime = new Date(startIso).getTime();
  const endTime = new Date(endIso).getTime();
  if (Number.isNaN(startTime) || Number.isNaN(endTime) || endTime < startTime) {
    return null;
  }
  return (endTime - startTime) / 1000;
}

function prettyJson(value: unknown): string {
  const serialized = JSON.stringify(value, null, 2);
  return serialized ?? 'null';
}

function resolveDeclineReason(
  mutationReason: string | null | undefined,
  command: CommandView | null | undefined,
): string | null {
  const trimmedMutationReason = mutationReason?.trim();
  if (trimmedMutationReason) {
    return trimmedMutationReason;
  }

  const rejectReason = command?.rejectReason?.trim();
  if (rejectReason) {
    return rejectReason;
  }

  const blockingReasons = command?.validationResult?.blockingReasons ?? [];
  if (blockingReasons.length > 0) {
    return blockingReasons.join('; ');
  }

  return null;
}

function reasonToVietnamese(reason: string): string {
  const normalized = reason.toLowerCase();
  if (normalized.includes('hmi_enable_hardware_commands')) {
    return 'Biến môi trường bật lệnh phần cứng chưa được đặt true.';
  }
  if (normalized.includes('evidence file is missing')) {
    return 'Thiếu file minh chứng cổng phần cứng.';
  }
  if (normalized.includes('approved=true')) {
    return 'Biên bản chưa được phê duyệt chính thức.';
  }
  if (normalized.includes('approvedby')) {
    return 'Thiếu thông tin người phê duyệt.';
  }
  if (normalized.includes('approvedat')) {
    return 'Thiếu thời điểm phê duyệt theo ISO8601.';
  }
  if (normalized.includes('report sha256')) {
    return 'Checksum báo cáo không khớp với tệp minh chứng.';
  }
  if (normalized.includes('timing/jitter')) {
    return 'Checklist timing/jitter chưa đạt.';
  }
  if (normalized.includes('disconnect-reconnect')) {
    return 'Checklist disconnect-reconnect chưa đạt.';
  }
  if (normalized.includes('robot_status semantics')) {
    return 'Checklist semantics robot_status chưa đạt.';
  }
  if (normalized.includes('joint source precedence')) {
    return 'Checklist ưu tiên nguồn joint chưa đạt.';
  }
  if (normalized.includes('audit visibility')) {
    return 'Checklist hiển thị audit chưa đạt.';
  }
  if (normalized.includes('runtime state')) {
    return 'Trạng thái runtime hiện tại đang chặn lệnh.';
  }
  if (normalized.includes('telemetry') && normalized.includes('stale')) {
    return 'Telemetry nguồn bắt buộc đang stale hoặc unavailable.';
  }
  if (normalized.includes('preflight')) {
    return 'Preflight phần cứng thất bại.';
  }
  return 'Điều kiện an toàn chưa đạt, hệ thống giữ fail-closed.';
}

function hardwareGateLabel(unlocked: boolean): string {
  return unlocked ? 'UNLOCKED · MỞ KHÓA' : 'LOCKED · KHÓA';
}

function summarizeMutationResponse(response: CommandMutationResponse): string {
  const stateLabel = humanizeLabel(response.command.lifecycleState);
  if (response.accepted) {
    return `Accepted · ${stateLabel}`;
  }
  const reason = resolveDeclineReason(response.reason, response.command);
  return reason ? `Declined · ${reason}` : `Declined · ${stateLabel}`;
}

function buildTraceSteps(command: CommandView | null): TraceStepView[] {
  if (!command) {
    return [];
  }

  const lifecycle = command.lifecycleState;
  const validation = command.validationResult ?? null;
  const execution = command.executionResult ?? null;
  const blockingReasons = validation?.blockingReasons ?? [];
  const confirmationReasons = validation?.confirmationReasons ?? [];
  const parsedAction = command.parsedIntent ? command.parsedIntent['action'] : undefined;

  const parseStatus: PipelineStatus =
    lifecycle === 'PARSING' ? 'active' : lifecycle !== 'RECEIVED' ? 'done' : 'pending';
  const validateStatus: PipelineStatus =
    lifecycle === 'VALIDATING'
      ? 'active'
      : isInStateSet(lifecycle, VALIDATED_OR_BEYOND)
        ? 'done'
        : 'pending';
  const confirmationStatus: PipelineStatus =
    lifecycle === 'NEEDS_CONFIRMATION'
      ? 'active'
      : isInStateSet(lifecycle, CONFIRMED_OR_BEYOND)
        ? 'done'
        : 'pending';
  const dispatchStatus: PipelineStatus =
    lifecycle === 'EXECUTION_REQUESTED'
      ? 'active'
      : isInStateSet(lifecycle, EXECUTION_REQUESTED_OR_BEYOND)
        ? 'done'
        : 'pending';
  const executingStatus: PipelineStatus =
    lifecycle === 'EXECUTING'
      ? 'active'
      : isInStateSet(lifecycle, EXECUTING_OR_BEYOND)
        ? 'done'
        : 'pending';
  const terminalStatus: PipelineStatus =
    lifecycle === 'SUCCEEDED'
      ? 'done'
      : isInStateSet(lifecycle, TERMINAL_FAILURE_STATES)
        ? 'error'
        : 'pending';

  const parseSummary =
    parsedAction
      ? `Parsed action ${String(parsedAction)} from ${command.intentSource} input.`
      : lifecycle === 'REJECTED'
        ? `Parser/enrichment rejected this command.`
        : 'Waiting for parser output.';

  const validationSummary =
    validation === null
      ? 'Validation has not run yet.'
      : validation.accepted
        ? `Validation accepted · risk=${validation.riskLevel ?? 'unknown'}.`
        : `Validation rejected: ${blockingReasons.join('; ') || 'no reason provided'}.`;

  const confirmationSummary = command.planFingerprint
    ? `Plan fingerprint ready (${command.planFingerprint.slice(0, 12)}...).`
    : lifecycle === 'NEEDS_CONFIRMATION'
      ? 'Waiting for operator confirmation.'
      : 'Confirmation gate not reached.';

  const dispatchSummary = isInStateSet(lifecycle, EXECUTION_REQUESTED_OR_BEYOND)
    ? 'Confirmed command forwarded to execution boundary.'
    : 'Execution request not sent yet.';

  const executingSummary = execution
    ? `Execution status=${execution.status} · ${execution.summary}`
    : lifecycle === 'EXECUTING'
      ? 'Execution requested; waiting for ROS result.'
      : 'Execution boundary not reached.';

  const terminalReason = resolveDeclineReason(null, command);
  const resultSummary =
    command.finalState !== null
      ? terminalReason
        ? `${humanizeLabel(command.finalState)} · ${terminalReason}`
        : humanizeLabel(command.finalState)
      : 'No terminal result yet.';

  return [
    {
      key: 'parse',
      label: 'Step 1 · Parse intent',
      status: parseStatus,
      summary: parseSummary,
      details: {
        rawText: command.rawText,
        intentSource: command.intentSource,
        parsedIntent: command.parsedIntent ?? null,
      },
    },
    {
      key: 'validate',
      label: 'Step 2 · Validate command',
      status: validateStatus,
      summary: validationSummary,
      details: validation ? { validationResult: validation } : null,
    },
    {
      key: 'confirm',
      label: 'Step 3 · Confirmation gate',
      status: confirmationStatus,
      summary: confirmationSummary,
      details: {
        planFingerprint: command.planFingerprint,
        confirmationExpiresAt: command.confirmationExpiresAt,
        confirmationReasons,
      },
    },
    {
      key: 'dispatch',
      label: 'Step 4 · Dispatch request',
      status: dispatchStatus,
      summary: dispatchSummary,
      details: {
        dispatchedToRos: execution?.dispatchedToRos ?? false,
        executionStatus: execution?.status ?? null,
      },
    },
    {
      key: 'executing',
      label: 'Step 5 · Executing',
      status: executingStatus,
      summary: executingSummary,
      details: execution ? { executionResult: execution } : null,
    },
    {
      key: 'result',
      label: 'Step 6 · Terminal result',
      status: terminalStatus,
      summary: resultSummary,
      details: {
        finalState: command.finalState,
        rejectReason: command.rejectReason,
        executionResult: command.executionResult ?? null,
      },
    },
  ];
}

interface GP4HMIProps {
  client: GP4BridgeClient;
  sessionId: string;
  operatorId: string;
}

export function GP4HMI({ client, sessionId, operatorId }: GP4HMIProps) {
  const [draft, setDraft] = useState('');
  const [clockText, setClockText] = useState(() => formatClock(new Date()));
  const [submitError, setSubmitError] = useState<string | null>(null);
  const [actionFeedback, setActionFeedback] = useState<ActionFeedbackView | null>(null);
  const [latestMutationReason, setLatestMutationReason] = useState<string | null>(null);

  const {
    state,
    isController,
    blockingRuntime,
    submitCommand,
    confirmCommandById,
    acquireControllerLease,
    releaseLease,
    confirmActiveCommand,
    abortActiveCommand,
  } = useGP4Bridge(client, sessionId, operatorId);

  const readOnlyBridge = state.capabilities.readOnly;
  const canSubmitCommands =
    state.capabilities.canSubmitCommands &&
    state.capabilities.commandIngressAvailable &&
    !blockingRuntime &&
    isController;
  const canAcquireLease = state.capabilities.canAcquireLease;
  const canReleaseLease = !readOnlyBridge && isController && state.lease.leaseToken !== null;
  const canConfirmCommands = state.capabilities.canConfirmCommands && !blockingRuntime && isController;
  const canAbortCommands = (state.capabilities.canCancelCommands || state.capabilities.canAbortCommands) && isController;
  const hardwareGate = state.capabilities.hardwareGate;
  const hardwareGateReasons = hardwareGate.reasons ?? [];
  const primaryHardwareGateReason =
    hardwareGateReasons[0] ??
    'Dual gate is not satisfied: runtime flag + signed evidence checklist are required.';
  const primaryHardwareGateReasonVi = reasonToVietnamese(primaryHardwareGateReason);

  const activeCommand = state.activeCommand;
  const blockingReasons = activeCommand?.validationResult?.blockingReasons ?? [];
  const confirmationReasons = activeCommand?.validationResult?.confirmationReasons ?? [];
  const executionSummary = activeCommand?.executionResult?.summary ?? null;
  const traceSteps = useMemo(() => buildTraceSteps(activeCommand), [activeCommand]);
  const declineReason = useMemo(
    () => resolveDeclineReason(latestMutationReason, activeCommand),
    [latestMutationReason, activeCommand],
  );

  const pushActionFeedback = (level: LogLevel, message: string, reason?: string | null) => {
    setActionFeedback({
      id: `action-${Date.now()}`,
      timestamp: new Date().toISOString(),
      level,
      message,
    });
    setLatestMutationReason(reason?.trim() || null);
  };

  useEffect(() => {
    const timer = window.setInterval(() => {
      setClockText(formatClock(new Date()));
    }, 1000);

    return () => {
      window.clearInterval(timer);
    };
  }, []);

  const orderedJoints = useMemo(() => {
    const jointMap = new Map(state.jointPositions.map((joint) => [joint.name, joint]));
    return JOINT_ORDER.map((name) =>
      jointMap.get(name) ?? { name, positionDeg: null, minDeg: -180, maxDeg: 180 },
    );
  }, [state.jointPositions]);

  const pipeline = useMemo(() => buildPipeline(activeCommand?.lifecycleState), [activeCommand?.lifecycleState]);

  const commandCount = useMemo(() => {
    if (state.replayItems.length > 0) {
      return state.replayItems.length;
    }
    return state.messages.filter((message) => message.origin === 'operator').length;
  }, [state.messages, state.replayItems.length]);

  const cycleSeconds = useMemo(() => durationSeconds(activeCommand), [activeCommand]);

  const connectionMap = useMemo(() => {
    return new Map(state.connections.map((connection) => [connection.name, connection]));
  }, [state.connections]);

  const statusPills = useMemo<StatusPillView[]>(() => {
    const ros2 = connectionMap.get('ros2');
    const moveit2 = connectionMap.get('moveit2');

    return [
      {
        key: 'ros2',
        label: `ROS2 ${ros2?.health === 'healthy' ? 'Connected' : ros2?.health === 'degraded' ? 'Degraded' : 'Down'}`,
        tone: toneFromConnectionHealth(ros2?.health),
      },
      {
        key: 'servo',
        label: `Servo ${state.runtime.robotStatus.servoState}`,
        tone: toneFromServoState(state.runtime.robotStatus.servoState),
      },
      {
        key: 'mode',
        label: `${state.mode === 'sim' ? 'Simulation' : state.mode === 'hardware' ? 'Hardware' : 'Mode Unknown'}`,
        tone: toneFromMode(state.mode),
      },
      {
        key: 'moveit2',
        label: `MoveIt2 ${moveit2?.health === 'healthy' ? 'Ready' : moveit2?.health === 'degraded' ? 'Degraded' : 'Down'}`,
        tone: toneFromConnectionHealth(moveit2?.health),
      },
      {
        key: 'command',
        label: `Command ${activeCommand?.lifecycleState ? humanizeLabel(activeCommand.lifecycleState) : 'Idle'}`,
        tone: toneFromLifecycle(activeCommand?.lifecycleState),
      },
    ];
  }, [activeCommand?.lifecycleState, connectionMap, state.mode, state.runtime.robotStatus.servoState]);

  const topicRows = useMemo<TopicView[]>(() => {
    if (state.telemetrySources.length > 0) {
      return [...state.telemetrySources]
        .sort((left, right) => Number(right.active) - Number(left.active))
        .slice(0, 6)
        .map((source) => ({
          key: source.name,
          name: source.topic,
          rateLabel: toTopicRateLabel(source.freshnessState),
          fillWidth: toTopicFillWidth(source.freshnessState),
        }));
    }

    return state.connections.slice(0, 6).map((connection) => ({
      key: connection.name,
      name: connection.label,
      rateLabel:
        connection.health === 'healthy'
          ? 'FRESH'
          : connection.health === 'degraded'
            ? 'STALE'
            : 'DOWN',
      fillWidth:
        connection.health === 'healthy' ? 100 : connection.health === 'degraded' ? 40 : 10,
    }));
  }, [state.connections, state.telemetrySources]);

  const logEntries = useMemo<LogEntryView[]>(() => {
    const entries = state.messages.slice(-14).map((message) => ({
      id: message.id,
      time: formatTimestamp(message.timestamp),
      level: toLogLevel(message),
      message: message.text,
    }));
    if (actionFeedback) {
      entries.unshift({
        id: actionFeedback.id,
        time: formatTimestamp(actionFeedback.timestamp),
        level: actionFeedback.level,
        message: actionFeedback.message,
      });
    }

    if (entries.length > 0) {
      return entries.slice(0, 14);
    }

    return [
      {
        id: 'runtime-bootstrap',
        time: formatTimestamp(state.generatedAt),
        level: state.runtime.blocking ? 'warn' : 'info',
        message: state.runtime.statusText,
      },
    ];
  }, [actionFeedback, state.generatedAt, state.messages, state.runtime.blocking, state.runtime.statusText]);

  const handleSubmit = async () => {
    const rawText = draft.trim();
    if (!rawText || !canSubmitCommands) {
      return;
    }

    try {
      const response = await submitCommand(rawText);
      if (!response.accepted) {
        const reason = resolveDeclineReason(response.reason, response.command);
        setSubmitError(reason ?? 'Command rejected by supervisor validation.');
        pushActionFeedback('err', `Submit declined · ${reason ?? 'no reason provided'}`, reason);
        return;
      }
      setDraft('');
      setSubmitError(null);
      pushActionFeedback('ok', `Submit response · ${summarizeMutationResponse(response)}`, response.reason);

      const shouldAutoConfirmSimCommand =
        response.command.lifecycleState === 'NEEDS_CONFIRMATION' &&
        response.command.mode === 'sim' &&
        response.command.planFingerprint !== null;
      if (shouldAutoConfirmSimCommand) {
        try {
          const confirmResponse = await confirmCommandById(
            response.commandId,
            response.command.planFingerprint as string,
          );
          const confirmReason = resolveDeclineReason(confirmResponse.reason, confirmResponse.command);
          if (!confirmResponse.accepted) {
            pushActionFeedback(
              'err',
              `Sim auto-confirm declined · ${confirmReason ?? 'no reason provided'}`,
              confirmReason,
            );
            return;
          }
          pushActionFeedback(
            'ok',
            `Sim auto-confirm · ${summarizeMutationResponse(confirmResponse)}`,
            confirmResponse.reason,
          );
        } catch (error: unknown) {
          const message = error instanceof Error ? error.message : 'Sim auto-confirm failed.';
          pushActionFeedback('err', `Sim auto-confirm failed · ${message}`, message);
        }
      }
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Failed to submit intent.';
      setSubmitError(message);
      pushActionFeedback('err', `Submit failed · ${message}`, message);
    }
  };

  const handleConfirmClick = async () => {
    try {
      const response = await confirmActiveCommand();
      if (!response) {
        pushActionFeedback('warn', 'Confirm skipped · no active command available.');
        return;
      }
      const reason = resolveDeclineReason(response.reason, response.command);
      if (!response.accepted) {
        pushActionFeedback('err', `Confirm declined · ${reason ?? 'no reason provided'}`, reason);
        return;
      }
      pushActionFeedback('ok', `Confirm response · ${summarizeMutationResponse(response)}`, response.reason);
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Failed to confirm command.';
      pushActionFeedback('err', `Confirm failed · ${message}`, message);
    }
  };

  const handleAbortClick = async (reason: string) => {
    try {
      const response = await abortActiveCommand(reason);
      if (!response) {
        pushActionFeedback('warn', 'Abort skipped · no active command available.');
        return;
      }
      const resolvedReason = resolveDeclineReason(response.reason, response.command);
      if (!response.accepted) {
        pushActionFeedback('err', `Abort declined · ${resolvedReason ?? 'no reason provided'}`, resolvedReason);
        return;
      }
      pushActionFeedback('warn', `Abort response · ${summarizeMutationResponse(response)}`, resolvedReason);
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Failed to abort command.';
      pushActionFeedback('err', `Abort failed · ${message}`, message);
    }
  };

  const handleInputKeyDown = (event: KeyboardEvent<HTMLTextAreaElement>) => {
    if (event.key === 'Enter' && !event.shiftKey) {
      event.preventDefault();
      void handleSubmit();
    }
  };

  const canAbortTopbar = canAbortCommands && activeCommand !== null;

  return (
    <div className="hmi-shell">
      <div className="hmi-app">
        <header className="topbar">
          <div className="logo">
            <div className="logo-icon">GP4</div>
            GP4 <span>HMI</span>
          </div>
          <div className="top-divider" />
          <div className="status-pills">
            {statusPills.map((pill) => (
              <span key={pill.key} className={`pill ${pill.tone}`}>
                <span className={`dot ${pill.tone}`}></span>
                {pill.label}
              </span>
            ))}
          </div>
          <div className="top-right">
            <span className="top-time">{clockText}</span>
            <div className="top-divider"></div>
            <button
              className="estop-btn"
              disabled={!canAbortTopbar}
              onClick={() => {
                void handleAbortClick('Operator requested topbar abort from HMI.');
              }}
            >
              Abort command
            </button>
          </div>
        </header>

        <aside className="left-panel">
          <div className="panel-header">Robot Monitor</div>

          <section className="section">
            <div className="section-title">Joint Positions (deg)</div>
            {orderedJoints.map((joint) => (
              <div key={joint.name} className="joint-row">
                <div className="joint-header">
                  <span className="joint-name">{joint.name}</span>
                  <span className="joint-val">{formatAngle(joint.positionDeg)}</span>
                </div>
                <div className="joint-bar">
                  <div className="joint-fill" style={{ width: `${toPercent(joint)}%` }}></div>
                </div>
              </div>
            ))}
          </section>

          <section className="section">
            <div className="section-title">Runtime Snapshot</div>
            <div className="pose-grid">
              <div className="pose-cell">
                <div className="pose-label">Mode</div>
                <div className="pose-value">{state.mode}</div>
              </div>
              <div className="pose-cell">
                <div className="pose-label">Frame</div>
                <div className="pose-value">{activeCommand?.frameUsed ?? '--'}</div>
              </div>
              <div className="pose-cell">
                <div className="pose-label">Planner</div>
                <div className="pose-value">{activeCommand?.plannerUsed ?? '--'}</div>
              </div>
              <div className="pose-cell">
                <div className="pose-label">Risk</div>
                <div className="pose-value">{activeCommand?.riskLevel ?? '--'}</div>
              </div>
              <div className="pose-cell">
                <div className="pose-label">Servo</div>
                <div className="pose-value">{state.runtime.robotStatus.servoState}</div>
              </div>
              <div className="pose-cell">
                <div className="pose-label">Alarm</div>
                <div className="pose-value">{state.runtime.robotStatus.alarmState}</div>
              </div>
            </div>
          </section>

          <section className="section">
            <div className="section-title">Command Pipeline</div>
            <div className="pipeline">
              {pipeline.map((step) => (
                <div key={step.key} className={`pipe-step ${step.status}`}>
                  <div className="pipe-num">{step.marker}</div>
                  <span className="pipe-label">{step.label}</span>
                </div>
              ))}
            </div>
          </section>

          <section className="section section-grow">
            <div className="section-title">Quick Commands</div>
            <div className="quick-cmds">
              {INTENT_TEMPLATES.map((template) => (
                <button
                  key={template.id}
                  type="button"
                  className="qcmd"
                  onClick={() => {
                    setDraft(template.intent);
                    setSubmitError(null);
                  }}
                >
                  <span className="cmd-icon">&gt;</span>
                  <span className="qcmd-text">
                    <span className="qcmd-title">{template.title}</span>
                    <span className="qcmd-subtitle">{template.subtitle}</span>
                  </span>
                </button>
              ))}
            </div>
          </section>
        </aside>

        <main className="center-panel">
          <div className="chat-header">
            <span className="chat-title">LLM Command Interface - Yaskawa GP4</span>
            <span className="chat-sub">
              transport {state.transportState} · schema {state.schemaVersion} · {readOnlyBridge ? 'read only' : 'command ingress enabled'} · gate {hardwareGateLabel(hardwareGate.unlocked)}
            </span>
          </div>

          <div className="chat-messages">
            <section className={`hardware-gate-panel ${hardwareGate.unlocked ? 'unlocked' : 'locked'}`}>
              <div className="hardware-gate-header">
                <div className="hardware-gate-title">Hardware Gate / Cổng phần cứng</div>
                <span className={`hardware-gate-pill ${hardwareGate.unlocked ? 'unlocked' : 'locked'}`}>
                  {hardwareGateLabel(hardwareGate.unlocked)}
                </span>
              </div>
              <div className="hardware-gate-line">
                Flag / Cờ bật phần cứng: <strong>{hardwareGate.flagEnabled ? 'ON' : 'OFF'}</strong>
              </div>
              <div className="hardware-gate-line">
                Evidence / Minh chứng: <code>{hardwareGate.evidencePath}</code>
              </div>
              {!hardwareGate.unlocked ? (
                <>
                  <div className="hardware-gate-reason">
                    EN: {primaryHardwareGateReason}
                  </div>
                  <div className="hardware-gate-reason vi">
                    VI: {primaryHardwareGateReasonVi}
                  </div>
                </>
              ) : (
                <div className="hardware-gate-reason ok">
                  EN+VI: Hardware gate passed. Hệ thống cho phép command ingress phần cứng.
                </div>
              )}
              {hardwareGate.checklist ? (
                <details className="hardware-gate-json">
                  <summary>Checklist JSON / Chi tiết checklist</summary>
                  <pre>{prettyJson(hardwareGate.checklist)}</pre>
                </details>
              ) : null}
            </section>

            <RuntimeStateBanner runtime={state.runtime} />

            {state.messages.length === 0 ? (
              <div className="msg system">
                <div className="msg-avatar">SYS</div>
                <div className="msg-body">
                  <div className="msg-name">System · waiting</div>
                  <div className="msg-bubble">
                    Waiting for bridge-backed lifecycle events. This panel renders backend truth only.
                  </div>
                </div>
              </div>
            ) : null}

            {state.messages.map((message) => {
              const role = toMessageRole(message.origin);
              const needsConfirmation =
                message.tag === 'NEEDS_CONFIRMATION' && activeCommand?.commandId === message.commandId;

              return (
                <div key={message.id} className={`msg ${role}`}>
                  <div className="msg-avatar">{avatarForRole(role)}</div>
                  <div className="msg-body">
                    <div className="msg-name">
                      {nameForRole(role)} · {formatTimestamp(message.timestamp)}
                    </div>
                    {message.tag ? <span className={tagClassName(message.tag)}>{humanizeLabel(message.tag)}</span> : null}
                    <div className="msg-bubble">{message.text}</div>

                    {needsConfirmation ? (
                      <div className="confirm-btns">
                        <button
                          className="btn-confirm yes"
                          disabled={!canConfirmCommands || !activeCommand?.planFingerprint}
                          onClick={() => {
                            void handleConfirmClick();
                          }}
                        >
                          Confirm and execute
                        </button>
                        <button
                          className="btn-confirm no"
                          disabled={!canAbortCommands}
                          onClick={() => {
                            void handleAbortClick('Operator aborted from HMI confirmation panel.');
                          }}
                        >
                          Abort
                        </button>
                      </div>
                    ) : null}
                  </div>
                </div>
              );
            })}
          </div>

          <div className="chat-input-wrap">
            <div className="input-row">
              <textarea
                className="chat-input"
                rows={1}
                value={draft}
                disabled={!canSubmitCommands}
                onChange={(event) => setDraft(event.target.value)}
                onKeyDown={handleInputKeyDown}
                placeholder={
                  readOnlyBridge
                    ? state.mode === 'hardware' && !hardwareGate.unlocked
                      ? `Hardware gate locked: ${primaryHardwareGateReason} | VI: ${primaryHardwareGateReasonVi}`
                      : 'Command ingress is read only until mode + telemetry + preflight gates are satisfied.'
                    : 'Type intent in English or Vietnamese, then press Enter or Submit.'
                }
              />
              <button
                className="send-btn"
                disabled={!canSubmitCommands || draft.trim().length === 0}
                onClick={() => {
                  void handleSubmit();
                }}
              >
                Submit
              </button>
            </div>
            {submitError ? <div className="input-error">{submitError}</div> : null}
            {actionFeedback ? (
              <div className={`input-feedback ${actionFeedback.level}`}>
                {formatTimestamp(actionFeedback.timestamp)} · {actionFeedback.message}
              </div>
            ) : null}
            <div className="hint-row">
              {INTENT_TEMPLATES.map((template) => (
                <button
                  key={`hint-${template.id}`}
                  type="button"
                  className="hint"
                  onClick={() => {
                    setDraft(template.intent);
                    setSubmitError(null);
                  }}
                >
                  {template.intent}
                </button>
              ))}
            </div>
          </div>
        </main>

        <aside className="right-panel">
          <div className="panel-header">System Metrics</div>

          <div className="kpi-grid">
            <div className="kpi">
              <div className="kpi-label">Cycle time</div>
              <div className="kpi-value cyan">{formatMetric(cycleSeconds, 1, ' s')}</div>
              <div className="kpi-sub">last execution</div>
            </div>
            <div className="kpi">
              <div className="kpi-label">Commands</div>
              <div className="kpi-value green">{commandCount}</div>
              <div className="kpi-sub">session total</div>
            </div>
            <div className="kpi">
              <div className="kpi-label">Plan score</div>
              <div className="kpi-value blue">{formatMetric(state.planMetrics?.score ?? null, 1)}</div>
              <div className="kpi-sub">latest plan</div>
            </div>
            <div className="kpi">
              <div className="kpi-label">Cartesian</div>
              <div className="kpi-value amber">{formatMetric(state.planMetrics?.cartesianCompletionPct ?? null, 1, '%')}</div>
              <div className="kpi-sub">completion</div>
            </div>
          </div>

          <section className="section">
            <div className="section-title">Telemetry Sources</div>
            <div className="topic-monitor">
              {topicRows.map((row) => (
                <div key={row.key} className="topic-row">
                  <div className="topic-name">{row.name}</div>
                  <div className="topic-hz">{row.rateLabel}</div>
                  <div className="topic-bar">
                    <div className="topic-fill" style={{ width: `${row.fillWidth}%` }}></div>
                  </div>
                </div>
              ))}
            </div>
          </section>

          <section className="section">
            <div className="section-title">Command Review</div>
            <div className="summary-chip">{activeCommand?.summaryLabel ?? 'No active command review'}</div>
            <div
              className={`result-chip ${
                activeCommand?.lifecycleState === 'SUCCEEDED'
                  ? 'success'
                  : activeCommand?.lifecycleState === 'FAILED' ||
                      activeCommand?.lifecycleState === 'REJECTED' ||
                      activeCommand?.lifecycleState === 'CANCELLED' ||
                      activeCommand?.lifecycleState === 'EXPIRED'
                    ? 'fail'
                    : 'pending'
              }`}
            >
              {activeCommand?.lifecycleState ? humanizeLabel(activeCommand.lifecycleState) : 'Idle'}
            </div>
            <div className="lease-caption">
              {activeCommand?.planFingerprint
                ? `Plan fingerprint: ${activeCommand.planFingerprint.slice(0, 12)}...`
                : 'No validated plan fingerprint yet.'}
            </div>
            {confirmationReasons.length > 0 ? (
              <div className="lease-caption">Confirm reasons: {confirmationReasons.join(' ')}</div>
            ) : null}
            {blockingReasons.length > 0 ? (
              <div className="lease-caption">Blocking reasons: {blockingReasons.join(' ')}</div>
            ) : null}
            {declineReason ? <div className="lease-caption lease-caption-error">Decline reason: {declineReason}</div> : null}
            {executionSummary ? <div className="lease-caption">{executionSummary}</div> : null}
            {traceSteps.length > 0 ? (
              <div className="trace-list">
                {traceSteps.map((step) => (
                  <div key={step.key} className={`trace-step ${step.status}`}>
                    <div className="trace-head">
                      <span className="trace-label">{step.label}</span>
                      <span className={`trace-state ${step.status}`}>{step.status.toUpperCase()}</span>
                    </div>
                    <div className="trace-summary">{step.summary}</div>
                    {step.details ? (
                      <details className="trace-json">
                        <summary>View JSON</summary>
                        <pre>{prettyJson(step.details)}</pre>
                      </details>
                    ) : null}
                  </div>
                ))}
              </div>
            ) : null}
          </section>

          <section className="section">
            <div className="section-title">Control Lease</div>
            <div className="lease-actions">
              {canReleaseLease ? (
                <button className="secondary-btn" onClick={() => void releaseLease()}>
                  Release lease
                </button>
              ) : canAcquireLease ? (
                <button className="secondary-btn" onClick={() => void acquireControllerLease()}>
                  Request control lease
                </button>
              ) : null}
            </div>
            <div className="lease-caption">
              {readOnlyBridge
                ? state.mode === 'hardware'
                  ? `Hardware gate locked: ${primaryHardwareGateReason}`
                  : 'Telemetry is live, but command ingress stays read-only until command-capable mode and freshness gates are satisfied.'
                : state.lease.statusText}
            </div>
            {readOnlyBridge && state.mode === 'hardware' ? (
              <div className="lease-caption">VI: {primaryHardwareGateReasonVi}</div>
            ) : null}
            <div className="lease-caption">
              Execution allowed: {state.capabilities.executionAllowed ? 'yes' : 'no'} · Replay: {state.capabilities.replayAvailable ? 'yes' : 'no'}
            </div>
          </section>

          <div className="panel-header panel-header-secondary">System Log</div>
          <div className="log-area">
            {logEntries.map((entry) => (
              <div key={entry.id} className="log-entry">
                <span className="log-time">{entry.time}</span>
                <span className={`log-level ${entry.level}`}>{entry.level.toUpperCase()}</span>
                <span className="log-msg">{entry.message}</span>
              </div>
            ))}
          </div>
        </aside>
      </div>
    </div>
  );
}
