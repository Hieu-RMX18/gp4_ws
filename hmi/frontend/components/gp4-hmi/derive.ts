// Pure derivation helpers extracted from GP4HMI.tsx.
// No React, no side-effects — only data transforms.

import type {
  BridgeConnection,
  ChatMessage,
  CommandLifecycleState,
  CommandMutationResponse,
  CommandView,
  JointPosition,
  MessageOrigin,
  PipelineTrace,
  SequenceView,
  TelemetrySourceStatus,
} from '../../../shared/contracts';

import type {
  LogEntryView,
  LogLevel,
  MessageRole,
  PillTone,
  PipelineStatus,
  TraceStepView,
} from './types';

// ── Constants ───────────────────────────────────────────────────────────────

export const JOINT_ORDER = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];

const DISPLAY_TIME_ZONE = 'Asia/Ho_Chi_Minh';

export const VALIDATED_OR_BEYOND: ReadonlySet<CommandLifecycleState> = new Set([
  'NEEDS_CONFIRMATION', 'CONFIRMED', 'EXECUTION_REQUESTED', 'EXECUTING',
  'SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELLED', 'EXPIRED',
]);

export const CONFIRMED_OR_BEYOND: ReadonlySet<CommandLifecycleState> = new Set([
  'CONFIRMED', 'EXECUTION_REQUESTED', 'EXECUTING',
  'SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELLED', 'EXPIRED',
]);

export const EXECUTION_REQUESTED_OR_BEYOND: ReadonlySet<CommandLifecycleState> = new Set([
  'EXECUTION_REQUESTED', 'EXECUTING',
  'SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELLED', 'EXPIRED',
]);

export const EXECUTING_OR_BEYOND: ReadonlySet<CommandLifecycleState> = new Set([
  'EXECUTING', 'SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELLED', 'EXPIRED',
]);

export const TERMINAL_FAILURE_STATES: ReadonlySet<CommandLifecycleState> = new Set([
  'FAILED', 'REJECTED', 'CANCELLED', 'EXPIRED',
]);

// ── Primitives ──────────────────────────────────────────────────────────────

export function isInStateSet(
  state: CommandLifecycleState | null | undefined,
  allowed: ReadonlySet<CommandLifecycleState>,
): boolean {
  return state !== null && state !== undefined && allowed.has(state);
}

export function toPercent(joint: JointPosition): number {
  if (joint.positionDeg === null) return 0;
  const span = joint.maxDeg - joint.minDeg;
  if (span <= 0) return 0;
  return Math.max(0, Math.min(100, ((joint.positionDeg - joint.minDeg) / span) * 100));
}

export function isNearLimit(joint: JointPosition): boolean {
  if (joint.positionDeg === null) return false;
  const span = joint.maxDeg - joint.minDeg;
  if (span <= 0) return false;
  const pct = (joint.positionDeg - joint.minDeg) / span;
  return pct <= 0.05 || pct >= 0.95;
}

export function formatAngle(value: number | null): string {
  if (value === null) return '--';
  return `${value >= 0 ? '+' : ''}${value.toFixed(1)}°`;
}

export function formatMetric(value: number | null, digits = 2, suffix = ''): string {
  if (value === null) return '--';
  return `${value.toFixed(digits)}${suffix}`;
}

// ── Timestamp helpers ───────────────────────────────────────────────────────

export function parseBackendTimestamp(value: string): Date | null {
  const timeOnlyMatch = /^(\d{2}):(\d{2}):(\d{2})(?:\.(\d+))?$/.exec(value);
  if (timeOnlyMatch) {
    const now = new Date();
    const ms = timeOnlyMatch[4] ? Number(timeOnlyMatch[4].padEnd(3, '0').slice(0, 3)) : 0;
    const parsed = new Date(
      Date.UTC(now.getUTCFullYear(), now.getUTCMonth(), now.getUTCDate(),
        Number(timeOnlyMatch[1]), Number(timeOnlyMatch[2]), Number(timeOnlyMatch[3]), ms),
    );
    return Number.isNaN(parsed.getTime()) ? null : parsed;
  }
  const hasTimezone = /(?:Z|[+-]\d{2}:?\d{2})$/i.test(value);
  const parsed = new Date(hasTimezone ? value : `${value}Z`);
  return Number.isNaN(parsed.getTime()) ? null : parsed;
}

function getDateTimeParts(value: Date): Record<string, string> {
  return new Intl.DateTimeFormat('en-GB', {
    timeZone: DISPLAY_TIME_ZONE, year: 'numeric', month: '2-digit', day: '2-digit',
    hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false,
  })
    .formatToParts(value)
    .reduce<Record<string, string>>((parts, part) => {
      if (part.type !== 'literal') parts[part.type] = part.value;
      return parts;
    }, {});
}

export function formatTimestamp(value: string): string {
  const parsed = parseBackendTimestamp(value);
  if (!parsed) return value;
  const parts = getDateTimeParts(parsed);
  const msMatch = /\.(\d+)/.exec(value);
  const msStr = msMatch ? `.${msMatch[1].slice(0, 3)}` : '';
  return `${parts.hour}:${parts.minute}:${parts.second}${msStr}`;
}

export function formatClock(value: Date): string {
  const parts = getDateTimeParts(value);
  return `${parts.year}-${parts.month}-${parts.day} ${parts.hour}:${parts.minute}:${parts.second} GMT+7`;
}

// ── Message / chat helpers ──────────────────────────────────────────────────

export function shouldShowMessageInSystemLog(message: ChatMessage): boolean {
  // Include all system/assistant messages so the full pipeline is visible
  return message.origin !== 'operator';
}

export function toMessageRole(origin: MessageOrigin): MessageRole {
  if (origin === 'operator') return 'user';
  if (origin === 'assistant') return 'assistant';
  return 'system';
}

export function avatarForRole(role: MessageRole): string {
  if (role === 'user') return 'OP';
  if (role === 'assistant') return 'LLM';
  return 'SYS';
}

export function nameForRole(role: MessageRole): string {
  if (role === 'user') return 'Operator';
  if (role === 'assistant') return 'Assistant (LLM)';
  return 'System';
}

export function tagClassName(tag?: string | null): string {
  if (!tag) return 'tag';
  const normalized = tag.toLowerCase();
  if (normalized === 'needs_confirmation') return 'tag confirm';
  if (normalized === 'confirmed' || normalized === 'execution_requested') return 'tag planned';
  if (normalized === 'executing') return 'tag executing';
  if (normalized === 'succeeded') return 'tag success';
  if (normalized === 'failed' || normalized === 'rejected' || normalized === 'cancelled' || normalized === 'expired') return 'tag error';
  if (normalized === 'validating' || normalized === 'validated') return 'tag validated';
  if (normalized === 'parsing' || normalized === 'received') return 'tag parsed';
  return 'tag';
}

export function toLogLevel(message: ChatMessage): LogLevel {
  const tag = (message.tag ?? '').toLowerCase();
  if (tag === 'succeeded' || tag === 'confirmed' || tag === 'execution_requested') return 'ok';
  if (tag === 'needs_confirmation' || tag === 'validating') return 'warn';
  if (tag === 'failed' || tag === 'rejected' || tag === 'cancelled' || tag === 'expired') return 'err';
  if (message.origin === 'assistant') return 'ok';
  return 'info';
}

// ── Tone mappers ────────────────────────────────────────────────────────────

export function toneFromConnectionHealth(health: BridgeConnection['health'] | undefined): PillTone {
  if (health === 'healthy') return 'green';
  if (health === 'degraded') return 'amber';
  return 'red';
}

export function toneFromServoState(servo: 'ON' | 'OFF' | 'UNKNOWN'): PillTone {
  if (servo === 'ON') return 'green';
  if (servo === 'OFF') return 'red';
  return 'amber';
}

export function toneFromMode(mode: 'sim' | 'hardware' | 'unknown'): PillTone {
  if (mode === 'sim') return 'blue';
  if (mode === 'hardware') return 'amber';
  return 'gray';
}

export function toneFromLifecycle(state: CommandLifecycleState | null | undefined): PillTone {
  if (state === null || state === undefined) return 'gray';
  if (state === 'SUCCEEDED') return 'green';
  if (TERMINAL_FAILURE_STATES.has(state)) return 'red';
  if (state === 'EXECUTING' || state === 'EXECUTION_REQUESTED') return 'cyan';
  if (state === 'NEEDS_CONFIRMATION') return 'amber';
  return 'blue';
}

// ── Telemetry topic views ───────────────────────────────────────────────────

export function toTopicFillWidth(freshness: TelemetrySourceStatus['freshnessState']): number {
  if (freshness === 'fresh') return 100;
  if (freshness === 'stale') return 40;
  return 10;
}

export function toTopicRateLabel(freshness: TelemetrySourceStatus['freshnessState']): string {
  if (freshness === 'fresh') return 'FRESH';
  if (freshness === 'stale') return 'STALE';
  return 'DOWN';
}

// ── Reason / gate helpers ───────────────────────────────────────────────────

export function humanizeLabel(value: string): string {
  return value.toLowerCase().split('_')
    .map((token) => token.charAt(0).toUpperCase() + token.slice(1)).join(' ');
}

export function resolveDeclineReason(
  mutationReason: string | null | undefined,
  command: CommandView | null | undefined,
): string | null {
  const trimmed = mutationReason?.trim();
  if (trimmed) return trimmed;
  const reject = command?.rejectReason?.trim();
  if (reject) return reject;
  const blocking = command?.validationResult?.blockingReasons ?? [];
  if (blocking.length > 0) return blocking.join('; ');
  return null;
}

export function summarizeMutationResponse(response: CommandMutationResponse): string {
  const lifecycleState = response.sequence?.lifecycleState ?? response.command?.lifecycleState;
  const stateLabel = lifecycleState ? humanizeLabel(lifecycleState) : 'Idle';
  if (response.accepted) return `Accepted · ${stateLabel}`;
  const reason = resolveDeclineReason(response.reason, response.command ?? null);
  return reason ? `Declined · ${reason}` : `Declined · ${stateLabel}`;
}

export function durationSeconds(command: CommandView | null): number | null {
  if (!command) return null;
  const startIso = command.confirmAt ?? command.createdAt;
  const endIso = command.executeAt;
  if (!startIso || !endIso) return null;
  const startTime = new Date(startIso).getTime();
  const endTime = new Date(endIso).getTime();
  if (Number.isNaN(startTime) || Number.isNaN(endTime) || endTime < startTime) return null;
  return (endTime - startTime) / 1000;
}

export function prettyJson(value: unknown): string {
  return JSON.stringify(value, null, 2) ?? 'null';
}

export function formatPlanFingerprintLog(job: CommandView | SequenceView | null): string {
  return job?.planFingerprint
    ? `Plan fingerprint: ${job.planFingerprint.slice(0, 12)}...`
    : 'No validated plan fingerprint yet.';
}

export function buildReviewLogEntries(
  job: CommandView | SequenceView | null,
  blockingReasons: readonly string[],
  declineReason: string | null,
  fallbackTimestamp: string,
): LogEntryView[] {
  const time = formatTimestamp(job?.createdAt ?? fallbackTimestamp);
  const entries: LogEntryView[] = [
    { id: 'review-plan-fingerprint', time, level: 'info', message: formatPlanFingerprintLog(job), source: null },
  ];
  if (blockingReasons.length > 0) {
    entries.push({ id: 'review-blocking-reasons', time, level: 'warn', message: `Blocking reasons: ${blockingReasons.join(' ')}`, source: null });
  }
  if (declineReason) {
    entries.push({ id: 'review-decline-reason', time, level: 'err', message: `Decline reason: ${declineReason}`, source: null });
  }
  return entries;
}

// ── Pipeline trace steps ────────────────────────────────────────────────────

function groupTraces(
  traces: PipelineTrace[],
  phases: string[],
): PipelineTrace[] {
  return traces.filter(t => phases.includes(t.phase));
}

export function buildTraceSteps(command: CommandView | null): TraceStepView[] {
  if (!command) return [];

  const lifecycle = command.lifecycleState;
  const validation = command.validationResult ?? null;
  const execution = command.executionResult ?? null;
  const blockingReasons = validation?.blockingReasons ?? [];
  const confirmationReasons = validation?.confirmationReasons ?? [];
  const parsedAction = command.parsedIntent ? command.parsedIntent['action'] : undefined;
  const allTraces: PipelineTrace[] = command.pipelineTraces ?? [];

  const parseStatus: PipelineStatus =
    lifecycle === 'PARSING' ? 'active' : lifecycle !== 'RECEIVED' ? 'done' : 'pending';
  const validateStatus: PipelineStatus =
    lifecycle === 'VALIDATING' ? 'active' : isInStateSet(lifecycle, VALIDATED_OR_BEYOND) ? 'done' : 'pending';
  const confirmationStatus: PipelineStatus =
    lifecycle === 'NEEDS_CONFIRMATION' ? 'active' : isInStateSet(lifecycle, CONFIRMED_OR_BEYOND) ? 'done' : 'pending';
  const dispatchStatus: PipelineStatus =
    lifecycle === 'EXECUTION_REQUESTED' ? 'active' : isInStateSet(lifecycle, EXECUTION_REQUESTED_OR_BEYOND) ? 'done' : 'pending';
  const executingStatus: PipelineStatus =
    lifecycle === 'EXECUTING' ? 'active' : isInStateSet(lifecycle, EXECUTING_OR_BEYOND) ? 'done' : 'pending';
  const terminalStatus: PipelineStatus =
    lifecycle === 'SUCCEEDED' ? 'done' : isInStateSet(lifecycle, TERMINAL_FAILURE_STATES) ? 'error' : 'pending';

  const parseSummary = parsedAction
    ? `Parsed action ${String(parsedAction)} from ${command.intentSource} input.`
    : lifecycle === 'REJECTED' ? 'Parser/enrichment rejected this command.' : 'Waiting for parser output.';

  const validationSummary = validation === null
    ? 'Validation has not run yet.'
    : validation.accepted
      ? `Validation accepted · risk=${validation.riskLevel ?? 'unknown'}.`
      : `Validation rejected: ${blockingReasons.join('; ') || 'no reason provided'}.`;

  const confirmationSummary = command.planFingerprint
    ? `Plan fingerprint ready (${command.planFingerprint.slice(0, 12)}...).`
    : lifecycle === 'NEEDS_CONFIRMATION' ? 'Waiting for operator confirmation.' : 'Confirmation gate not reached.';

  const dispatchSummary = isInStateSet(lifecycle, EXECUTION_REQUESTED_OR_BEYOND)
    ? 'Confirmed command forwarded to execution boundary.' : 'Execution request not sent yet.';

  const executingSummary = execution
    ? `Execution status=${execution.status} · ${execution.summary}`
    : lifecycle === 'EXECUTING' ? 'Execution requested; waiting for ROS result.' : 'Execution boundary not reached.';

  const terminalReason = resolveDeclineReason(null, command);
  const resultSummary = command.finalState !== null
    ? terminalReason ? `${humanizeLabel(command.finalState)} · ${terminalReason}` : humanizeLabel(command.finalState)
    : 'No terminal result yet.';

  return [
    { key: 'parse', label: 'Step 1 · Parse intent', status: parseStatus, summary: parseSummary,
      details: { rawText: command.rawText, intentSource: command.intentSource, parsedIntent: command.parsedIntent ?? null },
      traces: groupTraces(allTraces, ['ingress', 'reasoning', 'parsing']) },
    { key: 'validate', label: 'Step 2 · Validate command', status: validateStatus, summary: validationSummary,
      details: validation ? { validationResult: validation } : null,
      traces: groupTraces(allTraces, ['validation']) },
    { key: 'confirm', label: 'Step 3 · Confirmation gate', status: confirmationStatus, summary: confirmationSummary,
      details: { planFingerprint: command.planFingerprint, confirmationExpiresAt: command.confirmationExpiresAt, confirmationReasons },
      traces: groupTraces(allTraces, ['confirmation']) },
    { key: 'dispatch', label: 'Step 4 · Dispatch request', status: dispatchStatus, summary: dispatchSummary,
      details: { dispatchedToRos: execution?.dispatchedToRos ?? false, executionStatus: execution?.status ?? null },
      traces: groupTraces(allTraces, ['routing']) },
    { key: 'executing', label: 'Step 5 · Executing', status: executingStatus, summary: executingSummary,
      details: execution ? { executionResult: execution } : null,
      traces: groupTraces(allTraces, ['execution']) },
    { key: 'result', label: 'Step 6 · Terminal result', status: terminalStatus, summary: resultSummary,
      details: { finalState: command.finalState, rejectReason: command.rejectReason, executionResult: command.executionResult ?? null },
      traces: groupTraces(allTraces, ['terminal', 'complete']) },
  ];
}
