import { useEffect, useMemo, useState, type KeyboardEvent } from 'react';

import type {
  BridgeConnection,
  ChatMessage,
  CommandLifecycleState,
  CommandView,
  GP4BridgeClient,
  JointPosition,
  MessageOrigin,
  TelemetrySourceStatus,
} from '../../shared/contracts';
import { useGP4Bridge } from '../hooks/useGP4Bridge';
import { RuntimeStateBanner } from './RuntimeStateBanner';

const JOINT_ORDER = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];

const QUICK_COMMANDS = [
  'home',
  'stop',
  'move up 10 cm',
  'move down 10 cm',
  'move joint 1 +10 deg',
  'move joint 2 -5 deg',
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

interface GP4HMIProps {
  client: GP4BridgeClient;
  sessionId: string;
  operatorId: string;
}

export function GP4HMI({ client, sessionId, operatorId }: GP4HMIProps) {
  const [draft, setDraft] = useState('');
  const [clockText, setClockText] = useState(() => formatClock(new Date()));
  const [submitError, setSubmitError] = useState<string | null>(null);

  const {
    state,
    isController,
    blockingRuntime,
    submitCommand,
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

  const activeCommand = state.activeCommand;
  const blockingReasons = activeCommand?.validationResult?.blockingReasons ?? [];
  const confirmationReasons = activeCommand?.validationResult?.confirmationReasons ?? [];
  const executionSummary = activeCommand?.executionResult?.summary ?? null;

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

    if (entries.length > 0) {
      return entries;
    }

    return [
      {
        id: 'runtime-bootstrap',
        time: formatTimestamp(state.generatedAt),
        level: state.runtime.blocking ? 'warn' : 'info',
        message: state.runtime.statusText,
      },
    ];
  }, [state.generatedAt, state.messages, state.runtime.blocking, state.runtime.statusText]);

  const handleSubmit = async () => {
    const rawText = draft.trim();
    if (!rawText || !canSubmitCommands) {
      return;
    }

    try {
      const response = await submitCommand(rawText);
      if (!response.accepted) {
        setSubmitError(response.reason ?? 'Command rejected by supervisor validation.');
        return;
      }
      setDraft('');
      setSubmitError(null);
    } catch (error: unknown) {
      setSubmitError(error instanceof Error ? error.message : 'Failed to submit intent.');
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
                void abortActiveCommand('Operator requested topbar abort from HMI.');
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
              {QUICK_COMMANDS.map((command) => (
                <button
                  key={command}
                  type="button"
                  className="qcmd"
                  onClick={() => {
                    setDraft(command);
                    setSubmitError(null);
                  }}
                >
                  <span className="cmd-icon">&gt;</span>
                  {command}
                </button>
              ))}
            </div>
          </section>
        </aside>

        <main className="center-panel">
          <div className="chat-header">
            <span className="chat-title">LLM Command Interface - Yaskawa GP4</span>
            <span className="chat-sub">
              transport {state.transportState} · schema {state.schemaVersion} · {readOnlyBridge ? 'read only' : 'command ingress enabled'}
            </span>
          </div>

          <div className="chat-messages">
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
                            void confirmActiveCommand();
                          }}
                        >
                          Confirm and execute
                        </button>
                        <button
                          className="btn-confirm no"
                          disabled={!canAbortCommands}
                          onClick={() => {
                            void abortActiveCommand('Operator aborted from HMI confirmation panel.');
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
                    ? 'Command ingress is read only until simulation mode and telemetry freshness gates are satisfied.'
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
            <div className="hint-row">
              {QUICK_COMMANDS.map((hint) => (
                <button
                  key={`hint-${hint}`}
                  type="button"
                  className="hint"
                  onClick={() => {
                    setDraft(hint);
                    setSubmitError(null);
                  }}
                >
                  {hint}
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
            {executionSummary ? <div className="lease-caption">{executionSummary}</div> : null}
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
                ? 'Telemetry remains live but command ingress stays read only until simulation mode is active and hardware freshness is verified.'
                : state.lease.statusText}
            </div>
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
