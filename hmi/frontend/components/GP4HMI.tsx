import { useMemo, useState } from 'react';

import type { CommandLifecycleState, GP4BridgeClient, JointPosition, MessageTag } from '../../shared/contracts';
import { useGP4Bridge } from '../hooks/useGP4Bridge';
import { RuntimeStateBanner } from './RuntimeStateBanner';

const STATE_STEPS: CommandLifecycleState[] = [
  'IDLE',
  'PARSED',
  'VALIDATED',
  'PLANNED',
  'QUALITY_CHECKED',
  'READY_FOR_CONFIRM',
  'EXECUTING',
  'SUCCEEDED',
];

const JOINT_ORDER = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];

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

function tagClassName(tag?: MessageTag): string {
  if (!tag) {
    return 'tag';
  }

  const normalized = tag.toLowerCase();
  if (normalized === 'quality_checked') {
    return 'tag planned';
  }
  if (normalized === 'ready_for_confirm') {
    return 'tag confirm';
  }
  if (normalized === 'executing') {
    return 'tag executing';
  }
  if (normalized === 'failed' || normalized === 'rejected' || normalized === 'debug_required') {
    return 'tag error';
  }
  return `tag ${normalized}`;
}

function lifecycleIndex(state: CommandLifecycleState | null | undefined): number {
  if (!state) {
    return -1;
  }
  return STATE_STEPS.indexOf(state);
}

interface GP4HMIProps {
  client: GP4BridgeClient;
  sessionId: string;
  operatorId: string;
}

export function GP4HMI({ client, sessionId, operatorId }: GP4HMIProps) {
  const [draft, setDraft] = useState('');
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
  const canSubmitCommands = state.capabilities.canSubmitCommands && !blockingRuntime && isController;
  const canAcquireLease = state.capabilities.canAcquireLease && !readOnlyBridge;
  const canReleaseLease = !readOnlyBridge && isController && state.lease.leaseToken !== null;
  const canConfirmCommands = state.capabilities.canConfirmCommands && !blockingRuntime && isController;
  const canAbortCommands = state.capabilities.canAbortCommands && isController;

  const activeStepIndex = lifecycleIndex(state.activeCommand?.lifecycleState);
  const orderedJoints = useMemo(() => {
    const jointMap = new Map(state.jointPositions.map((joint) => [joint.name, joint]));
    return JOINT_ORDER.map((name) => jointMap.get(name) ?? { name, positionDeg: null, minDeg: -180, maxDeg: 180 });
  }, [state.jointPositions]);

  const handleSubmit = async () => {
    const rawText = draft.trim();
    if (!rawText || !canSubmitCommands) {
      return;
    }
    await submitCommand(rawText);
    setDraft('');
  };

  return (
    <div className="hmi-shell">
      <div className="hmi">
        <div className="topbar">
          <div className="topbar-left">
            <div className="logo">GP4 Robot HMI</div>
            <span className={`mode-badge ${state.mode === 'hardware' ? 'hw' : ''}`}>
              {state.mode === 'unknown' ? 'bridge pending' : state.mode === 'hardware' ? 'hardware' : 'simulation'}
            </span>
            <span className={`lease-badge ${isController ? 'controller' : 'observer'}`}>{readOnlyBridge ? 'read only' : isController ? 'controller' : 'observer'}</span>
          </div>
          <div className="topbar-right">
            {state.connections.map((connection, index) => (
              <div key={connection.name} className="topbar-link-group">
                <div className={`dot ${connection.health === 'healthy' ? 'green' : connection.health === 'degraded' ? 'amber' : 'red'}`}></div>
                <span className="dot-label">{connection.label}</span>
                {index < state.connections.length - 1 ? <div className="sep" /> : null}
              </div>
            ))}
          </div>
        </div>

        <div className="chat-area">
          <div className="messages">
            <RuntimeStateBanner runtime={state.runtime} />

            {state.messages.length === 0 ? (
              <div className="msg bot">
                <span className="msg-meta">system</span>
                <div className="bubble empty-bubble">
                  Waiting for bridge-backed lifecycle events. This panel renders backend truth only.
                </div>
              </div>
            ) : null}

            {state.messages.map((message) => (
              <div key={message.id} className={`msg ${message.origin === 'operator' ? 'user' : 'bot'}`}>
                <span className="msg-meta">{message.origin === 'operator' ? message.timestamp : `system · ${message.timestamp}`}</span>
                {message.tag ? <span className={tagClassName(message.tag)}>{message.tag.replace(/_/g, ' ')}</span> : null}
                <div className="bubble">{message.text}</div>
                {message.tag === 'READY_FOR_CONFIRM' && state.activeCommand?.commandId === message.commandId ? (
                  <>
                    <button
                      className="confirm-btn"
                      disabled={!canConfirmCommands}
                      onClick={() => {
                        void confirmActiveCommand();
                      }}
                    >
                      Confirm &amp; Execute
                    </button>
                    <button
                      className="abort-btn"
                      disabled={!canAbortCommands}
                      onClick={() => {
                        void abortActiveCommand('Operator aborted from HMI');
                      }}
                    >
                      Abort
                    </button>
                  </>
                ) : null}
              </div>
            ))}
          </div>

          <div className="input-bar">
            <textarea
              value={draft}
              disabled={!canSubmitCommands}
              onChange={(event) => setDraft(event.target.value)}
              placeholder={readOnlyBridge ? 'Telemetry bridge v1 is read-only. Command submission is disabled.' : 'Nhập lệnh tự nhiên... (vd: di chuyển lên 10cm, vẽ hình tròn, về home)'}
            />
            <button className="send-btn" disabled={!canSubmitCommands || !draft.trim()} onClick={handleSubmit}>
              Gửi ↗
            </button>
          </div>
        </div>

        <div className="sidebar">
          <div className="section">
            <div className="sec-title">State machine</div>
            <div className="state-steps">
              {STATE_STEPS.map((step, index) => {
                const isDone = activeStepIndex > index;
                const isActive = activeStepIndex === index;
                return (
                  <div key={step} className={`state-item ${isDone ? 'done' : ''} ${isActive ? 'active' : ''}`.trim()}>
                    <div className="state-icon">{isDone ? '✓' : isActive ? '●' : ''}</div>
                    {step.replace(/_/g, ' ')}
                  </div>
                );
              })}
            </div>
          </div>

          <div className="section">
            <div className="sec-title">Joint positions (deg)</div>
            <div className="joints">
              {orderedJoints.map((joint) => (
                <div key={joint.name} className="joint-row">
                  <span className="joint-label">{joint.name}</span>
                  <div className="joint-bar-bg">
                    <div className="joint-bar" style={{ width: `${toPercent(joint)}%` }} />
                  </div>
                  <span className="joint-val">{formatAngle(joint.positionDeg)}</span>
                </div>
              ))}
            </div>
          </div>

          <div className="section">
            <div className="sec-title">Plan quality (last)</div>
            <div className="metrics">
              <div className="metric-card"><div className="metric-label">Score</div><div className="metric-val good">{formatMetric(state.planMetrics?.score ?? null)}</div></div>
              <div className="metric-card"><div className="metric-label">Path length</div><div className="metric-val">{formatMetric(state.planMetrics?.pathLengthRad ?? null, 2, ' rad')}</div></div>
              <div className="metric-card"><div className="metric-label">Smoothness</div><div className="metric-val good">{formatMetric(state.planMetrics?.smoothness ?? null)}</div></div>
              <div className="metric-card"><div className="metric-label">Clearance</div><div className="metric-val warn">{formatMetric(state.planMetrics?.clearanceM ?? null, 2, ' m')}</div></div>
              <div className="metric-card"><div className="metric-label">Cartesian %</div><div className="metric-val good">{formatMetric(state.planMetrics?.cartesianCompletionPct ?? null, 1, '%')}</div></div>
              <div className="metric-card"><div className="metric-label">Replan #</div><div className="metric-val">{formatMetric(state.planMetrics?.replanCount ?? null, 0)}</div></div>
            </div>
          </div>

          <div className="section">
            <div className="sec-title">Robot status</div>
            <div className="robot-status">
              <div className="status-row"><span className="status-key">Servo</span><span className={`status-val ${state.runtime.robotStatus.servoState === 'ON' ? 'on' : state.runtime.robotStatus.servoState === 'OFF' ? 'off' : 'warn'}`}>{state.runtime.robotStatus.servoState}</span></div>
              <div className="status-row"><span className="status-key">E-stop</span><span className={`status-val ${state.runtime.robotStatus.eStop === 'CLEAR' ? 'on' : state.runtime.robotStatus.eStop === 'ACTIVE' ? 'off' : 'warn'}`}>{state.runtime.robotStatus.eStop}</span></div>
              <div className="status-row"><span className="status-key">Alarm</span><span className={`status-val ${state.runtime.robotStatus.alarmState === 'NONE' ? 'on' : state.runtime.robotStatus.alarmState === 'ACTIVE' ? 'off' : 'warn'}`}>{state.runtime.robotStatus.alarmState}</span></div>
              <div className="status-row"><span className="status-key">Mode</span><span className="status-val">{state.runtime.robotStatus.motionMode ?? '--'}</span></div>
              <div className="status-row"><span className="status-key">Trajectory pts</span><span className="status-val">{state.runtime.robotStatus.trajectoryPointsUsed !== null && state.runtime.robotStatus.trajectoryPointsCapacity !== null ? `${state.runtime.robotStatus.trajectoryPointsUsed} / ${state.runtime.robotStatus.trajectoryPointsCapacity}` : '--'}</span></div>
            </div>
          </div>

          <div className="section">
            <div className="sec-title">Last command</div>
            <span className="cmd-chip">{state.activeCommand?.summaryLabel ?? 'No backend command yet'}</span>
            <span className={`result-chip ${state.activeCommand?.lifecycleState === 'SUCCEEDED' ? 'success' : state.activeCommand?.lifecycleState === 'FAILED' || state.activeCommand?.lifecycleState === 'REJECTED' || state.activeCommand?.lifecycleState === 'ABORTED' ? 'fail' : 'pending'}`}>
              {state.activeCommand?.lifecycleState?.toLowerCase().replace(/_/g, ' ') ?? 'pending bridge'}
            </span>
            <div className="lease-actions">
              {canAcquireLease ? (
                <button className="secondary-btn" onClick={() => void acquireControllerLease()}>
                  Request control lease
                </button>
              ) : canReleaseLease ? (
                <button className="secondary-btn" onClick={() => void releaseLease()}>
                  Release lease
                </button>
              ) : null}
            </div>
            <div className="lease-caption">
              {readOnlyBridge
                ? 'Telemetry bridge v1 is read-only. Lease acquisition and command control stay backend-disabled.'
                : state.lease.statusText}
            </div>
            <div className="lease-caption">
              {readOnlyBridge ? 'Control-capable ROS paths remain fail-closed in this build.' : null}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
