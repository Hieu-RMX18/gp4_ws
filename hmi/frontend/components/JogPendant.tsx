import { useState } from 'react';
import type { JogBridgeStatusSnapshot, JogCommandRequest, JogMode, JointPosition } from '../../shared/contracts';

interface JogPendantProps {
  jogBridgeStatus: JogBridgeStatusSnapshot;
  jointPositions: JointPosition[];
  onActivateBridge: () => Promise<{ accepted: boolean; message: string }>;
  onDeactivateBridge: () => Promise<{ accepted: boolean; message: string }>;
  onJogCommand: (cmd: JogCommandRequest) => Promise<{ accepted: boolean; message: string }>;
}

// Yaskawa GP4 axis layout — matches physical teach pendant
const LEFT_AXES = [
  { index: 0, label: 'S', sublabel: 'Joint 1' },
  { index: 1, label: 'L', sublabel: 'Joint 2' },
  { index: 2, label: 'U', sublabel: 'Joint 3' },
];
const RIGHT_AXES = [
  { index: 3, label: 'R', sublabel: 'Joint 4' },
  { index: 4, label: 'B', sublabel: 'Joint 5' },
  { index: 5, label: 'T', sublabel: 'Joint 6' },
];

type BridgePillTone = 'green' | 'amber' | 'red' | 'gray' | 'blue';

function bridgePillTone(state: string): BridgePillTone {
  if (state === 'ACTIVE') return 'green';
  if (state === 'READY') return 'blue';
  if (state === 'STARTING' || state === 'HALTING' || state === 'BUSY_RETRY') return 'amber';
  if (state === 'ERROR' || state === 'REJECTED_NOT_READY' || state === 'REJECTED_FJT_ACTIVE' || state === 'TIMEOUT') return 'red';
  return 'gray';
}

export function JogPendant({
  jogBridgeStatus: s,
  jointPositions,
  onActivateBridge,
  onDeactivateBridge,
  onJogCommand,
}: JogPendantProps) {
  const [mode, setMode] = useState<JogMode>('discrete');
  const [velocityScale, setVelocityScale] = useState(0.05);
  const [stepDegrees, setStepDegrees] = useState(1.0);
  const [feedback, setFeedback] = useState<string | null>(null);
  const [activating, setActivating] = useState(false);

  const canJog = s.state === 'READY' || s.state === 'ACTIVE';
  const canActivate = s.state !== 'ACTIVE' && s.state !== 'STARTING';
  const canDeactivate = s.state !== 'IDLE';
  const errorText = s.lastError || s.rejectionReason || null;

  const handleActivate = async () => {
    setActivating(true);
    setFeedback(null);
    try {
      const r = await onActivateBridge();
      setFeedback(r.accepted ? 'Bridge activated' : `Activate failed: ${r.message}`);
    } catch (e) {
      setFeedback(`Activate error: ${e instanceof Error ? e.message : 'unknown'}`);
    } finally {
      setActivating(false);
    }
  };

  const handleDeactivate = async () => {
    setActivating(true);
    setFeedback(null);
    try {
      const r = await onDeactivateBridge();
      setFeedback(r.accepted ? 'Bridge deactivated' : `Deactivate failed: ${r.message}`);
    } catch (e) {
      setFeedback(`Deactivate error: ${e instanceof Error ? e.message : 'unknown'}`);
    } finally {
      setActivating(false);
    }
  };

  const handleJog = async (jointIndex: number, direction: 1 | -1) => {
    try {
      const r = await onJogCommand({ jointIndex, direction, mode, velocityScale, stepDegrees });
      if (!r.accepted) setFeedback(`Jog rejected: ${r.message}`);
      else setFeedback(null);
    } catch (e) {
      setFeedback(`Jog error: ${e instanceof Error ? e.message : 'unknown'}`);
    }
  };

  return (
    <div className="jog-pendant">
      <div className="jog-experimental-banner">
        <span className="jog-experimental-icon">⚠ EXPERIMENTAL — Joint Jog Pendant</span>
        <span className="jog-experimental-sub">
          Point-queue mode via MotoROS2. Verify robot has stopped after deactivation.
        </span>
      </div>

      {/* ── Bridge Status ──────────────────────────────────────────── */}
      <div className="jog-section">
        <div className="jog-section-title">Bridge Status</div>
        <div className="jog-bridge-status">
          <span className={`jog-bridge-pill ${bridgePillTone(s.state)}`}>
            {s.state}
          </span>
          <div className="jog-status-grid">
            <div className="jog-status-cell">
              <span className="jog-status-label">Robot Ready</span>
              <span className={`jog-status-val ${s.robotReady ? 'green' : 'red'}`}>
                {s.robotReady ? 'YES' : 'NO'}
              </span>
            </div>
            <div className="jog-status-cell">
              <span className="jog-status-label">Servo Active</span>
              <span className={`jog-status-val ${s.servoActive ? 'green' : 'red'}`}>
                {s.servoActive ? 'YES' : 'NO'}
              </span>
            </div>
            <div className="jog-status-cell">
              <span className="jog-status-label">Bridge Active</span>
              <span className={`jog-status-val ${s.bridgeActive ? 'green' : 'red'}`}>
                {s.bridgeActive ? 'YES' : 'NO'}
              </span>
            </div>
            <div className="jog-status-cell">
              <span className="jog-status-label">Points Queued</span>
              <span className="jog-status-val cyan">{s.pointsQueued}</span>
            </div>
          </div>

          <div style={{ display: 'flex', gap: '8px' }}>
            <button
              className="jog-bridge-btn jog-bridge-activate"
              disabled={!canActivate || activating}
              onClick={() => void handleActivate()}
            >
              Activate
            </button>
            <button
              className="jog-bridge-btn jog-bridge-deactivate"
              disabled={!canDeactivate || activating}
              onClick={() => void handleDeactivate()}
            >
              Deactivate
            </button>
          </div>

          {errorText ? <div className="jog-error">{errorText}</div> : null}
          {feedback ? <div className="jog-error">{feedback}</div> : null}
        </div>
      </div>

      {/* ── Parameters ─────────────────────────────────────────────── */}
      <div className="jog-section">
        <div className="jog-section-title">Parameters</div>
        <div className="jog-param-row">
          <span className="jog-param-label">Mode</span>
          <div className="jog-mode-toggle">
            <button
              className={`jog-mode-btn ${mode === 'discrete' ? 'active' : ''}`}
              disabled={!canJog}
              onClick={() => setMode('discrete')}
            >
              Discrete
            </button>
            <button
              className={`jog-mode-btn ${mode === 'continuous' ? 'active' : ''}`}
              disabled={!canJog}
              onClick={() => setMode('continuous')}
            >
              Continuous
            </button>
          </div>
        </div>
        <div className="jog-param-row">
          <span className="jog-param-label">Velocity {(velocityScale * 100).toFixed(0)}%</span>
          <input
            type="range"
            className="jog-slider"
            min={1}
            max={30}
            value={Math.round(velocityScale * 100)}
            disabled={!canJog}
            onChange={(e) => setVelocityScale(Number(e.target.value) / 100)}
          />
        </div>
        {mode === 'discrete' ? (
          <div className="jog-param-row">
            <span className="jog-param-label">Step {stepDegrees.toFixed(1)}°</span>
            <input
              type="range"
              className="jog-slider"
              min={5}
              max={50}
              value={Math.round(stepDegrees * 10)}
              disabled={!canJog}
              onChange={(e) => setStepDegrees(Number(e.target.value) / 10)}
            />
          </div>
        ) : null}
      </div>

      {/* ── Joints (Yaskawa teach pendant layout) ────────────────── */}
      <div className="jog-section">
        <div className="jog-section-title">Joints</div>
        {canJog ? (
          <div className="jog-tp-columns">
            {[LEFT_AXES, RIGHT_AXES].map((group, gi) => (
              <div key={gi} className="jog-tp-col">
                <div className="jog-tp-col-label">{gi === 0 ? 'Base' : 'Wrist'}</div>
                {group.map((axis) => {
                  const jp = jointPositions[axis.index];
                  const deg = jp?.positionDeg?.toFixed(1) ?? '--';
                  return (
                    <div key={axis.label} className="jog-tp-axis">
                      <div className="jog-tp-axis-header">
                        <span className="jog-tp-axis-label">{axis.label}</span>
                        <span className="jog-tp-axis-val">{deg}°</span>
                      </div>
                      <div className="jog-btn-pair">
                        <button
                          className={`jog-btn ${gi === 0 ? 'jog-btn-neg' : 'jog-btn-neg'} jog-tp-btn-left`}
                          disabled={!canJog}
                          onClick={() => void handleJog(axis.index, -1)}
                        >
                          {axis.label}−
                        </button>
                        <button
                          className={`jog-btn ${gi === 0 ? 'jog-btn-pos' : 'jog-btn-pos'} jog-tp-btn-right`}
                          disabled={!canJog}
                          onClick={() => void handleJog(axis.index, 1)}
                        >
                          {axis.label}+
                        </button>
                      </div>
                    </div>
                  );
                })}
              </div>
            ))}
          </div>
        ) : (
          <div className="jog-blocked-msg">
            Activate bridge to enable joint jogging.
          </div>
        )}
      </div>

      {/* ── Stop ───────────────────────────────────────────────────── */}
      <div className="jog-stop-section">
        <button
          className="jog-stop-btn"
          disabled={!canDeactivate || activating}
          onClick={() => void handleDeactivate()}
        >
          STOP
        </button>
      </div>

      {s.bridgeActive ? (
        <div className="jog-exclusive-notice">
          Jog bridge is active — FJT trajectory path is blocked.
        </div>
      ) : null}
    </div>
  );
}
