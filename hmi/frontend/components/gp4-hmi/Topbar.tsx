import { useState } from 'react';
import type { ServoControlResponse } from '../../../shared/contracts';
import type { StatusPillView, LogLevel } from './types';

interface TopbarProps {
  statusPills: StatusPillView[];
  clockText: string;
  canAbort: boolean;
  canStartServo: boolean;
  canHoldServo: boolean;
  onStartServo: () => Promise<ServoControlResponse | null>;
  onHoldServo: () => Promise<ServoControlResponse | null>;
  onAbort: () => void;
  onActionFeedback: (level: LogLevel, message: string) => void;
}

export function Topbar({
  statusPills,
  clockText,
  canAbort,
  canStartServo,
  canHoldServo,
  onStartServo,
  onHoldServo,
  onAbort,
  onActionFeedback,
}: TopbarProps) {
  const [pendingServoAction, setPendingServoAction] = useState<'start' | 'hold' | null>(null);

  const runServoAction = async (action: 'start' | 'hold') => {
    const actionAllowed = action === 'start' ? canStartServo : canHoldServo;
    if (!actionAllowed || pendingServoAction !== null) {
      return;
    }
    setPendingServoAction(action);
    try {
      const res = action === 'start' ? await onStartServo() : await onHoldServo();
      const label = action === 'start' ? 'Servo START' : 'Servo HOLD';
      if (res === null) {
        onActionFeedback('err', `${label} blocked · controller lease required`);
        return;
      }
      onActionFeedback(res.accepted ? 'ok' : 'err', `${label} · ${res.message}`);
    } catch (e: unknown) {
      const label = action === 'start' ? 'Servo START' : 'Servo HOLD';
      onActionFeedback('err', `${label} failed · ${e instanceof Error ? e.message : String(e)}`);
    } finally {
      setPendingServoAction(null);
    }
  };

  return (
    <header className="topbar">
      <div className="logo">
        <div className="logo-icon">HMI</div>
        GP4 <span>Yaskawa</span>
      </div>
      <div className="top-divider" />
      <div className="status-pills">
        {statusPills.map((pill) => {
          const toneIcon: Record<string, string> = {
            green: '✓',
            blue: '●',
            cyan: '●',
            amber: '⚠',
            red: '✕',
            gray: '○',
          };
          return (
            <span key={pill.key} className={`pill ${pill.tone}`}>
              <span className={`dot ${pill.tone}`}></span>
              <span className="pill-icon">{toneIcon[pill.tone]}</span>
              {pill.label}
            </span>
          );
        })}
      </div>
      <div className="top-right">
        <span className="top-time">{clockText}</span>
        <div className="top-divider"></div>
        <div className="servo-group">
          <button
            className="servo-btn servo-start"
            disabled={!canStartServo || pendingServoAction !== null}
            onClick={() => { void runServoAction('start'); }}
            title={canStartServo ? 'Servo START' : 'Hardware gate and controller lease required'}
          >
            {pendingServoAction === 'start' ? 'STARTING' : 'START'}
          </button>
          <button
            className="servo-btn servo-stop"
            disabled={!canHoldServo || pendingServoAction !== null}
            onClick={() => { void runServoAction('hold'); }}
            title={canHoldServo ? 'Servo HOLD' : 'Hardware mode and controller lease required'}
          >
            {pendingServoAction === 'hold' ? 'HOLDING' : 'HOLD'}
          </button>
        </div>
        <div className="top-divider"></div>
        <button
          className="estop-btn"
          disabled={!canAbort}
          onClick={onAbort}
        >
          Abort command
        </button>
      </div>
    </header>
  );
}
