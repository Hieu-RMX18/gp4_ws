import type { GP4BridgeClient } from '../../../shared/contracts';
import type { StatusPillView, LogLevel } from './types';

interface TopbarProps {
  statusPills: StatusPillView[];
  clockText: string;
  canAbort: boolean;
  client: GP4BridgeClient;
  onAbort: () => void;
  onActionFeedback: (level: LogLevel, message: string) => void;
}

export function Topbar({
  statusPills,
  clockText,
  canAbort,
  client,
  onAbort,
  onActionFeedback,
}: TopbarProps) {
  return (
    <header className="topbar">
      <div className="logo">
        <div className="logo-icon">HMI</div>
        GP4 <span>Yaskawa</span>
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
          className="servo-btn servo-start"
          onClick={async () => {
            try {
              const res = await client.startServo();
              onActionFeedback(res.accepted ? 'ok' : 'err', `Servo START · ${res.message}`);
            } catch (e: unknown) {
              onActionFeedback('err', `Servo START failed · ${e instanceof Error ? e.message : String(e)}`);
            }
          }}
        >
          START
        </button>
        <button
          className="servo-btn servo-stop"
          onClick={async () => {
            try {
              const res = await client.stopServo();
              onActionFeedback(res.accepted ? 'ok' : 'err', `Servo HOLD · ${res.message}`);
            } catch (e: unknown) {
              onActionFeedback('err', `Servo HOLD failed · ${e instanceof Error ? e.message : String(e)}`);
            }
          }}
        >
          HOLD
        </button>
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
