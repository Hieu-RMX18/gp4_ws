import type { CommandView, JointPosition, RuntimeSnapshot } from '../../../shared/contracts';
import { formatAngle, isNearLimit, toPercent } from './derive';

interface JointMonitorProps {
  orderedJoints: JointPosition[];
  runtime: RuntimeSnapshot;
  activeCommand: CommandView | null;
}

export function JointMonitor({ orderedJoints, runtime, activeCommand }: JointMonitorProps) {
  return (
    <>
      <section className="section">
        <div className="section-title">Joint Positions (deg)</div>
        {orderedJoints.map((joint) => {
          const unavailable = joint.positionDeg === null;
          const nearLimit = isNearLimit(joint);
          return (
            <div key={joint.name} className="joint-row">
              <div className="joint-header">
                <span className="joint-name">{joint.name}</span>
                <span className="joint-val">
                  {formatAngle(joint.positionDeg)}
                  {unavailable ? <span className="joint-unavailable"> unavailable</span> : null}
                  {nearLimit ? <span className="joint-near-limit"> near limit</span> : null}
                </span>
              </div>
              <div className="joint-bar">
                <div className="joint-fill" style={{ width: `${toPercent(joint)}%` }}></div>
              </div>
            </div>
          );
        })}
      </section>

      <section className="section">
        <div className="section-title">Runtime Snapshot</div>
        <div className="pose-grid">
          <div className="pose-cell">
            <div className="pose-label">Mode</div>
            <div className="pose-value">{runtime.mode}</div>
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
            <div className="pose-value">{runtime.robotStatus.servoState}</div>
          </div>
          <div className="pose-cell">
            <div className="pose-label">Alarm</div>
            <div className="pose-value">{runtime.robotStatus.alarmState}</div>
          </div>
        </div>
      </section>
    </>
  );
}
