import type { JointPosition } from '../../../shared/contracts';
import { formatAngle, isNearLimit, toPercent } from './derive';

interface JointMonitorProps {
  orderedJoints: JointPosition[];
}

export function JointMonitor({ orderedJoints }: JointMonitorProps) {
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

    </>
  );
}
