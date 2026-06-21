import React from 'react';
import type { TaskEvent } from '../../../shared/contracts';

interface RobotStatusStripProps {
  latestHardwareEvent: TaskEvent | undefined;
}

export function RobotStatusStrip({ latestHardwareEvent }: RobotStatusStripProps) {
  if (!latestHardwareEvent) {
    return (
      <div className="robot-status-strip">
        <span className="status-badge warning">NO HARDWARE DATA</span>
      </div>
    );
  }

  const { data } = latestHardwareEvent;
  const isEStop = Boolean(data?.e_stopped);
  const inError = Boolean(data?.in_error);
  const isServoOn = Boolean(data?.servo_on);
  const mode = Number(data?.mode);

  return (
    <div className="robot-status-strip">
      <span className={`status-badge ${isEStop ? 'error' : 'ok'}`}>
        E-STOP: {isEStop ? 'ACTIVE' : 'CLEAR'}
      </span>
      <span className={`status-badge ${inError ? 'error' : 'ok'}`}>
        ALARM: {inError ? `ACTIVE (${data?.error_code || '?'})` : 'NONE'}
      </span>
      <span className={`status-badge ${isServoOn ? 'ok' : 'warning'}`}>
        SERVO: {isServoOn ? 'ON' : 'OFF'}
      </span>
      <span className="status-badge">
        MODE: {mode}
      </span>
    </div>
  );
}
