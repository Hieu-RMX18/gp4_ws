import type { ConsoleEvent } from '../../../shared/contracts';

interface RuntimeConsoleProps {
  events: ConsoleEvent[];
}

export function RuntimeConsole({ events }: RuntimeConsoleProps) {
  const display = events.length === 0
    ? [{ stage: 'waiting', timestamp: new Date().toISOString(), fields: 'No pipeline events yet.' }]
    : events.slice(-20);
  return (
    <>
      <div className="panel-header panel-header-secondary">Runtime Console</div>
      <div className="log-area console-area">
        {display.map((evt, idx) => (
          <div key={`${evt.timestamp}-${idx}`} className="log-entry console-entry">
            <span className="log-time">{evt.timestamp.slice(11, 19)}</span>
            <span className="log-level console-stage">{evt.stage}</span>
            <span className="log-msg">{evt.fields ?? ''}</span>
          </div>
        ))}
      </div>
    </>
  );
}
