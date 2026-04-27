import type { LogEntryView } from './types';

interface SystemLogProps {
  logEntries: LogEntryView[];
}

export function SystemLog({ logEntries }: SystemLogProps) {
  return (
    <>
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
    </>
  );
}
