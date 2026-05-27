import { useEffect, useRef } from 'react';

import type { LogEntryView } from './types';

interface SystemLogProps {
  logEntries: LogEntryView[];
}

export function SystemLog({ logEntries }: SystemLogProps) {
  const areaRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (areaRef.current) {
      areaRef.current.scrollTop = areaRef.current.scrollHeight;
    }
  }, [logEntries.length]);

  return (
    <>
      <div className="panel-header panel-header-secondary">System Log</div>
      <div className="log-area" ref={areaRef}>
        {logEntries.map((entry) => (
          <div key={entry.id} className="log-entry">
            <span className="log-time">{entry.time}</span>
            <span className={`log-level ${entry.level}`}>{entry.level.toUpperCase()}</span>
            {entry.source && <span className="log-source">{entry.source}</span>}
            <span className="log-msg">{entry.message}</span>
          </div>
        ))}
      </div>
    </>
  );
}
