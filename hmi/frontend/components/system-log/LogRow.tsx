import React, { useState } from 'react';
import type { TaskEvent } from '../../../shared/contracts';

interface LogRowProps {
  event: TaskEvent;
}

export function LogRow({ event }: LogRowProps) {
  const [expanded, setExpanded] = useState(false);

  return (
    <>
      <tr className="log-row" onClick={() => setExpanded(!expanded)}>
        <td>{event.ts}</td>
        <td className={`log-level-${event.level}`}>{event.level}</td>
        <td>{event.source}</td>
        <td>{event.category}</td>
        <td>{event.event}</td>
        <td>{event.detail}</td>
      </tr>
      {expanded && (
        <tr className="log-detail-expanded">
          <td colSpan={6}>
            <pre className="log-json-block">
              {JSON.stringify(event.data, null, 2)}
            </pre>
          </td>
        </tr>
      )}
    </>
  );
}
