import React, { useMemo, useState } from 'react';
import type { TaskEvent } from '../../../shared/contracts';
import { LogRow } from './LogRow';
import { RobotStatusStrip } from './RobotStatusStrip';
import './system-log.css';

interface SystemLogProps {
  events: TaskEvent[];
}

export function SystemLog({ events }: SystemLogProps) {
  const [filterLevel, setFilterLevel] = useState<string>('ALL');
  const [searchQuery, setSearchQuery] = useState('');

  const filteredEvents = useMemo(() => {
    return events.filter((ev) => {
      if (filterLevel !== 'ALL' && ev.level !== filterLevel) return false;
      if (searchQuery) {
        const q = searchQuery.toLowerCase();
        return (
          ev.source.toLowerCase().includes(q) ||
          ev.category.toLowerCase().includes(q) ||
          ev.event.toLowerCase().includes(q) ||
          ev.detail.toLowerCase().includes(q)
        );
      }
      return true;
    });
  }, [events, filterLevel, searchQuery]);

  const latestHardware = useMemo(() => {
    return events.find((ev) => ev.category === 'HARDWARE');
  }, [events]);

  const handleExport = () => {
    const dataStr = 'data:text/json;charset=utf-8,' + encodeURIComponent(JSON.stringify(events, null, 2));
    const downloadAnchorNode = document.createElement('a');
    downloadAnchorNode.setAttribute('href', dataStr);
    downloadAnchorNode.setAttribute('download', 'gp4_system_log.json');
    document.body.appendChild(downloadAnchorNode);
    downloadAnchorNode.click();
    downloadAnchorNode.remove();
  };

  return (
    <div className="system-log-container">
      <div className="system-log-header">
        <div className="system-log-filters">
          <select value={filterLevel} onChange={(e) => setFilterLevel(e.target.value)}>
            <option value="ALL">All Levels</option>
            <option value="INFO">INFO</option>
            <option value="WARN">WARN</option>
            <option value="ERR">ERR</option>
            <option value="DEBUG">DEBUG</option>
          </select>
          <input
            type="text"
            placeholder="Search logs..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
          />
        </div>
        <button onClick={handleExport}>Export JSON</button>
      </div>

      <div className="system-log-table-container">
        <table className="system-log-table">
          <thead>
            <tr>
              <th>Timestamp</th>
              <th>Level</th>
              <th>Source</th>
              <th>Category</th>
              <th>Event</th>
              <th>Detail</th>
            </tr>
          </thead>
          <tbody>
            {filteredEvents.map((ev, i) => (
              <LogRow key={i} event={ev} />
            ))}
            {filteredEvents.length === 0 && (
              <tr>
                <td colSpan={6} style={{ textAlign: 'center', padding: '1rem' }}>
                  No events found.
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>

      <RobotStatusStrip latestHardwareEvent={latestHardware} />
    </div>
  );
}
