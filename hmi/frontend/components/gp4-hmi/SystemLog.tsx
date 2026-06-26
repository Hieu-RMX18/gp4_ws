import { useRef, useState, useMemo } from 'react';
import { formatTimestamp } from './derive';
import type { TaskEvent } from '../../../shared/contracts';

interface SystemLogProps {
  taskEvents: TaskEvent[];
  onReconnect: () => void;
}

export function SystemLog({ taskEvents, onReconnect }: SystemLogProps) {
  const [levelFilter, setLevelFilter] = useState<'ALL' | 'INFO' | 'WARN' | 'ERR' | 'DEBUG'>('ALL');
  const [searchText, setSearchText] = useState('');
  const [expandedEventIdx, setExpandedEventIdx] = useState<number | null>(null);

  const systemRef = useRef<HTMLDivElement>(null);

  const filteredEvents = useMemo(() => {
    return taskEvents.filter((ev) => {
      if (levelFilter !== 'ALL' && ev.level !== levelFilter) {
        return false;
      }
      if (searchText.trim().length > 0) {
        const q = searchText.toLowerCase();
        return (
          ev.detail.toLowerCase().includes(q) ||
          ev.event.toLowerCase().includes(q) ||
          ev.source.toLowerCase().includes(q) ||
          ev.category.toLowerCase().includes(q)
        );
      }
      return true;
    });
  }, [taskEvents, levelFilter, searchText]);

  const handleExportJson = () => {
    const jsonString = `data:text/json;charset=utf-8,${encodeURIComponent(
      JSON.stringify(filteredEvents, null, 2),
    )}`;
    const downloadAnchor = document.createElement('a');
    downloadAnchor.setAttribute('href', jsonString);
    downloadAnchor.setAttribute('download', `hmi_system_events_${new Date().toISOString().slice(0, 19)}.json`);
    document.body.appendChild(downloadAnchor);
    downloadAnchor.click();
    downloadAnchor.remove();
  };

  return (
    <div className="system-log-container">
      <div className="system-events-wrap">
          <div className="log-filters">
            <select
              value={levelFilter}
              onChange={(e) => setLevelFilter(e.target.value as any)}
              className="filter-select"
            >
              <option value="ALL">ALL LEVELS</option>
              <option value="INFO">INFO</option>
              <option value="WARN">WARN</option>
              <option value="ERR">ERROR</option>
              <option value="DEBUG">DEBUG</option>
            </select>
            <input
              type="text"
              placeholder="Search events..."
              value={searchText}
              onChange={(e) => setSearchText(e.target.value)}
              className="filter-input"
            />
            <div className="filter-actions">
              <button
                type="button"
                className="filter-action-btn export-btn"
                title="Export filtered events as JSON"
                onClick={handleExportJson}
              >
                Export
              </button>
              <button
                type="button"
                className="filter-action-btn reconnect-btn"
                title="Force refresh telemetry bridge connection"
                onClick={onReconnect}
              >
                Update
              </button>
            </div>
          </div>

          <div className="system-events-list" ref={systemRef}>
            {filteredEvents.length === 0 ? (
              <div className="events-empty">No system events match the current filter.</div>
            ) : (
              filteredEvents.map((ev, idx) => {
                const isExpanded = expandedEventIdx === idx;
                return (
                  <div
                    key={`event-${idx}`}
                    className={`event-card ${ev.level.toLowerCase()}`}
                    onClick={() => setExpandedEventIdx(isExpanded ? null : idx)}
                  >
                    <div className="event-card-header">
                      <span className="event-time">{formatTimestamp(ev.ts)}</span>
                      <span className={`event-level-tag ${ev.level.toLowerCase()}`}>{ev.level}</span>
                      <span className="event-category">{ev.category} / {ev.source}</span>
                    </div>
                    <div className="event-card-body">
                      <div className="event-name">{ev.event}</div>
                      <div className="event-detail">{ev.detail}</div>
                    </div>
                    {isExpanded && ev.data && Object.keys(ev.data).length > 0 && (
                      <div className="event-card-data" onClick={(e) => e.stopPropagation()}>
                        <pre>{JSON.stringify(ev.data, null, 2)}</pre>
                      </div>
                    )}
                  </div>
                );
              })
            )}
          </div>
        </div>
    </div>
  );
}

