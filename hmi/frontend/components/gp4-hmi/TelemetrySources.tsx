import type { TopicView } from './types';

interface TelemetrySourcesProps {
  topicRows: TopicView[];
}

export function TelemetrySources({ topicRows }: TelemetrySourcesProps) {
  return (
    <section className="section">
      <div className="section-title">Telemetry Sources</div>
      <div className="topic-monitor">
        {topicRows.map((row) => (
          <div key={row.key} className="topic-row">
            <div className="topic-name">{row.name}</div>
            <div className="topic-hz">{row.rateLabel}</div>
            <div className="topic-bar">
              <div className="topic-fill" style={{ width: `${row.fillWidth}%` }}></div>
            </div>
          </div>
        ))}
      </div>
    </section>
  );
}
