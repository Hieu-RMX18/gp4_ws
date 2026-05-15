import type { PipelineTrace } from '../../../shared/contracts';
import type { TraceStepView } from './types';
import { prettyJson } from './derive';

interface CommandPipelinePanelProps {
  traceSteps: TraceStepView[];
}

function TraceMicroRow({ trace }: { trace: PipelineTrace }) {
  const levelClass = trace.level === 'error' || trace.level === 'warn' ? 'trace-micro-warn' : 'trace-micro-info';
  return (
    <details className={`trace-micro ${levelClass}`}>
      <summary>
        <span className="trace-micro-layer">{trace.layer}</span>
        <span className="trace-micro-event">{trace.event}</span>
        <span className="trace-micro-summary">{trace.summary}</span>
      </summary>
      {trace.details != null ? (
        <pre className="trace-micro-detail">{prettyJson(trace.details as Record<string, unknown>)}</pre>
      ) : null}
    </details>
  );
}

export function CommandPipelinePanel({ traceSteps }: CommandPipelinePanelProps) {
  if (traceSteps.length === 0) {
    return null;
  }

  return (
    <section className="section command-review-section">
      <div className="section-title">Command Pipeline</div>
      <div className="trace-list">
        {traceSteps.map((step) => (
          <div key={step.key} className={`trace-step ${step.status}`}>
            <div className="trace-head">
              <span className="trace-label">{step.label}</span>
              <span className={`trace-state ${step.status}`}>{step.status.toUpperCase()}</span>
            </div>
            <div className="trace-summary">{step.summary}</div>
            {step.traces.length > 0 && (
              <div className="trace-micro-list">
                {step.traces.map((t, i) => (
                  <TraceMicroRow key={i} trace={t} />
                ))}
              </div>
            )}
            {step.details ? (
              <details className="trace-json">
                <summary>View JSON</summary>
                <pre>{prettyJson(step.details)}</pre>
              </details>
            ) : null}
          </div>
        ))}
      </div>
    </section>
  );
}
