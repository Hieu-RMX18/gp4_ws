import type { TraceStepView } from './types';
import { prettyJson } from './derive';

interface CommandPipelinePanelProps {
  traceSteps: TraceStepView[];
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
