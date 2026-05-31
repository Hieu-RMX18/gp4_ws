import type { PlanMetrics } from '../../../shared/contracts';
import { formatMetric } from './derive';

interface SystemMetricsProps {
  cycleSeconds: number | null;
  commandCount: number;
  planMetrics: PlanMetrics | null;
}

export function SystemMetrics({ cycleSeconds, commandCount, planMetrics }: SystemMetricsProps) {
  return (
    <div className="kpi-grid">
      <div className="kpi">
        <div className="kpi-label">Cycle time</div>
        <div className="kpi-value cyan">{formatMetric(cycleSeconds, 1, ' s')}</div>
        <div className="kpi-sub">last execution</div>
      </div>
      <div className="kpi">
        <div className="kpi-label">Commands</div>
        <div className="kpi-value green">{commandCount}</div>
        <div className="kpi-sub">session total</div>
      </div>
      <div className="kpi">
        <div className="kpi-label">Plan score</div>
        <div className="kpi-value blue">{formatMetric(planMetrics?.score ?? null, 1)}</div>
        <div className="kpi-sub">latest plan</div>
      </div>
      <div className="kpi">
        <div className="kpi-label">Cartesian</div>
        <div className="kpi-value amber">{formatMetric(planMetrics?.cartesianCompletionPct ?? null, 1, '%')}</div>
        <div className="kpi-sub">completion</div>
      </div>
    </div>
  );
}
