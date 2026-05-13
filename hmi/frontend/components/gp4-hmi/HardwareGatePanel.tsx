import type { HardwareGateStatus } from '../../../shared/contracts';
import { hardwareGateLabel, prettyJson, reasonToVietnamese } from './derive';

interface HardwareGatePanelProps {
  hardwareGate: HardwareGateStatus;
}

export function HardwareGatePanel({ hardwareGate }: HardwareGatePanelProps) {
  const reasons = hardwareGate.reasons ?? [];
  const primaryReason =
    reasons[0] ?? 'Hardware evidence is advisory; runtime preflight controls execution.';
  const primaryReasonVi = reasonToVietnamese(primaryReason);

  return (
    <section className={`hardware-gate-panel ${hardwareGate.unlocked ? 'unlocked' : 'locked'}`}>
      <div className="hardware-gate-header">
        <div className="hardware-gate-title">Hardware Evidence / Minh chứng phần cứng</div>
        <span className={`hardware-gate-pill ${hardwareGate.unlocked ? 'unlocked' : 'locked'}`}>
          {hardwareGateLabel(hardwareGate.unlocked)}
        </span>
      </div>
      <div className="hardware-gate-line">
        Flag / Cờ bật phần cứng: <strong>{hardwareGate.flagEnabled ? 'ON' : 'OFF'}</strong>
      </div>
      <div className="hardware-gate-line">
        Evidence / Minh chứng: <code>{hardwareGate.evidencePath}</code>
      </div>
      {!hardwareGate.unlocked ? (
        <>
          <div className="hardware-gate-reason">
            EN: {primaryReason}
          </div>
          <div className="hardware-gate-reason vi">
            VI: {primaryReasonVi}
          </div>
        </>
      ) : (
        <div className="hardware-gate-reason ok">
          EN+VI: Hardware evidence recorded. Runtime preflight still controls execution.
        </div>
      )}
      {hardwareGate.checklist ? (
        <details className="hardware-gate-json">
          <summary>Checklist JSON / Chi tiết checklist</summary>
          <pre>{prettyJson(hardwareGate.checklist)}</pre>
        </details>
      ) : null}
    </section>
  );
}
