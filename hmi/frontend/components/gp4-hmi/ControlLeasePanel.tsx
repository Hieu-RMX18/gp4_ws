import type { BridgeCapabilities, LeaseView } from '../../../shared/contracts';
import { reasonToVietnamese } from './derive';

interface ControlLeasePanelProps {
  lease: LeaseView;
  capabilities: BridgeCapabilities;
  readOnlyBridge: boolean;
  isController: boolean;
  mode: 'sim' | 'hardware' | 'unknown';
  canAcquireLease: boolean;
  canReleaseLease: boolean;
  onAcquire: () => void;
  onRelease: () => void;
}

export function ControlLeasePanel({
  lease,
  capabilities,
  readOnlyBridge,
  isController,
  mode,
  canAcquireLease,
  canReleaseLease,
  onAcquire,
  onRelease,
}: ControlLeasePanelProps) {
  const hardwareGate = capabilities.hardwareGate;
  const reasons = hardwareGate.reasons ?? [];
  const primaryReason =
    reasons[0] ?? 'Dual gate is not satisfied: runtime flag + signed evidence checklist are required.';
  const primaryReasonVi = reasonToVietnamese(primaryReason);

  return (
    <section className="section">
      <div className="section-title">Control Lease</div>
      <div className="lease-actions">
        {canReleaseLease ? (
          <button className="secondary-btn" onClick={onRelease}>
            Release lease
          </button>
        ) : canAcquireLease ? (
          <button className="secondary-btn" onClick={onAcquire}>
            Request control lease
          </button>
        ) : null}
      </div>
      <div className="lease-caption">
        {readOnlyBridge
          ? mode === 'hardware'
            ? `Hardware gate locked: ${primaryReason}`
            : 'Telemetry is live, but command ingress stays read-only until command-capable mode and freshness gates are satisfied.'
          : lease.statusText}
      </div>
      {readOnlyBridge && mode === 'hardware' ? (
        <div className="lease-caption">VI: {primaryReasonVi}</div>
      ) : null}
      <div className="lease-caption">
        Execution allowed: {capabilities.executionAllowed ? 'yes' : 'no'} · Replay: {capabilities.replayAvailable ? 'yes' : 'no'}
      </div>
    </section>
  );
}
