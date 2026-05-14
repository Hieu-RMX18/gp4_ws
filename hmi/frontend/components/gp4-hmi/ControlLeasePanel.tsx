import type { BridgeCapabilities, LeaseView } from '../../../shared/contracts';

interface ControlLeasePanelProps {
  lease: LeaseView;
  capabilities: BridgeCapabilities;
  readOnlyBridge: boolean;
  canAcquireLease: boolean;
  canReleaseLease: boolean;
  onAcquire: () => void;
  onRelease: () => void;
}

export function ControlLeasePanel({
  lease,
  capabilities,
  readOnlyBridge,
  canAcquireLease,
  canReleaseLease,
  onAcquire,
  onRelease,
}: ControlLeasePanelProps) {
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
          ? 'Telemetry is live, but command ingress stays read-only until command-capable mode and freshness gates are satisfied.'
          : lease.statusText}
      </div>
      <div className="lease-caption">
        Execution allowed: {capabilities.executionAllowed ? 'yes' : 'no'} · Replay: {capabilities.replayAvailable ? 'yes' : 'no'}
      </div>
    </section>
  );
}
