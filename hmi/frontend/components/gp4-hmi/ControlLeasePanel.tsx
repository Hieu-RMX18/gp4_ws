import type { BridgeCapabilities, LeaseView, RuntimeMode } from '../../../shared/contracts';

interface ControlLeasePanelProps {
  lease: LeaseView;
  capabilities: BridgeCapabilities;
  readOnlyBridge: boolean;
  isController: boolean;
  mode: RuntimeMode;
  canAcquireLease: boolean;
  canReleaseLease: boolean;
  leasePendingAction: 'acquire' | 'release' | null;
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
  leasePendingAction,
  onAcquire,
  onRelease,
}: ControlLeasePanelProps) {
  const stateClass = leasePendingAction ? 'pending' : isController ? 'owned' : canAcquireLease ? 'available' : 'readonly';
  const stateLabel = leasePendingAction === 'acquire'
    ? 'Requesting controller lease'
    : leasePendingAction === 'release'
      ? 'Releasing controller lease'
      : isController
        ? 'Controller lease active'
        : canAcquireLease
          ? 'Lease available'
          : 'Read-only';

  return (
    <section className="section">
      <div className="section-title">Control Lease</div>
      <div className={`lease-state-card ${stateClass}`} aria-live="polite">
        <span className="lease-led" />
        <span>{stateLabel}</span>
      </div>
      <div className="lease-actions">
        {canReleaseLease ? (
          <button className="secondary-btn" disabled={leasePendingAction !== null} onClick={onRelease}>
            {leasePendingAction === 'release' ? 'Releasing...' : 'Release lease'}
          </button>
        ) : canAcquireLease ? (
          <button className="secondary-btn" disabled={leasePendingAction !== null} onClick={onAcquire}>
            {leasePendingAction === 'acquire' ? 'Requesting...' : 'Request control lease'}
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
      <div className="lease-caption">
        Role: {isController ? 'controller' : 'observer'} · Mode: {mode}
      </div>
    </section>
  );
}
