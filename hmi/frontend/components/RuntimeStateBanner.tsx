import type { RuntimeSnapshot } from '../../shared/contracts';

const BLOCKING_STATES = new Set(['FAULT', 'ESTOP', 'LOST_CONN', 'SAFETY_BLOCKED']);

interface RuntimeStateBannerProps {
  runtime: RuntimeSnapshot;
}

export function RuntimeStateBanner({ runtime }: RuntimeStateBannerProps) {
  if (!runtime || runtime.systemState === 'NORMAL') {
    return null;
  }

  const blocking = BLOCKING_STATES.has(runtime.systemState);

  return (
    <div className={blocking ? 'runtime-banner runtime-banner-blocking' : 'runtime-banner'}>
      <div className="runtime-banner-title">{runtime.systemState}</div>
      <div className="runtime-banner-text">{runtime.statusText}</div>
      {blocking ? (
        <div className="runtime-banner-caption">Control actions stay backend-blocked until the runtime state clears.</div>
      ) : null}
    </div>
  );
}
