import { useEffect, useState } from 'react';
import { GP4HMI } from './components/GP4HMI';
import { JogPendant } from './components/JogPendant';
import { createBridgeClient } from './bridgeClient';
import type {
  GP4BridgeClient,
  JogBridgeStatusSnapshot,
  JogCommandRequest,
  JointPosition,
} from '../shared/contracts';
import './styles/gp4-hmi.css';

const bridgeClient = createBridgeClient('/api/hmi');

type AppTab = 'command' | 'jog';

function getOrCreateSessionId(): string {
  const key = 'gp4-hmi-session-id';
  const existing = window.localStorage.getItem(key);
  if (existing) {
    return existing;
  }
  const generated = crypto.randomUUID();
  window.localStorage.setItem(key, generated);
  return generated;
}

function getOperatorId(): string {
  return window.localStorage.getItem('gp4-hmi-operator-id') || 'operator';
}

// ── Jog Pendant Wrapper ─────────────────────────────────────────────────────

const DEFAULT_JOG_STATUS: JogBridgeStatusSnapshot = {
  state: 'IDLE',
  pointsQueued: 0,
  effectiveHz: 0,
  robotReady: false,
  servoActive: false,
  bridgeActive: false,
  lastError: '',
  rejectionReason: '',
};

const DEFAULT_JOINTS: JointPosition[] = [
  { name: 'joint_1_s', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_2_l', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_3_u', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_4_r', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_5_b', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_6_t', positionDeg: null, minDeg: -180, maxDeg: 180 },
];

interface JogPendantWrapperProps {
  client: GP4BridgeClient;
  sessionId: string;
  operatorId: string;
}

function JogPendantWrapper({ client, sessionId, operatorId }: JogPendantWrapperProps) {
  const [jogStatus, setJogStatus] = useState<JogBridgeStatusSnapshot>(DEFAULT_JOG_STATUS);
  const [jointPositions, setJointPositions] = useState<JointPosition[]>(DEFAULT_JOINTS);

  useEffect(() => {
    const disconnect = client.connect({
      sessionId,
      operatorId,
      onEvent: (event) => {
        if (event.type === 'snapshot' && event.snapshot.jointPositions?.length) {
          setJointPositions(event.snapshot.jointPositions);
        }
        if (event.type === 'jog_bridge_status') {
          setJogStatus(event.jogBridgeStatus);
        }
      },
      onTransportStateChange: () => {},
    });
    return disconnect;
  }, [client, sessionId, operatorId]);

  return (
    <JogPendant
      jogBridgeStatus={jogStatus}
      jointPositions={jointPositions}
      onActivateBridge={() => client.activateJogBridge()}
      onDeactivateBridge={() => client.deactivateJogBridge()}
      onJogCommand={(cmd: JogCommandRequest) => client.sendJogCommand(cmd)}
    />
  );
}

// ── App Root ────────────────────────────────────────────────────────────────

export function App() {
  const [activeTab, setActiveTab] = useState<AppTab>('command');
  const sessionId = getOrCreateSessionId();
  const operatorId = getOperatorId();

  return (
    <div className="app-root">
      <nav className="app-tabs">
        <button
          type="button"
          className={`app-tab ${activeTab === 'command' ? 'active' : ''}`}
          onClick={() => setActiveTab('command')}
        >
          Command Interface
        </button>
        <button
          type="button"
          className={`app-tab ${activeTab === 'jog' ? 'active' : ''}`}
          onClick={() => setActiveTab('jog')}
        >
          Joint Jog Pendant
        </button>
      </nav>
      {activeTab === 'command' ? (
        <GP4HMI client={bridgeClient} sessionId={sessionId} operatorId={operatorId} />
      ) : (
        <JogPendantWrapper client={bridgeClient} sessionId={sessionId} operatorId={operatorId} />
      )}
    </div>
  );
}

export default App;
