import { useState } from 'react';
import { GP4HMI } from './components/GP4HMI';
import { JogPendant } from './components/JogPendant';
import { createBridgeClient } from './bridgeClient';
import { useGP4Bridge } from './hooks/useGP4Bridge';
import type { JogCommandRequest } from '../shared/contracts';
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

// ── App Root — single WebSocket owner ───────────────────────────────────────

export function App() {
  const [activeTab, setActiveTab] = useState<AppTab>('command');
  const sessionId = getOrCreateSessionId();
  const operatorId = getOperatorId();

  const bridge = useGP4Bridge(bridgeClient, sessionId, operatorId);

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
        <GP4HMI bridge={bridge} />
      ) : (
        <JogPendant
          jogBridgeStatus={bridge.jogBridgeStatus}
          jointPositions={bridge.state.jointPositions}
          onActivateBridge={() => bridgeClient.activateJogBridge()}
          onDeactivateBridge={() => bridgeClient.deactivateJogBridge()}
          onJogCommand={(cmd: JogCommandRequest) => bridgeClient.sendJogCommand(cmd)}
        />
      )}
    </div>
  );
}

export default App;
