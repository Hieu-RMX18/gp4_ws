import { GP4HMI } from './components/GP4HMI';
import { createBridgeClient } from './bridgeClient';
import './styles/gp4-hmi.css';

const bridgeClient = createBridgeClient('/api/hmi');

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

export function App() {
  return <GP4HMI client={bridgeClient} sessionId={getOrCreateSessionId()} operatorId={getOperatorId()} />;
}

export default App;
