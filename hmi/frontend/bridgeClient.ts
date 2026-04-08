import type {
  CommandActionRequest,
  CommandActionResponse,
  GP4BridgeClient,
  HmiStreamEvent,
  LeaseView,
  LeaseStateResponse,
  LeaseAcquireRequest,
  LeaseMutationResponse,
  LeaseReleaseRequest,
  LeaseRenewRequest,
  RuntimeStateResponse,
  ConnectionStateResponse,
  ReplayDetail,
  ReplayListQuery,
  ReplayListResponse,
  SubmitCommandRequest,
  SubmitCommandResponse,
  TransportState,
} from '../shared/contracts';

const READ_ONLY_REASON =
  'Telemetry bridge v1 is read-only. Lease, command, and replay write paths are disabled.';
const SUPPORTED_SCHEMA_VERSION = 'telemetry.v1';
const RECONNECT_BASE_DELAY_MS = 500;
const RECONNECT_MAX_DELAY_MS = 8000;
const SOCKET_STALE_TIMEOUT_MS = 15000;

function toWebSocketUrl(baseUrl: string, sessionId: string, operatorId: string): string {
  const httpUrl = new URL(baseUrl, window.location.origin);
  httpUrl.protocol = httpUrl.protocol === 'https:' ? 'wss:' : 'ws:';
  httpUrl.pathname = `${httpUrl.pathname.replace(/\/$/, '')}/stream`;
  httpUrl.searchParams.set('session_id', sessionId);
  httpUrl.searchParams.set('operator_id', operatorId);
  return httpUrl.toString();
}

function readOnlyLeaseView(): LeaseView {
  return {
    leaseId: null,
    leaseToken: null,
    role: 'observer',
    ownsControl: false,
    holderOperatorId: null,
    holderSessionId: null,
    acquiredAt: null,
    expiresAt: null,
    statusText: READ_ONLY_REASON,
    canForceTakeover: false,
  };
}

async function getJson<TResponse>(url: string): Promise<TResponse> {
  const response = await fetch(url);
  if (!response.ok) {
    const text = await response.text();
    throw new Error(text || `Request failed with ${response.status}`);
  }
  return (await response.json()) as TResponse;
}

function isCompatibleSchemaVersion(schemaVersion: unknown): boolean {
  return schemaVersion === SUPPORTED_SCHEMA_VERSION;
}

function assertCompatibleSchemaVersion(payload: { schemaVersion?: unknown }, context: string): void {
  if (!isCompatibleSchemaVersion(payload.schemaVersion)) {
    throw new Error(`${context} schema mismatch: expected ${SUPPORTED_SCHEMA_VERSION}.`);
  }
}

function computeReconnectDelayMs(attempt: number): number {
  const unclamped = RECONNECT_BASE_DELAY_MS * 2 ** Math.max(0, attempt);
  const bounded = Math.min(RECONNECT_MAX_DELAY_MS, unclamped);
  const jitterFactor = 0.8 + Math.random() * 0.4;
  return Math.round(Math.min(RECONNECT_MAX_DELAY_MS, bounded * jitterFactor));
}

async function getVersionedJson<TResponse extends { schemaVersion: string }>(url: string, context: string): Promise<TResponse> {
  const payload = await getJson<TResponse>(url);
  assertCompatibleSchemaVersion(payload, context);
  return payload;
}

function validateStreamEvent(event: HmiStreamEvent): HmiStreamEvent {
  if (event.type === 'snapshot') {
    assertCompatibleSchemaVersion(event.snapshot, 'snapshot');
  }
  if (event.type === 'heartbeat') {
    assertCompatibleSchemaVersion(event, 'heartbeat');
  }
  return event;
}

export function createBridgeClient(basePath = '/api/hmi'): GP4BridgeClient {
  return {
    connect({ sessionId, operatorId, onEvent, onTransportStateChange }) {
      let socket: WebSocket | null = null;
      let reconnectTimer: number | null = null;
      let staleSocketTimer: number | null = null;
      let closedByClient = false;
      let reconnectAttempt = 0;

      const clearReconnectTimer = () => {
        if (reconnectTimer !== null) {
          window.clearTimeout(reconnectTimer);
          reconnectTimer = null;
        }
      };

      const clearStaleSocketTimer = () => {
        if (staleSocketTimer !== null) {
          window.clearTimeout(staleSocketTimer);
          staleSocketTimer = null;
        }
      };

      const armStaleSocketTimer = () => {
        clearStaleSocketTimer();
        staleSocketTimer = window.setTimeout(() => {
          if (socket && socket.readyState === WebSocket.OPEN) {
            socket.close(4000, 'Bridge heartbeat timeout');
          }
        }, SOCKET_STALE_TIMEOUT_MS);
      };

      const closeSocketFailClosed = (reason: string) => {
        clearStaleSocketTimer();
        onTransportStateChange?.('disconnected');
        if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) {
          socket.close(4002, reason.slice(0, 120));
        }
      };

      const scheduleReconnect = () => {
        if (closedByClient || reconnectTimer !== null) {
          return;
        }
        onTransportStateChange?.('connecting');
        const delayMs = computeReconnectDelayMs(reconnectAttempt);
        reconnectAttempt += 1;
        reconnectTimer = window.setTimeout(() => {
          reconnectTimer = null;
          connectSocket();
        }, delayMs);
      };

      const connectSocket = () => {
        clearReconnectTimer();
        clearStaleSocketTimer();
        onTransportStateChange?.('connecting');
        socket = new WebSocket(toWebSocketUrl(basePath, sessionId, operatorId));

        socket.addEventListener('open', () => {
          reconnectAttempt = 0;
          armStaleSocketTimer();
          onTransportStateChange?.('connected');
        });

        socket.addEventListener('message', (messageEvent) => {
          armStaleSocketTimer();
          try {
            const payload = validateStreamEvent(JSON.parse(messageEvent.data) as HmiStreamEvent);
            onEvent(payload);
          } catch (error) {
            console.error('Closing HMI bridge socket after invalid stream payload.', error);
            closeSocketFailClosed('Invalid bridge payload');
          }
        });

        socket.addEventListener('close', () => {
          clearStaleSocketTimer();
          socket = null;
          onTransportStateChange?.('disconnected');
          scheduleReconnect();
        });

        socket.addEventListener('error', () => {
          clearStaleSocketTimer();
          onTransportStateChange?.('disconnected');
        });
      };

      connectSocket();

      return () => {
        closedByClient = true;
        clearReconnectTimer();
        clearStaleSocketTimer();
        if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) {
          socket.close();
        }
      };
    },

    acquireLease(_: LeaseAcquireRequest): Promise<LeaseMutationResponse> {
      return Promise.resolve({
        accepted: false,
        lease: readOnlyLeaseView(),
        reason: READ_ONLY_REASON,
      });
    },

    renewLease(_: LeaseRenewRequest): Promise<LeaseMutationResponse> {
      return Promise.resolve({
        accepted: false,
        lease: readOnlyLeaseView(),
        reason: READ_ONLY_REASON,
      });
    },

    releaseLease(_: LeaseReleaseRequest): Promise<LeaseMutationResponse> {
      return Promise.resolve({
        accepted: false,
        lease: readOnlyLeaseView(),
        reason: READ_ONLY_REASON,
      });
    },

    submitCommand(_: SubmitCommandRequest): Promise<SubmitCommandResponse> {
      return Promise.resolve({
        accepted: false,
        commandId: null,
        reason: READ_ONLY_REASON,
      });
    },

    confirmCommand(commandId: string, _: CommandActionRequest): Promise<CommandActionResponse> {
      return Promise.resolve({
        accepted: false,
        commandId,
        reason: READ_ONLY_REASON,
      });
    },

    abortCommand(commandId: string, _: CommandActionRequest): Promise<CommandActionResponse> {
      return Promise.resolve({
        accepted: false,
        commandId,
        reason: READ_ONLY_REASON,
      });
    },

    getRuntimeState(sessionId: string, operatorId: string): Promise<RuntimeStateResponse> {
      const url = new URL(`${basePath}/runtime-state`, window.location.origin);
      url.searchParams.set('session_id', sessionId);
      url.searchParams.set('operator_id', operatorId);
      return getVersionedJson(url.toString(), 'runtime-state');
    },

    getConnectionState(): Promise<ConnectionStateResponse> {
      return getVersionedJson(new URL(`${basePath}/connection-state`, window.location.origin).toString(), 'connection-state');
    },

    getLeaseState(sessionId: string, operatorId: string): Promise<LeaseStateResponse> {
      const url = new URL(`${basePath}/lease-state`, window.location.origin);
      url.searchParams.set('session_id', sessionId);
      url.searchParams.set('operator_id', operatorId);
      return getVersionedJson(url.toString(), 'lease-state');
    },

    listReplay(_: ReplayListQuery | undefined): Promise<ReplayListResponse> {
      return Promise.resolve({ items: [] });
    },

    getReplayDetail(_: string): Promise<ReplayDetail> {
      return Promise.reject(new Error('Replay detail is not exposed by telemetry bridge v1.'));
    },
  };
}

export function isBlockingTransportState(state: TransportState): boolean {
  return state !== 'connected';
}
