import type {
  CommandCancelRequest,
  CommandConfirmRequest,
  CommandIntentRequest,
  CommandMutationResponse,
  ConnectionStateResponse,
  GP4BridgeClient,
  HmiStreamEvent,
  LeaseAcquireRequest,
  LeaseMutationResponse,
  LeaseReleaseRequest,
  LeaseRenewRequest,
  LeaseStateResponse,
  ReplayDetail,
  ReplayListQuery,
  ReplayListResponse,
  RuntimeStateResponse,
  TransportState,
} from '../shared/contracts';

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

async function getJson<TResponse>(url: string): Promise<TResponse> {
  const response = await fetch(url);
  if (!response.ok) {
    const text = await response.text();
    throw new Error(text || `Request failed with ${response.status}`);
  }
  return (await response.json()) as TResponse;
}

async function postJson<TResponse>(url: string, body: unknown): Promise<TResponse> {
  const response = await fetch(url, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(body),
  });
  if (!response.ok) {
    let detail = `Request failed with ${response.status}`;
    try {
      const payload = (await response.json()) as { detail?: string };
      if (payload.detail) {
        detail = payload.detail;
      }
    } catch {
      const text = await response.text();
      if (text) {
        detail = text;
      }
    }
    throw new Error(detail);
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

async function getVersionedJson<TResponse extends { schemaVersion: string }>(
  url: string,
  context: string,
): Promise<TResponse> {
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

function withQuery(
  basePath: string,
  route: string,
  query: Record<string, string | undefined | null>,
): string {
  const url = new URL(`${basePath}${route}`, window.location.origin);
  Object.entries(query).forEach(([key, value]) => {
    if (value !== undefined && value !== null && value !== '') {
      url.searchParams.set(key, value);
    }
  });
  return url.toString();
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

    acquireLease(request: LeaseAcquireRequest): Promise<LeaseMutationResponse> {
      return postJson(`${basePath}/lease/acquire`, request);
    },

    renewLease(request: LeaseRenewRequest): Promise<LeaseMutationResponse> {
      return postJson(`${basePath}/lease/renew`, request);
    },

    releaseLease(request: LeaseReleaseRequest): Promise<LeaseMutationResponse> {
      return postJson(`${basePath}/lease/release`, request);
    },

    submitCommand(request: CommandIntentRequest): Promise<CommandMutationResponse> {
      return postJson(`${basePath}/commands/intent`, request);
    },

    confirmCommand(commandId: string, request: CommandConfirmRequest): Promise<CommandMutationResponse> {
      return postJson(`${basePath}/commands/${encodeURIComponent(commandId)}/confirm`, request);
    },

    abortCommand(commandId: string, request: CommandCancelRequest): Promise<CommandMutationResponse> {
      return postJson(`${basePath}/commands/${encodeURIComponent(commandId)}/cancel`, request);
    },

    getRuntimeState(sessionId: string, operatorId: string): Promise<RuntimeStateResponse> {
      return getVersionedJson(
        withQuery(basePath, '/runtime-state', { session_id: sessionId, operator_id: operatorId }),
        'runtime-state',
      );
    },

    getConnectionState(): Promise<ConnectionStateResponse> {
      return getVersionedJson(withQuery(basePath, '/connection-state', {}), 'connection-state');
    },

    getLeaseState(sessionId: string, operatorId: string): Promise<LeaseStateResponse> {
      return getVersionedJson(
        withQuery(basePath, '/lease-state', { session_id: sessionId, operator_id: operatorId }),
        'lease-state',
      );
    },

    listReplay(query?: ReplayListQuery): Promise<ReplayListResponse> {
      return getJson(
        withQuery(basePath, '/replay', {
          session_id: query?.sessionId,
          operator_id: query?.operatorId,
          final_state: query?.finalState,
          from: query?.from,
          to: query?.to,
          limit: query?.limit?.toString(),
        }),
      );
    },

    getReplayDetail(commandId: string): Promise<ReplayDetail> {
      return getJson(`${basePath}/replay/${encodeURIComponent(commandId)}`);
    },
  };
}

export function isBlockingTransportState(state: TransportState): boolean {
  return state !== 'connected';
}
