import type {
  CommandCancelRequest,
  CommandConfirmRequest,
  CommandIntentRequest,
  CommandMutationResponse,
  ConnectionStateResponse,
  GP4BridgeClient,
  HmiStreamEvent,
  JogCommandRequest,
  LeaseAcquireRequest,
  LeaseMutationResponse,
  LeaseReleaseRequest,
  LeaseRenewRequest,
  LeaseStateResponse,
  ReplayDetail,
  ReplayListQuery,
  ReplayListResponse,
  RuntimeStateResponse,
  ServoControlResponse,
  TransportState,
} from '../shared/contracts';

const SUPPORTED_SCHEMA_VERSION = 'telemetry.v1';
const RECONNECT_BASE_DELAY_MS = 500;
const RECONNECT_MAX_DELAY_MS = 8000;
const SOCKET_STALE_TIMEOUT_MS = 15000;
const MAX_LOG_TEXT_LEN = 480;

function shortenText(value: string): string {
  if (value.length <= MAX_LOG_TEXT_LEN) {
    return value;
  }
  return `${value.slice(0, MAX_LOG_TEXT_LEN)}...(truncated)`;
}

function toPrintable(payload: unknown): unknown {
  if (payload === null || payload === undefined) {
    return payload;
  }
  if (typeof payload === 'string') {
    return shortenText(payload);
  }
  if (typeof payload === 'number' || typeof payload === 'boolean') {
    return payload;
  }
  if (Array.isArray(payload)) {
    return payload.slice(0, 10).map((item) => toPrintable(item));
  }
  if (typeof payload === 'object') {
    const source = payload as Record<string, unknown>;
    const next: Record<string, unknown> = {};
    Object.entries(source).forEach(([key, value]) => {
      next[key] = toPrintable(value);
    });
    return next;
  }
  return String(payload);
}

function traceClient(level: 'info' | 'warn' | 'error', label: string, payload?: unknown): void {
  if (payload === undefined) {
    console[level](`[HMI bridge] ${label}`);
    return;
  }
  console[level](`[HMI bridge] ${label}`, toPrintable(payload));
}

function toWebSocketUrl(baseUrl: string, sessionId: string, operatorId: string): string {
  const httpUrl = new URL(baseUrl, window.location.origin);
  httpUrl.protocol = httpUrl.protocol === 'https:' ? 'wss:' : 'ws:';
  httpUrl.pathname = `${httpUrl.pathname.replace(/\/$/, '')}/stream`;
  httpUrl.searchParams.set('session_id', sessionId);
  httpUrl.searchParams.set('operator_id', operatorId);
  return httpUrl.toString();
}

async function getJson<TResponse>(url: string): Promise<TResponse> {
  traceClient('info', `HTTP GET ${url}`);
  const response = await fetch(url);
  traceClient('info', `HTTP GET response ${url}`, { status: response.status, ok: response.ok });
  if (!response.ok) {
    const text = await response.text();
    traceClient('error', `HTTP GET failed ${url}`, { status: response.status, detail: text });
    throw new Error(text || `Request failed with ${response.status}`);
  }
  const payload = (await response.json()) as TResponse;
  traceClient('info', `HTTP GET payload ${url}`, payload);
  return payload;
}

async function postJson<TResponse>(url: string, body: unknown): Promise<TResponse> {
  traceClient('info', `HTTP POST ${url}`, body);
  const response = await fetch(url, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(body),
  });
  traceClient('info', `HTTP POST response ${url}`, { status: response.status, ok: response.ok });
  if (!response.ok) {
    let detail = `Request failed with ${response.status}`;
    try {
      const payload = (await response.json()) as { detail?: string };
      if (payload.detail) {
        detail = payload.detail;
      }
      traceClient('error', `HTTP POST failed ${url}`, { status: response.status, detail: payload.detail ?? payload });
    } catch {
      const text = await response.text();
      if (text) {
        detail = text;
      }
      traceClient('error', `HTTP POST failed ${url}`, { status: response.status, detail: text });
    }
    throw new Error(detail);
  }
  const payload = (await response.json()) as TResponse;
  traceClient('info', `HTTP POST payload ${url}`, payload);
  return payload;
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
      let failClosedTerminal = false;
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
        failClosedTerminal = true;
        clearStaleSocketTimer();
        onTransportStateChange?.('disconnected');
        if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) {
          socket.close(4002, reason.slice(0, 120));
        }
      };

      const scheduleReconnect = () => {
        if (closedByClient || failClosedTerminal || reconnectTimer !== null) {
          return;
        }
        onTransportStateChange?.('connecting');
        const delayMs = computeReconnectDelayMs(reconnectAttempt);
        reconnectAttempt += 1;
        traceClient('warn', 'WebSocket reconnect scheduled', {
          sessionId,
          operatorId,
          reconnectAttempt,
          delayMs,
        });
        reconnectTimer = window.setTimeout(() => {
          reconnectTimer = null;
          connectSocket();
        }, delayMs);
      };

      const connectSocket = () => {
        clearReconnectTimer();
        clearStaleSocketTimer();
        onTransportStateChange?.('connecting');
        const socketUrl = toWebSocketUrl(basePath, sessionId, operatorId);
        traceClient('info', 'WebSocket connecting', { socketUrl });
        socket = new WebSocket(socketUrl);

        socket.addEventListener('open', () => {
          reconnectAttempt = 0;
          armStaleSocketTimer();
          onTransportStateChange?.('connected');
          traceClient('info', 'WebSocket connected', { sessionId, operatorId });
        });

        socket.addEventListener('message', (messageEvent) => {
          armStaleSocketTimer();
          try {
            const payload = validateStreamEvent(JSON.parse(messageEvent.data) as HmiStreamEvent);
            if (payload.type === 'command_lifecycle') {
              traceClient('info', 'WebSocket event command_lifecycle', {
                commandId: payload.command.commandId,
                lifecycleState: payload.command.lifecycleState,
                finalState: payload.command.finalState,
                messageCount: payload.messages?.length ?? 0,
              });
            } else if (payload.type === 'sequence_lifecycle') {
              traceClient('info', 'WebSocket event sequence_lifecycle', {
                sequenceId: payload.sequence.sequenceId,
                lifecycleState: payload.sequence.lifecycleState,
                finalState: payload.sequence.finalState,
                stepCount: payload.sequence.stepCount,
              });
            } else if (payload.type === 'lease_state') {
              traceClient('info', 'WebSocket event lease_state', {
                ownsControl: payload.lease.ownsControl,
                role: payload.lease.role,
                statusText: payload.lease.statusText,
              });
            } else if (payload.type === 'connection_state') {
              traceClient('warn', 'WebSocket event connection_state', {
                transportState: payload.transportState,
              });
            } else if (payload.type === 'snapshot') {
              traceClient('info', 'WebSocket event snapshot', {
                transportState: payload.snapshot.transportState,
                telemetryState: payload.snapshot.telemetryState,
                runtimeState: payload.snapshot.runtime.systemState,
                mode: payload.snapshot.mode,
              });
            }
            onEvent(payload);
          } catch (error) {
            console.error('Closing HMI bridge socket after invalid stream payload.', error);
            closeSocketFailClosed('Invalid bridge payload');
          }
        });

        socket.addEventListener('close', (event) => {
          clearStaleSocketTimer();
          socket = null;
          onTransportStateChange?.('disconnected');
          traceClient('warn', 'WebSocket closed', {
            code: event.code,
            reason: event.reason,
            wasClean: event.wasClean,
            closedByClient,
            failClosedTerminal,
          });
          scheduleReconnect();
        });

        socket.addEventListener('error', () => {
          clearStaleSocketTimer();
          onTransportStateChange?.('disconnected');
          traceClient('error', 'WebSocket transport error', { sessionId, operatorId });
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

    confirmSequence(sequenceId: string, request: CommandConfirmRequest): Promise<CommandMutationResponse> {
      return postJson(`${basePath}/sequences/${encodeURIComponent(sequenceId)}/confirm`, request);
    },

    abortCommand(commandId: string, request: CommandCancelRequest): Promise<CommandMutationResponse> {
      return postJson(`${basePath}/commands/${encodeURIComponent(commandId)}/cancel`, request);
    },

    abortSequence(sequenceId: string, request: CommandCancelRequest): Promise<CommandMutationResponse> {
      return postJson(`${basePath}/sequences/${encodeURIComponent(sequenceId)}/cancel`, request);
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

    getSequence(sequenceId: string) {
      return getJson(`${basePath}/sequences/${encodeURIComponent(sequenceId)}`);
    },

    async activateJogBridge(): Promise<{ accepted: boolean; message: string }> {
      return postJson(`${basePath}/jog/activate`, {});
    },

    async deactivateJogBridge(): Promise<{ accepted: boolean; message: string }> {
      return postJson(`${basePath}/jog/deactivate`, {});
    },

    sendJogCommand(cmd: JogCommandRequest): Promise<{ accepted: boolean; message: string }> {
      return postJson(`${basePath}/jog/command`, cmd);
    },

    async startServo(): Promise<ServoControlResponse> {
      return postJson(`${basePath}/servo/start`, {});
    },

    async stopServo(): Promise<ServoControlResponse> {
      return postJson(`${basePath}/servo/stop`, {});
    },
  };
}

export function isBlockingTransportState(state: TransportState): boolean {
  return state !== 'connected';
}
