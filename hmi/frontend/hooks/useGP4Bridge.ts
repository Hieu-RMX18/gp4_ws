import { useCallback, useEffect, useMemo, useState } from 'react';

import type {
  CommandMutationResponse,
  GP4BridgeClient,
  HmiStateSnapshot,
  HmiStreamEvent,
  JointPosition,
  LeaseMutationResponse,
  ReplayListItem,
  TransportState,
} from '../../shared/contracts';

const DEFAULT_JOINTS: JointPosition[] = [
  { name: 'joint_1_s', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_2_l', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_3_u', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_4_r', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_5_b', positionDeg: null, minDeg: -180, maxDeg: 180 },
  { name: 'joint_6_t', positionDeg: null, minDeg: -180, maxDeg: 180 },
];

function createDisconnectedSnapshot(): HmiStateSnapshot {
  return {
    schemaVersion: 'telemetry.v1',
    generatedAt: new Date().toISOString(),
    transportState: 'disconnected',
    telemetryState: 'unavailable',
    telemetrySources: [],
    mode: 'unknown',
    connections: [
      { name: 'ros2', label: 'ROS 2', health: 'down' },
      { name: 'moveit2', label: 'MoveIt 2', health: 'down' },
      { name: 'llm', label: 'LLM', health: 'down' },
      { name: 'motoros2', label: 'MotoROS2', health: 'down' },
    ],
    capabilities: {
      readOnly: true,
      canAcquireLease: false,
      canSubmitCommands: false,
      canConfirmCommands: false,
      canCancelCommands: false,
      canAbortCommands: false,
      commandIngressAvailable: false,
      confirmationAvailable: false,
      executionAllowed: false,
      replayAvailable: false,
      simOnly: true,
      hardwareGate: {
        unlocked: false,
        reasons: ['hardware gate status unavailable while bridge is disconnected'],
        flagEnabled: false,
        evidencePath: 'hmi/data/hardware_gate.json',
        approvedBy: null,
        approvedAt: null,
        reportPath: null,
        reportSha256: null,
        reportSha256Match: false,
        checklist: null,
      },
    },
    lease: {
      leaseId: null,
      leaseToken: null,
      role: 'observer',
      ownsControl: false,
      holderOperatorId: null,
      holderSessionId: null,
      acquiredAt: null,
      expiresAt: null,
      statusText: 'Observer mode — waiting for telemetry bridge lease state',
      canForceTakeover: false,
    },
    runtime: {
      systemState: 'LOST_CONN',
      blocking: true,
      statusText: 'Telemetry bridge disconnected.',
      mode: 'unknown',
      robotStatus: {
        servoState: 'UNKNOWN',
        eStop: 'UNKNOWN',
        alarmState: 'UNKNOWN',
        motionMode: null,
        trajectoryPointsUsed: null,
        trajectoryPointsCapacity: null,
        readinessMessage: 'No backend runtime snapshot available.',
      },
    },
    messages: [],
    activeCommand: null,
    jointPositions: DEFAULT_JOINTS,
    planMetrics: null,
    replayItems: [],
  };
}

function applyEvent(snapshot: HmiStateSnapshot, event: HmiStreamEvent): HmiStateSnapshot {
  switch (event.type) {
    case 'snapshot':
      return event.snapshot;
    case 'lease_state':
      return {
        ...snapshot,
        generatedAt: new Date().toISOString(),
        lease: event.lease,
        capabilities: event.capabilities,
      };
    case 'heartbeat':
      return {
        ...snapshot,
        generatedAt: event.generatedAt,
        transportState: event.transportState,
        telemetryState: event.telemetryState,
      };
    case 'command_lifecycle':
      return {
        ...snapshot,
        generatedAt: new Date().toISOString(),
        activeCommand: event.command,
        planMetrics: event.planMetrics ?? event.command.metrics ?? snapshot.planMetrics,
        messages: event.messages ? [...snapshot.messages, ...event.messages] : snapshot.messages,
      };
    case 'replay_updated':
      return {
        ...snapshot,
        generatedAt: new Date().toISOString(),
        replayItems: event.replayItems,
      };
    case 'connection_state':
      return {
        ...snapshot,
        generatedAt: new Date().toISOString(),
        transportState: event.transportState,
        connections: event.connections ?? snapshot.connections,
        telemetryState: event.transportState === 'connected' ? snapshot.telemetryState : 'unavailable',
        runtime:
          event.transportState === 'connected'
            ? snapshot.runtime
            : {
                ...snapshot.runtime,
                systemState: 'LOST_CONN',
                blocking: true,
                statusText: 'Telemetry bridge disconnected.',
              },
      };
    default:
      return snapshot;
  }
}

export interface UseGp4BridgeResult {
  state: HmiStateSnapshot;
  transportState: TransportState;
  isController: boolean;
  blockingRuntime: boolean;
  submitCommand: (rawText: string) => Promise<CommandMutationResponse>;
  confirmCommandById: (commandId: string, planFingerprint: string) => Promise<CommandMutationResponse>;
  acquireControllerLease: () => Promise<LeaseMutationResponse>;
  releaseLease: () => Promise<LeaseMutationResponse | null>;
  confirmActiveCommand: () => Promise<CommandMutationResponse | null>;
  abortActiveCommand: (reason?: string) => Promise<CommandMutationResponse | null>;
  refreshReplay: () => Promise<ReplayListItem[]>;
}

export function useGP4Bridge(
  client: GP4BridgeClient,
  sessionId: string,
  operatorId: string,
): UseGp4BridgeResult {
  const [state, setState] = useState<HmiStateSnapshot>(createDisconnectedSnapshot);
  const [transportState, setTransportState] = useState<TransportState>('disconnected');

  useEffect(() => {
    const disconnect = client.connect({
      sessionId,
      operatorId,
      onEvent: (event) => {
        setState((current) => applyEvent(current, event));
      },
      onTransportStateChange: (nextState) => {
        setTransportState(nextState);
        setState((current) => applyEvent(current, { type: 'connection_state', transportState: nextState }));
      },
    });

    return disconnect;
  }, [client, operatorId, sessionId]);

  useEffect(() => {
    if (state.capabilities.readOnly || !state.lease.ownsControl || !state.lease.leaseToken) {
      return undefined;
    }

    const interval = window.setInterval(() => {
      void client
        .renewLease({
          sessionId,
          operatorId,
          leaseToken: state.lease.leaseToken as string,
        })
        .then((response) => {
          setState((current) => ({
            ...current,
            lease: response.lease,
          }));
        })
        .catch(() => {
          setTransportState('disconnected');
        });
    }, 5000);

    return () => window.clearInterval(interval);
  }, [client, operatorId, sessionId, state.capabilities.readOnly, state.lease.leaseToken, state.lease.ownsControl]);

  const acquireControllerLease = useCallback(() => {
    return client.acquireLease({
      sessionId,
      operatorId,
      requestedRole: 'controller',
    }).then((response) => {
      setState((current) => ({ ...current, lease: response.lease }));
      return response;
    });
  }, [client, operatorId, sessionId]);

  const releaseLease = useCallback(async () => {
    if (!state.lease.leaseToken) {
      return null;
    }
    const response = await client.releaseLease({
      sessionId,
      operatorId,
      leaseToken: state.lease.leaseToken,
    });
    setState((current) => ({ ...current, lease: response.lease }));
    return response;
  }, [client, operatorId, sessionId, state.lease.leaseToken]);

  const submitCommand = useCallback(async (rawText: string) => {
    const response = await client.submitCommand({
      sessionId,
      operatorId,
      leaseToken: state.lease.leaseToken,
      intentText: rawText,
      mode: state.mode,
    });
    if (response.snapshot) {
      setState(response.snapshot);
    }
    return response;
  }, [client, operatorId, sessionId, state.lease.leaseToken, state.mode]);

  const confirmCommandById = useCallback(async (commandId: string, planFingerprint: string) => {
    const response = await client.confirmCommand(commandId, {
      sessionId,
      operatorId,
      leaseToken: state.lease.leaseToken,
      planFingerprint,
    });
    if (response.snapshot) {
      setState(response.snapshot);
    }
    return response;
  }, [client, operatorId, sessionId, state.lease.leaseToken]);

  const confirmActiveCommand = useCallback(async () => {
    if (!state.activeCommand || !state.activeCommand.planFingerprint) {
      return null;
    }
    return confirmCommandById(state.activeCommand.commandId, state.activeCommand.planFingerprint);
  }, [confirmCommandById, state.activeCommand]);

  const abortActiveCommand = useCallback(async (reason?: string) => {
    if (!state.activeCommand) {
      return null;
    }
    const response = await client.abortCommand(state.activeCommand.commandId, {
      sessionId,
      operatorId,
      leaseToken: state.lease.leaseToken,
      reason,
    });
    if (response.snapshot) {
      setState(response.snapshot);
    }
    return response;
  }, [client, operatorId, sessionId, state.activeCommand, state.lease.leaseToken]);

  const refreshReplay = useCallback(async () => {
    const response = await client.listReplay({ limit: 25 });
    setState((current) => ({ ...current, replayItems: response.items }));
    return response.items;
  }, [client]);

  const blockingRuntime = useMemo(() => {
    return state.runtime.blocking || transportState !== 'connected';
  }, [state.runtime.blocking, transportState]);

  return {
    state,
    transportState,
    isController: state.lease.ownsControl && !state.capabilities.readOnly,
    blockingRuntime,
    submitCommand,
    confirmCommandById,
    acquireControllerLease,
    releaseLease,
    confirmActiveCommand,
    abortActiveCommand,
    refreshReplay,
  };
}
