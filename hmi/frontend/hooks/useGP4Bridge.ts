import { useCallback, useEffect, useMemo, useState } from 'react';

import type {
  CommandView,
  CommandMutationResponse,
  GP4BridgeClient,
  HmiStateSnapshot,
  HmiStreamEvent,
  JogBridgeStatusSnapshot,
  JointPosition,
  LeaseMutationResponse,
  ReplayListItem,
  SequenceView,
  ServoControlResponse,
  TaskEvent,
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
    activeSequence: null,
    jointPositions: DEFAULT_JOINTS,
    planMetrics: null,
    replayItems: [],
    toolPose: null,
  };
}

function mergeSequenceStep(sequence: SequenceView, command: CommandView): SequenceView {
  return {
    ...sequence,
    currentStepIndex: command.sequenceStepIndex ?? sequence.currentStepIndex ?? null,
    steps: sequence.steps.map((step) => (step.commandId === command.commandId ? command : step)),
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
      if (snapshot.activeSequence && event.command.parentSequenceId === snapshot.activeSequence.sequenceId) {
        return {
          ...snapshot,
          generatedAt: new Date().toISOString(),
          activeCommand: event.command,
          activeSequence: mergeSequenceStep(snapshot.activeSequence, event.command),
          planMetrics: event.planMetrics ?? event.command.metrics ?? snapshot.planMetrics,
          messages: event.messages ? [...snapshot.messages, ...event.messages] : snapshot.messages,
        };
      }
      return {
        ...snapshot,
        generatedAt: new Date().toISOString(),
        activeCommand: event.command,
        planMetrics: event.planMetrics ?? event.command.metrics ?? snapshot.planMetrics,
        messages: event.messages ? [...snapshot.messages, ...event.messages] : snapshot.messages,
      };
    case 'sequence_lifecycle':
      return {
        ...snapshot,
        generatedAt: new Date().toISOString(),
        activeSequence: event.sequence,
        activeCommand:
          snapshot.activeCommand && snapshot.activeCommand.parentSequenceId === event.sequence.sequenceId
            ? snapshot.activeCommand
            : event.sequence.steps[event.sequence.currentStepIndex ?? 0] ?? snapshot.activeCommand,
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

export const DEFAULT_JOG_STATUS: JogBridgeStatusSnapshot = {
  state: 'IDLE',
  pointsQueued: 0,
  effectiveHz: 0,
  robotReady: false,
  servoActive: false,
  bridgeActive: false,
  lastError: '',
  rejectionReason: '',
};

export interface UseGp4BridgeResult {
  state: HmiStateSnapshot;
  transportState: TransportState;
  jogBridgeStatus: JogBridgeStatusSnapshot;
  taskEvents: TaskEvent[];
  isController: boolean;
  blockingRuntime: boolean;
  submitCommand: (rawText: string) => Promise<CommandMutationResponse>;
  submitQuickCommand: (quickCommandId: string) => Promise<CommandMutationResponse>;
  confirmCommandById: (commandId: string, planFingerprint: string) => Promise<CommandMutationResponse>;
  acquireControllerLease: () => Promise<LeaseMutationResponse>;
  releaseLease: () => Promise<LeaseMutationResponse | null>;
  confirmActiveCommand: () => Promise<CommandMutationResponse | null>;
  abortActiveCommand: (reason?: string) => Promise<CommandMutationResponse | null>;
  startServo: () => Promise<ServoControlResponse | null>;
  holdServo: () => Promise<ServoControlResponse | null>;
  refreshReplay: () => Promise<ReplayListItem[]>;
}

export function useGP4Bridge(
  client: GP4BridgeClient,
  sessionId: string,
  operatorId: string,
): UseGp4BridgeResult {
  const [state, setState] = useState<HmiStateSnapshot>(createDisconnectedSnapshot);
  const [transportState, setTransportState] = useState<TransportState>('disconnected');
  const [jogBridgeStatus, setJogBridgeStatus] = useState<JogBridgeStatusSnapshot>(DEFAULT_JOG_STATUS);
  const [taskEvents, setTaskEvents] = useState<TaskEvent[]>([]);

  useEffect(() => {
    const disconnect = client.connect({
      sessionId,
      operatorId,
      onEvent: (event) => {
        if (event.type === 'jog_bridge_status') {
          setJogBridgeStatus(event.jogBridgeStatus);
          return;
        }
        if (event.type === 'task_event') {
          setTaskEvents((current) => {
            const next = [event.taskEvent, ...current];
            if (next.length > 500) next.length = 500;
            return next;
          });
          return;
        }
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
    if (!state.lease.ownsControl || !state.lease.leaseToken) {
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
  }, [client, operatorId, sessionId, state.lease.leaseToken, state.lease.ownsControl]);

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
    if (state.mode !== 'sim' && state.mode !== 'hardware') {
      throw new Error('Command mode is not command-capable.');
    }
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

  const submitQuickCommand = useCallback(async (quickCommandId: string) => {
    if (state.mode !== 'sim' && state.mode !== 'hardware') {
      throw new Error('Command mode is not command-capable.');
    }
    const response = await client.submitCommand({
      sessionId,
      operatorId,
      leaseToken: state.lease.leaseToken,
      quickCommandId,
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
    if (state.activeSequence?.planFingerprint) {
      const response = await client.confirmSequence(state.activeSequence.sequenceId, {
        sessionId,
        operatorId,
        leaseToken: state.lease.leaseToken,
        planFingerprint: state.activeSequence.planFingerprint,
      });
      if (response.snapshot) {
        setState(response.snapshot);
      }
      return response;
    }
    if (!state.activeCommand?.planFingerprint) {
      return null;
    }
    return confirmCommandById(state.activeCommand.commandId, state.activeCommand.planFingerprint);
  }, [client, confirmCommandById, operatorId, sessionId, state.activeCommand, state.activeSequence, state.lease.leaseToken]);

  const abortActiveCommand = useCallback(async (reason?: string) => {
    if (state.activeSequence) {
      const response = await client.abortSequence(state.activeSequence.sequenceId, {
        sessionId,
        operatorId,
        leaseToken: state.lease.leaseToken,
        reason,
      });
      if (response.snapshot) {
        setState(response.snapshot);
      }
      return response;
    }
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
  }, [client, operatorId, sessionId, state.activeCommand, state.activeSequence, state.lease.leaseToken]);

  const startServo = useCallback(async () => {
    if (!state.lease.leaseToken) {
      return null;
    }
    return client.startServo({
      sessionId,
      operatorId,
      leaseToken: state.lease.leaseToken,
    });
  }, [client, operatorId, sessionId, state.lease.leaseToken]);

  const holdServo = useCallback(async () => {
    if (!state.lease.leaseToken) {
      return null;
    }
    return client.stopServo({
      sessionId,
      operatorId,
      leaseToken: state.lease.leaseToken,
    });
  }, [client, operatorId, sessionId, state.lease.leaseToken]);

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
    jogBridgeStatus,
    taskEvents,
    isController: state.lease.ownsControl && state.lease.leaseToken !== null,
    blockingRuntime,
    submitCommand,
    submitQuickCommand,
    confirmCommandById,
    acquireControllerLease,
    releaseLease,
    confirmActiveCommand,
    abortActiveCommand,
    startServo,
    holdServo,
    refreshReplay,
  };
}
