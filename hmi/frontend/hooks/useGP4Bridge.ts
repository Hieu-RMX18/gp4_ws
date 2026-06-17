import { useCallback, useEffect, useMemo, useState, useRef } from 'react';

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
  reconnect: () => void;
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
  const [reconnectTrigger, setReconnectTrigger] = useState(0);

  const stateRef = useRef(state);
  useEffect(() => {
    stateRef.current = state;
  }, [state]);

  const logTaskEvent = useCallback((
    level: 'DEBUG' | 'INFO' | 'WARN' | 'ERR',
    source: string,
    category: string,
    event: string,
    detail: string,
    data: Record<string, unknown> = {}
  ) => {
    setTaskEvents((current) => {
      const e: TaskEvent = {
        ts: new Date().toISOString(),
        level,
        source,
        category,
        event,
        detail,
        data,
      };
      const next = [e, ...current];
      if (next.length > 500) next.length = 500;
      return next;
    });
  }, []);

  const reconnect = useCallback(() => {
    logTaskEvent('INFO', 'hmi_client', 'TRANSPORT', 'reconnect_requested', 'Manual telemetry update requested. Re-connecting WebSocket link...');
    setReconnectTrigger((prev) => prev + 1);
  }, [logTaskEvent]);

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

        if (event.type === 'command_lifecycle') {
          const cmd = event.command;
          let generatedDetail = '';
          let generatedEvent = '';
          let level: 'INFO' | 'WARN' | 'ERR' = 'INFO';

          switch (cmd.lifecycleState) {
            case 'RECEIVED':
              generatedEvent = 'command_received';
              generatedDetail = `Command "${cmd.rawText}" accepted by Supervisor. ID: ${cmd.commandId}`;
              break;
            case 'PARSING':
              generatedEvent = 'intent_parsing';
              generatedDetail = `LLM gateway is parsing intent...`;
              break;
            case 'VALIDATING':
              generatedEvent = 'command_validating';
              generatedDetail = `Validating kinematics, collision-free path, and safety limits.`;
              break;
            case 'NEEDS_CONFIRMATION':
              generatedEvent = 'needs_confirmation';
              generatedDetail = `Kinematic check complete. Waiting for operator confirmation. (Risk: ${cmd.riskLevel || 'low'})`;
              level = 'WARN';
              break;
            case 'CONFIRMED':
              generatedEvent = 'command_confirmed';
              generatedDetail = `Operator confirmed execution. Launching action sequence.`;
              break;
            case 'EXECUTION_REQUESTED':
              generatedEvent = 'execution_requested';
              generatedDetail = `Request sent to ROS action client /execute_motion...`;
              break;
            case 'EXECUTING':
              generatedEvent = 'motion_executing';
              generatedDetail = `Robot is moving. Stream active...`;
              break;
            case 'SUCCEEDED':
              generatedEvent = 'motion_succeeded';
              generatedDetail = `Motion completed successfully. Pose target reached.`;
              break;
            case 'FAILED':
              generatedEvent = 'motion_failed';
              generatedDetail = `Execution failed: ${cmd.rejectReason || 'Unknown error'}`;
              level = 'ERR';
              break;
            case 'REJECTED':
              generatedEvent = 'command_rejected';
              generatedDetail = `Command rejected: ${cmd.rejectReason || 'Validation failed'}`;
              level = 'ERR';
              break;
            case 'CANCELLED':
              generatedEvent = 'command_cancelled';
              generatedDetail = `Command execution was cancelled by operator.`;
              level = 'WARN';
              break;
            case 'EXPIRED':
              generatedEvent = 'confirmation_expired';
              generatedDetail = `Confirmation time window expired.`;
              level = 'WARN';
              break;
          }

          if (generatedEvent) {
            logTaskEvent(level, 'supervisor', 'COMMAND', generatedEvent, generatedDetail, {
              commandId: cmd.commandId,
              state: cmd.lifecycleState,
            });
          }
        } else if (event.type === 'sequence_lifecycle') {
          const seq = event.sequence;
          let generatedDetail = '';
          let generatedEvent = '';
          let level: 'INFO' | 'WARN' | 'ERR' = 'INFO';

          switch (seq.lifecycleState) {
            case 'RECEIVED':
              generatedEvent = 'sequence_received';
              generatedDetail = `Sequence accepted by Supervisor. ID: ${seq.sequenceId} (${seq.stepCount} steps)`;
              break;
            case 'EXECUTING':
              generatedEvent = 'sequence_executing';
              generatedDetail = `Executing sequence step ${typeof seq.currentStepIndex === 'number' ? (seq.currentStepIndex + 1) : '?'}/${seq.stepCount}`;
              break;
            case 'SUCCEEDED':
              generatedEvent = 'sequence_succeeded';
              generatedDetail = `Sequence completed successfully!`;
              break;
            case 'FAILED':
              generatedEvent = 'sequence_failed';
              generatedDetail = `Sequence failed at step ${typeof seq.currentStepIndex === 'number' ? (seq.currentStepIndex + 1) : '?'}: ${seq.rejectReason || 'Unknown error'}`;
              level = 'ERR';
              break;
            case 'CANCELLED':
              generatedEvent = 'sequence_cancelled';
              generatedDetail = `Sequence execution cancelled by operator.`;
              level = 'WARN';
              break;
          }

          if (generatedEvent) {
            logTaskEvent(level, 'supervisor', 'SEQUENCE', generatedEvent, generatedDetail, {
              sequenceId: seq.sequenceId,
              state: seq.lifecycleState,
            });
          }
        } else if (event.type === 'snapshot') {
          const prevStatus = stateRef.current?.runtime?.robotStatus;
          const nextStatus = event.snapshot?.runtime?.robotStatus;

          if (prevStatus && nextStatus) {
            if (prevStatus.servoState !== nextStatus.servoState && nextStatus.servoState !== 'UNKNOWN') {
              logTaskEvent(
                nextStatus.servoState === 'ON' ? 'INFO' : 'WARN',
                'hw_adapter',
                'ROBOT',
                'servo_state_changed',
                `Robot drives/servo state: ${nextStatus.servoState}`,
                { servoState: nextStatus.servoState }
              );
            }
            if (prevStatus.eStop !== nextStatus.eStop && nextStatus.eStop !== 'UNKNOWN') {
              logTaskEvent(
                nextStatus.eStop === 'ACTIVE' ? 'ERR' : 'INFO',
                'hw_adapter',
                'ROBOT',
                'estop_changed',
                `Emergency stop is now: ${nextStatus.eStop}`,
                { eStop: nextStatus.eStop }
              );
            }
            if (prevStatus.alarmState !== nextStatus.alarmState && nextStatus.alarmState !== 'UNKNOWN') {
              logTaskEvent(
                nextStatus.alarmState === 'ACTIVE' ? 'ERR' : 'INFO',
                'hw_adapter',
                'ROBOT',
                'alarm_changed',
                `Robot alarm state changed to: ${nextStatus.alarmState} (${nextStatus.readinessMessage})`,
                { alarmState: nextStatus.alarmState, message: nextStatus.readinessMessage }
              );
            }
          }

          const prevPose = stateRef.current?.toolPose;
          const nextPose = event.snapshot?.toolPose;
          if (prevPose && nextPose) {
            const dist = Math.sqrt(
              Math.pow(nextPose.x - prevPose.x, 2) +
              Math.pow(nextPose.y - prevPose.y, 2) +
              Math.pow(nextPose.z - prevPose.z, 2)
            );
            if (dist > 0.005) {
              logTaskEvent(
                'INFO',
                'hw_adapter',
                'ROBOT',
                'pose_changed',
                `Robot TCP moved: X=${nextPose.x.toFixed(3)}, Y=${nextPose.y.toFixed(3)}, Z=${nextPose.z.toFixed(3)}`,
                { prevPose, nextPose }
              );
            }
          }
        }

        setState((current) => applyEvent(current, event));
      },
      onTransportStateChange: (nextState) => {
        setTransportState(nextState);
        logTaskEvent(
          nextState === 'connected' ? 'INFO' : 'WARN',
          'hmi_bridge',
          'TRANSPORT',
          `bridge_${nextState}`,
          `HMI WebSocket transport state changed to: ${nextState.toUpperCase()}`,
          { state: nextState }
        );
        setState((current) => applyEvent(current, { type: 'connection_state', transportState: nextState }));
      },
    });

    return disconnect;
  }, [client, operatorId, sessionId, reconnectTrigger, logTaskEvent]);

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
    logTaskEvent('INFO', 'hmi_client', 'COMMAND', 'submitting_intent', `Sending user intent to supervisor: "${rawText}"`, { rawText });
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
  }, [client, operatorId, sessionId, state.lease.leaseToken, state.mode, logTaskEvent]);

  const submitQuickCommand = useCallback(async (quickCommandId: string) => {
    if (state.mode !== 'sim' && state.mode !== 'hardware') {
      throw new Error('Command mode is not command-capable.');
    }
    logTaskEvent('INFO', 'hmi_client', 'COMMAND', 'submitting_intent', `Sending quick command to supervisor: "${quickCommandId}"`, { quickCommandId });
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
  }, [client, operatorId, sessionId, state.lease.leaseToken, state.mode, logTaskEvent]);

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
    logTaskEvent('INFO', 'hmi_client', 'COMMAND', 'confirming_command', 'Confirming command execution...', {
      commandId: state.activeCommand?.commandId,
      sequenceId: state.activeSequence?.sequenceId,
    });
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
  }, [client, confirmCommandById, operatorId, sessionId, state.activeCommand, state.activeSequence, state.lease.leaseToken, logTaskEvent]);

  const abortActiveCommand = useCallback(async (reason?: string) => {
    logTaskEvent('INFO', 'hmi_client', 'COMMAND', 'aborting_command', `Sending cancellation request... Reason: ${reason || 'none'}`, {
      commandId: state.activeCommand?.commandId,
      sequenceId: state.activeSequence?.sequenceId,
      reason,
    });
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
  }, [client, operatorId, sessionId, state.activeCommand, state.activeSequence, state.lease.leaseToken, logTaskEvent]);

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
    reconnect,
  };
}
