import { useEffect, useMemo, useRef, useState } from 'react';

import type { UseGp4BridgeResult } from '../hooks/useGP4Bridge';
import type { ActionFeedbackView, LogEntryView, StatusPillView, TopicView } from './gp4-hmi/types';
import {
  buildReviewLogEntries,
  buildTraceSteps,
  formatClock,
  formatTimestamp,
  humanizeLabel,
  JOINT_ORDER,
  resolveDeclineReason,
  shouldShowMessageInSystemLog,
  summarizeMutationResponse,
  toLogLevel,
  toneFromConnectionHealth,
  toneFromLifecycle,
  toneFromMode,
  toneFromServoState,
  toTopicFillWidth,
  toTopicRateLabel,
} from './gp4-hmi/derive';
import type { LogLevel } from './gp4-hmi/types';
import { RuntimeStateBanner } from './RuntimeStateBanner';
import {
  Topbar,
  JointMonitor,
  CommandPipelinePanel,
  QuickCommands,
  ChatPanel,
  CommandComposer,
  TcpPosePanel,
  TelemetrySources,
  ControlLeasePanel,
  SystemLog,
} from './gp4-hmi';

interface GP4HMIProps {
  bridge: UseGp4BridgeResult;
}

export function GP4HMI({ bridge }: GP4HMIProps) {
  const [draft, setDraft] = useState('');
  const [clockText, setClockText] = useState(() => formatClock(new Date()));
  const [submitError, setSubmitError] = useState<string | null>(null);
  const [actionFeedback, setActionFeedback] = useState<ActionFeedbackView | null>(null);
  const [latestMutationReason, setLatestMutationReason] = useState<string | null>(null);
  const [leasePendingAction, setLeasePendingAction] = useState<'acquire' | 'release' | null>(null);
  const submitPendingRef = useRef(false);
  const confirmPendingRef = useRef(false);

  const {
    state,
    isController,
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
  } = bridge;

  const readOnlyBridge = state.capabilities.readOnly;
  const canSubmitCommands =
    state.capabilities.canSubmitCommands &&
    state.capabilities.commandIngressAvailable &&
    !blockingRuntime &&
    isController;
  const canAcquireLease = state.capabilities.canAcquireLease;
  const canReleaseLease = isController && state.lease.leaseToken !== null;
  const canConfirmCommands = state.capabilities.canConfirmCommands && !blockingRuntime && isController;
  const canAbortCommands = (state.capabilities.canCancelCommands || state.capabilities.canAbortCommands) && isController;
  const activeCommand = state.activeCommand;
  const activeSequence = state.activeSequence;
  const activeReviewJob = activeSequence ?? activeCommand;
  const blockingReasons = useMemo(
    () => activeSequence?.validationResult?.blockingReasons ?? activeCommand?.validationResult?.blockingReasons ?? [],
    [activeSequence?.validationResult?.blockingReasons, activeCommand?.validationResult?.blockingReasons],
  );
  const traceSteps = useMemo(() => buildTraceSteps(activeCommand), [activeCommand]);
  const declineReason = useMemo(
    () => resolveDeclineReason(latestMutationReason, activeCommand),
    [latestMutationReason, activeCommand],
  );

  const pushActionFeedback = (level: LogLevel, message: string, reason?: string | null) => {
    setActionFeedback({
      id: `action-${Date.now()}`,
      timestamp: new Date().toISOString(),
      level,
      message,
    });
    setLatestMutationReason(reason?.trim() || null);
  };

  useEffect(() => {
    const timer = window.setInterval(() => setClockText(formatClock(new Date())), 1000);
    return () => window.clearInterval(timer);
  }, []);

  const orderedJoints = useMemo(() => {
    const jointMap = new Map(state.jointPositions.map((j) => [j.name, j]));
    return JOINT_ORDER.map((name) =>
      jointMap.get(name) ?? { name, positionDeg: null, minDeg: -180, maxDeg: 180 },
    );
  }, [state.jointPositions]);

  const connectionMap = useMemo(
    () => new Map(state.connections.map((c) => [c.name, c])),
    [state.connections],
  );

  const statusPills = useMemo<StatusPillView[]>(() => {
    const ros2 = connectionMap.get('ros2');
    const moveit2 = connectionMap.get('moveit2');
    const llm = connectionMap.get('llm');
    const controlLabel = leasePendingAction
      ? 'Control Pending'
      : isController
        ? 'Control Lease'
        : canAcquireLease
          ? 'Control Available'
          : 'Control Read-only';
    return [
      { key: 'ros2', label: `ROS2 ${ros2?.health === 'healthy' ? 'Connected' : ros2?.health === 'degraded' ? 'Degraded' : 'Down'}`, tone: toneFromConnectionHealth(ros2?.health) },
      { key: 'llm', label: `LLM ${llm?.health === 'healthy' ? 'OK' : llm?.health === 'degraded' ? 'Degraded' : 'Down'}`, tone: toneFromConnectionHealth(llm?.health) },
      { key: 'servo', label: `Servo ${state.runtime.robotStatus.servoState}`, tone: toneFromServoState(state.runtime.robotStatus.servoState) },
      { key: 'control', label: controlLabel, tone: leasePendingAction ? 'amber' : isController ? 'green' : canAcquireLease ? 'amber' : 'gray' },
      { key: 'mode', label: `${state.mode === 'sim' ? 'Simulation' : state.mode === 'hardware' ? 'Hardware' : 'Mode Unknown'}`, tone: toneFromMode(state.mode) },
      { key: 'moveit2', label: `MoveIt2 ${moveit2?.health === 'healthy' ? 'Ready' : moveit2?.health === 'degraded' ? 'Degraded' : 'Down'}`, tone: toneFromConnectionHealth(moveit2?.health) },
      { key: 'command', label: `${activeSequence ? 'Sequence' : 'Command'} ${activeReviewJob?.lifecycleState ? humanizeLabel(activeReviewJob.lifecycleState) : 'Idle'}`, tone: toneFromLifecycle(activeReviewJob?.lifecycleState) },
    ];
  }, [activeReviewJob?.lifecycleState, activeSequence, canAcquireLease, connectionMap, isController, leasePendingAction, state.mode, state.runtime.robotStatus.servoState]);

  const topicRows = useMemo<TopicView[]>(() => {
    if (state.telemetrySources.length > 0) {
      return [...state.telemetrySources]
        .sort((a, b) => Number(b.active) - Number(a.active))
        .slice(0, 6)
        .map((s) => ({ key: s.name, name: s.topic, rateLabel: toTopicRateLabel(s.freshnessState), fillWidth: toTopicFillWidth(s.freshnessState) }));
    }
    return state.connections.slice(0, 6).map((c) => ({
      key: c.name, name: c.label,
      rateLabel: c.health === 'healthy' ? 'FRESH' : c.health === 'degraded' ? 'STALE' : 'DOWN',
      fillWidth: c.health === 'healthy' ? 100 : c.health === 'degraded' ? 40 : 10,
    }));
  }, [state.connections, state.telemetrySources]);

  const logEntries = useMemo<LogEntryView[]>(() => {
    const reviewEntries = buildReviewLogEntries(activeReviewJob, blockingReasons, declineReason, state.generatedAt);
    const entries: LogEntryView[] = state.messages
      .filter(shouldShowMessageInSystemLog)
      .slice(-25)
      .map((m) => ({ id: m.id, time: formatTimestamp(m.timestamp), level: toLogLevel(m), message: m.text, source: m.source ?? null }));
    entries.unshift(...reviewEntries);
    if (actionFeedback) {
      entries.unshift({ id: actionFeedback.id, time: formatTimestamp(actionFeedback.timestamp), level: actionFeedback.level, message: actionFeedback.message, source: null });
    }
    if (entries.length > 0) return entries.slice(0, 30);
    return [{ id: 'runtime-bootstrap', time: formatTimestamp(state.generatedAt), level: state.runtime.blocking ? 'warn' as const : 'info' as const, message: state.runtime.statusText, source: null }];
  }, [actionFeedback, activeReviewJob, blockingReasons, declineReason, state.generatedAt, state.messages, state.runtime.blocking, state.runtime.statusText]);

  // ── Action handlers ─────────────────────────────────────────────────────

  const handleSubmit = async () => {
    if (submitPendingRef.current) return;
    const rawText = draft.trim();
    if (!rawText || !canSubmitCommands) return;

    submitPendingRef.current = true;
    try {
      const response = await submitCommand(rawText);
      if (!response.accepted) {
        const reason = resolveDeclineReason(response.reason, response.command);
        setSubmitError(reason ?? 'Command rejected by supervisor validation.');
        pushActionFeedback('err', `Submit declined · ${reason ?? 'no reason provided'}`, reason);
        return;
      }
      setDraft('');
      setSubmitError(null);
      pushActionFeedback('ok', `Submit response · ${summarizeMutationResponse(response)}`, response.reason);

      const shouldAutoConfirmSim =
        response.jobType === 'command' &&
        response.command !== null &&
        response.command.lifecycleState === 'NEEDS_CONFIRMATION' &&
        response.command.mode === 'sim' &&
        response.command.planFingerprint !== null;
      if (shouldAutoConfirmSim) {
        const cmd = response.command as NonNullable<typeof response.command>;
        try {
          const cr = await confirmCommandById(response.commandId as string, cmd.planFingerprint as string);
          const crReason = resolveDeclineReason(cr.reason, cr.command);
          if (!cr.accepted) {
            pushActionFeedback('err', `Sim auto-confirm declined · ${crReason ?? 'no reason provided'}`, crReason);
            return;
          }
          pushActionFeedback('ok', `Sim auto-confirm · ${summarizeMutationResponse(cr)}`, cr.reason);
        } catch (e: unknown) {
          const msg = e instanceof Error ? e.message : 'Sim auto-confirm failed.';
          pushActionFeedback('err', `Sim auto-confirm failed · ${msg}`, msg);
        }
      }
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : 'Failed to submit intent.';
      setSubmitError(msg);
      pushActionFeedback('err', `Submit failed · ${msg}`, msg);
    } finally {
      submitPendingRef.current = false;
    }
  };

  const handleConfirmClick = async () => {
    if (confirmPendingRef.current) return;
    confirmPendingRef.current = true;
    try {
      const response = await confirmActiveCommand();
      if (!response) { pushActionFeedback('warn', 'Confirm skipped · no active command available.'); return; }
      const reason = resolveDeclineReason(response.reason, response.command);
      if (!response.accepted) { pushActionFeedback('err', `Confirm declined · ${reason ?? 'no reason provided'}`, reason); return; }
      pushActionFeedback('ok', `Confirm response · ${summarizeMutationResponse(response)}`, response.reason);
    } catch (e: unknown) {
      pushActionFeedback('err', `Confirm failed · ${e instanceof Error ? e.message : 'unknown'}`, null);
    } finally {
      confirmPendingRef.current = false;
    }
  };

  const handleAbortClick = async (reason: string) => {
    try {
      const response = await abortActiveCommand(reason);
      if (!response) { pushActionFeedback('warn', 'Abort skipped · no active command available.'); return; }
      const resolved = resolveDeclineReason(response.reason, response.command);
      if (!response.accepted) { pushActionFeedback('err', `Abort declined · ${resolved ?? 'no reason provided'}`, resolved); return; }
      pushActionFeedback('warn', `Abort response · ${summarizeMutationResponse(response)}`, resolved);
    } catch (e: unknown) {
      pushActionFeedback('err', `Abort failed · ${e instanceof Error ? e.message : 'unknown'}`, null);
    }
  };

  const handleAcquireLease = async () => {
    if (!canAcquireLease || leasePendingAction !== null) return;
    setLeasePendingAction('acquire');
    try {
      const response = await acquireControllerLease();
      if (!response.accepted) {
        const reason = response.reason ?? 'controller lease was not granted';
        pushActionFeedback('err', `Lease request declined · ${reason}`, reason);
        return;
      }
      pushActionFeedback('ok', 'Control lease acquired · servo controls are now lease-authorized');
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : 'unknown';
      pushActionFeedback('err', `Lease request failed · ${msg}`, msg);
    } finally {
      setLeasePendingAction(null);
    }
  };

  const handleReleaseLease = async () => {
    if (!canReleaseLease || leasePendingAction !== null) return;
    setLeasePendingAction('release');
    try {
      const response = await releaseLease();
      if (!response) {
        pushActionFeedback('warn', 'Lease release skipped · no active lease token');
        return;
      }
      if (!response.accepted) {
        const reason = response.reason ?? 'controller lease was not released';
        pushActionFeedback('err', `Lease release declined · ${reason}`, reason);
        return;
      }
      pushActionFeedback('ok', 'Control lease released');
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : 'unknown';
      pushActionFeedback('err', `Lease release failed · ${msg}`, msg);
    } finally {
      setLeasePendingAction(null);
    }
  };

  const canAbortTopbar = canAbortCommands && (activeSequence !== null || activeCommand !== null);
  const hasControllerLease = isController && state.lease.leaseToken !== null;
  const robotAllowsServoControl =
    state.runtime.robotStatus.eStop === 'CLEAR' &&
    state.runtime.robotStatus.alarmState === 'NONE';
  const canStartServo =
    state.mode === 'hardware' &&
    state.transportState === 'connected' &&
    hasControllerLease &&
    robotAllowsServoControl &&
    state.runtime.robotStatus.motionMode === 'AUTO' &&
    state.runtime.robotStatus.servoState !== 'ON';
  const canHoldServo =
    state.mode === 'hardware' &&
    state.transportState === 'connected' &&
    hasControllerLease &&
    robotAllowsServoControl;

  // ── Render ──────────────────────────────────────────────────────────────

  return (
    <div className="hmi-shell">
      <div className="hmi-app">
        <Topbar
          statusPills={statusPills}
          clockText={clockText}
          canAbort={canAbortTopbar}
          canStartServo={canStartServo}
          canHoldServo={canHoldServo}
          onStartServo={startServo}
          onHoldServo={holdServo}
          onAbort={() => void handleAbortClick('Operator requested topbar abort from HMI.')}
          onActionFeedback={pushActionFeedback}
        />

        <aside className="left-panel">
          <div className="panel-header">Robot Monitor</div>
          <CommandPipelinePanel traceSteps={traceSteps} />
          <QuickCommands canSubmit={canSubmitCommands} onSelect={(quickCommandId) => {
            if (!canSubmitCommands) return;
            void (async () => {
              try {
                const response = await submitQuickCommand(quickCommandId);
                if (!response.accepted) {
                  const reason = resolveDeclineReason(response.reason, response.command);
                  pushActionFeedback('err', `Quick command declined · ${reason ?? 'no reason provided'}`, reason);
                  return;
                }
                pushActionFeedback('ok', `Quick command · ${summarizeMutationResponse(response)}`, response.reason);
                const shouldAutoConfirmSim =
                  response.jobType === 'command' &&
                  response.command !== null &&
                  response.command.lifecycleState === 'NEEDS_CONFIRMATION' &&
                  response.command.mode === 'sim' &&
                  response.command.planFingerprint !== null;
                if (shouldAutoConfirmSim) {
                  const cmd = response.command as NonNullable<typeof response.command>;
                  try {
                    const cr = await confirmCommandById(response.commandId as string, cmd.planFingerprint as string);
                    const crReason = resolveDeclineReason(cr.reason, cr.command);
                    if (!cr.accepted) {
                      pushActionFeedback('err', `Sim auto-confirm declined · ${crReason ?? 'no reason provided'}`, crReason);
                      return;
                    }
                    pushActionFeedback('ok', `Sim auto-confirm · ${summarizeMutationResponse(cr)}`, cr.reason);
                  } catch (e: unknown) {
                    const msg = e instanceof Error ? e.message : 'Sim auto-confirm failed.';
                    pushActionFeedback('err', `Sim auto-confirm failed · ${msg}`, msg);
                  }
                }
              } catch (e: unknown) {
                const msg = e instanceof Error ? e.message : 'Quick command failed.';
                pushActionFeedback('err', `Quick command failed · ${msg}`, msg);
              }
            })();
          }} />
          <JointMonitor orderedJoints={orderedJoints} />
        </aside>

        <main className="center-panel">
          <div className="chat-header">
            <span className="chat-title">LLM Command Interface - Yaskawa GP4</span>
            <span className="chat-sub">
              transport {state.transportState} · schema {state.schemaVersion} · {readOnlyBridge ? 'read only' : 'command ingress enabled'}
            </span>
          </div>

          <div className="chat-messages">
            <RuntimeStateBanner runtime={state.runtime} />
            <ChatPanel
              messages={state.messages}
              activeCommand={activeCommand}
              activeSequence={activeSequence}
              canConfirm={canConfirmCommands}
              canAbort={canAbortCommands}
              activeReviewJobFingerprint={activeReviewJob?.planFingerprint ?? null}
              onConfirm={() => void handleConfirmClick()}
              onAbort={() => void handleAbortClick('Operator aborted from HMI confirmation panel.')}
            />
          </div>

          <CommandComposer
            draft={draft}
            onDraftChange={setDraft}
            canSubmit={canSubmitCommands}
            readOnlyBridge={readOnlyBridge}
            mode={state.mode}
            submitError={submitError}
            actionFeedback={actionFeedback}
            onSubmit={() => void handleSubmit()}
            onClearError={() => setSubmitError(null)}
          />
        </main>

        <aside className="right-panel">
          <div className="panel-header">TCP Pose</div>
          <TcpPosePanel toolPose={state.toolPose} />
          <TelemetrySources topicRows={topicRows} />
          <ControlLeasePanel
            lease={state.lease}
            capabilities={state.capabilities}
            readOnlyBridge={readOnlyBridge}
            isController={isController}
            mode={state.mode}
            canAcquireLease={canAcquireLease}
            canReleaseLease={canReleaseLease}
            leasePendingAction={leasePendingAction}
            onAcquire={() => void handleAcquireLease()}
            onRelease={() => void handleReleaseLease()}
          />
          <SystemLog logEntries={logEntries} />
        </aside>
      </div>
    </div>
  );
}
