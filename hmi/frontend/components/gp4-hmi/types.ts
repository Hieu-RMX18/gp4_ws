// Presentational view-model types shared between GP4HMI sub-components.
// These are display concerns only; they must never replace the authoritative
// shapes in shared/contracts.ts.

export type PillTone = 'green' | 'blue' | 'amber' | 'red' | 'cyan' | 'gray';

// Display role for chat bubbles. We intentionally keep `assistant` instead of
// `robot` so the UI cannot imply that LLM output is robot telemetry truth.
export type MessageRole = 'user' | 'assistant' | 'system';

export type LogLevel = 'info' | 'ok' | 'warn' | 'err';
export type PipelineStatus = 'done' | 'active' | 'pending' | 'error';

export interface TopicView {
  key: string;
  name: string;
  rateLabel: string;
  fillWidth: number;
}

export interface LogEntryView {
  id: string;
  time: string;
  level: LogLevel;
  message: string;
  source?: string | null;
}

export interface ActionFeedbackView {
  id: string;
  timestamp: string;
  level: LogLevel;
  message: string;
}

export interface TraceStepView {
  key: string;
  label: string;
  status: PipelineStatus;
  summary: string;
  details: Record<string, unknown> | null;
  traces: import('../../../shared/contracts').PipelineTrace[];
}

export interface StatusPillView {
  key: string;
  label: string;
  tone: PillTone;
}
