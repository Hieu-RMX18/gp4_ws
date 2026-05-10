import type { KeyboardEvent } from 'react';
import type { HardwareGateStatus } from '../../../shared/contracts';
import type { ActionFeedbackView } from './types';
import { formatTimestamp, hardwareGateLabel, reasonToVietnamese } from './derive';
import { INTENT_TEMPLATES } from './intentTemplates';

interface CommandComposerProps {
  draft: string;
  onDraftChange: (value: string) => void;
  canSubmit: boolean;
  readOnlyBridge: boolean;
  mode: 'sim' | 'hardware' | 'unknown';
  hardwareGate: HardwareGateStatus;
  submitError: string | null;
  actionFeedback: ActionFeedbackView | null;
  onSubmit: () => void;
  onClearError: () => void;
}

export function CommandComposer({
  draft,
  onDraftChange,
  canSubmit,
  readOnlyBridge,
  mode,
  hardwareGate,
  submitError,
  actionFeedback,
  onSubmit,
  onClearError,
}: CommandComposerProps) {
  const reasons = hardwareGate.reasons ?? [];
  const primaryReason =
    reasons[0] ?? 'Dual gate is not satisfied: runtime flag + signed evidence checklist are required.';
  const primaryReasonVi = reasonToVietnamese(primaryReason);

  const handleKeyDown = (event: KeyboardEvent<HTMLTextAreaElement>) => {
    if (event.nativeEvent.isComposing || event.repeat) {
      return;
    }
    if (event.key === 'Enter' && (event.ctrlKey || event.metaKey)) {
      event.preventDefault();
      onSubmit();
    }
  };

  const placeholder = readOnlyBridge
    ? mode === 'hardware' && !hardwareGate.unlocked
      ? `Hardware gate locked: ${primaryReason} | VI: ${primaryReasonVi}`
      : 'Command ingress is read only until mode + telemetry + preflight gates are satisfied.'
    : 'Type intent in English or Vietnamese. Ctrl+Enter to submit · Shift+Enter for newline.';

  return (
    <div className="chat-input-wrap">
      {!canSubmit ? (
        <div className="input-blocked-banner">
          <span className="input-blocked-icon">⊘</span>
          <span>
            {readOnlyBridge
              ? 'Command ingress is blocked. Check telemetry freshness, hardware gate, and runtime state.'
              : 'Submit disabled — waiting for command-capable conditions.'}
          </span>
        </div>
      ) : null}
      <div className="input-row">
        <textarea
          className="chat-input"
          rows={1}
          value={draft}
          disabled={!canSubmit}
          onChange={(e) => onDraftChange(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
        />
        <button
          className="send-btn"
          disabled={!canSubmit || draft.trim().length === 0}
          onClick={onSubmit}
        >
          Submit
        </button>
      </div>
      {submitError ? <div className="input-error">{submitError}</div> : null}
      {actionFeedback ? (
        <div className={`input-feedback ${actionFeedback.level}`}>
          {formatTimestamp(actionFeedback.timestamp)} · {actionFeedback.message}
        </div>
      ) : null}
      <div className="hint-row">
        {INTENT_TEMPLATES.map((template) => (
          <button
            key={`hint-${template.id}`}
            type="button"
            className="hint"
            onClick={() => {
              onDraftChange(template.intent);
              onClearError();
            }}
          >
            {template.intent}
          </button>
        ))}
      </div>
    </div>
  );
}
