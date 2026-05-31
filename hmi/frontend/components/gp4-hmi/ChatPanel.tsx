import type { ChatMessage, CommandView, SequenceView } from '../../../shared/contracts';
import {
  avatarForRole,
  formatTimestamp,
  humanizeLabel,
  nameForRole,
  tagClassName,
  toMessageRole,
} from './derive';

interface ChatPanelProps {
  messages: ChatMessage[];
  activeCommand: CommandView | null;
  activeSequence: SequenceView | null;
  canConfirm: boolean;
  canAbort: boolean;
  activeReviewJobFingerprint: string | null;
  onConfirm: () => void;
  onAbort: () => void;
}

export function ChatPanel({
  messages,
  activeCommand,
  activeSequence,
  canConfirm,
  canAbort,
  activeReviewJobFingerprint,
  onConfirm,
  onAbort,
}: ChatPanelProps) {
  if (messages.length === 0) {
    return (
      <div className="msg system">
        <div className="msg-avatar">SYS</div>
        <div className="msg-body">
          <div className="msg-name">System · waiting</div>
          <div className="msg-bubble">
            Waiting for bridge-backed lifecycle events. This panel renders backend truth only.
          </div>
        </div>
      </div>
    );
  }

  return (
    <>
      {messages.map((message) => {
        const role = toMessageRole(message.origin);
        const needsConfirmation =
          message.tag === 'NEEDS_CONFIRMATION' &&
          (activeCommand?.commandId === message.commandId ||
            activeSequence?.sequenceId === message.commandId);

        return (
          <div key={message.id} className={`msg ${role === 'assistant' ? 'robot' : role}`}>
            <div className="msg-avatar">{avatarForRole(role)}</div>
            <div className="msg-body">
              <div className="msg-name">
                {nameForRole(role)} · {formatTimestamp(message.timestamp)}
              </div>
              {message.tag ? (
                <span className={tagClassName(message.tag)}>{humanizeLabel(message.tag)}</span>
              ) : null}
              <div className="msg-bubble">{message.text}</div>

              {needsConfirmation ? (
                <div className="confirm-btns">
                  <button
                    className="btn-confirm yes"
                    disabled={!canConfirm || !activeReviewJobFingerprint}
                    onClick={onConfirm}
                  >
                    Confirm and execute
                  </button>
                  <button
                    className="btn-confirm no"
                    disabled={!canAbort}
                    onClick={onAbort}
                  >
                    Abort
                  </button>
                </div>
              ) : null}
            </div>
          </div>
        );
      })}
    </>
  );
}
