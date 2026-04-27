import { INTENT_TEMPLATES } from './intentTemplates';

interface QuickCommandsProps {
  onSelect: (intent: string) => void;
}

export function QuickCommands({ onSelect }: QuickCommandsProps) {
  return (
    <section className="section section-grow">
      <div className="section-title">Quick Commands</div>
      <div className="quick-cmds">
        {INTENT_TEMPLATES.map((template) => (
          <button
            key={template.id}
            type="button"
            className="qcmd"
            onClick={() => onSelect(template.intent)}
          >
            <span className="cmd-icon">&gt;</span>
            <span className="qcmd-text">
              <span className="qcmd-title">{template.title}</span>
              <span className="qcmd-subtitle">{template.subtitle}</span>
            </span>
          </button>
        ))}
      </div>
    </section>
  );
}
