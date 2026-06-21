import type { ToolPose } from '../../../shared/contracts';

interface TcpPosePanelProps {
  toolPose: { tcp: ToolPose; tool0: ToolPose } | null;
}

export function TcpPosePanel({ toolPose }: TcpPosePanelProps) {
  const formatPose = (p: ToolPose | undefined) => {
    if (!p) return { xyz: '--  --  --', rpy: '--  --  --' };
    return {
      xyz: `X ${p.x.toFixed(3)}  Y ${p.y.toFixed(3)}  Z ${p.z.toFixed(3)}`,
      rpy: `R ${(p.roll * 180 / Math.PI).toFixed(1)}°  P ${(p.pitch * 180 / Math.PI).toFixed(1)}°  Y ${(p.yaw * 180 / Math.PI).toFixed(1)}°`
    };
  };

  const tool0Fmt = formatPose(toolPose?.tool0);
  const tcpFmt = formatPose(toolPose?.tcp);

  return (
    <div className="tcp-pose-panel">
      <div className="tcp-row" style={{ marginTop: '4px' }}>
        <span className="tcp-label" style={{ fontWeight: 'bold' }}>TCP XYZ</span>
        <span className="tcp-value" style={{ fontWeight: 'bold', color: '#10b981' }}>{tcpFmt.xyz}</span>
      </div>
      <div className="tcp-row">
        <span className="tcp-label">TCP RPY</span>
        <span className="tcp-value">{tcpFmt.rpy}</span>
      </div>
      
      <div className="tcp-row" style={{ marginTop: '8px', opacity: 0.8 }}>
        <span className="tcp-label" style={{ fontSize: '0.9em' }}>tool0 XYZ</span>
        <span className="tcp-value" style={{ fontSize: '0.9em' }}>{tool0Fmt.xyz}</span>
      </div>
      <div className="tcp-row" style={{ opacity: 0.8 }}>
        <span className="tcp-label" style={{ fontSize: '0.9em' }}>tool0 RPY</span>
        <span className="tcp-value" style={{ fontSize: '0.9em' }}>{tool0Fmt.rpy}</span>
      </div>
      
      <div className="tcp-frame">{toolPose ? `frame: ${toolPose.tcp.frameId}` : 'frame: --'}</div>
    </div>
  );
}
