import type { ToolPose } from '../../../shared/contracts';

interface TcpPosePanelProps {
  toolPose: ToolPose | null;
}

export function TcpPosePanel({ toolPose }: TcpPosePanelProps) {
  const xyz = toolPose
    ? `X ${toolPose.x.toFixed(3)}  Y ${toolPose.y.toFixed(3)}  Z ${toolPose.z.toFixed(3)}`
    : '--  --  --';
  const rpy = toolPose
    ? `R ${(toolPose.roll * 180 / Math.PI).toFixed(1)}°  P ${(toolPose.pitch * 180 / Math.PI).toFixed(1)}°  Y ${(toolPose.yaw * 180 / Math.PI).toFixed(1)}°`
    : '--  --  --';

  return (
    <div className="tcp-pose-panel">
      <div className="tcp-row">
        <span className="tcp-label">tool0 XYZ</span>
        <span className="tcp-value">{xyz}</span>
      </div>
      <div className="tcp-row">
        <span className="tcp-label">tool0 RPY</span>
        <span className="tcp-value">{rpy}</span>
      </div>
      <div className="tcp-frame">{toolPose ? `frame: ${toolPose.frameId}` : 'frame: --'}</div>
    </div>
  );
}
