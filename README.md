# GP4 / YRC1000micro — ROS 2 Agentic Robot Control

LLM-driven motion planning and execution system for the **Yaskawa GP4** industrial robot arm
with **YRC1000micro** controller, built on **ROS 2 Humble + MoveIt 2 + MotoROS2**.

## Architecture

\`\`\`
LLM / user intent (natural language)
  → llm_gateway        (parse / normalize / schema validation / approval request)
  → safety             (semantic checks + workspace bounds + readiness gate)
  → motion_core        (planning, IK, smoothing, quality gate, query service)
       → primitives    (PTP / LIN / HOME / MOVE_REL)
       → logic         (WAIT, STOP, SET_SPEED)
       → services      (GET_POSE query client)
       → planner router
       → IK selector   (TRAC-IK + Distance)
       → trajectory post-processor (TOTG + Ruckig)
       → quality gate  (wrist-flip guard + point budget + fraction check)
  → hw_adapter         (sole execution backend + device services)
       → AlarmReset / IoSet / GetCurrentPose servers
       → StartTrajMode / StopMotion / ResetError
       → /yaskawa/follow_joint_trajectory
  → MotoROS2 / GP4
\`\`\`

### Ownership Rules

| Package | Responsibility |
|---------|---------------|
| \`llm_gateway\` | Parse / normalize / schema validation / approval request |
| \`safety\` | Semantic checks + workspace bounds + readiness gate (fail-closed) |
| \`motion_core\` | Planning, IK, smoothing, quality gate |
| \`hw_adapter\` | **Sole execution backend**; hardware integration services (I/O, recovery) |
| \`supervisor\` | Audit + diagnostics + benchmark logging |
| \`primitives\` | Core motion sub-primitives (PTP, LIN, CIRC, etc.) |
| \`interfaces\` | Action/Service/Msg definitions (ExecuteMotion, AlarmReset, IoSet, etc.) |
| \`gp4_bringup\` | Launch files (sim.launch.py, system.launch.py) |
| \`gp4_moveit_config\` | MoveIt 2 configuration |

## Supported Primitives (Version 1.2)

The system supports 12 public primitives fully wired end-to-end:

| Primitive | Type | Description |
|-----------|------|-------------|
| **HOME** | Motion | Move to factory home position |
| **PTP** | Motion | Joint-space planning |
| **LIN** | Motion | Cartesian linear planning |
| **MOVE_REL**| Motion | Relative translation in base_link (delta_x, y, z) |
| **GET_POSE**| Query | Query current TCP pose in real-time |
| **SET_SPEED**| Logic | Stateless velocity scaling for current request |
| **WAIT** | Logic | Execution-aware pause (seconds) |
| **STOP** | Logic | Emergency halt of motion and cancellation of goals |
| **MOVE_JOINT**| Joint | Targeted motion to a specific joint index |
| **MOVE_JOINTS**| Joint | Full joint configuration move (alias to PTP) |
| **IO_SET** | Device | Set robot/tool digital I/O address |
| **ALARM_RESET**| Recovery| Trigger alarm reset on robot controller |

## Quick Start

### Build

\`\`\`bash
source /opt/ros/humble/setup.bash
cd ~/gp4_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
\`\`\`

### Run (Simulation)

\`\`\`bash
# Full stack simulation with fake hardware
ros2 launch gp4_bringup sim.launch.py
\`\`\`

### Send a Motion Command

\`\`\`bash
# 1. MOVE_JOINT (Targeted joint move)
ros2 topic pub --once /llm_raw_command std_msgs/msg/String \\
  "data: '{\\"primitive_type\\": \\"MOVE_JOINT\\", \\"joint_index\\": 0, \\"joint_angle\\": 0.1, \\"velocity_scale\\": 0.1}'"

# 2. WAIT (Pause execution)
ros2 topic pub --once /llm_raw_command std_msgs/msg/String \\
  "data: '{\\"primitive_type\\": \\"WAIT\\", \\"wait_duration_sec\\": 2.0}'"

# 3. STOP (Immediate halt)
ros2 topic pub --once /llm_raw_command std_msgs/msg/String \\
  "data: '{\\"primitive_type\\": \\"STOP\\"}'"

# 4. IO_SET (Device control)
ros2 topic pub --once /llm_raw_command std_msgs/msg/String \\
  "data: '{\\"primitive_type\\": \\"IO_SET\\", \\"io_address\\": 10010, \\"io_value\\": 1}'"
\`\`\`

## Environment

| Variable | Purpose |
|----------|---------|
| \`GP4_LLM_API_KEY\` | API key for the LLM gateway backend |

## License

Apache License 2.0
