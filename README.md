# GP4 / YRC1000micro — ROS 2 Agentic Robot Control

LLM-driven motion planning and execution system for the **Yaskawa GP4** industrial robot arm
with **YRC1000micro** controller, built on **ROS 2 Humble + MoveIt 2 + MotoROS2**.

## Architecture

```
LLM / user intent (natural language)
  → llm_gateway        (parse / normalize / schema validation / approval request)
  → safety             (semantic checks + workspace bounds + readiness gate)
  → motion_core        (planning, IK, smoothing, quality gate)
       → primitives    (PTP / LIN / CIRC / HOME / approach / retract / sequence)
       → planner router
       → IK selector   (TRAC-IK + Distance)
       → trajectory post-processor (TOTG + Ruckig)
       → quality gate  (wrist-flip guard + point budget + fraction check)
  → hw_adapter         (sole execution backend)
       → StartTrajMode / StopMotion / ResetError
       → /yaskawa/follow_joint_trajectory
  → MotoROS2 / GP4
```

### Ownership Rules

| Package | Responsibility |
|---------|---------------|
| `llm_gateway` | Parse / normalize / schema validation / approval request |
| `safety` | Semantic checks + workspace bounds + readiness gate (fail-closed) |
| `motion_core` | Planning, IK, smoothing, quality gate — **plan only, never executes** |
| `hw_adapter` | **Sole execution backend** for both simulation and real hardware |
| `supervisor` | Audit + diagnostics + benchmark logging |
| `primitives` | PTP/LIN/CIRC/HOME/approach/retract/sequence implementations |
| `interfaces` | Custom msgs, srvs, actions (ExecuteMotion, DispatchTrajectory, ValidateCommand) |
| `gp4_bringup` | Launch files for simulation and real hardware |
| `gp4_moveit_config` | MoveIt 2 config (SRDF, kinematics, joint limits, controllers) |

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- TRAC-IK solver
- `industrial_msgs` package

### Build

```bash
source /opt/ros/humble/setup.bash
cd ~/gp4_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Run (Simulation)

```bash
ros2 launch gp4_bringup system.launch.py use_fake_hardware:=true
```

### Run (Real Hardware)

**Pre-requisites:**
1. micro-ROS Agent running on UDP port 8888
2. `GP4_LLM_API_KEY` environment variable set
3. Robot in AUTO mode, drives powered, no errors

```bash
export GP4_LLM_API_KEY="your-api-key-here"
ros2 launch gp4_bringup system.launch.py use_fake_hardware:=false
```

### Send a Motion Command

```bash
# Via LLM intent (natural language)
ros2 topic pub --once /llm_intent std_msgs/msg/String "data: 'move to home position'"

# Via direct raw command
ros2 topic pub --once /llm_raw_command std_msgs/msg/String \
  "data: '{\"primitive_type\": \"HOME\", \"velocity_scale\": 0.1}'"
```

## Key Design Principles

1. **`hw_adapter` is the only component allowed to talk to MotoROS2 execution APIs.**
2. **`motion_core` may only plan + validate + post-process. It never executes directly.**
3. **Simulation and real hardware use the same `/execute_motion` contract.**
4. **Every trajectory passes through the same validation pipeline.**
5. **Safety is fail-closed.**

## Environment

| Variable | Purpose |
|----------|---------|
| `RMW_IMPLEMENTATION` | Must be `rmw_fastrtps_cpp` (set automatically by launch files) |
| `GP4_LLM_API_KEY` | API key for the LLM backend |

## License

Apache License 2.0
