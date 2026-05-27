# Yaskawa GP4 / YRC1000micro — ROS 2 Agentic Robot Control

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Rolling-green.svg)](https://moveit.ros.org/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

An end-to-end, deterministic LLM-driven motion planning and execution system for the **Yaskawa GP4** industrial robot arm with **YRC1000micro** controller. Built on **ROS 2 Humble + MoveIt 2 + MotoROS2**, translating natural language intents into collision-aware, hardware-safe robot actions.

---

## Table of Contents

1. [Core Features](#core-features)
2. [System Architecture](#system-architecture)
3. [Safety & Constraints](#safety--constraints)
4. [Supported Primitives](#supported-primitives)
5. [Prerequisites](#prerequisites)
6. [Installation & Build](#installation--build)
7. [Getting Started](#getting-started)
8. [Example Commands](#example-commands)
9. [HMI Web Interface](#hmi-web-interface)
10. [Configuration](#configuration)

---

## Core Features

- **LLM Intent Gateway:** Translates user requests into formal structured commands with strictly enforced JSON schemas.
- **Fail-Closed Safety Engine:** Evaluates targets against workspace bounds, forbidden zones, and mechanical constraints before planning occurs.
- **Multi-Stage Safety Guards:** Three-stage protection pipeline:
  - **Stage A (Pre-Planning):** JointPositionGuard in PrimitiveRouterDispatch validates against operational limits before trajectory downsampling
  - **Stage B (Quality Gate):** JointPositionGuard and ManipulabilityGuard (Yoshikawa index) validate plan quality before dispatch
  - **Stage C (Dispatch Boundary):** JointPositionGuard in hw_adapter validates final trajectory before hardware execution
  - **WristFlipGuard:** Tracks cumulative wrist rotation to prevent excessive joint wrapping
- **Advanced Motion Core:** Collision-aware planning via MoveIt 2, TRAC-IK inverse kinematics, and smooth trajectory generation (TOTG + Ruckig). Execution logic is split into focused modules per primitive type.
- **Hardened Execution Pipeline:** Connects via MotoROS2 driver. Separates motion dispatch, state queries, and hardware I/O into distinct execution paths.
- **HMI Web Interface:** React 18 + FastAPI bridge providing telemetry monitoring, real-time observability console, command ingress, jog pendant, and session management.
- **CI/CD Validation:** GitHub Actions with real tool invocations (colcon build/test, pytest, clang-tidy, vulture) and safety chain validation.

---

## System Architecture

The project enforces rigid separation of concerns. Raw LLM output is isolated from controller execution until it passes multiple stages of semantic and geometric validation.

```
User Intent
    │
    ▼
llm_gateway  ──► /validate_command ──► safety
                                           │
                                           ▼
                                      motion_core  ──► /hw_adapter/dispatch_trajectory
                                                               │
                                                               ▼
                                                    hw_adapter ──► Yaskawa YRC1000micro
```

### ROS 2 Packages

| Package             | Language | Responsibility |
|---------------------|----------|----------------|
| `interfaces`        | —        | Shared ROS contracts: `ExecuteMotion`, `DispatchTrajectory`, `ValidateCommand`, `GetCurrentPose`, etc. |
| `llm_gateway`       | Python   | Parses LLM payloads, validates against `llm_schema.yaml`, normalizes units/defaults, calls `/validate_command` and `/execute_motion`. |
| `safety`            | Python   | Exposes `/validate_command` service. Workspace bounds, forbidden zones, singularity checks, defense-in-depth before planning. |
| `motion_core`       | C++      | Owns `execute_motion` action server. Routes primitives to Pilz/OMPL planners, post-processes trajectories, dispatches to `hw_adapter`. |
| `hw_adapter`        | C++      | Owns `dispatch_trajectory` action server. Wraps MotoROS2 FollowJointTrajectory. Explicit `sim_mode` bypass for fake hardware. |
| `supervisor`        | C++      | Audit logging, execution monitoring, diagnostic publishing. |
| `primitives`        | C++      | Library of primitive implementations and dispatch helpers. |
| `jog_pendant`       | C++      | **Experimental** MoveIt Servo + MotoROS2 bridge for real-time joint jogging. NOT for production use. |
| `gp4_moveit_config` | —        | URDF/SRDF, planners, controllers, MoveIt launch/config. |
| `gp4_bringup`       | Python   | Composes sim vs hardware launches and the LLM stack. |

### HMI Stack

| Component                 | Description |
|---------------------------|-------------|
| `hmi/frontend`            | React 18 + Vite + TypeScript. Proxies `/api` and WebSocket to backend. |
| `hmi/backend/api`         | FastAPI app with REST endpoints and `/api/hmi/stream` WebSocket. |
| `hmi/backend/ros/`        | ROS bridge: `adapter.py` + `telemetry_snapshot`, `command_dispatch`, `jog_dispatch` mixins. |
| `hmi/backend/services/`   | `supervisor_service`, `jog_service`, `session_lock_service`, `telemetry_bridge_service`, `audit_service`. |

---

## Safety & Constraints

This system controls real industrial hardware. **Safety overrides speed and convenience.**

- **No Direct LLM Actuation:** Natural language is abstracted through a deterministic validation pipeline before any motion command reaches the controller.
- **Fail Closed:** Any parse failure, IK failure, collision warning, or hardware timeout aborts the active plan.
- **Conservative Defaults:** Velocity and acceleration are throttled regardless of user request unless explicitly verified.

### Current Safety Limits

| Parameter                        | Value             |
|----------------------------------|-------------------|
| Max velocity scale               | `0.06`            |
| Max acceleration scale           | `0.06`            |
| Max MOVE_REL translation         | `0.21 m`          |
| Workspace X                      | `[-0.45, 0.45] m` |
| Workspace Y                      | `[-0.16, 0.52] m` |
| Workspace Z                      | `[0.15, 0.65] m`  |
| HMI review velocity threshold    | `0.05`            |
| HMI review distance from HOME    | `> 0.30 m`        |
| Operational joint limits         | Per-joint, e.g. J5 ±1.80 rad (±103°), configurable via `safety_rules.yaml` |

Forbidden zones (pre-planning, 30 mm inflation):
- `front_wall_guard` — station front face at Y = −0.197 m
- `right_wall_guard` — station side wall at X = −0.482 m
- `floor_clearance_guard` — table/floor clearance Z < 0.20 m

### Safety Guard Configuration

Safety limits are loaded from `src/safety/config/safety_rules.yaml` via the `safety_rules_yaml_path` ROS parameter at node startup. This configuration drives:
- JointPositionGuard operational limits
- Workspace bounds
- Forbidden zone definitions
- Velocity/acceleration scaling

The `tools/validate_safety_chain.py` script validates constant synchronization between MOVE_REL workspace/forbidden-zone definitions in code and `safety_rules.yaml` at CI time.

### Hardware Validation Status

The full pipeline (HMI → ReviewIntent → safety → MoveIt → hw_adapter → YRC1000micro)
has been commissioned and is running on real hardware. Hardware telemetry validation
Phase A is confirmed (see `hmi/HARDWARE_TELEMETRY_VALIDATION.md`). Safety guards
(JointPositionGuard, ManipulabilityGuard, WristFlipGuard) are active in the hardware
execution path.

Remaining deferred items:
- `IO_SET` primitive — not yet validated on hardware
- Real TCP offset — not yet measured and approved

For D435i hand-eye calibration, refer to `docs/perception/d435i_hand_eye_calibration_runbook.md`.
Hand-eye calibration was performed on 2026-05-23 (12 samples, 2.614mm reprojection
error, PARK solver). Re-calibrate after any physical camera remount.

### Known Open Items

- **Perception Calibration:** D435i hand-eye calibration has been performed (2026-05-23, 12 samples, 2.614mm reprojection error). Re-calibrate if extrinsics drift or after any physical camera remount.

---

## Supported Primitives

13 public primitives fully integrated across the 4-tier pipeline. 
Source of truth for implementation logic: `src/primitives/`.

### Motion

| Primitive      | Description |
|----------------|-------------|
| `HOME`         | Move to safe factory default position. |
| `PTP`          | Point-to-Point joint configuration move (Pilz). |
| `LIN`          | Cartesian linear translation (Pilz). |
| `CIRC`         | Circular arc interpolation via an intermediate waypoint (Pilz). |
| `MOVE_REL`     | Relative Cartesian displacement from current TCP. |
| `MOVE_JOINT`   | Move a single joint to a target angle. |
| `MOVE_JOINTS`  | Multi-axis PTP configuration sequence. |

### Logic & Device

| Primitive      | Description |
|----------------|-------------|
| `WAIT`         | Blocking pause for a target duration in seconds. |
| `STOP`         | Immediate halt; cancels current goals. |
| `SET_SPEED`    | Apply dynamic velocity scalar to subsequent motions. |
| `GET_POSE`     | Query current Cartesian XYZ/RPY and joint positions. |
| `IO_SET`       | Deferred for hardware; do not treat MotoROS2 IO as validated yet. |
| `ALARM_RESET`  | Submit fault reset to active MotoROS2 driver. |

### Internal (not LLM-callable)

| Primitive          | Role |
|--------------------|------|
| `approach`         | Sub-primitive of blended_sequence. |
| `retract`          | Sub-primitive of blended_sequence. |
| `blended_sequence` | Chained primitive execution. |

---

## Prerequisites

- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill (desktop install)
- **Middleware:** `FastRTPS` (`rmw_fastrtps_cpp`)
- **Robot Controller:** YRC1000micro with latest MotoROS2 application file (`.out`)
- **Python:** 3.10+ (for `llm_gateway`, `safety`, HMI backend)
- **Node.js:** 18+ (for HMI frontend)
- **Dependencies:** `moveit2`, `ros2_control`, `motoros2_client_interface_dependencies`, `pilz_industrial_motion_planner`, `trac_ik`
- **Workspace dependency manifest:** `references/gp4_ws_dependencies.repos`

---

## Installation & Build

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Do not keep a project .venv active for ROS commands.
# ROS 2 Humble uses the system Python 3.10; an active venv can hide packages
# such as rclpy, scipy, or generated interfaces.
deactivate 2>/dev/null || true

# 3. Fetch pinned workspace dependency sources.
cd ~/gp4_ws
vcs import . < references/gp4_ws_dependencies.repos

# 4. Install ROS dependencies
rosdep install --from-paths src --ignore-src -y

# 5. Build active workspace packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 6. Source overlay
source install/setup.bash

# 7. HMI backend dependencies
pip3 install --user -r hmi/requirements.txt

# 7. HMI frontend
cd ~/gp4_ws/hmi/frontend && npm ci
```

> If `interfaces` changes, rebuild it first before rebuilding downstream packages:
> ```bash
> colcon build --packages-select interfaces && source install/setup.bash
> ```

---

## Getting Started

### Simulation (Fake Hardware)

```bash
source install/setup.bash
ros2 launch gp4_bringup sim.launch.py
```

### Physical Hardware

Make sure the YRC1000micro is in REMOTE mode, E-STOPs are cleared, and MotoROS2 is broadcasting.

```bash
source install/setup.bash
ros2 launch gp4_bringup hw.launch.py robot_ip:=192.168.1.33 agent_ip:=192.168.1.99
```

### HMI Web Interface

```bash
# Backend
source install/setup.bash
python3 -m uvicorn hmi.backend.api.app:app --host 127.0.0.1 --port 8000

# Frontend (dev)
cd ~/gp4_ws/hmi/frontend && npm run dev
```

### Camera Perception: Run D435i and Detect Objects

Use this flow to bring up the Intel RealSense D435i, confirm ROS 2 topics are live, and test object detection from point-cloud clustering. This is perception-only; it does not command robot motion.

#### 1. Terminal 1 — launch camera only

```bash
cd ~/gp4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

ros2 launch gp4_perception camera.launch.py \
  serial:=943222073917 \
  depth_profile:=848x480x30 \
  color_profile:=1280x720x30 \
  align_depth:=true \
  enable_sync:=true \
  pointcloud:=true
```

Keep this terminal running. Do not type topic names such as `/camera/color/image_raw` directly into Bash; topic names must be used through `ros2 topic ...` commands.

#### 2. Terminal 2 — verify camera topics and parameters

```bash
cd ~/gp4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

ros2 node list | rg "camera|realsense"
ros2 topic list | rg camera
ros2 topic info /camera/depth/color/points -v
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/color/points
ros2 topic echo /camera/color/camera_info --once
```

Expected topics:

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/color/points`

Expected QoS for RealSense image/depth streams: `BEST_EFFORT` reliability and `VOLATILE` durability. If `ros2 topic list | rg camera` prints nothing, the camera node is not running or the checking terminal is using a different `ROS_DOMAIN_ID` or RMW implementation.

#### 3. Terminal 1 — launch full perception stack for object detection

Stop the camera-only launch with `Ctrl+C`, then start the full stack:

```bash
ros2 launch gp4_perception perception_full.launch.py serial:=943222073917
```

This starts:

- `realsense2_camera_node` — publishes color, depth, aligned point cloud, and camera TF.
- `scene_processor` — subscribes to `/camera/depth/color/points` and `/camera/color/camera_info`, crops the workspace ROI, removes the dominant plane, clusters objects, and publishes detections.
- `tf_publisher` — publishes the configured camera-to-base transform.
- `detection_visualizer` — overlays 3D detections onto the color image and publishes `/perception/annotated_image`.

#### 4. Terminal 2 — observe detections

Place a rigid object on the visible table area, then run:

```bash
ros2 topic echo /perception/status --once
ros2 topic echo /perception/detections --once
ros2 topic hz /perception/detections
ros2 topic echo /perception/annotated_image --once
```

For RViz visualization:

```bash
rviz2 -d src/gp4_perception/config/perception.rviz
```

Detection output is `vision_msgs/msg/Detection3DArray` in `base_link`. The current detector is geometric: it clusters point-cloud objects and labels them by rough color and shape, for example `red_box`, `blue_sphere`, or `cylinder`.

#### 5. Troubleshooting quick checks

| Symptom | Check |
|---------|-------|
| `bash: /camera/...: No such file or directory` | A topic name was typed as a shell command. Use `ros2 topic echo /camera/...` or `ros2 topic hz /camera/...`. |
| `topic ... does not appear to be published yet` | Confirm the launch terminal is still running, then compare `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` in both terminals. |
| No D435i device appears | Run `rs-enumerate-devices` and `lsusb | rg "8086|Intel|RealSense"`; fix USB3 cable, power, or udev before debugging ROS. |
| `/camera/depth/color/points` missing | Launch with `pointcloud:=true`, `align_depth:=true`, and `enable_sync:=true`. |
| Detections are empty | Confirm the object lies inside `src/gp4_perception/config/perception.yaml` workspace ROI and that `tf_publisher` provides a transform to `base_link`. |
| Annotated image only says waiting for TF | Check `ros2 run tf2_ros tf2_echo camera_color_optical_frame base_link`. |

### Joint Jogging (Experimental)

```bash
ros2 launch jog_pendant jog_pendant_experimental.launch.py
```

---

## Example Commands

Use the HMI command interface or the `ReviewIntent` service path. The legacy
`gp4_cmd` helper CLI and direct raw-command topic path were removed during the
W8 cleanup; do not publish raw motion payloads directly from operator text.

```bash
# Start the command-capable sim stack first.
ros2 launch gp4_bringup sim.launch.py

# In another shell, start the HMI API and frontend.
python3 -m uvicorn hmi.backend.api.app:app --host 127.0.0.1 --port 8000
cd ~/gp4_ws/hmi/frontend && npm run dev
```

### E2E Testing

A manual full-pipeline simulation test script is available:

```bash
cd ~/gp4_ws
source install/setup.bash
python3 tools/e2e/test_full_pipeline.py
```

This script launches `sim.launch.py` and executes a test sequence:
`HOME -> GET_POSE -> MOVE_REL -> GET_POSE -> PTP` to validate the complete
motion pipeline in simulation.

---

## HMI Web Interface

The HMI provides a browser-based operator panel with:
- **Telemetry panel** — joint positions, TCP pose, robot status, gateway/LLM state
- **Command ingress** — submit natural-language text through `ReviewIntent` and the supervisor validation pipeline
- **Observability Console** — real-time command pipeline tracing, execution monitoring, and task validation
- **Jog pendant** — real-time joint jogging (requires `jog_pendant` stack running)
- **Session management** — operator session lock and audit trail

The HMI command path follows the same safety pipeline: `ValidateCommand → ExecuteMotion`. It does **not** bypass to MotoROS2 directly. Human confirmation is owned by the HMI supervisor lease/confirm flow before dispatch; the `ExecuteMotion` action no longer carries an approval flag.

Full spec: `hmi/README.md`
ROS interface inventory: `docs/hmi/HMI_ROS_INTERFACES.md`

---

## Configuration

| File | Purpose |
|------|---------|
| `motoros2_config.yaml` | MotoROS2 namespace, agent IP/port, joint names, QoS |
| `src/safety/config/safety_rules.yaml` | Workspace bounds, velocity caps, forbidden zones, operational joint limits |
| `src/llm_gateway/config/llm_schema.yaml` | Authoritative command schema |
| `src/gp4_moveit_config/config/kinematics.yaml` | TRAC-IK solver config |
| `src/gp4_bringup/config/scene_objects.yaml` | Collision objects in planning scene |

| Environment Variable | Description |
|----------------------|-------------|
| `GP4_LLM_API_KEY`    | API key for `llm_gateway` LLM backend |
| `RMW_IMPLEMENTATION` | Set to `rmw_fastrtps_cpp` for hardware launch |
| `ROS_DOMAIN_ID`      | Keep at `0` for this GP4 workspace |

## Testing & Validation

### Unit Tests

```bash
# Motion core tests (safety guards)
colcon test --packages-select motion_core

# Perception tests
colcon test --packages-select gp4_perception

# HMI backend tests
pytest hmi/backend -v
```

### Safety Chain Validation

```bash
python3 tools/validate_safety_chain.py
```

Validates:
- Perception calibration freshness and quality
- MOVE_REL workspace/forbidden-zone constant synchronization with `safety_rules.yaml`
- Safety guard configuration consistency

Recommended shell setup:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# GP4 workspace: isolate from other ROS2 stacks on same network
export ROS_DOMAIN_ID=0
```

---

*Research/thesis/demo system — not ISO 10218 production certified. Treat as real-hardware-adjacent at all times.*

## Implementation History

### W1: Safety Guards & CI/CD Hardening (COMPLETE)

Implemented multi-stage safety guard system:
- **JointPositionGuard:** 3-stage placement (A: pre-downsample in PrimitiveRouterDispatch, B: QualityGate, C: hw_adapter dispatch boundary)
- **ManipulabilityGuard:** Yoshikawa index via MoveIt Jacobian, wired into QualityGate Stage B
- **WristFlipGuard:** Extended with cumulative rotation tracking per joint
- **CI/CD:** Replaced stub jobs with real tool invocations (colcon build/test, pytest, clang-tidy, vulture)
- **Validation:** Extended `tools/validate_safety_chain.py` with MOVE_REL workspace/forbidden-zone constant sync checks
- **E2E Testing:** Added `tools/e2e/test_full_pipeline.py` for manual full-pipeline simulation testing
- **J5 Limit:** Widened to ±1.80 rad (±103°) per operator approval 2026-05-17, accommodating home pose J5=-1.602 rad
- **System commissioned:** Full pipeline running on real hardware (HMI → safety → MoveIt → hw_adapter → YRC1000micro)
