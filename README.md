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
- **Advanced Motion Core:** Collision-aware planning via MoveIt 2, TRAC-IK inverse kinematics, and smooth trajectory generation (TOTG + Ruckig). Execution logic is split into focused modules per primitive type.
- **Hardened Execution Pipeline:** Connects via MotoROS2 driver. Separates motion dispatch, state queries, and hardware I/O into distinct execution paths.
- **HMI Web Interface:** React 18 + FastAPI bridge providing telemetry monitoring, command ingress, jog pendant, and session management.

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
| Max MOVE_REL translation         | `0.05 m`          |
| Workspace X                      | `[-0.45, 0.45] m` |
| Workspace Y                      | `[-0.16, 0.52] m` |
| Workspace Z                      | `[0.23, 0.52] m`  |
| HMI review velocity threshold    | `0.05`            |
| HMI review distance from HOME    | `> 0.30 m`        |

Forbidden zones (pre-planning, 30 mm inflation):
- `front_wall_guard` — station front face at Y = −0.197 m
- `right_wall_guard` — station side wall at X = −0.482 m
- `floor_clearance_guard` — table/floor clearance Z < 0.20 m

---

## Supported Primitives

13 public primitives fully integrated across the 4-tier pipeline. Source of truth: `src/primitives/PRIMITIVE_SHORTLIST.md`.

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
| `IO_SET`       | Address PLC logical I/O blocks via MotoROS2. |
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

---

## Installation & Build

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Install ROS dependencies
cd ~/gp4_ws
rosdep install --from-paths src --ignore-src -y

# 3. Build active packages
colcon build --packages-select interfaces gp4_moveit_config safety motion_core primitives hw_adapter llm_gateway supervisor jog_pendant gp4_bringup --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. Source overlay
source install/setup.bash

# 5. HMI backend dependencies
pip3 install --user -r hmi/requirements.txt

# 6. HMI frontend
cd ~/gp4_ws/hmi/frontend && npm install
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

### Joint Jogging (Experimental)

```bash
ros2 launch jog_pendant jog_pendant_experimental.launch.py
```

---

## Example Commands

Use the helper CLI — it validates against the installed schema and prints the payload before publishing.

```bash
# Move all joints to zero
ros2 run llm_gateway gp4_cmd move-joints 0 0 0 0 0 0 --speed 0.05

# Relative move in base frame
ros2 run llm_gateway gp4_cmd move-rel --z -0.03 --speed 0.05

# Linear move to a pose
ros2 run llm_gateway gp4_cmd lin --xyz 0.30 0.10 0.42 --rpy 180 0 0 --speed 0.05

# HOME / STOP / WAIT
ros2 run llm_gateway gp4_cmd home --speed 0.05
ros2 run llm_gateway gp4_cmd stop
ros2 run llm_gateway gp4_cmd wait 3

# Send directly as ExecuteMotion action (bypass topic)
ros2 run llm_gateway gp4_cmd --transport action home --speed 0.05

# Load from YAML file
cat <<'YAML' > /tmp/move_rel.yaml
primitive_type: MOVE_REL
delta_z: -0.03
velocity_scale: 0.05
YAML
ros2 run llm_gateway gp4_cmd from-file /tmp/move_rel.yaml
```

---

## HMI Web Interface

The HMI provides a browser-based operator panel with:
- **Telemetry panel** — joint positions, robot status, gateway/LLM state
- **Command ingress** — submit structured commands through the supervisor validation pipeline
- **Jog pendant** — real-time joint jogging (requires `jog_pendant` stack running)
- **Session management** — operator session lock and audit trail

The HMI command path follows the same safety pipeline: `ValidateCommand → ExecuteMotion`. It does **not** bypass to MotoROS2 directly. Human confirmation is owned by the HMI/supervisor before dispatch; `ExecuteMotion.require_approval` is a deprecated wire-compatibility field and confirmed commands are dispatched with `require_approval=false`.

Full spec: `hmi/HMI_V2_COMMAND_INGRESS.md`

---

## Configuration

| File | Purpose |
|------|---------|
| `motoros2_config.yaml` | MotoROS2 namespace, agent IP/port, joint names, QoS |
| `src/safety/config/safety_rules.yaml` | Workspace bounds, velocity caps, forbidden zones |
| `src/llm_gateway/config/llm_schema.yaml` | Authoritative command schema |
| `src/gp4_moveit_config/config/kinematics.yaml` | TRAC-IK solver config |
| `src/gp4_bringup/config/scene_objects.yaml` | Collision objects in planning scene |

| Environment Variable | Description |
|----------------------|-------------|
| `GP4_LLM_API_KEY`    | API key for `llm_gateway` LLM backend |
| `GP4_LLM_ENV_FILE`   | Local `.env` file path, usually `/home/hieu2/gp4_ws/.env` |
| `RMW_IMPLEMENTATION` | Set to `rmw_fastrtps_cpp` for hardware launch |
| `ROS_DOMAIN_ID`      | Keep at `39` for this GP4 workspace to isolate it from other ROS 2 stacks |

Recommended shell setup:

```bash
export GP4_LLM_ENV_FILE=/home/hieu2/gp4_ws/.env
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# GP4 workspace: isolate from other ROS2 stacks on same network
export ROS_DOMAIN_ID=39
```

---

*Research/thesis/demo system — not ISO 10218 production certified. Treat as real-hardware-adjacent at all times.*
