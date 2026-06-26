# Yaskawa GP4 / YRC1000micro — ROS 2 Agentic Robot Control

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Humble-green.svg)](https://moveit.ros.org/)
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
11. [Testing & Validation](#testing--validation)

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

## Supported Primitives

13 public primitives are represented across the 4-tier pipeline; hardware validation status is called out where it differs.
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

### ReAct semantic pick/place

The gateway resolves pick/place goals through `src/llm_gateway/config/station_semantic_map.yaml`.

**Station semantic map** — Defines named regions (`conveyor`, `fixture`), zone offsets, object class aliases, and approach axes. Geometry values that have not been physically measured use `VERIFY_CONFIG`. The map is loaded at runtime; any field still containing `VERIFY_CONFIG` causes planning to fail closed with `verify_config_required`. This file is separate from `safety_rules.yaml` and must not be merged with it.

**Composite pick/place commands** — The agent uses `compile_goal` to resolve `pick_and_place` DSL into an ordered skill sequence: `refresh_scene` → `approach_object` → `pick_object` → `place_object` → `verify_postcondition`. Each composite tool emits a validated Semantic IR `sequence` that still passes through `/validate_command`, `motion_core`, supervisor gates, and the hardware adapter. Composite picks and places each count as one motion iteration against the ReAct budget.

**Scene cache** — Perception results are cached for 2 s, keyed by `(class_filter, frame)`. The cache is invalidated by `refresh_scene`, by any non-IDLE robot state, and when a motion tool's result metadata contains `tool_changed_world=True`. Cache hits are observable via `payload.cache_hit=True` in `QueryPerceptionTool` results.

**Gripper verification** — Gripper open/close/verify fail closed when any I/O address or value in `safety_rules.yaml` still contains `VERIFY_CONFIG`, when the robot is not `IDLE`, or when the `WriteSingleIO`/`ReadSingleIO` ROS services are unavailable. Fill verified I/O config values before hardware use.

**MTC optional path** — Composite tools query `mtc_select()` to choose between an MTC-backed pick/place path and the validated primitive sequence path. MTC is selected only when the MTC service is ready and all required inputs (object pose, destination, gripper config) are verified. If MTC is unavailable, the primitive sequence path is used. If both are unavailable, the tool returns `capability_unavailable`.

**Closed-loop postcondition** — After a world-changing action, the agent calls `verify_postcondition` to query perception and confirm the object is in the expected destination region. If verification fails, the agent has one repair attempt (`max_repair=1`) before the sequence halts with a structured `postcondition_failed` error.

---

### FactoryTask task planning

The ReAct agent produces **FactoryTask** JSON as its final output, not raw Semantic IR. Semantic IR remains an internal compiler artifact only.

| Component | File | Role |
|-----------|------|------|
| `FactoryTask` / `TaskNode` | `factory_task.py` | LLM-facing task tree contract (sequence, skill, repeat, for_each, retry, fallback, observe, wait_until) |
| `WorldModel` | `factory_task.py` | Grounding layer; provides object poses and collections from live perception cache |
| `PolicyEngine` | `factory_task.py` | Records a visible decision (`allow`, retry, fallback, replan) for every node at plan time |
| `TaskCompiler` | `factory_task.py` | Translates static FactoryTask trees to guarded Semantic IR for supervisor review; rejects runtime-control nodes with a clear error |
| `TaskRuntime` | `factory_task.py` | Executes FactoryTask control flow (retry, fallback, replan) via an injected skill executor; never bypasses safety validation |
| `CompiledTask` | `factory_task.py` | Carries the resulting Semantic IR, the runtime plan, and the list of policy decisions for HMI display |

**LLM output contract** — The system prompt instructs the model to emit a single FactoryTask JSON object. Final responses containing raw `intent` / Semantic IR are rejected and trigger a repair attempt. FactoryTask skill nodes such as `go_home`, `move_named_pose`, `pick_object`, `place_object`, `verify_scene`, `draw_shape`, and `draw_text` are the only valid skill names.

**Runtime-control path** — When the FactoryTask root is a runtime-control node (`retry`, `fallback`, `repeat`, `for_each`), `TaskCompiler` raises `FactoryTaskError`. The gateway then returns a runtime-execution sentinel IR that carries the full task tree and visible policy decisions in `metadata.runtime_plan` / `metadata.policy_decisions`. The HMI `planSummary` exposes `factoryTask`, `factoryTaskRuntimePlan`, and `factoryTaskPolicyDecisions` keys so operators can inspect the plan before confirming.

**Replan policy** — FactoryTask payloads may include `replan_policy.max_replans` to control how many replan cycles `TaskRuntime` will attempt when a skill returns `requests_replan=True`. The default is 1 attempt; failing that, execution halts and a `TaskRuntimeReport` is returned with all policy decisions recorded.

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

# 8. HMI frontend
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

Perception-only — does not command robot motion. Full runbook: [`src/gp4_perception/README.md`](src/gp4_perception/README.md).
```bash 
# Calib
cd /home/hieu2/gp4_ws
source install/setup.bash
ros2 launch gp4_perception calibration_collect.launch.py serial:=943222073917

```

```bash
# Camera only
ros2 launch gp4_perception camera.launch.py serial:=943222073917

# Full perception stack (camera + unified visualization)
source install/setup.bash
ros2 launch gp4_perception perception_full.launch.py serial:=943222073917
```

`perception_full.launch.py` starts:

- `realsense2_camera_node` — color, depth, aligned point cloud, camera TF
- `scene_processor` — ROI crop, plane removal, Euclidean clustering, MoveIt collision objects, `/perception/status`
- `unified_visualizer` — RGB+depth HSV contour detection, temporal tracking, preprocessing dashboard, publishes `/perception/detections` (Detection3DArray), `/perception/annotated_image`, and debug topics
- `calibration_service` / `tf_publisher` — hand-eye extrinsics and camera→base_link TF

Key output topics:

| Topic | Publisher | Type |
|-------|-----------|------|
| `/perception/detections` | `unified_visualizer` | Detection3DArray |
| `/perception/annotated_image` | `unified_visualizer` | Image |
| `/perception/debug_dashboard_image` | `unified_visualizer` | Image |
| `/perception/zoom_roi_image` | `unified_visualizer` | Image |
| `/perception/status` | `scene_processor` | String |

```bash
rviz2 -d src/gp4_perception/config/perception.rviz
```

### Joint Jogging (Experimental)

```bash
ros2 launch jog_pendant jog_pendant_experimental.launch.py
```

---

## Example Commands

Use the HMI command interface or the `/llm_gateway/review_intent` service path.

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
