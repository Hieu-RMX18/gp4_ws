# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
More details about catalog robot and offical reposyaskawa check `home/hieu2/gp4_ws/references/`.
Before fixing code, trace the existing data flow and prefer replacing or removing the current owner logic over layering parallel logic on top.

## Workspace snapshot
- ROS 2 Humble / Ubuntu 22.04 colcon workspace for a Yaskawa Motoman GP4 with YRC1000micro via MotoROS2.
- Main MoveIt planning group: `gp4_arm`.
- MotoROS2 namespace: `/yaskawa/*` (configured in `motoros2_config.yaml`).
- Research/thesis/demo system — NOT ISO 10218 production. Treat as real-hardware-adjacent at all times.

## Read first
- `README.md` — full workspace overview, safety limits, primitives list, and setup.
- `motoros2_config.yaml` — controller namespace, agent IP/port, joint names, QoS.
- `.claude/CLAUDE.md` — project-local Claude rules/skills index.
- `docs/archive/` — archived historical plans (do not treat as current).

## Common commands
Run from `~/gp4_ws`.

### Environment
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export GP4_LLM_ENV_FILE=/home/hieu2/gp4_ws/.env
```

### Build
```bash
colcon build --packages-select interfaces gp4_moveit_config gp4_station safety motion_core primitives hw_adapter llm_gateway supervisor jog_pendant gp4_bringup --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If `interfaces` changes, rebuild it first then re-source:
```bash
colcon build --packages-select interfaces && source install/setup.bash
```

### Tests
C++ packages (GoogleTest):
```bash
colcon test --packages-select motion_core --output-on-failure
colcon test-result --packages-select motion_core --verbose
```

Python packages (pytest):
```bash
colcon test --packages-select safety --output-on-failure
colcon test-result --packages-select safety --verbose
# Or directly:
cd src/safety && python -m pytest tests/ -v
cd src/llm_gateway && python -m pytest tests/ -v
```

### Launch
```bash
source install/setup.bash

# Simulation (self-contained, no prerequisites)
ros2 launch gp4_bringup sim.launch.py

# Real hardware (requires micro-ROS Agent on UDP 8888, RMW_IMPLEMENTATION set)
ros2 launch gp4_bringup hw.launch.py robot_ip:=192.168.1.33 agent_ip:=192.168.1.99

# MoveIt-only (config debugging)
ros2 launch gp4_bringup moveit_only.launch.py

# LLM stack only (gateway + safety, no motion)
ros2 launch gp4_bringup llm_stack.launch.py
```

### CLI tool
```bash
ros2 run llm_gateway gp4_cmd home --speed 0.05
ros2 run llm_gateway gp4_cmd lin --xyz 0.30 0.10 0.42 --rpy 180 0 0 --speed 0.05
ros2 run llm_gateway gp4_cmd move-rel --z -0.03 --speed 0.05
ros2 run llm_gateway gp4_cmd move-joints 0 0 0 0 0 0 --speed 0.05
ros2 run llm_gateway gp4_cmd from-file /tmp/command.yaml
ros2 run llm_gateway gp4_cmd --transport action home --speed 0.05  # bypass topic, use action
```

### HMI
```bash
# Backend
source install/setup.bash
python3 -m uvicorn hmi.backend.api.app:app --host 127.0.0.1 --port 8000

# Frontend (dev)
cd ~/gp4_ws/hmi/frontend && npm run dev
```

## Architecture

### Execution pipeline

```
User Intent → llm_gateway (TaskCompiler) → task_runtime → /execute_motion → motion_core → /hw_adapter/dispatch_trajectory → hw_adapter → MotoROS2 → Robot
```

Each layer is strictly separated: LLM parsing into FactoryTask, grounding, runtime execution, motion planning, hardware dispatch. `llm_gateway` now only routes through `task_runtime` (no legacy branching).

### Active packages

| Package | Lang | Role |
|---------|------|------|
| `interfaces` | — | Shared ROS contracts (`ExecuteMotion`, `DispatchTrajectory`, `ValidateCommand`, `GetCurrentPose`). Build first when changed. |
| `llm_gateway` | Python | LLM intent parsing, schema validation, unit normalization, intent routing. Owns `gp4_cmd` CLI. |
| `safety` | Python | Fail-closed safety gate: workspace bounds, forbidden zones, velocity caps, MOVE_REL delta checks. |
| `motion_core` | C++ | `ExecuteMotion` action server. Routes primitives to Pilz/OMPL planners via MoveIt2, post-processes trajectories. |
| `primitives` | C++ | Primitive planning library (PTP, LIN, CIRC, HOME, approach/retract, blended sequences). |
| `hw_adapter` | C++ | `DispatchTrajectory` action server. MotoROS2 bridge, start-state validation, session management, error recovery. |
| `supervisor` | C++ | Audit logging (rosbag2 + JSONL), execution monitoring, diagnostics publishing. |
| `jog_pendant` | C++ | **Experimental** MoveIt Servo jog bridge. Not for production. |
| `gp4_moveit_config` | — | URDF/SRDF, Pilz/OMPL/CHOMP planners, TRAC-IK kinematics, joint limits, controllers. |
| `gp4_station` | — | Station URDF/xacro, workcell collision meshes. |
| `gp4_bringup` | Python | Launch composition (sim, hw, llm_stack, moveit_only). |

### Key ROS2 interfaces

| Interface | Type | Server → Client |
|-----------|------|-----------------|
| `/execute_motion` | Action | motion_core ← llm_gateway |
| `/hw_adapter/dispatch_trajectory` | Action | hw_adapter ← motion_core |
| `/follow_joint_trajectory` | Action | MotoROS2 ← hw_adapter |
| `/validate_command` | Service | safety ← llm_gateway |
| `/get_current_pose` | Service | motion_core ← llm_gateway |
| `/yaskawa/robot_status` | Topic | MotoROS2 → safety, hw_adapter |
| `/joint_states` | Topic | MotoROS2 → hw_adapter |

### Vendor packages (do not modify)
- `src/motoman_ros2_support_packages`, `src/yaskawa-global`, `src/ros-industrial`, `trac_ik/`
- `src/motoros2_client_interface_dependencies`

### HMI stack
- `hmi/frontend/` — React 18 + Vite + TypeScript
- `hmi/backend/api/` — FastAPI REST + WebSocket (`/api/hmi/stream`)
- `hmi/backend/ros/` — ROS2 adapter with telemetry, command dispatch, jog dispatch
- `hmi/backend/services/` — supervisor, jog, session lock, telemetry bridge, audit

## Key conventions
- **Joint names**: `joint_1_s`, `joint_2_l`, `joint_3_u`, `joint_4_r`, `joint_5_b`, `joint_6_t`.
- **Conservative defaults**: velocity_scale=0.06, acceleration_scale=0.06.
- **Safety config**: `src/safety/config/safety_rules.yaml` — workspace bounds, forbidden zones, velocity caps.
- **LLM schema**: `src/llm_gateway/config/llm_schema.yaml` — authoritative command schema (13 public primitives).
- **Conventional commits**: `feat:`, `fix:`, `refactor:`, `docs:`, `test:`, `chore:` with optional `(scope)`.
- **No hallucinated APIs** — read actual config/code before assuming any topic, service, or action exists.
- **Master plan**: Written in Vietnamese. Translate relevant sections when referencing.

## Key config files

| File | Purpose |
|------|---------|
| `motoros2_config.yaml` | MotoROS2 namespace, agent IP/port, joint names, QoS |
| `src/safety/config/safety_rules.yaml` | Workspace bounds, velocity caps, forbidden zones |
| `src/llm_gateway/config/llm_schema.yaml` | Authoritative JSON command schema |
| `src/gp4_moveit_config/config/kinematics.yaml` | TRAC-IK solver (0.05s timeout, 3 attempts) |
| `src/gp4_moveit_config/config/joint_limits.yaml` | Per-joint velocity/acceleration/jerk limits |
| `src/gp4_moveit_config/config/pilz_cartesian_limits.yaml` | Cartesian velocity/acceleration limits |
| `src/gp4_bringup/config/scene_objects.yaml` | Collision objects in MoveIt planning scene |

<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **gp4_ws** (56221 symbols, 98890 relationships, 300 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

> If any GitNexus tool warns the index is stale, run `npx gitnexus analyze` in terminal first.

## Always Do

- **MUST run impact analysis before editing any symbol.** Before modifying a function, class, or method, run `gitnexus_impact({target: "symbolName", direction: "upstream"})` and report the blast radius (direct callers, affected processes, risk level) to the user.
- **MUST run `gitnexus_detect_changes()` before committing** to verify your changes only affect expected symbols and execution flows.
- **MUST warn the user** if impact analysis returns HIGH or CRITICAL risk before proceeding with edits.
- When exploring unfamiliar code, use `gitnexus_query({query: "concept"})` to find execution flows instead of grepping. It returns process-grouped results ranked by relevance.
- When you need full context on a specific symbol — callers, callees, which execution flows it participates in — use `gitnexus_context({name: "symbolName"})`.

## Never Do

- NEVER edit a function, class, or method without first running `gitnexus_impact` on it.
- NEVER ignore HIGH or CRITICAL risk warnings from impact analysis.
- NEVER rename symbols with find-and-replace — use `gitnexus_rename` which understands the call graph.
- NEVER commit changes without running `gitnexus_detect_changes()` to check affected scope.

## Resources

| Resource | Use for |
|----------|---------|
| `gitnexus://repo/gp4_ws/context` | Codebase overview, check index freshness |
| `gitnexus://repo/gp4_ws/clusters` | All functional areas |
| `gitnexus://repo/gp4_ws/processes` | All execution flows |
| `gitnexus://repo/gp4_ws/process/{name}` | Step-by-step execution trace |

## CLI

| Task | Read this skill file |
|------|---------------------|
| Understand architecture / "How does X work?" | `.claude/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| Blast radius / "What breaks if I change X?" | `.claude/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| Trace bugs / "Why is X failing?" | `.claude/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / refactor | `.claude/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| Tools, resources, schema reference | `.claude/skills/gitnexus/gitnexus-guide/SKILL.md` |
| Index, status, clean, wiki CLI commands | `.claude/skills/gitnexus/gitnexus-cli/SKILL.md` |

<!-- gitnexus:end -->
