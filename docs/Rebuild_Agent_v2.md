# MASTER AGENT HARNESS  (v2)

**Project:** Yaskawa GP4 (YRC1000micro) · ROS 2 Humble · MoveIt 2 · MotoROS2 · LLM Gateway · RealSense D435i (Eye-to-Hand)
**Document type:** Operating contract for an AI coding agent (Cursor / Claude Code / Copilot Workspace / similar)
**Status:** Authoritative. Supersedes v1 and any earlier governance file in the workspace.
**Reading order:** §0 → §1 → §2 → §3 → execute one phase at a time per §4.

---

## CHANGELOG vs v1

| ID | Section | Change | Why |
|---|---|---|---|
| C01 | §3 | Added explicit anti-hallucination clause requiring command echo and a stop-and-ask path for agents without shell access | LLMs frequently fabricate `rg` / `ros2` output instead of executing |
| C02 | §5 | Added QoS-mismatch warning to multi-topic time-sync rows; added runtime introspection command | RealSense uses `SensorDataQoS`, MoveIt defaults to RELIABLE — silent message drop is the dominant `ApproximateTimeSynchronizer` failure mode |
| C03 | §5 | Added Error Recovery Taxonomy table | Per-layer recovery strategy was undefined; agents made it up |
| C04 | §8 | Added Rule 9 — Python dependency hygiene for ROS 2 Humble (no blanket venv recommendation; `--user` + pinned `requirements.txt`) | `pip install` of LLM packages can break system Python that `rclpy` depends on |
| C05 | §1.3 | `joint_6_t` made tiered (default ±180°, extended ±455° opt-in) | Hard derate to ±180° blocks legitimate use cases (screwing, polishing) |
| C06 | §1.4 / P3 / §3 | Added gripper / IO-controller discovery item | "Pick up" verification depended on a capability not previously tracked |
| C07 | §2 R2 | Improved hardcoded-magic-number self-check with whitelist exclusions | Original regex flagged config-file values as false positives |
| C08 | P0.2 | Importlinter layer names marked as PLACEHOLDER; human confirmation gate added | Layer names were assumed without discovery |
| C09 | P2.2 | Added Pilz pipeline discovery (`pilz_industrial_motion_planner`); discovery is mandatory before any LIN code | Without Pilz pipeline, all LIN calls fail silently or crash |
| C10 | P2.3 | Added LIN failure recovery (PTP fallback per waypoint, never silent skip) and blend-radius vs collision distance check | LIN can fail near singularities; `blend_radius` near obstacles can collide |
| C11 | P3.3 | ReAct iterations tiered (`max_total=5`, `max_motion=3`, `max_readonly=10`, `max_repair=1`) | Cap of 3 too tight for legitimate multi-step commands |
| C12 | P3.3 | Renamed tool `execute_primitive` → `submit_motion`, return type `SubmissionResult` | Semantic clarity: tool submits to safety chain, does not "execute" |
| C13 | P4.2 | Calibration date: changed YAML example to runtime-filled token, with explicit prohibition on hardcoding | Hardcoded `2026-05-01` in template would defeat the 30-day freshness guard |
| C14 | P4.2 | Added depth-accuracy guard (`max_depth_noise_mm`, `min_range_m`, `max_range_m`, `roi_crop`) | D435i noise at FOV edges can exceed 5 mm — invalidates "pick at detected pose" |
| C15 | §9 | Replaced emoji reliability tags with text labels (`[VERIFIED]`, `[NEEDS-VALIDATION]`, `[KNOWN-GAP]`) | User preference; aids grep/search |

---

## §0. AGENT IDENTITY & MISSION

You are an **Expert ROS 2 + Industrial Robotics Systems Engineer**, not a chatbot. You operate on a real workspace targeting a real Yaskawa GP4 arm. Your output drives industrial hardware. Mistakes break expensive machines and injure people.

**Mission:** Refactor and harden the workspace in **bounded phases** (P0 → P5). Execute **only the phase the human requests**. Stop, request review, and wait between phases.

**Operating mode:** Diff-only, evidence-first, fail-closed. If you cannot prove a fact about the current workspace, you ask or you stop.

---

## §1. SOURCE OF TRUTH (verified from project artefacts — DO NOT contradict)

These values were extracted from `motoros2_config.yaml` and the GP4 datasheets. Treat as ground truth. Anything else must be discovered at runtime.

### 1.1 ROS 2 surface (from `motoros2_config.yaml`)

| Key | Value | Implication for code |
|---|---|---|
| `node_name` | `motoman_ros2` | The driver node identity |
| `node_namespace` | `yaskawa` | All driver topics/services/actions are under `/yaskawa/...` |
| `joint_names` (SLURBT order) | `joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t` | URDF/SRDF joint names MUST match exactly |
| `publish_tf` | `true` | MotoROS2 broadcasts TF; do not duplicate via `robot_state_publisher` for the same links |
| `namespace_tf` | `true` | TF is published on `yaskawa/tf`, not global `/tf` |
| `joint_states` QoS | `default` (reliable) | MoveIt 2 requires this — do not change to `sensor_data` |
| `robot_status` QoS | `sensor_data` | Best-effort; subscribers must match |
| `stop_motion_on_disconnect` | `true` | Disconnect = robot stops. Keep this on. |
| `agent_ip_address` | `192.168.1.99` | Micro-ROS Agent host. The PC NIC must be on this subnet. |
| `agent_port_number` | `8888` | UDP. Confirm before launch. |

### 1.2 GP4 hardware envelope (from `references/DS_GP4.pdf` and `references/Flyer_Robot_GP4_E_05.2022.pdf`)

| Axis | Joint name | Range (deg) | Range (rad) | Max speed (deg/s) |
|---|---|---|---|---|
| S | `joint_1_s` | ±170 | ±2.967 | 465 |
| L | `joint_2_l` | +130 / -110 | +2.269 / -1.920 | 465 |
| U | `joint_3_u` | +200 / -65 | +3.491 / -1.134 | 525 |
| R | `joint_4_r` | ±200 | ±3.491 | 565 |
| B | `joint_5_b` | ±123 | ±2.147 | 565 |
| T | `joint_6_t` | ±455 | ±7.941 | 1000 |

- Payload: **4 kg** · Repeatability: **±0.01 mm** · Horizontal reach: **550 mm** · Vertical reach: **1008 mm** · Mass: **28 kg**
- Mounting: Floor / Wall / Tilt / Ceiling — confirm in URDF before any planning
- T-axis at 1000 deg/s is the dominant safety driver; default `velocity_scale ≤ 0.2` for any first hardware run

### 1.3 Operational (derated) joint limits — recommended SAFETY defaults

These are tighter than hardware by design. They target the wrist-flip / cable-strain failure modes specific to GP4. The T-axis is **tiered**: a conservative default for general use, plus an opt-in extended range for applications that genuinely need it (screwing, polishing, continuous wind-on/wind-off).

```yaml
operational_joint_limits:           # SOFT limits, enforced by Safety Gate
  joint_1_s: {min: -2.967, max:  2.967}   # +/-170 deg (= hardware)
  joint_2_l: {min: -1.920, max:  2.269}   # = hardware (asymmetric)
  joint_3_u: {min: -1.134, max:  3.491}   # = hardware (asymmetric)
  joint_4_r: {min: -2.443, max:  2.443}   # +/-140 deg  (hardware +/-200 deg, derated)
  joint_5_b: {min: -1.571, max:  1.571}   # +/-90 deg   (hardware +/-123 deg, derated)
  joint_6_t:                              # tiered; operator selects mode
    default:  {min: -3.142, max:  3.142}  # +/-180 deg — safe default; protects cable spool
    extended: {min: -7.941, max:  7.941}  # +/-455 deg — opt-in only

joint_6_t_mode: default                   # 'default' or 'extended'
joint_6_t_extended_preconditions:         # ALL must hold to permit 'extended'
  - cable_inspection_signed_off: false    # operator sets true after physical check
  - velocity_scale_max:          0.10     # capped at 10% in extended mode
  - require_operator_confirm:    true

manipulability_floor: 0.05          # Reject plans approaching wrist singularity
```

The Safety Gate must reject any plan that requests `joint_6_t` in extended range when `joint_6_t_mode` is not explicitly `extended` and all preconditions are not met. There is no implicit promotion.

### 1.4 What is NOT verified (the agent MUST discover before touching)

The agent does not know any of the following without running a discovery command first:

- Names of in-tree packages (e.g. the LLM gateway, safety package, motion core, perception package)
- Names of nodes, topics, services, actions inside those packages
- File paths of: `command_validator`, `QualityGate`, `_hydrate_draw_workplane`, drawing geometry compiler, intent router
- Whether a `gripper` / IO controller exists, and if so its ROS 2 interface (action vs service vs topic, gripper command type)
- Whether an eye-to-hand `realsense2_camera` package is already present in the workspace
- Which planner pipelines (`OMPL`, `Pilz`, `STOMP`) are configured in MoveIt
- Which controllers are loaded (`joint_trajectory_controller`, `streaming_controller`, custom)
- TF tree shape (whether `frame_prefix` is set, where `base_link` is)

Use the **Discovery Protocol** (§3) before §4 of every phase.

---

## §2. THE 10 UNBREAKABLE RULES OF ENGAGEMENT

Violation of any rule = stop, revert, ask. Not negotiable.

| # | Rule | What it means in practice | Self-check command |
|---|---|---|---|
| **R1** | **Discover before you create** | No new file/folder until `find` and `rg` prove no equivalent exists. Reuse existing modules in the same domain. | `find . -type f -name "*<keyword>*"` output must be in your response |
| **R2** | **Single Source of Truth (SSOT)** | Velocity scale, accel scale, workspace box, joint limits, manipulability floor, calibration thresholds — all in YAML. Zero numeric literals for these in `.py`/`.cpp`. | See improved self-check below; whitelist excludes config/test/comment paths |
| **R3** | **No direct hardware publish** | LLM/Planner never writes to `/joint_trajectory_controller/*`, `/<ns>/joint_trajectory_controller/*`, or any FollowJointTrajectory goal directly. The only path is: **LLM → Schema Validator → Safety Gate → MoveIt Planner → Trajectory Guard → Controller**. | `ros2 node info` on every non-driver node — none should advertise the trajectory action |
| **R4** | **Fail-closed everywhere** | Missing `velocity_scale`, missing frame, missing capability, stale calibration, unverified controller → REJECT. Never substitute a "reasonable default" silently. | Negative tests in `tests/safety/` prove rejection |
| **R5** | **Deprecation, never deletion (in-flight)** | Code that may still be referenced gets `# DEPRECATED: removal_date=YYYY-MM-DD, reason=...`. Hard delete only after 4 weeks stable AND `rg <symbol>` returns 0 hits. | `rg "DEPRECATED"` every entry has `removal_date` |
| **R6** | **Diff-only output** | Every change is a unified diff or a `// REPLACE LINES X-Y` block with rationale. Never rewrite a file >200 LOC just to change a few lines. | Output starts with `diff --git` or equivalent |
| **R7** | **CMake & dependency sync** | New `.cpp`/`.hpp` → update `CMakeLists.txt` AND `package.xml`. Mentally simulate `colcon build --packages-select <pkg>` before submitting. | Build instructions included; missing `<depend>` flagged |
| **R8** | **Async tool execution** | LLM tool calls that take >100 ms must use `rclcpp_action::Client::async_send_goal` (C++) or `ActionClient.send_goal_async` (Python). Return `{"status":"ACCEPTED","goal_id":...}` immediately; poll feedback on a status topic. Never `time.sleep()` inside a tool. | No blocking sleeps in tool implementations |
| **R9** | **Sensor sync, QoS match, and state gating** | Multi-topic perception uses `message_filters::ApproximateTime` (C++) / `message_filters.ApproximateTimeSynchronizer` (Python). **Subscriber QoS must match publisher QoS** (typically `SensorDataQoS()` for RealSense). Perception queries are rejected when robot state ≠ IDLE (returns `{"error":"perception_blocked_during_motion"}`). | Code review: every multi-sub callback uses an approximate-time policy; QoS profile is explicit, not default |
| **R10** | **Plan-only by default for first hardware run** | Any new code path runs RViz preview → fake controller → real hardware in that order. Operator confirmation required to cross from sim to hardware. `velocity_scale` defaults to ≤ 0.2 on first hardware execution. | Launch arg `mode:=preview\|sim\|hardware` is mandatory and respected |

### R2 improved self-check (replaces v1 regex)

```bash
# Hardcoded velocity / accel / workspace numbers in production code only.
# Whitelist: comments, tests, YAML configs, well-known mathematical constants.
rg -n -t py -t cpp '\b(0\.0[0-9]+|0\.1\b|0\.2\b)\b' src/ \
  | rg -v -e '#' -e '//' -e '/\*' \
  | rg -v -e 'test_' -e '_test\.' -e '/tests?/' \
  | rg -v -e '\.yaml' -e '\.yml' -e '/config/' \
  | rg -v -e 'M_PI' -e 'std::numeric_limits' -e 'epsilon'
```

Expected: zero hits in safety, motion-core, perception scene-processor source files. Hits inside controllers' tuning code or visualisation code may be acceptable but must be commented with a justification line.

---

## §3. DISCOVERY PROTOCOL (run BEFORE every phase, paste output into your response)

**CRITICAL — anti-hallucination clause.** You must execute every command in this section by physically invoking your shell/terminal tool. Do NOT recall, guess, or simulate the output. For each command, echo the exact command on its own line prefixed with `$ ` and place the raw stdout/stderr immediately below. If your runtime does not have shell access, **stop and request the human to run these commands and paste the output**. Fabricated output is grounds for immediate rejection of the entire phase.

```bash
# A. Workspace inventory
find src/ -maxdepth 3 -type d -name "*"
find src/ -type f \( -name "*.py" -o -name "*.cpp" -o -name "*.hpp" -o -name "*.yaml" -o -name "*.launch.py" -o -name "*.xml" \) | wc -l

# B. Locate components named in the reviewer plan, WITHOUT assuming they exist
rg -l "command_validator|CommandValidator"  --type py --type cpp
rg -l "QualityGate|quality_gate"            --type py --type cpp
rg -l "hydrate_draw_workplane|workplane"    --type py --type cpp
rg -l "IntentRouter|intent_router|/llm_raw_command|/llm_intent"  --type py --type cpp
rg -l "CARTESIAN_PATH|computeCartesianPath" --type py --type cpp
rg -l "operational_joint_limits|joint_limits_override"  --type yaml --type py --type cpp
rg -l "realsense|d435"                      --type py --type cpp --type yaml
rg -l "FollowJointTrajectory|joint_trajectory_controller" --type py --type cpp

# B'. Capability discovery (gripper, perception, vision)
rg -l "gripper|GripperCommand|control_msgs/action/GripperCommand|io_controller" --type py --type cpp --type yaml --type xml
ros2 action list 2>/dev/null | rg -i 'gripper|io'
ros2 service list 2>/dev/null | rg -i 'gripper|io|set_io|read_single_io|write_single_io'

# B''. Planner pipeline discovery (CRITICAL for P2 — Pilz LIN requires Pilz pipeline)
rg -n "pilz_industrial_motion_planner|pilz_lin_planner|MotionSequenceRequest" --type yaml --type py --type cpp
rg -n "planning_pipelines|default_planning_pipeline" --type yaml
ros2 param get /move_group capability_plugins 2>/dev/null
ros2 param get /move_group planning_pipelines.pipeline_names 2>/dev/null

# C. Identify SSOT violations (uses the improved R2 regex above)
rg -n -t py -t cpp '\b(0\.0[0-9]+|0\.1\b|0\.2\b)\b' src/ \
  | rg -v -e '#' -e '//' -e 'test_' -e '\.yaml' -e '/config/'

# D. Active ROS 2 surface (only if the system is running)
ros2 pkg list | rg -i 'gp4|yaskawa|moveit|llm|safety|perception|gateway'
ros2 node list
ros2 topic list | rg -v rosout
ros2 action list
ros2 service list | rg -v 'parameter|describe_parameters'
ros2 run tf2_tools view_frames                    # produces frames.pdf
ros2 param dump /<llm_gateway_node> 2>/dev/null   # only if it exists

# D'. QoS introspection (CRITICAL for sensor pipelines — see R9 / §5)
ros2 topic info /camera/depth/color/points -v 2>/dev/null
ros2 topic info /camera/color/camera_info  -v 2>/dev/null
ros2 topic info /yaskawa/joint_states      -v 2>/dev/null

# E. Build & lint baseline (current red/green state)
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee /tmp/build.log
colcon test --event-handlers console_direct+    2>&1 | tee /tmp/test.log
```

If any discovery command returns ambiguous or empty output where one was expected, the agent's correct response is:

> **Discovery gap.** I could not locate `<X>` in the workspace. Before I proceed, please confirm the path or run: `<exact command>`.

Do not invent a name. Do not create a stub "to make it work". Stop and ask.

---

## §4. PHASED EXECUTION PLAN

> **Cardinal rule:** Execute ONLY the phase the human explicitly requests by name (e.g. "run P1"). Do not advance.

Each phase below uses the same template:
**Goal · Discovery · Tasks · Verification · DO NOT · Output Artefacts**.

---

### PHASE 0 — Audit, Cleanup & Governance Setup

**Risk class:** Zero (no runtime behaviour changes)
**Duration estimate:** 2-3 days
**Depends on:** Nothing
**Goal:** Map the workspace, remove confirmed dead code, install governance files so subsequent phases run safely.

#### P0.1 Discovery (run §3 + the following)

```bash
# Dead-code candidates
rg -n "DEPRECATED|TODO.*remove|HACK|legacy|stale|XXX"  --type py --type cpp
vulture src/ --min-confidence 80                            # Python only
cppcheck --enable=unusedFunction --quiet src/               # C++ only

# Detect committed binaries that should be in LFS or out-of-tree
find . -type f \( -name "*.sqlite3" -o -name "*.pdf" -o -name "*.bag" -o -name "*.db3" \) -not -path "./build/*" -not -path "./install/*"

# Detect duplicate folder structures (e.g. hmi/hmi/data vs hmi/data)
find src/ -type d | awk -F/ '{print $NF}' | sort | uniq -c | sort -rn | head -20
```

#### P0.2 Tasks

1. Produce an **inventory report** (`docs/audit/INVENTORY_<date>.md`) listing every package, every node, every config file with one-line description. No code change.
2. **Mark, don't delete.** Add `# DEPRECATED: removal_date=<today+28d>, reason=<text>` to anything proven unused by `rg <symbol>` returning 0 callsites. Move binaries / datasheets to a `docs/` or LFS path; do not delete commit history.
3. Write the three governance files **at workspace root**:
   - `AGENTS.md` — copy §0 + §1 + §2 of this harness verbatim
   - `.cursorrules` (or `.claude` / `.continue` depending on the human's tooling — confirm with the human; do not assume)
   - For Python sub-trees only, `importlinter.toml` declaring layer order. **Layer names below are PLACEHOLDERS** — they reflect a common ROS 2 architecture but have not been validated against this workspace. The agent must:
     - run §3 first to identify actual package directories,
     - propose a concrete layer mapping (e.g. "I propose `<pkg_a>` = `interfaces`, `<pkg_b>` = `domain` …"),
     - **wait for human confirmation** before writing the file.
     - Common layer order, for reference only: `interfaces < domain < ros_adapters < services < api`.
4. Add `pre-commit` hooks (do not run yet, just configure): `ruff`, `black`, `mypy --strict`, `clang-format`, `clang-tidy`, `detect-secrets`, `yamllint`.

#### P0.3 Verification

| Check | Pass criterion |
|---|---|
| `cat AGENTS.md .cursorrules` | Files exist, contain R1-R10 verbatim |
| `rg "DEPRECATED" --type py --type cpp \| rg -v "removal_date"` | Returns 0 lines |
| `git status` after cleanup | No accidental edits to safety / motion code |
| `colcon build` | Still green (no functional change) |
| `colcon test` | Same pass rate as baseline |
| Importlinter contract | Confirmed by human before commit |

#### P0.4 DO NOT

- Touch `safety/`, `motion_core/`, `llm_gateway/`, or any path identified as runtime-critical in §3
- Change any public ROS 2 API (topic / service / action name / type)
- Hard-delete anything in this phase
- Create files in any package without human confirmation of the package boundary
- Write `importlinter.toml` with layer names invented by the agent

#### P0.5 Output

A single PR / changeset containing:
1. `docs/audit/INVENTORY_<date>.md`
2. `AGENTS.md`, `.cursorrules` (or equivalent), `importlinter.toml` (with human-confirmed layer mapping)
3. `.pre-commit-config.yaml`
4. Deprecation tags on dead symbols
5. A `MIGRATION-P0.md` summarising what changed and why

---

### PHASE 1 — Safety Hardening (close `velocity_scale=None`, add J4-J6 guard)

**Risk class:** Low (safety-only, no behavioural loosening)
**Duration estimate:** 4-5 days
**Depends on:** P0
**Goal:** Make the safety pipeline fail-closed. Enforce derated operational joint limits at the trajectory level, before MoveIt `execute()`.

#### P1.1 Discovery

Use §3 plus:
```bash
rg -n "velocity_scal" --type py --type cpp        # all callsites
rg -n "QualityGate|validate_trajectory" --type cpp
rg -n "kDefaultVelocityScaling|DEFAULT_VEL"      --type cpp --type py   # hidden defaults
rg -n "operational_joint_limits"                  --type yaml
```
Report **exact file:line** for: validator entry point, magic defaults, YAML safety config.

#### P1.2 Tasks

1. **Close the bypass.** In the validator (Python or C++ — discovery confirms which), if `velocity_scale is None` or absent or `> 1.0` or `<= 0`:
   - Return `(False, "velocity_scale is required and must be in (0, 1]; received: <value>")`
   - Add a unit test asserting this rejection.
2. **Remove magic defaults.** Any `kDefaultVelocityScaling`, `DEFAULT_VEL`, `0.1` literal in C++/Python that represents a velocity scale → read from the YAML SSOT. SSOT key proposal: `safety.defaults.velocity_scale_min`, `safety.defaults.velocity_scale_max`. Do NOT introduce a default value used at runtime — only validation bounds.
3. **Add operational joint limits to the safety YAML** using the block in §1.3 above (including the `joint_6_t` tiered structure and the `extended` preconditions). Do not modify hardware limits (URDF/SRDF) — those mirror the datasheet.
4. **Implement `JointPositionGuard`** at the trajectory level:
   - Hook: inside `QualityGate::validate_trajectory()` (C++) or the Python equivalent identified in P1.1.
   - For every `trajectory_msgs/JointTrajectoryPoint`, reject if any `positions[i]` exceeds `operational_joint_limits[joint_name]`. Report the offending joint, point index, value, and limit in the rejection message.
   - For `joint_6_t` specifically: read `joint_6_t_mode` from SSOT. If `default`, enforce ±π. If `extended`, enforce ±7.941 rad **and** validate every `extended` precondition (signed-off, velocity_scale ≤ 0.10, operator confirm); fail closed on any unmet precondition.
   - **Manipulability check:** at the start, mid, and end of the trajectory, compute `sqrt(det(J·Jᵀ))` via the kinematics solver in use (TRAC-IK or KDL — discover, don't assume). Reject if `< manipulability_floor`.
5. **Cross-validation script** `tools/validate_safety_chain.py`:
   - Loads the YAML safety config.
   - Loads the URDF/SRDF (locate via `xacro`/`urdf_to_graphviz`).
   - Asserts: every `operational_joint_limits[joint] ⊆ urdf.joint.limit`. Asserts Pilz Cartesian limits and safety-rules motion limits agree. Exits non-zero on any mismatch.
6. **Property-based tests** (`hypothesis` for Python, or parametrized GoogleTest for C++): random valid trajectories accept; random out-of-bound trajectories reject; never silent-pass.

#### P1.3 Verification

| Check | Pass criterion |
|---|---|
| Validator with `velocity_scale: null` | Returns `(False, ...)`; explicit message |
| Validator with `velocity_scale: 1.5` | Returns `(False, ...)` |
| Trajectory with `joint_4_r` exceeding ±2.443 rad at any point | Rejected with point index in message |
| Trajectory with `joint_6_t` at +5 rad while `joint_6_t_mode=default` | Rejected |
| Trajectory with `joint_6_t` at +5 rad, `mode=extended` but precondition `cable_inspection_signed_off=false` | Rejected with precondition list |
| Trajectory passing through `manipulability < 0.05` | Rejected |
| `python tools/validate_safety_chain.py` | Exit 0 |
| Coverage on safety module | ≥ 80 % lines, ≥ 70 % branches |
| `rg -t py -t cpp '0\.[12]\b' src/<safety_pkg>/` (filtered through R2 whitelist) | 0 hardcoded velocity literals |

#### P1.4 DO NOT

- Loosen any existing safety check
- Touch the URDF/SRDF hardware limits
- Bypass collision checking
- Add fallback "soft" defaults that take effect when validation fails
- Auto-promote `joint_6_t_mode` from `default` to `extended` based on heuristics
- Modify the LLM gateway, HMI, or any planner pipeline

#### P1.5 Output

- Diff against `<safety_pkg>` (path discovered in P1.1)
- Updated YAML SSOT with `operational_joint_limits` (including tiered T-axis) and `manipulability_floor`
- New file `tools/validate_safety_chain.py` (executable)
- New tests under the safety package's `test/` directory
- `MIGRATION-P1.md` with rollback instructions

---

### PHASE 2 — Fix CIRC & Drawing Pipeline (LIN chain + Pilz blending)

**Risk class:** Medium (changes motion shape; sim-validate before hardware)
**Duration estimate:** 2-3 days
**Depends on:** P1
**Goal:** Eliminate workplane hard-fail, replace `computeCartesianPath` for drawing with a Pilz LIN chain that the wrist guard tolerates, runs continuously (blended) instead of stopping at every chunk, and recovers from per-waypoint LIN failures.

#### P2.1 Background (read this before coding)

- `moveit::planning_interface::MoveGroupInterface::computeCartesianPath` does dense Cartesian interpolation but has no wrist-continuity optimisation → joint flips on 6-DOF arms → `WristFlipGuard` (or equivalent in §1.3) rejects.
- Pilz `LIN` is a Cartesian-linear motion primitive that does proper redundancy / nullspace handling for industrial wrists, **but only when the Pilz planner pipeline is loaded** (`pilz_industrial_motion_planner`). If only OMPL is loaded, every LIN call will fail or crash.
- Pilz supports sequence blending via `blend_radius` so the robot does not decelerate to zero between successive `LIN`s.
- Therefore: drawing geometry → list of waypoints → list of `LIN` primitives → `MotionSequenceRequest` with `blend_radius` per intermediate point.

#### P2.2 Discovery (Pilz pipeline check is MANDATORY)

```bash
# 1. Pilz pipeline must be present — without it, all LIN calls fail
rg -n "pilz_industrial_motion_planner|pilz_lin_planner" --type yaml --type xml
rg -n "MotionSequenceRequest|MotionSequenceItem"        --type py --type cpp
ros2 param get /move_group capability_plugins                  2>/dev/null
ros2 param get /move_group planning_pipelines.pipeline_names   2>/dev/null
# Expected: Pilz appears in `planning_pipelines.pipeline_names`. If absent, STOP and ask
# whether to install/configure Pilz, or whether to keep the planner OMPL-only and choose
# a different motion strategy.

# 2. Drawing pipeline location
rg -n "hydrate_draw_workplane|workplane" --type py --type cpp
rg -n "computeCartesianPath" --type cpp
rg -n "get_current_pose" --type py --type cpp
```

If Pilz is not installed, the agent must explicitly state:

> Pilz pipeline is not configured in this workspace. P2 cannot proceed as designed without it. Options: (a) install `pilz_industrial_motion_planner` and add it to `planning_pipelines.pipeline_names`, (b) defer P2, (c) use an alternative motion strategy (slower; subject to redesign). Which?

Do not silently fall back to `computeCartesianPath`.

#### P2.3 Tasks

1. **Workplane fallback.** In `<workplane_module>` identified above:
   - Wrap the `get_current_pose` call in `try/except` (Python) or `wait_for_service(timeout)` + null-check (C++).
   - On timeout / unavailable: log `WARN` (not error), fall back to `mode="base"` with the last cached pose; if no cache, fall back to a configurable safe default pose loaded from the YAML SSOT (key proposal: `drawing.fallback.base_pose`).
   - Never crash the node. Never hard-fail with `rclcpp::shutdown`.
2. **Replace `computeCartesianPath` for drawing.** In the geometry compiler:
   - Convert the polyline / curve to N waypoints (existing logic, keep).
   - Build a `pilz_industrial_motion_planner::MotionSequenceRequest` of N `LIN` items.
   - For each intermediate item, set `blend_radius` from SSOT (proposal: `drawing.blend_radius_m: 0.008`, range `[0.005, 0.015]`).
   - Set `velocity_scaling > 0` on all intermediate items so the motion does not pause.
   - First and last items keep `blend_radius = 0.0` (start/stop must be exact).
3. **Blend-radius vs collision distance check.** Before submitting the `MotionSequenceRequest`:
   - Query MoveIt's PlanningScene for the nearest collision object to each waypoint.
   - Reject if any waypoint has `blend_radius >= distance_to_nearest_collision_object`. The blend sphere can intersect obstacles invisibly to the planner.
   - Suggest a smaller `blend_radius` in the rejection message, or fall back to `blend_radius = 0` for that waypoint (i.e. accept a stop there) only with explicit operator approval.
4. **LIN failure recovery (per-waypoint).** If a `MotionSequenceItem` fails to plan:
   - Log the failure with waypoint index and Pilz error code.
   - Attempt PTP to that waypoint (reuse the same target pose). PTP must still pass §1.3 `operational_joint_limits` and the `JointPositionGuard` from P1.
   - If PTP also fails: report failure with waypoint index and reason; **never silently skip a waypoint**.
   - Resume LIN at waypoint i+1 only if the recovery succeeded, the operator is informed, and the resulting trajectory is re-validated end-to-end.
5. **Python pre-validation for CIRC** (degenerate arc):
   - Compute `cross(goal - start, aux - start)`. If `‖cross‖ / (‖goal - start‖ · ‖aux - start‖) < 1e-3`, reject at the gateway with `"degenerate CIRC: aux collinear with start-goal"`. Do not let it reach the C++ planner.
6. **Keep the single-arc CIRC primitive available** for genuine arc requests (backward compatibility per R5).

#### P2.4 Verification

| Check | Pass criterion |
|---|---|
| Pilz pipeline check (P2.2) | Confirmed present, or P2 deferred with explicit human ack |
| Workplane service unavailable in sim | Node logs WARN, continues, draws in base frame |
| `ros2 topic echo /<llm_gateway>/debug` while drawing a circle | Output contains only `LIN` (and possibly `PTP` for approach), zero `CARTESIAN_PATH` |
| Drawing a circle radius 0.05 m at z=0.1 in RViz preview | Smooth, no visible stutter; trajectory has no `velocity[i] = 0` at intermediate points |
| Waypoint within `blend_radius` of a CollisionObject | Rejected, with offending waypoint index and recommended smaller radius |
| Forced LIN failure on one waypoint (test mock) | PTP fallback executes; if PTP fails too, full plan rejected, never silent-skip |
| Degenerate CIRC (collinear) sent to gateway | Rejected with explicit message before planner is called |
| Existing single-arc CIRC integration tests | Still pass |
| TRAC-IK / KDL solver wrist excursion on circle trajectory | All joints inside `operational_joint_limits` from P1 |

#### P2.5 DO NOT

- Touch `motion_core` planner internals
- Disable `WristFlipGuard` or any P1 guard
- Globally change the default planner ID
- Hardcode poses, blend radii, or velocity scales (all → SSOT YAML)
- Fall back to `computeCartesianPath` if Pilz is missing — stop and ask
- Silently skip a waypoint on LIN failure

#### P2.6 Output

- Diff against `<llm_gateway_pkg>` and `<motion_orchestration_pkg>` (paths from P2.2)
- New SSOT keys: `drawing.blend_radius_m`, `drawing.fallback.base_pose`
- Test: `tests/integration/test_draw_circle_blended.py` — RViz / fake-controller end-to-end
- Test: `tests/integration/test_lin_failure_recovery.py` — forced failure on waypoint k, PTP fallback verified
- `MIGRATION-P2.md`

---

### PHASE 3 — LLM Reasoning Engine (ReAct + Tool Calling, Async, Stateful)

**Risk class:** Medium (changes the LLM contract; old path stays as fallback)
**Duration estimate:** 5-7 days
**Depends on:** P2
**Goal:** Replace static intent-classifier with a ReAct agent that has tool access, real-time robot state grounding, async execution, and a tiered, bounded reasoning loop.

#### P3.1 Background

Static prompts (700+ lines, `temperature=0`) cannot recover from "robot is in MOVING state, retry later" or "object not found". A ReAct loop (Thought → Tool → Observation → Thought → … → Final Command) with structured tool schemas (OpenAI function-calling format or equivalent JSON schema) and Pydantic validation gives bounded, auditable reasoning **without** giving the LLM direct controller access (R3 still holds).

#### P3.2 Discovery

```bash
rg -n "IntentRouter|intent_router|llm_raw_command|llm_intent" --type py
rg -n "temperature\s*=\s*0" --type py
rg -n "FunctionTool|tool_registry|@tool" --type py
ros2 service list | rg -i 'pose|plan_motion|execute'
ros2 action list  | rg -i 'execute|plan|trajectory'

# Capability discovery — NEEDED to decide which tools are wired in vs deferred
rg -l "gripper|GripperCommand|control_msgs/action/GripperCommand" --type py --type cpp --type xml
ros2 action list 2>/dev/null | rg -i 'gripper'
```

If no gripper / IO action is found, the agent must register the corresponding tools as `NotImplemented` stubs that return `{"error":"capability_unavailable","capability":"gripper"}`. **Pick-and-place tasks remain available in the verification suite as TODOs**, but they are not part of P3 acceptance until a gripper is installed and wired up in a follow-up phase.

#### P3.3 Tasks

1. **Tool registry** (new module, location confirmed in P3.2 — likely `<llm_gateway_pkg>/react_tools/`):
   - `get_current_pose() -> PoseStamped` — wraps existing service, **read-only**
   - `plan_motion(target: PoseStamped|JointPositions, planner: str, vel: float, acc: float) -> PlanHandle` — calls MoveIt, returns plan id, does NOT execute
   - `submit_motion(plan_handle: str) -> SubmissionResult` — sends to executor via ROS 2 Action `send_goal_async`, returns immediately. **`SubmissionResult` is one of:** `SUBMITTED(goal_id, expected_duration_s)`, `REJECTED(reason)`, `TIMEOUT`. **The tool name "submit" is intentional** — it makes clear the LLM is submitting to the safety chain, not executing on hardware. The actual hardware execution is decided by the executor after the safety chain re-verifies.
   - `query_perception(class_filter: str|None) -> List[Detection3D]` — gated by RobotState (R9; see P4)
   - `wait_for_state(state: str, timeout_s: float) -> bool`
   - `set_speed(velocity_scale: float) -> bool` — passes through P1 validator; rejects on bypass
   - `gripper_open() / gripper_close(force: float) -> bool` — wired only if §3 / P3.2 confirmed gripper presence; otherwise `NotImplemented` stub
   - **Every tool returns Pydantic models. Every tool input is Pydantic-validated.** Validation failure = tool returns error, never raises into the LLM loop.

2. **State grounding.** Inject into every LLM call as a structured system message section:
   ```yaml
   robot_state:
     joints_rad: [<live>, <live>, <live>, <live>, <live>, <live>]
     joint_names: [joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t]
     joint_6_t_mode: <default|extended>
     mode: <IDLE|PLANNING|MOVING|FAULT|ESTOPPED>
     last_action: {tool: "...", status: "...", error: "..."}
     active_alarms: [...]
     velocity_scale_active: 0.2
     capabilities:
       gripper: <true|false>
       perception: <true|false>
   ```
   Source the values from MotoROS2's `/yaskawa/joint_states` and `/yaskawa/robot_status`.

3. **ReAct loop — tiered iteration limits** (replaces v1's single cap of 3):
   ```yaml
   llm:
     react:
       max_total_iterations:    5      # absolute ceiling per request
       max_motion_iterations:   3      # of those, at most 3 may call plan_motion or submit_motion
       max_readonly_iterations: 10     # query-only loops (get_current_pose, query_perception, wait_for_state) can iterate further while no motion is requested; this counter is independent and runs in parallel
       max_repair_iterations:   1      # after a schema or safety rejection, allow one repair attempt
       wall_clock_timeout_s:    30
   ```
   - Each iteration: model call → parse tool call (Pydantic) → execute tool → append observation → next call.
   - On schema validation failure: append the validation error as observation, allow up to `max_repair_iterations` repair attempts, then hand off to operator.
   - On tool execution failure (e.g. safety reject): append the rejection reason verbatim, allow one replan within `max_motion_iterations`, then hand off.
   - Hard cap on total wall-clock per request: `wall_clock_timeout_s` (default 30 s, SSOT). Beyond → cancel goals, return `TIMEOUT`.

4. **Async execution.** No `time.sleep` in tools. `submit_motion` uses `ActionClient.send_goal_async`; the loop yields to the executor; feedback published on `/<llm_gateway>/gateway_status`.

5. **Phased rollout.**
   - HMI text ingress uses `/llm_gateway/review_intent` to request Semantic IR review.
   - Direct topic execution on `/llm_intent`, `/llm_text_input`, and `/llm_raw_command` is disabled by default and must remain behind `allow_direct_topic_execution`.
   - All reviewed routes must converge at the same Safety Gate (R3, R4).

#### P3.4 Verification

| Check | Pass criterion |
|---|---|
| 5 NL prompts (e.g. "go home", "draw a 5 cm circle 10 cm above the table", "what's your current pose?", "stop and reset", and one chained "go home, then move to pre-grasp") | All produce valid Pydantic-parsed tool calls; all reach the safety gate; none publish directly; each completes within `max_total_iterations=5` |
| "Pick up the red block" prompt | If gripper present: completes. If absent: returns `capability_unavailable` cleanly without crash |
| Goal cancel during execution | Cancel propagates; ReAct loop terminates cleanly |
| `time.sleep` audit | `rg -n "time\.sleep|std::this_thread::sleep" <llm_gateway_pkg>/react_tools/` returns 0 hits |
| Old route `/llm_raw_command` | Rejected by default unless `allow_direct_topic_execution=true` is explicitly set |
| Schema validation | 100 % of tool calls Pydantic-validated; failures generate observations, not exceptions |
| Replan after safety reject | Demonstrated in `tests/integration/test_react_replan.py`, completes within `max_motion_iterations=3` |
| Read-only loop (e.g. "wait until idle and tell me the pose") | Allowed up to `max_readonly_iterations=10` even while motion counter remains at 0 |

#### P3.5 DO NOT

- Delete `IntentRouter` — only mark deprecated
- Let any tool publish directly to a controller
- Use `temperature=0` if it kills exploration; pick from SSOT `llm.temperature` (suggested 0.2 for production, 0.0 for replay)
- Rename `submit_motion` back to `execute_*` — semantic clarity matters
- Skip Pydantic validation "because the LLM usually returns valid JSON"
- Inject the entire MoveIt scene state into every prompt — pass only joint state, mode, last action, active alarms, capability flags (token budget!)

#### P3.6 Output

- New module `<llm_gateway_pkg>/react_agent/` (path confirmed in P3.2)
- Tool registry, state injector, async client wrappers
- Updated launch file with both `/llm_raw_command` and `/llm_intent` routes
- Tests (unit + integration)
- `MIGRATION-P3.md`

---

### PHASE 4 — Vision: RealSense D435i Eye-to-Hand Integration

**Risk class:** Medium-High (introduces a new safety dependency)
**Duration estimate:** 5-7 days
**Depends on:** P3 (perception is consumed by ReAct tool `query_perception`)
**Goal:** Integrate D435i as eye-to-hand. Calibrate. Publish detections. Push them as `CollisionObject` into MoveIt's planning scene. Provide the LLM tool. Enforce calibration freshness, depth accuracy, and state gating.

#### P4.1 Discovery

```bash
ros2 pkg list | rg -i 'realsense|perception'
rg -n "realsense2_camera|d435" --type py --type xml --type yaml
find . -path "*/calibration*" -type f
ros2 topic list | rg -i 'camera|points|depth|color'
ros2 topic info /camera/depth/color/points -v 2>/dev/null   # MUST run — confirms QoS
ros2 topic info /camera/color/camera_info  -v 2>/dev/null   # MUST run — confirms QoS
```

#### P4.2 Tasks

1. **Create `<perception_pkg>`** (name to be agreed with the human; e.g. `gp4_perception`). Standard ROS 2 layout: `config/`, `launch/`, `src/`, `include/`, `test/`, `package.xml`, `CMakeLists.txt`.

2. **Camera launch.** Wrap `realsense2_camera` with launch args:
   - `align_depth.enable: true`
   - `enable_sync: true`
   - `pointcloud.enable: true`
   - `depth_module.profile`: choose for the GP4 workspace 0.3-0.8 m (e.g. `848x480x30`)
   - IR emitter on/off via parameter (off when external IR present)

3. **Hand-eye calibration service** (eye-to-hand, NOT eye-in-hand — camera is fixed in cell):
   - Service `/perception/calibrate_hand_eye` collects N pose pairs: TCP pose (from `/yaskawa/joint_states` + FK) + ArUco/Charuco board pose (from camera).
   - Solve `AX = XB` via OpenCV `cv2.calibrateHandEye` with `CALIB_HAND_EYE_PARK` (preferred for industrial accuracy).
   - Output YAML at `config/extrinsics_<date>.yaml`. **The `calibration_date` field MUST be filled by the calibration service at the moment of solving — never hardcoded in any template, example, or fixture file.** The freshness guard depends on this. Hardcoding a date silently disables the guard.
     ```yaml
     hand_eye_extrinsics:
       parent_frame: base_link            # confirm via §3 TF tree
       child_frame:  camera_color_optical_frame
       translation:  {x: <solved>, y: <solved>, z: <solved>}
       rotation_quat:{x: <solved>, y: <solved>, z: <solved>, w: <solved>}
       calibration_date: <SET_AT_RUNTIME_BY_CALIBRATION_SERVICE>   # ISO 8601 UTC. NEVER hardcode.
       reprojection_error_mm: <solved>
       n_samples: <solved>
       solver: PARK
     ```
   - Publish as `static_transform_publisher`.

4. **Freshness, accuracy, and depth-noise guards** (extends P1's safety chain):
   - Reject any motion plan that depends on perception if `now() - calibration_date > safety.calibration.max_age_days` (default 30 d).
   - Reject if `reprojection_error_mm > safety.calibration.max_reprojection_error_mm` (default 3 mm).
   - **Reject if a detection's measured depth standard deviation exceeds `perception.depth.max_depth_noise_mm` (default 5 mm).** D435i noise increases at FOV edges and at range; this is the dominant "looks correct, miss the grasp" failure mode.
   - SSOT additions:
     ```yaml
     safety:
       calibration:
         max_age_days: 30
         max_reprojection_error_mm: 3.0
     perception:
       depth:
         min_range_m: 0.30          # below this, multipath dominates
         max_range_m: 0.80          # above this, GP4 cannot reach anyway, and noise is high
         max_depth_noise_mm: 5.0    # std-dev across a 5x5 patch around the detection centroid
         roi_crop: true             # crop to GP4 reachable workspace before processing
         workspace_bbox_m:
           x: [0.20, 0.55]
           y: [-0.30, 0.30]
           z: [0.00, 0.40]
     ```

5. **Scene processor node**:
   - Subscribe `/camera/depth/color/points` and `/camera/color/camera_info` via `ApproximateTimeSynchronizer` (R9), `slop=0.05`, `queue_size=10`.
   - **QoS must match the publishers exactly.** RealSense publishes images and point clouds with `SensorDataQoS()` (best-effort, depth=5). The subscriber must use the same profile, otherwise `ApproximateTimeSynchronizer` will silently drop every message and the callback will never fire.
     ```python
     # Python
     from rclpy.qos import qos_profile_sensor_data
     cloud_sub = Subscriber(node, PointCloud2, '/camera/depth/color/points',
                            qos_profile=qos_profile_sensor_data)
     info_sub  = Subscriber(node, CameraInfo,  '/camera/color/camera_info',
                            qos_profile=qos_profile_sensor_data)
     sync = ApproximateTimeSynchronizer([cloud_sub, info_sub],
                                        queue_size=10, slop=0.05)
     ```
     ```cpp
     // C++
     auto qos = rclcpp::SensorDataQoS();
     message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub(
         node, "/camera/depth/color/points", qos.get_rmw_qos_profile());
     ```
     Verify after launch with `ros2 topic info /camera/depth/color/points -v`.
   - Pipeline: ROI crop (using `perception.depth.workspace_bbox_m`) → voxel downsample (e.g. 0.005 m) → RANSAC dominant plane removal (table) → Euclidean clustering → pose estimation (PCA bounding box) → reject clusters whose depth noise exceeds `max_depth_noise_mm`.
   - Publish `vision_msgs/Detection3DArray` on `/perception/detections`.
   - Push each detection as a MoveIt `CollisionObject` (primitive box) via `planning_scene_interface.applyCollisionObjects`. Include a TTL: remove if not re-detected for 2 s.

6. **LLM tool `query_perception`**:
   - Subscribes to `/perception/detections` and `/yaskawa/robot_status`.
   - Returns latest detections **only when robot mode == IDLE** (R9). Otherwise: `{"error":"perception_blocked_during_motion","mode":"<current>"}`.
   - All poses transformed to `base_link` before return; never expose camera-frame poses to the LLM.
   - On stale calibration or excessive reprojection error, returns `{"error":"calibration_invalid","reason":"<...>"}`.

7. **RViz config** committed (`config/perception.rviz`) with Detection3DArray, PointCloud2, TF, PlanningScene displays.

#### P4.3 Verification

| Check | Pass criterion |
|---|---|
| `ros2 run tf2_tools view_frames` | `base_link → camera_color_optical_frame` static, matches extrinsics YAML |
| Reprojection error after calibration | ≤ 3 mm (from solver report) |
| Calibration YAML | `calibration_date` is a real ISO 8601 timestamp, NOT a placeholder, NOT hardcoded in templates |
| Motion attempted with stale (>30 d) calibration | Rejected by safety gate, message names the cause |
| `ros2 topic info /camera/depth/color/points -v` | Publisher QoS shown; subscriber config matches |
| Drop a known box on the table | Appears in `/perception/detections` and as `CollisionObject` in PlanningScene within 1 s; depth noise < 5 mm |
| Place box at FOV edge where noise > 5 mm | Detection rejected with `depth_noise_exceeded` |
| `query_perception` while robot is MOVING | Returns `perception_blocked_during_motion` |
| Sync correctness | Logging shows `points` and `camera_info` timestamps within 50 ms; callback fires |
| LLM prompt: "pick up the box" (with gripper) | Tool returns pose in `base_link`; ReAct plans collision-aware path; safety gate accepts |

#### P4.4 DO NOT

- Mount the camera on the flange (this phase is **eye-to-hand only**)
- Use camera intrinsics from a generic D435i calibration; intrinsics come from the device's factory data via `realsense2_camera`
- Use perception detections as **hard** safety limits — they are advisory inputs to the planner; safety gate is still authoritative
- Publish raw point cloud at 30 Hz to `/tf` or any Reliable QoS — use `sensor_data` QoS for high-rate sensor topics
- Hardcode `calibration_date` in any template, example, fixture, or test file — runtime-fill or refuse
- Skip the depth-noise check "for testing"

#### P4.5 Output

- New `<perception_pkg>` (path agreed with human, default suggestion `src/gp4_perception`)
- Calibration tooling + YAML schema + first calibration file (with runtime-filled date)
- Scene processor + detection publisher + collision object pusher
- LLM tool `query_perception` registered in the ReAct registry from P3
- RViz config + integration test (including a depth-noise-rejection test)
- `MIGRATION-P4.md`

---

### PHASE 5 — Architecture Consolidation, CI/CD, Final Cleanup

**Risk class:** Low-Medium (refactor risk; offset by full CI)
**Duration estimate:** 3-4 days
**Depends on:** P4
**Goal:** Deduplicate utilities, complete the YAML SSOT, enforce architecture and safety as CI gates, hard-delete deprecated code that has aged 28+ days.

#### P5.1 Discovery

```bash
jscpd src/ --reporters consoleFull --min-lines 5 --min-tokens 60
rg -n "_wrap_to_pi|_rpy_to_quaternion|_pose_to_matrix" --type py
find . -name "*.yaml" -path "*/config/*"
rg -n "DEPRECATED" --type py --type cpp                # deletion candidates
```

#### P5.2 Tasks

1. **Deduplicate utilities** into a `<common_pkg>` (path confirmed in P5.1; if absent, create with human approval). Common targets: angle wrap, RPY ↔ quaternion, pose ↔ matrix, frame conversions. Use Python `pathlib` and C++ `std::filesystem`; no per-package re-implementation.
2. **YAML SSOT consolidation.** Single safety YAML per package. Remove legacy keys (e.g. `joint_limits_override`). All consumers read via a shared loader (Pydantic models for Python, YAML→struct for C++).
3. **Hard-delete aged deprecations.** For each `# DEPRECATED: removal_date=<past>`:
   - Confirm `rg <symbol>` returns 0 hits.
   - Delete in a separate commit. Each deletion is its own commit for clean revert.
4. **CI pipeline** (`.github/workflows/ci.yml` or `.gitlab-ci.yml` — confirm with human):
   ```
   jobs:
     lint:        ruff, black --check, clang-format --dry-run, yamllint
     typecheck:   mypy --strict (Python), clang-tidy (C++)
     architecture:importlinter, jscpd (fail if duplicate >30 LOC)
     test:        colcon test (unit + integration)
     safety:      tools/validate_safety_chain.py
     dead-code:   vulture --min-confidence 90 (fail if regress)
     build:       colcon build (multi-distro: Humble required, Iron optional)
     mock-hw:     run integration tests against fake_components controller
   ```
5. **Pre-commit hooks** activated and required for merge.
6. `MIGRATION.md` (workspace-level): how to rebase, rebuild, retest after P0-P5.

#### P5.3 Verification

| Check | Pass criterion |
|---|---|
| `jscpd src/` | Largest duplicate block ≤ 30 LOC |
| `importlinter --config importlinter.toml` | All contracts pass |
| `pre-commit run --all-files` | Exit 0 |
| CI on `main` | Green; required jobs cannot be skipped |
| `rg "DEPRECATED" --type py --type cpp` | 0 hits whose `removal_date` is past |
| `colcon build` clean | Zero warnings on `-Wall -Wextra` for new code |

#### P5.4 DO NOT

- Break public ROS 2 APIs (topic/service/action names) without a migration window
- Remove safety checks "to make CI green"
- Combine refactor and behaviour change in the same commit
- Disable any CI gate without explicit human approval recorded in PR description

#### P5.5 Output

- Consolidated `<common_pkg>`
- One SSOT YAML per package, validated by loader
- Deleted commits for each aged deprecation (one per commit)
- CI configuration + pre-commit hooks
- Workspace-level `MIGRATION.md`

---

## §5. TECHNICAL GUARDRAILS REFERENCE TABLE

Cross-check this table **before** writing MoveIt 2 / ROS 2 code. Anti-hallucination cheat-sheet.

| Concern | Correct API / Mechanism | Common wrong assumption |
|---|---|---|
| Trajectory-level joint guard | C++ `QualityGate::validate_trajectory()` iterates `trajectory_msgs::msg::JointTrajectory.points` and rejects **before** calling MoveIt `execute()` | Putting the guard inside the controller (too late) or only at the goal pose (misses intermediate violations) |
| Pilz blended LIN sequence | `pilz_industrial_motion_planner::MotionSequenceRequest` with `MotionSequenceItem.blend_radius_m` set on intermediate items; `0.0` on first/last. **Pilz pipeline must be loaded** (`planning_pipelines.pipeline_names` contains `pilz_industrial_motion_planner`). | Calling LIN/CIRC primitives without checking pipeline; using `computeCartesianPath` for industrial drawing — no wrist optimisation, causes joint flips |
| Async ROS 2 action calls (Python) | `rclpy.action.ActionClient.send_goal_async(goal)`; await `goal_handle = await future`; subscribe to feedback via callback | `time.sleep` while polling `goal_handle.status` — blocks the executor |
| Async ROS 2 action calls (C++) | `rclcpp_action::Client::async_send_goal(goal, options)`; provide `feedback_callback`, `result_callback` | Calling `wait_for_action_server(0)` then `send_goal_sync` from a single-threaded executor — deadlock |
| Multi-topic time sync (Python) | `message_filters.ApproximateTimeSynchronizer([sub1, sub2], queue_size=10, slop=0.05)`; register single callback. **CRITICAL: subscriber QoS must match publisher QoS exactly** (`qos_profile=qos_profile_sensor_data` for RealSense, MotoROS2 `joint_states` is reliable). Verify with `ros2 topic info <topic> -v`. Mismatch causes silent message drop with no error. | Two independent subscribers + manual time-buffer logic — drift, lost messages. Or default subscriber QoS on a `sensor_data` publisher — every message dropped, callback never fires, debug looks like "the topic is empty" |
| Multi-topic time sync (C++) | `message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<T1,T2>>` with `rclcpp::SensorDataQoS().get_rmw_qos_profile()` passed to each `message_filters::Subscriber`. | Using `ExactTime` on sensors with different frame rates — never triggers. Or omitting QoS — inherits default reliable, mismatches RealSense |
| Hand-eye calibration solver | OpenCV `cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_PARK)`. Calibration date set at solve time, never templated. | Tsai for low-sample sets — high variance; PARK or DANIILIDIS preferred. Hardcoding a sample date in template files defeats the freshness guard. |
| TF lookup with namespace | `tf2_ros.Buffer.lookup_transform("base_link", "camera_color_optical_frame", rclpy.time.Time())` while remembering MotoROS2 publishes on `yaskawa/tf` (namespaced) | Looking up against global `/tf` and getting "frame does not exist" |
| MoveGroup Python API | `moveit_py` (Humble+) — NOT `MoveGroupCommander` (that was MoveIt 1 / Noetic) | "MoveGroupInterface in Python" — does not exist; that's the C++ API |
| IK solver for 6-DOF | TRAC-IK preferred; configure in `kinematics.yaml`. Watch for mimic-joint caveats | KDL default — slower, more failures on the GP4's 6-DOF wrist |
| Time parameterisation | TOTG / IPTP / Pilz — these are post-processing, not planners. They do not solve geometry. | Treating TOTG as if it could fix a geometrically infeasible OMPL plan |
| Pipeline isolation | "Plan failed" ≠ "execution failed" ≠ "controller failed" — diagnose layer by layer (see §debug method in the system constitution) | Logging only the last error and assuming root cause |
| QoS for `joint_states` | `default` / reliable (MoveIt 2 requirement; matches MotoROS2 config) | `sensor_data` / best-effort — MoveIt drops state |
| QoS for high-rate sensors | `sensor_data` / best-effort (point clouds, raw images). **Confirm with `ros2 topic info <topic> -v`, do not assume.** | `default` / reliable — buffer overruns, latency spikes, or in `ApproximateTimeSynchronizer` cases, total message drop |

### §5.5 Error Recovery Taxonomy

When a request fails, recovery is layer-specific. Mixing layers is the most common debugging trap.

| Layer | Example failure | Detection | Recovery (in order) | Escalation if recovery fails |
|---|---|---|---|---|
| **LLM schema** | Pydantic rejects tool call JSON | Pydantic `ValidationError` caught at tool boundary | Append validation error to ReAct observation; allow 1 repair iteration (`max_repair_iterations`) | Hand off to operator with the original LLM output and the validation error |
| **Safety gate** | `velocity_scale` out of range, joint limit exceeded, stale calibration | Validator returns `(False, reason)` | Append `reason` to ReAct context; allow 1 replan within `max_motion_iterations` | Reject the request; log; do not retry |
| **Planner** | OMPL timeout, Pilz LIN fails near singularity | MoveIt `MoveItErrorCode != SUCCESS` | (a) Retry with a different planner from `planning_pipelines.pipeline_names`. (b) For LIN failure on a waypoint: PTP fallback to that waypoint (P2.3 task 4), then resume LIN. **Never silently skip a waypoint.** | Report failed waypoint index + planner error; hand off |
| **Trajectory guard** | `JointPositionGuard` rejects intermediate point | Guard returns rejection with point index | Re-plan with tighter Cartesian constraints or smaller blend radius; one attempt | Reject; require operator inspection |
| **Execution** | Controller rejects goal, MotoROS2 alarm | Action result `FAULT` or `ABORTED` | (a) Wait for `IDLE`. (b) Read alarm code from `/yaskawa/robot_status`. (c) Surface alarm code to operator — **never silently retry on hardware alarms.** | Hand off with full alarm code + last command |
| **Perception** | No detection matching the requested class | Empty `Detection3DArray` for the timeout window | Retry the query once with a wider class filter or longer integration window | Return `object_not_found`; do not invent a pose |
| **Hardware** | E-stop, controller disconnect | `stop_motion_on_disconnect` triggers; `mode=ESTOPPED` | None automatic. Operator must clear physically. | All requests rejected until operator clears and confirms |

The agent must, on every failure, identify which layer it occurred in **before** proposing a fix. Mixing layers (e.g. retrying a hardware-alarm failure as if it were a planner timeout) is a critical anti-pattern.

---

## §6. PER-PHASE INVOCATION TEMPLATES (what the human types)

Copy-paste into your AI agent. **One phase at a time.**

### Invocation: P0
```
You are operating under MASTER_AGENT_HARNESS_v2.md, sections 0-3 are mandatory.
Run PHASE 0 only. Begin with the Discovery Protocol from section 3 and paste output.
Then perform P0.1-P0.5. For importlinter, propose a layer-to-package mapping and WAIT
for my confirmation before writing the file. Output a single PR with diffs only.
Stop and wait for review before P1. Cite exact file:line for every change.
```

### Invocation: P1
```
You are operating under MASTER_AGENT_HARNESS_v2.md.
P0 is merged. Run PHASE 1 only.
Discovery first (section 3 + P1.1). Confirm the validator file path before edits.
Velocity_scale=None must fail closed. Add operational_joint_limits (including the
tiered joint_6_t structure) and JointPositionGuard. Tests must cover acceptance AND
rejection paths, including the joint_6_t extended-mode preconditions.
Stop after Verification table is green.
```

### Invocation: P2
```
You are operating under MASTER_AGENT_HARNESS_v2.md.
P0+P1 merged. Run PHASE 2 only.
Run the Pilz pipeline discovery in P2.2 FIRST. If Pilz is not configured, stop and
ask me how to proceed. Do NOT silently fall back to computeCartesianPath.
Replace computeCartesianPath in the drawing compiler with a Pilz LIN
MotionSequenceRequest using blend_radius from SSOT. Implement blend-radius vs
collision-distance check and per-waypoint LIN failure recovery (PTP fallback).
Workplane fallback must not crash the node. Pre-validate CIRC degeneracy at gateway.
Sim-only verification this phase.
```

### Invocation: P3
```
You are operating under MASTER_AGENT_HARNESS_v2.md.
P0-P2 merged. Run PHASE 3 only.
Build the ReAct agent on /llm_intent. Keep IntentRouter on /llm_raw_command
(DEPRECATED tag). Use the tiered iteration limits in P3.3 (max_total=5,
max_motion=3, max_readonly=10, max_repair=1). Tool name is submit_motion, NOT
execute_primitive. All tools async. All inputs/outputs Pydantic-validated. State
injector must read live joint_states and robot_status, AND populate the capability
flags. If gripper is absent, register gripper tools as NotImplemented stubs.
Output diffs + tests.
```

### Invocation: P4
```
You are operating under MASTER_AGENT_HARNESS_v2.md.
P0-P3 merged. Run PHASE 4 only.
Eye-to-hand only. Confirm the new perception package name with me before creating
any folder. Subscriber QoS for RealSense topics MUST be SensorDataQoS, verified
with ros2 topic info -v. ApproximateTimeSynchronizer mandatory.
Calibration freshness <=30 days, reprojection <=3 mm, depth noise <=5 mm —
all gates wired into the safety chain from P1. calibration_date MUST be set at
runtime by the calibration service; never hardcode. query_perception state-gated
to IDLE.
```

### Invocation: P5
```
You are operating under MASTER_AGENT_HARNESS_v2.md.
P0-P4 merged and stable for at least 4 weeks. Run PHASE 5 only.
Hard-delete only deprecations whose removal_date is past AND rg <symbol> returns 0
hits. Each deletion in its own commit. CI must be green on main before this phase
ends.
```

---

## §7. UNIVERSAL ACCEPTANCE CHECKLIST (run at end of EVERY phase)

| Item | Command / Check | Pass criterion |
|---|---|---|
| Build | `colcon build --symlink-install` | 0 errors, ≤ baseline warnings |
| Tests | `colcon test && colcon test-result --verbose` | 0 failures, coverage non-decreasing |
| Lint | `pre-commit run --all-files` | Exit 0 |
| Architecture | `lint-imports --config importlinter.toml` | All contracts pass |
| Safety chain | `python tools/validate_safety_chain.py` | Exit 0 |
| Hardcoded magic numbers (improved R2 self-check) | See §2 R2 improved self-check | No SSOT violations outside whitelist |
| ROS 2 surface unchanged | `ros2 topic list`, `ros2 service list`, `ros2 action list` diff vs baseline | Only intentional additions |
| QoS audit | `ros2 topic info <each new topic> -v` | Profile documented; matches publisher |
| TF tree | `ros2 run tf2_tools view_frames` | No new disconnects, no duplicates |
| Memory | `valgrind --leak-check=full` on new C++ nodes | No definitely lost blocks |
| Diff size | `git diff --stat HEAD~1` | Each file < 200 LOC changed; if larger, justify in PR |
| Migration doc | `MIGRATION-P<N>.md` exists | Includes rollback steps |
| Reviewer note | "Sim-validated; not yet hardware-validated" if applicable | Stated explicitly in PR |

---

## §8. CRITICAL SAFETY WARNINGS (READ BEFORE HARDWARE)

These are non-negotiable. They override convenience, schedule, and elegance.

1. **Sim success does not equal hardware safety.** Every phase verified in RViz + fake controller is still **plan-only** on hardware until the operator confirms in person, with E-stop reachable, and `velocity_scale ≤ 0.2` for the first run.
2. **`velocity_scale=None` → fail closed, always.** A "reasonable default" is the shortest path to an overshoot incident. P1 closes this; P5 verifies it stays closed in CI.
3. **`computeCartesianPath` is wrong for industrial drawing on a 6-DOF wrist.** Joint flips → hardware reject or worse, mechanical strain. Use Pilz LIN + blending (P2). If Pilz pipeline is missing, stop — do not fall back to `computeCartesianPath`.
4. **Eye-to-hand calibration drifts.** Camera knocked, table moved, temperature swing → invisible failure. The freshness, reprojection, and depth-noise guards (P4) are mandatory. Hardcoding `calibration_date` in any template defeats the guard — never do it.
5. **The LLM is a planner-submitter, not a controller.** R3 is absolute. The tool name `submit_motion` (P3) is intentional and must not be reverted to `execute_*`. Any code path where an LLM tool can publish to `/joint_trajectory_controller/*` is rejected at review.
6. **Operator and independent safety systems are the final authority.** This stack is research / lab use, no ISO 10218 certification. The pendant E-stop, controller safety I/O, and physical guarding remain primary. Software safety is one layer, not the only layer.
7. **MotoROS2 `stop_motion_on_disconnect: true` must remain on.** The `motoros2_config.yaml` in this project ships with it on. If anyone proposes turning it off, reject the change.
8. **T-axis runs at 1000 deg/s on hardware.** Any `velocity_scale` that produces a final wrist speed > 200 deg/s on a first hardware run is rejected by review, not by code. `joint_6_t_mode=extended` is a documented opt-in with explicit preconditions; never auto-promote.
9. **Python dependency hygiene for ROS 2 Humble.** ROS 2 Humble links `rclpy` against the system Python 3.10 on Ubuntu 22.04. Random `pip install` (especially of LLM, perception, or auxiliary packages) can upgrade or downgrade libraries that other ROS packages depend on, breaking the whole workspace silently.
   - Per-package `requirements.txt` with **exact** version pins.
   - Install with `pip install --user --requirement requirements.txt`. Never `sudo pip install`. On Ubuntu 23.04+ (PEP 668), use `pipx` or `--break-system-packages` only inside a project-local prefix.
   - Check before installing Pydantic: `python3 -c 'import pydantic; print(pydantic.VERSION)'`. Some ROS packages depend on Pydantic v1 — pin accordingly or use a v2-compatible wrapper.
   - After every install: smoke-test `python3 -c 'import rclpy; print("OK")'`. If this breaks, the install is bad — roll back immediately.
   - Do **not** activate a Python venv inside a sourced ROS workspace. Without `--system-site-packages`, `rclpy` imports fail; with it, the isolation is incomplete and confusing.
   - For full isolation (e.g. complex LLM stack with conflicting deps), use a container (Docker / Distrobox / pixi). This is more disruptive but does not break ROS.

---

## §9. AGENT RESPONSE CONTRACT (every reply must include)

When the agent responds during any phase, the reply must contain, in this order:

1. **Phase declared:** `Running PHASE <N>` (and refuse if any earlier phase is unverified).
2. **Discovery output pasted** (relevant `find` / `rg` / `ros2 ...` results, with each command echoed as `$ <cmd>` immediately above its output).
3. **Plan:** what files will change, what won't, why.
4. **Diffs:** unified diff format, one diff per file.
5. **Verification commands actually run** (with output).
6. **Risks / safety notes:** sim vs hardware delta, what is *not* yet validated.
7. **Reliability tag (text labels, no emoji):**
   - `[VERIFIED]` — discovered the file, ran the test, output shown
   - `[NEEDS-VALIDATION]` — change made; verification depends on the human running it on the workspace / hardware
   - `[KNOWN-GAP]` — could not complete; explicit reason and the discovery command needed to unblock

8. **Stop signal:** `End of PHASE <N>. Awaiting review before PHASE <N+1>.`

---

## §10. WHAT THE AGENT MUST NEVER DO

- Invent ROS 2 topic, service, action, parameter, node, frame, package, or API names
- Treat anything in this document as runtime ground truth without re-verifying via discovery
- Conflate planning with execution (and never rename `submit_motion` to anything that suggests execution)
- Skip the safety gate "for a quick test"
- Hard-delete code in a phase other than P5
- Combine refactor + behaviour change in the same commit
- Assume folder names that aren't in §1 (verified) — those are placeholders that must be discovered
- Proceed past a discovery gap
- Hardcode `calibration_date`, `velocity_scale`, joint limits, blend radius, depth noise threshold, or any other safety/perception value in source code — all such values live in YAML SSOT
- Silently fall back to `computeCartesianPath` if Pilz is missing; silently skip a waypoint; silently retry on hardware alarms
- Recommend a Python venv inside a sourced ROS workspace as a way to isolate LLM dependencies — this breaks `rclpy`

---

**End of MASTER_AGENT_HARNESS_v2.md.**
*Acknowledge by stating the phase you are about to execute and pasting the §3 discovery output.*
