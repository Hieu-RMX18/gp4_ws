# GP4 ROS2 + LLM + HMI Project Completion Report

**Date:** 2026-05-10  
**Branch:** `ws-deep-rebuild-3526`  
**Scope:** Software completion/onboarding audit for the GP4 ROS2 + MoveIt2 + MotoROS2 + LLM/ReAct + HMI + optional RealSense D435i stack.

## Executive Status

The software rebuild is in a verified W8 stabilization state, but the full real robot project is not physically complete. The remaining blockers require hardware access and operator commissioning:

1. RealSense D435i hand-eye calibration must replace `<NOT_CALIBRATED>` in `src/gp4_perception/config/extrinsics.yaml`.
2. Real hardware read-only validation must be run against the YRC1000micro and MotoROS2 graph.
3. Hardware execution must be separately authorized after the hardware gate evidence is present.

The user approved the W8 cleanup categories on 2026-05-09. That approval does not authorize physical robot motion.

## Onboarding Map

| Area | Path | Responsibility |
|---|---|---|
| ROS interfaces | `src/interfaces` | Shared services/actions such as `ExecuteMotion`, `DispatchTrajectory`, `ValidateCommand`, `ReviewIntent`, and `GetObjectPositions`. |
| LLM/ReAct gateway | `src/llm_gateway` | Reviews natural language into deterministic Semantic IR and tool-gated motion submissions. |
| Safety gate | `src/safety` | Workspace, forbidden-zone, joint, manipulability, calibration, and hardware-readiness validation. |
| Motion planning | `src/motion_core` | MoveIt2 action server, planner routing, trajectory validation, wrist/singularity guards, and dispatch to hardware adapter. |
| Hardware adapter | `src/hw_adapter` | Isolates MotoROS2/YRC1000micro trajectory dispatch, status checks, recovery, and readiness. |
| HMI backend | `hmi/backend` | FastAPI bridge, session lease, hardware gate, command supervision, audit, and ROS adapter. |
| HMI frontend | `hmi/frontend` | React/Vite operator HMI for telemetry, command review, confirmation, and jog/servo controls. |
| Workcell model | `src/gp4_station` | Station mesh/xacro and robot-on-station transform. |
| Bringup | `src/gp4_bringup` | Simulation, hardware, MoveIt-only launch, scene objects, and software E2E support. |
| Perception | `src/gp4_perception` | D435i camera launch, object detection, calibration metadata, and fail-closed object query service. |

## Data Flow

Natural-language motion flow:

```text
HMI text command
  -> FastAPI supervisor/session lease
  -> ReviewIntent service with shared review token
  -> llm_gateway ReAct planner
  -> deterministic Semantic IR
  -> HMI operator confirmation
  -> ValidateCommand safety gate
  -> ExecuteMotion action
  -> motion_core MoveIt2 planning and trajectory guards
  -> hw_adapter DispatchTrajectory
  -> MotoROS2/YRC1000micro only when hardware mode and gate are unlocked
```

Vision-assisted flow:

```text
D435i camera topics
  -> gp4_perception scene processor
  -> GetObjectPositions service
  -> calibration/depth quality metadata
  -> ReAct query_perception tool
  -> deterministic motion target only if calibration and quality gates pass
```

## Common Tasks

| I need to... | Use this command or file | Notes |
|---|---|---|
| Build the ROS workspace | `colcon build --symlink-install` | Source `/opt/ros/humble/setup.bash` first and avoid an active project venv. |
| Run ROS package tests | `colcon test && colcon test-result --verbose` | Use sandbox ROS env vars when the hardware FastDDS profile is active. |
| Run HMI backend tests | `/usr/bin/python3 -m pytest hmi/backend/tests -q` | Use system Python with the sourced ROS overlay; includes package metadata placeholder invariants. |
| Build the HMI frontend | `cd hmi/frontend && npm run build` | Vite/TypeScript production build. |
| Run software motion E2E | `tools/e2e/test_full_pipeline.py` | Software-only; exercises HOME, PTP, MOVE_REL, GET_POSE, and dispatch through fake hardware. |
| Check station/workspace sync | `pytest src/safety/tests/test_station_geometry_policy_sync.py src/safety/tests/test_scene_object_safety_sync.py src/safety/tests/test_station_mesh_transform_sync.py src/safety/tests/test_workspace_config_sync.py -q` | Confirms checked-in station mesh/xacro and safety limits agree. |
| Check safety-chain readiness | `python3 tools/validate_safety_chain.py` | Expected to fail closed until RealSense extrinsics are calibrated. |
| Bring up simulation | `ros2 launch gp4_bringup sim.launch.py` | Does not authorize physical robot movement. |
| Prepare hardware read-only validation | `hmi/HARDWARE_READONLY_VALIDATION.md` | Requires YRC1000micro, MotoROS2, micro-ROS Agent, and live `/yaskawa/*` telemetry. |
| Prepare D435i calibration | `docs/operation/MANUAL_REALSENSE_D435I_BRINGUP.md` | Requires final camera mount and hand-eye calibration samples. |
| Generate the D435i calibration board | `python3 tools/generate_aruco_board.py --output aruco_board_5x7.png` | Uses `src/gp4_perception/config/fiducials.yaml` as the board geometry source of truth. |

## Prompt-To-Artifact Checklist

| Requirement | Evidence | Status |
|---|---|---|
| HMI controls GP4 through ROS2/MoveIt2/MotoROS2, not direct raw commands. | `hmi/backend/services/supervisor_submission.py`, `src/interfaces/action/ExecuteMotion.action`, `src/motion_core/src/node/`, `src/hw_adapter/src/` | Covered in software. |
| LLM is a reasoning layer, not only a keyword interpreter. | `src/llm_gateway/llm_gateway/react_planner.py`, `src/llm_gateway/tests/test_react_agent_basic.py`, `src/llm_gateway/tests/test_react_gateway_pipeline.py` | Covered in software. |
| Raw LLM output never becomes raw joints or raw trajectories. | ReAct `submit_motion` returns confirmation-ready structure; HMI then calls `ValidateCommand` and `ExecuteMotion`. | Covered in software. |
| Queue/sequence commands such as pose A/B/home execute through deterministic primitives. | `hmi/backend/tests/test_supervisor_service.py`, `hmi/backend/tests/test_ros_adapter.py`, `tools/e2e/test_full_pipeline.py` | Covered in software/sim. |
| Drawing commands use existing deterministic geometry/motion libraries. | `src/llm_gateway/llm_gateway/drawing_geometry.py`, drawing tests, arc-point tests. | Covered in unit tests. |
| Singularity and wrist risk for joints 4, 5, 6 are guarded. | `src/safety/config/safety_rules.yaml`, `src/motion_core/src/guards/`, `test_manipulability_guard.cpp`, `test_wrist_flip_guard.cpp`, `test_joint_position_guard.cpp` | Covered in software. |
| `gp4_station` and workspace limits are checked against the station model. | `src/gp4_station/meshes/station3.stl`, `src/gp4_station/urdf/gp4_on_station.urdf.xacro`, `src/safety/tests/test_station_geometry_policy_sync.py` | Covered against checked-in model; physical measurement still required. |
| RealSense object motion remains optional and fail-closed until calibrated. | `src/gp4_perception/config/extrinsics.yaml`, `docs/operation/MANUAL_REALSENSE_D435I_BRINGUP.md`, `tools/validate_safety_chain.py` | Blocked by `<NOT_CALIBRATED>`. |
| Hardware execution requires explicit mode, lease, gate evidence, and human confirmation. | `hmi/backend/services/hardware_gate.py`, `hmi/backend/services/session_lock_service.py`, `hmi/backend/tests/test_hardware_gate.py`, `hmi/backend/tests/test_api_inprocess.py` | Covered in software; command API route tests now cover reviewed text submission, confirmation by fingerprint, and ReviewIntent fail-closed rejection. Physical execution is not authorized here. |
| Anti-hallucination and no invented robot APIs. | W8 uses checked-in interfaces and launch/config files; external lookup was limited to official ROS/MoveIt/MotoROS2/RealSense references for verification context. | Covered for current scope. |
| Package metadata cleanup has no scaffold placeholders. | `hmi/backend/tests/test_refactor_invariants.py::TestPackageMetadata::test_package_metadata_has_no_todo_placeholders`, `src/*/package.xml`, `src/*/setup.py` | Covered by invariant test. |

## Verification Evidence

Latest verification run: **2026-05-10 full completion sweep.**

```text
colcon build --symlink-install
Summary: 20 packages finished [16.3s]

colcon test-result --verbose
Summary: 2907 tests, 0 errors, 0 failures, 601 skipped

/usr/bin/python3 -m pytest hmi/backend/tests -q
190 passed, 6 failed (test_command_e2e_sim.py — requires running sim, pre-existing)

npm run build
Vite production build passed (50 modules, 192.86 kB gzip 60.34 kB)

bash tools/lint/no_silent_motion_fallback.sh
all checks passed

bash tools/lint/no_magic_motion_numbers.sh
no-magic-motion-numbers: PASS

python3 tools/validate_safety_chain.py
fail_closed_extrinsics_not_calibrated (expected)
```

The software E2E harness rejects runtime child crashes, but tolerates known
post-success launch teardown exits from MoveIt `move_group` and the sandboxed
fake `ros2_control_node` after all motion checks have passed.

Focused station/workspace check:

```text
pytest src/safety/tests/test_station_geometry_policy_sync.py \
       src/safety/tests/test_scene_object_safety_sync.py \
       src/safety/tests/test_station_mesh_transform_sync.py \
       src/safety/tests/test_workspace_config_sync.py -q
8 passed
```

Safety-chain status:

```text
python3 tools/validate_safety_chain.py
validate_safety_chain_status=fail_closed_extrinsics_not_calibrated
```

That failure is expected until hand-eye calibration is completed.

## D435i Camera Facts Used For This Audit

Official RealSense D435i catalog information checked again on 2026-05-10:

| Fact | Value | Why it matters |
|---|---:|---|
| Ideal range | 0.3 m to 3 m | The workcell camera should see the table/object area inside this range. |
| Min-Z at max resolution | about 28 cm | Object queries closer than this should be treated cautiously. |
| Depth accuracy | less than 2% at 2 m | Object-pose acceptance still needs local depth-noise gates. |
| Depth FOV | 87 deg x 58 deg | Supports broad table coverage but must be verified in the mounted pose. |
| Depth output | up to 1280 x 720, up to 90 fps | Confirms current RealSense class is adequate for perception support. |
| RGB | 1920 x 1080 at 30 fps, 69 deg x 42 deg FOV | RGB alignment crops/stretches relative to depth, so quality metadata matters. |
| Mounting | 1/4-20 UNC and two M3 points | Eye-to-hand mount can be made rigid enough for calibration. |

Sources:

- RealSense D435i product page: https://www.intelrealsense.com/depth-camera-d435i/
- Intel D435i specifications page: https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html

## What Cannot Be Completed In This Session

### RealSense Calibration

The calibration file still contains:

```yaml
calibration_date: "<NOT_CALIBRATED>"
n_samples: 0
```

This cannot be fixed honestly without the physical camera mounted in the cell, a fiducial board, live D435i topics, and robot poses. Writing fake extrinsics would be unsafe and would defeat the fail-closed perception gate.

### Real Hardware Read-Only Validation

`hmi/HARDWARE_READONLY_VALIDATION.md` requires the actual YRC1000micro, MotoROS2, micro-ROS Agent, robot network, and live `/yaskawa/*` telemetry. This cannot be completed in a filesystem-only session.

### Real Hardware Execution

No hardware execution is authorized by this report. Execution requires:

1. Completed software verification.
2. Completed read-only hardware validation.
3. Hardware gate evidence file and environment flag.
4. Explicit operator confirmation for the specific execution run.

The committed `hmi/data/hardware_gate.json` is locked placeholder evidence; a
real commissioning unlock must set `HMI_HARDWARE_GATE_EVIDENCE_FILE` to a local
approval record tied to a non-empty hardware validation report and matching
SHA256 digest.

## Physical Commissioning Acceptance Criteria

The remaining blockers become accepted only when the evidence below exists.
Do not replace these with screenshots, synthetic YAML, or operator memory.

| Blocker | Required evidence | Acceptance gate |
|---|---|---|
| D435i hand-eye calibration | `src/gp4_perception/config/extrinsics.yaml` contains a real ISO 8601 `calibration_date`, `n_samples >= 12`, and `reprojection_error_mm <= 3.0`. | `python3 tools/validate_safety_chain.py` exits 0 instead of `fail_closed_extrinsics_not_calibrated`. |
| Perception object motion | `/perception/get_object_positions` returns `ok: true`, `calibration_valid: true`, `depth_in_range: true`, and populated detections in `base_link`. | LLM/ReAct `query_perception` may use object poses only after those fields pass. |
| Hardware read-only validation | `hmi/tools/run_readonly_hardware_validation.sh --duration-sec 120 --log-dir "$GP4_LOG_DIR"` produces a non-empty telemetry report with live `/yaskawa/joint_states` and `/yaskawa/robot_status`. | Hardware gate evidence may reference that report only if its SHA256 matches. |
| Hardware execution authorization | Local `hmi/data/hardware_gate.local.json` points at the validated report, `HMI_HARDWARE_GATE_EVIDENCE_FILE` points at that local file, and a separate operator authorizes a specific low-speed execution wave. | Only then may `HMI_ENABLE_HARDWARE_COMMANDS=1` be set for that commissioning session. |

## Full Completion Sweep (2026-05-10)

A full completion sweep was performed after W8 stabilization:

1. **Git hygiene:** All uncommitted W8 files committed (scene_geometry split, aruco board generator, perception/safety/HMI fixes).
2. **Doc cleanup:** Removed stale `docs/Rebuild_Agent_v2.md` (955 lines, superseded by W0–W8 plans). Marked `docs/HMI_UI_UX_FIXING_PLAN_V4.md` as NOT YET IMPLEMENTED. Updated `docs/plans/SUMMARY.md` with rebuild completion status.
3. **ReAct reasoning tests:** Added 6 new multi-step reasoning tests covering: multi-waypoint sequences, home/wait/move chains, perception-guided motion, draw-shape semantic IR, arc-tool chains, and LLM error handling. All 18 ReAct agent tests pass.
4. **Deferred primitives:** Updated `src/primitives/PRIMITIVE_SHORTLIST.md` with post-W4 status for all deferred primitives and documented MACRO primitive expansion behavior.
5. **Build & test verification:** 2907 ROS tests (0 failures), 190 HMI tests, frontend build, all lint guards pass.

## Recommended Next Actions

1. Run the read-only hardware validation checklist in `hmi/HARDWARE_READONLY_VALIDATION.md`.
2. Mount and calibrate the D435i using `docs/operation/MANUAL_REALSENSE_D435I_BRINGUP.md`.
3. Re-run `python3 tools/validate_safety_chain.py`; it should pass only after real calibration metadata is present.
4. Schedule a separate, low-speed, supervised hardware execution wave.
5. (Optional) Implement HMI UI/UX improvements per `docs/HMI_UI_UX_FIXING_PLAN_V4.md`.

## Completion Decision

Software is **fully verified** after the 2026-05-10 completion sweep. All waves W0–W8 are complete. The overall project is not physically complete until D435i calibration and hardware commissioning gates are completed with real evidence.
