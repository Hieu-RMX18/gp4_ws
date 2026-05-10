# W8 — Second Cleanup Wave: Stabilize Reorg, Verify Budgets, Close Drift

**Wave class:** Maintenance / stabilization
**Risk:** Low
**Scheduled date:** 2026-05-19
**Early execution note:** This file also documents the current W8 stabilization work already present on `ws-deep-rebuild-3526`.
**Depends on:** W0 through W7 complete

---

## Goal

W8 keeps the rebuild maintainable after the W6/W7 changes by validating the current file splits, closing obvious drift, and preserving the existing fail-closed safety behavior. It does not add robot behavior or enable hardware execution. The reviewed text ingress service `interfaces/srv/ReviewIntent.srv` replaces deprecated direct topic/CLI ingress for HMI text commands, and `interfaces/srv/GetObjectPositions.srv` is expanded with perception quality metadata so calibrated-object responses expose freshness and depth confidence. `ReviewIntent` carries HMI session/operator/command metadata and a shared `GP4_REVIEW_INTENT_TOKEN`; the gateway derives routing mode from its own runtime mode instead of trusting the request. The HMI HTTP servo endpoints also intentionally require `sessionId`, `operatorId`, and `leaseToken` request bodies so servo START/HOLD stays behind the controller lease and hardware gate.

---

## Tasks

### W8.T1 — Verify Current Reorg

- Confirm the moved `hw_adapter` and `motion_core` C++ source files are referenced through `CMakeLists.txt`.
- Confirm deleted legacy `llm_gateway` modules are replaced by `intent_engine.py`, `react_planner.py`, and `ReviewIntent` without reintroducing old console entry points.
- Keep migration reports under `docs/MIGRATION-W*.md`; root-level `MIGRATION-W*.md` files remain removed.

### W8.T2 — Test-Only HMI ROS Interface Fakes

- Keep production HMI dispatch fail-closed when ROS interfaces are unavailable.
- In HMI backend tests, use local fake `ExecuteMotion.Goal`, `SequenceStep`, and `Pose` objects so `_build_execute_motion_goal()` can be validated without a sourced ROS interface package.
- Verify BLENDED_SEQUENCE maps every step into the ExecuteMotion goal shape.

### W8.T3 — Hygiene and Drift Closure

- Restore accidental config drift in `src/llm_gateway/config/llm.yaml` unless a human explicitly approves the changed value.
- Fix `git diff --check` whitespace issues.
- Replace scaffold package metadata placeholders in ROS package manifests/setup
  files with the workspace's existing maintainer/license convention.
- Ignore local generated artifacts and convenience symlinks that should not be committed: HMI SQLite audit files and `src/` symlinks to reference dependency trees.
- Document the HMI HTTP servo body contract change and cover it with backend route-level tests.

### W8.T4 — Verification

- Run the required harness: `git branch --show-current`, `colcon list`, and `git status --short`.
- Run `colcon build --symlink-install`.
- Run ROS tests with a local sandbox environment when the hardware FastDDS profile is active:
  `env -u FASTRTPS_DEFAULT_PROFILES_FILE ROS_LOG_DIR=/tmp/gp4_ws_ros_log ROS_HOME=/tmp/gp4_ws_ros_home ROS_LOCALHOST_ONLY=1 colcon test`
- Run `colcon test-result --verbose`.
- Run HMI backend tests: `pytest hmi/backend/tests -q`.
- Run HMI frontend build: `npm run build` from `hmi/frontend`.
- Run the standalone software-only full-pipeline E2E after sourcing the workspace:
  `env -u FASTRTPS_DEFAULT_PROFILES_FILE ROS_LOG_DIR=/tmp/gp4_ws_ros_log ROS_HOME=/tmp/gp4_ws_ros_home ROS_LOCALHOST_ONLY=1 python3 tools/e2e/test_full_pipeline.py`.
- Run guards: `git diff --check`, `bash tools/lint/no_silent_motion_fallback.sh`, `bash tools/lint/no_magic_motion_numbers.sh`, `bash tools/lint/file_size_budget.sh`, and `python3 tools/lint/aged_deprecation_check.py`.
- Run `python3 tools/validate_safety_chain.py`; the only acceptable failure is the known fail-closed `<NOT_CALIBRATED>` perception extrinsics state.

## Human Approval Checkpoint

**Approval status:** Approved by the user for W8 software cleanup continuation on
2026-05-09. This approval covers the cleanup categories below only. It does not
authorize RealSense hand-eye calibration results, real-hardware execution, or
any bypass of the hardware gate.

Before committing W8, the reviewer must approve these cleanup categories:

1. `MIGRATION-W*.md` root files are rehomed to `docs/MIGRATION-W*.md`.
2. GP4 datasheet PDFs are rehomed from the workspace root to `references/*.pdf` and remain tracked.
3. `hw_adapter` and `motion_core` monolithic source files are replaced by same-package subdirectories (`dispatch/`, `monitoring/`, `node/`, `session/`, `execution/`, `guards/`, and `planning/`).
4. Legacy `llm_gateway` helper modules are consolidated into `intent_engine.py` and `react_planner.py`; old CLI and benchmark entry points are removed.
5. `src/gp4_moveit_config/launch/real_robot.launch.py` remains removed only if `src/gp4_bringup/launch/hw.launch.py` is the approved hardware bringup entrypoint.

### Commit Inclusion Checklist

Before creating a commit or PR, include these currently untracked W8 files with the tracked edits; otherwise the verified diff is not reproducible from a clean checkout:

- Rehomed migration reports: `docs/MIGRATION-W0.md` through `docs/MIGRATION-W7.md`.
- Rehomed GP4 datasheets: `references/DS_GP4.pdf` and `references/Flyer_Robot_GP4_E_05.2022.pdf`.
- Pinned external workspace dependency manifest: `references/gp4_ws_dependencies.repos`.
- W8 plan and software E2E coverage: `docs/plans/W8_second_cleanup_wave.md`, `src/gp4_bringup/test/test_moveit_only_launch.py`, and `tools/e2e/test_full_pipeline.py`.
- HMI/API coverage: `hmi/backend/tests/test_api_inprocess.py`.
- Reviewed text ingress interface: `src/interfaces/srv/ReviewIntent.srv`.
- Consolidated LLM gateway modules/tests: `src/llm_gateway/llm_gateway/intent_engine.py`, `src/llm_gateway/llm_gateway/react_planner.py`, `src/llm_gateway/tests/test_react_gateway_pipeline.py`, and `src/llm_gateway/tests/test_react_tools/test_motion_tools.py`.
- `hw_adapter` split sources under `src/hw_adapter/src/common.hpp`, `dispatch/`, `monitoring/`, `node/`, and `session/`.
- `motion_core` split sources under `src/motion_core/src/execution/`, `guards/`, `node/`, and `planning/`.

## Current Completion Audit

Code/simulation evidence collected for this W8 state:

| Requirement | Evidence |
|---|---|
| HMI local parser stays narrow | `hmi/backend/tests/test_supervisor_service.py` focused review/local-fallback tests pass; complex motion text is rejected when gateway review is unavailable. |
| LLM/ReAct produces Semantic IR before primitives | `src/llm_gateway/tests` pass in source mode; generated `ReviewIntent` contract test passes when the workspace is sourced. |
| HMI HTTP command and servo contracts are lease-gated | `hmi/backend/tests/test_api_inprocess.py` covers required command/servo request fields, reviewed text command submission, operator confirmation by plan fingerprint, ReviewIntent fail-closed rejection, controller-lease rejection, and valid hardware-gated START/HOLD route dispatch. |
| ROS execution path works in simulation | `tools/e2e/test_full_pipeline.py` passes after staging through SRDF `poseB`: `HOME` -> `GET_POSE` -> `PTP software staging` -> `GET_POSE` -> bounded `MOVE_REL` -> `GET_POSE` -> current-pose `PTP`. Runtime child crashes still fail the E2E; the harness only tolerates known post-success launch teardown exits (`-11`, `SIGTERM`, or `SIGKILL`) from MoveIt `move_group` and the sandboxed fake `ros2_control_node`. `llm_gateway_node` has regression coverage for externally-triggered rclpy shutdown and must exit cleanly. `motion_core_node` has explicit shutdown grace in sim launch so it must exit cleanly instead of being SIGKILL-tolerated. The conservative `ManipulabilityGuard` floor remains unchanged at `0.05`. |
| Safety gates remain fail-closed | `python tools/validate_safety_chain.py` returns the documented fail-closed extrinsics status and no other safety-chain error. |
| Reorg compiles and tests | `colcon build --symlink-install`, sandboxed `colcon test`, and `colcon test-result --verbose` report 2896 tests, 0 errors, 0 failures, 601 skipped. |
| HMI frontend/backend remain buildable | `pytest hmi/backend/tests -q` reports 188 passed, 6 skipped. `npm run build` from `hmi/frontend` passes. TCP-socket HMI E2E tests are present but skipped in this sandbox. |

Original GP4 + LLM + vision objective coverage:

| Objective | Current W8 evidence | Status / remaining blocker |
|---|---|---|
| Natural-language operator text is reviewed into deterministic structure before any robot command is built. | `interfaces/srv/ReviewIntent.srv`; `src/llm_gateway/llm_gateway/llm_gateway_node.py`; `src/llm_gateway/llm_gateway/react_planner.py`; `hmi/backend/tests/test_supervisor_service.py`; `src/llm_gateway/tests/test_react_gateway_pipeline.py` | Covered in software. Direct gateway topic execution stays disabled by default and HMI text ingress uses token-protected `ReviewIntent`; hardware runtime rejects review requests unless `GP4_REVIEW_INTENT_TOKEN` is configured and matched. |
| Raw LLM output never becomes raw joint commands, raw trajectories, or direct MotoROS2 calls. | `src/llm_gateway/llm_gateway/react_planner.py`; `hmi/backend/ros/command_dispatch.py`; `src/safety/safety/execution_gate.py`; `src/motion_core/src/node/motion_core_node.cpp`; `src/hw_adapter/src/node/hw_adapter_node.cpp` | Covered in software. ReAct `submit_motion` returns `READY_FOR_CONFIRM`; execution still crosses HMI confirmation, `ValidateCommand`, `ExecuteMotion`, `motion_core`, and `hw_adapter`. |
| Every motion-capable task crosses a fail-closed safety gate before planning/execution. | `src/safety/safety/execution_gate.py`; `src/safety/safety/command_validator.py`; `src/safety/safety/workspace_guard.py`; `src/safety/config/safety_rules.yaml`; `tools/validate_safety_chain.py`; `src/safety/tests/` | Covered, with the documented expected fail-closed perception extrinsics result until calibration is complete. |
| MoveIt2 plans for the GP4 arm using the known group, joint set, and conservative limits. | `src/gp4_moveit_config/config/motoman_gp4.srdf`; `src/gp4_moveit_config/config/joint_limits.yaml`; `src/motion_core/include/motion_core/primitive_router_dispatch.hpp`; `src/motion_core/src/planning/planner_router.cpp`; `tools/e2e/test_full_pipeline.py` | Proven in software/simulation only. Real MoveIt-to-hardware execution is still not claimed. |
| MotoROS2/YRC1000micro execution is isolated behind a hardware adapter, not called by LLM or HMI code. | `src/motion_core/src/node/motion_core_node.cpp`; `src/hw_adapter/include/hw_adapter/backend_capabilities.hpp`; `src/hw_adapter/src/dispatch/trajectory_executor.cpp`; `src/hw_adapter/src/session/motoros2_session_manager.cpp`; `hmi/HARDWARE_READONLY_VALIDATION.md` | Adapter path is present and guarded. Real hardware validation remains read-only until a separate authorized hardware execution pass. |
| RealSense D435i perception can support object queries only after calibration and quality gates pass. | `src/gp4_perception/launch/camera.launch.py`; `src/gp4_perception/gp4_perception/scene_processor.py`; `src/gp4_perception/gp4_perception/query_perception_tool.py`; `src/gp4_perception/config/extrinsics.yaml`; `docs/operation/MANUAL_REALSENSE_D435I_BRINGUP.md` | Blocked by `calibration_date: "<NOT_CALIBRATED>"`. Detection queries are expected to reject fail-closed until hand-eye calibration is completed. |
| Hardware execution requires explicit hardware mode, controller lease, operator confirmation, and hardware-gate evidence. | `hmi/backend/services/hardware_gate.py`; `hmi/backend/services/session_lock_service.py`; `hmi/backend/services/supervisor_validation.py`; `hmi/backend/services/supervisor_execution.py`; `hmi/backend/tests/test_api_inprocess.py`; `hmi/backend/tests/test_hardware_gate.py` | Covered in software. Gate remains locked unless `HMI_ENABLE_HARDWARE_COMMANDS` and approved evidence both pass; the checked-in evidence file is locked by default, `HMI_HARDWARE_GATE_EVIDENCE_FILE` may point at a local approval record, and placeholder reports such as `/dev/null` are rejected. |
| Robot-specific constants stay aligned: `/yaskawa`, `gp4_arm`, `tool0`, and GP4 joint names. | `src/hw_adapter/include/hw_adapter/backend_capabilities.hpp`; `src/gp4_moveit_config/config/motoman_gp4.srdf`; `src/gp4_moveit_config/config/ros2_controllers.yaml`; `hmi/HARDWARE_READONLY_VALIDATION.md` | Covered by config and validation docs. Real TCP offset remains `tool0` until measured and approved. |

Prompt-to-artifact audit checklist:

| Prompt requirement | Concrete artifact / gate | Current status |
|---|---|---|
| Natural-language HMI input must be reviewed by LLM/gateway, not accepted as external structured robot commands. | `interfaces/srv/ReviewIntent.srv`; `hmi/backend/services/supervisor_submission.py`; `hmi/backend/tests/test_api_inprocess.py::test_command_intent_route_review_and_confirm_flow`; `hmi/backend/tests/test_api_inprocess.py::test_command_intent_route_fails_closed_when_review_intent_rejects`; `hmi/backend/tests/test_api_inprocess.py::test_command_intent_contract_rejects_structured_intent_ingress`; `hmi/backend/tests/test_supervisor_service.py::test_external_structured_semantic_intent_is_rejected`; `src/llm_gateway/tests/test_react_gateway_pipeline.py::test_review_intent_rejects_missing_review_token_before_llm_call` | Covered in software. |
| LLM must be a ReAct-style reasoner with tools, not only a keyword interpreter. | `src/llm_gateway/llm_gateway/react_planner.py` ReAct prompt/tool loop; tools for current pose, planning, perception, speed, wait, and submit; `src/llm_gateway/tests/test_react_agent_basic.py`; `src/llm_gateway/tests/test_react_gateway_pipeline.py`; `src/llm_gateway/tests/test_react_tools/test_motion_tools.py` | Covered as a software architecture. Live external model quality is not claimed in W8. |
| Raw LLM output must not become raw joints, raw trajectories, or direct MotoROS2 calls. | `SubmitMotionTool` returns `READY_FOR_CONFIRM`; HMI confirmation path uses `ValidateCommand` then `ExecuteMotion`; MotoROS2 stays behind `motion_core` and `hw_adapter`. | Covered in software. |
| Commands like `poseA -> poseB -> home`, queue/sequence execution, and continued trajectory segments must work through deterministic primitives. | `hmi/backend/tests/test_api_inprocess.py::test_reviewed_named_pose_sequence_api_confirms_without_tcp_socket`; `hmi/backend/tests/test_supervisor_service.py::test_gateway_review_named_pose_sequence_confirms_in_order`; `hmi/backend/tests/test_supervisor_service.py::test_sequence_confirm_executes_child_steps_in_order`; `hmi/backend/tests/test_ros_adapter.py::test_blended_sequence_maps_steps_to_execute_motion_goal` | Covered in software. |
| Motion library must include basic MoveIt-friendly primitives before complex motion. | `src/llm_gateway/tests/test_intent_router.py`; `src/llm_gateway/tests/test_semantic_validator.py`; `src/motion_core/src/planning/primitive_router_dispatch.cpp`; `tools/e2e/test_full_pipeline.py` exercises HOME, PTP, MOVE_REL, GET_POSE through MoveIt/hw_adapter simulation. | Covered in software/simulation for representative primitives. |
| Draw circle/radius and shape/alphabet motions must compile to existing deterministic motion, not generated robot code. | `src/llm_gateway/tests/test_draw_shape.py`; `src/llm_gateway/tests/test_draw_text.py`; `src/llm_gateway/tests/test_drawing_geometry.py`; `src/llm_gateway/tests/test_react_tools/test_compute_arc_points.py` | Covered in unit tests. Full physical drawing accuracy is not claimed. |
| Prevent singularity and dangerous wrist behavior, especially joints 4, 5, and 6. | `src/safety/config/safety_rules.yaml` operational joint limits, manipulability floor, cumulative rotation limits; `src/motion_core/src/guards/`; `src/motion_core/test/test_joint_position_guard.cpp`; `src/motion_core/test/test_manipulability_guard.cpp`; `src/motion_core/test/test_wrist_flip_guard.cpp`; `src/safety/tests/test_gp4_safety_check.py` | Covered in software guards and tests. Real hardware validation remains required. |
| Check `gp4_station` and workspace limits against the real station model. | `src/gp4_station/urdf/gp4_on_station.urdf.xacro`; `src/gp4_station/meshes/station3.stl`; `src/safety/config/safety_rules.yaml`; `src/safety/tests/test_station_geometry_policy_sync.py`; `src/safety/tests/test_scene_object_safety_sync.py`; `src/safety/tests/test_station_mesh_transform_sync.py` | Covered against the checked-in station mesh/xacro. Physical remeasurement is still required before hardware execution. |
| Vision can move to a red circle/object only after calibrated perception passes quality gates. | `src/gp4_perception/gp4_perception/scene_processor.py`; `src/gp4_perception/gp4_perception/query_perception_tool.py`; `src/gp4_perception/test/test_scene_processor.py`; `src/llm_gateway/tests/test_react_tools/test_motion_tools.py`; `src/gp4_perception/config/extrinsics.yaml` | Blocked fail-closed by `<NOT_CALIBRATED>` extrinsics. |
| Hardware execution must require runtime hardware mode, controller lease, operator confirmation, and evidence. | `hmi/backend/services/hardware_gate.py`; `hmi/backend/services/session_lock_service.py`; `hmi/backend/services/supervisor_execution.py`; `hmi/backend/tests/test_hardware_gate.py`; `hmi/backend/tests/test_api_inprocess.py` | Covered in software; real hardware execution remains forbidden until separately authorized, with a local non-empty hardware report evidence file. |
| Anti-hallucination: no invented topics/APIs and no new dependencies without approval. | W8 uses existing ROS interfaces/packages plus reviewed `ReviewIntent.srv` and the documented `GetObjectPositions.srv` perception-quality metadata expansion. External lookup was limited to official ROS/MoveIt/MotoROS2/RealSense references for verification context. | Covered for W8 scope. |
| Cleanup placeholders must not remain in ROS package metadata. | `hmi/backend/tests/test_refactor_invariants.py::TestPackageMetadata::test_package_metadata_has_no_todo_placeholders`; `src/*/package.xml`; `src/*/setup.py` | Covered by invariant test. |

W8 is not project-complete until:

1. The human approves the cleanup categories above.
2. RealSense hand-eye extrinsics are calibrated per `docs/operation/MANUAL_REALSENSE_D435I_BRINGUP.md`.
3. Real-hardware read-only validation is completed per `hmi/HARDWARE_READONLY_VALIDATION.md`.
4. Any hardware execution run is separately authorized with hardware gate evidence and operator approval.

---

## Don't

- Do not call real hardware.
- Do not bypass safety-chain validation.
- Do not add further ROS msg/srv/action contract changes beyond the reviewed `ReviewIntent` ingress service and the `GetObjectPositions` perception-quality response metadata.
- Do not expose servo START/HOLD through body-less HMI HTTP calls; these endpoints must keep `sessionId`, `operatorId`, and `leaseToken`.
- Do not commit generated SQLite audit databases, build products, or local dependency symlinks.
- Do not reintroduce deprecated LLM CLI or benchmark entry points.

---

## Output

State explicitly whether W8 verification is clean, and list any remaining fail-closed blockers such as uncalibrated perception extrinsics.
