# GP4 Workspace Deep-Clean — Audit Findings Ledger

**Date:** 2026-04-26  
**Branch:** `chore/workspace-deep-clean-2026-04-26`  
**Auditor:** Cascade (automated trace) + Hieu (principal review)

---

## 6.1 LLM gateway → safety

| ID | Layer | Risk | Severity | Evidence (file:line) | Recommendation |
|----|-------|------|----------|----------------------|----------------|
| A-01 | llm_gateway | Natural language never bypasses validation | info | `llm_gateway_node.py:265-267` — every routed command passes `_schema_validator.validate()` + `_normalize_and_validate()` before dispatch | No action. Fail-closed confirmed. |
| A-02 | llm_gateway | ValidateCommand service called for every dispatch | info | `llm_gateway_node.py:512-521` — `_build_validate_request()` constructs the service request; `_on_validation_done` rejects on `response.valid == False` | No action. |
| A-03 | llm_gateway | Error/reject branch always publishes `rejected` status | info | `llm_gateway_node.py:195,209,228,254,278,290` — every exception branch calls `_reject()` | No action. |
| A-04 | safety | CommandValidator clamps velocity/acceleration | info | `command_validator.py:56-57,72-76` — rejects if `> max_velocity_scale (0.06)` | No action. |
| A-05 | safety | WorkspaceGuard checks workspace bounds + forbidden zones | info | `workspace_guard.py:109-134` — AABB check for both bounds and 3 forbidden zones | No action. |
| A-06 | safety | ExecutionGate fail-closed readiness check | info | `execution_gate.py:79-92` — rejects ALL commands (except ALARM_RESET) when `is_robot_ready == False` | No action. |
| A-07 | safety | MOVE_REL delta gate (defense-in-depth) | info | `execution_gate.py:225-257` — max delta norm 0.05 m, base_link only, non-zero required | No action. |
| A-08 | safety | CIRC + CARTESIAN_PATH waypoints validated against WorkspaceGuard | info | `execution_gate.py:150-214` — each waypoint checked individually | No action. |

---

## 6.2 Safety configuration vs hard caps

| ID | Layer | Risk | Severity | Evidence (file:line) | Recommendation |
|----|-------|------|----------|----------------------|----------------|
| B-01 | safety × motion_core | `max_velocity_scale` consistent end-to-end: 0.06 | info | `safety_rules.yaml:20,25`, `trajectory_post_processor.hpp:16-17`, `hmi/backend/ros/adapter.py` `DEFAULT_MOTION_VELOCITY_SCALE = 0.06` | No action. Contract consistent. |
| B-02 | safety | Forbidden zones include 30 mm margin (size_y=0.06 m = 60 mm total, ±30 mm from wall center) | info | `safety_rules.yaml:30-58` | No action. Matches documented 30 mm margin. |
| B-03 | safety | `max_move_rel_translation: 0.05` in config matches `_MOVE_REL_MAX_DELTA_NORM` fallback | info | `safety_rules.yaml:27`, `execution_gate.py:12` | No action. |

---

## 6.3 motion_core → hw_adapter

| ID | Layer | Risk | Severity | Evidence (file:line) | Recommendation |
|----|-------|------|----------|----------------------|----------------|
| C-01 | motion_core | Wrist-flip guard: per-joint thresholds (J123=25°, J45=45°, J6=30°) checked on every consecutive waypoint pair | info | `wrist_flip_guard.hpp:17-19`, `wrist_flip_guard.cpp:71-108` | No action. More conservative than the legacy single 30° for large joints. |
| C-02 | motion_core | 200-point trajectory cap enforced at TOTG post-processor AND quality gate | info | `trajectory_post_processor.hpp:15`, `quality_gate.hpp:16` | No action. |
| C-03 | motion_core | Default velocity/acceleration scaling = 0.06 at motion_core level | info | `trajectory_post_processor.hpp:16-17` | No action. Matches safety config. |
| **C-04** | **primitives** | **`kCartesianJumpThreshold` is non-zero (1.0 in primitive_lin, 1.5 in primitive_router_dispatch)** | **low** | `primitives/src/primitive_lin.cpp:27`, `motion_core/include/motion_core/primitive_router_dispatch.hpp:110` | **Plan expected 0.0 (disabled). Non-zero values enable MoveIt's jump detection heuristic which can cause path truncation on valid trajectories. Recommend unifying to 0.0 in a separate ticket if jump-detection is not intentional.** |
| C-05 | hw_adapter | Single-goal enforcement: rejects dispatch if `execution_in_progress_ || dispatch_goal_reserved_` | info | `hw_adapter_dispatch.cpp:28`, `hw_adapter_node.cpp:233` | No action. No async motion overlap possible. |
| C-06 | hw_adapter | hw_adapter waits for FollowJointTrajectory result before releasing slot | info | `hw_adapter_node.cpp` `execute_trajectory_internal()` blocks until result or timeout | No action. |
| **C-07** | **primitives × motion_core** | **`kCartesianJumpThreshold` inconsistency: 1.0 vs 1.5 between two compilation units** | **medium** | `primitives/src/primitive_lin.cpp:27` = 1.0, `motion_core/include/motion_core/primitive_router_dispatch.hpp:110` = 1.5 | **Two different thresholds for the same MoveIt parameter. If jump detection is kept, unify to a single value. If disabled, set both to 0.0.** |

---

## 6.4 HMI command + freshness gates

| ID | Layer | Risk | Severity | Evidence (file:line) | Recommendation |
|----|-------|------|----------|----------------------|----------------|
| D-01 | hmi | Hardware gate is env var + JSON evidence dual gate | info | `hardware_gate.py` — `HardwareGateEvaluator` requires both `HMI_ENABLE_HARDWARE_COMMANDS=1` and a valid approved evidence file; `HMI_HARDWARE_GATE_EVIDENCE_FILE` may point at local commissioning evidence while the checked-in JSON remains locked | No action. |
| D-02 | hmi | Lease TTL = 15 s default, renew extends by TTL | info | `session_lock_service.py:24-25` — `ttl_seconds=15`, `_purge_if_expired()` auto-revokes | No action. |
| D-03 | hmi | Force takeover is auditable | info | `session_lock_service.py:52-67` — `force_takeover` flag recorded in lease record, audit trail via supervisor `_trace()` | No action. |
| D-04 | hmi | Replay endpoints are read-only (GET only) | info | `app.py:233,251` — `@app.get('/api/hmi/replay')`, `@app.get('/api/hmi/replay/{command_id}')` | No action. No execute path off `/replay`. |
| D-05 | hmi | Confirmation window = 30 s default, expired commands auto-rejected | info | `supervisor_service.py:73` — `confirmation_window_sec=30.0`; `supervisor_lifecycle.py` `_expire_pending_confirmations()` | No action. |
| D-06 | hmi | FastDDS confirmed as sole middleware | info | No CycloneDDS references in source code. Only in docs/rules as a constraint. | No action. |

---

## Summary

| Severity | Count |
|----------|-------|
| info | 18 |
| low | 1 |
| medium | 1 |
| high | 0 |
| safety-critical | 0 |

**No safety-critical findings.** Two findings (C-04, C-07) relate to `kCartesianJumpThreshold` inconsistency across compilation units. This does not block hardware execution — the non-zero values are more conservative than disabled (0.0) — but the inconsistency should be unified in a follow-up ticket.

All other paths are fail-closed and consistent with documented contracts.
