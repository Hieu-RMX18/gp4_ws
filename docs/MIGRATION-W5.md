# MIGRATION-W5.md — HMI Aggressive Consolidation

**Wave:** W5
**Branch:** ws-deep-rebuild-3526
**Date:** 2026-05-04
**Status:** COMPLETE

## Verification Results

### New tests (all passing)
- `src/llm_gateway/tests/test_hydrate_workplane_service.py` — 8 passed
- `hmi/backend/tests/test_intent_resolution_via_rpc.py` — 8 passed

### Existing tests (no regressions)
- `hmi/backend/tests/` — 113 passed, 3 pre-existing failures (unrelated to W5: `BLENDED_SEQUENCE` validation + `DEFAULT_JOINT_NAMES` import)
- `src/llm_gateway/tests/` — pre-existing tests unaffected

### Syntax validation
- All modified Python files compile cleanly

## Discovery Summary

### HMI modules with duplicated logic

| HMI module | Lines | Duplicated logic | Canonical source |
|---|---|---|---|
| `intent_resolution.py` | 440-479 | `_hydrate_draw_workplane` | `llm_gateway/command_pipeline.py:65-128` |
| `intent_constants.py` | 1-141 | `SUPPORTED_PRIMITIVES`, `PLANNER_DEFAULTS`, `_ALLOWED_FIELDS_BY_PRIMITIVE`, `_OLD_ACTIONS` | `llm_gateway/normalizer.py`, `llm_gateway/intent_router.py` |
| `intent_normalization.py` | 76-340 | `_normalize_command`, `_normalize_pose`, `_normalize_joint_target` | `llm_gateway/normalizer.py` |
| `supervisor_validation.py` | 57-173 | `_validate_command` pre-flight checks | `safety/command_validator.py` |
| `supervisor_execution.py` | 20-83 | `_confirm_command_internal` confirm gate | Should be in supervisor pkg |

### Existing ROS surfaces consumed by HMI

- `/validate_command` (service) — already used
- `/execute_motion` (action) — already used
- `/get_current_pose` (service) — already used
- `/llm_text_input` (topic) — publish only

### New ROS services needed

None exist yet. All must be created.

## W5.T1 — HMI → ROS call surface mapping

| HMI module (current) | Operation | New ROS surface | Implementation |
|---|---|---|---|
| `intent_resolution.py:440-479` `_hydrate_draw_workplane` | Hydrate draw workplane | `/llm_gateway/hydrate_workplane` | Service server in `llm_gateway_node.py`, calls `command_pipeline.hydrate_draw_workplane` |
| `intent_constants.py` `SUPPORTED_PRIMITIVES`, `PLANNER_DEFAULTS`, `_ALLOWED_FIELDS_BY_PRIMITIVE` | Static config | `/llm_gateway/get_primitive_constants` | Service server in `llm_gateway_node.py`, returns constants from shared YAML |
| `intent_normalization.py` `_normalize_command` | Normalize primitive command | `/validate_command` (existing) | Already used by llm_gateway; HMI sends raw → receives normalized |
| `supervisor_validation.py:57` `_validate_command` | Pre-flight validation | `/validate_command` (existing) | Already called via ROS adapter |
| `supervisor_execution.py:20` `_confirm_command_internal` | Confirm gate | `/supervisor/confirm_execution` | New Python service node in supervisor pkg |

## Files changed

### New files
- `src/interfaces/srv/HydrateWorkplane.srv`
- `src/interfaces/srv/GetPrimitiveConstants.srv`
- `src/interfaces/srv/ConfirmExecution.srv`
- `src/supervisor/scripts/confirm_execution_service.py`
- `src/llm_gateway/test/test_hydrate_workplane_service.py`
- `src/supervisor/test/test_confirm_execution_service.py`
- `hmi/backend/tests/test_intent_resolution_via_rpc.py`

### Modified files
- `src/interfaces/CMakeLists.txt` — add new srv files
- `src/llm_gateway/llm_gateway/llm_gateway_node.py` — add service servers
- `src/supervisor/CMakeLists.txt` — install Python service script
- `hmi/backend/services/intent_resolution.py` — replace `_hydrate_draw_workplane` with RPC
- `hmi/backend/services/intent_constants.py` — replace with RPC-based fetching
- `hmi/backend/ros/adapter.py` — add new service clients + readiness
- `hmi/backend/ros/telemetry_snapshot.py` — add readiness flags
- `docs/hmi/HMI_ROS_INTERFACES.md` — add new services
- `hmi/ARCHITECTURE.vi.md` — update flow description
