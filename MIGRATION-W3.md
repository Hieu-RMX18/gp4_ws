# MIGRATION-W3 — ReAct Agent on /llm_intent

**Wave:** W3
**Branch:** ws-deep-rebuild-3526
**Date:** 2026-05-04

## What changed

### T0 — ReAct agent skeleton
- New: `src/llm_gateway/llm_gateway/react/` module
  - `agent.py` — `ReActAgent` class: multi-turn reasoning loop with tool-use → semantic IR
  - `tool_registry.py` — `Tool` base class, `ToolRegistry`, `ToolResult`
  - `iteration_budget.py` — `IterationBudget` / `IterationCounters` with tiered limits (total/motion/readonly/repair) + wall-clock timeout
  - `state_injector.py` — `StateInjector`: snapshot builder for robot state (joints, mode, alarms, capabilities, velocity scale)
  - `__init__.py` — exports all public symbols

### T1 — ReAct tool implementations
- New: `src/llm_gateway/llm_gateway/react/tools/`
  - `get_current_pose.py` — calls `/get_current_pose` ROS service, returns PoseStamped dict
  - `plan_motion.py` — validates motion target via `/validate_command` service, returns `plan_id`
  - `submit_motion.py` — submits planned motion via `/execute_motion` action, returns `goal_id`
  - `wait_for_state.py` — single-shot snapshot check for `IDLE`/`MOVING`/`PLANNING`/`FAULT` (no `time.sleep` per DON'T list)
  - `set_speed.py` — updates `velocity_scale` on `StateInjector` for downstream motion tools
  - `query_perception.py` — **stub** (W4 will implement): returns `perception_not_yet_implemented`
  - `gripper_open.py` / `gripper_close.py` — **stubs** until gripper capability wired: returns `capability_unavailable`
  - `compute_arc_points.py` — pure local geometry: computes start/auxiliary/target poses for `CIRC` from center/radius/angles/plane normal (no ROS calls)

### T2 — Wire ReAct into `llm_gateway_node`
- `llm_gateway_node.py`:
  - `_load_react_enabled()` reads `llm.react.enabled` from `safety_rules.yaml` SSOT
  - If enabled: `process_intent()` routes to `ReActAgent.run()` first; on success feeds semantic IR into existing `_process_llm_payload()` pipeline
  - On ReAct handoff (budget exceeded / repair exhausted): **fallback** to legacy single-shot `IntentRouter` path with warning log
  - Legacy path marked `# DEPRECATED: removal_date=2026-06-01, reason=replaced_by_react_in_W3`

### T3 — LLM config temperature from SSOT
- `llm_config.py`:
  - `_default_safety_rules_path()` resolves `safety_rules.yaml` from package share or source tree
  - `_load_safety_temperature()` reads `llm.react.temperature` (default 0.0)
  - `load_llm_backend_config()` uses `_load_safety_temperature()` as fallback for `temperature`

### T4 — `llm_client.py` multi-turn support
- Added `generate_response_from_messages(messages: list[dict[str, str]])` method
- Added `_build_request_from_messages()` to construct OpenAI-compatible payload from pre-built message list
- Original `generate_response(user_input)` path unchanged

### T5 — Add `MACRO` primitive
- `llm_schema.yaml`:
  - Added `MACRO` to `primitive_type` enum
  - Added `if/then` conditional schema: requires `steps` array (min 1, max 10), each item is self-referential `$ref`
- `semantic_validator.py`:
  - Added `MACRO` to `_ALLOWED_PRIMITIVES`
  - Added `MACRO` validation: non-empty `steps`, max 10 steps
- `normalizer.py`:
  - Added `MACRO` to non-motion primitives bypass (no planner defaults)
- `test_contract_consistency.py`:
  - Added `MACRO` to `_FROZEN_PUBLIC_PRIMITIVES` and `_NON_PLANNING_PRIMITIVES`
  - Added `MACRO` to `draw_shape`/`draw_text` intent mappings

### T6 — Safety rules SSOT keys
- `safety_rules.yaml`:
  - Added `llm.react` section:
    - `enabled: true`
    - `max_total_iterations: 5`
    - `max_motion_iterations: 3`
    - `max_readonly_iterations: 10`
    - `max_repair_iterations: 1`
    - `wall_clock_timeout_s: 30`
    - `temperature: 0.2`

### T7 — Tests
- New: `src/llm_gateway/tests/test_react_iteration_budget.py` — 10 tests covering budget tiering, readonly/motion/total exhaustion, combo tool counting
- New: `src/llm_gateway/tests/test_react_state_injector.py` — 5 tests for default snapshot, joint state updates, status updates, velocity scale, capabilities
- New: `src/llm_gateway/tests/test_react_agent_basic.py` — 6 tests: final IR return, one tool-call-then-final, unknown tool recovery, budget exceeded handoff, schema invalid then repair, repair exhausted handoff
- New: `src/llm_gateway/tests/test_react_tools/test_compute_arc_points.py` — 4 tests: 90° XY arc, zero-sweep rejection, negative radius rejection, YZ plane arc

### T8 — Build/verification
- `setup.py`: added `llm_gateway.react` and `llm_gateway.react.tools` to `packages=[]`
- Full workspace build: 19 packages, 0 failures
- `llm_gateway`: 334 tests, 0 errors, 0 failures

## How to roll back

```bash
git revert df6a6e4
```

Or disable ReAct without reverting:
```bash
# Edit src/safety/config/safety_rules.yaml
llm:
  react:
    enabled: false
```

## Risk notes

- **Jog pendant test failures** are pre-existing and unrelated to W3 (see W1/W2 status).
- **Deprecated path** (`/llm_intent` single-shot via `IntentRouter`) will be removed on `2026-06-01`.
- **Perception / gripper tools** are stubs returning errors; they do not block W3 but must be implemented in W4.
- **compute_arc_points** uses a right-handed basis construction; verify orientation matches physical arm before using for real CIRC arcs.
- **temperature=0.2** from `safety_rules.yaml` is intentionally low to reduce hallucination risk in tool-call loops.
