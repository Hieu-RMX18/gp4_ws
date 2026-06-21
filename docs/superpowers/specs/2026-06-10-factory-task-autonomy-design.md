# Design Specification: Transition to FactoryTask Autonomy Architecture

**Date:** 2026-06-10
**Author:** Antigravity
**Status:** Under Review (Revised after codebase audit)

---

## 1. Problem Statement

The `llm_gateway` package currently has **two conflicting execution paths** for processing natural language commands:

1. **Legacy Direct Path** (`_direct_review_semantic_ir`, line 806): Regex-based parser that intercepts simple commands (e.g., "move to pose A", "go home", "stop") **before the LLM ever sees them**. Outputs static Semantic IR.
2. **FactoryTask Path** (`ReActAgent` → `TaskCompiler`): LLM generates a FactoryTask Behavior Tree. The compiler attempts to flatten it into Semantic IR.

**Both paths converge on the same bottleneck: `IntentRouter`**, which performs static lookups of pose names in SRDF and station semantic map. When a target (e.g., "A", "red_box") is not pre-registered, `IntentRouter` raises a `ValueError` and the command fails.

### Root Cause Chain (verified from code)

```
User: "move to pose A"
  → _direct_review_semantic_ir (line 926) catches it
    → _direct_named_pose_review_semantic_ir finds no match for "A"
    → station_scene_graph.resolve_region("a") finds no match
    → Returns None (falls through)
  → ReActAgent.run() generates FactoryTask with move_named_pose
  → _compile_factory_task_review_result (line 977):
    → TaskCompiler.compile() SUCCEEDS (no runtime-control nodes)
    → Returns compiled Semantic IR {"intent": "move_named_pose", "pose_name": "a"}
  → IntentRouter._route_named_pose() looks up "a" in SRDF → NOT FOUND
  → ValueError → Command rejected
```

For objects requiring perception (e.g., "red_box"):
```
  → _prime_factory_task_world_model tries query_perception
    → If camera offline or object not visible → fails
  → TaskCompiler raises FactoryTaskError("world model has no grounded pose")
  → Returns {"error": "WORLD_MODEL_UNGROUNDED"} → Command rejected
```

---

## 2. Current Architecture (Verified)

### _generate_review_semantic_ir (line 921-975):
```
1. Check semantic cache              → cache hit: return cached
2. _direct_review_semantic_ir()      → regex match: return static IR (SKIP LLM)
3. ReActAgent.run()                  → FactoryTask: compile it
                                     → error: pass through
                                     → non-FactoryTask: REACT_HANDOFF error
4. Fallback LLM client               → parse raw response
```

### _compile_factory_task_review_result (line 977-1045) — THREE branches:
```
Branch A: TaskCompiler succeeds       → return compiled Semantic IR → IntentRouter
Branch B: FactoryTaskError (grounding) → return WORLD_MODEL_UNGROUNDED error
Branch C: FactoryTaskError (runtime)   → return FACTORY_TASK_RUNTIME_INTENT sentinel
```

**Key insight:** Only Branch C bypasses IntentRouter. Branches A and B both fail when targets are not statically registered.

### ReAct prompt (line 331):
> "Unknown object poses... must produce MISSING_SLOT **or** a FactoryTask observe step before motion."

The prompt already allows the LLM to choose `observe` over `MISSING_SLOT`, but the LLM defaults to `MISSING_SLOT` because it is simpler. The fix is to **prioritize** the observe/runtime path.

### Existing perception priming (line 1047-1057):
`_prime_factory_task_world_model` already queries camera/perception before compilation. But it runs only at review time, not at execution time. If perception fails at review, the entire command is rejected.

---

## 3. Target Architecture

### Principle: ALL FactoryTasks become runtime sentinels

The core change is to make `_compile_factory_task_review_result` **always** return `FACTORY_TASK_RUNTIME_INTENT` sentinel, regardless of whether the FactoryTask is simple or complex. This way:
- Simple sequences (`move_named_pose` → `go_home`) get the sentinel with a flat runtime plan
- Complex sequences (`retry` → `pick_object` → `verify_scene`) get the sentinel with a tree runtime plan
- IntentRouter is bypassed entirely for all FactoryTask-originated commands
- TaskRuntime handles pose resolution dynamically at execution time

### Data Flow (New):
```
User input
  → _direct_review_semantic_ir: REMOVE (except "stop" for safety)
  → ReActAgent.run(): generates FactoryTask JSON
  → _compile_factory_task_review_result:
      → parse_factory_task() validates structure
      → _prime_factory_task_world_model() tries perception (best-effort, non-blocking)
      → ALWAYS return FACTORY_TASK_RUNTIME_INTENT sentinel with full runtime_plan
  → HMI displays operator_summary for confirmation
  → TaskRuntime.run() executes tree with live perception + motion
```

---

## 4. Proposed Changes (Exact Files & Lines)

### A. llm_gateway_node.py

#### Change 1: Remove direct regex interception (lines 806-859, 926-940)
- Delete `_direct_review_semantic_ir` static method entirely
- In `_generate_review_semantic_ir` (line 921), remove lines 926-940 (the call to direct review and early return)
- **Keep** the "stop" fast-path as a 2-line safety check before LLM call:
  ```python
  normalized = " ".join(str(intent_text or "").strip().lower().split()).strip(" .!?")
  if normalized in _DIRECT_STOP_REVIEW_TEXTS:
      return {"intent": "stop"}
  ```

#### Change 2: Force runtime sentinel in compiler (lines 977-1045)
- Modify `_compile_factory_task_review_result` to:
  1. Parse and validate FactoryTask structure (keep)
  2. Run `_prime_factory_task_world_model` best-effort (keep, but don't fail on error)
  3. Build `runtime_plan` from raw payload (keep `_factory_task_runtime_plan`)
  4. **Always** return `FACTORY_TASK_RUNTIME_INTENT` sentinel carrying the full tree
  5. Remove Branch A (compiled Semantic IR) and Branch B (`WORLD_MODEL_UNGROUNDED` error)

#### Change 3: Remove dead helper functions
- Delete `_direct_joint_review_semantic_ir` (standalone function)
- Delete `_direct_cartesian_review_semantic_ir` (standalone function)
- Delete `_direct_named_pose_review_semantic_ir` (standalone function)
- Delete associated constants: `_STATION_NAV_PREFIXES`, `_CARTESIAN_MOVE_PREFIXES`, `_LINEAR_UNIT_ALIASES` (if only used by direct parsers)

### B. react_planner.py

#### Change 4: Prioritize observe over MISSING_SLOT (line 331)
Replace:
```
Unknown object poses, stale perception, missing calibration, missing frame, or unknown region must produce MISSING_SLOT or a FactoryTask observe step before motion.
```
With:
```
Unknown object poses, stale perception, missing calibration, missing frame, or unknown region: ALWAYS generate a FactoryTask with an observe_station step before the motion step. The TaskRuntime will query perception at execution time. Only return MISSING_SLOT if the operator's command is fundamentally incomplete (e.g., "move" with no target at all).
```

### C. Tests

#### Change 5: Update test_direct_review_regex.py
- Remove or rewrite tests that assert direct regex parsing behavior
- Add test verifying that simple text like "move to pose A" reaches ReActAgent (not intercepted)

#### Change 6: Update test_react_gateway_pipeline.py
- Add test: simple "move to A then go home" must produce `FACTORY_TASK_RUNTIME_INTENT` (not compiled Semantic IR)
- Update existing test assertions that expect compiled Semantic IR from FactoryTasks

---

## 5. What We Are NOT Changing

- **IntentRouter**: Kept intact for backward compatibility with `/llm_raw_command` path
- **TaskCompiler**: Kept intact; just not used as the final output path anymore
- **TaskRuntime**: No changes needed; it already handles the sentinel
- **semantic_ir_contract.py**: Already supports `FACTORY_TASK_RUNTIME_INTENT`
- **Safety validators**: All motion still goes through SemanticValidator, execution_gate, command_validator

---

## 6. Risk Assessment

| Risk | Mitigation |
|------|-----------|
| "stop" command delayed by LLM latency | Keep 2-line stop fast-path before LLM |
| LLM generates invalid FactoryTask | `parse_factory_task()` validates structure before sentinel |
| Camera offline → no perception | `_prime_factory_task_world_model` is best-effort; TaskRuntime retries at execution |
| Regression in drawing commands | Drawing still works via ReAct → FactoryTask draw_shape skill |
| Test breakage | Update ~3 test files alongside changes |

---

## 7. Verification Plan

1. `colcon build --symlink-install` — must succeed
2. `colcon test` — update failing tests, all must pass
3. Manual test: "move to pose A then come to pose B"
   - Expected: `FACTORY_TASK_RUNTIME_INTENT` with `runtime_plan` containing 2 `move_named_pose` skills
   - NOT expected: `MISSING_SLOT` or `ValueError` from IntentRouter
4. Manual test: "pick the red box"
   - Expected: `FACTORY_TASK_RUNTIME_INTENT` with `observe_station` + `pick_object` skills
   - NOT expected: `WORLD_MODEL_UNGROUNDED` error
