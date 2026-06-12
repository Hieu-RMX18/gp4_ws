# LLM Gateway R4 — Thin Node + Remove Legacy Dispatch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Collapse the two parallel execution paths into one — route every command through `task_runtime` — then delete the orphaned legacy dispatch logic so `llm_gateway_node.py` becomes a thin ROS host (< ~700 lines) per spec §6.

**Architecture:** After R2, `task_runtime` actuates motion and is the intended sole motion emitter. The legacy path (`process_intent`/`process_raw_command` → `_process_sequence` → `_dispatch_normalized_command` → `_on_goal_sent`/`_on_execution_done`) still exists in parallel. R4 (1) routes tier-1/single commands through the runtime confirm path, (2) deletes the now-orphan legacy methods (each guarded by a grep that proves zero callers), (3) extracts the runtime skill-executor wiring into `runtime_skill_executor.py`.

**Tech Stack:** rclpy, pytest, colcon. Phase R4 of `docs/superpowers/specs/2026-06-12-llm-gateway-remediation-design.md` §6. **Highest-risk phase — do it last, with R3's task_events live so behavior is observable.**

---

## Verified preconditions + risks

- Legacy motion emitter: `process_raw_command`/`process_intent` (topic subs `/llm_intent`, `/llm_text_input`) → `_process_llm_payload` → `_normalize_and_validate` → `_process_single_command`/`_process_sequence` → `_dispatch_sequence_step`/`_dispatch_normalized_command` → `send_goal_async` (`:1902`) → `_on_goal_sent` → `_on_execution_done`. Helper state: `_SequenceExecutionState`.
- **SHARED — do NOT delete:** `_normalize_and_validate` is also called by R2's `_runtime_command_is_safe`. `_SceneSnapshotCache` is used by perception. `_goal_mapper`, `_validate_client`, `_execute_client`, `_wait_for_future_without_spinning` are all reused by the runtime path. Verify each symbol's callers before deleting anything.
- **CLI risk:** `gp4_cmd` (CLI) defaults to topic transport (`/llm_text_input`) and has `--transport action`. Removing the topic path breaks the CLI default. Decision required in Task 1.
- `direct_commands.parse` produces tier-1 Semantic IR for ~5 safe commands; these currently flow through review→confirm like FactoryTasks (the confirm path branches on `is_factory_task_runtime_sentinel`).

## Open decisions (resolve before Task 2)

1. **CLI topic transport:** (a) keep `/llm_text_input` subscription but reroute its handler through the runtime confirm path, or (b) make `gp4_cmd` default to the action/service transport and delete the topic path. Recommend (a) for a thin bridge that preserves the CLI, with the legacy *dispatch internals* still deleted.
2. **Tier-1 single commands:** wrap each direct_commands result as a single-skill FactoryTask and run it through `TaskRuntime`, so there is exactly one motion emitter. Confirm the safety/IR for tier-1 (home/stop/get_pose/wait) round-trips through `_validate_runtime_command`.

## File structure

| File | Change |
|------|--------|
| `src/llm_gateway/llm_gateway/runtime_skill_executor.py` | Create — the `_execute_skill` closure + `_runtime_command_is_safe`/`_dispatch_runtime_goal` logic, taking the node's clients as injected deps |
| `src/llm_gateway/llm_gateway/llm_gateway_node.py` | Delete legacy dispatch methods; thin to wiring; delegate execution to `runtime_skill_executor` |
| `src/llm_gateway/tests/` | Update/remove tests asserting legacy dispatch internals |

---

## Task 1: Route tier-1 single commands through task_runtime

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
- Test: `src/llm_gateway/tests/test_tier1_via_runtime.py`

- [ ] **Step 1: Resolve the CLI decision**

Run: `grep -rnE "transport|/llm_text_input|/llm_intent|publish" src/llm_gateway/llm_gateway/gp4_cmd*.py` (or wherever the CLI lives — `grep -rl "def main" src/llm_gateway`). Confirm how the CLI sends commands. Record the decision (recommend: keep topic subscription as a thin bridge into the runtime path).

- [ ] **Step 2: Write the failing test**

Create `src/llm_gateway/tests/test_tier1_via_runtime.py`:
```python
from llm_gateway.llm_gateway_node import LLMGatewayNode


def test_tier1_home_runs_through_runtime(monkeypatch):
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()
    ran = {"skills": []}
    monkeypatch.setattr(node, "_run_single_command_via_runtime",
                        lambda ir: ran["skills"].append(ir.get("intent")) or True)

    ir = {"intent": "home", "_parse_source": "direct"}
    assert node._execute_tier1_command(ir) is True
    assert ran["skills"] == ["home"]
```

- [ ] **Step 3: Run test to verify it fails**

Run: `python -m pytest tests/test_tier1_via_runtime.py -q`
Expected: FAIL — methods missing.

- [ ] **Step 4: Implement tier-1 → single-skill FactoryTask → runtime**

Add `_execute_tier1_command` / `_run_single_command_via_runtime` that wrap a direct_commands Semantic IR into a one-node `FactoryTask` and run it through the same `TaskRuntime` + `_execute_skill` used by `_on_confirm_factory_task_runtime`. Reuse `_validate_runtime_command` so the single command goes safety→dispatch identically. No new motion emitter.

- [ ] **Step 5: Run + full suite + build + commit**

```bash
python -m pytest tests/test_tier1_via_runtime.py tests/ -q
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_tier1_via_runtime.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "feat(llm_gateway): R4 route tier-1 commands through task_runtime (single emitter)"
```

---

## Task 2: Extract `runtime_skill_executor.py`

**Files:**
- Create: `src/llm_gateway/llm_gateway/runtime_skill_executor.py`
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
- Test: `src/llm_gateway/tests/test_runtime_skill_executor.py`

- [ ] **Step 1: Impact analysis**

Run: `gitnexus_impact({target: "_validate_runtime_command", direction: "upstream"})` and the same for `_execute_skill`, `_dispatch_runtime_goal`. Confirm callers are only the confirm path + tier-1 path.

- [ ] **Step 2: Write the failing test**

Create `src/llm_gateway/tests/test_runtime_skill_executor.py`:
```python
from llm_gateway.runtime_skill_executor import RuntimeSkillExecutor
from llm_gateway.task_runtime import RuntimeStepResult


class _FakeNodeDeps:
    def semantic_ir_for_skill(self, name, args): return {"intent": name}
    def validate_and_dispatch(self, semantic_ir):
        return RuntimeStepResult(success=True)


def test_executor_grounds_validates_dispatches():
    ex = RuntimeSkillExecutor(_FakeNodeDeps())
    result = ex("pick_object", {"object_id": "w"})
    assert result.success is True
```

- [ ] **Step 3: Run test to verify it fails**

Run: `python -m pytest tests/test_runtime_skill_executor.py -q`
Expected: FAIL — module missing.

- [ ] **Step 4: Implement the extraction**

Create `runtime_skill_executor.py` with a `RuntimeSkillExecutor` that takes a small deps object (methods: `semantic_ir_for_skill`, `validate_and_dispatch`) and is callable `(name, args) -> RuntimeStepResult`. Move the `_execute_skill` body there. The node passes an adapter exposing `_semantic_ir_for_runtime_skill` and `_validate_runtime_semantic_ir` as those two methods. Keep `_runtime_command_is_safe`/`_dispatch_runtime_goal` on the node (they hold ROS clients) or move them too behind the deps interface — pick the split that keeps the node free of execution branching.

- [ ] **Step 5: Run + full suite + build + commit**

```bash
python -m pytest tests/ -q
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/runtime_skill_executor.py src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_runtime_skill_executor.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "refactor(llm_gateway): R4 extract runtime_skill_executor from node"
```

---

## Task 3: Delete orphaned legacy dispatch (grep-guarded, one method per commit)

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
- Modify/remove: tests asserting legacy internals

For EACH legacy symbol below, in this order, do: (a) grep prove zero non-test callers, (b) delete it, (c) delete/Update its tests, (d) run full suite + build, (e) commit. Never batch — one symbol per commit so a regression is bisectable.

Order (leaf-first to keep intermediate states green):
1. `_on_execution_done`
2. `_on_goal_sent`
3. the legacy `send_goal_async` dispatch method (the one at `:1902`, NOT the runtime `_dispatch_runtime_goal`)
4. `_dispatch_normalized_command`
5. `_dispatch_sequence_step`
6. `_process_sequence`
7. `_process_single_command`
8. `_process_llm_payload`
9. `_SequenceExecutionState`
10. legacy `process_intent` / `process_raw_command` bodies (replace with the thin tier-1/runtime bridge from Task 1, or remove subs if CLI moved to action transport per Task 1 decision)

- [ ] **Step per symbol: prove orphan**

```bash
grep -rnE "\b<SYMBOL>\b" src/llm_gateway/llm_gateway hmi --include=*.py | grep -v "def <SYMBOL>"
```
Expected: empty (or test-only). If a non-test caller remains, that symbol is NOT orphan yet — stop and re-route it first.

- [ ] **Step per symbol: delete + test + build + commit**

```bash
# after deleting <SYMBOL> and its tests:
python -m pytest tests/ -q
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add -A src/llm_gateway
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "refactor(llm_gateway): R4 remove legacy <SYMBOL>"
```

> **DO NOT delete** `_normalize_and_validate`, `_SceneSnapshotCache`, `_goal_mapper`, `_validate_client`, `_execute_client`, `_wait_for_future_without_spinning`, `_query_perception_detections` — all reused by the runtime path.

---

## Task 4: Verify thin node + docs

**Files:**
- Modify (docs): `.claude/rules/llm-gateway.md` (if present), root `CLAUDE.md`, `README.md`

- [ ] **Step 1: Confirm the shrink**

Run: `wc -l src/llm_gateway/llm_gateway/llm_gateway_node.py`
Expected: < ~700 lines (target <600; if 700–900, list what logic remains and whether it is pure wiring). The node should contain: parameter declaration, subscriptions, service/action server handlers, action/service clients, publishers, and runtime wiring — no parse/normalize/sequence/dispatch branching.

- [ ] **Step 2: Single-emitter assertion**

Run: `grep -nE "send_goal_async" src/llm_gateway/llm_gateway/*.py`
Expected: exactly one occurrence, inside `runtime_dispatch.py` (or the node method it wraps). If `send_goal_async` appears anywhere else, a legacy emitter survived.

- [ ] **Step 3: Full suite + build**

```bash
python -m pytest tests/ -q
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash
gitnexus_detect_changes()   # confirm only expected symbols/flows changed
```

- [ ] **Step 4: Update docs**

Update root `CLAUDE.md` llm_gateway description + `.claude/rules/llm-gateway.md` (pipeline now: direct_commands/task_planner → TaskCompiler grounding → task_runtime sole emitter → /execute_motion; `/llm_gateway/task_events` topic). Per `when-to-update-claude-docs.md` (pipeline change trigger).

- [ ] **Step 5: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add -A
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "docs(llm_gateway): R4 update pipeline docs; node is now a thin ROS host"
npx gitnexus analyze
```

---

## Done criteria for R4

- [ ] Exactly one motion emitter: `send_goal_async` appears only in the runtime dispatch path.
- [ ] Legacy `_process_*`/`_dispatch_*`/`_on_goal_sent`/`_on_execution_done`/`_SequenceExecutionState` deleted; shared helpers retained.
- [ ] Tier-1 + topic/CLI commands route through `task_runtime`.
- [ ] `llm_gateway_node.py` < ~700 lines, pure wiring.
- [ ] Full suite green; build green; GitNexus reindexed; pipeline docs updated.

R4 completes the remediation: factory_task split (R1), closed-loop actuate (R2), unified System Log (R3), single-path thin node (R4).
