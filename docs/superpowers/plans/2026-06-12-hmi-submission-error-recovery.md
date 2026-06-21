# HMI Submission Error Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the raw `"ROS future timed out"` error string surfaced in the HMI chat with an operator-friendly message, and teach the HMI Phase 1 sequence adapter to expand the LLM-emitted FactoryTask `repeat` runtime node into N flat step entries so multi-step repeat commands no longer get rejected with the "FactoryTask runtime node 'repeat' is not supported by the HMI Phase 1 adapter" message.

**Architecture:**
- **Error mapping** lives in `hmi/backend/ros/command_dispatch.py` — add a `TimeoutError → user-friendly` mapper at the `_wait_for_future` boundary so the literal string `"ROS future timed out."` never leaks into the `Step 6/6 RESULT` line again. `_submit_text_for_gateway_review` in `hmi/backend/services/supervisor_submission.py` keeps using `str(exc)` but now receives the friendly string.
- **Repeat expansion** lives in `hmi/backend/services/supervisor_sequence.py::_runtime_plan_to_semantic_steps` — add a `repeat` branch that flattens its `body`/`children` N times into a flat list of semantic-IR dicts. Boundary validation: `count` must be a positive int ≤ 100, otherwise reject with a clear `IntentResolutionError` so the operator sees an actionable message, not a stack-trace-leaking `IntentResolutionError`.

**Tech Stack:** Python 3.10, pytest 9.0.3, FastAPI/HMI v2 supervisor services, ROS 2 rclpy client (read-only — no protocol changes).

---

## Test Baseline (verified 2026-06-12)

```
cd /home/hieu2/gp4_ws/hmi/backend
PYTHONPATH=/home/hieu2/gp4_ws:/home/hieu2/gp4_ws/src/llm_gateway \
  .venv/bin/python -m pytest tests/test_supervisor_sequence.py -q
# → 35 passed in 0.35s
```

**Test command for all tasks** (from `hmi/backend/`):
```bash
PYTHONPATH=/home/hieu2/gp4_ws:/home/hieu2/gp4_ws/src/llm_gateway \
  .venv/bin/python -m pytest tests/test_supervisor_sequence.py tests/test_ros_adapter.py tests/test_supervisor_service.py -q
```

---

## File Structure

| File | Responsibility |
|------|----------------|
| `hmi/backend/ros/command_dispatch.py` | `_wait_for_future` — adds error-mapping helper, raises `TimeoutError` carrying a human-friendly message |
| `hmi/backend/services/supervisor_submission.py` | `_submit_text_for_gateway_review` — pass through friendly string when timeout |
| `hmi/backend/services/supervisor_sequence.py` | `_runtime_plan_to_semantic_steps` — add `repeat` branch; add `REPEAT_MAX_COUNT` constant; add `_expand_repeat_node` static helper |
| `hmi/backend/tests/test_ros_adapter.py` | New tests: timeout mapped to friendly message; same timeout in non-review call sites |
| `hmi/backend/tests/test_supervisor_sequence.py` | New tests: `repeat` expansion (basic, nested, count=1, invalid count) |

No new files are created. Two new test classes are added to existing test files. No production file grows past 1000 lines.

---

## Task 1: Map `_wait_for_future` TimeoutError to a friendly message

**Files:**
- Modify: `hmi/backend/ros/command_dispatch.py:840-846` (`_wait_for_future`)
- Test: `hmi/backend/tests/test_ros_adapter.py` (add `TestWaitForFutureFriendlyMessage` class near existing `_FakeReviewClient` tests)

### Step 1: Write the failing test

Append the following test class to `hmi/backend/tests/test_ros_adapter.py` (find the existing `_FakeReviewClient` block and add after it):

```python
class TestWaitForFutureFriendlyMessage(unittest.TestCase):
    """Verify the HMI-facing error string is human-friendly, not the internal literal."""

    def test_timeout_error_message_includes_service_context(self) -> None:
        from hmi.backend.ros.command_dispatch import CommandDispatchMixin
        import threading

        # A future that never completes.
        future: dict[str, bool] = {"done": False}

        def _is_done() -> bool:
            return future["done"]

        class _StubFuture:
            def done(self) -> bool:
                return _is_done()

            def result(self):  # pragma: no cover - never reached
                raise AssertionError("result() must not be called on timeout")

        adapter = CommandDispatchMixin()
        with self.assertRaises(TimeoutError) as ctx:
            adapter._wait_for_future(
                _StubFuture(),  # type: ignore[arg-type]
                timeout_sec=0.05,
                context="review_intent call",
            )
        msg = str(ctx.exception)
        self.assertNotIn("ROS future timed out.", msg)
        self.assertIn("review_intent", msg)
        self.assertIn("timed out", msg.lower())

    def test_successful_future_returns_value(self) -> None:
        from hmi.backend.ros.command_dispatch import CommandDispatchMixin

        class _DoneFuture:
            def done(self) -> bool:
                return True

            def result(self):
                return {"ok": True}

        adapter = CommandDispatchMixin()
        result = adapter._wait_for_future(
            _DoneFuture(),  # type: ignore[arg-type]
            timeout_sec=0.5,
            context="synthetic",
        )
        self.assertEqual(result, {"ok": True})
```

### Step 2: Run test to verify it fails

```bash
cd /home/hieu2/gp4_ws/hmi/backend
PYTHONPATH=/home/hieu2/gp4_ws:/home/hieu2/gp4_ws/src/llm_gateway \
  .venv/bin/python -m pytest tests/test_ros_adapter.py::TestWaitForFutureFriendlyMessage -v
```

Expected: `TypeError: _wait_for_future() got an unexpected keyword argument 'context'` (or `KeyError`/`AttributeError` — the exact error doesn't matter as long as the new behaviour is not implemented yet).

### Step 3: Add the `context` parameter and friendly message to `_wait_for_future`

In `hmi/backend/ros/command_dispatch.py` replace the existing `_wait_for_future` (lines 840–846):

```python
    def _wait_for_future(
        self,
        future: Any,
        timeout_sec: float,
        *,
        context: str | None = None,
    ) -> Any:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while not future.done():
            if time.monotonic() >= deadline:
                scope = context or "ROS call"
                raise TimeoutError(
                    f"{scope} did not respond within {timeout_sec:.1f}s. "
                    "The ROS service/action may be busy, not running, or the "
                    "robot stack may need to be relaunched. Check "
                    "`ros2 node list` and `ros2 service list` before retrying."
                )
            time.sleep(0.05)
        return future.result()
```

`context` is keyword-only and optional so existing call sites keep working unchanged.

### Step 4: Run the new test, expect PASS

```bash
cd /home/hieu2/gp4_ws/hmi/backend
PYTHONPATH=/home/hieu2/gp4_ws:/home/hieu2/gp4_ws/src/llm_gateway \
  .venv/bin/python -m pytest tests/test_ros_adapter.py::TestWaitForFutureFriendlyMessage -v
```

Expected: 2 passed.

### Step 5: Run the full adapter test file, expect all pass

```bash
cd /home/hieu2/gp4_ws/hmi/backend
PYTHONPATH=/home/hieu2/gp4_ws:/home/hieu2/gp4_ws/src/llm_gateway \
  .venv/bin/python -m pytest tests/test_ros_adapter.py -q
```

Expected: all tests pass (existing tests use a 3-arg `_wait_for_future` and still work because `context` is keyword-only with a default).

### Step 6: Commit

```bash
cd /home/hieu2/gp4_ws
git add hmi/backend/ros/command_dispatch.py hmi/backend/tests/test_ros_adapter.py
git commit -m "fix(hmi): replace raw 'ROS future timed out' error with actionable message"
```

---

## Task 2: Pass friendly context from `submit_text_for_review` to `_wait_for_future`

**Files:**
- Modify: `hmi/backend/ros/adapter.py:311-314` (`submit_text_for_review` — pass `context`)
- Modify: `hmi/backend/ros/command_dispatch.py:225,484,595,649` — pass `context` to all remaining `_wait_for_future` call sites that should also surface a clear message

### Step 1: Update the `submit_text_for_review` call

In `hmi/backend/ros/adapter.py` at the call inside `submit_text_for_review` (lines 311–314), change:

```python
        response = self._wait_for_future(
            self._review_intent_client.call_async(request),
            DEFAULT_REVIEW_INTENT_TIMEOUT_SEC,
        )
```

to:

```python
        response = self._wait_for_future(
            self._review_intent_client.call_async(request),
            DEFAULT_REVIEW_INTENT_TIMEOUT_SEC,
            context=f"/llm_gateway/review_intent call (command_id={command_id})",
        )
```

### Step 2: Update the other four `_wait_for_future` call sites in `command_dispatch.py`

Each of these is a public adapter call; the friendly context lets the operator tell which ROS interface is slow. Add a `context=` argument that names the interface.

Replace each of the four `_wait_for_future` calls in `command_dispatch.py` at the noted line numbers with the change shown:

**Call site 1** (line 225, inside `submit_text_for_review`/similar — actually inside the `WorkspaceRosAdapter` dispatch path for `ValidateCommand`):

```python
        wrapped = self._wait_for_future(
            ...
            context=f"/validate_command call (command_id={command_id})",
        )
```

**Call site 2** (line 484):

```python
        response = self._wait_for_future(
            ...
            context=f"/get_current_pose call (command_id={command_id})",
        )
```

**Call site 3** (line 595, `ExecuteMotion` action goal):

```python
        goal_handle = self._wait_for_future(
            ...
            context=f"/execute_motion action goal (command_id={command_id})",
        )
```

**Call site 4** (line 649, `ExecuteMotion` action result):

```python
        wrapped_result = self._wait_for_future(
            ...
            context=f"/execute_motion action result (command_id={command_id})",
        )
```

The exact `command_id` variable name is whatever the enclosing function uses. Read the 5 lines of context around each call site to pick the right local variable (`command_id`, `goal.command_id`, etc.) — do not invent identifiers.

### Step 3: Run the full backend test suite

```bash
cd /home/hieu2/gp4_ws/hmi/backend
PYTHONPATH=/home/hieu2/gp4_ws:/home/hieu2/gp4_ws/src/llm_gateway \
  .venv/bin/python -m pytest tests/ -q
```

Expected: all tests pass. The existing `test_submit_text_for_review_uses_short_ready_timeout_and_review_sla_timeout` at `tests/test_ros_adapter.py:365-397` overrides `_wait_for_future` and ignores `context=`, so it must still pass.

### Step 4: Commit

```bash
cd /home/hieu2/gp4_ws
git add hmi/backend/ros/adapter.py hmi/backend/ros/command_dispatch.py
git commit -m "fix(hmi): name the ROS interface in timeout error messages"
```

---

## Task 3: Add `repeat` node branch to the runtime plan adapter

**Files:**
- Modify: `hmi/backend/services/supervisor_sequence.py:425-475` (`_runtime_plan_to_semantic_steps`)
- Test: `hmi/backend/tests/test_supervisor_sequence.py` (add `RuntimePlanRepeatNodeTests` class)

### Step 1: Write the failing tests

Append this test class to `hmi/backend/tests/test_supervisor_sequence.py`:

```python
class RuntimePlanRepeatNodeTests(unittest.TestCase):
    """Adapter must expand a FactoryTask runtime `repeat` node into N flat steps."""

    def _adapter(self) -> SupervisorSequenceMixin:
        # _runtime_plan_to_semantic_steps is a regular method (uses self only
        # via current_pose_loader, which we pass as a no-op).
        return SupervisorSequenceMixin()

    def test_repeat_node_expands_children_n_times(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "repeat",
            "count": 2,
            "body": {
                "type": "sequence",
                "children": [
                    {"type": "skill", "name": "go_home"},
                    {"type": "skill", "name": "stop"},
                ],
            },
        }
        steps = adapter._runtime_plan_to_semantic_steps(
            runtime_plan,
            current_pose_loader=lambda: None,
        )
        intents = [step["intent"] for step in steps]
        self.assertEqual(
            intents,
            ["go_home", "stop", "go_home", "stop"],
        )

    def test_repeat_node_count_one_returns_single_expansion(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "repeat",
            "count": 1,
            "body": {
                "type": "sequence",
                "children": [{"type": "skill", "name": "go_home"}],
            },
        }
        steps = adapter._runtime_plan_to_semantic_steps(
            runtime_plan,
            current_pose_loader=lambda: None,
        )
        self.assertEqual([step["intent"] for step in steps], ["go_home"])

    def test_repeat_node_with_zero_count_rejected(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "repeat",
            "count": 0,
            "body": {"type": "sequence", "children": []},
        }
        from hmi.backend.services.supervisor_validation import (
            IntentResolutionError,
        )

        with self.assertRaises(IntentResolutionError) as ctx:
            adapter._runtime_plan_to_semantic_steps(
                runtime_plan,
                current_pose_loader=lambda: None,
            )
        self.assertIn("count", str(ctx.exception).lower())

    def test_repeat_node_with_oversized_count_rejected(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "repeat",
            "count": 10_000,
            "body": {
                "type": "sequence",
                "children": [{"type": "skill", "name": "go_home"}],
            },
        }
        from hmi.backend.services.supervisor_validation import (
            IntentResolutionError,
        )

        with self.assertRaises(IntentResolutionError) as ctx:
            adapter._runtime_plan_to_semantic_steps(
                runtime_plan,
                current_pose_loader=lambda: None,
            )
        self.assertIn("100", str(ctx.exception))

    def test_repeat_node_missing_body_rejected(self) -> None:
        adapter = self._adapter()
        runtime_plan = {"type": "repeat", "count": 2}
        from hmi.backend.services.supervisor_validation import (
            IntentResolutionError,
        )

        with self.assertRaises(IntentResolutionError) as ctx:
            adapter._runtime_plan_to_semantic_steps(
                runtime_plan,
                current_pose_loader=lambda: None,
            )
        self.assertIn("body", str(ctx.exception).lower())

    def test_repeat_node_nested_inside_sequence(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "sequence",
            "children": [
                {"type": "skill", "name": "go_home"},
                {
                    "type": "repeat",
                    "count": 3,
                    "body": {
                        "type": "sequence",
                        "children": [{"type": "skill", "name": "stop"}],
                    },
                },
            ],
        }
        steps = adapter._runtime_plan_to_semantic_steps(
            runtime_plan,
            current_pose_loader=lambda: None,
        )
        intents = [step["intent"] for step in steps]
        self.assertEqual(intents, ["go_home", "stop", "stop", "stop"])
```

### Step 2: Run the new tests, expect FAIL

```bash
cd /home/hieu2/gp4_ws/hmi/backend
PYTHONPATH=/home/hieu2/gp4_ws:/home/hieu2/gp4_ws/src/llm_gateway \
  .venv/bin/python -m pytest tests/test_supervisor_sequence.py::RuntimePlanRepeatNodeTests -v
```

Expected: all 6 tests fail. Failures will be of the form `FactoryTask runtime node 'repeat' is not supported by the HMI Phase 1 adapter.`

### Step 3: Add the `REPEAT_MAX_COUNT` constant and `repeat` branch

In `hmi/backend/services/supervisor_sequence.py`, add a module-level constant near the existing `_BLENDED_SEQUENCE_ELIGIBLE` (around line 42):

```python
# Maximum number of iterations the HMI Phase 1 adapter will expand for a
# FactoryTask `repeat` runtime node. Prevents a malicious or buggy LLM
# payload from generating thousands of motion steps.
_REPEAT_MAX_COUNT = 100
```

Then replace the body of `_runtime_plan_to_semantic_steps` (lines 425–475) with the implementation that adds the `repeat` branch:

```python
    def _runtime_plan_to_semantic_steps(
        self,
        runtime_plan: dict[str, Any],
        *,
        current_pose_loader: Callable[[], dict[str, Any] | None] | None = None,
    ) -> list[dict[str, Any]]:
        node_type = str(runtime_plan.get("type") or "").strip().lower()
        if node_type == "sequence":
            children = runtime_plan.get("children")
            if not isinstance(children, list):
                raise IntentResolutionError(
                    "FactoryTask runtime sequence requires children."
                )
            steps: list[dict[str, Any]] = []
            for child in children:
                if not isinstance(child, dict):
                    raise IntentResolutionError(
                        "FactoryTask runtime sequence child must be an object."
                    )
                steps.extend(
                    self._runtime_plan_to_semantic_steps(
                        child,
                        current_pose_loader=current_pose_loader,
                    )
                )
            return steps
        if node_type == "repeat":
            body = runtime_plan.get("body")
            if not isinstance(body, dict):
                raise IntentResolutionError(
                    "FactoryTask runtime repeat node requires a 'body' object."
                )
            count = runtime_plan.get("count")
            if not isinstance(count, int) or isinstance(count, bool):
                raise IntentResolutionError(
                    "FactoryTask runtime repeat node requires an integer 'count'."
                )
            if count < 1:
                raise IntentResolutionError(
                    "FactoryTask runtime repeat count must be >= 1; got "
                    f"{count}."
                )
            if count > _REPEAT_MAX_COUNT:
                raise IntentResolutionError(
                    f"FactoryTask runtime repeat count {count} exceeds the "
                    f"HMI Phase 1 limit of {_REPEAT_MAX_COUNT}."
                )
            inner_steps = self._runtime_plan_to_semantic_steps(
                body,
                current_pose_loader=current_pose_loader,
            )
            return list(inner_steps) * count
        if node_type == "skill":
            semantic_ir = self._runtime_skill_to_semantic_ir(runtime_plan)
            intent = str(semantic_ir.get("intent") or "").strip().lower()
            if intent in {"draw_shape", "draw_text"}:
                semantic_ir = self._hydrate_draw_workplane(
                    semantic_ir,
                    current_pose_loader=current_pose_loader,
                )
                args = (
                    runtime_plan.get("args")
                    if isinstance(runtime_plan.get("args"), dict)
                    else {}
                )
                runtime_plan["args"] = {
                    **dict(args),
                    **{
                        key: value
                        for key, value in semantic_ir.items()
                        if key != "intent"
                    },
                }
            return [semantic_ir]
        raise IntentResolutionError(
            f"FactoryTask runtime node '{node_type or '<empty>'}' is not supported by the HMI Phase 1 adapter."
        )
```

The trailing `raise IntentResolutionError` is the original fallback (unchanged) — it stays so any future unsupported node type still produces a clear error.

### Step 4: Run the new tests, expect PASS

```bash
cd /home/hieu2/gp4_ws/hmi/backend
PYTHONPATH=/home/hieu2/gp4_ws:/home/hieu2/gp4_ws/src/llm_gateway \
  .venv/bin/python -m pytest tests/test_supervisor_sequence.py::RuntimePlanRepeatNodeTests -v
```

Expected: 6 passed.

### Step 5: Run the full test suite, expect all pass

```bash
cd /home/hieu2/gp4_ws/hmi/backend
PYTHONPATH=/home/hieu2/gp4_ws:/home/hieu2/gp4_ws/src/llm_gateway \
  .venv/bin/python -m pytest tests/ -q
```

Expected: 35 (baseline) + 6 (new) + 2 (Task 1 new) + existing all pass. Total at least 43, exact number depends on which other tests are added later.

### Step 6: Commit

```bash
cd /home/hieu2/gp4_ws
git add hmi/backend/services/supervisor_sequence.py hmi/backend/tests/test_supervisor_sequence.py
git commit -m "feat(hmi): expand FactoryTask repeat runtime node into N flat steps (Phase 1 adapter)"
```

---

## Self-Review (run before handoff)

1. **Spec coverage:**
   - Fix #1 "ROS future timed out" → user-friendly message: covered by Tasks 1+2 (mapping + context propagation).
   - Fix #2 `repeat` node unsupported: covered by Task 3 (repeat branch + count validation).
2. **Placeholder scan:** No "TBD", "TODO", "fill in details", or "similar to" references in code blocks.
3. **Type consistency:** `current_pose_loader: Callable[[], dict[str, Any] | None] | None` matches the existing signature on the method; new `context: str | None = None` keyword-only param is consistent across all call sites.
4. **Behaviour consistency:** The `repeat` branch's output (`list(inner_steps) * count`) is a flat list of semantic-IR dicts, identical in shape to what `sequence` produces — downstream `_parse_sequence_steps` and the per-step state machine do not need any change.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-06-12-hmi-submission-error-recovery.md`. Two execution options:

1. **Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.
2. **Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.
