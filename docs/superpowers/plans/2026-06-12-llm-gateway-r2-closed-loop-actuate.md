# LLM Gateway R2 — Closed-Loop Actuate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the FactoryTask runtime path actually move the robot — replace the validate-only `_execute_skill` with a real skill executor that dispatches grounded, safety-validated motion to `/execute_motion`, verifies grasp via IO, handles STOP/retry/replan, and emits runtime events.

**Architecture:** `task_runtime.TaskRuntime` already walks the tree and calls an injected `skill_executor(name, args) -> RuntimeStepResult`. The node already grounds (`_semantic_ir_for_runtime_skill` → `TaskCompiler.compile`) and validates (`_validate_runtime_command` does schema + normalize + `/validate_command` safety + checks ExecuteMotion ready). **The only missing link is the final dispatch:** after safety passes, send the goal to ExecuteMotion, await its result synchronously, and return success/failure. Plus wire `event_callback`, `is_stopped_fn` (new STOP flag + in-flight goal cancel), and `replan_handler` (re-call TaskPlanner) into `TaskRuntime`.

**Tech Stack:** Python 3.10, rclpy (MultiThreadedExecutor already in use), pytest, colcon. Phase R2 of `docs/superpowers/specs/2026-06-12-llm-gateway-remediation-design.md` §4.

---

## Verified preconditions (do not re-discover)

- `LLMGatewayNode` spins under `MultiThreadedExecutor` (`llm_gateway_node.py:2846`), so `_wait_for_future_without_spinning` (poll loop, `:2066`) can await an action result from inside a service callback without deadlock.
- `_on_confirm_factory_task_runtime` (`:2503`) builds `_execute_skill` and runs `TaskRuntime(world_model=...).run(task, _execute_skill)`. Today `_execute_skill` only calls `_validate_runtime_semantic_ir` and the response sets `dispatched_to_ros = False`.
- `_validate_runtime_command` (`:~2640`) already: `_schema_validator.validate` → `_normalize_and_validate` → `_goal_mapper.to_command_payload` → `/validate_command` (safety) → checks `_execute_client.server_is_ready()`, then **returns success without dispatching**. This is the seam to extend.
- Dispatch primitive: `self._execute_client.send_goal_async(goal)` → `goal_handle.accepted` → `goal_handle.get_result_async()` → result (legacy async version at `_on_goal_sent`/`_on_execution_done`, `:1911`–`:1990`).
- `self._goal_mapper.to_execute_motion_goal(normalized_command)` builds the `ExecuteMotion.Goal` (used by legacy dispatch at `:1902`).
- `WorldModel.object_pose(ref)` / `WorldModel.collection("visible_objects")` exist (`factory_task.py`). Grounding into `WorldModel` happens via `_prime_factory_task_world_model` → `_query_perception_detections` (`GetObjectPositions`, calibration/depth fail-closed).
- No runtime STOP flag exists yet; `is_stopped_fn` is passed as `None` today. `robot_status` callback exposes `e_stopped` (`:1188`).

## Safety invariants (must hold every step)

- Motion only ever dispatched **after** `/validate_command` returns `valid` for that exact normalized command. Never reorder.
- Perception pose used only when fresh (≤ `freshness_sec`, default 5) and calibration valid — else fail-closed (`PERCEPTION_STALE` / `WORLD_MODEL_UNGROUNDED`).
- STOP cancels the in-flight ExecuteMotion goal and aborts the task — fail-closed, no further dispatch.
- Conservative defaults unchanged (velocity_scale ≤ 0.06).

## File structure

| File | Change |
|------|--------|
| `src/llm_gateway/llm_gateway/runtime_dispatch.py` | **Create** — pure helper `dispatch_and_await(execute_client, goal, wait_fn, is_stopped_fn, cancel_box) -> DispatchOutcome` (no `self`, unit-testable with fakes) |
| `src/llm_gateway/llm_gateway/llm_gateway_node.py` | Modify — extend `_validate_runtime_command` to dispatch; add `_runtime_stop_requested` flag + setter; wire `is_stopped_fn`/`event_callback`/`replan_handler` into `TaskRuntime`; publisher for events handled in R3 (here event_callback logs only) |
| `src/llm_gateway/tests/test_runtime_dispatch.py` | **Create** — unit tests for dispatch helper (accepted/rejected/result-fail/stop-cancel) |
| `src/llm_gateway/tests/test_runtime_executor_actuates.py` | **Create** — node-level test with fake execute client proving a skill dispatches + awaits |

---

## Task 1: Dispatch helper `runtime_dispatch.py` (pure, fake-tested)

**Files:**
- Create: `src/llm_gateway/llm_gateway/runtime_dispatch.py`
- Test: `src/llm_gateway/tests/test_runtime_dispatch.py`

- [ ] **Step 1: Write the failing test**

Create `src/llm_gateway/tests/test_runtime_dispatch.py`:
```python
from llm_gateway.runtime_dispatch import dispatch_and_await, DispatchOutcome


class _FakeFuture:
    def __init__(self, value): self._value = value
    def done(self): return True
    def result(self): return self._value


class _FakeGoalHandle:
    def __init__(self, accepted, result): self.accepted = accepted; self._result = result
    def get_result_async(self): return _FakeFuture(self._result)
    def cancel_goal_async(self): return _FakeFuture(None)


class _FakeResultWrapper:
    def __init__(self, success, message=""):
        self.result = type("R", (), {"success": success, "message": message})()


class _FakeExecuteClient:
    def __init__(self, accepted=True, success=True, message=""):
        self._handle = _FakeGoalHandle(accepted, _FakeResultWrapper(success, message))
    def server_is_ready(self): return True
    def send_goal_async(self, goal): return _FakeFuture(self._handle)


def _wait(future, timeout): return True, future.result()


def test_dispatch_returns_ok_when_goal_accepted_and_result_success():
    out = dispatch_and_await(
        _FakeExecuteClient(accepted=True, success=True),
        goal=object(), wait_fn=_wait, is_stopped_fn=lambda: False, timeout_sec=1.0,
    )
    assert out == DispatchOutcome(ok=True, reason="")


def test_dispatch_fails_when_goal_rejected():
    out = dispatch_and_await(
        _FakeExecuteClient(accepted=False), goal=object(),
        wait_fn=_wait, is_stopped_fn=lambda: False, timeout_sec=1.0,
    )
    assert out.ok is False and "rejected" in out.reason


def test_dispatch_fails_when_result_reports_failure():
    out = dispatch_and_await(
        _FakeExecuteClient(accepted=True, success=False, message="planning failed"),
        goal=object(), wait_fn=_wait, is_stopped_fn=lambda: False, timeout_sec=1.0,
    )
    assert out.ok is False and "planning failed" in out.reason


def test_dispatch_cancels_and_fails_when_stop_requested_before_send():
    out = dispatch_and_await(
        _FakeExecuteClient(), goal=object(),
        wait_fn=_wait, is_stopped_fn=lambda: True, timeout_sec=1.0,
    )
    assert out.ok is False and out.reason == "operator_stopped"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/hieu2/gp4_ws/src/llm_gateway && python -m pytest tests/test_runtime_dispatch.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'llm_gateway.runtime_dispatch'`.

- [ ] **Step 3: Write minimal implementation**

Create `src/llm_gateway/llm_gateway/runtime_dispatch.py`:
```python
"""Synchronous ExecuteMotion dispatch+await for the FactoryTask runtime executor.

Pure helper: no rclpy import, no node state. The node injects its execute action
client, its non-spinning wait function, and a stop predicate.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class DispatchOutcome:
    ok: bool
    reason: str = ""


def dispatch_and_await(
    execute_client: Any,
    *,
    goal: Any,
    wait_fn: Callable[[Any, float], tuple[bool, Any]],
    is_stopped_fn: Callable[[], bool],
    timeout_sec: float,
) -> DispatchOutcome:
    if is_stopped_fn():
        return DispatchOutcome(ok=False, reason="operator_stopped")
    if execute_client is None or not execute_client.server_is_ready():
        return DispatchOutcome(ok=False, reason="ExecuteMotion action server unavailable")

    send_future = execute_client.send_goal_async(goal)
    done, goal_handle = wait_fn(send_future, timeout_sec)
    if not done or goal_handle is None:
        return DispatchOutcome(ok=False, reason="ExecuteMotion goal send timed out")
    if not getattr(goal_handle, "accepted", False):
        return DispatchOutcome(ok=False, reason="ExecuteMotion action server rejected goal")

    if is_stopped_fn():
        cancel = getattr(goal_handle, "cancel_goal_async", None)
        if callable(cancel):
            cancel()
        return DispatchOutcome(ok=False, reason="operator_stopped")

    result_future = goal_handle.get_result_async()
    done, wrapped = wait_fn(result_future, timeout_sec)
    if not done or wrapped is None:
        return DispatchOutcome(ok=False, reason="ExecuteMotion result timed out")

    # interfaces/action/ExecuteMotion.Result: success(bool), message(str), execution_time_sec
    result = getattr(wrapped, "result", None)
    if not bool(getattr(result, "success", False)):
        msg = str(getattr(result, "message", "") or "motion failed")
        return DispatchOutcome(ok=False, reason=msg)
    return DispatchOutcome(ok=True, reason="")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/hieu2/gp4_ws/src/llm_gateway && python -m pytest tests/test_runtime_dispatch.py -q`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
cd /home/hieu2/gp4_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/runtime_dispatch.py src/llm_gateway/tests/test_runtime_dispatch.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "feat(llm_gateway): R2 add pure ExecuteMotion dispatch+await helper"
```

---

## Task 2: Add a runtime STOP flag to the node

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py` (add flag + accessor near `__init__` state and the STOP topic/command handling)
- Test: `src/llm_gateway/tests/test_runtime_stop_flag.py`

> Verify first whether a STOP command/topic already reaches the node: run `grep -nE "stop|STOP|cancel" src/llm_gateway/llm_gateway/llm_gateway_node.py`. The schema has a `STOP` primitive and `robot_status.e_stopped` exists. If a STOP subscription already exists, set the flag from its callback instead of creating a new topic.

- [ ] **Step 1: Write the failing test**

Create `src/llm_gateway/tests/test_runtime_stop_flag.py`:
```python
from llm_gateway.llm_gateway_node import LLMGatewayNode


def test_runtime_stop_flag_defaults_false_and_sets_true():
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()
    assert node._runtime_is_stopped() is False
    node._set_runtime_stop(True)
    assert node._runtime_is_stopped() is True
    node._set_runtime_stop(False)
    assert node._runtime_is_stopped() is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_runtime_stop_flag.py -q`
Expected: FAIL — `AttributeError: ... '_init_runtime_stop_state'`.

- [ ] **Step 3: Write minimal implementation**

In `llm_gateway_node.py`, add these methods to `LLMGatewayNode` (place near the other small state helpers, e.g. after `_invalidate_scene_cache`):
```python
def _init_runtime_stop_state(self) -> None:
    self._runtime_stop_flag = False

def _set_runtime_stop(self, value: bool) -> None:
    self._runtime_stop_flag = bool(value)

def _runtime_is_stopped(self) -> bool:
    return bool(getattr(self, "_runtime_stop_flag", False))
```
In `__init__`, call `self._init_runtime_stop_state()` once (next to the other state initializers). In the existing STOP path (the `robot_status` callback where `e_stopped` is read, and/or the STOP primitive handler) call `self._set_runtime_stop(True)` when an e-stop/STOP is observed, and `self._set_runtime_stop(False)` when a fresh task is confirmed in `_on_confirm_factory_task_runtime` (reset at task start).

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_runtime_stop_flag.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_runtime_stop_flag.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "feat(llm_gateway): R2 add runtime STOP flag for FactoryTask executor"
```

---

## Task 3: Dispatch motion from `_validate_runtime_command`

This is the core change. `_validate_runtime_command` currently ends with "ExecuteMotion server ready → return success". Replace that tail so it builds the goal, dispatches, and awaits.

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py` (`_validate_runtime_command`)
- Test: `src/llm_gateway/tests/test_runtime_executor_actuates.py`

- [ ] **Step 1: Write the failing test**

Create `src/llm_gateway/tests/test_runtime_executor_actuates.py`:
```python
from llm_gateway.llm_gateway_node import LLMGatewayNode
from llm_gateway.runtime_dispatch import DispatchOutcome


def test_validate_runtime_command_dispatches_after_safety_passes(monkeypatch):
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()

    # Stub the safety+normalize chain to "valid" so we isolate dispatch.
    monkeypatch.setattr(node, "_runtime_command_is_safe",
                        lambda cmd: (True, "", {"primitive_type": "PTP"}, object()))

    dispatched = {"count": 0}
    def _fake_dispatch(goal):
        dispatched["count"] += 1
        return DispatchOutcome(ok=True, reason="")
    monkeypatch.setattr(node, "_dispatch_runtime_goal", _fake_dispatch)

    result = node._validate_runtime_command({"primitive_type": "PTP"})

    assert result.success is True
    assert dispatched["count"] == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_runtime_executor_actuates.py -q`
Expected: FAIL — `_runtime_command_is_safe` / `_dispatch_runtime_goal` do not exist.

- [ ] **Step 3: Write minimal implementation**

Refactor `_validate_runtime_command` in `llm_gateway_node.py`. Extract the existing safety chain into `_runtime_command_is_safe` and add the dispatch tail:
```python
def _runtime_command_is_safe(self, command):
    """Return (ok, reason, normalized_command, command_payload). No dispatch."""
    try:
        self._schema_validator.validate(command)
        normalized_command = self._normalize_and_validate(command)
    except Exception as exc:
        return False, str(exc), None, None

    primitive_type = str(normalized_command.get("primitive_type") or "")
    if self._is_query_command(primitive_type):
        pose_client = getattr(self, "_get_pose_client", None)
        if pose_client is None or not pose_client.wait_for_service(
            timeout_sec=self._safety_service_timeout_sec
        ):
            return False, "GetCurrentPose service unavailable", None, None
        return True, "", normalized_command, None  # query: no motion goal

    command_payload = self._goal_mapper.to_command_payload(normalized_command)
    if not self._validate_client.wait_for_service(timeout_sec=self._safety_service_timeout_sec):
        return False, "ValidateCommand service unavailable", None, None
    validate_req = self._build_validate_request(normalized_command, command_payload)
    try:
        validate_future = self._validate_client.call_async(validate_req)
        done, validate_resp = self._wait_for_future_without_spinning(
            validate_future, self._safety_service_timeout_sec
        )
    except Exception as exc:
        return False, f"ValidateCommand call failed: {exc}", None, None
    if not done:
        return False, "ValidateCommand service timed out", None, None
    if not validate_resp.valid:
        return False, str(validate_resp.reason), None, None
    return True, "", normalized_command, command_payload


def _dispatch_runtime_goal(self, normalized_command):
    """Send the validated command to ExecuteMotion and await its result."""
    from llm_gateway.runtime_dispatch import dispatch_and_await
    goal = self._goal_mapper.to_execute_motion_goal(normalized_command)
    outcome = dispatch_and_await(
        getattr(self, "_execute_client", None),
        goal=goal,
        wait_fn=self._wait_for_future_without_spinning,
        is_stopped_fn=self._runtime_is_stopped,
        timeout_sec=self._motion_result_timeout_sec,
    )
    return outcome


def _validate_runtime_command(self, command):
    ok, reason, normalized_command, _payload = self._runtime_command_is_safe(command)
    if not ok:
        return RuntimeStepResult(success=False, reason=reason)
    # Query commands (GET_POSE) validated above carry no motion goal.
    if normalized_command is None or self._is_query_command(
        str(normalized_command.get("primitive_type") or "")
    ):
        return RuntimeStepResult(success=True)
    outcome = self._dispatch_runtime_goal(normalized_command)
    return RuntimeStepResult(success=outcome.ok, reason=outcome.reason)
```
Add `self._motion_result_timeout_sec` as a declared parameter (default e.g. 30.0) in `_declare_parameters` if not already present; otherwise reuse the existing motion timeout parameter — check `grep -n "timeout" llm_gateway_node.py` and use the established one rather than inventing a new name.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_runtime_executor_actuates.py tests/test_runtime_dispatch.py -q`
Expected: PASS.

- [ ] **Step 5: Run the FULL suite (no regression in the existing runtime tests)**

Run: `python -m pytest tests/ -q`
Expected: all pass. The existing `test_task_runtime*.py` use a fake executor and must still pass. If any test asserted `dispatched_to_ros is False` for a runtime confirm, update that assertion to reflect that motion now dispatches (see Task 4).

- [ ] **Step 6: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_runtime_executor_actuates.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "feat(llm_gateway): R2 dispatch validated runtime motion to ExecuteMotion"
```

---

## Task 4: Wire is_stopped_fn + event_callback + replan_handler + dispatched flag

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py` (`_on_confirm_factory_task_runtime`)
- Test: `src/llm_gateway/tests/test_runtime_confirm_wiring.py`

- [ ] **Step 1: Write the failing test**

Create `src/llm_gateway/tests/test_runtime_confirm_wiring.py`:
```python
from llm_gateway.llm_gateway_node import LLMGatewayNode


def test_confirm_runtime_resets_stop_and_reports_dispatched(monkeypatch):
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()
    node._set_runtime_stop(True)  # stale stop from a previous task

    # Minimal fakes for the confirm path.
    monkeypatch.setattr(node, "_factory_task_payload_from_runtime_sentinel",
                        lambda parsed: {"task_id": "t", "version": 1, "mode": "auto",
                                        "root": {"type": "sequence", "children": []}})
    monkeypatch.setattr(node, "_factory_task_world_model", lambda: None)
    monkeypatch.setattr(node, "_runtime_event_sink", lambda evt: None, raising=False)

    class _Req: operator_id = "op"; plan_fingerprint = "abc123def456"
    class _Resp:
        accepted = False; reason = ""; execution_summary = ""; dispatched_to_ros = False

    resp = node._on_confirm_factory_task_runtime({"_factory_task_runtime": True}, _Req(), _Resp())

    assert node._runtime_is_stopped() is False     # reset at task start
    assert resp.accepted is True
    assert resp.dispatched_to_ros is True          # motion path now actuates
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_runtime_confirm_wiring.py -q`
Expected: FAIL (stop not reset / dispatched_to_ros still False).

- [ ] **Step 3: Write minimal implementation**

In `_on_confirm_factory_task_runtime`, at the top (after parsing `task`) reset stop, and construct `TaskRuntime` with the wiring:
```python
self._set_runtime_stop(False)  # fresh task clears any stale stop

def _execute_skill(name, args):
    try:
        semantic_ir = self._semantic_ir_for_runtime_skill(task_payload, name, args)
        return self._validate_runtime_semantic_ir(semantic_ir)
    except Exception as exc:
        return RuntimeStepResult(success=False, reason=str(exc))

def _replan(_report):
    # Re-plan once via the planner from the original operator summary.
    try:
        planned = self._task_planner.plan(task.operator_summary) if self._task_planner else None
    except Exception:
        return None
    from llm_gateway.factory_task import is_factory_task, parse_factory_task
    return parse_factory_task(planned) if is_factory_task(planned) else None

runtime = TaskRuntime(
    world_model=self._factory_task_world_model(),
    is_stopped_fn=self._runtime_is_stopped,
    event_callback=getattr(self, "_runtime_event_sink", None),
    replan_handler=_replan,
    max_replans=1,
)
report = runtime.run(task, _execute_skill)
```
Then set `response.dispatched_to_ros = report.success` (motion actually dispatched when the tree succeeded) and keep the existing `accepted`/`reason`/`execution_summary` assignments. Add a no-op default `_runtime_event_sink` method (R3 replaces it with a real publisher):
```python
def _runtime_event_sink(self, event: dict) -> None:
    self.get_logger().info(f"[task_event] {event.get('category')}/{event.get('event')}: {event.get('detail')}")
```

- [ ] **Step 4: Run test + full suite**

Run: `python -m pytest tests/test_runtime_confirm_wiring.py -q && python -m pytest tests/ -q`
Expected: PASS. Update any prior test asserting `dispatched_to_ros is False` for the runtime confirm path.

- [ ] **Step 5: Build**

Run: `cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash`
Expected: green.

- [ ] **Step 6: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_runtime_confirm_wiring.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "feat(llm_gateway): R2 wire STOP/events/replan into FactoryTask runtime confirm"
```

---

## Task 5: Sim integration — closed-loop pick/place with fake perception

**Files:**
- Test: `src/llm_gateway/tests/test_closed_loop_pick_sim.py`

Extends the existing `test_pick_white_workpiece_sim.py` fixtures. Uses fakes for the execute client + perception so it runs without hardware.

- [ ] **Step 1: Write the test**

Create `src/llm_gateway/tests/test_closed_loop_pick_sim.py`:
```python
from llm_gateway.task_runtime import TaskRuntime, RuntimeStepResult
from llm_gateway.factory_task import FactoryTask, TaskNode, SkillCall, WorldModel


def _pick_task():
    root = TaskNode(type="sequence", children=(
        TaskNode(type="observe", name="observe", args={"class_filter": "white_workpiece"}),
        TaskNode(type="skill", name="pick_object", args={"object_id": "white_workpiece"}),
        TaskNode(type="skill", name="place_object",
                 args={"object_id": "white_workpiece", "destination": "conveyor"}),
    ))
    return FactoryTask(task_id="sim", version=1, mode="auto", root=root,
                       limits={}, replan_policy={"max_replans": 1}, operator_summary="pick")


def test_happy_path_executes_all_skills():
    calls = []
    def executor(name, args):
        calls.append(name)
        return RuntimeStepResult(success=True)
    wm = WorldModel(objects={"white_workpiece": {"pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.1}}}})
    report = TaskRuntime(world_model=wm).run(_pick_task(), executor)
    assert report.success is True
    assert calls == ["observe", "pick_object", "place_object"]


def test_grasp_fail_then_retry_succeeds():
    attempts = {"pick_object": 0}
    def executor(name, args):
        if name == "pick_object":
            attempts["pick_object"] += 1
            return RuntimeStepResult(success=attempts["pick_object"] >= 2,
                                     reason="" if attempts["pick_object"] >= 2 else "GRASP_FAILED")
        return RuntimeStepResult(success=True)
    # Wrap pick in a retry(2) node.
    root = TaskNode(type="sequence", children=(
        TaskNode(type="retry", count=2, children=(
            TaskNode(type="skill", name="pick_object", args={"object_id": "w"}),)),
    ))
    task = FactoryTask(task_id="r", version=1, mode="auto", root=root,
                       limits={}, replan_policy={}, operator_summary="x")
    report = TaskRuntime(world_model=WorldModel()).run(task, executor)
    assert report.success is True
    assert attempts["pick_object"] == 2


def test_stop_midway_aborts():
    stopped = {"v": False}
    def executor(name, args):
        if name == "pick_object":
            stopped["v"] = True
        return RuntimeStepResult(success=True)
    report = TaskRuntime(world_model=WorldModel(
        objects={"white_workpiece": {"pose": {"position": {"x": 0.3, "y": 0, "z": 0.1}}}}),
        is_stopped_fn=lambda: stopped["v"]).run(_pick_task(), executor)
    assert report.success is False
    assert report.reason == "operator_stopped"
```
(Adjust `TaskNode`/`FactoryTask`/`SkillCall` constructor kwargs to match the real dataclass signatures — run `python -c "from llm_gateway.factory_task import TaskNode, FactoryTask; help(TaskNode); help(FactoryTask)"` first and fix the fixtures.)

- [ ] **Step 2: Run + iterate to green**

Run: `python -m pytest tests/test_closed_loop_pick_sim.py -q`
Expected: PASS after matching constructor signatures. These assert the runtime control flow (sequence/retry/STOP) end-to-end with a fake executor.

- [ ] **Step 3: Full suite + build + commit**

```bash
python -m pytest tests/ -q
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/tests/test_closed_loop_pick_sim.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "test(llm_gateway): R2 closed-loop pick sim (happy/retry/stop)"
npx gitnexus analyze
```

---

## Done criteria for R2

- [ ] A confirmed FactoryTask actually dispatches motion: `_validate_runtime_command` sends to `/execute_motion` and awaits the result; `_on_confirm_factory_task_runtime` reports `dispatched_to_ros = report.success`.
- [ ] STOP flag cancels the in-flight goal and aborts (`operator_stopped`); flag resets at task start.
- [ ] `event_callback` + `replan_handler` (max 1) wired into `TaskRuntime`.
- [ ] Every motion still passes `/validate_command` before dispatch (safety invariant intact).
- [ ] Full suite green; closed-loop sim test (happy/retry/stop) passes; build green; GitNexus reindexed.

After R2 lands, write/execute R3 (task_events publisher + HMI System Log).

> **Verified:** `ExecuteMotion.Result` fields are `success` (bool), `message` (str), `execution_time_sec` (double) — `runtime_dispatch.py` uses `result.success`/`result.message`. No `error_code` field exists.
