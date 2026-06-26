# Grasp Pipeline Hardware Commissioning Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the end-to-end vision → LLM → grasp-IO pipeline actually pick a `yellow_box` and place it on the `fixture` on real hardware (no simulation), by fixing the three functional gaps that block a real grasp.

**Architecture:** `pick_object`/`place_object` are FactoryTask skills compiled in `factory_task.py:_compile_skill` into a Semantic-IR `sequence`, dispatched step-by-step through the existing `io_set → IO_SET → motion_core → hw_adapter` and `absolute_move_ptp/move_relative → ExecuteMotion` paths. We only change what those two skills compile to. We do **not** add new dispatch plumbing — the routes already work (proven by the new air-valve `io_set` command).

**Tech Stack:** Python 3.10 (llm_gateway), ROS 2 Humble, pytest. MotoROS2 WriteSingleIO on the YRC1000micro.

## Global Constraints

- Conservative motion only: `velocity_scale=0.06`, `acceleration_scale=0.06` (verbatim from `safety_rules.yaml motion_limits`).
- Gripper IO is air/vacuum on **address 10017** (`open_output_value=0`, `close_output_value=1`), grasp feedback input **address 30017** active value `1` (from `src/safety/config/safety_rules.yaml:150-158`). Never hardcode these in code — read from the verified `GripperConfig`.
- `factory_task.py` is NOT allowed to call ROS hardware services. It may only *emit* Semantic IR; the node injects gripper addresses at compile time.
- Tool-down grasp orientation = quaternion `{x:1.0, y:0.0, z:0.0, w:0.0}` (matches `safety_rules.yaml safe_home`).
- No simulation. Validation that needs hardware is a **manual commissioning checklist** (Task 6), not an automated test.

---

### Task 1: Gripper compile-config injected into TaskCompiler

**Files:**
- Modify: `src/llm_gateway/llm_gateway/factory_task.py` (add dataclass near `class TaskCompiler` ~line 405; add `gripper` param to `TaskCompiler.__init__`)
- Test: `src/llm_gateway/tests/test_grasp_pipeline.py` (create)

**Interfaces:**
- Produces: `GripperCompileConfig(close: tuple[int,int], open: tuple[int,int])`; `TaskCompiler(*, world_model=..., policy_engine=..., gripper: GripperCompileConfig | None = None)`. `self._gripper` available inside `_compile_skill`.

- [ ] **Step 1: Write the failing test**

```python
# src/llm_gateway/tests/test_grasp_pipeline.py
from llm_gateway.factory_task import TaskCompiler, GripperCompileConfig, WorldModel

def test_taskcompiler_accepts_gripper_config():
    gc = GripperCompileConfig(close=(10017, 1), open=(10017, 0))
    tc = TaskCompiler(world_model=WorldModel(), gripper=gc)
    assert tc._gripper.close == (10017, 1)
    assert tc._gripper.open == (10017, 0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/llm_gateway && python -m pytest tests/test_grasp_pipeline.py::test_taskcompiler_accepts_gripper_config -v`
Expected: FAIL with `ImportError: cannot import name 'GripperCompileConfig'`

- [ ] **Step 3: Write minimal implementation**

In `factory_task.py`, immediately above `class TaskCompiler:` (~line 405) add:

```python
@dataclass(frozen=True)
class GripperCompileConfig:
    """Real gripper IO addresses, injected by the node at compile time.

    factory_task.py never reads ROS config directly; the node passes the
    verified GripperConfig values in as (address, value) pairs.
    """
    close: tuple[int, int]
    open: tuple[int, int]
```

In `TaskCompiler.__init__`, add the parameter and store it:

```python
    def __init__(
        self,
        *,
        world_model: WorldModel | None = None,
        policy_engine: PolicyEngine | None = None,
        gripper: "GripperCompileConfig | None" = None,
    ) -> None:
        self._world_model = world_model or WorldModel()
        self._policy_engine = policy_engine or PolicyEngine()
        self._gripper = gripper
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/llm_gateway && python -m pytest tests/test_grasp_pipeline.py::test_taskcompiler_accepts_gripper_config -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/llm_gateway/factory_task.py src/llm_gateway/tests/test_grasp_pipeline.py
git commit -m "feat(factory_task): inject gripper IO config into TaskCompiler"
```

---

### Task 2: `pick_object` compiles a real approach→descend→close→lift sequence

**Files:**
- Modify: `src/llm_gateway/llm_gateway/factory_task.py:571-635` (the `if node.name in {"pick_object", "place_object", "pick_and_place"}:` block in `_compile_skill`)
- Test: `src/llm_gateway/tests/test_grasp_pipeline.py`

**Interfaces:**
- Consumes: `self._gripper` (Task 1), `self._world_model.object_pose(object_ref)`.
- Produces: `pick_object` → `{"intent":"sequence","steps":[approach(absolute_move_ptp, tool-down, z+clearance), descend(move_relative -z), io_set(close addr/val), lift(move_relative +z)]}`. Optional args: `approach_clearance_m` (default 0.08), `descend_m` (default = clearance).

- [ ] **Step 1: Write the failing test**

```python
def _gripper():
    return GripperCompileConfig(close=(10017, 1), open=(10017, 0))

def _wm():
    # detection object + the fixture region (geometry.center form)
    return WorldModel(
        objects={"yellow_box": {"pose": {"position": {"x": 0.30, "y": 0.10, "z": 0.05},
                                          "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}},
        regions={"fixture": {"geometry": {"center": {"x": 0.281, "y": 0.182, "z": 0.25}}}},
    )

def _compile_skill_dict(skill_name, args):
    from llm_gateway.factory_task import parse_factory_task
    payload = {"task_type": "factory_task", "version": "1.0", "task_id": "t",
               "root": {"type": "skill", "name": skill_name, "args": args}}
    tc = TaskCompiler(world_model=_wm(), gripper=_gripper())
    return tc.compile(parse_factory_task(payload)).semantic_ir

def test_pick_object_emits_grasp_sequence_with_real_io():
    ir = _compile_skill_dict("pick_object", {"object_ref": "yellow_box"})
    steps = ir["steps"]
    purposes = [s["metadata"]["purpose"] for s in steps]
    assert purposes == ["pick_approach", "pick_descend", "pick_gripper", "pick_lift"]
    approach = steps[0]
    assert approach["intent"] == "absolute_move_ptp"
    assert approach["target_pose"]["position"]["z"] == 0.05 + 0.08
    assert approach["target_pose"]["orientation"] == {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
    assert steps[1]["intent"] == "move_relative" and steps[1]["delta"]["z"] == -0.08
    io = steps[2]
    assert io["intent"] == "io_set" and io["io_address"] == 10017 and io["io_value"] == 1
    assert steps[3]["intent"] == "move_relative" and steps[3]["delta"]["z"] == 0.08
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/llm_gateway && python -m pytest tests/test_grasp_pipeline.py::test_pick_object_emits_grasp_sequence_with_real_io -v`
Expected: FAIL (current code emits `io_address: 0` and no descend/lift)

- [ ] **Step 3: Write minimal implementation**

Replace the entire `if node.name in {"pick_object", "place_object", "pick_and_place"}:` block (currently `factory_task.py:571-635`) with:

```python
        if node.name in {"pick_object", "place_object", "pick_and_place"}:
            if self._gripper is None:
                raise FactoryTaskError(f"{node.name} at {path} requires gripper config")
            frame = str(args.get("reference_frame") or "base_link")
            clearance = float(args.get("approach_clearance_m", 0.08))
            descend = float(args.get("descend_m", clearance))

            if node.name == "place_object":
                destination = args.get("destination")
                if not destination:
                    raise FactoryTaskError(f"place_object at {path} requires destination")
                target = self._world_model.object_pose(destination)
                io_address, io_value = self._gripper.open
                purpose = "place"
            else:
                object_ref = args.get("object_ref") or args.get("object") or args.get("object_id")
                if object_ref is None:
                    raise FactoryTaskError(f"{node.name} at {path} requires object_ref")
                target = self._world_model.object_pose(object_ref)
                io_address, io_value = self._gripper.close
                purpose = "pick"

            pos = target.get("position") or target
            approach_pose = {
                "position": {
                    "x": float(pos["x"]),
                    "y": float(pos["y"]),
                    "z": float(pos["z"]) + clearance,
                },
                "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
            }
            return {
                "intent": "sequence",
                "metadata": {"source": f"factory_task.{node.name}", "tool_changed_world": True},
                "steps": [
                    {"intent": "absolute_move_ptp", "target_pose": approach_pose,
                     "reference_frame": frame, "metadata": {"purpose": f"{purpose}_approach"}},
                    {"intent": "move_relative", "delta": {"x": 0.0, "y": 0.0, "z": -descend},
                     "reference_frame": frame, "metadata": {"purpose": f"{purpose}_descend"}},
                    {"intent": "io_set", "io_address": int(io_address), "io_value": int(io_value),
                     "metadata": {"purpose": f"{purpose}_gripper"}},
                    {"intent": "move_relative", "delta": {"x": 0.0, "y": 0.0, "z": descend},
                     "reference_frame": frame, "metadata": {"purpose": f"{purpose}_lift"}},
                ],
            }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/llm_gateway && python -m pytest tests/test_grasp_pipeline.py::test_pick_object_emits_grasp_sequence_with_real_io -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/llm_gateway/factory_task.py src/llm_gateway/tests/test_grasp_pipeline.py
git commit -m "feat(factory_task): pick_object emits real approach/descend/grasp/lift with configured gripper IO"
```

---

### Task 3: `place_object` moves to the destination region (fixture)

**Files:**
- Test: `src/llm_gateway/tests/test_grasp_pipeline.py` (implementation already done in Task 2's shared block — this task locks the destination behavior with its own test)

**Interfaces:**
- Consumes: Task 2 compile block; `WorldModel.object_pose("fixture")` resolving the region center.

- [ ] **Step 1: Write the failing test**

```python
def test_place_object_moves_to_destination_region_and_opens():
    ir = _compile_skill_dict("place_object", {"object": "yellow_box", "destination": "fixture"})
    steps = ir["steps"]
    approach = steps[0]
    # Must approach the FIXTURE center (0.281,0.182,0.25)+clearance, not the object (0.30,0.10,0.05)
    assert approach["target_pose"]["position"]["x"] == 0.281
    assert approach["target_pose"]["position"]["y"] == 0.182
    assert approach["target_pose"]["position"]["z"] == 0.25 + 0.08
    io = steps[2]
    assert io["io_address"] == 10017 and io["io_value"] == 0  # open/release
    assert [s["metadata"]["purpose"] for s in steps] == \
        ["place_approach", "place_descend", "place_gripper", "place_lift"]
```

- [ ] **Step 2: Run test to verify it fails or passes**

Run: `cd src/llm_gateway && python -m pytest tests/test_grasp_pipeline.py::test_place_object_moves_to_destination_region_and_opens -v`
Expected: PASS (Task 2's block already implements destination resolution). If FAIL, fix the `place_object` branch in Task 2 before continuing — destination must resolve via `self._world_model.object_pose(destination)`.

- [ ] **Step 3: Add a missing-destination guard test**

```python
def test_place_object_requires_destination():
    import pytest
    from llm_gateway.factory_task import FactoryTaskError
    with pytest.raises(FactoryTaskError):
        _compile_skill_dict("place_object", {"object": "yellow_box"})
```

- [ ] **Step 4: Run both tests**

Run: `cd src/llm_gateway && python -m pytest tests/test_grasp_pipeline.py -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/tests/test_grasp_pipeline.py
git commit -m "test(factory_task): lock place_object destination-region resolution"
```

---

### Task 4: Node injects the verified gripper config when compiling runtime skills

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py:1876-1896` (`_semantic_ir_for_runtime_skill`)
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py` (add import of `GripperCompileConfig` and a small helper)

**Interfaces:**
- Consumes: `self._gripper_adapter._config` (already verified-checked at the top of `_semantic_ir_for_runtime_skill`), `TaskCompiler(..., gripper=...)` (Task 1).

- [ ] **Step 1: Add the import**

At the top of `llm_gateway_node.py`, in the existing factory_task import block, add `GripperCompileConfig`:

```python
from llm_gateway.factory_task import (
    TaskCompiler,
    parse_factory_task,
    is_factory_task,
    GripperCompileConfig,
)
```
(Match the existing import style/grouping already present for `TaskCompiler`/`parse_factory_task`.)

- [ ] **Step 2: Inject gripper config at compile**

In `_semantic_ir_for_runtime_skill` (~line 1891), change:

```python
        compiled = TaskCompiler(world_model=self._factory_task_world_model()).compile(
            parse_factory_task(single_task_payload)
        )
```
to:

```python
        gripper_compile_config = None
        config = getattr(getattr(self, "_gripper_adapter", None), "_config", None)
        if config is not None and config.verified():
            gripper_compile_config = GripperCompileConfig(
                close=(int(config.close_output_address), int(config.close_output_value)),
                open=(int(config.open_output_address), int(config.open_output_value)),
            )
        compiled = TaskCompiler(
            world_model=self._factory_task_world_model(),
            gripper=gripper_compile_config,
        ).compile(parse_factory_task(single_task_payload))
```

- [ ] **Step 3: Verify imports/compile clean**

Run: `cd src/llm_gateway && python -c "import llm_gateway.llm_gateway_node"`
Expected: no error (no exception printed).

- [ ] **Step 4: Run the full gateway test suite to catch regressions**

Run: `cd src/llm_gateway && python -m pytest tests/ -q`
Expected: PASS (note any pre-existing failures unrelated to grasp; the new `test_grasp_pipeline.py` must pass).

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/llm_gateway/llm_gateway_node.py
git commit -m "feat(gateway): pass verified gripper IO addresses into skill compilation"
```

---

### Task 5: LLM planner knows the pick→verify→place-to-fixture shape

**Files:**
- Modify: `src/llm_gateway/llm_gateway/task_planner.py:396-404` (few-shot examples block)

**Interfaces:**
- Produces: a FactoryTask example for the exact commissioning command so the LLM emits `observe_station → pick_object → verify_grasp → place_object{destination: fixture}`.

- [ ] **Step 1: Add the few-shot example**

After the existing `"go to red box"` example (~line 403), add:

```python
User: "gắp yellow box thả ở gá phôi"  (grasp yellow box, place on fixture)
→ {"task_type": "factory_task", "version": "1.0", "task_id": "pick-yellow-place-fixture", "replan_policy": {"max_replans": 1, "on_world_change": "replan_before_motion"}, "root": {"type": "sequence", "children": [{"type": "observe", "name": "observe_station", "args": {"region": "station"}}, {"type": "skill", "name": "pick_object", "args": {"object": "yellow_box"}}, {"type": "skill", "name": "verify_grasp", "args": {}}, {"type": "skill", "name": "place_object", "args": {"object": "yellow_box", "destination": "fixture"}}]}}
```

- [ ] **Step 2: Sanity-check the planner offline**

Run: `cd /home/admin4/gp4_ws && GP4_LLM_ENV_FILE=$PWD/.env python test_planner.py`
Expected: prints a FactoryTask JSON whose `root.children` contains `pick_object`, `verify_grasp`, and `place_object` with `destination: fixture`. (This requires a live LLM endpoint per `.env`; if the endpoint is offline, skip and rely on Task 2/3 unit tests.)

- [ ] **Step 3: Commit**

```bash
git add src/llm_gateway/llm_gateway/task_planner.py
git commit -m "feat(task_planner): add pick→verify→place-to-fixture few-shot example"
```

---

### Task 6: Hardware commissioning checklist (manual, no simulation)

**Files:**
- Create: `docs/perception/grasp-commissioning-checklist.md`

This task produces no code — it is the on-hardware bring-up sequence. Each item is a gate; do not proceed past a failing gate.

- [ ] **Step 1: Write the checklist file**

```markdown
# Grasp Pipeline Commissioning Checklist (real hardware)

Run order. Hand on e-stop for every motion step. velocity_scale = 0.06.

## A. Bench the gripper IO alone (NO motion)
1. Launch hw stack: `ros2 launch gp4_bringup hw.launch.py robot_ip:=192.168.1.33 agent_ip:=192.168.1.99`
2. Close (grasp): `ros2 service call /yaskawa/write_single_io motoros2_interfaces/srv/WriteSingleIO "{address: 10017, value: 1}"`
   → CONFIRM the air/vacuum actuates and physically grips a yellow_box held by hand.
3. Read feedback: `ros2 service call /yaskawa/read_single_io motoros2_interfaces/srv/ReadSingleIO "{address: 30017}"`
   → CONFIRM value == 1 while grasped, 0 when released.
4. Open: `WriteSingleIO {address: 10017, value: 0}` → CONFIRM release.
   GATE: if address/value/feedback are wrong, fix `safety_rules.yaml gripper:` before any motion.

## B. Perception quality at the grasp pose
5. Place a real yellow_box on the conveyor pick area.
6. `python test_svc.py` (class_filter yellow_box) → CONFIRM one detection, score, and a sane base_link XYZ.
7. Tune `perception.yaml min_publish_confidence` UP from the debug value 0.20 until only the true box publishes
   (start 0.45). Confirm the published Z matches the real box top within a few mm.
   GATE: do not grasp on a DEGRADED_DEPTH / low-confidence detection. Z error here = TCP crash.

## C. Motion dry-run, gripper disconnected (air OFF at the valve)
8. Send "đi tới yellow box" via HMI/gp4_cmd. CONFIRM approach pose is ABOVE the box (z + 0.08), tool-down.
9. Tune `approach_clearance_m` / `descend_m` (skill args) so descend stops at the real grasp surface, not into the table.
   GATE: jog limits and descend depth verified before reconnecting air.

## D. Full closed loop
10. Reconnect air. Command "gắp yellow box thả ở gá phôi".
11. Watch the supervisor confirm gate; approve. CONFIRM sequence:
    approach → descend → close(10017=1) → verify_grasp(30017==1) → lift →
    approach fixture → descend → open(10017=0) → lift.
12. CONFIRM box ends on the fixture. If verify_grasp fails, runtime stops before moving — re-tune B/C.
```

- [ ] **Step 2: Commit**

```bash
git add docs/perception/grasp-commissioning-checklist.md
git commit -m "docs: add grasp pipeline hardware commissioning checklist"
```

---

### Task 7: Remove root debug scripts (clean tree before commissioning)

**Files:**
- Delete: `test_planner.py`, `test_svc.py` (repo root) — **only after Task 5 Step 2 and the checklist are done with them.**

ponytail: these are throwaway bench scripts with hardcoded `/home/admin4` paths. Keep them out of the committed tree. If you still want them, move under `tools/` with path args instead of constants — but YAGNI: the commissioning checklist already covers their job.

- [ ] **Step 1: Confirm they are not imported anywhere**

Run: `cd /home/admin4/gp4_ws && grep -rn "test_planner\|test_svc" --include=*.py src/ hmi/ tools/`
Expected: no hits (they are standalone).

- [ ] **Step 2: Delete and commit**

```bash
git rm test_planner.py test_svc.py
git commit -m "chore: remove root bench scripts with hardcoded paths"
```

---

## Out of scope (deliberately not in this plan)

- ponytail: No new descend-by-bbox-height logic. Descend depth is a tuned arg (`descend_m`), because real depth/box-height varies — that's a calibration knob (Task 6 C), not code.
- ponytail: No retreat/retry/fallback tree for failed grasp. `verify_grasp` already halts the runtime sequence on failure; add retry only if commissioning shows it's needed.
- The untracked `src/motion_core/src/planning/ik_solver_checker.cpp` is not wired into CMake (dead) — leave it or delete separately; unrelated to grasp.
- Perception threshold restoration is a tuning step (Task 6 B), not a code revert, since the operator may need the relaxed values to see detections at all.

## Self-review notes

- Spec coverage: Gap 1 (pick sequence) → Task 2; Gap 2 (place destination) → Tasks 2-3; gripper-IO address bug → Tasks 1,2,4; LLM path → Task 5; hardware bring-up → Task 6; cleanup → Task 7. ✓
- Type consistency: `GripperCompileConfig(close, open)` defined Task 1, consumed identically in Tasks 2 and 4. `self._gripper` used only after the `if self._gripper is None` guard. ✓
- No placeholders: all code blocks are concrete; no TODO/TBD. ✓
