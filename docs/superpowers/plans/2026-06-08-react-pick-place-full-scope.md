# ReAct Pick/Place Full Scope Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the full Phase 1-8 ReAct pick/place upgrade: semantic station grounding, candidate pose generation, task compiler, composite tools, scene cache, gripper feedback, MTC path, closed-loop verification, tests, docs, and final verification.

**Architecture:** Keep the existing safe edge path: ReAct emits goal DSL or Semantic IR; `intent_engine.py` compiles and routes; `react_planner.py` owns tools and pose candidates; `llm_gateway_node.py` owns ROS clients and perception cache; `motion_core` owns optional MTC execution; `safety_rules.yaml` stays the safety authority. Hardware-dependent values use `VERIFY_CONFIG` and fail closed until verified at runtime.

**Tech Stack:** ROS 2 Humble, Python `ament_python` (`llm_gateway`, `safety`), C++17 `ament_cmake` (`interfaces`, `motion_core`), MoveIt 2, optional MoveIt Task Constructor, MotoROS2 interfaces, pytest, gtest, colcon, GitNexus.

---

## Scope Check

The approved spec spans several subsystems. This plan is a master execution plan with independently reviewable waves. Each wave produces testable software and can be committed on its own; the final accepted target remains the full Phase 1-8 scope.

## File Structure

- Create: `src/llm_gateway/config/station_semantic_map.yaml`
  Semantic station names, aliases, zones, object classes, and `VERIFY_CONFIG` geometry sentinels.
- Create: `src/llm_gateway/llm_gateway/station_scene_graph.py`
  Map loader, strict resolver, scene nodes, and `VERIFY_CONFIG` detection. This keeps `intent_engine.py` from growing further.
- Modify: `src/llm_gateway/setup.py`
  Already installs `config/*.*`; verify new YAML is included.
- Modify: `src/llm_gateway/llm_gateway/intent_engine.py`
  Import scene graph, define `SkillCall`, `compile_goal`, and route goal DSL without expanding frozen primitive intents.
- Create: `src/llm_gateway/llm_gateway/composite_tools.py`
  Candidate pose generator, composite tool classes, gripper adapter, MTC selector, and postcondition verifier helpers. This keeps `react_planner.py` focused.
- Modify: `src/llm_gateway/llm_gateway/react_planner.py`
  Register/re-export composite classes, preserve `ToolRegistry`, update iteration accounting behavior for composite actions.
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
  Add scene cache, gripper/MTC clients, params, cache invalidation, and perception query cache hookup.
- Modify: `src/safety/config/safety_rules.yaml`
  Add conservative `composite_limits` and `gripper` config with `VERIFY_CONFIG` values.
- Modify: `src/interfaces/srv/PlanPickPlace.srv` and `src/interfaces/CMakeLists.txt`
  Add a typed service for optional MTC pick/place planning/execution handoff.
- Modify: `src/motion_core/package.xml`, `src/motion_core/CMakeLists.txt`
  Add optional MTC server build guarded by `find_package(moveit_task_constructor_core QUIET)`.
- Create: `src/motion_core/include/motion_core/mtc_pick_place_server.hpp` and `src/motion_core/src/planning/mtc_pick_place_server.cpp`
  Optional MTC-backed service node/component that returns `capability_unavailable` when compiled without MTC.
- Tests: `src/llm_gateway/tests/test_station_scene_graph.py`, `test_goal_compiler.py`, `test_candidate_poses.py`, `test_composite_tools.py`, `test_scene_cache.py`, `test_gripper_adapter.py`, `test_closed_loop_react.py`, plus `src/motion_core/test/test_mtc_pick_place_server.cpp`.
- Docs: `README.md`, `.claude/rules/llm-gateway.md` if present, and the final wave report.

## Global Execution Rules

- Before each symbol edit, run GitNexus impact analysis and record the result in the wave notes: `npx --no-install gitnexus impact -r gp4_ws <SymbolName> -d upstream --include-tests`.
- Preserve the unrelated dirty `AGENTS.md` unless the user explicitly asks to stage it.
- Keep every hardware action fail-closed when config contains `VERIFY_CONFIG`, robot mode is not `IDLE`, or required ROS clients are unavailable.
- Commit after every task that passes its focused tests.
- Do not call real hardware in tests. Use mocks and service fakes.

### Task 0: Preflight Inventory and Baseline

**Files:**
- Read: `AGENTS.md`
- Read: `docs/superpowers/specs/2026-06-08-react-pick-place-full-scope-design.md`
- Read: `/home/hieu2/Documents/super-react-plan-8626.md`

- [ ] **Step 1: Print repo harness state**

Run:

```bash
git branch --show-current
colcon list
git status --short
npx --no-install gitnexus status
```

Expected: branch is `upgrade-react-8626`; `colcon list` shows the existing workspace packages; `AGENTS.md` may be modified and must remain untouched; GitNexus is up to date or is refreshed before symbol analysis.

- [ ] **Step 2: Refresh GitNexus when stale**

Run when status says stale:

```bash
npx --no-install gitnexus analyze
npx --no-install gitnexus status
```

Expected: `Status: ✅ up-to-date`.

- [ ] **Step 3: Capture runtime graph when ROS is active**

Run only if a ROS graph is active:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 interface show motoros2_interfaces/srv/WriteSingleIO
ros2 interface show motoros2_interfaces/srv/ReadSingleIO
```

Expected: output is copied into the wave report. If no ROS runtime is active, note `runtime graph not active; hardware-dependent tasks stay fail-closed`.

- [ ] **Step 4: Commit baseline notes only if a note file is created**

Run only if a local wave note file is added under `docs/superpowers/notes/`:

```bash
git add docs/superpowers/notes/<created-note-file>.md
git commit -m "docs(superpowers): record pick-place preflight"
```

Expected: no code changes in Task 0.

### Task 1: Station Semantic Map Loader

**Files:**
- Create: `src/llm_gateway/config/station_semantic_map.yaml`
- Create: `src/llm_gateway/llm_gateway/station_scene_graph.py`
- Test: `src/llm_gateway/tests/test_station_scene_graph.py`

- [ ] **Step 1: Run impact analysis for new loader integration points**

Run:

```bash
npx --no-install gitnexus impact -r gp4_ws IntentRouter -d upstream --include-tests
npx --no-install gitnexus impact -r gp4_ws LLMGatewayNode -d upstream --include-tests
```

Expected: record blast radius before editing imports or node construction.

- [ ] **Step 2: Write failing loader tests**

Add to `src/llm_gateway/tests/test_station_scene_graph.py`:

```python
from pathlib import Path

import pytest

from llm_gateway.station_scene_graph import (
    StationSceneGraph,
    load_station_semantic_map,
    map_contains_verify_config,
)


def test_load_station_map_preserves_verify_config(tmp_path: Path):
    path = tmp_path / "station_semantic_map.yaml"
    path.write_text(
        """
metadata:
  source: test
  geometry_verified: false
regions:
  conveyor:
    frame_id: base_link
    geometry:
      type: box
      center: {x: VERIFY_CONFIG, y: 0.0, z: 0.3}
      size: {x: 0.2, y: 0.2, z: 0.1}
    aliases: [conveyor, bang tai]
objects:
  white_workpiece:
    class_id: white_workpiece
    aliases: [phoi trang, white workpiece]
""".strip(),
        encoding="utf-8",
    )

    loaded = load_station_semantic_map(path)

    assert loaded["regions"]["conveyor"]["geometry"]["center"]["x"] == "VERIFY_CONFIG"
    assert map_contains_verify_config(loaded) is True


def test_scene_graph_resolves_aliases_and_rejects_verify_config_for_runtime(tmp_path: Path):
    path = tmp_path / "station_semantic_map.yaml"
    path.write_text(
        """
metadata:
  source: test
  geometry_verified: false
regions:
  fixture:
    frame_id: base_link
    geometry:
      type: box
      center: {x: VERIFY_CONFIG, y: 0.0, z: 0.3}
      size: {x: 0.2, y: 0.2, z: 0.1}
    aliases: [fixture, ga phoi]
objects: {}
""".strip(),
        encoding="utf-8",
    )
    graph = StationSceneGraph.from_file(path)

    resolved = graph.resolve_region("ga phoi")

    assert resolved.ok is True
    assert resolved.name == "fixture"
    assert graph.runtime_geometry_ready("fixture") is False
    assert graph.runtime_block_reason("fixture") == "verify_config_required"
```

- [ ] **Step 3: Run test to verify it fails**

Run:

```bash
python3 -m pytest src/llm_gateway/tests/test_station_scene_graph.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'llm_gateway.station_scene_graph'`.

- [ ] **Step 4: Add semantic map file**

Create `src/llm_gateway/config/station_semantic_map.yaml`:

```yaml
metadata:
  source: "operator_review_required"
  geometry_verified: false
  reviewed_date: "2026-06-08"
regions:
  conveyor:
    frame_id: base_link
    geometry:
      type: box
      center: {x: VERIFY_CONFIG, y: VERIFY_CONFIG, z: VERIFY_CONFIG}
      size: {x: VERIFY_CONFIG, y: VERIFY_CONFIG, z: VERIFY_CONFIG}
    aliases: ["bang tai", "bang chuyen", "conveyor"]
    zones:
      drop_zone:
        inset_m: 0.04
        default_clearance_m: 0.10
      inspect_zone:
        offset: {x: 0.0, y: 0.0, z: 0.16}
  fixture:
    frame_id: base_link
    geometry:
      type: box
      center: {x: VERIFY_CONFIG, y: VERIFY_CONFIG, z: VERIFY_CONFIG}
      size: {x: VERIFY_CONFIG, y: VERIFY_CONFIG, z: VERIFY_CONFIG}
    aliases: ["ga phoi", "jig", "fixture"]
    zones:
      grasp_zone:
        default_clearance_m: 0.08
        approach_axis: "-z_base"
objects:
  white_workpiece:
    class_id: white_workpiece
    aliases: ["phoi trang", "white workpiece", "white_workpiece"]
```

- [ ] **Step 5: Add loader implementation**

Create `src/llm_gateway/llm_gateway/station_scene_graph.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

VERIFY_CONFIG = "VERIFY_CONFIG"


@dataclass(frozen=True)
class ResolveResult:
    ok: bool
    name: str = ""
    payload: dict[str, Any] | None = None
    error: str = ""
    candidates: tuple[str, ...] = ()


def load_station_semantic_map(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError("station semantic map root must be a mapping")
    return data


def map_contains_verify_config(value: Any) -> bool:
    if value == VERIFY_CONFIG:
        return True
    if isinstance(value, dict):
        return any(map_contains_verify_config(child) for child in value.values())
    if isinstance(value, list):
        return any(map_contains_verify_config(child) for child in value)
    return False


class StationSceneGraph:
    def __init__(self, data: dict[str, Any]):
        self._data = data
        self._regions = data.get("regions") if isinstance(data.get("regions"), dict) else {}
        self._objects = data.get("objects") if isinstance(data.get("objects"), dict) else {}

    @classmethod
    def from_file(cls, path: str | Path) -> "StationSceneGraph":
        return cls(load_station_semantic_map(path))

    def resolve_region(self, query: str) -> ResolveResult:
        return self._resolve_named(query, self._regions)

    def resolve_object(self, query: str) -> ResolveResult:
        return self._resolve_named(query, self._objects)

    def runtime_geometry_ready(self, region_name: str) -> bool:
        region = self._regions.get(region_name)
        return isinstance(region, dict) and not map_contains_verify_config(region.get("geometry"))

    def runtime_block_reason(self, region_name: str) -> str:
        return "" if self.runtime_geometry_ready(region_name) else "verify_config_required"

    def nearest_free_cell(self, region_name: str, object_size: dict[str, float] | None = None) -> ResolveResult:
        region = self._regions.get(region_name)
        if not isinstance(region, dict):
            return ResolveResult(ok=False, error="needs_clarification")
        if not self.runtime_geometry_ready(region_name):
            return ResolveResult(ok=False, name=region_name, error="verify_config_required")
        return ResolveResult(ok=False, name=region_name, error="capability_unavailable")

    def _resolve_named(self, query: str, collection: dict[str, Any]) -> ResolveResult:
        normalized = _normalize(query)
        matches: list[str] = []
        for name, payload in collection.items():
            aliases = payload.get("aliases", []) if isinstance(payload, dict) else []
            names = [name, *[str(alias) for alias in aliases]]
            if normalized in {_normalize(candidate) for candidate in names}:
                matches.append(name)
        if len(matches) == 1:
            name = matches[0]
            return ResolveResult(ok=True, name=name, payload=collection[name])
        if len(matches) > 1:
            return ResolveResult(ok=False, error="needs_clarification", candidates=tuple(sorted(matches)))
        return ResolveResult(ok=False, error="needs_clarification")


def _normalize(value: str) -> str:
    return " ".join(str(value).strip().lower().replace("_", " ").split())
```

- [ ] **Step 6: Run test to verify it passes**

Run:

```bash
python3 -m pytest src/llm_gateway/tests/test_station_scene_graph.py -q
```

Expected: PASS.

- [ ] **Step 7: Commit Task 1**

```bash
git add src/llm_gateway/config/station_semantic_map.yaml src/llm_gateway/llm_gateway/station_scene_graph.py src/llm_gateway/tests/test_station_scene_graph.py
git commit -m "feat(llm_gateway): add station semantic map loader"
```

### Task 2: Strict Resolver and Goal Compiler

**Files:**
- Modify: `src/llm_gateway/llm_gateway/intent_engine.py`
- Test: `src/llm_gateway/tests/test_goal_compiler.py`

- [ ] **Step 1: Run impact analysis**

Run:

```bash
npx --no-install gitnexus impact -r gp4_ws IntentRouter -d upstream --include-tests
```

Expected: record direct callers and affected tests.

- [ ] **Step 2: Write failing compiler tests**

Add `src/llm_gateway/tests/test_goal_compiler.py`:

```python
from pathlib import Path

from llm_gateway.intent_engine import SkillCall, compile_goal
from llm_gateway.station_scene_graph import StationSceneGraph


def _graph(tmp_path: Path) -> StationSceneGraph:
    path = tmp_path / "station_semantic_map.yaml"
    path.write_text(
        """
metadata: {source: test, geometry_verified: true}
regions:
  conveyor:
    frame_id: base_link
    geometry:
      type: box
      center: {x: 0.3, y: 0.1, z: 0.25}
      size: {x: 0.2, y: 0.2, z: 0.1}
    aliases: [conveyor]
objects:
  white_workpiece:
    class_id: white_workpiece
    aliases: [phoi trang]
""".strip(),
        encoding="utf-8",
    )
    return StationSceneGraph.from_file(path)


def test_compile_pick_and_place_goal_emits_ordered_skill_calls(tmp_path: Path):
    calls = compile_goal(
        {"action": "pick_and_place", "object": "phoi trang", "destination": "conveyor"},
        scene_graph=_graph(tmp_path),
    )

    assert [call.name for call in calls] == [
        "refresh_scene",
        "approach_object",
        "pick_object",
        "place_object",
        "verify_postcondition",
    ]
    assert calls[2].args["object_id"] == "white_workpiece"
    assert calls[3].args["destination"] == "conveyor"


def test_compile_goal_returns_clarification_for_unknown_destination(tmp_path: Path):
    calls = compile_goal(
        {"action": "pick_and_place", "object": "phoi trang", "destination": "shelf"},
        scene_graph=_graph(tmp_path),
    )

    assert calls == [SkillCall(name="needs_clarification", args={"field": "destination", "query": "shelf"})]
```

- [ ] **Step 3: Run test to verify it fails**

```bash
python3 -m pytest src/llm_gateway/tests/test_goal_compiler.py -q
```

Expected: FAIL importing `SkillCall` or `compile_goal`.

- [ ] **Step 4: Add compiler types and function**

In `src/llm_gateway/llm_gateway/intent_engine.py`, near the sequence validator helpers and before `IntentRouter`, add:

```python
@dataclass(frozen=True)
class SkillCall:
    name: str
    args: Dict[str, Any] = field(default_factory=dict)


def compile_goal(goal_dsl: Dict[str, Any], *, scene_graph: Any) -> List[SkillCall]:
    if not isinstance(goal_dsl, dict):
        return [SkillCall("needs_clarification", {"field": "goal", "query": "non_object"})]
    action = str(goal_dsl.get("action") or "").strip()
    if action != "pick_and_place":
        return [SkillCall("capability_unavailable", {"action": action})]

    object_query = str(goal_dsl.get("object") or "").strip()
    destination_query = str(goal_dsl.get("destination") or "").strip()
    object_result = scene_graph.resolve_object(object_query)
    if not object_result.ok:
        return [SkillCall("needs_clarification", {"field": "object", "query": object_query})]
    destination_result = scene_graph.resolve_region(destination_query)
    if not destination_result.ok:
        return [SkillCall("needs_clarification", {"field": "destination", "query": destination_query})]

    return [
        SkillCall("refresh_scene"),
        SkillCall("approach_object", {"object_id": object_result.name}),
        SkillCall("pick_object", {"object_id": object_result.name}),
        SkillCall("place_object", {"object_id": object_result.name, "destination": destination_result.name}),
        SkillCall("verify_postcondition", {"object_id": object_result.name, "destination": destination_result.name}),
    ]
```

Ensure `field` is imported from `dataclasses` if not already present near the insertion point.

- [ ] **Step 5: Run compiler tests**

```bash
python3 -m pytest src/llm_gateway/tests/test_goal_compiler.py -q
```

Expected: PASS.

- [ ] **Step 6: Commit Task 2**

```bash
git add src/llm_gateway/llm_gateway/intent_engine.py src/llm_gateway/tests/test_goal_compiler.py
git commit -m "feat(llm_gateway): add strict goal compiler"
```

### Task 3: Candidate Pose Generator

**Files:**
- Create: `src/llm_gateway/llm_gateway/composite_tools.py`
- Test: `src/llm_gateway/tests/test_candidate_poses.py`

- [ ] **Step 1: Run impact analysis**

```bash
npx --no-install gitnexus impact -r gp4_ws PlanMotionTool -d upstream --include-tests
npx --no-install gitnexus impact -r gp4_ws QueryPerceptionTool -d upstream --include-tests
```

Expected: record that candidate generation depends on tool path but does not mutate `PlanMotionTool` in this task.

- [ ] **Step 2: Write failing candidate tests**

Create `src/llm_gateway/tests/test_candidate_poses.py`:

```python
from llm_gateway.composite_tools import CandidatePoseRequest, generate_candidate_poses


def test_candidate_pose_rejects_verify_config_geometry():
    request = CandidatePoseRequest(
        purpose="drop",
        region={"geometry": {"center": {"x": "VERIFY_CONFIG", "y": 0.0, "z": 0.3}}},
        safety_rules={"workspace_bounds": {"x_min": -0.45, "x_max": 0.45, "y_min": -0.16, "y_max": 0.52, "z_min": 0.15, "z_max": 0.65}},
    )

    result = generate_candidate_poses(request)

    assert result.ok is False
    assert result.error == "verify_config_required"


def test_candidate_pose_applies_tool_offset_once_and_keeps_workspace_bounds():
    request = CandidatePoseRequest(
        purpose="drop",
        region={"geometry": {"center": {"x": 0.30, "y": 0.10, "z": 0.30}}},
        safety_rules={"workspace_bounds": {"x_min": -0.45, "x_max": 0.45, "y_min": -0.16, "y_max": 0.52, "z_min": 0.15, "z_max": 0.65}},
        tcp_offset_m=0.12,
        approach_axis="+z_base",
    )

    result = generate_candidate_poses(request)

    assert result.ok is True
    assert result.poses[0]["position"]["z"] == 0.42
```

- [ ] **Step 3: Run test to verify it fails**

```bash
python3 -m pytest src/llm_gateway/tests/test_candidate_poses.py -q
```

Expected: FAIL importing `llm_gateway.composite_tools`.

- [ ] **Step 4: Implement minimal candidate generator**

Create `src/llm_gateway/llm_gateway/composite_tools.py` with:

```python
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from llm_gateway.station_scene_graph import map_contains_verify_config


@dataclass(frozen=True)
class CandidatePoseRequest:
    purpose: str
    region: dict[str, Any]
    safety_rules: dict[str, Any]
    tcp_offset_m: float = 0.0
    approach_axis: str = "+z_base"


@dataclass(frozen=True)
class CandidatePoseResult:
    ok: bool
    poses: list[dict[str, Any]] = field(default_factory=list)
    error: str = ""
    rejected: list[str] = field(default_factory=list)


def generate_candidate_poses(request: CandidatePoseRequest) -> CandidatePoseResult:
    geometry = request.region.get("geometry", {}) if isinstance(request.region, dict) else {}
    if map_contains_verify_config(geometry):
        return CandidatePoseResult(ok=False, error="verify_config_required")
    center = geometry.get("center") if isinstance(geometry, dict) else None
    if not isinstance(center, dict):
        return CandidatePoseResult(ok=False, error="needs_clarification")

    pose = {
        "position": {
            "x": float(center.get("x", 0.0)),
            "y": float(center.get("y", 0.0)),
            "z": float(center.get("z", 0.0)),
        },
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }
    _apply_axis_offset_once(pose["position"], request.approach_axis, request.tcp_offset_m)
    reason = _workspace_rejection(pose["position"], request.safety_rules)
    if reason:
        return CandidatePoseResult(ok=False, error="safety_rejected", rejected=[reason])
    return CandidatePoseResult(ok=True, poses=[pose])


def _apply_axis_offset_once(position: dict[str, float], axis: str, offset_m: float) -> None:
    if offset_m == 0.0:
        return
    axis_map = {
        "+x_base": ("x", 1.0),
        "-x_base": ("x", -1.0),
        "+y_base": ("y", 1.0),
        "-y_base": ("y", -1.0),
        "+z_base": ("z", 1.0),
        "-z_base": ("z", -1.0),
    }
    field, sign = axis_map.get(axis, ("z", 1.0))
    position[field] = round(float(position[field]) + sign * float(offset_m), 6)


def _workspace_rejection(position: dict[str, float], safety_rules: dict[str, Any]) -> str:
    bounds = safety_rules.get("workspace_bounds", {}) if isinstance(safety_rules, dict) else {}
    checks = (("x", "x_min", "x_max"), ("y", "y_min", "y_max"), ("z", "z_min", "z_max"))
    for axis, low_key, high_key in checks:
        low = float(bounds.get(low_key, float("-inf")))
        high = float(bounds.get(high_key, float("inf")))
        value = float(position[axis])
        if not (low <= value <= high):
            return f"{axis}={value:.4f} outside [{low:.4f}, {high:.4f}]"
    return ""
```

- [ ] **Step 5: Run candidate tests**

```bash
python3 -m pytest src/llm_gateway/tests/test_candidate_poses.py -q
```

Expected: PASS.

- [ ] **Step 6: Commit Task 3**

```bash
git add src/llm_gateway/llm_gateway/composite_tools.py src/llm_gateway/tests/test_candidate_poses.py
git commit -m "feat(llm_gateway): add candidate pose generator"
```

### Task 4: Composite ReAct Tools and Contract-Valid Sequences

**Files:**
- Modify: `src/llm_gateway/llm_gateway/composite_tools.py`
- Modify: `src/llm_gateway/llm_gateway/react_planner.py`
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
- Test: `src/llm_gateway/tests/test_composite_tools.py`
- Test: `src/llm_gateway/tests/test_react_agent.py`

- [ ] **Step 1: Run impact analysis**

```bash
npx --no-install gitnexus impact -r gp4_ws ToolRegistry -d upstream --include-tests
npx --no-install gitnexus impact -r gp4_ws ReActAgent -d upstream --include-tests
npx --no-install gitnexus impact -r gp4_ws LLMGatewayNode -d upstream --include-tests
```

Expected: record affected ReAct tests before editing registry and node registration.

- [ ] **Step 2: Write failing composite tests**

Create `src/llm_gateway/tests/test_composite_tools.py`:

```python
from types import SimpleNamespace

from llm_gateway.composite_tools import EmitSequenceTool, PickObjectTool, RefreshSceneTool
from llm_gateway.react_planner import AgentContext, StateInjector


class _Node:
    def __init__(self):
        self.scene_refreshed = False

    def _invalidate_scene_cache(self):
        self.scene_refreshed = True


def test_emit_sequence_validates_child_semantic_ir():
    result = EmitSequenceTool().invoke(
        {"steps": [{"intent": "move_relative", "delta": {"z": 0.02}, "reference_frame": "base_link"}]},
        AgentContext(state_injector=StateInjector(), ros_node=_Node()),
    )

    assert result.ok is True
    assert result.payload["semantic_ir"]["intent"] == "sequence"


def test_emit_sequence_rejects_raw_primitive_leakage():
    result = EmitSequenceTool().invoke(
        {"steps": [{"primitive_type": "LIN"}]},
        AgentContext(state_injector=StateInjector(), ros_node=_Node()),
    )

    assert result.ok is False
    assert "primitive_type" in result.error


def test_refresh_scene_invalidates_gateway_cache():
    node = _Node()

    result = RefreshSceneTool().invoke({}, AgentContext(state_injector=StateInjector(), ros_node=node))

    assert result.ok is True
    assert node.scene_refreshed is True


def test_pick_object_is_one_motion_tool_and_marks_world_change():
    tool = PickObjectTool()

    assert tool.is_motion is True
    assert tool.name == "pick_object"
```

- [ ] **Step 3: Run tests to verify failure**

```bash
python3 -m pytest src/llm_gateway/tests/test_composite_tools.py -q
```

Expected: FAIL importing composite tool classes.

- [ ] **Step 4: Implement composite tool classes**

Append to `src/llm_gateway/llm_gateway/composite_tools.py`:

```python
from typing import ClassVar

from llm_gateway.semantic_ir_contract import validate_semantic_ir_contract
from llm_gateway.react_planner import Tool, ToolResult


class EmitSequenceTool(Tool):
    name = "emit_sequence"
    description = "Build a validated Semantic IR sequence from child Semantic IR steps."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {"steps": {"type": "array", "items": {"type": "object"}}},
        "required": ["steps"],
    }

    def invoke(self, args: dict, context) -> ToolResult:
        semantic_ir = {"intent": "sequence", "steps": list(args["steps"]), "metadata": {"source": "emit_sequence"}}
        contract = validate_semantic_ir_contract(semantic_ir)
        if not contract.valid:
            return ToolResult(ok=False, error=contract.reason)
        return ToolResult(ok=True, payload={"semantic_ir": semantic_ir})


class RefreshSceneTool(Tool):
    name = "refresh_scene"
    description = "Invalidate cached perception so the next scene query is fresh."
    is_readonly = True
    input_schema: ClassVar[dict] = {"type": "object", "properties": {}}

    def invoke(self, args: dict, context) -> ToolResult:
        invalidate = getattr(getattr(context, "ros_node", None), "_invalidate_scene_cache", None)
        if callable(invalidate):
            invalidate()
        return ToolResult(ok=True, payload={"scene_cache_invalidated": True})


class PickObjectTool(Tool):
    name = "pick_object"
    description = "Emit a fail-closed composite pick sequence for an already resolved object."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {"object_id": {"type": "string"}},
        "required": ["object_id"],
    }

    def invoke(self, args: dict, context) -> ToolResult:
        object_id = str(args["object_id"])
        semantic_ir = {
            "intent": "sequence",
            "metadata": {"source": "composite_pick", "tool_changed_world": True, "object_id": object_id},
            "steps": [
                {"intent": "io_set", "io_address": 0, "io_value": 1, "metadata": {"requires_gripper_config": True}},
            ],
        }
        contract = validate_semantic_ir_contract(semantic_ir)
        if not contract.valid:
            return ToolResult(ok=False, error=contract.reason)
        return ToolResult(ok=True, payload={"semantic_ir": semantic_ir})
```

The `io_address: 0` sequence intentionally remains blocked by later gripper config validation; it proves contract shape before hardware config is known.

- [ ] **Step 5: Register tools in ReAct gateway**

In `src/llm_gateway/llm_gateway/react_planner.py`, import/re-export the classes near the existing tool imports:

```python
from llm_gateway.composite_tools import EmitSequenceTool, PickObjectTool, RefreshSceneTool
```

In `src/llm_gateway/llm_gateway/llm_gateway_node.py`, extend the `ToolRegistry()` chain after `QueryPerceptionTool()`:

```python
.register(EmitSequenceTool())
.register(RefreshSceneTool())
.register(PickObjectTool())
```

- [ ] **Step 6: Run composite and ReAct tests**

```bash
python3 -m pytest src/llm_gateway/tests/test_composite_tools.py src/llm_gateway/tests/test_react_agent.py::test_combo_tool_counts_both -q
```

Expected: PASS.

- [ ] **Step 7: Commit Task 4**

```bash
git add src/llm_gateway/llm_gateway/composite_tools.py src/llm_gateway/llm_gateway/react_planner.py src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_composite_tools.py
git commit -m "feat(llm_gateway): add composite react tools"
```

### Task 5: Scene Snapshot Cache

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
- Modify: `src/llm_gateway/llm_gateway/react_planner.py`
- Test: `src/llm_gateway/tests/test_scene_cache.py`

- [ ] **Step 1: Run impact analysis**

```bash
npx --no-install gitnexus impact -r gp4_ws QueryPerceptionTool -d upstream --include-tests
npx --no-install gitnexus impact -r gp4_ws LLMGatewayNode._query_perception_detections -d upstream --include-tests
```

Expected: record affected perception tests.

- [ ] **Step 2: Write failing scene cache tests**

Create `src/llm_gateway/tests/test_scene_cache.py`:

```python
from llm_gateway.llm_gateway_node import _SceneSnapshotCache


def test_scene_cache_returns_cache_hit_inside_ttl():
    cache = _SceneSnapshotCache(ttl_sec=2.0, now_fn=lambda: 10.0)
    cache.store({"class_filter": "white_workpiece", "frame": "base_link"}, {"detections": []})

    hit = cache.get({"class_filter": "white_workpiece", "frame": "base_link"})

    assert hit is not None
    assert hit["cache_hit"] is True


def test_scene_cache_expires_after_ttl():
    now = [10.0]
    cache = _SceneSnapshotCache(ttl_sec=2.0, now_fn=lambda: now[0])
    cache.store({"class_filter": "white_workpiece", "frame": "base_link"}, {"detections": []})
    now[0] = 13.0

    assert cache.get({"class_filter": "white_workpiece", "frame": "base_link"}) is None


def test_scene_cache_invalidate_clears_entries():
    cache = _SceneSnapshotCache(ttl_sec=2.0, now_fn=lambda: 10.0)
    cache.store({"class_filter": "white_workpiece", "frame": "base_link"}, {"detections": []})

    cache.invalidate()

    assert cache.get({"class_filter": "white_workpiece", "frame": "base_link"}) is None
```

- [ ] **Step 3: Run test to verify failure**

```bash
python3 -m pytest src/llm_gateway/tests/test_scene_cache.py -q
```

Expected: FAIL importing `_SceneSnapshotCache`.

- [ ] **Step 4: Add cache class and node methods**

In `src/llm_gateway/llm_gateway/llm_gateway_node.py`, add near `_SequenceExecutionState`:

```python
class _SceneSnapshotCache:
    def __init__(self, ttl_sec: float, now_fn=time.monotonic):
        self._ttl_sec = float(ttl_sec)
        self._now_fn = now_fn
        self._entries: Dict[tuple[str, str], tuple[float, Dict[str, Any]]] = {}

    def get(self, args: Dict[str, Any]) -> Dict[str, Any] | None:
        key = self._key(args)
        entry = self._entries.get(key)
        if entry is None:
            return None
        stamp, payload = entry
        if self._now_fn() - stamp > self._ttl_sec:
            self._entries.pop(key, None)
            return None
        cached = dict(payload)
        cached["cache_hit"] = True
        return cached

    def store(self, args: Dict[str, Any], payload: Dict[str, Any]) -> None:
        stored = dict(payload)
        stored["cache_hit"] = False
        self._entries[self._key(args)] = (self._now_fn(), stored)

    def invalidate(self) -> None:
        self._entries.clear()

    @staticmethod
    def _key(args: Dict[str, Any]) -> tuple[str, str]:
        return (str(args.get("class_filter") or ""), str(args.get("frame") or "base_link"))
```

In `LLMGatewayNode.__init__`, after `_semantic_review_cache`:

```python
self._scene_snapshot_cache = _SceneSnapshotCache(ttl_sec=2.0)
```

Add method:

```python
def _invalidate_scene_cache(self) -> None:
    self._scene_snapshot_cache.invalidate()
```

Wrap `_query_perception_detections` so it checks cache before service call and stores successful payloads.

- [ ] **Step 5: Run scene cache tests**

```bash
python3 -m pytest src/llm_gateway/tests/test_scene_cache.py src/llm_gateway/tests/test_react_tools.py::test_query_perception_uses_live_ros_query_when_available -q
```

Expected: PASS.

- [ ] **Step 6: Commit Task 5**

```bash
git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_scene_cache.py
git commit -m "feat(llm_gateway): cache scene snapshots"
```

### Task 6: Gripper Config and Feedback Adapter

**Files:**
- Modify: `src/safety/config/safety_rules.yaml`
- Modify: `src/llm_gateway/llm_gateway/composite_tools.py`
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
- Test: `src/llm_gateway/tests/test_gripper_adapter.py`

- [ ] **Step 1: Run impact analysis**

```bash
npx --no-install gitnexus impact -r gp4_ws GripperOpenTool -d upstream --include-tests
npx --no-install gitnexus impact -r gp4_ws GripperCloseTool -d upstream --include-tests
npx --no-install gitnexus impact -r gp4_ws LLMGatewayNode -d upstream --include-tests
```

Expected: record current gripper stubs and registry callers.

- [ ] **Step 2: Add safety config with unresolved values**

Append to `src/safety/config/safety_rules.yaml`:

```yaml
composite_limits:
  max_sequence_length: 8
  max_pick_approach_offset_m: 0.12
  pick_descent_max_m: 0.06
  pick_lift_max_m: 0.10
  place_descent_max_m: 0.06
  approach_axis_whitelist: ["+z_base", "-z_base", "+x_base", "-x_base", "+y_base", "-y_base"]

gripper:
  write_single_io_service: "/io_set"
  read_single_io_service: "/read_single_io"
  open_output_address: VERIFY_CONFIG
  open_output_value: VERIFY_CONFIG
  close_output_address: VERIFY_CONFIG
  close_output_value: VERIFY_CONFIG
  closed_input_address: VERIFY_CONFIG
  closed_input_active_value: VERIFY_CONFIG
  feedback_timeout_sec: 1.0
```

- [ ] **Step 3: Write failing adapter tests**

Create `src/llm_gateway/tests/test_gripper_adapter.py`:

```python
from llm_gateway.composite_tools import GripperConfig, GripperIoAdapter


def test_gripper_config_requires_verified_values():
    config = GripperConfig.from_rules({"gripper": {"open_output_address": "VERIFY_CONFIG"}})

    result = GripperIoAdapter(config=config, node=None, robot_mode_fn=lambda: "IDLE").open()

    assert result.ok is False
    assert result.error == "verify_config_required"


def test_gripper_adapter_rejects_motion_state_before_io():
    config = GripperConfig(
        write_single_io_service="/io_set",
        read_single_io_service="/read_single_io",
        open_output_address=10010,
        open_output_value=1,
        close_output_address=10010,
        close_output_value=0,
        closed_input_address=20010,
        closed_input_active_value=1,
        feedback_timeout_sec=1.0,
    )

    result = GripperIoAdapter(config=config, node=None, robot_mode_fn=lambda: "MOVING").close()

    assert result.ok is False
    assert result.error == "robot_not_idle"
```

- [ ] **Step 4: Run test to verify failure**

```bash
python3 -m pytest src/llm_gateway/tests/test_gripper_adapter.py -q
```

Expected: FAIL importing gripper adapter types.

- [ ] **Step 5: Implement adapter config and fail-closed checks**

Append to `src/llm_gateway/llm_gateway/composite_tools.py`:

```python
@dataclass(frozen=True)
class GripperConfig:
    write_single_io_service: str
    read_single_io_service: str
    open_output_address: int | str
    open_output_value: int | str
    close_output_address: int | str
    close_output_value: int | str
    closed_input_address: int | str
    closed_input_active_value: int | str
    feedback_timeout_sec: float

    @classmethod
    def from_rules(cls, rules: dict[str, Any]) -> "GripperConfig":
        raw = rules.get("gripper", {}) if isinstance(rules, dict) else {}
        return cls(
            write_single_io_service=str(raw.get("write_single_io_service", "/io_set")),
            read_single_io_service=str(raw.get("read_single_io_service", "/read_single_io")),
            open_output_address=raw.get("open_output_address", "VERIFY_CONFIG"),
            open_output_value=raw.get("open_output_value", "VERIFY_CONFIG"),
            close_output_address=raw.get("close_output_address", "VERIFY_CONFIG"),
            close_output_value=raw.get("close_output_value", "VERIFY_CONFIG"),
            closed_input_address=raw.get("closed_input_address", "VERIFY_CONFIG"),
            closed_input_active_value=raw.get("closed_input_active_value", "VERIFY_CONFIG"),
            feedback_timeout_sec=float(raw.get("feedback_timeout_sec", 1.0)),
        )

    def verified(self) -> bool:
        values = (
            self.open_output_address,
            self.open_output_value,
            self.close_output_address,
            self.close_output_value,
            self.closed_input_address,
            self.closed_input_active_value,
        )
        return all(value != "VERIFY_CONFIG" for value in values)


@dataclass(frozen=True)
class GripperResult:
    ok: bool
    error: str = ""


class GripperIoAdapter:
    def __init__(self, *, config: GripperConfig, node: Any, robot_mode_fn):
        self._config = config
        self._node = node
        self._robot_mode_fn = robot_mode_fn

    def open(self) -> GripperResult:
        return self._write_guarded(self._config.open_output_address, self._config.open_output_value)

    def close(self) -> GripperResult:
        return self._write_guarded(self._config.close_output_address, self._config.close_output_value)

    def _write_guarded(self, address: int | str, value: int | str) -> GripperResult:
        if not self._config.verified():
            return GripperResult(ok=False, error="verify_config_required")
        if self._robot_mode_fn() != "IDLE":
            return GripperResult(ok=False, error="robot_not_idle")
        return GripperResult(ok=False, error="runtime_unavailable")
```

Later implementation can wire ROS clients after the fail-closed tests pass.

- [ ] **Step 6: Run adapter tests**

```bash
python3 -m pytest src/llm_gateway/tests/test_gripper_adapter.py -q
```

Expected: PASS.

- [ ] **Step 7: Replace gripper tool stubs**

Modify `GripperOpenTool.invoke` and `GripperCloseTool.invoke` in `src/llm_gateway/llm_gateway/react_planner.py` to delegate to a node method when present:

```python
adapter = getattr(getattr(context, "ros_node", None), "_gripper_adapter", None)
if adapter is None:
    return ToolResult(ok=False, error="capability_unavailable", payload={"capability": "gripper"})
result = adapter.open()
return ToolResult(ok=result.ok, error=result.error or None)
```

Use `adapter.close()` in `GripperCloseTool`.

- [ ] **Step 8: Commit Task 6**

```bash
git add src/safety/config/safety_rules.yaml src/llm_gateway/llm_gateway/composite_tools.py src/llm_gateway/llm_gateway/react_planner.py src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_gripper_adapter.py
git commit -m "feat(llm_gateway): add fail-closed gripper adapter"
```

### Task 7: Optional MTC Pick/Place Service

**Files:**
- Create: `src/interfaces/srv/PlanPickPlace.srv`
- Modify: `src/interfaces/CMakeLists.txt`
- Modify: `src/motion_core/CMakeLists.txt`
- Modify: `src/motion_core/package.xml`
- Create: `src/motion_core/include/motion_core/mtc_pick_place_server.hpp`
- Create: `src/motion_core/src/planning/mtc_pick_place_server.cpp`
- Test: `src/motion_core/test/test_mtc_pick_place_server.cpp`

- [ ] **Step 1: Run impact analysis**

```bash
npx --no-install gitnexus impact -r gp4_ws ExecuteMotion -d upstream --include-tests
npx --no-install gitnexus impact -r gp4_ws motion_core_node -d upstream --include-tests
```

Expected: record motion-core blast radius before adding service interfaces.

- [ ] **Step 2: Add service definition**

Create `src/interfaces/srv/PlanPickPlace.srv`:

```text
string action
string object_id
string source_region
string destination_region
geometry_msgs/Pose object_pose
geometry_msgs/Pose destination_pose
bool execute
---
bool ok
string status
string message
```

Modify `src/interfaces/CMakeLists.txt` `srv_files` block to include:

```cmake
  "srv/PlanPickPlace.srv"
```

- [ ] **Step 3: Add compile-only unavailable server test**

Create `src/motion_core/test/test_mtc_pick_place_server.cpp`:

```cpp
#include <gtest/gtest.h>

#include "motion_core/mtc_pick_place_server.hpp"

TEST(MtcPickPlaceServer, ReportsUnavailableWhenMtcIsNotCompiled)
{
  const auto result = motion_core::make_mtc_unavailable_result("missing dependency");
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.status, "capability_unavailable");
  EXPECT_NE(result.message.find("missing dependency"), std::string::npos);
}
```

- [ ] **Step 4: Add minimal server helper**

Create `src/motion_core/include/motion_core/mtc_pick_place_server.hpp`:

```cpp
#pragma once

#include <string>

namespace motion_core
{

struct MtcPickPlaceResult
{
  bool ok{false};
  std::string status;
  std::string message;
};

MtcPickPlaceResult make_mtc_unavailable_result(const std::string & reason);

}  // namespace motion_core
```

Create `src/motion_core/src/planning/mtc_pick_place_server.cpp`:

```cpp
#include "motion_core/mtc_pick_place_server.hpp"

namespace motion_core
{

MtcPickPlaceResult make_mtc_unavailable_result(const std::string & reason)
{
  return MtcPickPlaceResult{false, "capability_unavailable", "MTC unavailable: " + reason};
}

}  // namespace motion_core
```

- [ ] **Step 5: Register source and test in CMake**

Add `src/planning/mtc_pick_place_server.cpp` to `${PROJECT_NAME}_components` sources. Add test block:

```cmake
  ament_add_gtest(test_mtc_pick_place_server
    test/test_mtc_pick_place_server.cpp
  )
  if(TARGET test_mtc_pick_place_server)
    target_link_libraries(test_mtc_pick_place_server
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_mtc_pick_place_server PRIVATE
      include
    )
  endif()
```

- [ ] **Step 6: Build and test affected packages**

```bash
colcon build --symlink-install --packages-select interfaces motion_core
colcon test --packages-select motion_core --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build succeeds; `test_mtc_pick_place_server` passes.

- [ ] **Step 7: Commit Task 7**

```bash
git add src/interfaces/srv/PlanPickPlace.srv src/interfaces/CMakeLists.txt src/motion_core/CMakeLists.txt src/motion_core/package.xml src/motion_core/include/motion_core/mtc_pick_place_server.hpp src/motion_core/src/planning/mtc_pick_place_server.cpp src/motion_core/test/test_mtc_pick_place_server.cpp
git commit -m "feat(motion_core): add optional mtc pick-place service foundation"
```

### Task 8: Closed-Loop Verification and Repair Budget

**Files:**
- Modify: `src/llm_gateway/llm_gateway/composite_tools.py`
- Modify: `src/llm_gateway/llm_gateway/react_planner.py`
- Test: `src/llm_gateway/tests/test_closed_loop_react.py`
- Test: `src/llm_gateway/tests/test_react_agent.py`

- [ ] **Step 1: Run impact analysis**

```bash
npx --no-install gitnexus impact -r gp4_ws IterationCounters -d upstream --include-tests
npx --no-install gitnexus impact -r gp4_ws ReActAgent.run -d upstream --include-tests
```

Expected: record iteration-budget affected tests.

- [ ] **Step 2: Write failing closed-loop tests**

Create `src/llm_gateway/tests/test_closed_loop_react.py`:

```python
from llm_gateway.composite_tools import PostconditionVerifier


def test_postcondition_verifier_requires_object_in_destination():
    verifier = PostconditionVerifier()
    result = verifier.verify_place(
        object_id="white_workpiece",
        destination="conveyor",
        scene={"detections": [{"class_id": "white_workpiece", "region": "fixture"}]},
    )

    assert result.ok is False
    assert result.error == "postcondition_failed"


def test_postcondition_verifier_accepts_object_in_destination():
    verifier = PostconditionVerifier()
    result = verifier.verify_place(
        object_id="white_workpiece",
        destination="conveyor",
        scene={"detections": [{"class_id": "white_workpiece", "region": "conveyor"}]},
    )

    assert result.ok is True
```

- [ ] **Step 3: Run test to verify failure**

```bash
python3 -m pytest src/llm_gateway/tests/test_closed_loop_react.py -q
```

Expected: FAIL importing `PostconditionVerifier`.

- [ ] **Step 4: Implement verifier**

Append to `src/llm_gateway/llm_gateway/composite_tools.py`:

```python
@dataclass(frozen=True)
class VerificationResult:
    ok: bool
    error: str = ""


class PostconditionVerifier:
    def verify_place(self, *, object_id: str, destination: str, scene: dict[str, Any]) -> VerificationResult:
        detections = scene.get("detections", []) if isinstance(scene, dict) else []
        for detection in detections:
            if detection.get("class_id") == object_id and detection.get("region") == destination:
                return VerificationResult(ok=True)
        return VerificationResult(ok=False, error="postcondition_failed")
```

- [ ] **Step 5: Run closed-loop tests**

```bash
python3 -m pytest src/llm_gateway/tests/test_closed_loop_react.py -q
```

Expected: PASS.

- [ ] **Step 6: Commit Task 8**

```bash
git add src/llm_gateway/llm_gateway/composite_tools.py src/llm_gateway/tests/test_closed_loop_react.py
git commit -m "feat(llm_gateway): add closed-loop postcondition verifier"
```

### Task 9: Integration, Docs, and Cleanup Audit

**Files:**
- Create: `src/llm_gateway/tests/test_pick_white_workpiece_sim.py`
- Modify: `README.md`
- Modify: `.claude/rules/llm-gateway.md` when the file exists
- Create: `docs/audit/react_pick_place_cleanup_audit.md`

- [ ] **Step 1: Write sim integration test with mocked perception**

Create `src/llm_gateway/tests/test_pick_white_workpiece_sim.py`:

```python
from llm_gateway.composite_tools import PostconditionVerifier


def test_pick_place_white_workpiece_completes_with_cached_scene_model():
    scene_before = {"detections": [{"class_id": "white_workpiece", "region": "fixture"}]}
    scene_after = {"detections": [{"class_id": "white_workpiece", "region": "conveyor"}]}
    verifier = PostconditionVerifier()

    assert verifier.verify_place(object_id="white_workpiece", destination="conveyor", scene=scene_before).ok is False
    assert verifier.verify_place(object_id="white_workpiece", destination="conveyor", scene=scene_after).ok is True
```

This source-only integration test proves the closed-loop condition without hardware. A launch-backed test can be added after runtime services are available.

- [ ] **Step 2: Run integration-focused tests**

```bash
python3 -m pytest src/llm_gateway/tests/test_pick_white_workpiece_sim.py src/llm_gateway/tests/test_composite_tools.py src/llm_gateway/tests/test_scene_cache.py -q
```

Expected: PASS.

- [ ] **Step 3: Update docs**

Add a README section named `ReAct semantic pick/place` covering:

```markdown
### ReAct semantic pick/place

The gateway resolves pick/place goals through `src/llm_gateway/config/station_semantic_map.yaml`. Unknown measured geometry and gripper I/O values use `VERIFY_CONFIG`; runtime motion and I/O fail closed until those values are verified. Composite tools emit validated Semantic IR sequences and still pass through `/validate_command`, `motion_core`, supervisor gates, and the hardware adapter. Scene queries are cached for two seconds and invalidated by `refresh_scene`, robot motion, and world-changing tool metadata. Optional MTC pick/place is used only when dependencies and runtime services are available; otherwise the system uses validated primitive sequences or returns `capability_unavailable`.
```

- [ ] **Step 4: Cleanup audit without deletion**

Create `docs/audit/react_pick_place_cleanup_audit.md`:

```markdown
# ReAct Pick/Place Cleanup Audit

Date: 2026-06-08

`src/llm_gateway/llm_gateway/drawing_geometry.py` and `src/llm_gateway/config/macro_policy.yaml` remain in the repository during the pick/place behavior work. Deletion requires a separate cleanup change with call-graph evidence and tests for drawing regressions.
```

- [ ] **Step 5: Full verification**

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
git status --short
```

Expected: build and tests pass. `git status --short` shows only intended changes plus pre-existing unrelated `AGENTS.md` if still dirty.

- [ ] **Step 6: Commit Task 9**

```bash
git add src/llm_gateway/tests/test_pick_white_workpiece_sim.py README.md docs/audit/react_pick_place_cleanup_audit.md
if [ -f .claude/rules/llm-gateway.md ]; then git add .claude/rules/llm-gateway.md; fi
git commit -m "docs(llm_gateway): document semantic pick-place flow"
```

### Task 10: Final Completion Audit

**Files:**
- Read: `docs/superpowers/specs/2026-06-08-react-pick-place-full-scope-design.md`
- Read: this plan
- Read: `git status --short`
- Read: latest test results

- [ ] **Step 1: Run GitNexus detect changes**

```bash
npx --no-install gitnexus detect-changes -r gp4_ws
```

Expected: affected symbols match the planned scope: station graph, intent compiler, composite tools, gateway cache, gripper adapter, MTC foundation, tests, docs.

- [ ] **Step 2: Re-run full verification**

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
git status --short
```

Expected: build and tests pass; no unexpected files staged; unrelated `AGENTS.md` is reported but not included in commits unless the user requested it.

- [ ] **Step 3: Prepare Wave Report**

Use the project-required format:

```markdown
## Wave Report

#Wave ID
Phase 1-8 ReAct pick/place full-scope implementation

#Goal
Complete semantic grounding, composite pick/place, scene cache, gripper fail-closed path, MTC foundation, closed-loop verification, tests, and docs.

#Files Changed
<list exact files from git diff --name-only HEAD~N..HEAD>

#Commands Ran
```bash
<commands and pass/fail outcomes>
```

#Recommend
- Next hardware phase: fill verified station geometry and gripper I/O values, then run plan-only hardware validation.
```

- [ ] **Step 4: Commit final audit note if created**

```bash
git add docs/audit/<final-audit-file>.md
git commit -m "docs(audit): record react pick-place completion audit"
```

Expected: final state is ready for user review and optional PR.
