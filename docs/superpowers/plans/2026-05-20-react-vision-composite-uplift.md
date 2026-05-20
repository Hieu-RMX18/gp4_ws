# ReAct + Vision Composite Uplift Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add composite ReAct tools (pick/place/approach/retreat), a scene snapshot cache, and matching safety caps so the LLM can drive industrial pick-and-place within the existing fail-closed pipeline without bypassing safety or motion_core.

**Architecture:** Composite tools emit Semantic IR (`sequence` or single-intent) that still flows through `/validate_command → motion_core → hw_adapter`. A new `SceneSnapshotCache` deduplicates perception calls per ReAct cycle. Safety adds `composite_limits` enforcement on top of existing rules. No new ROS2 packages, no MoveIt2 changes.

**Tech Stack:** Python 3.10 (`llm_gateway`, `safety`, `gp4_perception`), ROS2 Humble, pytest, jsonschema, PyYAML, `colcon`.

**Spec reference:** `docs/superpowers/specs/2026-05-20-react-vision-composite-uplift-design.md`.

---

## File Structure

- **New** `src/llm_gateway/llm_gateway/scene_cache.py` — `SceneSnapshotCache` (TTL + manual + motion-mode + tool-changed-world invalidation).
- **New** `src/llm_gateway/llm_gateway/composite_tools.py` — composite ReAct tools (`ApproachObjectTool`, `RetreatTool`, `PickObjectTool`, `PlaceObjectTool`, `EmitSequenceTool`, `RefreshSceneTool`) and a small `_safety_caps_check` helper.
- **Modify** `src/llm_gateway/llm_gateway/react_planner.py` — only `QueryPerceptionTool.invoke` to consult the cache; no relocation of existing tool classes (out of scope).
- **Modify** `src/llm_gateway/llm_gateway/llm_gateway_node.py` — instantiate `SceneSnapshotCache`, gate registration of composites behind `react.composite_tools_enabled` param, register the new tools, plumb `tool_changed_world` invalidation through `submit_motion`.
- **Modify** `src/safety/config/safety_rules.yaml` — add `composite_limits` block.
- **New** `src/safety/safety/composite_limits.py` — pure helper that returns caps from a loaded safety rules dict.
- **Modify** `src/safety/safety/__init__.py` (only if needed to export the helper).
- **New** `src/llm_gateway/tests/test_scene_cache.py`.
- **New** `src/llm_gateway/tests/test_composite_tools.py`.
- **New** `src/safety/tests/test_composite_limits.py`.
- **New** `src/llm_gateway/tests/test_composite_ir_contract.py`.
- **New** `src/llm_gateway/tests/test_pick_place_sim.py` (integration, marked `@pytest.mark.integration`).
- **Modify** `README.md` (Composite ReAct flows subsection) and `.claude/rules/llm-gateway.md` (tool surface update).

Existing 800-line guideline note: `llm_gateway_node.py` and `react_planner.py` are already over 800 L. Tasks below add focused new modules rather than restructuring those files, per spec §5.5.

---

## Task 1: SceneSnapshotCache (pure utility, TDD)

**Files:**
- Create: `src/llm_gateway/llm_gateway/scene_cache.py`
- Test: `src/llm_gateway/tests/test_scene_cache.py`

- [ ] **Step 1: Write the failing test**

Create `src/llm_gateway/tests/test_scene_cache.py`:

```python
"""Unit tests for SceneSnapshotCache."""

from __future__ import annotations

import pytest

from llm_gateway.scene_cache import SceneSnapshotCache


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def cache():
    clock = FakeClock()
    return SceneSnapshotCache(ttl_seconds=2.0, clock=clock), clock


def test_miss_then_hit(cache):
    c, clock = cache
    assert c.get(class_filter="red_block", frame="base_link") is None
    c.put(class_filter="red_block", frame="base_link", payload={"count": 1})
    hit = c.get(class_filter="red_block", frame="base_link")
    assert hit == {"count": 1}


def test_ttl_expiry(cache):
    c, clock = cache
    c.put(class_filter="red_block", frame="base_link", payload={"count": 1})
    clock.t = 2.0001
    assert c.get(class_filter="red_block", frame="base_link") is None


def test_manual_invalidate(cache):
    c, clock = cache
    c.put(class_filter="red_block", frame="base_link", payload={"count": 1})
    c.invalidate(reason="refresh_scene")
    assert c.get(class_filter="red_block", frame="base_link") is None


def test_motion_mode_invalidate(cache):
    c, clock = cache
    c.put(class_filter="red_block", frame="base_link", payload={"count": 1})
    c.on_robot_mode("EXECUTING")
    assert c.get(class_filter="red_block", frame="base_link") is None


def test_tool_changed_world_invalidate(cache):
    c, clock = cache
    c.put(class_filter="red_block", frame="base_link", payload={"count": 1})
    c.on_motion_complete(tool_changed_world=True)
    assert c.get(class_filter="red_block", frame="base_link") is None


def test_motion_complete_without_world_change_keeps_cache(cache):
    c, clock = cache
    c.put(class_filter="red_block", frame="base_link", payload={"count": 1})
    c.on_motion_complete(tool_changed_world=False)
    assert c.get(class_filter="red_block", frame="base_link") == {"count": 1}


def test_distinct_keys_are_isolated(cache):
    c, _ = cache
    c.put(class_filter="red_block", frame="base_link", payload={"count": 1})
    c.put(class_filter="blue_cube", frame="base_link", payload={"count": 7})
    assert c.get(class_filter="red_block", frame="base_link") == {"count": 1}
    assert c.get(class_filter="blue_cube", frame="base_link") == {"count": 7}


def test_default_clock_uses_monotonic(monkeypatch):
    cache = SceneSnapshotCache(ttl_seconds=0.01)
    cache.put(class_filter="x", frame="base_link", payload={"v": 1})
    assert cache.get(class_filter="x", frame="base_link") == {"v": 1}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/llm_gateway && python -m pytest tests/test_scene_cache.py -v`
Expected: `ImportError` — `llm_gateway.scene_cache` not found.

- [ ] **Step 3: Write minimal implementation**

Create `src/llm_gateway/llm_gateway/scene_cache.py`:

```python
"""Scene snapshot cache for ReAct perception calls.

Caches `query_perception` results within a ReAct cycle to cut latency and
iteration count. Invalidated by TTL, manual refresh, motion-mode transition,
or motion completions that changed the world (pick/place).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Tuple


@dataclass
class _CacheEntry:
    payload: Any
    inserted_at: float


class SceneSnapshotCache:
    """TTL cache keyed by (class_filter, frame).

    Thread-safety is not provided; the gateway invokes ReAct serially.
    """

    def __init__(
        self,
        ttl_seconds: float = 2.0,
        clock: Optional[Callable[[], float]] = None,
    ) -> None:
        self._ttl = float(ttl_seconds)
        self._clock = clock or time.monotonic
        self._entries: Dict[Tuple[str, str], _CacheEntry] = {}

    def _key(self, class_filter: str, frame: str) -> Tuple[str, str]:
        return (class_filter or "", frame or "base_link")

    def get(self, *, class_filter: str, frame: str) -> Optional[Any]:
        entry = self._entries.get(self._key(class_filter, frame))
        if entry is None:
            return None
        if self._clock() - entry.inserted_at > self._ttl:
            self._entries.pop(self._key(class_filter, frame), None)
            return None
        return entry.payload

    def put(self, *, class_filter: str, frame: str, payload: Any) -> None:
        self._entries[self._key(class_filter, frame)] = _CacheEntry(
            payload=payload,
            inserted_at=self._clock(),
        )

    def invalidate(self, *, reason: str = "") -> None:
        self._entries.clear()

    def on_robot_mode(self, mode: str) -> None:
        if mode != "IDLE":
            self.invalidate(reason=f"mode={mode}")

    def on_motion_complete(self, *, tool_changed_world: bool) -> None:
        if tool_changed_world:
            self.invalidate(reason="tool_changed_world")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/llm_gateway && python -m pytest tests/test_scene_cache.py -v`
Expected: all 8 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/llm_gateway/scene_cache.py src/llm_gateway/tests/test_scene_cache.py
git commit -m "feat(llm_gateway): add SceneSnapshotCache for ReAct perception reuse"
```

---

## Task 2: Wire cache into `QueryPerceptionTool`

**Files:**
- Modify: `src/llm_gateway/llm_gateway/react_planner.py` (only `QueryPerceptionTool.invoke`, lines 1559–1636)
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py` (instantiate cache, wire to ReAct context)
- Test: `src/llm_gateway/tests/test_react_tools.py` (extend with cache-hit test)

- [ ] **Step 1: Write the failing test**

Append to `src/llm_gateway/tests/test_react_tools.py`:

```python
def test_query_perception_uses_scene_cache(monkeypatch):
    """Second call within TTL reuses cache; cache_hit flag is set."""
    from llm_gateway.react_planner import QueryPerceptionTool
    from llm_gateway.scene_cache import SceneSnapshotCache

    tool = QueryPerceptionTool()
    cache = SceneSnapshotCache(ttl_seconds=2.0)

    call_count = {"n": 0}

    def fake_live_query(args):
        call_count["n"] += 1
        return {
            "ok": True,
            "payload": {"detections": [{"class_id": "red_block"}], "count": 1},
        }

    class FakeStateInjector:
        def snapshot(self):
            return {"robot_state": {"mode": "IDLE"}}

    class FakeNode:
        _query_perception_detections = staticmethod(fake_live_query)
        scene_cache = cache

    class FakeContext:
        ros_node = FakeNode()
        state_injector = FakeStateInjector()

    args = {"class_filter": "red_block"}
    first = tool.invoke(args, FakeContext())
    second = tool.invoke(args, FakeContext())

    assert first.ok is True and second.ok is True
    assert call_count["n"] == 1, "second call must hit the cache"
    assert second.payload.get("cache_hit") is True
    assert first.payload.get("cache_hit") is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/llm_gateway && python -m pytest tests/test_react_tools.py::test_query_perception_uses_scene_cache -v`
Expected: FAIL — `call_count["n"] == 2` (no cache lookup yet).

- [ ] **Step 3: Modify `QueryPerceptionTool.invoke`**

In `src/llm_gateway/llm_gateway/react_planner.py`, replace the body of `QueryPerceptionTool.invoke` (currently lines 1573–1636) with:

```python
    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        snapshot = context.state_injector.snapshot()
        mode = snapshot.get("robot_state", {}).get("mode", "IDLE")
        if mode != "IDLE":
            return ToolResult(
                ok=False,
                error=f"perception_blocked_during_motion (mode={mode})",
            )

        node = getattr(context, "ros_node", None)
        scene_cache = getattr(node, "scene_cache", None)
        class_filter = args.get("class_filter", "") or ""
        frame = args.get("frame", "base_link") or "base_link"

        if scene_cache is not None:
            cached = scene_cache.get(class_filter=class_filter, frame=frame)
            if cached is not None:
                enriched = dict(cached)
                enriched["cache_hit"] = True
                return ToolResult(ok=True, payload=enriched)

        live_query = getattr(node, "_query_perception_detections", None)
        if callable(live_query):
            result = live_query(args)
            raw_detections = (result.get("payload") or {}).get("detections", [])
            are_ros_msgs = (
                raw_detections
                and _format_detections_from_ros is not None
                and not isinstance(raw_detections[0], dict)
            )
            if are_ros_msgs:
                try:
                    formatted = _format_detections_from_ros(raw_detections)
                    if class_filter:
                        cf = class_filter.strip().lower()
                        formatted = [
                            d
                            for d in formatted
                            if cf in d.get("class_id", "").lower()
                            or cf in d.get("description", "").lower()
                        ]
                    summary_parts = [d["description"] for d in formatted]
                    result["payload"] = {
                        "detections": formatted,
                        "count": len(formatted),
                        "summary": "; ".join(summary_parts)
                        if summary_parts
                        else "No objects detected.",
                    }
                except Exception:
                    pass

            payload = dict(result.get("payload") or {})
            payload["cache_hit"] = False
            if scene_cache is not None and result.get("ok"):
                scene_cache.put(
                    class_filter=class_filter,
                    frame=frame,
                    payload=payload,
                )
            return ToolResult(
                ok=bool(result.get("ok")),
                error=result.get("error"),
                payload=payload,
            )

        if not _W4_AVAILABLE:
            return ToolResult(
                ok=False,
                error="perception_not_available",
                payload={"hint": "gp4_perception package is not installed or built"},
            )
        result = query_perception(args=args, context_state=snapshot)
        payload = dict(result.get("payload") or {})
        payload["cache_hit"] = False
        if scene_cache is not None and result.get("ok"):
            scene_cache.put(
                class_filter=class_filter,
                frame=frame,
                payload=payload,
            )
        return ToolResult(
            ok=result["ok"],
            error=result.get("error"),
            payload=payload,
        )
```

- [ ] **Step 4: Wire `scene_cache` on the node**

In `src/llm_gateway/llm_gateway/llm_gateway_node.py`, locate the `__init__` of `LLMGatewayNode` (search for `self._react_plan_cache: Dict[str, Any] = {}` around line 203) and immediately below it add:

```python
        from llm_gateway.scene_cache import SceneSnapshotCache

        scene_cache_ttl = float(
            self.declare_parameter("react.scene_cache_ttl_s", 2.0)
            .get_parameter_value()
            .double_value
        )
        self.scene_cache = SceneSnapshotCache(ttl_seconds=scene_cache_ttl)
```

Also, in the same file, find any spot the node reacts to robot-mode change (search for `_on_robot_status` or `state_injector.update_robot_status`). Inside the existing callback body, append:

```python
        try:
            self.scene_cache.on_robot_mode(status.get("mode", "IDLE"))
        except Exception:
            pass
```

If no such hook exists yet, defer the wiring to Task 5 where motion completion calls land. Add a TODO comment and leave the cache TTL-only invalidation for now — the unit test already asserts the get/put contract, motion-mode invalidation is exercised in Task 5.

- [ ] **Step 5: Run test to verify it passes**

Run: `cd src/llm_gateway && python -m pytest tests/test_react_tools.py -v`
Expected: all previously-passing tests still pass; new `test_query_perception_uses_scene_cache` PASS.

- [ ] **Step 6: Commit**

```bash
git add src/llm_gateway/llm_gateway/react_planner.py \
        src/llm_gateway/llm_gateway/llm_gateway_node.py \
        src/llm_gateway/tests/test_react_tools.py
git commit -m "feat(llm_gateway): plumb SceneSnapshotCache through QueryPerceptionTool"
```

---

## Task 3: Safety `composite_limits` config + loader helper

**Files:**
- Modify: `src/safety/config/safety_rules.yaml`
- Create: `src/safety/safety/composite_limits.py`
- Test: `src/safety/tests/test_composite_limits.py`

- [ ] **Step 1: Write the failing test**

Create `src/safety/tests/test_composite_limits.py`:

```python
"""Unit tests for safety.composite_limits."""

from __future__ import annotations

import pytest

from safety.composite_limits import (
    CompositeLimits,
    composite_limits_from_rules,
    DEFAULT_COMPOSITE_LIMITS,
)


def test_defaults_used_when_block_missing():
    limits = composite_limits_from_rules({})
    assert limits == DEFAULT_COMPOSITE_LIMITS


def test_override_via_rules():
    rules = {
        "composite_limits": {
            "max_sequence_length": 4,
            "max_pick_approach_offset_m": 0.08,
        }
    }
    limits = composite_limits_from_rules(rules)
    assert limits.max_sequence_length == 4
    assert limits.max_pick_approach_offset_m == 0.08
    # untouched keys keep defaults
    assert limits.pick_descent_max_m == DEFAULT_COMPOSITE_LIMITS.pick_descent_max_m


def test_approach_axis_whitelist_enforced():
    limits = composite_limits_from_rules({})
    assert "-z_tool" in limits.approach_axis_whitelist
    assert "+z_tool" not in limits.approach_axis_whitelist


def test_negative_value_rejected():
    with pytest.raises(ValueError, match="non-negative"):
        composite_limits_from_rules(
            {"composite_limits": {"pick_descent_max_m": -0.01}}
        )


def test_sequence_length_must_be_positive_int():
    with pytest.raises(ValueError, match="positive int"):
        composite_limits_from_rules(
            {"composite_limits": {"max_sequence_length": 0}}
        )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/safety && python -m pytest tests/test_composite_limits.py -v`
Expected: `ImportError` — `safety.composite_limits` missing.

- [ ] **Step 3: Create the helper**

Create `src/safety/safety/composite_limits.py`:

```python
"""Composite-flow safety caps loaded from safety_rules.yaml."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Tuple


@dataclass(frozen=True)
class CompositeLimits:
    max_sequence_length: int = 8
    max_pick_approach_offset_m: float = 0.12
    pick_descent_max_m: float = 0.06
    pick_lift_max_m: float = 0.10
    place_descent_max_m: float = 0.06
    approach_axis_whitelist: Tuple[str, ...] = field(
        default_factory=lambda: ("-z_tool", "-z_base")
    )


DEFAULT_COMPOSITE_LIMITS = CompositeLimits()


_NON_NEG_FIELDS = (
    "max_pick_approach_offset_m",
    "pick_descent_max_m",
    "pick_lift_max_m",
    "place_descent_max_m",
)


def composite_limits_from_rules(rules: dict[str, Any] | None) -> CompositeLimits:
    """Return a CompositeLimits built from a loaded safety_rules dict.

    Missing keys fall back to DEFAULT_COMPOSITE_LIMITS. Invalid values raise
    ValueError so misconfiguration fails closed before any motion.
    """
    if not rules:
        return DEFAULT_COMPOSITE_LIMITS
    block = rules.get("composite_limits") or {}

    max_seq = int(block.get("max_sequence_length", DEFAULT_COMPOSITE_LIMITS.max_sequence_length))
    if max_seq <= 0:
        raise ValueError("composite_limits.max_sequence_length must be a positive int")

    values: dict[str, Any] = {"max_sequence_length": max_seq}
    for fld in _NON_NEG_FIELDS:
        default = getattr(DEFAULT_COMPOSITE_LIMITS, fld)
        v = float(block.get(fld, default))
        if v < 0.0:
            raise ValueError(f"composite_limits.{fld} must be non-negative, got {v}")
        values[fld] = v

    whitelist = tuple(
        str(x) for x in block.get(
            "approach_axis_whitelist",
            list(DEFAULT_COMPOSITE_LIMITS.approach_axis_whitelist),
        )
    )
    if not whitelist:
        raise ValueError("composite_limits.approach_axis_whitelist must be non-empty")
    values["approach_axis_whitelist"] = whitelist

    return CompositeLimits(**values)
```

- [ ] **Step 4: Update `safety_rules.yaml`**

Append the following block at the end of `src/safety/config/safety_rules.yaml` (before the operational_joint_limits block, keep file order tidy):

```yaml
# Composite-flow caps for ReAct PickObject / PlaceObject / Approach / Retreat tools.
# All values here are stricter than primitive-level caps; tightening is safe,
# loosening requires a new safety review per .claude/rules/safety-first.md.
composite_limits:
  max_sequence_length: 8
  max_pick_approach_offset_m: 0.12
  pick_descent_max_m: 0.06
  pick_lift_max_m: 0.10
  place_descent_max_m: 0.06
  approach_axis_whitelist:
    - "-z_tool"
    - "-z_base"
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd src/safety && python -m pytest tests/test_composite_limits.py -v`
Expected: all 5 tests PASS.

- [ ] **Step 6: Run full safety test suite to confirm no regression**

Run: `cd src/safety && python -m pytest tests/ -v`
Expected: previously-passing tests still pass.

- [ ] **Step 7: Commit**

```bash
git add src/safety/safety/composite_limits.py \
        src/safety/tests/test_composite_limits.py \
        src/safety/config/safety_rules.yaml
git commit -m "feat(safety): add composite_limits config + loader helper"
```

---

## Task 4: Composite-tool safety pre-check helper

**Files:**
- Create: `src/llm_gateway/llm_gateway/composite_tools.py` (skeleton + `_safety_caps_check`)
- Test: `src/llm_gateway/tests/test_composite_tools.py`

- [ ] **Step 1: Write the failing test**

Create `src/llm_gateway/tests/test_composite_tools.py`:

```python
"""Unit tests for composite ReAct tools."""

from __future__ import annotations

import pytest

from llm_gateway.composite_tools import (
    PickRequest,
    safety_caps_check,
)
from safety.composite_limits import DEFAULT_COMPOSITE_LIMITS


def test_safety_caps_pick_ok():
    req = PickRequest(
        object_id="red_block",
        approach_offset_m=0.05,
        grasp_descent_m=0.04,
        lift_m=0.06,
        approach_axis="-z_tool",
    )
    ok, reason = safety_caps_check(req, DEFAULT_COMPOSITE_LIMITS)
    assert ok is True and reason == ""


def test_safety_caps_pick_offset_too_large():
    req = PickRequest(
        object_id="red_block",
        approach_offset_m=0.25,
        grasp_descent_m=0.04,
        lift_m=0.06,
        approach_axis="-z_tool",
    )
    ok, reason = safety_caps_check(req, DEFAULT_COMPOSITE_LIMITS)
    assert ok is False
    assert "approach_offset_m" in reason


def test_safety_caps_axis_not_whitelisted():
    req = PickRequest(
        object_id="red_block",
        approach_offset_m=0.05,
        grasp_descent_m=0.04,
        lift_m=0.06,
        approach_axis="+y_base",
    )
    ok, reason = safety_caps_check(req, DEFAULT_COMPOSITE_LIMITS)
    assert ok is False
    assert "approach_axis" in reason


def test_safety_caps_negative_descent_rejected():
    req = PickRequest(
        object_id="red_block",
        approach_offset_m=0.05,
        grasp_descent_m=-0.01,
        lift_m=0.06,
        approach_axis="-z_tool",
    )
    ok, reason = safety_caps_check(req, DEFAULT_COMPOSITE_LIMITS)
    assert ok is False
    assert "grasp_descent_m" in reason
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: `ImportError` — `llm_gateway.composite_tools` missing.

- [ ] **Step 3: Create skeleton module + helper**

Create `src/llm_gateway/llm_gateway/composite_tools.py`:

```python
"""Composite ReAct tools that emit Semantic IR (sequence/single-intent).

All tools route through the existing /validate_command → motion_core →
hw_adapter pipeline. None of them talk to MoveIt2 or hw_adapter directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, ClassVar, Optional

from safety.composite_limits import CompositeLimits


@dataclass(frozen=True)
class PickRequest:
    object_id: str
    approach_offset_m: float
    grasp_descent_m: float
    lift_m: float
    approach_axis: str


@dataclass(frozen=True)
class PlaceRequest:
    target_pose: Optional[dict]
    object_id: Optional[str]
    approach_offset_m: float
    descent_m: float
    approach_axis: str


def safety_caps_check(
    req: PickRequest | PlaceRequest, limits: CompositeLimits
) -> tuple[bool, str]:
    """Return (ok, reason) given a composite request and loaded safety caps."""

    if req.approach_axis not in limits.approach_axis_whitelist:
        return (
            False,
            f"approach_axis '{req.approach_axis}' not in whitelist "
            f"{list(limits.approach_axis_whitelist)}",
        )

    if req.approach_offset_m < 0:
        return False, "approach_offset_m must be non-negative"
    if req.approach_offset_m > limits.max_pick_approach_offset_m:
        return (
            False,
            f"approach_offset_m {req.approach_offset_m} exceeds cap "
            f"{limits.max_pick_approach_offset_m}",
        )

    if isinstance(req, PickRequest):
        if req.grasp_descent_m < 0:
            return False, "grasp_descent_m must be non-negative"
        if req.grasp_descent_m > limits.pick_descent_max_m:
            return (
                False,
                f"grasp_descent_m {req.grasp_descent_m} exceeds cap "
                f"{limits.pick_descent_max_m}",
            )
        if req.lift_m < 0:
            return False, "lift_m must be non-negative"
        if req.lift_m > limits.pick_lift_max_m:
            return (
                False,
                f"lift_m {req.lift_m} exceeds cap {limits.pick_lift_max_m}",
            )
    else:
        if req.descent_m < 0:
            return False, "descent_m must be non-negative"
        if req.descent_m > limits.place_descent_max_m:
            return (
                False,
                f"descent_m {req.descent_m} exceeds cap {limits.place_descent_max_m}",
            )

    return True, ""
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: all 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/llm_gateway/composite_tools.py \
        src/llm_gateway/tests/test_composite_tools.py
git commit -m "feat(llm_gateway): composite tools skeleton + safety caps helper"
```

---

## Task 5: `ApproachObjectTool` + `RetreatTool`

**Files:**
- Modify: `src/llm_gateway/llm_gateway/composite_tools.py`
- Modify: `src/llm_gateway/tests/test_composite_tools.py`

- [ ] **Step 1: Write the failing tests**

Append to `src/llm_gateway/tests/test_composite_tools.py`:

```python
from llm_gateway.composite_tools import (
    ApproachObjectTool,
    RetreatTool,
)
from llm_gateway.react_planner import ToolResult


class _FakeStateInjector:
    def snapshot(self):
        return {
            "robot_state": {"mode": "IDLE"},
            "current_pose": {
                "position": {"x": 0.30, "y": 0.10, "z": 0.30},
                "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
                "frame_id": "base_link",
            },
        }


class _FakeNode:
    def __init__(self, detection=None, limits=None):
        from safety.composite_limits import DEFAULT_COMPOSITE_LIMITS

        self.scene_cache_detection = detection
        self.composite_limits = limits or DEFAULT_COMPOSITE_LIMITS
        self.submitted_ir: list[dict] = []

    def submit_semantic_ir(self, ir: dict) -> dict:
        self.submitted_ir.append(ir)
        return {"ok": True, "plan_id": "p-001"}

    def query_scene_object(self, object_id: str):
        return self.scene_cache_detection


class _FakeContext:
    def __init__(self, node):
        self.ros_node = node
        self.state_injector = _FakeStateInjector()


def test_approach_emits_lin_with_offset_above_object():
    detection = {
        "object_id": "red_block",
        "pose": {
            "position": {"x": 0.40, "y": 0.05, "z": 0.20},
            "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
            "frame_id": "base_link",
        },
    }
    node = _FakeNode(detection=detection)
    tool = ApproachObjectTool()
    res = tool.invoke(
        {"object_id": "red_block", "offset_m": 0.05, "approach_axis": "-z_tool"},
        _FakeContext(node),
    )
    assert isinstance(res, ToolResult) and res.ok is True
    assert node.submitted_ir, "approach must submit a Semantic IR"
    ir = node.submitted_ir[-1]
    assert ir["intent"] == "absolute_move_lin"
    target_z = ir["target"]["position"]["z"]
    assert abs(target_z - (0.20 + 0.05)) < 1e-6
    assert ir["metadata"]["source"] == "composite_approach"


def test_approach_rejects_when_object_not_found():
    node = _FakeNode(detection=None)
    tool = ApproachObjectTool()
    res = tool.invoke(
        {"object_id": "red_block", "offset_m": 0.05, "approach_axis": "-z_tool"},
        _FakeContext(node),
    )
    assert res.ok is False
    assert "object_not_found" in (res.error or "")
    assert node.submitted_ir == []


def test_approach_rejects_offset_over_safety_cap():
    detection = {
        "object_id": "red_block",
        "pose": {
            "position": {"x": 0.40, "y": 0.05, "z": 0.20},
            "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
            "frame_id": "base_link",
        },
    }
    node = _FakeNode(detection=detection)
    tool = ApproachObjectTool()
    res = tool.invoke(
        {"object_id": "red_block", "offset_m": 0.25, "approach_axis": "-z_tool"},
        _FakeContext(node),
    )
    assert res.ok is False
    assert "safety_cap_violation" in (res.error or "")
    assert node.submitted_ir == []


def test_retreat_emits_lin_in_axis_direction():
    node = _FakeNode()
    tool = RetreatTool()
    res = tool.invoke(
        {"offset_m": 0.05, "axis": "-z_tool"},
        _FakeContext(node),
    )
    assert res.ok is True
    ir = node.submitted_ir[-1]
    assert ir["intent"] == "absolute_move_lin"
    # axis -z_tool means we LIFT in base_link, so target z > current z
    assert ir["target"]["position"]["z"] > 0.30
    assert ir["metadata"]["source"] == "composite_retreat"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: 4 new tests FAIL (`ApproachObjectTool`, `RetreatTool` not defined).

- [ ] **Step 3: Implement the tools**

Append to `src/llm_gateway/llm_gateway/composite_tools.py`:

```python
from llm_gateway.react_planner import Tool, ToolResult


_AXIS_VECTOR = {
    # axis name -> unit vector applied to target position to LIFT/RETREAT.
    # "-z_tool" means tool's -Z (downward when wrist is upright); to RETREAT
    # along -z_tool we add +z_base to the position. This is a conservative
    # approximation valid when the tool quaternion roughly aligns -Z_tool to
    # -Z_base (the GP4 tabletop pick configuration). Composite tools fail
    # closed for other tool orientations until §11 future work lands.
    "-z_tool": (0.0, 0.0, 1.0),
    "-z_base": (0.0, 0.0, 1.0),
}


def _approach_position(target: dict, axis: str, offset_m: float) -> dict:
    ux, uy, uz = _AXIS_VECTOR[axis]
    return {
        "x": target["x"] + ux * offset_m,
        "y": target["y"] + uy * offset_m,
        "z": target["z"] + uz * offset_m,
    }


def _resolve_object(node, object_id: str) -> Optional[dict]:
    resolver = getattr(node, "query_scene_object", None)
    if callable(resolver):
        return resolver(object_id)
    return None


def _load_limits(node) -> CompositeLimits:
    return getattr(node, "composite_limits", None) or CompositeLimits()


def _submit(node, ir: dict) -> ToolResult:
    submitter = getattr(node, "submit_semantic_ir", None)
    if not callable(submitter):
        return ToolResult(ok=False, error="submit_semantic_ir not wired on node")
    result = submitter(ir)
    if result.get("ok"):
        return ToolResult(ok=True, payload=result)
    return ToolResult(
        ok=False,
        error=result.get("error", "submit_failed"),
        payload=result,
    )


class ApproachObjectTool(Tool):
    name = "approach_object"
    description = (
        "Move to a pose offset above (or away from) a perceived object along "
        "an approach axis. Emits absolute_move_lin Semantic IR."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "object_id": {"type": "string"},
            "offset_m": {"type": "number"},
            "approach_axis": {"type": "string"},
        },
        "required": ["object_id", "offset_m", "approach_axis"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = context.ros_node
        limits = _load_limits(node)
        req = PickRequest(
            object_id=args["object_id"],
            approach_offset_m=float(args["offset_m"]),
            grasp_descent_m=0.0,
            lift_m=0.0,
            approach_axis=args["approach_axis"],
        )
        ok, reason = safety_caps_check(req, limits)
        if not ok:
            return ToolResult(ok=False, error=f"safety_cap_violation: {reason}")

        detection = _resolve_object(node, req.object_id)
        if not detection:
            return ToolResult(
                ok=False, error=f"object_not_found: {req.object_id}"
            )

        target = detection["pose"]["position"]
        approach_pos = _approach_position(target, req.approach_axis, req.approach_offset_m)
        ir = {
            "intent": "absolute_move_lin",
            "target": {
                "position": approach_pos,
                "orientation": detection["pose"]["orientation"],
                "frame_id": detection["pose"].get("frame_id", "base_link"),
            },
            "metadata": {"source": "composite_approach"},
        }
        return _submit(node, ir)


class RetreatTool(Tool):
    name = "retreat"
    description = "Move the TCP back along an axis by offset_m. Emits absolute_move_lin Semantic IR."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "offset_m": {"type": "number"},
            "axis": {"type": "string"},
        },
        "required": ["offset_m", "axis"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = context.ros_node
        limits = _load_limits(node)
        axis = args["axis"]
        offset_m = float(args["offset_m"])
        if axis not in limits.approach_axis_whitelist:
            return ToolResult(
                ok=False,
                error=f"safety_cap_violation: retreat axis '{axis}' not in whitelist",
            )
        if offset_m < 0 or offset_m > limits.max_pick_approach_offset_m:
            return ToolResult(
                ok=False,
                error=f"safety_cap_violation: offset_m {offset_m} out of [0, "
                f"{limits.max_pick_approach_offset_m}]",
            )

        current = context.state_injector.snapshot().get("current_pose")
        if not current:
            return ToolResult(ok=False, error="current_pose_unavailable")

        retreat_pos = _approach_position(current["position"], axis, offset_m)
        ir = {
            "intent": "absolute_move_lin",
            "target": {
                "position": retreat_pos,
                "orientation": current["orientation"],
                "frame_id": current.get("frame_id", "base_link"),
            },
            "metadata": {"source": "composite_retreat"},
        }
        return _submit(node, ir)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/llm_gateway/composite_tools.py \
        src/llm_gateway/tests/test_composite_tools.py
git commit -m "feat(llm_gateway): ApproachObjectTool + RetreatTool composites"
```

---

## Task 6: `PickObjectTool`

**Files:**
- Modify: `src/llm_gateway/llm_gateway/composite_tools.py`
- Modify: `src/llm_gateway/tests/test_composite_tools.py`

- [ ] **Step 1: Write the failing test**

Append to `src/llm_gateway/tests/test_composite_tools.py`:

```python
from llm_gateway.composite_tools import PickObjectTool


def test_pick_emits_sequence_ir_with_correct_steps():
    detection = {
        "object_id": "red_block",
        "pose": {
            "position": {"x": 0.40, "y": 0.05, "z": 0.20},
            "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
            "frame_id": "base_link",
        },
    }
    node = _FakeNode(detection=detection)
    tool = PickObjectTool()
    res = tool.invoke(
        {
            "object_id": "red_block",
            "approach_offset_m": 0.05,
            "grasp_descent_m": 0.04,
            "lift_m": 0.06,
            "approach_axis": "-z_tool",
        },
        _FakeContext(node),
    )
    assert res.ok is True
    ir = node.submitted_ir[-1]
    assert ir["intent"] == "sequence"
    intents = [step["intent"] for step in ir["steps"]]
    assert intents == [
        "absolute_move_lin",   # approach
        "io_set",              # open_gripper
        "absolute_move_lin",   # descent
        "io_set",              # close_gripper
        "absolute_move_lin",   # lift
    ]
    assert ir["metadata"]["source"] == "composite_pick"
    assert ir["metadata"]["tool_changed_world"] is True


def test_pick_safety_cap_lift_too_large():
    detection = {
        "object_id": "red_block",
        "pose": {
            "position": {"x": 0.40, "y": 0.05, "z": 0.20},
            "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
            "frame_id": "base_link",
        },
    }
    node = _FakeNode(detection=detection)
    tool = PickObjectTool()
    res = tool.invoke(
        {
            "object_id": "red_block",
            "approach_offset_m": 0.05,
            "grasp_descent_m": 0.04,
            "lift_m": 0.50,
            "approach_axis": "-z_tool",
        },
        _FakeContext(node),
    )
    assert res.ok is False
    assert "lift_m" in (res.error or "")
    assert node.submitted_ir == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: 2 new tests FAIL (`PickObjectTool` not defined).

- [ ] **Step 3: Implement `PickObjectTool`**

Append to `src/llm_gateway/llm_gateway/composite_tools.py`:

```python
_GRIPPER_IO_PIN = 0  # io_set pin reserved for the gripper command.


def _gripper_step(open_: bool) -> dict:
    return {
        "intent": "io_set",
        "pin": _GRIPPER_IO_PIN,
        "value": 1 if open_ else 0,
        "metadata": {"source": "composite_pick"},
    }


def _lin_step(position: dict, orientation: dict, frame: str, source: str) -> dict:
    return {
        "intent": "absolute_move_lin",
        "target": {
            "position": position,
            "orientation": orientation,
            "frame_id": frame,
        },
        "metadata": {"source": source},
    }


def _descent_position(target: dict, axis: str, descent_m: float) -> dict:
    ux, uy, uz = _AXIS_VECTOR[axis]
    # Descent goes OPPOSITE to the lift/approach vector.
    return {
        "x": target["x"] - ux * descent_m,
        "y": target["y"] - uy * descent_m,
        "z": target["z"] - uz * descent_m,
    }


class PickObjectTool(Tool):
    name = "pick_object"
    description = (
        "Approach an object, open the gripper, descend, close, and lift. "
        "Emits one Semantic IR `sequence` with all steps."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "object_id": {"type": "string"},
            "approach_offset_m": {"type": "number"},
            "grasp_descent_m": {"type": "number"},
            "lift_m": {"type": "number"},
            "approach_axis": {"type": "string"},
        },
        "required": [
            "object_id",
            "approach_offset_m",
            "grasp_descent_m",
            "lift_m",
            "approach_axis",
        ],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = context.ros_node
        limits = _load_limits(node)
        req = PickRequest(
            object_id=args["object_id"],
            approach_offset_m=float(args["approach_offset_m"]),
            grasp_descent_m=float(args["grasp_descent_m"]),
            lift_m=float(args["lift_m"]),
            approach_axis=args["approach_axis"],
        )
        ok, reason = safety_caps_check(req, limits)
        if not ok:
            return ToolResult(ok=False, error=f"safety_cap_violation: {reason}")

        detection = _resolve_object(node, req.object_id)
        if not detection:
            return ToolResult(
                ok=False, error=f"object_not_found: {req.object_id}"
            )

        target_pos = detection["pose"]["position"]
        orient = detection["pose"]["orientation"]
        frame = detection["pose"].get("frame_id", "base_link")

        approach_pos = _approach_position(
            target_pos, req.approach_axis, req.approach_offset_m
        )
        grasp_pos = _descent_position(
            approach_pos, req.approach_axis, req.grasp_descent_m
        )
        lift_pos = _approach_position(grasp_pos, req.approach_axis, req.lift_m)

        steps = [
            _lin_step(approach_pos, orient, frame, "composite_pick"),
            _gripper_step(open_=True),
            _lin_step(grasp_pos, orient, frame, "composite_pick"),
            _gripper_step(open_=False),
            _lin_step(lift_pos, orient, frame, "composite_pick"),
        ]
        if len(steps) > limits.max_sequence_length:
            return ToolResult(
                ok=False,
                error=f"safety_cap_violation: sequence length {len(steps)} "
                f"exceeds max {limits.max_sequence_length}",
            )

        ir = {
            "intent": "sequence",
            "steps": steps,
            "metadata": {
                "source": "composite_pick",
                "tool_changed_world": True,
            },
        }
        return _submit(node, ir)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/llm_gateway/composite_tools.py \
        src/llm_gateway/tests/test_composite_tools.py
git commit -m "feat(llm_gateway): PickObjectTool emits sequence IR"
```

---

## Task 7: `PlaceObjectTool`

**Files:**
- Modify: `src/llm_gateway/llm_gateway/composite_tools.py`
- Modify: `src/llm_gateway/tests/test_composite_tools.py`

- [ ] **Step 1: Write the failing test**

Append to `src/llm_gateway/tests/test_composite_tools.py`:

```python
from llm_gateway.composite_tools import PlaceObjectTool


def test_place_emits_sequence_with_open_gripper_step():
    node = _FakeNode()
    tool = PlaceObjectTool()
    res = tool.invoke(
        {
            "target_pose": {
                "position": {"x": 0.35, "y": -0.05, "z": 0.18},
                "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
                "frame_id": "base_link",
            },
            "approach_offset_m": 0.05,
            "descent_m": 0.04,
            "approach_axis": "-z_tool",
        },
        _FakeContext(node),
    )
    assert res.ok is True
    ir = node.submitted_ir[-1]
    assert ir["intent"] == "sequence"
    intents = [step["intent"] for step in ir["steps"]]
    # approach (lin) -> descent (lin) -> open (io_set) -> retreat (lin)
    assert intents == [
        "absolute_move_lin",
        "absolute_move_lin",
        "io_set",
        "absolute_move_lin",
    ]
    assert ir["metadata"]["source"] == "composite_place"
    assert ir["metadata"]["tool_changed_world"] is True


def test_place_safety_cap_descent_too_large():
    node = _FakeNode()
    tool = PlaceObjectTool()
    res = tool.invoke(
        {
            "target_pose": {
                "position": {"x": 0.35, "y": -0.05, "z": 0.18},
                "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
                "frame_id": "base_link",
            },
            "approach_offset_m": 0.05,
            "descent_m": 0.20,
            "approach_axis": "-z_tool",
        },
        _FakeContext(node),
    )
    assert res.ok is False
    assert "descent_m" in (res.error or "")
    assert node.submitted_ir == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: 2 new tests FAIL.

- [ ] **Step 3: Implement `PlaceObjectTool`**

Append to `src/llm_gateway/llm_gateway/composite_tools.py`:

```python
class PlaceObjectTool(Tool):
    name = "place_object"
    description = (
        "Approach a target pose, descend, release the gripper, retreat. "
        "Emits one Semantic IR `sequence`."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "target_pose": {"type": "object"},
            "object_id": {"type": "string"},
            "approach_offset_m": {"type": "number"},
            "descent_m": {"type": "number"},
            "approach_axis": {"type": "string"},
        },
        "required": ["approach_offset_m", "descent_m", "approach_axis"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = context.ros_node
        limits = _load_limits(node)
        req = PlaceRequest(
            target_pose=args.get("target_pose"),
            object_id=args.get("object_id"),
            approach_offset_m=float(args["approach_offset_m"]),
            descent_m=float(args["descent_m"]),
            approach_axis=args["approach_axis"],
        )
        ok, reason = safety_caps_check(req, limits)
        if not ok:
            return ToolResult(ok=False, error=f"safety_cap_violation: {reason}")

        target_pose: Optional[dict] = req.target_pose
        if target_pose is None and req.object_id:
            detection = _resolve_object(node, req.object_id)
            if detection:
                target_pose = detection["pose"]
        if target_pose is None:
            return ToolResult(
                ok=False,
                error="place_target_missing: provide target_pose or known object_id",
            )

        target_pos = target_pose["position"]
        orient = target_pose["orientation"]
        frame = target_pose.get("frame_id", "base_link")

        approach_pos = _approach_position(
            target_pos, req.approach_axis, req.approach_offset_m
        )
        descent_pos = _descent_position(
            approach_pos, req.approach_axis, req.descent_m
        )
        retreat_pos = _approach_position(
            descent_pos, req.approach_axis, req.approach_offset_m
        )

        steps = [
            _lin_step(approach_pos, orient, frame, "composite_place"),
            _lin_step(descent_pos, orient, frame, "composite_place"),
            _gripper_step(open_=True),
            _lin_step(retreat_pos, orient, frame, "composite_place"),
        ]
        if len(steps) > limits.max_sequence_length:
            return ToolResult(
                ok=False,
                error=f"safety_cap_violation: sequence length {len(steps)} "
                f"exceeds max {limits.max_sequence_length}",
            )

        ir = {
            "intent": "sequence",
            "steps": steps,
            "metadata": {
                "source": "composite_place",
                "tool_changed_world": True,
            },
        }
        return _submit(node, ir)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/llm_gateway/composite_tools.py \
        src/llm_gateway/tests/test_composite_tools.py
git commit -m "feat(llm_gateway): PlaceObjectTool emits sequence IR"
```

---

## Task 8: `EmitSequenceTool` + `RefreshSceneTool`

**Files:**
- Modify: `src/llm_gateway/llm_gateway/composite_tools.py`
- Modify: `src/llm_gateway/tests/test_composite_tools.py`

- [ ] **Step 1: Write the failing tests**

Append to `src/llm_gateway/tests/test_composite_tools.py`:

```python
from llm_gateway.composite_tools import EmitSequenceTool, RefreshSceneTool


def test_emit_sequence_validates_via_contract_and_submits():
    node = _FakeNode()
    tool = EmitSequenceTool()
    res = tool.invoke(
        {
            "steps": [
                {
                    "intent": "absolute_move_lin",
                    "target": {
                        "position": {"x": 0.30, "y": 0.05, "z": 0.25},
                        "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
                        "frame_id": "base_link",
                    },
                },
                {"intent": "wait", "duration_s": 0.5},
            ],
            "metadata": {"reason": "demo"},
        },
        _FakeContext(node),
    )
    assert res.ok is True
    ir = node.submitted_ir[-1]
    assert ir["intent"] == "sequence" and len(ir["steps"]) == 2
    assert ir["metadata"]["source"] == "composite_emit_sequence"


def test_emit_sequence_rejects_unknown_intent():
    node = _FakeNode()
    tool = EmitSequenceTool()
    res = tool.invoke(
        {"steps": [{"intent": "fly_to_moon"}]},
        _FakeContext(node),
    )
    assert res.ok is False
    assert "Unsupported semantic intent" in (res.error or "")
    assert node.submitted_ir == []


def test_emit_sequence_rejects_exceeding_max_length():
    node = _FakeNode()
    tool = EmitSequenceTool()
    too_many = [{"intent": "wait", "duration_s": 0.1} for _ in range(20)]
    res = tool.invoke({"steps": too_many}, _FakeContext(node))
    assert res.ok is False
    assert "max_sequence_length" in (res.error or "")


def test_refresh_scene_invalidates_cache():
    from llm_gateway.scene_cache import SceneSnapshotCache

    cache = SceneSnapshotCache(ttl_seconds=10.0)
    cache.put(class_filter="red_block", frame="base_link", payload={"v": 1})

    class N(_FakeNode):
        def __init__(self):
            super().__init__()
            self.scene_cache = cache

    node = N()
    tool = RefreshSceneTool()
    res = tool.invoke({}, _FakeContext(node))
    assert res.ok is True
    assert cache.get(class_filter="red_block", frame="base_link") is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: 4 new tests FAIL.

- [ ] **Step 3: Implement the tools**

Append to `src/llm_gateway/llm_gateway/composite_tools.py`:

```python
from llm_gateway.semantic_ir_contract import validate_semantic_ir_contract


class EmitSequenceTool(Tool):
    name = "emit_sequence"
    description = (
        "Submit a Semantic IR `sequence` of pre-built steps. Each step is "
        "validated by the Semantic IR contract before submission."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "steps": {"type": "array", "items": {"type": "object"}},
            "metadata": {"type": "object"},
        },
        "required": ["steps"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = context.ros_node
        limits = _load_limits(node)
        steps = args.get("steps", [])
        if not steps:
            return ToolResult(ok=False, error="emit_sequence requires non-empty steps")
        if len(steps) > limits.max_sequence_length:
            return ToolResult(
                ok=False,
                error=f"safety_cap_violation: max_sequence_length {limits.max_sequence_length}"
                f" exceeded (got {len(steps)})",
            )

        ir = {
            "intent": "sequence",
            "steps": steps,
            "metadata": {
                **(args.get("metadata") or {}),
                "source": "composite_emit_sequence",
            },
        }
        contract = validate_semantic_ir_contract(ir)
        if not contract.valid:
            return ToolResult(
                ok=False,
                error=contract.reason,
                payload={"hint": contract.hint},
            )
        return _submit(node, ir)


class RefreshSceneTool(Tool):
    name = "refresh_scene"
    description = (
        "Invalidate the perception cache so the next query_perception call "
        "fetches a fresh scene."
    )
    is_readonly = True
    input_schema: ClassVar[dict] = {"type": "object", "properties": {}}

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = context.ros_node
        cache = getattr(node, "scene_cache", None)
        if cache is None:
            return ToolResult(
                ok=False, error="scene_cache not configured on node"
            )
        cache.invalidate(reason="refresh_scene_tool")
        return ToolResult(ok=True, payload={"invalidated": True})
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_tools.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/llm_gateway/llm_gateway/composite_tools.py \
        src/llm_gateway/tests/test_composite_tools.py
git commit -m "feat(llm_gateway): EmitSequenceTool + RefreshSceneTool"
```

---

## Task 9: Composite IR contract regression test

**Files:**
- Create: `src/llm_gateway/tests/test_composite_ir_contract.py`

- [ ] **Step 1: Write the test**

Create `src/llm_gateway/tests/test_composite_ir_contract.py`:

```python
"""End-to-end contract check: composite tools emit IR that the
Semantic IR contract validator accepts.

This catches drift between composite-emitted IR and validator rules without
needing a live gateway.
"""

from __future__ import annotations

from llm_gateway.composite_tools import (
    ApproachObjectTool,
    PickObjectTool,
    PlaceObjectTool,
)
from llm_gateway.react_planner import ToolResult
from llm_gateway.semantic_ir_contract import validate_semantic_ir_contract
from safety.composite_limits import DEFAULT_COMPOSITE_LIMITS


class _StateInjector:
    def snapshot(self):
        return {
            "robot_state": {"mode": "IDLE"},
            "current_pose": {
                "position": {"x": 0.30, "y": 0.10, "z": 0.30},
                "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
                "frame_id": "base_link",
            },
        }


class _Node:
    def __init__(self):
        self.composite_limits = DEFAULT_COMPOSITE_LIMITS
        self.submitted_ir: list[dict] = []

    def submit_semantic_ir(self, ir):
        self.submitted_ir.append(ir)
        return {"ok": True, "plan_id": "p-100"}

    def query_scene_object(self, oid):
        return {
            "object_id": oid,
            "pose": {
                "position": {"x": 0.40, "y": 0.05, "z": 0.20},
                "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
                "frame_id": "base_link",
            },
        }


class _Ctx:
    def __init__(self):
        self.ros_node = _Node()
        self.state_injector = _StateInjector()


def _run(tool, args):
    ctx = _Ctx()
    res = tool.invoke(args, ctx)
    assert isinstance(res, ToolResult) and res.ok, res.error
    return ctx.ros_node.submitted_ir[-1]


def test_approach_ir_passes_contract():
    ir = _run(
        ApproachObjectTool(),
        {"object_id": "red_block", "offset_m": 0.05, "approach_axis": "-z_tool"},
    )
    assert validate_semantic_ir_contract(ir).valid


def test_pick_ir_passes_contract():
    ir = _run(
        PickObjectTool(),
        {
            "object_id": "red_block",
            "approach_offset_m": 0.05,
            "grasp_descent_m": 0.04,
            "lift_m": 0.06,
            "approach_axis": "-z_tool",
        },
    )
    assert validate_semantic_ir_contract(ir).valid


def test_place_ir_passes_contract():
    ir = _run(
        PlaceObjectTool(),
        {
            "target_pose": {
                "position": {"x": 0.35, "y": -0.05, "z": 0.18},
                "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
                "frame_id": "base_link",
            },
            "approach_offset_m": 0.05,
            "descent_m": 0.04,
            "approach_axis": "-z_tool",
        },
    )
    assert validate_semantic_ir_contract(ir).valid
```

- [ ] **Step 2: Run test**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_ir_contract.py -v`
Expected: all 3 tests PASS. If any fail because the contract rejects an intent
used here (e.g., `io_set`), update the IR step builder to use only frozen
intents and add the failing combo to `test_composite_tools.py` regression.

- [ ] **Step 3: Commit**

```bash
git add src/llm_gateway/tests/test_composite_ir_contract.py
git commit -m "test(llm_gateway): composite IR survives Semantic IR contract"
```

---

## Task 10: Register composites + wire `submit_semantic_ir` + cache invalidation hooks

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`

- [ ] **Step 1: Write a focused integration test (no ROS spin)**

Create `src/llm_gateway/tests/test_composite_registration.py`:

```python
"""Verify that composite tools are wired into the gateway's ReAct registry
when the feature flag is enabled and the node exposes the expected
attributes (composite_limits, scene_cache, submit_semantic_ir,
query_scene_object).
"""

from __future__ import annotations

import pytest


def test_registry_contains_composite_tools(monkeypatch):
    from llm_gateway import llm_gateway_node as mod

    class _Stub(mod.LLMGatewayNode):
        def __init__(self):  # bypass rclpy.Node.__init__
            self._composite_tools_enabled = True

        # Stub out ROS interactions used during build_tool_registry.
        def declare_parameter(self, *args, **kwargs):
            class P:
                def get_parameter_value(self):
                    return type("V", (), {"double_value": 2.0, "bool_value": True})()

            return P()

    stub = _Stub()
    registry = mod.LLMGatewayNode._build_tool_registry(stub)
    names = {t["name"] for t in registry.list_tools()}
    assert {
        "approach_object",
        "retreat",
        "pick_object",
        "place_object",
        "emit_sequence",
        "refresh_scene",
    }.issubset(names)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/llm_gateway && python -m pytest tests/test_composite_registration.py -v`
Expected: FAIL — `_build_tool_registry` either missing or doesn't register composites.

- [ ] **Step 3: Refactor registry construction in `llm_gateway_node.py`**

In `src/llm_gateway/llm_gateway/llm_gateway_node.py`, locate the existing inline `ToolRegistry()....register(...)` chain near line 177 (the block ending with `.register(ComputeArcPointsTool())`). Replace it with a call to a new method:

```python
        self._tool_registry = self._build_tool_registry()
```

Then add the method on `LLMGatewayNode` (anywhere above `__init__`'s closing):

```python
    def _build_tool_registry(self) -> ToolRegistry:
        from llm_gateway.composite_tools import (
            ApproachObjectTool,
            RetreatTool,
            PickObjectTool,
            PlaceObjectTool,
            EmitSequenceTool,
            RefreshSceneTool,
        )

        registry = (
            ToolRegistry()
            .register(GetCurrentPoseTool())
            .register(PlanMotionTool())
            .register(SubmitMotionTool())
            .register(WaitForStateTool())
            .register(SetSpeedTool())
            .register(QueryPerceptionTool())
            .register(GripperOpenTool())
            .register(GripperCloseTool())
            .register(ComputeArcPointsTool())
        )

        composites_enabled = getattr(self, "_composite_tools_enabled", None)
        if composites_enabled is None:
            composites_enabled = bool(
                self.declare_parameter("react.composite_tools_enabled", True)
                .get_parameter_value()
                .bool_value
            )

        if composites_enabled:
            (
                registry.register(ApproachObjectTool())
                .register(RetreatTool())
                .register(PickObjectTool())
                .register(PlaceObjectTool())
                .register(EmitSequenceTool())
                .register(RefreshSceneTool())
            )
        return registry
```

- [ ] **Step 4: Add the `submit_semantic_ir` + `query_scene_object` + `composite_limits` plumbing**

Still inside `LLMGatewayNode`, add three small adapters. Place them next to the other ReAct helpers (search for `_query_perception_detections`):

```python
    def submit_semantic_ir(self, ir: Dict[str, Any]) -> Dict[str, Any]:
        """Adapter used by composite tools.

        Routes through the standard IR review path so /validate_command
        runs before any motion is dispatched. Returns {ok, error, plan_id}.
        """
        review_outcome = self._review_semantic_ir(ir)  # existing private method
        if not review_outcome.get("ok"):
            return review_outcome
        # Existing helper that calls execute_motion; do not duplicate.
        return self._submit_reviewed_ir(review_outcome["normalized_ir"])

    def query_scene_object(self, object_id: str) -> Optional[Dict[str, Any]]:
        """Resolve an object_id to a pose using the perception snapshot cache."""
        cache = getattr(self, "scene_cache", None)
        if cache is None:
            return None
        snapshot = cache.get(class_filter=object_id, frame="base_link")
        if not snapshot:
            return None
        for detection in snapshot.get("detections", []):
            if (
                detection.get("class_id", "").lower() == object_id.lower()
                or object_id.lower() in detection.get("description", "").lower()
            ):
                return {
                    "object_id": object_id,
                    "pose": detection.get("pose"),
                }
        return None

    @property
    def composite_limits(self):
        from safety.composite_limits import composite_limits_from_rules
        from safety.policy_loader import load_safety_rules

        cached = getattr(self, "_composite_limits_cached", None)
        if cached is not None:
            return cached
        rules = load_safety_rules()
        self._composite_limits_cached = composite_limits_from_rules(rules)
        return self._composite_limits_cached
```

Note: if `_review_semantic_ir` or `_submit_reviewed_ir` do not exist with those exact names in the file, locate the equivalents used by the existing `raw_intent_cb` / `review_intent` flow (search for `validate_command` and `execute_motion`) and adapt the calls. Do **not** duplicate validation or execution code — only wrap what already exists.

- [ ] **Step 5: Add tool-changed-world invalidation when a sequence submission succeeds**

In `submit_semantic_ir`, immediately after `_submit_reviewed_ir` returns success, append:

```python
        if review_outcome["normalized_ir"].get("metadata", {}).get(
            "tool_changed_world"
        ):
            cache = getattr(self, "scene_cache", None)
            if cache is not None:
                cache.on_motion_complete(tool_changed_world=True)
```

- [ ] **Step 6: Run all gateway tests**

Run: `cd src/llm_gateway && python -m pytest tests/ -v`
Expected: all previously-passing tests still pass; `test_composite_registration.py` PASS.

- [ ] **Step 7: Commit**

```bash
git add src/llm_gateway/llm_gateway/llm_gateway_node.py \
        src/llm_gateway/tests/test_composite_registration.py
git commit -m "feat(llm_gateway): register composite tools + wire submit_semantic_ir"
```

---

## Task 11: Pick/place simulation integration test

**Files:**
- Create: `src/llm_gateway/tests/test_pick_place_sim.py`

- [ ] **Step 1: Write the integration test**

Create `src/llm_gateway/tests/test_pick_place_sim.py`:

```python
"""Integration test: ReAct drives a pick within budget using fake perception.

Marked @pytest.mark.integration. Spawns the LLM gateway node in-process with a
stubbed LLM backend that emits a fixed ReAct trace and a stubbed perception
service that returns a single red_block detection.
"""

from __future__ import annotations

import pytest


pytestmark = pytest.mark.integration


def test_pick_red_block_uses_at_most_5_iterations(monkeypatch):
    from llm_gateway.react_planner import IterationBudget
    from llm_gateway import llm_gateway_node as mod

    # 1. Set up fake LLM trace.
    fake_react_trace = [
        # iter 1: query_perception
        '{"tool": "query_perception", "args": {"class_filter": "red_block"}}',
        # iter 2: pick_object
        '{"tool": "pick_object", "args": {'
        '"object_id": "red_block", "approach_offset_m": 0.05, '
        '"grasp_descent_m": 0.04, "lift_m": 0.06, "approach_axis": "-z_tool"}}',
        # iter 3: handoff
        '{"handoff": "success", "reason": "pick complete"}',
    ]

    def fake_llm_call(prompt, history):
        return fake_react_trace[len(history)]

    # 2. Boot the node with stubbed dependencies.
    node = mod.LLMGatewayNode.for_test(
        llm_call=fake_llm_call,
        perception_payload={
            "ok": True,
            "payload": {
                "detections": [
                    {
                        "class_id": "red_block",
                        "description": "red block at x=0.4 y=0.05 z=0.2",
                        "pose": {
                            "position": {"x": 0.40, "y": 0.05, "z": 0.20},
                            "orientation": {
                                "x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0,
                            },
                            "frame_id": "base_link",
                        },
                    }
                ],
                "count": 1,
            },
        },
        budget=IterationBudget(max_total=5, max_motion=2, wall_clock_timeout_s=5.0),
    )

    outcome = node.run_react_turn("pick up the red block")

    assert outcome["status"] == "success"
    assert outcome["iterations_used"] <= 3
    submitted = outcome["submitted_ir_history"]
    assert any(ir["intent"] == "sequence" for ir in submitted)
    # cache hit rate: with only one perception call needed, hit-rate condition
    # in the spec acceptance criteria is trivially satisfied; assert no
    # duplicate live perception fetches.
    assert outcome["perception_live_calls"] == 1
```

- [ ] **Step 2: If `for_test` / `run_react_turn` helpers don't exist, add them**

In `src/llm_gateway/llm_gateway/llm_gateway_node.py`, add a classmethod that constructs a node bypassing `rclpy.Node.__init__` for unit/integration use. Search for similar patterns already used by `test_react_gateway_pipeline.py` first; reuse what exists. If nothing reusable exists, add:

```python
    @classmethod
    def for_test(
        cls,
        *,
        llm_call,
        perception_payload,
        budget,
    ):
        instance = cls.__new__(cls)
        instance._composite_tools_enabled = True
        instance.scene_cache = __import__(
            "llm_gateway.scene_cache", fromlist=["SceneSnapshotCache"]
        ).SceneSnapshotCache(ttl_seconds=2.0)
        instance._react_plan_cache = {}
        instance._perception_payload = perception_payload
        instance._perception_live_calls = 0
        instance._llm_call = llm_call
        instance._submitted_ir_history = []
        instance._tool_registry = cls._build_tool_registry(instance)
        instance._budget = budget
        return instance

    def _query_perception_detections(self, args):
        self._perception_live_calls += 1
        return self._perception_payload

    def submit_semantic_ir(self, ir):
        # Test mode: skip /validate_command (covered by other tests) and
        # record the IR so the test can inspect it.
        self._submitted_ir_history.append(ir)
        cache = getattr(self, "scene_cache", None)
        if cache and ir.get("metadata", {}).get("tool_changed_world"):
            cache.on_motion_complete(tool_changed_world=True)
        return {"ok": True, "plan_id": f"p-{len(self._submitted_ir_history):04d}"}

    def run_react_turn(self, user_text: str) -> dict:
        # Loops the fake llm_call until handoff or budget exceeded. Reuse the
        # existing ReActAgent if possible; this minimal driver only exists for
        # the integration test to avoid pulling in the full ROS stack.
        from llm_gateway.react_planner import ReActAgent, AgentContext, StateInjector

        agent = ReActAgent(
            tool_registry=self._tool_registry,
            llm_call=self._llm_call,
            budget=self._budget,
        )
        context = AgentContext(
            ros_node=self,
            state_injector=StateInjector(),
        )
        result = agent.run(user_text=user_text, context=context)
        return {
            "status": result.status,
            "iterations_used": result.iterations_used,
            "submitted_ir_history": self._submitted_ir_history,
            "perception_live_calls": self._perception_live_calls,
        }
```

If the `ReActAgent.run` signature in the codebase differs, adapt the call accordingly (look for the existing method around `react_planner.py:1900`). Do not invent new public surface; only add the thin `for_test` helper.

- [ ] **Step 3: Run the integration test**

Run: `cd src/llm_gateway && python -m pytest tests/test_pick_place_sim.py -v -m integration`
Expected: PASS, with `iterations_used <= 3` and `perception_live_calls == 1`.

- [ ] **Step 4: Commit**

```bash
git add src/llm_gateway/llm_gateway/llm_gateway_node.py \
        src/llm_gateway/tests/test_pick_place_sim.py
git commit -m "test(llm_gateway): pick_red_block sim driver + iteration-budget assertion"
```

---

## Task 12: Full suite + colcon smoke

- [ ] **Step 1: Run all touched package tests via colcon**

```bash
cd ~/gp4_ws
colcon test --packages-select llm_gateway safety --output-on-failure
colcon test-result --packages-select llm_gateway safety --verbose
```

Expected: zero failures. Address any regression before continuing.

- [ ] **Step 2: Build to confirm no syntax/typing breakage**

```bash
cd ~/gp4_ws
colcon build --packages-select interfaces llm_gateway safety motion_core primitives hw_adapter \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Expected: build green.

- [ ] **Step 3: Smoke-run gateway in dry mode (no hardware)**

```bash
source install/setup.bash
ros2 launch gp4_bringup llm_stack.launch.py 2>&1 | head -40
```

Expected: gateway boots, parameter `react.composite_tools_enabled` defaults to `true`, no exception on registry build. Stop with `Ctrl-C` after confirming.

- [ ] **Step 4: Commit any incidental fixes**

If steps 1–3 surface small fixes, commit them under a single message:

```bash
git add -p
git commit -m "fix(llm_gateway): post-integration cleanup for composite uplift"
```

---

## Task 13: Documentation updates

**Files:**
- Modify: `README.md` (Composite ReAct flows subsection under primitives)
- Modify: `.claude/rules/llm-gateway.md` (tool surface block)

- [ ] **Step 1: Update README**

Locate the "Primitives" / "Public commands" section in `README.md`. Add a subsection:

```markdown
### Composite ReAct flows (W9, sim-tested)

The LLM gateway exposes high-level ReAct tools that wrap the primitive layer:

- `approach_object`, `retreat` — single LIN move with axis-offset arithmetic.
- `pick_object`, `place_object` — emit a `sequence` IR: approach → io_set
  (gripper) → LIN descent → io_set → LIN lift / retreat.
- `emit_sequence` — submit a pre-built `sequence` IR after Semantic IR
  contract validation.
- `refresh_scene` — invalidate the perception snapshot cache.

All composite-emitted Semantic IR still flows through `/validate_command →
motion_core → hw_adapter`. Composite-flow caps live under
`composite_limits` in `src/safety/config/safety_rules.yaml`.

Disable composites with `react.composite_tools_enabled:=false` on the
gateway node (e.g., first hardware run).
```

- [ ] **Step 2: Update `.claude/rules/llm-gateway.md`**

Locate the rule file (or create only if it already exists per the file map in
the spec); add a one-line note after the tool list:

```markdown
- Composite tools (W9): approach_object, retreat, pick_object, place_object,
  emit_sequence, refresh_scene. Gated by `react.composite_tools_enabled`.
  Caps: see safety_rules.yaml#composite_limits.
```

Do not create new rule files — if `.claude/rules/llm-gateway.md` does not
exist, append the note to the existing `.claude/CLAUDE.md` under "Project-
Specific Rules" instead.

- [ ] **Step 3: Commit docs**

```bash
git add README.md .claude/rules/llm-gateway.md .claude/CLAUDE.md
git commit -m "docs: composite ReAct flows + safety caps reference"
```

---

## Acceptance Checklist (from spec §10)

- [ ] Composite tools registered in `ToolRegistry`; `colcon build` green (Task 12).
- [ ] Pick sim test completes in ≤ 5 ReAct iterations (Task 11).
- [ ] Perception cache hit rate ≥ 50% in a 5-iteration pick flow — sim test
      asserts `perception_live_calls == 1` for a 3-iteration flow (Task 11).
- [ ] All existing unit/integration tests still green (Task 12).
- [ ] Safety caps prevent composite IR exceeding configured limits
      (Tasks 4, 6, 7).
- [ ] No new file exceeding 800 lines (`composite_tools.py` ≈ 350 L).
- [ ] README and `.claude/rules/llm-gateway.md` updated (Task 13).

---

## Risks & Mitigations

- **Axis approximation** — `_AXIS_VECTOR` treats `-z_tool` and `-z_base` as
  `+Z_base` displacement. Valid for tabletop pick with wrist upright.
  Composite tools refuse other axes (whitelist enforced). Generalising to
  arbitrary tool orientations is deferred to a follow-up plan.
- **`submit_semantic_ir` wrapper** — Task 10 leans on existing private helpers
  (`_review_semantic_ir`, `_submit_reviewed_ir`). If their actual names differ
  in this branch, do not invent new ones — wire the wrapper to the methods
  that already implement review and submission in `llm_gateway_node.py`.
- **Cache invalidation race** — robot-mode invalidation is best-effort
  (hook may not be wired yet). The integration test in Task 11 still passes
  because `_perception_live_calls` only counts pre-cache fetches, but a
  follow-up to wire `on_robot_mode` into the existing robot-status callback
  is listed in the spec §5.1.
- **Sequence size caps** — `max_sequence_length=8` accommodates the longest
  composite (pick = 5 steps). Bumping this requires a safety review per
  [safety-first.md](../../.claude/rules/safety-first.md).
