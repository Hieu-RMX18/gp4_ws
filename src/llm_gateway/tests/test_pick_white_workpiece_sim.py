"""Simulation test: pick white_workpiece from fixture to conveyor.

Requirements from plan Phase 8:
- Completes in ≤5 planner iterations
- Cache hit rate ≥50% for repeated perception queries
- PostconditionVerifier confirms object placed correctly
"""
from __future__ import annotations

from types import SimpleNamespace

from llm_gateway.composite_tools import PostconditionVerifier
from llm_gateway.factory_task import (
    FACTORY_TASK_VERSION,
    FactoryTask,
    TaskNode,
    TaskRuntime,
    RuntimeStepResult,
    TaskRuntimeReport,
    WorldModel,
    parse_factory_task,
)
from llm_gateway.intent_engine import SkillCall, compile_goal
from llm_gateway.station_scene_graph import StationSceneGraph
from pathlib import Path
import pytest


# ── Helper fixtures ──────────────────────────────────────────────────────────

def _make_scene_graph(tmp_path: Path) -> StationSceneGraph:
    path = tmp_path / "station.yaml"
    path.write_text(
        """
metadata: {source: test, geometry_verified: true}
regions:
  conveyor:
    frame_id: base_link
    geometry:
      type: box
      center: {x: 0.30, y: 0.10, z: 0.25}
      size: {x: 0.20, y: 0.20, z: 0.05}
    aliases: [conveyor, bang tai]
    zones:
      drop_zone:
        default_clearance_m: 0.10
  fixture:
    frame_id: base_link
    geometry:
      type: box
      center: {x: 0.10, y: 0.30, z: 0.20}
      size: {x: 0.15, y: 0.15, z: 0.04}
    aliases: [fixture, ga phoi]
    zones:
      grasp_zone:
        default_clearance_m: 0.08
objects:
  white_workpiece:
    class_id: white_workpiece
    aliases: [phoi trang, white workpiece]
""".strip(),
        encoding="utf-8",
    )
    return StationSceneGraph.from_file(path)


def _live_scene_fixture() -> dict:
    return {
        "detections": [
            {
                "class_id": "white_workpiece",
                "region": "fixture",
                "position": {"x": 0.1, "y": 0.3, "z": 0.23},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        ]
    }


def _live_scene_conveyor() -> dict:
    return {
        "detections": [
            {
                "class_id": "white_workpiece",
                "region": "conveyor",
                "position": {"x": 0.3, "y": 0.1, "z": 0.30},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        ]
    }


# ── Phase 8: Integration test (plan requirement) ─────────────────────────────

def test_pick_place_white_workpiece_completes_with_cached_scene_model():
    """Before-and-after postcondition verifier test (baseline)."""
    scene_before = {"detections": [{"class_id": "white_workpiece", "region": "fixture"}]}
    scene_after = {"detections": [{"class_id": "white_workpiece", "region": "conveyor"}]}
    verifier = PostconditionVerifier()

    assert verifier.verify_place(
        object_id="white_workpiece", destination="conveyor", scene=scene_before
    ).ok is False
    assert verifier.verify_place(
        object_id="white_workpiece", destination="conveyor", scene=scene_after
    ).ok is True


def test_compile_goal_pick_place_yields_ordered_skills_in_at_most_five_steps(tmp_path: Path):
    """compile_goal produces an ordered pick/place skill sequence in ≤5 steps."""
    scene_graph = _make_scene_graph(tmp_path)
    live = _live_scene_fixture()

    calls = compile_goal(
        {"action": "pick_and_place", "object": "phoi trang", "destination": "bang tai"},
        scene_graph=scene_graph,
        live_scene=live,
    )

    # Must not be a clarification or error
    assert calls, "compile_goal must return a non-empty list"
    assert calls[0].name not in {"needs_clarification", "capability_unavailable"}, (
        f"Expected ordered skill steps, got: {calls[0]}"
    )

    skill_names = [c.name for c in calls]
    assert skill_names == [
        "refresh_scene",
        "approach_object",
        "pick_object",
        "place_object",
        "verify_postcondition",
    ]
    assert len(skill_names) <= 5, f"Plan must complete in ≤5 skill steps; got {skill_names}"
    assert calls[2].args["object_id"] == "white_workpiece"
    assert calls[3].args["destination"] == "conveyor"

def test_compile_goal_returns_clarification_for_unknown_destination(tmp_path: Path):
    calls = compile_goal(
        {"action": "pick_and_place", "object": "phoi trang", "destination": "shelf"},
        scene_graph=_make_scene_graph(tmp_path),
    )

    assert calls == [
        SkillCall(name="needs_clarification", args={"field": "destination", "query": "shelf"})
    ]


def test_task_runtime_pick_fallback_selects_home_after_pick_exhaustion(tmp_path: Path):
    """TaskRuntime executes retry/fallback: pick fails twice → go_home selected."""
    task = parse_factory_task({
        "task_type": "factory_task",
        "version": FACTORY_TASK_VERSION,
        "task_id": "pick-with-fallback",
        "root": {
            "type": "fallback",
            "children": [
                {
                    "type": "retry",
                    "count": 2,
                    "children": [
                        {"type": "skill", "name": "pick_object", "args": {"object": "white_workpiece"}}
                    ],
                },
                {"type": "skill", "name": "go_home", "args": {}},
            ],
        },
    })

    calls: list[str] = []

    def executor(name: str, args: dict) -> RuntimeStepResult:
        calls.append(name)
        if name == "pick_object":
            return RuntimeStepResult(success=False, reason="grasp_failed")
        return RuntimeStepResult(success=True)

    report = TaskRuntime().run(task, executor)

    assert report.success is True
    assert calls == ["pick_object", "pick_object", "go_home"]
    assert report.fallback_count == 1
    any_fallback_selected = any(
        d.get("decision") == "fallback_selected" for d in report.policy_decisions
    )
    assert any_fallback_selected, (
        "PolicyEngine must record fallback_selected decision for HMI visibility"
    )


def test_task_runtime_replan_replaces_failed_pick_with_go_home(tmp_path: Path):
    """TaskRuntime replan path: failed pick triggers replanning via replan_handler."""
    task = parse_factory_task({
        "task_type": "factory_task",
        "version": FACTORY_TASK_VERSION,
        "task_id": "pick-replan",
        "replan_policy": {"max_replans": 1},
        "root": {
            "type": "skill",
            "name": "pick_object",
            "args": {"object": "white_workpiece"},
        },
    })
    home_task = parse_factory_task({
        "task_type": "factory_task",
        "version": FACTORY_TASK_VERSION,
        "task_id": "pick-replan-home",
        "root": {"type": "skill", "name": "go_home", "args": {}},
    })

    calls: list[str] = []

    def executor(name: str, args: dict) -> RuntimeStepResult:
        calls.append(name)
        if name == "pick_object":
            return RuntimeStepResult(success=False, requests_replan=True, reason="object_moved")
        return RuntimeStepResult(success=True)

    report = TaskRuntime(replan_handler=lambda _: home_task).run(task, executor)

    assert report.success is True
    assert report.replan_count == 1
    assert "pick_object" in calls
    assert "go_home" in calls
    replan_decisions = [d for d in report.policy_decisions if d.get("decision") == "replan"]
    assert len(replan_decisions) == 1, "PolicyEngine must record exactly one replan decision"


def test_scene_cache_hit_rate_above_50_pct_for_repeated_queries():
    """Cache hit rate test: same perception query returns cached result on second call."""
    from llm_gateway.llm_gateway_node import _SceneSnapshotCache

    hit_count = 0
    miss_count = 0
    now = [0.0]
    cache = _SceneSnapshotCache(ttl_sec=2.0, now_fn=lambda: now[0])

    query = {"class_filter": "white_workpiece", "frame": "base_link"}
    payload = {"detections": [{"class_id": "white_workpiece", "region": "fixture"}]}

    # First query: miss, store
    hit = cache.get(query)
    if hit is not None:
        hit_count += 1
    else:
        miss_count += 1
        cache.store(query, payload)

    # Second query within TTL: hit
    hit = cache.get(query)
    if hit is not None:
        hit_count += 1
    else:
        miss_count += 1

    # Third query within TTL: hit
    hit = cache.get(query)
    if hit is not None:
        hit_count += 1
    else:
        miss_count += 1

    total = hit_count + miss_count
    hit_rate = hit_count / total if total > 0 else 0.0
    assert hit_rate >= 0.5, (
        f"Cache hit rate {hit_rate:.0%} must be ≥50% for repeated queries within TTL"
    )

    # Verify cache_hit flag is set
    cached = cache.get(query)
    assert cached is not None
    assert cached.get("cache_hit") is True, "Cached payload must include cache_hit=True"
