"""Tests for ReAct iteration budget tiering."""

from llm_gateway.react_planner import IterationBudget, IterationCounters
from llm_gateway.react_planner import Tool


class FakeReadonlyTool(Tool):
    name = "readonly_tool"
    is_readonly = True


class FakeMotionTool(Tool):
    name = "motion_tool"
    is_motion = True


class FakeComboTool(Tool):
    name = "combo_tool"
    is_readonly = True
    is_motion = True


def test_counters_start_at_zero():
    c = IterationCounters()
    assert c.total == 0
    assert c.motion == 0
    assert c.readonly == 0
    assert c.repair == 0


def test_can_invoke_readonly_within_budget():
    budget = IterationBudget(max_total=5, max_readonly=3)
    c = IterationCounters()
    t = FakeReadonlyTool()
    ok, reason = c.can_invoke(t, budget)
    assert ok is True
    assert reason == ""


def test_readonly_exhausted():
    budget = IterationBudget(max_total=5, max_readonly=2)
    c = IterationCounters()
    t = FakeReadonlyTool()
    c.record(t)
    c.record(t)
    ok, reason = c.can_invoke(t, budget)
    assert ok is False
    assert "max_readonly exceeded" in reason


def test_motion_exhausted():
    budget = IterationBudget(max_total=5, max_motion=1)
    c = IterationCounters()
    t = FakeMotionTool()
    c.record(t)
    ok, reason = c.can_invoke(t, budget)
    assert ok is False
    assert "max_motion exceeded" in reason


def test_total_exhausted():
    budget = IterationBudget(max_total=2)
    c = IterationCounters()
    t = FakeReadonlyTool()
    c.record(t)
    c.record(t)
    ok, reason = c.can_invoke(t, budget)
    assert ok is False
    assert "max_total exceeded" in reason


def test_combo_tool_counts_both():
    c = IterationCounters()
    t = FakeComboTool()
    c.record(t)
    assert c.total == 1
    assert c.motion == 1
    assert c.readonly == 1
