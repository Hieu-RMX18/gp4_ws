"""Unit tests for llm_gateway.command_pipeline — pure helpers.

These tests run without ROS2 because the helpers have no ROS dependencies.
"""

from __future__ import annotations

import json
from typing import Any, Dict, Optional

import pytest

from llm_gateway.command_pipeline import (
    command_from_sanitized_json,
    hydrate_draw_workplane,
    prepare_execution_command,
)


# ──────────────────────────────────────────────────────────────────────
# prepare_execution_command
# ──────────────────────────────────────────────────────────────────────


def test_plan_only_is_not_executable():
    cmd = {"primitive_type": "LIN", "plan_only": True, "require_approval": False}
    with pytest.raises(ValueError, match="plan_only_not_executable"):
        prepare_execution_command(cmd)
    # Source dict must not be mutated.
    assert cmd["require_approval"] is False


def test_direct_dispatch_always_clears_require_approval():
    cmd = {"primitive_type": "PTP", "require_approval": True}
    out = prepare_execution_command(cmd)
    assert out["require_approval"] is False
    # Source dict must not be mutated.
    assert cmd["require_approval"] is True


def test_dispatch_preserves_false_require_approval():
    cmd = {"primitive_type": "PTP", "require_approval": False}
    out = prepare_execution_command(cmd)
    assert out["require_approval"] is False


# ──────────────────────────────────────────────────────────────────────
# command_from_sanitized_json
# ──────────────────────────────────────────────────────────────────────


class _FakeSchemaValidator:
    def __init__(self, should_raise: bool = False):
        self.should_raise = should_raise
        self.last_validated: Dict[str, Any] | None = None

    def validate(self, command: Dict[str, Any]) -> None:
        self.last_validated = command
        if self.should_raise:
            raise ValueError("schema mismatch")


def test_empty_sanitized_json_returns_fallback():
    fallback = {"primitive_type": "HOME"}
    validator = _FakeSchemaValidator()
    out = command_from_sanitized_json("", fallback, validator)
    assert out is fallback
    assert validator.last_validated is None


def test_sanitized_json_decodes_and_validates():
    validator = _FakeSchemaValidator()
    payload = {"primitive_type": "PTP", "velocity_scale": 0.05}
    out = command_from_sanitized_json(json.dumps(payload), {}, validator)
    assert out == payload
    assert validator.last_validated == payload


def test_sanitized_json_non_object_rejected():
    validator = _FakeSchemaValidator()
    with pytest.raises(ValueError, match="JSON object"):
        command_from_sanitized_json("[]", {}, validator)


def test_sanitized_json_schema_failure_propagates():
    validator = _FakeSchemaValidator(should_raise=True)
    with pytest.raises(ValueError, match="schema mismatch"):
        command_from_sanitized_json('{"x": 1}', {}, validator)


def test_sanitized_json_invalid_json_raises():
    validator = _FakeSchemaValidator()
    with pytest.raises(json.JSONDecodeError):
        command_from_sanitized_json("{not json", {}, validator)


# ──────────────────────────────────────────────────────────────────────
# hydrate_draw_workplane
# ──────────────────────────────────────────────────────────────────────


def _pose(x: float = 0.3, y: float = 0.0, z: float = 0.3) -> Dict[str, Any]:
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def _noop_fetcher(_frame: str) -> Optional[Dict[str, Any]]:
    raise AssertionError("fetcher should not be called")


def test_hydrate_non_draw_intent_passthrough():
    payload = {"intent": "move_to_home"}
    out = hydrate_draw_workplane(payload, _noop_fetcher)
    assert out is payload


def test_hydrate_non_dict_passthrough():
    out = hydrate_draw_workplane("not a dict", _noop_fetcher)  # type: ignore[arg-type]
    assert out == "not a dict"


def test_hydrate_draw_shape_tool_mode_injects_current_pose():
    payload = {"intent": "draw_shape", "workplane": {"mode": "tool"}}
    fetched_pose = _pose()
    calls = []

    def fetch(frame: str) -> Optional[Dict[str, Any]]:
        calls.append(frame)
        return fetched_pose

    out = hydrate_draw_workplane(payload, fetch)
    assert out["workplane"]["origin"] == fetched_pose
    assert calls == ["base_link"]
    # Original must not be mutated.
    assert "origin" not in payload["workplane"]


def test_hydrate_draw_text_no_workplane_creates_tool_mode():
    """draw_text without any workplane defaults to tool mode + hydrates."""
    payload = {"intent": "draw_text"}
    fetched_pose = _pose()
    out = hydrate_draw_workplane(payload, lambda _f: fetched_pose)
    assert out["workplane"]["mode"] == "tool"
    assert out["workplane"]["origin"] == fetched_pose


def test_hydrate_base_mode_skips_fetch():
    payload = {"intent": "draw_shape", "workplane": {"mode": "base"}}
    out = hydrate_draw_workplane(payload, _noop_fetcher)
    assert out == payload


def test_hydrate_tool_mode_with_explicit_origin_skips_fetch():
    existing_origin = _pose(0.1, 0.2, 0.3)
    payload = {
        "intent": "draw_shape",
        "workplane": {"mode": "tool", "origin": existing_origin},
    }
    out = hydrate_draw_workplane(payload, _noop_fetcher)
    assert out["workplane"]["origin"] == existing_origin


def test_hydrate_start_pose_present_skips_fetch():
    payload = {
        "intent": "draw_shape",
        "workplane": {"mode": "tool"},
        "start_pose": _pose(),
    }
    out = hydrate_draw_workplane(payload, _noop_fetcher)
    assert "origin" not in out["workplane"]


def test_hydrate_missing_pose_service_raises():
    payload = {"intent": "draw_shape", "workplane": {"mode": "tool"}}
    with pytest.raises(ValueError, match="missing_workplane"):
        hydrate_draw_workplane(payload, lambda _f: None)
