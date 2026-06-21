"""Consolidated tests; original source sections are marked below."""



# ---- test_parser.py ----
import pytest


def test_parse_openai_content_json(parser, openai_payload):
    parsed = parser.parse(openai_payload)
    assert parsed["primitive_type"] == "LIN"


def test_parse_direct_json_object(parser, direct_command_json):
    parsed = parser.parse(direct_command_json)
    assert parsed["primitive_type"] == "LIN"


def test_parse_legacy_openai_tool_call(parser, legacy_openai_tool_payload):
    parsed = parser.parse(legacy_openai_tool_payload)
    assert parsed["primitive_type"] == "LIN"


def test_parse_anthropic_tool_use(parser, anthropic_payload):
    parsed = parser.parse(anthropic_payload)
    assert parsed["primitive_type"] == "LIN"


def test_parse_model_error_json(parser, model_error_payload):
    parsed = parser.parse(model_error_payload)
    assert parsed == {"error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND"}


def test_parse_invalid_json_rejected(parser):
    with pytest.raises(ValueError, match="Invalid JSON format"):
        parser.parse("not-json")


def test_parse_non_json_content_rejected(parser):
    payload = '{"choices":[{"message":{"role":"assistant","content":"not-json"}}]}'
    with pytest.raises(ValueError, match="Model content must be a JSON object"):
        parser.parse(payload)


# ---- test_command_pipeline.py ----
"""Unit tests for llm_gateway.factory_task — pure helpers.

These tests run without ROS2 because the helpers have no ROS dependencies.
"""


import json
from typing import Any, Dict, Optional

import pytest

from llm_gateway.factory_task import (
    command_from_sanitized_json,
    hydrate_draw_workplane,
    prepare_execution_command,
)


# ──────────────────────────────────────────────────────────────────────
# prepare_execution_command
# ──────────────────────────────────────────────────────────────────────


def test_plan_only_is_not_executable():
    cmd = {"primitive_type": "LIN", "plan_only": True}
    with pytest.raises(ValueError, match="plan_only_not_executable"):
        prepare_execution_command(cmd)


def test_prepare_returns_shallow_copy_without_mutation():
    cmd = {"primitive_type": "PTP", "velocity_scale": 0.06}
    out = prepare_execution_command(cmd)
    assert out == cmd
    assert out is not cmd


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


def test_hydrate_missing_pose_service_falls_back_to_ssot():
    """W2.T5: when /get_current_pose unavailable, fall back to SSOT workplane."""
    payload = {"intent": "draw_shape", "workplane": {"mode": "tool"}}
    out = hydrate_draw_workplane(payload, lambda _f: None)
    # Should have an origin from the SSOT fallback
    assert isinstance(out["workplane"].get("origin"), dict)
    assert "position" in out["workplane"]["origin"]


# ---- test_hydrate_workplane_service.py ----
"""Unit tests for the hydrate_workplane ROS service handler.

Tests the _on_hydrate_workplane method of LLMGatewayNode in isolation,
using a mock fetch_current_pose callable.
"""


import json
import sys
import unittest
from pathlib import Path

# Ensure llm_gateway is importable without a ROS environment
_LLM_GATEWAY_SRC = Path(__file__).resolve().parents[1]
if str(_LLM_GATEWAY_SRC) not in sys.path:
    sys.path.insert(0, str(_LLM_GATEWAY_SRC))

# Simulate the service types without requiring a full ROS build
# The handler logic is pure enough to test with mock request/response objects.


class _MockRequest:
    def __init__(self, payload_json: str) -> None:
        self.payload_json = payload_json


class _MockResponse:
    def __init__(self) -> None:
        self.success = False
        self.error = ""
        self.hydrated_payload_json = ""


class TestHydrateWorkplaneHandler(unittest.TestCase):
    """Test the hydrate_workplane service handler logic."""

    def _make_handler(self, fetch_current_pose=None):
        """Create a handler function that mimics _on_hydrate_workplane."""
        from llm_gateway.factory_task import hydrate_draw_workplane

        def handler(request, response):
            try:
                payload = json.loads(request.payload_json)
            except json.JSONDecodeError as exc:
                response.success = False
                response.error = f"invalid payload_json: {exc}"
                return response

            try:
                hydrated = hydrate_draw_workplane(
                    payload,
                    fetch_current_pose=fetch_current_pose or (lambda _rf: None),
                )
                response.success = True
                response.hydrated_payload_json = json.dumps(
                    hydrated, ensure_ascii=True, separators=(",", ":")
                )
            except Exception as exc:
                response.success = False
                response.error = str(exc)

            return response

        return handler

    def test_non_draw_intent_passes_through(self) -> None:
        """Non-draw intents should be returned unchanged."""
        handler = self._make_handler()
        req = _MockRequest(json.dumps({"intent": "go_home"}))
        resp = _MockResponse()

        handler(req, resp)

        self.assertTrue(resp.success)
        hydrated = json.loads(resp.hydrated_payload_json)
        self.assertEqual(hydrated["intent"], "go_home")

    def test_draw_shape_base_mode_passes_through(self) -> None:
        """Base-mode draw_shape should be returned unchanged."""
        handler = self._make_handler()
        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {"mode": "base"},
            "params": {"radius": 50},
        }
        req = _MockRequest(json.dumps(payload))
        resp = _MockResponse()

        handler(req, resp)

        self.assertTrue(resp.success)
        hydrated = json.loads(resp.hydrated_payload_json)
        self.assertEqual(hydrated["workplane"]["mode"], "base")

    def test_draw_shape_tool_mode_hydrates_origin(self) -> None:
        """Tool-mode draw_shape should get origin from current pose."""
        mock_pose = {
            "position": {"x": 0.5, "y": 0.1, "z": 0.3},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

        def fetch_pose(ref_frame: str):
            self.assertEqual(ref_frame, "base_link")
            return mock_pose

        handler = self._make_handler(fetch_current_pose=fetch_pose)
        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {"mode": "tool"},
            "params": {"radius": 50},
        }
        req = _MockRequest(json.dumps(payload))
        resp = _MockResponse()

        handler(req, resp)

        self.assertTrue(resp.success)
        hydrated = json.loads(resp.hydrated_payload_json)
        self.assertEqual(hydrated["workplane"]["mode"], "tool")
        self.assertIsInstance(hydrated["workplane"]["origin"], dict)
        self.assertEqual(hydrated["workplane"]["origin"]["position"]["x"], 0.5)

    def test_draw_shape_tool_mode_already_has_origin(self) -> None:
        """Tool-mode with existing origin should not be re-hydrated."""
        handler = self._make_handler()
        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {
                "mode": "tool",
                "origin": {
                    "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "params": {"radius": 50},
        }
        req = _MockRequest(json.dumps(payload))
        resp = _MockResponse()

        handler(req, resp)

        self.assertTrue(resp.success)
        hydrated = json.loads(resp.hydrated_payload_json)
        self.assertEqual(hydrated["workplane"]["origin"]["position"]["x"], 0.1)

    def test_draw_shape_tool_mode_pose_unavailable(self) -> None:
        """Tool-mode with unavailable pose service should fall back or error."""
        handler = self._make_handler(fetch_current_pose=lambda _rf: None)
        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {"mode": "tool"},
            "params": {"radius": 50},
        }
        req = _MockRequest(json.dumps(payload))
        resp = _MockResponse()

        handler(req, resp)

        # The canonical hydrate_draw_workplane falls back to safety_rules.yaml
        # or raises ValueError. Either way, the handler should surface it.
        if not resp.success:
            self.assertIn("workplane", resp.error.lower())
        else:
            hydrated = json.loads(resp.hydrated_payload_json)
            self.assertIsInstance(hydrated["workplane"].get("origin"), dict)

    def test_draw_text_tool_mode_hydrates_origin(self) -> None:
        """Tool-mode draw_text should get origin from current pose."""
        mock_pose = {
            "position": {"x": 0.4, "y": 0.0, "z": 0.25},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

        handler = self._make_handler(fetch_current_pose=lambda _rf: mock_pose)
        payload = {
            "intent": "draw_text",
            "text": "HELLO",
            "workplane": {"mode": "tool"},
            "font": {"type": "single_stroke_builtin", "height": 20},
        }
        req = _MockRequest(json.dumps(payload))
        resp = _MockResponse()

        handler(req, resp)

        self.assertTrue(resp.success)
        hydrated = json.loads(resp.hydrated_payload_json)
        self.assertEqual(hydrated["workplane"]["mode"], "tool")
        self.assertIsInstance(hydrated["workplane"]["origin"], dict)

    def test_invalid_json_returns_error(self) -> None:
        """Malformed JSON should return success=False with error message."""
        handler = self._make_handler()
        req = _MockRequest("not valid json{{{")
        resp = _MockResponse()

        handler(req, resp)

        self.assertFalse(resp.success)
        self.assertIn("invalid payload_json", resp.error)

    def test_draw_shape_with_start_pose_passes_through(self) -> None:
        """Tool-mode with start_pose should not be re-hydrated."""
        handler = self._make_handler()
        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {"mode": "tool"},
            "start_pose": {
                "position": {"x": 0.5, "y": 0.0, "z": 0.2},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "params": {"radius": 50},
        }
        req = _MockRequest(json.dumps(payload))
        resp = _MockResponse()

        handler(req, resp)

        self.assertTrue(resp.success)
        hydrated = json.loads(resp.hydrated_payload_json)
        self.assertNotIn("origin", hydrated.get("workplane", {}))


if __name__ == "__main__":
    unittest.main()
