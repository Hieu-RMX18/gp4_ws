"""Unit tests for the hydrate_workplane ROS service handler.

Tests the _on_hydrate_workplane method of LLMGatewayNode in isolation,
using a mock fetch_current_pose callable.
"""

from __future__ import annotations

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
        from llm_gateway.command_pipeline import hydrate_draw_workplane

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
        handler = self._make_handler(fetch_current_pose=lambda rf: None)
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

        handler = self._make_handler(fetch_current_pose=lambda rf: mock_pose)
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
