"""Integration tests for IntentResolutionService using the ROS RPC pathway.

Verifies that _hydrate_draw_workplane correctly delegates to the ROS adapter's
hydrate_workplane method when available, and falls back to local logic otherwise.
"""

from __future__ import annotations

import json
import unittest
from unittest.mock import MagicMock

from hmi.backend.services.intent_resolution import (
    IntentResolutionError,
    IntentResolutionService,
)


class _MockRosAdapter:
    """Minimal mock ROS adapter for testing the RPC pathway."""

    def __init__(self, *, hydrate_result: dict | None = None) -> None:
        self.hydrate_calls: list[str] = []
        self._hydrate_result = hydrate_result

    def hydrate_workplane(self, *, payload_json: str) -> dict:
        self.hydrate_calls.append(payload_json)
        if self._hydrate_result is not None:
            return dict(self._hydrate_result)
        # Default: hydrate by adding origin to the input payload
        payload = json.loads(payload_json)
        if isinstance(payload.get("workplane"), dict):
            payload["workplane"] = dict(payload["workplane"])
            payload["workplane"]["origin"] = {
                "position": {"x": 0.5, "y": 0.1, "z": 0.3},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        return {
            "success": True,
            "hydrated_payload_json": json.dumps(payload),
        }


class TestIntentResolutionViaRpc(unittest.TestCase):
    """Test that IntentResolutionService uses the ROS RPC pathway."""

    def setUp(self) -> None:
        self.mock_ros = _MockRosAdapter()
        self.service = IntentResolutionService(ros_adapter=self.mock_ros)

    def test_hydrate_draw_workplane_calls_ros_adapter(self) -> None:
        """_hydrate_draw_workplane should call ros_adapter.hydrate_workplane."""
        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {"mode": "tool"},
            "params": {"radius": 50},
        }

        result = self.service._hydrate_draw_workplane(
            payload, current_pose_loader=None
        )

        self.assertEqual(len(self.mock_ros.hydrate_calls), 1)
        self.assertEqual(result["workplane"]["mode"], "tool")
        self.assertIsInstance(result["workplane"]["origin"], dict)

    def test_hydrate_non_draw_intent_skips_rpc(self) -> None:
        """Non-draw intents should skip the RPC call entirely."""
        payload = {"intent": "go_home"}

        result = self.service._hydrate_draw_workplane(
            payload, current_pose_loader=None
        )

        self.assertEqual(len(self.mock_ros.hydrate_calls), 0)
        self.assertEqual(result["intent"], "go_home")

    def test_hydrate_rpc_failure_raises_error(self) -> None:
        """RPC failure should raise IntentResolutionError."""
        ros = _MockRosAdapter(
            hydrate_result={"success": False, "error": "pose service unavailable"}
        )
        svc = IntentResolutionService(ros_adapter=ros)
        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {"mode": "tool"},
            "params": {"radius": 50},
        }

        with self.assertRaises(IntentResolutionError) as ctx:
            svc._hydrate_draw_workplane(payload, current_pose_loader=None)

        self.assertIn("workplane hydration failed", str(ctx.exception))

    def test_hydrate_falls_back_without_adapter(self) -> None:
        """Without ROS adapter, should use local fallback logic."""
        svc = IntentResolutionService(ros_adapter=None)
        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {"mode": "base"},
            "params": {"radius": 50},
        }

        result = svc._hydrate_draw_workplane(payload, current_pose_loader=None)

        self.assertEqual(result["workplane"]["mode"], "base")

    def test_hydrate_fallback_tool_mode_needs_pose(self) -> None:
        """Fallback tool mode without pose loader should raise error."""
        svc = IntentResolutionService(ros_adapter=None)
        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {"mode": "tool"},
            "params": {"radius": 50},
        }

        with self.assertRaises(IntentResolutionError) as ctx:
            svc._hydrate_draw_workplane(payload, current_pose_loader=None)

        self.assertIn("missing_workplane", str(ctx.exception))

    def test_hydrate_fallback_tool_mode_with_pose_loader(self) -> None:
        """Fallback tool mode with pose loader should hydrate locally."""
        svc = IntentResolutionService(ros_adapter=None)
        mock_pose = {
            "position": {"x": 0.6, "y": 0.2, "z": 0.4},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

        def pose_loader():
            return mock_pose

        payload = {
            "intent": "draw_shape",
            "shape_type": "circle",
            "workplane": {"mode": "tool"},
            "params": {"radius": 50},
        }

        result = svc._hydrate_draw_workplane(
            payload, current_pose_loader=pose_loader
        )

        self.assertEqual(result["workplane"]["origin"]["position"]["x"], 0.6)

    def test_rpc_roundtrip_preserves_payload_structure(self) -> None:
        """RPC roundtrip should preserve all non-workplane fields."""
        payload = {
            "intent": "draw_text",
            "text": "ABC",
            "units": "mm",
            "frame_id": "base_link",
            "workplane": {"mode": "tool"},
            "font": {"type": "single_stroke_builtin", "height": 20},
        }

        result = self.service._hydrate_draw_workplane(
            payload, current_pose_loader=None
        )

        self.assertEqual(result["intent"], "draw_text")
        self.assertEqual(result["text"], "ABC")
        self.assertEqual(result["units"], "mm")
        self.assertEqual(result["font"]["height"], 20)


class TestAdapterReadiness(unittest.TestCase):
    """Test that adapter readiness tracks new W5 services."""

    def test_telemetry_state_has_w5_fields(self) -> None:
        """_TelemetryState should include W5 readiness fields."""
        from hmi.backend.ros.telemetry_snapshot import _TelemetryState

        state = _TelemetryState()
        self.assertFalse(state.hydrate_workplane_ready)
        self.assertFalse(state.get_primitive_constants_ready)
        self.assertFalse(state.confirm_execution_ready)
        self.assertIsNone(state.hydrate_workplane_ready_at)
        self.assertIsNone(state.get_primitive_constants_ready_at)
        self.assertIsNone(state.confirm_execution_ready_at)


if __name__ == "__main__":
    unittest.main()
