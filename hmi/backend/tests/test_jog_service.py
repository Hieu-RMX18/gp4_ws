from __future__ import annotations

import unittest

from hmi.backend.services.jog_pendant_service import JogPendantService
from hmi.backend.services.jog_service import (
    DEFAULT_JOG_STATUS,
    JogBridgeState,
    JogBridgeStatusView,
    JogService,
    _build_jog_status_event,
)


class JogServiceContractTests(unittest.TestCase):
    def test_compatibility_alias_keeps_legacy_import_path(self) -> None:
        self.assertIs(JogPendantService, JogService)

    def test_build_status_event_serializes_expected_payload(self) -> None:
        status = JogBridgeStatusView(
            state=JogBridgeState.ACTIVE,
            points_queued=3,
            effective_hz=42.5,
            robot_ready=True,
            servo_active=True,
            bridge_active=True,
            last_error="",
            rejection_reason="",
        )
        event = _build_jog_status_event(status)

        self.assertEqual(event["type"], "jog_bridge_status")
        payload = event["jogBridgeStatus"]
        self.assertEqual(payload["state"], "ACTIVE")
        self.assertEqual(payload["pointsQueued"], 3)
        self.assertAlmostEqual(payload["effectiveHz"], 42.5)
        self.assertTrue(payload["robotReady"])
        self.assertTrue(payload["servoActive"])
        self.assertTrue(payload["bridgeActive"])
        self.assertEqual(payload["lastError"], "")
        self.assertEqual(payload["rejectionReason"], "")

    def test_default_status_is_safe_idle(self) -> None:
        self.assertEqual(DEFAULT_JOG_STATUS.state, JogBridgeState.IDLE)
        self.assertFalse(DEFAULT_JOG_STATUS.bridge_active)
        self.assertFalse(DEFAULT_JOG_STATUS.robot_ready)
        self.assertFalse(DEFAULT_JOG_STATUS.servo_active)

    def test_activate_bridge_fails_closed_when_ros_node_is_unavailable(self) -> None:
        service = JogService()

        accepted, message = service.activate_bridge()

        self.assertFalse(accepted)
        self.assertIn('ROS node unavailable', message)

    def test_deactivate_bridge_fails_closed_when_ros_node_is_unavailable(self) -> None:
        service = JogService()

        accepted, message = service.deactivate_bridge()

        self.assertFalse(accepted)
        self.assertIn('ROS node unavailable', message)


if __name__ == "__main__":
    unittest.main()
