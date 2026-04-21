"""Regression tests for refactor invariants.

These tests freeze the public contracts, HMI trust boundary, and launch
entrypoints that must survive all refactor waves unchanged.
"""
from __future__ import annotations

import importlib
import importlib.util
import os
import sys
import unittest
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[3]
_LLM_GATEWAY_SRC = _REPO_ROOT / "src" / "llm_gateway"
if str(_LLM_GATEWAY_SRC) not in sys.path:
    sys.path.insert(0, str(_LLM_GATEWAY_SRC))


class TestPackageImports(unittest.TestCase):
    """Verify critical modules remain importable at their current paths."""

    def test_llm_gateway_node_importable(self) -> None:
        spec = importlib.util.find_spec("llm_gateway.llm_gateway_node")
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.origin)
        source = Path(spec.origin).read_text(encoding="utf-8")
        self.assertIn("class LLMGatewayNode", source)
        self.assertIn("def main", source)

    def test_intent_router_importable(self) -> None:
        mod = importlib.import_module("llm_gateway.intent_router")
        self.assertTrue(hasattr(mod, "IntentRouter"))
        self.assertTrue(hasattr(mod, "RouteResult"))

    def test_supervisor_service_importable(self) -> None:
        mod = importlib.import_module("hmi.backend.services.supervisor_service")
        self.assertTrue(hasattr(mod, "SupervisorService"))

    def test_adapter_importable(self) -> None:
        mod = importlib.import_module("hmi.backend.ros.adapter")
        self.assertTrue(hasattr(mod, "WorkspaceRosAdapter"))

    def test_intent_resolution_importable(self) -> None:
        mod = importlib.import_module("hmi.backend.services.intent_resolution")
        self.assertTrue(hasattr(mod, "IntentResolutionService"))

    def test_jog_service_importable(self) -> None:
        mod = importlib.import_module("hmi.backend.services.jog_service")
        self.assertTrue(hasattr(mod, "JogService"))


class TestHmiTrustBoundary(unittest.TestCase):
    """Verify HMI safety boundary elements exist."""

    def test_hardware_gate_service_exists(self) -> None:
        mod = importlib.import_module("hmi.backend.services.hardware_gate")
        self.assertTrue(hasattr(mod, "HardwareGateEvaluator"))
        self.assertTrue(hasattr(mod, "HARDWARE_GATE_ENV"))

    def test_session_lock_service_exists(self) -> None:
        mod = importlib.import_module("hmi.backend.services.session_lock_service")
        self.assertTrue(hasattr(mod, "SessionLockService"))

    def test_plan_fingerprint_in_contracts(self) -> None:
        contracts = importlib.import_module("hmi.backend.api.contracts")
        model = getattr(contracts, "ConfirmCommandRequest", None)
        if model is None:
            model = getattr(contracts, "CommandConfirmRequestModel", None)
        self.assertIsNotNone(model)

        fields = getattr(model, "model_fields", None)
        if fields is not None:
            self.assertIn("planFingerprint", fields)
            return
        annotations = getattr(model, "__annotations__", {})
        self.assertIn("planFingerprint", annotations)

    def test_supervisor_has_plan_fingerprint_method(self) -> None:
        mod = importlib.import_module("hmi.backend.services.supervisor_service")
        self.assertTrue(hasattr(mod.SupervisorService, "_plan_fingerprint"))


class TestLaunchEntrypoints(unittest.TestCase):
    """Verify all frozen launch files exist on disk."""

    BRINGUP_LAUNCH = (
        Path(os.environ.get("GP4_WS", os.path.expanduser("~/gp4_ws")))
        / "src"
        / "gp4_bringup"
        / "launch"
    )

    FROZEN_LAUNCHES = [
        "sim.launch.py",
        "hw.launch.py",
        "system.launch.py",
        "moveit_only.launch.py",
        "llm_stack.launch.py",
    ]

    def test_frozen_launch_files_exist(self) -> None:
        for name in self.FROZEN_LAUNCHES:
            path = self.BRINGUP_LAUNCH / name
            self.assertTrue(path.exists(), f"Frozen launch entrypoint missing: {name}")


class TestJointNameContract(unittest.TestCase):
    """Verify the canonical 6-joint GP4 naming."""

    EXPECTED = (
        "joint_1_s",
        "joint_2_l",
        "joint_3_u",
        "joint_4_r",
        "joint_5_b",
        "joint_6_t",
    )

    def test_adapter_joint_names(self) -> None:
        from hmi.backend.ros.adapter import DEFAULT_JOINT_NAMES

        self.assertEqual(tuple(DEFAULT_JOINT_NAMES), self.EXPECTED)

    def test_intent_resolution_joint_names(self) -> None:
        from hmi.backend.services.intent_resolution import JOINT_NAMES

        self.assertEqual(tuple(JOINT_NAMES), self.EXPECTED)


class TestConstants(unittest.TestCase):
    """Verify the consolidated constants module."""

    def test_joint_names_tuple(self) -> None:
        from hmi.backend.domain.constants import GP4_JOINT_NAMES

        self.assertEqual(len(GP4_JOINT_NAMES), 6)
        self.assertEqual(GP4_JOINT_NAMES[0], "joint_1_s")
        self.assertEqual(GP4_JOINT_NAMES[5], "joint_6_t")
        self.assertIsInstance(GP4_JOINT_NAMES, tuple)

    def test_joint_count(self) -> None:
        from hmi.backend.domain.constants import GP4_JOINT_COUNT, GP4_JOINT_NAMES

        self.assertEqual(GP4_JOINT_COUNT, len(GP4_JOINT_NAMES))


if __name__ == "__main__":
    unittest.main()
