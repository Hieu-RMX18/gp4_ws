"""Shared test configuration and fixtures for the llm_gateway test suite.

Test Environment Tiers
──────────────────────
Tier 1 — Source-only / lightweight mode:
    Works with ``python3 -m pytest`` from the package root.
    No colcon workspace or built ROS interfaces required.
    rclpy must be importable (system-installed ros-humble-rclpy).

Tier 2 — Colcon / built-workspace mode:
    Requires ``source install/setup.bash`` so the custom ``interfaces``
    package is on PYTHONPATH.  Tests marked ``@pytest.mark.ros_integration``
    run only in this tier.

Skip policy:
    - Individual tests that need ``interfaces`` use
      ``pytest.importorskip("interfaces")`` and carry
      ``@pytest.mark.ros_integration`` for selective execution.
    - ``test_integration.py`` uses a module-level guard that skips the
      entire module when ``interfaces`` is unavailable.
    - No test should be silently excluded from collection.
"""

import json
import os
from pathlib import Path
import sys
import tempfile

import pytest


SAFETY_SOURCE_ROOT = Path(__file__).resolve().parents[2] / "safety"
if SAFETY_SOURCE_ROOT.exists() and str(SAFETY_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SAFETY_SOURCE_ROOT))


@pytest.fixture(scope="session")
def schema_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "llm_schema.yaml")


@pytest.fixture(scope="session")
def macro_policy_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


@pytest.fixture
def parser():
    from llm_gateway.factory_task import LLMParser

    return LLMParser()


@pytest.fixture
def validator(schema_path: str):
    from llm_gateway.factory_task import SchemaValidator

    return SchemaValidator(schema_path)


@pytest.fixture
def normalizer():
    from llm_gateway.factory_task import Normalizer

    return Normalizer(default_velocity_scale=0.06, default_acceleration_scale=0.06)


@pytest.fixture
def semantic_validator():
    from llm_gateway.factory_task import SemanticValidator

    return SemanticValidator()


@pytest.fixture(autouse=True)
def disable_planner_for_legacy_gateway_tests(request, monkeypatch):
    """Keep legacy gateway integration tests on the mocked single-shot LLM path."""
    if request.module.__name__.split(".")[-1] not in {
        "test_get_pose",
        "test_integration",
    }:
        return
    try:
        from llm_gateway.llm_gateway_node import LLMGatewayNode
    except ImportError:
        return
    monkeypatch.setattr(LLMGatewayNode, "_load_planner_enabled", lambda self: False)


@pytest.fixture
def canonical_command() -> dict:
    return {
        "primitive_type": "LIN",
        "target_pose": {
            "position": {"x": 0.30, "y": 0.0, "z": 0.30},
            "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
        },
        "velocity_scale": 0.06,
    }


@pytest.fixture
def direct_command_json(canonical_command: dict) -> str:
    return json.dumps(canonical_command)


@pytest.fixture
def openai_payload(canonical_command: dict) -> str:
    payload = {
        "choices": [
            {
                "message": {
                    "role": "assistant",
                    "content": json.dumps(canonical_command),
                }
            }
        ]
    }
    return json.dumps(payload)


@pytest.fixture
def legacy_openai_tool_payload(canonical_command: dict) -> str:
    payload = {
        "choices": [
            {
                "message": {
                    "tool_calls": [
                        {
                            "id": "call_001",
                            "type": "function",
                            "function": {
                                "name": "execute_motion",
                                "arguments": json.dumps(canonical_command),
                            },
                        }
                    ]
                }
            }
        ]
    }
    return json.dumps(payload)


@pytest.fixture
def anthropic_payload(canonical_command: dict) -> str:
    payload = {
        "content": [
            {
                "type": "tool_use",
                "id": "toolu_001",
                "name": "execute_motion",
                "input": canonical_command,
            }
        ]
    }
    return json.dumps(payload)


@pytest.fixture
def model_error_payload() -> str:
    payload = {
        "choices": [
            {
                "message": {
                    "role": "assistant",
                    "content": '{"error":"UNSUPPORTED_OR_AMBIGUOUS_COMMAND"}',
                }
            }
        ]
    }
    return json.dumps(payload)


@pytest.fixture(scope="module")
def ros_integration_context():
    pytest.importorskip(
        "interfaces", reason="requires colcon-sourced workspace with built interfaces"
    )
    import rclpy
    from interfaces.srv import ValidateCommand

    ros_log_dir = Path(tempfile.gettempdir()) / "ros_logs_llm_gateway_tests"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

    initialized_here = False
    if not rclpy.ok():
        rclpy.init()
        initialized_here = True

    probe_node = None
    try:
        probe_node = rclpy.create_node("llm_gateway_test_probe")
        probe_node.create_client(ValidateCommand, "/validate_command_probe")
    except Exception as exc:
        if probe_node is not None:
            probe_node.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()
        pytest.skip(f"ROS client type-support unavailable in this environment: {exc}")
    if probe_node is not None:
        probe_node.destroy_node()

    try:
        yield
    finally:
        if initialized_here and rclpy.ok():
            rclpy.shutdown()
