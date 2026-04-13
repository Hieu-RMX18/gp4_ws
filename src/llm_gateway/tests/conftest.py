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
from pathlib import Path

import pytest


@pytest.fixture(scope="session")
def schema_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "llm_schema.yaml")


@pytest.fixture(scope="session")
def macro_policy_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


@pytest.fixture
def parser():
    from llm_gateway.parser import LLMParser

    return LLMParser()


@pytest.fixture
def validator(schema_path: str):
    from llm_gateway.schema_validator import SchemaValidator

    return SchemaValidator(schema_path)


@pytest.fixture
def normalizer():
    from llm_gateway.normalizer import Normalizer

    return Normalizer(default_velocity_scale=0.06, default_acceleration_scale=0.06)


@pytest.fixture
def semantic_validator():
    from llm_gateway.semantic_validator import SemanticValidator

    return SemanticValidator()


@pytest.fixture
def canonical_command() -> dict:
    return {
        "primitive_type": "LIN",
        "target_pose": {
            "position": {"x": 0.35, "y": 0.1, "z": 0.2},
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
