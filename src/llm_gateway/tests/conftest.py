import json
from pathlib import Path

import pytest

from llm_gateway.normalizer import Normalizer
from llm_gateway.parser import LLMParser
from llm_gateway.schema_validator import SchemaValidator


@pytest.fixture(scope="session")
def schema_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "command_schema.json")


@pytest.fixture
def parser() -> LLMParser:
    return LLMParser()


@pytest.fixture
def validator(schema_path: str) -> SchemaValidator:
    return SchemaValidator(schema_path)


@pytest.fixture
def normalizer() -> Normalizer:
    return Normalizer(default_velocity_scale=0.1, default_acceleration_scale=0.1)


@pytest.fixture
def canonical_function_call() -> dict:
    return {
        "name": "execute_motion",
        "arguments": {
            "primitive_type": "LIN",
            "target_pose": {
                "position": {"x": 350.0, "y": 100.0, "z": 200.0},
                "orientation": {"roll": 0.0, "pitch": 90.0, "yaw": 180.0},
            },
            "velocity_scale": 0.2,
            "require_approval": False,
        },
    }


@pytest.fixture
def openai_payload(canonical_function_call: dict) -> str:
    payload = {
        "choices": [
            {
                "message": {
                    "tool_calls": [
                        {
                            "id": "call_001",
                            "type": "function",
                            "function": {
                                "name": canonical_function_call["name"],
                                "arguments": json.dumps(canonical_function_call["arguments"]),
                            },
                        }
                    ]
                }
            }
        ]
    }
    return json.dumps(payload)


@pytest.fixture
def anthropic_payload(canonical_function_call: dict) -> str:
    payload = {
        "content": [
            {
                "type": "tool_use",
                "id": "toolu_001",
                "name": canonical_function_call["name"],
                "input": canonical_function_call["arguments"],
            }
        ]
    }
    return json.dumps(payload)
