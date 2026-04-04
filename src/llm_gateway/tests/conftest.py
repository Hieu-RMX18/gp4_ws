import json
from pathlib import Path

import pytest

from llm_gateway.normalizer import Normalizer
from llm_gateway.parser import LLMParser
from llm_gateway.semantic_validator import SemanticValidator
from llm_gateway.schema_validator import SchemaValidator


@pytest.fixture(scope="session")
def schema_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "llm_schema.yaml")


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
def semantic_validator() -> SemanticValidator:
    return SemanticValidator()


@pytest.fixture
def canonical_command() -> dict:
    return {
        "primitive_type": "LIN",
        "target_pose": {
            "position": {"x": 0.35, "y": 0.1, "z": 0.2},
            "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
        },
        "velocity_scale": 0.2,
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
