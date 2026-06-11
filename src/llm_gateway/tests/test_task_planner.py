"""Tests for single-shot task_planner module."""

from __future__ import annotations

import importlib
import sys

from llm_gateway.task_planner import StateInjector, TaskPlanner


class _FakeParser:
    def parse(self, text: str):
        if text == "BAD_JSON":
            raise ValueError("invalid JSON")
        import json

        return json.loads(text)


class _FakeLLMClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.messages = []
        self._system_prompt = "system prompt"

    def generate_response_from_messages(self, messages):
        self.messages.append(messages)
        if not self.responses:
            raise AssertionError("unexpected LLM call")
        return self.responses.pop(0)


class _FakeSchemaValidator:
    schema = {"type": "object"}


_FACTORY_TASK_JSON = """
{
  "task_type": "factory_task",
  "version": "1.0",
  "task_id": "home",
  "root": {"type": "skill", "name": "go_home", "args": {}}
}
"""


def _planner(responses, state_injector=None, max_repair=1):
    return TaskPlanner(
        llm_client=_FakeLLMClient(responses),
        state_injector=state_injector or StateInjector(),
        schema_validator=_FakeSchemaValidator(),
        payload_parser=_FakeParser(),
        max_repair=max_repair,
    )


def test_valid_factory_task_json_returns_payload():
    planner = _planner([_FACTORY_TASK_JSON])

    result = planner.plan("về nhà")

    assert result["task_type"] == "factory_task"
    assert result["root"]["name"] == "go_home"


def test_missing_slot_error_passthrough():
    planner = _planner([
        '{"error":"MISSING_SLOT","missing_fields":["distance"],"hint":"distance needed"}'
    ])

    result = planner.plan("hạ xuống")

    assert result == {
        "error": "MISSING_SLOT",
        "missing_fields": ["distance"],
        "hint": "distance needed",
    }


def test_invalid_json_retries_then_returns_unsupported_error():
    planner = _planner(["BAD_JSON", "BAD_JSON"], max_repair=1)

    result = planner.plan("nonsense")

    assert result["error"] == "UNSUPPORTED_OR_AMBIGUOUS_COMMAND"
    assert "invalid JSON" in result["message"]


def test_state_context_is_included_in_messages():
    state = StateInjector()
    state.update_joint_states({"position": [1, 2, 3, 4, 5, 6]})
    state.set_available_named_poses(["ready", "home"])
    client = _FakeLLMClient([_FACTORY_TASK_JSON])
    planner = TaskPlanner(
        llm_client=client,
        state_injector=state,
        schema_validator=_FakeSchemaValidator(),
        payload_parser=_FakeParser(),
    )

    planner.plan("go home")

    user_message = client.messages[0][1]["content"]
    assert "Current robot/world state JSON" in user_message
    assert "Operator command:" in user_message
    assert "go home" in user_message
    assert "joint_1_s" in user_message
    assert "ready" in user_message


def test_task_planner_imports_without_rclpy(monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)
    module = importlib.import_module("llm_gateway.task_planner")

    assert hasattr(module, "TaskPlanner")
