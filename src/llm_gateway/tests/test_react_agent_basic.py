"""Tests for ReAct agent basic behaviour."""

import json
from unittest.mock import MagicMock


from llm_gateway.react.agent import ReActAgent
from llm_gateway.react.iteration_budget import IterationBudget
from llm_gateway.react.state_injector import StateInjector
from llm_gateway.react.tool_registry import Tool, ToolRegistry, ToolResult


class FakeLLMClient:
    def __init__(self, responses):
        self._responses = iter(responses)

    def generate_response_from_messages(self, messages):
        return next(self._responses)


class EchoTool(Tool):
    name = "echo"
    description = "Echoes input."
    is_readonly = True
    input_schema = {"type": "object", "properties": {"msg": {"type": "string"}}}

    def invoke(self, args, context):
        return ToolResult(ok=True, payload={"echo": args.get("msg", "")})


def _make_agent(responses, tools=None):
    client = FakeLLMClient(responses)
    registry = ToolRegistry()
    if tools:
        for t in tools:
            registry.register(t)
    budget = IterationBudget(
        max_total=5,
        max_motion=3,
        max_readonly=10,
        max_repair=1,
        wall_clock_timeout_s=30,
    )
    schema_validator = MagicMock()
    schema_validator.validate_against_schema.return_value = (True, "")
    agent = ReActAgent(
        llm_client=client,
        tool_registry=registry,
        state_injector=StateInjector(),
        budget=budget,
        schema_validator=schema_validator,
    )
    return agent


def test_agent_returns_final_semantic_ir():
    agent = _make_agent([json.dumps({"primitive_type": "HOME"})])
    result = agent.run("go home", "req1")
    assert result == {"primitive_type": "HOME"}


def test_agent_one_tool_call_then_final():
    agent = _make_agent(
        [
            json.dumps({"tool_call": "echo", "args": {"msg": "hello"}}),
            json.dumps({"primitive_type": "HOME"}),
        ],
        tools=[EchoTool()],
    )
    result = agent.run("go home", "req1")
    assert result == {"primitive_type": "HOME"}


def test_agent_unknown_tool_continues():
    agent = _make_agent(
        [
            json.dumps({"tool_call": "nonexistent", "args": {}}),
            json.dumps({"primitive_type": "HOME"}),
        ]
    )
    result = agent.run("go home", "req1")
    assert result == {"primitive_type": "HOME"}


def test_agent_budget_exceeded():
    responses = [
        json.dumps({"tool_call": "echo", "args": {"msg": "1"}}),
    ] * 10
    agent = _make_agent(responses, tools=[EchoTool()])
    result = agent.run("loop", "req1")
    assert result.get("_handoff") is True
    assert "max_total exceeded" in result.get("reason", "")


def test_agent_schema_invalid_then_repair():
    responses = [
        json.dumps({"primitive_type": "INVALID_TYPE"}),
        json.dumps({"primitive_type": "HOME"}),
    ]
    schema_validator = MagicMock()
    schema_validator.validate_against_schema.side_effect = [
        (False, "bad schema"),
        (True, ""),
    ]
    client = FakeLLMClient(responses)
    budget = IterationBudget(
        max_total=5,
        max_motion=3,
        max_readonly=10,
        max_repair=1,
        wall_clock_timeout_s=30,
    )
    agent = ReActAgent(
        llm_client=client,
        tool_registry=ToolRegistry(),
        state_injector=StateInjector(),
        budget=budget,
        schema_validator=schema_validator,
    )
    result = agent.run("go home", "req1")
    assert result == {"primitive_type": "HOME"}


def test_agent_repair_exhausted_handoff():
    responses = [json.dumps({"primitive_type": "INVALID_TYPE"})] * 10
    schema_validator = MagicMock()
    schema_validator.validate_against_schema.return_value = (False, "bad schema")
    client = FakeLLMClient(responses)
    budget = IterationBudget(
        max_total=5,
        max_motion=3,
        max_readonly=10,
        max_repair=1,
        wall_clock_timeout_s=30,
    )
    agent = ReActAgent(
        llm_client=client,
        tool_registry=ToolRegistry(),
        state_injector=StateInjector(),
        budget=budget,
        schema_validator=schema_validator,
    )
    result = agent.run("loop", "req1")
    assert result.get("_handoff") is True
    assert "semantic_ir invalid after repair" in result.get("reason", "")
