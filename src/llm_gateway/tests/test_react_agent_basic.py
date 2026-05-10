"""Tests for ReAct agent basic behaviour."""

import json
from unittest.mock import MagicMock


from llm_gateway.react_planner import ReActAgent
from llm_gateway.react_planner import IterationBudget
from llm_gateway.react_planner import StateInjector
from llm_gateway.react_planner import Tool, ToolRegistry, ToolResult


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


class ContextProbeTool(Tool):
    name = "probe_context"
    description = "Records the provided AgentContext."
    is_readonly = True
    input_schema = {"type": "object", "properties": {}}

    def __init__(self):
        self.seen_ros_node = None

    def invoke(self, args, context):
        self.seen_ros_node = context.ros_node
        return ToolResult(
            ok=True, payload={"saw_ros_node": self.seen_ros_node is not None}
        )


def _make_agent(responses, tools=None, schema_validator=None, ros_node=None):
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
    if schema_validator is None:
        schema_validator = MagicMock()
        schema_validator.validate_against_schema.return_value = (True, "")
    agent = ReActAgent(
        llm_client=client,
        tool_registry=registry,
        state_injector=StateInjector(),
        budget=budget,
        schema_validator=schema_validator,
        ros_node=ros_node,
    )
    return agent


def test_agent_returns_final_semantic_ir():
    agent = _make_agent([json.dumps({"intent": "go_home"})])
    result = agent.run("go home")
    assert result == {"intent": "go_home"}


def test_agent_accepts_intent_router_semantic_ir_without_primitive_schema():
    schema_validator = MagicMock()
    schema_validator.validate_against_schema.side_effect = AssertionError(
        "semantic IR must be routed before primitive schema validation"
    )
    agent = _make_agent(
        [json.dumps({"intent": "go_home"})],
        schema_validator=schema_validator,
    )
    result = agent.run("go home")
    assert result == {"intent": "go_home"}


def test_agent_extracts_semantic_ir_from_openai_chat_wrapper():
    response = {
        "choices": [
            {
                "message": {
                    "content": json.dumps({"intent": "go_home"}, separators=(",", ":"))
                }
            }
        ]
    }
    agent = _make_agent([json.dumps(response)])
    result = agent.run("go home")
    assert result == {"intent": "go_home"}


def test_agent_one_tool_call_then_final():
    agent = _make_agent(
        [
            json.dumps({"tool_call": "echo", "args": {"msg": "hello"}}),
            json.dumps({"intent": "go_home"}),
        ],
        tools=[EchoTool()],
    )
    result = agent.run("go home")
    assert result == {"intent": "go_home"}


def test_agent_extracts_tool_call_from_openai_chat_tool_wrapper():
    response = {
        "choices": [
            {
                "message": {
                    "tool_calls": [
                        {
                            "type": "function",
                            "function": {
                                "name": "echo",
                                "arguments": json.dumps({"msg": "hello"}),
                            },
                        }
                    ]
                }
            }
        ]
    }
    agent = _make_agent(
        [
            json.dumps(response),
            json.dumps({"intent": "go_home"}),
        ],
        tools=[EchoTool()],
    )
    result = agent.run("go home")
    assert result == {"intent": "go_home"}


def test_agent_extracts_tool_call_from_openai_chat_function_wrapper():
    response = {
        "choices": [
            {
                "message": {
                    "function_call": {
                        "name": "echo",
                        "arguments": json.dumps({"msg": "hello"}),
                    }
                }
            }
        ]
    }
    agent = _make_agent(
        [
            json.dumps(response),
            json.dumps({"intent": "go_home"}),
        ],
        tools=[EchoTool()],
    )
    result = agent.run("go home")
    assert result == {"intent": "go_home"}


def test_agent_passes_ros_node_to_tool_context():
    tool = ContextProbeTool()
    ros_node = object()
    agent = _make_agent(
        [
            json.dumps({"tool_call": "probe_context", "args": {}}),
            json.dumps({"intent": "go_home"}),
        ],
        tools=[tool],
        ros_node=ros_node,
    )
    result = agent.run("probe")
    assert result == {"intent": "go_home"}
    assert tool.seen_ros_node is ros_node


def test_agent_unknown_tool_continues():
    agent = _make_agent(
        [
            json.dumps({"tool_call": "nonexistent", "args": {}}),
            json.dumps({"intent": "go_home"}),
        ]
    )
    result = agent.run("go home")
    assert result == {"intent": "go_home"}


def test_agent_budget_exceeded():
    responses = [
        json.dumps({"tool_call": "echo", "args": {"msg": "1"}}),
    ] * 10
    agent = _make_agent(responses, tools=[EchoTool()])
    result = agent.run("loop")
    assert result.get("_handoff") is True
    assert "max_total exceeded" in result.get("reason", "")


def test_agent_schema_invalid_then_repair():
    responses = [
        json.dumps({"primitive_type": "INVALID_TYPE"}),
        json.dumps({"intent": "go_home"}),
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
    result = agent.run("go home")
    assert result == {"intent": "go_home"}


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
    result = agent.run("loop")
    assert result.get("_handoff") is True
    assert "semantic_ir invalid after repair" in result.get("reason", "")


def test_agent_rejects_primitive_json_as_final_output():
    responses = [
        json.dumps({"primitive_type": "HOME"}),
        json.dumps({"intent": "go_home"}),
    ]
    agent = _make_agent(responses)
    result = agent.run("go home")
    assert result == {"intent": "go_home"}
