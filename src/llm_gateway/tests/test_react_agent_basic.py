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


# ── Complex multi-step reasoning tests ──────────────────────────────


def test_agent_multi_step_sequence_a_b_home():
    """ReAct produces a sequence for 'move to A, then B, then home'."""
    semantic_ir = {
        "intent": "sequence",
        "steps": [
            {
                "intent": "absolute_move_ptp",
                "target_pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.4}},
                "reference_frame": "base_link",
            },
            {
                "intent": "absolute_move_ptp",
                "target_pose": {"position": {"x": 0.2, "y": -0.1, "z": 0.35}},
                "reference_frame": "base_link",
            },
            {"intent": "go_home"},
        ],
    }
    agent = _make_agent([json.dumps(semantic_ir)])
    result = agent.run("move to x=0.3 y=0.1 z=0.4, then x=0.2 y=-0.1 z=0.35, then home")
    assert result["intent"] == "sequence"
    assert len(result["steps"]) == 3
    assert result["steps"][0]["intent"] == "absolute_move_ptp"
    assert result["steps"][2]["intent"] == "go_home"


def test_agent_sequence_home_wait_move():
    """Sequence: go home, wait 2s, move forward 5cm."""
    semantic_ir = {
        "intent": "sequence",
        "steps": [
            {"intent": "go_home"},
            {"intent": "wait", "wait_duration_sec": 2.0},
            {
                "intent": "move_relative",
                "delta": {"x": 5.0, "y": 0.0, "z": 0.0},
                "linear_unit": "cm",
                "reference_frame": "base_link",
            },
        ],
    }
    agent = _make_agent([json.dumps(semantic_ir)])
    result = agent.run("go home, wait 2 seconds, then move forward 5cm")
    assert result["intent"] == "sequence"
    assert len(result["steps"]) == 3
    assert result["steps"][1]["intent"] == "wait"
    assert result["steps"][2]["intent"] == "move_relative"


def test_agent_perception_tool_then_motion():
    """ReAct queries perception, then produces motion to detected object."""

    class FakePerceptionTool(Tool):
        name = "query_perception"
        description = "Query perception for objects."
        is_readonly = True
        input_schema = {
            "type": "object",
            "properties": {"class_filter": {"type": "string"}},
        }

        def invoke(self, args, context):
            return ToolResult(
                ok=True,
                payload={
                    "detections": [
                        {
                            "class_id": "red_sphere",
                            "position": {"x": 0.35, "y": 0.05, "z": 0.30},
                            "frame_id": "base_link",
                        }
                    ],
                    "count": 1,
                },
            )

    responses = [
        json.dumps({"tool_call": "query_perception", "args": {"class_filter": "red"}}),
        json.dumps(
            {
                "intent": "absolute_move_ptp",
                "target_pose": {"position": {"x": 0.35, "y": 0.05, "z": 0.30}},
                "reference_frame": "base_link",
            }
        ),
    ]
    agent = _make_agent(responses, tools=[FakePerceptionTool()])
    result = agent.run("move to the red object")
    assert result["intent"] == "absolute_move_ptp"
    assert result["target_pose"]["position"]["x"] == 0.35


def test_agent_draw_circle_semantic_ir():
    """ReAct produces draw_shape for 'draw circle 50mm'."""
    semantic_ir = {
        "intent": "draw_shape",
        "shape_type": "circle",
        "units": "mm",
        "frame_id": "base_link",
        "workplane": {"mode": "tool"},
        "params": {"radius": 50},
    }
    agent = _make_agent([json.dumps(semantic_ir)])
    result = agent.run("draw circle radius 50mm")
    assert result["intent"] == "draw_shape"
    assert result["shape_type"] == "circle"
    assert result["params"]["radius"] == 50


def test_agent_arc_tool_then_circular_move():
    """ReAct uses compute_arc_points tool, then produces circular_move intent."""

    class FakeArcTool(Tool):
        name = "compute_arc_points"
        description = "Compute arc points."
        is_readonly = True
        input_schema = {
            "type": "object",
            "required": ["center", "radius_m", "start_angle_rad", "sweep_angle_rad", "plane_normal"],
            "properties": {
                "center": {"type": "object"},
                "radius_m": {"type": "number"},
                "start_angle_rad": {"type": "number"},
                "sweep_angle_rad": {"type": "number"},
                "plane_normal": {"type": "object"},
            },
        }

        def invoke(self, args, context):
            return ToolResult(
                ok=True,
                payload={
                    "start_pose": {"pose": {"position": {"x": 0.35, "y": 0.0, "z": 0.4}}},
                    "auxiliary_pose": {"pose": {"position": {"x": 0.3, "y": 0.05, "z": 0.4}}},
                    "target_pose": {"pose": {"position": {"x": 0.25, "y": 0.0, "z": 0.4}}},
                },
            )

    responses = [
        json.dumps(
            {
                "tool_call": "compute_arc_points",
                "args": {
                    "center": {"x": 0.3, "y": 0.0, "z": 0.4},
                    "radius_m": 0.05,
                    "start_angle_rad": 0.0,
                    "sweep_angle_rad": 3.14159,
                    "plane_normal": {"x": 0.0, "y": 0.0, "z": 1.0},
                },
            }
        ),
        json.dumps(
            {
                "intent": "circular_move",
                "target_pose": {"position": {"x": 0.25, "y": 0.0, "z": 0.4}},
                "auxiliary_pose": {"position": {"x": 0.3, "y": 0.05, "z": 0.4}},
                "reference_frame": "base_link",
            }
        ),
    ]
    agent = _make_agent(responses, tools=[FakeArcTool()])
    result = agent.run("draw a semicircular arc radius 50mm")
    assert result["intent"] == "circular_move"
    assert result["target_pose"]["position"]["x"] == 0.25


def test_agent_llm_error_produces_handoff():
    """When the LLM client raises, agent produces a handoff rather than crashing."""

    class FailingClient:
        def generate_response_from_messages(self, messages):
            raise RuntimeError("LLM backend unreachable")

    agent = ReActAgent(
        llm_client=FailingClient(),
        tool_registry=ToolRegistry(),
        state_injector=StateInjector(),
        budget=IterationBudget(),
        schema_validator=MagicMock(),
    )
    result = agent.run("anything")
    assert result.get("_handoff") is True
    assert "llm_request_failed" in result["reason"]
