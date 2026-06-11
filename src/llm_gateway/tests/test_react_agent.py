"""Consolidated tests; original source sections are marked below."""



# ---- test_react_agent_basic.py ----
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

def _skill_node(name, args=None):
    return {"type": "skill", "name": name, "args": dict(args or {})}

def _factory_task(task_id="home", root=None):
    return {
        "task_type": "factory_task",
        "version": "1.0",
        "task_id": task_id,
        "root": root or _skill_node("go_home"),
    }


def test_agent_rejects_final_semantic_ir_and_repairs_to_factory_task():
    task = _factory_task("home")
    agent = _make_agent([json.dumps({"intent": "go_home"}), json.dumps(task)])

    result = agent.run("go home")
    assert result == task

def test_agent_returns_final_factory_task():
    factory_task = {
        "task_type": "factory_task",
        "version": "1.0",
        "task_id": "home-wait",
        "root": {
            "type": "sequence",
            "children": [
                {"type": "skill", "name": "go_home", "args": {}},
                {"type": "skill", "name": "wait", "args": {"wait_duration_sec": 1.0}},
            ],
        },
    }
    agent = _make_agent([json.dumps(factory_task)])

    result = agent.run("go home then wait")

    assert result == factory_task

def test_react_prompt_describes_factory_task_output_contract():
    agent = _make_agent([json.dumps(_factory_task("home"))])

    messages = agent._build_prompt("go home", StateInjector().snapshot(), [])
    system_prompt = messages[0]["content"]

    assert "FactoryTask" in system_prompt
    assert '"task_type":"factory_task"' in system_prompt
    assert "Do not output final Semantic IR" in system_prompt

def test_react_prompt_examples_do_not_expose_final_semantic_ir():
    agent = _make_agent([json.dumps(_factory_task("home"))])

    messages = agent._build_prompt("draw a circle", StateInjector().snapshot(), [])
    system_prompt = messages[0]["content"]

    assert 'Assistant: {"intent":' not in system_prompt
    assert 'Assistant: {"intent":"draw_shape"' not in system_prompt
    assert 'Assistant: {"intent":"draw_text"' not in system_prompt
    assert '"intent": "get_pose"' not in system_prompt
    assert "error intent" not in system_prompt
    assert '"name":"draw_shape"' in system_prompt
    assert '"name":"draw_text"' in system_prompt


def test_react_prompt_defaults_unspecified_draw_text_height():
    agent = _make_agent([json.dumps(_factory_task("home"))])

    messages = agent._build_prompt("write GP4", StateInjector().snapshot(), [])
    system_prompt = messages[0]["content"]

    assert "default font.height=20" in system_prompt
    assert 'User: "write GP4"' in system_prompt
    assert '"name":"draw_text"' in system_prompt
    assert '"height":20' in system_prompt

def test_agent_rejects_intent_router_semantic_ir_without_primitive_schema():
    schema_validator = MagicMock()
    schema_validator.validate_against_schema.side_effect = AssertionError(
        "legacy primitive schema validator must not be used for FactoryTask final output"
    )
    agent = _make_agent(
        [json.dumps({"intent": "go_home"}), json.dumps({"intent": "go_home"})],
        schema_validator=schema_validator,
    )
    result = agent.run("go home")
    assert result.get("_handoff") is True
    assert "FactoryTask" in result.get("reason", "")


def test_agent_extracts_factory_task_from_openai_chat_wrapper():
    task = _factory_task("home")
    response = {
        "choices": [
            {
                "message": {
                    "content": json.dumps(task, separators=(",", ":"))
                }
            }
        ]
    }
    agent = _make_agent([json.dumps(response)])
    result = agent.run("go home")
    assert result == task


def test_agent_one_tool_call_then_final():
    task = _factory_task("home")
    agent = _make_agent(
        [
            json.dumps({"tool_call": "echo", "args": {"msg": "hello"}}),
            json.dumps(task),
        ],
        tools=[EchoTool()],
    )
    result = agent.run("go home")
    assert result == task


def test_agent_extracts_tool_call_from_openai_chat_tool_wrapper():
    task = _factory_task("home")
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
            json.dumps(task),
        ],
        tools=[EchoTool()],
    )
    result = agent.run("go home")
    assert result == task


def test_agent_extracts_tool_call_from_openai_chat_function_wrapper():
    task = _factory_task("home")
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
            json.dumps(task),
        ],
        tools=[EchoTool()],
    )
    result = agent.run("go home")
    assert result == task


def test_agent_passes_ros_node_to_tool_context():
    tool = ContextProbeTool()
    ros_node = object()
    task = _factory_task("home")
    agent = _make_agent(
        [
            json.dumps({"tool_call": "probe_context", "args": {}}),
            json.dumps(task),
        ],
        tools=[tool],
        ros_node=ros_node,
    )
    result = agent.run("probe")
    assert result == task
    assert tool.seen_ros_node is ros_node


def test_agent_unknown_tool_continues():
    task = _factory_task("home")
    agent = _make_agent(
        [
            json.dumps({"tool_call": "nonexistent", "args": {}}),
            json.dumps(task),
        ]
    )
    result = agent.run("go home")
    assert result == task


def test_agent_budget_exceeded():
    responses = [
        json.dumps({"tool_call": "echo", "args": {"msg": "1"}}),
    ] * 10
    agent = _make_agent(responses, tools=[EchoTool()])
    result = agent.run("loop")
    assert result.get("_handoff") is True
    assert "max_total exceeded" in result.get("reason", "")


def test_agent_schema_invalid_then_repair():
    task = _factory_task("home")
    responses = [
        json.dumps({"primitive_type": "INVALID_TYPE"}),
        json.dumps(task),
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
    assert result == task


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
    assert "final response invalid after repair" in result.get("reason", "")


def test_agent_rejects_primitive_json_as_final_output():
    task = _factory_task("home")
    responses = [
        json.dumps({"primitive_type": "HOME"}),
        json.dumps(task),
    ]
    agent = _make_agent(responses)
    result = agent.run("go home")
    assert result == task


# ── Complex multi-step reasoning tests ──────────────────────────────


def test_agent_multi_step_sequence_a_b_home():
    """ReAct produces a FactoryTask sequence for 'move to A, then B, then home'."""
    task = _factory_task(
        "move-a-b-home",
        {
            "type": "sequence",
            "children": [
                _skill_node(
                    "move_cartesian",
                    {
                        "target_pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.4}},
                        "reference_frame": "base_link",
                    },
                ),
                _skill_node(
                    "move_cartesian",
                    {
                        "target_pose": {"position": {"x": 0.2, "y": -0.1, "z": 0.35}},
                        "reference_frame": "base_link",
                    },
                ),
                _skill_node("go_home"),
            ],
        },
    )
    agent = _make_agent([json.dumps(task)])
    result = agent.run("move to x=0.3 y=0.1 z=0.4, then x=0.2 y=-0.1 z=0.35, then home")
    assert result == task
    assert len(result["root"]["children"]) == 3
    assert result["root"]["children"][0]["name"] == "move_cartesian"
    assert result["root"]["children"][2]["name"] == "go_home"


def test_agent_sequence_home_wait_move():
    """Sequence: go home, wait 2s, move forward 5cm."""
    task = _factory_task(
        "home-wait-forward",
        {
            "type": "sequence",
            "children": [
                _skill_node("go_home"),
                _skill_node("wait", {"wait_duration_sec": 2.0}),
                _skill_node(
                    "move_relative",
                    {
                        "delta": {"x": 5.0, "y": 0.0, "z": 0.0},
                        "linear_unit": "cm",
                        "reference_frame": "base_link",
                    },
                ),
            ],
        },
    )
    agent = _make_agent([json.dumps(task)])
    result = agent.run("go home, wait 2 seconds, then move forward 5cm")
    assert result == task
    assert result["root"]["children"][1]["name"] == "wait"
    assert result["root"]["children"][2]["name"] == "move_relative"


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
            _factory_task(
                "move-red-object",
                _skill_node(
                    "move_to_object",
                    {"object_ref": "red_sphere", "pose": "approach"},
                ),
            )
        ),
    ]
    agent = _make_agent(responses, tools=[FakePerceptionTool()])
    result = agent.run("move to the red object")
    assert result["task_type"] == "factory_task"
    assert result["root"]["name"] == "move_to_object"
    assert result["root"]["args"]["object_ref"] == "red_sphere"


def test_agent_draw_circle_factory_task():
    """ReAct produces a FactoryTask draw_shape skill for 'draw circle 50mm'."""
    task = _factory_task(
        "draw-circle",
        _skill_node(
            "draw_shape",
            {
                "shape_type": "circle",
                "units": "mm",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "params": {"radius": 50},
            },
        ),
    )
    agent = _make_agent([json.dumps(task)])
    result = agent.run("draw circle radius 50mm")
    assert result == task
    assert result["root"]["name"] == "draw_shape"
    assert result["root"]["args"]["params"]["radius"] == 50


def test_agent_arc_tool_then_factory_task_draw_shape():
    """ReAct can use a helper tool, then still produce FactoryTask output."""

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
            _factory_task(
                "draw-arc",
                _skill_node(
                    "draw_shape",
                    {
                        "shape_type": "arc",
                        "units": "m",
                        "frame_id": "base_link",
                        "workplane": {"mode": "tool"},
                        "params": {"radius": 0.05},
                    },
                ),
            )
        ),
    ]
    agent = _make_agent(responses, tools=[FakeArcTool()])
    result = agent.run("draw a semicircular arc radius 50mm")
    assert result["task_type"] == "factory_task"
    assert result["root"]["name"] == "draw_shape"
    assert result["root"]["args"]["shape_type"] == "arc"


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


# ---- test_react_iteration_budget.py ----
"""Tests for ReAct iteration budget tiering."""

from llm_gateway.react_planner import IterationBudget, IterationCounters
from llm_gateway.react_planner import Tool


class FakeReadonlyTool(Tool):
    name = "readonly_tool"
    is_readonly = True


class FakeMotionTool(Tool):
    name = "motion_tool"
    is_motion = True


class FakeComboTool(Tool):
    name = "combo_tool"
    is_readonly = True
    is_motion = True


def test_counters_start_at_zero():
    c = IterationCounters()
    assert c.total == 0
    assert c.motion == 0
    assert c.readonly == 0
    assert c.repair == 0


def test_can_invoke_readonly_within_budget():
    budget = IterationBudget(max_total=5, max_readonly=3)
    c = IterationCounters()
    t = FakeReadonlyTool()
    ok, reason = c.can_invoke(t, budget)
    assert ok is True
    assert reason == ""


def test_readonly_exhausted():
    budget = IterationBudget(max_total=5, max_readonly=2)
    c = IterationCounters()
    t = FakeReadonlyTool()
    c.record(t)
    c.record(t)
    ok, reason = c.can_invoke(t, budget)
    assert ok is False
    assert "max_readonly exceeded" in reason


def test_motion_exhausted():
    budget = IterationBudget(max_total=5, max_motion=1)
    c = IterationCounters()
    t = FakeMotionTool()
    c.record(t)
    ok, reason = c.can_invoke(t, budget)
    assert ok is False
    assert "max_motion exceeded" in reason


def test_total_exhausted():
    budget = IterationBudget(max_total=2)
    c = IterationCounters()
    t = FakeReadonlyTool()
    c.record(t)
    c.record(t)
    ok, reason = c.can_invoke(t, budget)
    assert ok is False
    assert "max_total exceeded" in reason


def test_combo_tool_counts_both():
    c = IterationCounters()
    t = FakeComboTool()
    c.record(t)
    assert c.total == 1
    assert c.motion == 1
    assert c.readonly == 1


# ---- test_react_state_injector.py ----
"""Tests for ReAct state injector."""

import ast
from pathlib import Path

from llm_gateway.react_planner import StateInjector


def _subscription_qos_arg(constructor: ast.FunctionDef, topic: str) -> ast.AST:
    for node in ast.walk(constructor):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "create_subscription"
            and len(node.args) >= 4
            and isinstance(node.args[1], ast.Constant)
            and node.args[1].value == topic
        ):
            continue
        return node.args[3]
    raise AssertionError(f"missing {topic} subscription")


def test_react_state_subscriptions_use_sensor_qos():
    node_path = (
        Path(__file__).resolve().parents[1] / "llm_gateway" / "llm_gateway_node.py"
    )
    module = ast.parse(node_path.read_text(encoding="utf-8"))
    node_class = next(
        item
        for item in module.body
        if isinstance(item, ast.ClassDef) and item.name == "LLMGatewayNode"
    )
    constructor = next(
        item
        for item in node_class.body
        if isinstance(item, ast.FunctionDef) and item.name == "__init__"
    )

    for topic in ("/yaskawa/joint_states", "/yaskawa/robot_status"):
        qos_arg = _subscription_qos_arg(constructor, topic)
        assert isinstance(qos_arg, ast.Name)
        assert qos_arg.id == "qos_profile_sensor_data"


def test_default_snapshot():
    inj = StateInjector()
    snap = inj.snapshot()
    rs = snap["robot_state"]
    assert rs["joints_rad"] == [0.0] * 6
    assert rs["joint_names"] == [
        "joint_1_s",
        "joint_2_l",
        "joint_3_u",
        "joint_4_r",
        "joint_5_b",
        "joint_6_t",
    ]
    assert rs["mode"] == "IDLE"
    assert rs["active_alarms"] == []
    assert rs["velocity_scale_active"] == 0.06
    assert rs["capabilities"] == {"gripper": False, "perception": False}


def test_update_joint_states():
    inj = StateInjector()
    inj.update_joint_states({"position": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]})
    snap = inj.snapshot()
    assert snap["robot_state"]["joints_rad"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_update_robot_status():
    inj = StateInjector()
    inj.update_robot_status({"mode": "MOVING", "active_alarms": ["alarm1"]})
    snap = inj.snapshot()
    assert snap["robot_state"]["mode"] == "MOVING"
    assert snap["robot_state"]["active_alarms"] == ["alarm1"]


def test_set_velocity_scale():
    inj = StateInjector()
    inj.set_velocity_scale(0.25)
    assert inj.snapshot()["robot_state"]["velocity_scale_active"] == 0.25


def test_set_capabilities():
    inj = StateInjector()
    inj.set_capabilities(gripper=True, perception=False)
    caps = inj.snapshot()["robot_state"]["capabilities"]
    assert caps["gripper"] is True
    assert caps["perception"] is False
