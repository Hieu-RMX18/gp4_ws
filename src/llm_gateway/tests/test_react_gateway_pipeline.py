"""Source-level ReAct gateway pipeline tests.

These tests avoid DDS setup but exercise the same process_intent ->
_process_llm_payload path used by LLMGatewayNode after ReAct returns Semantic IR.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.ros_integration

from llm_gateway.intent_engine import GoalMapper
from llm_gateway.intent_engine import IntentRouter
from llm_gateway.intent_engine import Normalizer
from llm_gateway.intent_engine import LLMParser
from llm_gateway.intent_engine import SchemaValidator
from llm_gateway.intent_engine import SemanticValidator
from llm_gateway.react_planner import StateInjector


class _StaticReActAgent:
    def __init__(self, result):
        self.result = result
        self.user_text = None
        self.calls = []

    def run(self, user_text):
        self.user_text = user_text
        self.calls.append(user_text)
        return self.result


class _LegacyClientMustNotRun:
    def generate_response(self, user_text):
        raise AssertionError(f"legacy LLM path called for {user_text}")


class _RecordingLLMClient:
    def __init__(self, response=None, error=None):
        self.response = response or '{"intent":"go_home"}'
        self.error = error
        self.calls = []

    def generate_response(self, user_text):
        self.calls.append(user_text)
        if self.error is not None:
            raise self.error
        return self.response


def _gateway_node_type():
    pytest.importorskip(
        "interfaces", reason="requires built interfaces for LLMGatewayNode imports"
    )
    from llm_gateway.llm_gateway_node import LLMGatewayNode

    return LLMGatewayNode


def _make_gateway_shell(react_result):
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    node._runtime_mode = "hardware"
    node._review_intent_requires_token = False
    node._react_enabled = True
    node._react_agent = _StaticReActAgent(react_result)
    node._llm_client = _LegacyClientMustNotRun()
    node._parser = LLMParser()
    node._intent_router = IntentRouter(runtime_mode="hardware")
    node._schema_validator = SchemaValidator()
    node._normalizer = Normalizer(
        default_velocity_scale=0.06,
        default_acceleration_scale=0.06,
    )
    node._semantic_validator = SemanticValidator()
    node._goal_mapper = GoalMapper(
        default_velocity_scale=0.06,
        default_acceleration_scale=0.06,
    )
    node._hydrate_draw_workplane = lambda payload: payload
    node._request_current_pose_snapshot = lambda reference_frame: None
    node._emit_trace = lambda *args, **kwargs: None

    statuses = []
    dispatched = []
    node.publish_status = statuses.append
    node._dispatch_normalized_command = (
        lambda command, intent_text, **_: dispatched.append(
            {"command": command, "intent": intent_text}
        )
    )
    node._reject = lambda stage, reason, **_: (_ for _ in ()).throw(
        AssertionError(f"{stage}: {reason}")
    )
    return node, statuses, dispatched


def test_react_semantic_ir_go_home_routes_through_intent_router():
    node, statuses, dispatched = _make_gateway_shell({"intent": "go_home"})

    node.process_intent("go home")

    assert node._react_agent.user_text == "go home"
    assert "routed" in statuses
    assert dispatched[0]["intent"] == "go home"
    assert dispatched[0]["command"]["primitive_type"] == "HOME"


def test_review_intent_returns_semantic_ir_without_dispatching_motion():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "move_named_pose", "pose_name": "ready"}
    )
    request = SimpleNamespace(
        raw_text="move to ready",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert result.semantic_ir_json == (
        '{"intent":"move_named_pose","pose_name":"ready","_parse_source":"react"}'
    )
    assert node._react_agent.user_text == "move to ready"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_react_path_does_not_call_legacy_llm_when_llm_is_down():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "move_named_pose", "pose_name": "ready"}
    )
    node._llm_client = _RecordingLLMClient(error=TimeoutError("llm timed out"))
    request = SimpleNamespace(
        raw_text="move to ready",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert result.semantic_ir_json == (
        '{"intent":"move_named_pose","pose_name":"ready","_parse_source":"react"}'
    )
    assert node._llm_client.calls == []
    assert node._react_agent.user_text == "move to ready"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_stop_uses_direct_path_when_llm_is_down():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "error", "error": "react should not run for protective stop"}
    )
    node._llm_client = _RecordingLLMClient(error=TimeoutError("llm timed out"))
    request = SimpleNamespace(
        raw_text="stop",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert result.semantic_ir_json == (
        '{"intent":"stop","_parse_source":"direct_fast_path"}'
    )
    assert node._llm_client.calls == []
    assert node._react_agent.user_text is None
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_motion_text_uses_react_instead_of_llm_mismatch_fallback():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "move_named_pose", "pose_name": "ready"}
    )
    node._llm_client = _RecordingLLMClient('{"intent":"go_home"}')
    request = SimpleNamespace(
        raw_text="move to ready",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert node._llm_client.calls == []
    assert node._react_agent.user_text == "move to ready"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_motion_text_uses_react_instead_of_llm_extra_executable_field():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "move_named_pose", "pose_name": "ready"}
    )
    node._llm_client = _RecordingLLMClient(
        '{"intent":"move_named_pose","pose_name":"ready","joint_target":[0.0,0.0,0.0,0.0,0.0,0.0]}'
    )
    request = SimpleNamespace(
        raw_text="move to ready",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert node._llm_client.calls == []
    assert node._react_agent.user_text == "move to ready"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_direct_fast_path_emits_source_trace():
    node, _statuses, _dispatched = _make_gateway_shell(
        {"intent": "error", "error": "react should not run for trace source checks"}
    )
    node._llm_client = _RecordingLLMClient('{"intent":"go_home"}')
    traces = []
    node._emit_trace = lambda event, phase, **kwargs: traces.append(
        {"event": event, "phase": phase, **kwargs}
    )

    result = node._generate_review_semantic_ir("stop")

    assert result["intent"] == "stop"
    assert result["_parse_source"] == "direct_fast_path"
    assert [trace["event"] for trace in traces] == ["direct_pre_parsed"]
    assert traces[0]["source"] == "direct_fast_path"


def test_review_intent_reported_sequence_uses_react_base_link_path():
    raw_text = (
        "move to pose A then move to pose B then move home then move down 5cm "
        "then move to ready then move to the first pose"
    )
    node, statuses, dispatched = _make_gateway_shell(
        {
            "intent": "sequence",
            "steps": [
                {"intent": "move_named_pose", "pose_name": "poseA"},
                {"intent": "move_named_pose", "pose_name": "poseB"},
                {"intent": "go_home"},
                {
                    "intent": "move_relative",
                    "delta": {"x": 0.0, "y": 0.0, "z": -0.05},
                    "reference_frame": "base_link",
                },
                {"intent": "move_named_pose", "pose_name": "ready"},
                {"intent": "move_named_pose", "pose_name": "poseA"},
            ],
        }
    )
    request = SimpleNamespace(
        raw_text=raw_text,
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    import json

    semantic_ir = json.loads(result.semantic_ir_json)
    assert semantic_ir["_parse_source"] == "react"
    assert [step["intent"] for step in semantic_ir["steps"]] == [
        "move_named_pose",
        "move_named_pose",
        "go_home",
        "move_relative",
        "move_named_pose",
        "move_named_pose",
    ]
    assert semantic_ir["steps"][3] == {
        "intent": "move_relative",
        "delta": {"x": 0.0, "y": 0.0, "z": -0.05},
        "reference_frame": "base_link",
    }
    assert semantic_ir["steps"][5]["pose_name"] == "poseA"
    assert node._react_agent.calls == [raw_text]
    assert dispatched == []
    assert "routed" not in statuses


@pytest.mark.parametrize(
    "raw_text",
    [
        "di ve home",
        "di ve home nhanh mot chut",
        "đi về home nhanh một chút",
    ],
)
def test_review_intent_home_motion_uses_react_without_pose_tool(raw_text):
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "go_home"}
    )
    request = SimpleNamespace(
        raw_text=raw_text,
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert result.semantic_ir_json == (
        '{"intent":"go_home","_parse_source":"react"}'
    )
    assert node._react_agent.user_text == raw_text
    assert dispatched == []
    assert "routed" not in statuses


@pytest.mark.parametrize(
    ("raw_text", "react_result", "expected_semantic_ir_json"),
    [
        (
            "get pose",
            {"intent": "get_pose", "reference_frame": "base_link"},
            '{"intent":"get_pose","reference_frame":"base_link","_parse_source":"react"}',
        ),
        (
            "wait 2 s",
            {"intent": "wait", "wait_duration_sec": 2.0},
            '{"intent":"wait","wait_duration_sec":2.0,"_parse_source":"react"}',
        ),
        (
            "move down 1 cm",
            {
                "intent": "move_relative",
                "delta": {"x": 0.0, "y": 0.0, "z": -0.01},
                "reference_frame": "base_link",
            },
            '{"intent":"move_relative","delta":{"x":0.0,"y":0.0,"z":-0.01},"reference_frame":"base_link","_parse_source":"react"}',
        ),
        (
            "move right 1 cm",
            {
                "intent": "move_relative",
                "delta": {"x": 0.0, "y": 0.01, "z": 0.0},
                "reference_frame": "base_link",
            },
            '{"intent":"move_relative","delta":{"x":0.0,"y":0.01,"z":0.0},"reference_frame":"base_link","_parse_source":"react"}',
        ),
        (
            "move left 1 cm",
            {
                "intent": "move_relative",
                "delta": {"x": 0.0, "y": -0.01, "z": 0.0},
                "reference_frame": "base_link",
            },
            '{"intent":"move_relative","delta":{"x":0.0,"y":-0.01,"z":0.0},"reference_frame":"base_link","_parse_source":"react"}',
        ),
        (
            "move joint 2 5 deg",
            {"intent": "move_joint", "joint_index": 1, "joint_angle": 0.08726646259971647},
            '{"intent":"move_joint","joint_index":1,"joint_angle":0.08726646259971647,"_parse_source":"react"}',
        ),
        (
            "home, wait 1 s, then move down 1 cm",
            {
                "intent": "sequence",
                "steps": [
                    {"intent": "go_home"},
                    {"intent": "wait", "wait_duration_sec": 1.0},
                    {
                        "intent": "move_relative",
                        "delta": {"x": 0.0, "y": 0.0, "z": -0.01},
                        "reference_frame": "base_link",
                    },
                ],
            },
            '{"intent":"sequence","steps":[{"intent":"go_home"},{"intent":"wait","wait_duration_sec":1.0},{"intent":"move_relative","delta":{"x":0.0,"y":0.0,"z":-0.01},"reference_frame":"base_link"}],"_parse_source":"react"}',
        ),
    ],
)
def test_review_intent_operator_motion_text_uses_react(
    raw_text, react_result, expected_semantic_ir_json
):
    node, statuses, dispatched = _make_gateway_shell(react_result)
    node._llm_client = _RecordingLLMClient(expected_semantic_ir_json)
    request = SimpleNamespace(
        raw_text=raw_text,
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert result.semantic_ir_json == expected_semantic_ir_json
    assert node._react_agent.user_text == raw_text
    assert dispatched == []
    assert "routed" not in statuses

def test_review_intent_protective_stop_skips_react_and_llm():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "error", "error": "react should not run for protective stop"}
    )
    node._llm_client = _RecordingLLMClient(error=TimeoutError("llm timed out"))
    request = SimpleNamespace(
        raw_text="stop",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert result.semantic_ir_json == (
        '{"intent":"stop","_parse_source":"direct_fast_path"}'
    )
    assert node._llm_client.calls == []
    assert node._react_agent.user_text is None
    assert dispatched == []
    assert "routed" not in statuses

@pytest.mark.parametrize("raw_text", ["move down 2 cm", "move delta down 2 cm"])
def test_review_intent_relative_down_text_uses_react_semantic_ir(raw_text):
    node, statuses, dispatched = _make_gateway_shell(
        {
            "intent": "move_relative",
            "delta": {"x": 0.0, "y": 0.0, "z": -0.02},
            "reference_frame": "base_link",
        }
    )
    request = SimpleNamespace(
        raw_text=raw_text,
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert result.semantic_ir_json == (
        '{"intent":"move_relative","delta":{"x":0.0,"y":0.0,"z":-0.02},'
        '"reference_frame":"base_link","_parse_source":"react"}'
    )
    assert node._react_agent.calls == [raw_text]
    assert dispatched == []
    assert "routed" not in statuses

@pytest.mark.parametrize("raw_text", ["move down", "move delta down"])
def test_review_intent_relative_move_missing_distance_reports_user_message(raw_text):
    node, _statuses, dispatched = _make_gateway_shell(
        {
            "error": "MISSING_SLOT",
            "intent": "move_relative",
            "missing_fields": ["distance"],
            "hint": "relative move requires direction and distance.",
        }
    )
    request = SimpleNamespace(
        raw_text=raw_text,
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is False
    assert result.error == "relative move requires direction and distance."
    assert '"error":"MISSING_SLOT"' in result.semantic_ir_json
    assert node._react_agent.calls == [raw_text]
    assert dispatched == []


def test_tool_relative_review_move_uses_cached_pose_without_live_request():
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    node._get_cached_current_pose_snapshot = lambda reference_frame: {
        "position": {"x": 0.3, "y": 0.0, "z": 0.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.70710678, "w": 0.70710678},
    }
    node._request_current_pose_snapshot = lambda reference_frame: (_ for _ in ()).throw(
        AssertionError("review should not call live pose service")
    )

    result = node._resolve_tool_relative_review_move(
        {
            "intent": "move_relative",
            "delta": {"x": 0.05, "y": 0.0, "z": 0.0},
            "reference_frame": "tool0",
        }
    )

    assert result["intent"] == "move_relative"
    assert result["reference_frame"] == "base_link"
    assert result["delta"]["x"] == pytest.approx(0.0, abs=1e-8)
    assert result["delta"]["y"] == pytest.approx(0.05)


def test_tool_relative_review_move_fetches_live_pose_on_cache_miss():
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    node._get_cached_current_pose_snapshot = lambda reference_frame: None
    live_requests = []

    def request_pose(reference_frame):
        live_requests.append(reference_frame)
        return {
            "position": {"x": 0.3, "y": 0.0, "z": 0.3},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.70710678, "w": 0.70710678},
        }

    node._request_current_pose_snapshot = request_pose

    result = node._resolve_tool_relative_review_move(
        {
            "intent": "move_relative",
            "delta": {"x": 0.05, "y": 0.0, "z": 0.0},
            "reference_frame": "tool0",
        }
    )

    assert result["intent"] == "move_relative"
    assert result["reference_frame"] == "base_link"
    assert result["delta"]["x"] == pytest.approx(0.0, abs=1e-8)
    assert result["delta"]["y"] == pytest.approx(0.05)
    assert live_requests == ["base_link"]


def test_review_intent_resolves_react_tool_relative_move_before_routing():
    node, statuses, dispatched = _make_gateway_shell(
        {
            "intent": "move_relative",
            "delta": {"x": 0.05, "y": 0.0, "z": 0.0},
            "reference_frame": "tool0",
        }
    )
    node._get_cached_current_pose_snapshot = lambda reference_frame: {
        "position": {"x": 0.3, "y": 0.0, "z": 0.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.70710678, "w": 0.70710678},
    }
    request = SimpleNamespace(
        raw_text="move forward 5 cm in tool frame",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    semantic_ir = LLMParser().parse(result.semantic_ir_json)
    assert semantic_ir["reference_frame"] == "base_link"
    assert semantic_ir["delta"]["x"] == pytest.approx(0.0, abs=1e-8)
    assert semantic_ir["delta"]["y"] == pytest.approx(0.05)
    assert semantic_ir["_parse_source"] == "react"
    assert dispatched == []
    assert "routed" not in statuses


def test_tool_relative_review_move_fails_closed_without_cached_or_live_pose():
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    node._get_cached_current_pose_snapshot = lambda reference_frame: None
    node._request_current_pose_snapshot = lambda reference_frame: None

    result = node._resolve_tool_relative_review_move(
        {
            "intent": "move_relative",
            "delta": {"x": 0.05, "y": 0.0, "z": 0.0},
            "reference_frame": "tool0",
        }
    )

    assert result["error"] == "CURRENT_POSE_UNAVAILABLE"
    assert "current pose unavailable" in result["message"]


def test_review_intent_resolves_react_joint_delta_before_routing():
    node, statuses, dispatched = _make_gateway_shell(
        {
            "intent": "move_joint_delta",
            "joint_index": 0,
            "delta_angle": 15.0,
            "angular_unit": "deg",
        }
    )
    node._latest_joint_positions_rad = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    request = SimpleNamespace(
        raw_text="rotate joint s +15 degrees",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    import json

    semantic_ir = json.loads(result.semantic_ir_json)
    assert semantic_ir["intent"] == "move_joint"
    assert semantic_ir["joint_index"] == 0
    assert semantic_ir["joint_angle"] == pytest.approx(0.1 + 0.2617993877991494)
    assert semantic_ir["_parse_source"] == "react"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_joint_delta_fails_closed_with_clear_state_reason():
    node, _statuses, _dispatched = _make_gateway_shell(
        {
            "intent": "move_joint_delta",
            "joint_index": 0,
            "delta_angle": 15.0,
            "angular_unit": "deg",
        }
    )
    node._latest_joint_positions_rad = []
    request = SimpleNamespace(
        raw_text="rotate joint s +15 degrees",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is False
    assert "relative joint move" in result.error
    assert "current joint positions unavailable" in result.error


@pytest.mark.parametrize(
    ("raw_text", "expected_semantic_ir_json"),
    [
        (
            "draw a circle with radius 10 mm",
            '{"intent":"draw_shape","shape_type":"circle","units":"mm","frame_id":"base_link","workplane":{"mode":"tool"},"params":{"radius":10.0},"_parse_source":"react"}',
        ),
        (
            "write GP4",
            '{"intent":"draw_text","text":"GP4","units":"mm","frame_id":"base_link","workplane":{"mode":"tool"},"font":{"type":"single_stroke_builtin","height":20},"_parse_source":"react"}',
        ),
        (
            "vẽ hình tròn trong mặt phẳng hiện tại bán kính 5cm",
            '{"intent":"draw_shape","shape_type":"circle","units":"cm","frame_id":"base_link","workplane":{"mode":"tool"},"params":{"radius":5.0},"_parse_source":"react"}',
        ),
        (
            "vẽ hình chữ nhật rộng 6cm cao 3cm",
            '{"intent":"draw_shape","shape_type":"rectangle","units":"cm","frame_id":"base_link","workplane":{"mode":"tool"},"params":{"width":6.0,"height":3.0},"_parse_source":"react"}',
        ),
        (
            "vẽ chữ HELLO cao 2cm",
            '{"intent":"draw_text","text":"HELLO","units":"cm","frame_id":"base_link","workplane":{"mode":"tool"},"font":{"type":"single_stroke_builtin","height":2.0},"_parse_source":"react"}',
        ),
    ],
)
def test_review_intent_draw_commands_use_react_without_regex_validation(
    raw_text, expected_semantic_ir_json
):
    react_result = LLMParser().parse(expected_semantic_ir_json)
    react_result.pop("_parse_source", None)
    node, statuses, dispatched = _make_gateway_shell(
        react_result
    )
    node._runtime_mode = "sim"
    node._llm_client = _RecordingLLMClient(error=AssertionError("regex validation should not call LLM"))
    request = SimpleNamespace(
        raw_text=raw_text,
        runtime_mode="sim",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert result.semantic_ir_json == expected_semantic_ir_json
    assert node._react_agent.user_text == raw_text
    assert node._react_agent.calls == [raw_text]
    assert dispatched == []
    assert "routed" not in statuses

def test_review_intent_reuses_cached_draw_semantic_ir_without_llm_call():
    node, _statuses, dispatched = _make_gateway_shell(
        {
            "intent": "draw_shape",
            "shape_type": "square",
            "units": "cm",
            "frame_id": "base_link",
            "workplane": {"mode": "tool"},
            "params": {"side": 4.0},
        }
    )
    node._runtime_mode = "sim"
    node._llm_client = _RecordingLLMClient(error=AssertionError("cache path should not call LLM"))

    request = SimpleNamespace(
        raw_text="vẽ hình vuông cạnh 4cm",
        runtime_mode="sim",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    first = node._on_review_intent(
        request, SimpleNamespace(accepted=False, error="", semantic_ir_json="")
    )
    second = node._on_review_intent(
        request, SimpleNamespace(accepted=False, error="", semantic_ir_json="")
    )

    assert first.accepted is True
    assert second.accepted is True
    assert '"_parse_source":"semantic_cache"' in second.semantic_ir_json
    assert node._react_agent.calls == ["vẽ hình vuông cạnh 4cm"]
    assert dispatched == []


@pytest.mark.parametrize(
    ("raw_text", "expected_pose_name"),
    [
        ("move to pose A", "poseA"),
        ("move to posea", "poseA"),
        ("go to A", "poseA"),
        ("go to B", "poseB"),
        ("move to ready", "ready"),
        ("đến pose A", "poseA"),
        ("tới điểm B", "poseB"),
        ("ve poseb", "poseB"),
        ("move to first pose", "poseA"),
        ("move to the first pose", "poseA"),
    ],
)
def test_review_intent_named_pose_motion_uses_react(
    raw_text, expected_pose_name
):
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "move_named_pose", "pose_name": expected_pose_name}
    )
    request = SimpleNamespace(
        raw_text=raw_text,
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    import json

    semantic_ir = json.loads(result.semantic_ir_json)
    assert semantic_ir["intent"] == "move_named_pose"
    assert semantic_ir["pose_name"] == expected_pose_name
    assert node._react_agent.user_text == raw_text
    assert dispatched == []
    assert "routed" not in statuses


@pytest.mark.parametrize(
    ("raw_text", "react_result"),
    [
        (
            "move to Cartesian x 300 mm y 0 z 400",
            {
                "intent": "absolute_move_ptp",
                "target_pose": {"position": {"x": 300.0, "y": 0.0, "z": 400.0}},
                "linear_unit": "mm",
                "reference_frame": "base_link",
            },
        ),
    ],
)
def test_review_intent_cartesian_motion_text_uses_react(raw_text, react_result):
    node, statuses, dispatched = _make_gateway_shell(react_result)
    request = SimpleNamespace(
        raw_text=raw_text,
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert node._react_agent.user_text == raw_text
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_canonicalizes_legacy_move_to_named_pose_alias_in_sequence():
    node, statuses, dispatched = _make_gateway_shell(
        {
            "intent": "sequence",
            "steps": [
                {"intent": "move_to_named_pose", "pose_name": "poseA"},
                {"intent": "go_home"},
            ],
        }
    )
    request = SimpleNamespace(
        raw_text="di ve toi poseA roi ve home",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    semantic_ir = LLMParser().parse(result.semantic_ir_json)
    assert semantic_ir == {
        "intent": "sequence",
        "steps": [
            {"intent": "move_named_pose", "pose_name": "poseA"},
            {"intent": "go_home"},
        ],
        "_parse_source": "react",
    }
    assert node._react_agent.user_text == "di ve toi poseA roi ve home"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_rejects_runtime_mode_mismatch_before_llm_call(monkeypatch):
    gateway_module = pytest.importorskip(
        "llm_gateway.llm_gateway_node",
        reason="requires built interfaces for LLMGatewayNode imports",
    )

    routed_modes = []

    class _RuntimeModeSpyRouter:
        def __init__(self, runtime_mode: str):
            routed_modes.append(runtime_mode)

        def route(self, semantic_ir):
            _ = semantic_ir
            return SimpleNamespace(route_type="primitive", error_payload=None)

    monkeypatch.setattr(gateway_module, "IntentRouter", _RuntimeModeSpyRouter)
    node, statuses, dispatched = _make_gateway_shell({"intent": "go_home"})
    request = SimpleNamespace(
        raw_text="go home",
        runtime_mode="sim",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is False
    assert "runtime_mode mismatch" in result.error
    assert routed_modes == []
    assert node._react_agent.user_text is None
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_accepts_missing_review_token():
    node, _statuses, dispatched = _make_gateway_shell({"intent": "go_home"})
    request = SimpleNamespace(
        raw_text="go home",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert node._react_agent.user_text == "go home"
    assert dispatched == []


def test_review_intent_ignores_legacy_review_token_value():
    node, _statuses, dispatched = _make_gateway_shell({"intent": "go_home"})
    request = SimpleNamespace(
        raw_text="go home",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="review-token",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert node._react_agent.user_text == "go home"
    assert dispatched == []


def test_review_intent_accepts_when_token_not_required_and_not_configured():
    node, _statuses, dispatched = _make_gateway_shell({"intent": "go_home"})
    node._runtime_mode = "sim"
    node._review_intent_requires_token = False
    request = SimpleNamespace(
        raw_text="go home",
        runtime_mode="sim",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert node._react_agent.user_text == "go home"
    assert dispatched == []


def test_review_intent_rejects_missing_hmi_metadata_before_llm_call():
    node, _statuses, dispatched = _make_gateway_shell({"intent": "go_home"})
    request = SimpleNamespace(
        raw_text="go home",
        runtime_mode="hardware",
        session_id="",
        operator_id="operator-a",
        command_id="command-a",
        review_token="review-token",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is False
    assert "session_id" in result.error
    assert node._react_agent.user_text is None
    assert dispatched == []


def test_review_intent_generated_contract_fields_exist():
    pytest.importorskip(
        "interfaces", reason="requires built interfaces for ReviewIntent imports"
    )
    from interfaces.srv import ReviewIntent

    request = ReviewIntent.Request()
    response = ReviewIntent.Response()

    assert hasattr(request, "raw_text")
    assert hasattr(request, "runtime_mode")
    assert hasattr(request, "session_id")
    assert hasattr(request, "operator_id")
    assert hasattr(request, "command_id")
    assert hasattr(request, "review_token")
    assert hasattr(response, "accepted")
    assert hasattr(response, "error")
    assert hasattr(response, "semantic_ir_json")


def test_gateway_perception_query_fails_closed_when_depth_is_out_of_range():
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    response = SimpleNamespace(
        ok=True,
        failure_reason="",
        detections=[],
        calibration_valid=True,
        calibration_date_iso="2026-05-09T00:00:00Z",
        calibration_age_days=0.0,
        depth_in_range=False,
        depth_noise_mm_p95=8.0,
        stamp=SimpleNamespace(sec=1, nanosec=2),
    )
    node._get_object_positions_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: object(),
    )
    node._safety_service_timeout_sec = 1.0
    node._wait_for_future_without_spinning = lambda future, timeout: (True, response)

    result = node._query_perception_detections({"class_filter": "red_circle"})

    assert result["ok"] is False
    assert result["error"] == "depth_quality_invalid"
    assert result["payload"]["depth_noise_mm_p95"] == 8.0


def test_direct_gateway_topics_reject_by_default_without_dispatching():
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    node._direct_topic_execution_enabled = False
    rejections = []
    dispatched = []
    node._reject = lambda stage, reason, **kwargs: rejections.append(
        {"stage": stage, "reason": reason, "intent": kwargs.get("intent_text")}
    )
    node.process_intent = lambda intent_text: dispatched.append(intent_text)
    node.process_raw_command = lambda raw_payload: dispatched.append(raw_payload)

    node.intent_callback(SimpleNamespace(data="go home"))
    node.raw_command_callback(SimpleNamespace(data='{"primitive_type":"HOME"}'))

    assert [item["stage"] for item in rejections] == [
        "direct_text_topic_disabled",
        "direct_raw_topic_disabled",
    ]
    assert dispatched == []


def test_gateway_main_uses_bounded_executor_shutdown(monkeypatch):
    gateway_module = pytest.importorskip(
        "llm_gateway.llm_gateway_node",
        reason="requires built interfaces for LLMGatewayNode imports",
    )
    shutdown_timeouts = []
    destroyed_nodes = []
    rclpy_shutdown_calls = []

    class _FakeRclpy:
        @staticmethod
        def init(args=None):
            _ = args

        @staticmethod
        def ok():
            return True

        @staticmethod
        def shutdown():
            rclpy_shutdown_calls.append(True)

    class _FakeGatewayNode:
        def destroy_node(self):
            destroyed_nodes.append(True)

    class _FakeExecutor:
        def add_node(self, node):
            self.node = node

        def spin(self):
            raise KeyboardInterrupt()

        def shutdown(self, timeout_sec=None):
            shutdown_timeouts.append(timeout_sec)
            return False

    monkeypatch.setattr(gateway_module, "rclpy", _FakeRclpy)
    monkeypatch.setattr(gateway_module, "LLMGatewayNode", _FakeGatewayNode)
    monkeypatch.setattr(gateway_module, "MultiThreadedExecutor", _FakeExecutor)

    gateway_module.main([])

    assert shutdown_timeouts == [gateway_module.EXECUTOR_SHUTDOWN_TIMEOUT_SEC]
    assert destroyed_nodes == [True]
    assert rclpy_shutdown_calls == [True]


def test_gateway_main_skips_node_destroy_after_external_shutdown(monkeypatch):
    gateway_module = pytest.importorskip(
        "llm_gateway.llm_gateway_node",
        reason="requires built interfaces for LLMGatewayNode imports",
    )
    destroyed_nodes = []
    rclpy_shutdown_calls = []

    class _FakeRclpy:
        @staticmethod
        def init(args=None):
            _ = args

        @staticmethod
        def ok():
            return False

        @staticmethod
        def shutdown():
            rclpy_shutdown_calls.append(True)

    class _FakeGatewayNode:
        def destroy_node(self):
            destroyed_nodes.append(True)

    class _FakeExecutor:
        def add_node(self, node):
            self.node = node

        def spin(self):
            raise gateway_module.ExternalShutdownException()

        def shutdown(self, timeout_sec=None):
            _ = timeout_sec
            return True

    monkeypatch.setattr(gateway_module, "rclpy", _FakeRclpy)
    monkeypatch.setattr(gateway_module, "LLMGatewayNode", _FakeGatewayNode)
    monkeypatch.setattr(gateway_module, "MultiThreadedExecutor", _FakeExecutor)

    gateway_module.main([])

    assert destroyed_nodes == []
    assert rclpy_shutdown_calls == []


def _tri_state(value):
    return SimpleNamespace(val=value)


def test_gateway_caches_get_pose_result_for_react_tools():
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    node._latest_pose_by_frame = {}
    node._current_pose_cache_ttl_sec = 5.0

    pose = SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )

    node._cache_current_pose_snapshot("base_link", pose)

    cached = node._get_cached_current_pose_snapshot("base_link")
    assert cached == {
        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }
    cached["position"]["x"] = 9.9
    cached_again = node._get_cached_current_pose_snapshot("base_link")
    assert cached_again["position"]["x"] == 0.1


def test_review_intent_vietnamese_draw_circle_uses_react_path():
    """Vietnamese draw commands should be parsed by ReAct, not regex validation."""
    node, _statuses, dispatched = _make_gateway_shell(
        {
            "intent": "draw_shape",
            "shape_type": "circle",
            "units": "cm",
            "frame_id": "base_link",
            "workplane": {"mode": "tool"},
            "params": {"radius": 5.0},
        }
    )
    node._llm_client = _RecordingLLMClient(
        error=AssertionError("draw path should not validate regex")
    )
    request = SimpleNamespace(
        raw_text="vẽ hình tròn bán kính 5cm",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert "draw_shape" in result.semantic_ir_json
    assert '"params":{"radius":5.0}' in result.semantic_ir_json
    assert '"_parse_source":"react"' in result.semantic_ir_json
    assert node._llm_client.calls == []
    assert node._react_agent.user_text == "vẽ hình tròn bán kính 5cm"
    assert dispatched == []


def test_review_intent_vietnamese_draw_circle_with_plane_suffix_uses_react_path():
    """Trailing 'trong mặt phẳng hiện tại' must remain a ReAct draw request."""
    node, _statuses, dispatched = _make_gateway_shell(
        {
            "intent": "draw_shape",
            "shape_type": "circle",
            "units": "cm",
            "frame_id": "base_link",
            "workplane": {"mode": "tool"},
            "params": {"radius": 5.0},
        }
    )
    node._llm_client = _RecordingLLMClient(
        error=AssertionError("draw path should not validate regex")
    )
    request = SimpleNamespace(
        raw_text="vẽ hình tròn bán kính 5cm trong mặt phẳng hiện tại",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert "draw_shape" in result.semantic_ir_json
    assert '"params":{"radius":5.0}' in result.semantic_ir_json
    assert '"_parse_source":"react"' in result.semantic_ir_json
    assert node._llm_client.calls == []
    assert node._react_agent.user_text == "vẽ hình tròn bán kính 5cm trong mặt phẳng hiện tại"
    assert dispatched == []


def test_gateway_react_state_callbacks_update_state_injector():
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    node._react_state_injector = StateInjector()

    joint_names = [
        "joint_1_s",
        "joint_2_l",
        "joint_3_u",
        "joint_4_r",
        "joint_5_b",
        "joint_6_t",
    ]
    joint_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    joint_msg = SimpleNamespace(
        name=joint_names,
        position=joint_positions,
    )
    robot_status = SimpleNamespace(
        in_error=_tri_state(0),
        e_stopped=_tri_state(0),
        motion_possible=_tri_state(1),
        in_motion=_tri_state(1),
        error_codes=[42],
    )

    node._react_joint_state_callback(joint_msg)
    node._react_robot_status_callback(robot_status)

    snapshot = node._react_state_injector.snapshot()["robot_state"]
    assert snapshot["joint_names"] == joint_names
    assert snapshot["joints_rad"] == joint_positions
    assert node._latest_joint_positions_rad == joint_positions
    assert snapshot["mode"] == "MOVING"
    assert snapshot["active_alarms"] == ["42"]
