"""Source-level TaskPlanner gateway pipeline tests.

These tests avoid DDS setup but exercise the same process_intent ->
_process_llm_payload path used by LLMGatewayNode after TaskPlanner returns FactoryTask.
"""

from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.ros_integration

from llm_gateway.factory_task import GoalMapper
from llm_gateway.factory_task import IntentRouter
from llm_gateway.factory_task import Normalizer
from llm_gateway.factory_task import LLMParser
from llm_gateway.factory_task import SchemaValidator
from llm_gateway.factory_task import SemanticValidator
from llm_gateway.factory_task import TaskCompiler
from llm_gateway.factory_task import WorldModel
from llm_gateway.factory_task import parse_factory_task
from llm_gateway.task_planner import StateInjector
from llm_gateway.semantic_ir_contract import FACTORY_TASK_RUNTIME_INTENT


class _StaticTaskPlanner:
    def __init__(self, result):
        self.result = result
        self.user_text = None
        self.calls = []

    def plan(self, user_text):
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

def _skill_node(name, args=None):
    return {"type": "skill", "name": name, "args": dict(args or {})}

def _factory_task(task_id="task", root=None):
    return {
        "task_type": "factory_task",
        "version": "1.0",
        "task_id": task_id,
        "root": root or _skill_node("go_home"),
    }

def _factory_task_from_semantic_ir(task_id, payload):
    if not isinstance(payload, dict):
        return payload
    if payload.get("task_type") == "factory_task" or "error" in payload:
        return payload
    intent = payload.get("intent")
    if intent == "sequence":
        return _factory_task(
            task_id,
            {
                "type": "sequence",
                "children": [
                    _factory_task_from_semantic_node(step)
                    for step in payload.get("steps", [])
                ],
            },
        )
    return _factory_task(task_id, _factory_task_from_semantic_node(payload))

def _factory_task_from_semantic_node(payload):
    intent = payload.get("intent")
    args = {key: value for key, value in payload.items() if key != "intent"}
    if intent == "go_home":
        return _skill_node("go_home")
    if intent == "move_named_pose":
        return _skill_node("move_named_pose", args)
    if intent == "move_to_named_pose":
        return _skill_node("move_named_pose", args)
    if intent == "absolute_move_ptp":
        return _skill_node("move_cartesian", args)
    if intent == "absolute_move_lin":
        args = dict(args)
        args["motion_type"] = "lin"
        return _skill_node("move_cartesian", args)
    if intent in {
        "get_pose",
        "wait",
        "move_relative",
        "move_joint",
        "move_joint_delta",
        "move_joints",
        "set_speed",
        "draw_shape",
        "draw_text",
        "stop",
        "alarm_reset",
    }:
        return _skill_node(intent, args)
    return _skill_node(str(intent or "unsupported"), args)

def _review_payload(response):
    return LLMParser().parse(response.semantic_ir_json)

def _semantic_core(payload):
    if payload.get("_factory_task_runtime") is True:
        metadata = payload.get("metadata") if isinstance(payload, dict) else {}
        if isinstance(metadata, dict) and isinstance(metadata.get("runtime_plan"), dict):
            task_payload = {
                "task_type": "factory_task",
                "version": "1.0",
                "task_id": "review-core",
                "root": metadata["runtime_plan"],
            }
            compiled = TaskCompiler(world_model=WorldModel()).compile(
                parse_factory_task(task_payload)
            )
            return {
                key: value
                for key, value in compiled.semantic_ir.items()
                if key != "metadata"
            }
    return {
        key: value
        for key, value in payload.items()
        if key != "metadata" and not str(key).startswith("_")
    }

def _assert_planner_factory_task_review(
    payload, *, task_id="planner-task", runtime_type=None
):
    assert payload["intent"] == FACTORY_TASK_RUNTIME_INTENT
    assert payload["_factory_task_runtime"] is True
    assert payload["_parse_source"] == "llm_factory_task"
    metadata = payload.get("metadata")
    assert isinstance(metadata, dict)
    assert metadata["factory_task"]["task_id"] == task_id
    assert isinstance(metadata["runtime_plan"], dict)
    if runtime_type is not None:
        assert metadata["runtime_plan"]["type"] == runtime_type
    assert metadata["policy_decisions"][0]["decision"] == "allow"


def _gateway_node_type():
    pytest.importorskip(
        "interfaces", reason="requires built interfaces for LLMGatewayNode imports"
    )
    from llm_gateway.llm_gateway_node import LLMGatewayNode

    return LLMGatewayNode


def _make_gateway_shell(planner_result, *, raw_planner_result=False):
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    node._runtime_mode = "hardware"
    node._safety_service_timeout_sec = 2.0
    node._review_intent_requires_token = False
    node._planner_enabled = True
    if not raw_planner_result:
        planner_result = _factory_task_from_semantic_ir("planner-task", planner_result)
    node._task_planner = _StaticTaskPlanner(planner_result)
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


def test_removed_react_planner_module_is_not_available():
    import importlib

    with pytest.raises(ModuleNotFoundError):
        importlib.import_module("llm_gateway.react_planner")


def test_direct_process_intent_rejects_factory_task_motion_without_review():
    node, statuses, dispatched = _make_gateway_shell({"intent": "go_home"})
    rejections = []
    node._reject = lambda stage, reason, **kwargs: rejections.append(
        {"stage": stage, "reason": reason, "intent": kwargs.get("intent_text")}
    )

    node.process_intent("go home")

    assert node._task_planner.user_text == "go home"
    assert "routed" not in statuses
    assert dispatched == []
    assert rejections == [
        {
            "stage": "factory_task_requires_review",
            "reason": (
                "FactoryTask motion requests must be reviewed through "
                "/llm_gateway/review_intent and confirmed before execution."
            ),
            "intent": "go home",
        }
    ]


def test_review_intent_returns_factory_task_runtime_sentinel_without_dispatching_motion():
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
    payload = _review_payload(result)
    _assert_planner_factory_task_review(payload, runtime_type="skill")
    assert payload["metadata"]["runtime_plan"] == {
        "type": "skill",
        "name": "move_named_pose",
        "args": {"pose_name": "ready"},
    }
    assert node._task_planner.user_text == "move to ready"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_returns_runtime_sentinel_for_hmi_visible_policy_metadata():
    node, statuses, dispatched = _make_gateway_shell(
        {
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
    )
    request = SimpleNamespace(
        raw_text="go home then wait",
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
    payload = LLMParser().parse(result.semantic_ir_json)
    _assert_planner_factory_task_review(
        payload, task_id="home-wait", runtime_type="sequence"
    )
    assert payload["metadata"]["runtime_plan"]["children"] == [
        {"type": "skill", "name": "go_home"},
        {"type": "skill", "name": "wait", "args": {"wait_duration_sec": 1.0}},
    ]
    assert node._task_planner.user_text == "go home then wait"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_factory_task_move_to_object_preserves_runtime_grounding_plan():
    from llm_gateway.llm_gateway_node import _SceneSnapshotCache

    node, statuses, dispatched = _make_gateway_shell(
        {
            "task_type": "factory_task",
            "version": "1.0",
            "task_id": "move-apple",
            "root": {
                "type": "skill",
                "name": "move_to_object",
                "args": {"object_ref": "apple"},
            },
        }
    )
    node._scene_snapshot_cache = _SceneSnapshotCache(
        ttl_sec=2.0, now_fn=lambda: 10.0
    )
    node._scene_snapshot_cache.store(
        {"class_filter": "apple", "frame": "base_link"},
        {
            "detections": [
                {
                    "class_id": "apple",
                    "frame_id": "base_link",
                    "position": {"x": 0.31, "y": 0.12, "z": 0.24},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                }
            ],
            "calibration": {"valid": True},
        },
    )
    request = SimpleNamespace(
        raw_text="move to apple",
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
    payload = LLMParser().parse(result.semantic_ir_json)
    _assert_planner_factory_task_review(payload, task_id="move-apple", runtime_type="skill")
    assert payload["metadata"]["runtime_plan"] == {
        "type": "skill",
        "name": "move_to_object",
        "args": {"object_ref": "apple"},
    }
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_factory_task_move_to_object_does_not_query_perception_during_review():
    node, statuses, dispatched = _make_gateway_shell(
        {
            "task_type": "factory_task",
            "version": "1.0",
            "task_id": "move-red-box",
            "root": {
                "type": "skill",
                "name": "move_to_object",
                "args": {"object_ref": "red_box"},
            },
        }
    )
    live_queries = []

    def query_perception(args):
        live_queries.append(dict(args))
        return {
            "ok": True,
            "error": None,
            "payload": {
                "detections": [
                    {
                        "class_id": "red_box",
                        "frame_id": "base_link",
                        "position": {"x": 0.26, "y": 0.11, "z": 0.18},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        "score": 0.91,
                    }
                ],
                "calibration": {"valid": True},
                "depth_in_range": True,
            },
        }

    node._query_perception_detections = query_perception
    request = SimpleNamespace(
        raw_text="move to red box",
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
    payload = LLMParser().parse(result.semantic_ir_json)
    _assert_planner_factory_task_review(payload, task_id="move-red-box", runtime_type="skill")
    assert live_queries == []
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_factory_task_move_to_object_does_not_fail_early_when_perception_cannot_ground():
    node, _statuses, dispatched = _make_gateway_shell(
        {
            "task_type": "factory_task",
            "version": "1.0",
            "task_id": "move-red-box",
            "root": {
                "type": "skill",
                "name": "move_to_object",
                "args": {"object_ref": "red_box"},
            },
        }
    )
    live_queries = []

    def query_perception(args):
        live_queries.append(dict(args))
        return {"ok": False, "error": "perception_service_not_ready", "payload": None}

    node._query_perception_detections = query_perception
    request = SimpleNamespace(
        raw_text="move to red box",
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
    payload = LLMParser().parse(result.semantic_ir_json)
    _assert_planner_factory_task_review(payload, task_id="move-red-box", runtime_type="skill")
    assert live_queries == []
    assert dispatched == []


def test_review_intent_accepts_runtime_control_factory_task_with_runtime_plan_sentinel():
    """Runtime-control FactoryTasks (fallback/retry/repeat) are now accepted.
    The gateway produces a runtime-execution sentinel IR carrying the full task tree
    so TaskRuntime can execute it while the HMI shows the plan."""
    node, statuses, dispatched = _make_gateway_shell(
        {
            "task_type": "factory_task",
            "version": "1.0",
            "task_id": "fallback-home",
            "root": {
                "type": "fallback",
                "children": [
                    {"type": "skill", "name": "move_named_pose", "args": {"pose_name": "poseA"}},
                    {"type": "skill", "name": "go_home", "args": {}},
                ],
            },
        }
    )
    request = SimpleNamespace(
        raw_text="try poseA, otherwise go home",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True, f"Runtime-control FactoryTask must be accepted; error: {result.error}"
    payload = LLMParser().parse(result.semantic_ir_json)
    metadata = payload.get("metadata") or {}
    assert payload.get("intent") != "get_pose", (
        "Runtime-control FactoryTasks must not masquerade as query-only GET_POSE; "
        "the HMI executes GET_POSE immediately instead of handing off TaskRuntime."
    )
    assert "runtime_plan" in metadata, "Runtime-control FactoryTask must include runtime_plan in metadata"
    assert metadata["runtime_plan"]["type"] == "fallback"
    assert metadata["factory_task"]["task_id"] == "fallback-home"
    assert payload.get("_factory_task_runtime") is True
    policy = metadata.get("policy_decisions", [])
    assert policy[0]["decision"] == "allow"
    assert dispatched == []


def test_review_intent_rejects_planner_semantic_ir_final_payload():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "move_named_pose", "pose_name": "ready"},
        raw_planner_result=True,
    )
    request = SimpleNamespace(
        raw_text="move to ready carefully",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is False
    assert '"error":"UNSUPPORTED_OR_AMBIGUOUS_COMMAND"' in result.semantic_ir_json
    assert "FactoryTask" in result.error
    assert node._task_planner.user_text == "move to ready carefully"
    assert dispatched == []
    assert "routed" not in statuses

def test_review_intent_planner_path_does_not_call_legacy_llm_when_llm_is_down():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "move_named_pose", "pose_name": "ready"}
    )
    node._llm_client = _RecordingLLMClient(error=TimeoutError("llm timed out"))
    request = SimpleNamespace(
        raw_text="move to ready carefully",
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
    semantic_ir = _review_payload(result)
    assert _semantic_core(semantic_ir) == {
        "intent": "move_named_pose",
        "pose_name": "ready",
    }
    _assert_planner_factory_task_review(semantic_ir)
    assert node._llm_client.calls == []
    assert node._task_planner.user_text == "move to ready carefully"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_stop_uses_direct_path_when_llm_is_down():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "error", "error": "planner should not run for protective stop"}
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
    semantic_ir = LLMParser().parse(result.semantic_ir_json)
    assert semantic_ir["intent"] == "stop"
    assert semantic_ir["_parse_source"] == "direct"
    assert semantic_ir["_code_version"] == "unknown"
    assert node._llm_client.calls == []
    assert node._task_planner.user_text is None
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_motion_text_uses_planner_instead_of_llm_mismatch_fallback():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "move_named_pose", "pose_name": "ready"}
    )
    node._llm_client = _RecordingLLMClient('{"intent":"go_home"}')
    request = SimpleNamespace(
        raw_text="move to ready carefully",
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
    assert node._task_planner.user_text == "move to ready carefully"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_motion_text_uses_planner_instead_of_llm_extra_executable_field():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "move_named_pose", "pose_name": "ready"}
    )
    node._llm_client = _RecordingLLMClient(
        '{"intent":"move_named_pose","pose_name":"ready","joint_target":[0.0,0.0,0.0,0.0,0.0,0.0]}'
    )
    request = SimpleNamespace(
        raw_text="move to ready carefully",
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
    assert node._task_planner.user_text == "move to ready carefully"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_direct_tier_emits_source_trace():
    node, _statuses, _dispatched = _make_gateway_shell(
        {"intent": "error", "error": "planner should not run for trace source checks"}
    )
    node._llm_client = _RecordingLLMClient('{"intent":"go_home"}')
    traces = []
    node._emit_trace = lambda event, phase, **kwargs: traces.append(
        {"event": event, "phase": phase, **kwargs}
    )

    result = node._generate_review_semantic_ir("stop")

    assert result["intent"] == "stop"
    assert result["_parse_source"] == "direct"
    assert [trace["event"] for trace in traces] == ["direct_pre_parsed"]
    assert traces[0]["source"] == "direct"


def test_review_intent_reported_sequence_uses_llm_factory_task_path():
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
    semantic_ir = _review_payload(result)
    assert semantic_ir["_parse_source"] == "llm_factory_task"
    semantic_core = _semantic_core(semantic_ir)
    assert [step["intent"] for step in semantic_core["steps"]] == [
        "move_named_pose",
        "move_named_pose",
        "go_home",
        "move_relative",
        "move_named_pose",
        "move_named_pose",
    ]
    assert semantic_core["steps"][3] == {
        "intent": "move_relative",
        "delta": {"x": 0.0, "y": 0.0, "z": -0.05},
        "reference_frame": "base_link",
    }
    assert semantic_core["steps"][5]["pose_name"] == "poseA"
    _assert_planner_factory_task_review(semantic_ir)
    assert node._task_planner.user_text == raw_text
    assert dispatched == []
    assert "routed" not in statuses

def test_review_intent_named_pose_sequence_uses_llm_factory_task():
    node, statuses, dispatched = _make_gateway_shell(
        {
            "intent": "sequence",
            "steps": [
                {"intent": "move_named_pose", "pose_name": "poseA"},
                {"intent": "move_named_pose", "pose_name": "poseB"},
            ],
        }
    )
    request = SimpleNamespace(
        raw_text="move to pose A then come to pose B",
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
    semantic_ir = _review_payload(result)
    assert _semantic_core(semantic_ir) == {
        "intent": "sequence",
        "steps": [
            {"intent": "move_named_pose", "pose_name": "poseA"},
            {"intent": "move_named_pose", "pose_name": "poseB"},
        ],
    }
    assert semantic_ir["_parse_source"] == "llm_factory_task"
    _assert_planner_factory_task_review(semantic_ir)
    assert node._task_planner.user_text == "move to pose A then come to pose B"
    assert dispatched == []
    assert "routed" not in statuses


@pytest.mark.parametrize(
    "raw_text",
    [
        "di ve home nhanh mot chut",
        "đi về home nhanh một chút",
    ],
)
def test_review_intent_home_motion_uses_planner_without_pose_tool(raw_text):
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
    semantic_ir = _review_payload(result)
    assert _semantic_core(semantic_ir) == {"intent": "go_home"}
    _assert_planner_factory_task_review(semantic_ir)
    assert node._task_planner.user_text == raw_text
    assert dispatched == []
    assert "routed" not in statuses


@pytest.mark.parametrize(
    ("raw_text", "planner_result", "expected_semantic_ir_json", "expected_source"),
    [
        (
            "get pose",
            {"intent": "get_pose", "reference_frame": "base_link"},
            '{"intent":"get_pose","reference_frame":"base_link","_parse_source":"planner"}',
            "direct",
        ),
        (
            "wait 2 s",
            {"intent": "wait", "wait_duration_sec": 2.0},
            '{"intent":"wait","wait_duration_sec":2.0,"_parse_source":"planner"}',
            "direct",
        ),
        (
            "move down 1 cm",
            {
                "intent": "move_relative",
                "delta": {"x": 0.0, "y": 0.0, "z": -0.01},
                "reference_frame": "base_link",
            },
            '{"intent":"move_relative","delta":{"x":0.0,"y":0.0,"z":-0.01},"reference_frame":"base_link","_parse_source":"planner"}',
            "llm_factory_task",
        ),
        (
            "move right 1 cm",
            {
                "intent": "move_relative",
                "delta": {"x": 0.0, "y": 0.01, "z": 0.0},
                "reference_frame": "base_link",
            },
            '{"intent":"move_relative","delta":{"x":0.0,"y":0.01,"z":0.0},"reference_frame":"base_link","_parse_source":"planner"}',
            "llm_factory_task",
        ),
        (
            "move left 1 cm",
            {
                "intent": "move_relative",
                "delta": {"x": 0.0, "y": -0.01, "z": 0.0},
                "reference_frame": "base_link",
            },
            '{"intent":"move_relative","delta":{"x":0.0,"y":-0.01,"z":0.0},"reference_frame":"base_link","_parse_source":"planner"}',
            "llm_factory_task",
        ),
        (
            "move joint l 5 deg",
            {"intent": "move_joint", "joint_index": 1, "joint_angle": 0.08726646259971647},
            '{"intent":"move_joint","joint_index":1,"joint_angle":0.08726646259971647,"_parse_source":"planner"}',
            "llm_factory_task",
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
            '{"intent":"sequence","steps":[{"intent":"go_home"},{"intent":"wait","wait_duration_sec":1.0},{"intent":"move_relative","delta":{"x":0.0,"y":0.0,"z":-0.01},"reference_frame":"base_link"}],"_parse_source":"planner"}',
            "llm_factory_task",
        ),
    ],
)
def test_review_intent_operator_motion_text_uses_expected_parse_path(
    raw_text, planner_result, expected_semantic_ir_json, expected_source
):
    node, statuses, dispatched = _make_gateway_shell(planner_result)
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
    expected = LLMParser().parse(expected_semantic_ir_json)
    expected.pop("_parse_source", None)
    semantic_ir = _review_payload(result)
    assert _semantic_core(semantic_ir) == expected
    if expected_source == "llm_factory_task":
        _assert_planner_factory_task_review(semantic_ir)
        assert node._task_planner.user_text == raw_text
    else:
        assert semantic_ir["_parse_source"] == "direct"
        assert node._task_planner.user_text is None
    assert dispatched == []
    assert "routed" not in statuses

def test_review_intent_protective_stop_skips_planner_and_llm():
    node, statuses, dispatched = _make_gateway_shell(
        {"intent": "error", "error": "planner should not run for protective stop"}
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
    semantic_ir = LLMParser().parse(result.semantic_ir_json)
    assert semantic_ir["intent"] == "stop"
    assert semantic_ir["_parse_source"] == "direct"
    assert semantic_ir["_code_version"] == "unknown"
    assert node._llm_client.calls == []
    assert node._task_planner.user_text is None
    assert dispatched == []
    assert "routed" not in statuses

@pytest.mark.parametrize("raw_text", ["move down 2 cm", "move delta down 2 cm"])
def test_review_intent_relative_down_text_uses_llm_factory_task(raw_text):
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
    semantic_ir = _review_payload(result)
    assert _semantic_core(semantic_ir) == {
        "intent": "move_relative",
        "delta": {"x": 0.0, "y": 0.0, "z": -0.02},
        "reference_frame": "base_link",
    }
    assert semantic_ir["_parse_source"] == "llm_factory_task"
    _assert_planner_factory_task_review(semantic_ir)
    assert node._task_planner.user_text == raw_text
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
    assert node._task_planner.calls == [raw_text]
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


def test_review_intent_preserves_planner_tool_relative_move_for_confirm_time_validation():
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
    _assert_planner_factory_task_review(semantic_ir)
    runtime_args = semantic_ir["metadata"]["runtime_plan"]["args"]
    assert runtime_args["reference_frame"] == "tool0"
    assert runtime_args["delta"] == {"x": 0.05, "y": 0.0, "z": 0.0}
    assert semantic_ir["_parse_source"] == "llm_factory_task"
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


def test_review_intent_preserves_planner_joint_delta_for_confirm_time_validation():
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
    semantic_ir = _review_payload(result)
    _assert_planner_factory_task_review(semantic_ir)
    runtime_args = semantic_ir["metadata"]["runtime_plan"]["args"]
    assert runtime_args == {
        "joint_index": 0,
        "delta_angle": 15.0,
        "angular_unit": "deg",
    }
    assert semantic_ir["_parse_source"] == "llm_factory_task"
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_joint_delta_defers_state_validation_to_confirm_time():
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

    assert result.accepted is True
    assert result.error == ""
    semantic_ir = _review_payload(result)
    _assert_planner_factory_task_review(semantic_ir)


def test_review_intent_vietnamese_joint_delta_uses_llm_factory_task():
    node, statuses, dispatched = _make_gateway_shell(
        {
            "intent": "move_joint_delta",
            "joint_index": 2,
            "delta_angle": 15.0,
            "angular_unit": "deg",
        }
    )
    node._latest_joint_positions_rad = [0.0, 0.0, 0.2, 0.0, 0.0, 0.0]
    request = SimpleNamespace(
        raw_text="xoay khớp số 3 +15 độ",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    semantic_ir = LLMParser().parse(result.semantic_ir_json)
    assert _semantic_core(semantic_ir) == {
        "intent": "move_joint_delta",
        "joint_index": 2,
        "delta_angle": 15.0,
        "angular_unit": "deg",
    }
    assert semantic_ir["_parse_source"] == "llm_factory_task"
    assert node._task_planner.user_text == "xoay khớp số 3 +15 độ"
    assert dispatched == []
    assert "routed" not in statuses

def test_review_intent_vietnamese_absolute_joint_uses_llm_factory_task():
    node, statuses, dispatched = _make_gateway_shell(
        {
            "intent": "move_joint",
            "joint_index": 2,
            "joint_angle": 45.0,
            "angular_unit": "deg",
        }
    )
    request = SimpleNamespace(
        raw_text="xoay khớp số 3 sang góc 45 độ",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    semantic_ir = LLMParser().parse(result.semantic_ir_json)
    assert _semantic_core(semantic_ir) == {
        "intent": "move_joint",
        "joint_index": 2,
        "joint_angle": 45.0,
        "angular_unit": "deg",
    }
    assert semantic_ir["_parse_source"] == "llm_factory_task"
    _assert_planner_factory_task_review(semantic_ir)
    assert node._task_planner.user_text == "xoay khớp số 3 sang góc 45 độ"
    assert dispatched == []
    assert "routed" not in statuses

@pytest.mark.parametrize(
    ("raw_text", "expected_semantic_ir_json"),
    [
        (
            "draw a circle with radius 10 mm",
            '{"intent":"draw_shape","shape_type":"circle","units":"mm","frame_id":"base_link","workplane":{"mode":"tool"},"params":{"radius":10.0},"_parse_source":"planner"}',
        ),
        (
            "write GP4",
            '{"intent":"draw_text","text":"GP4","units":"mm","frame_id":"base_link","workplane":{"mode":"tool"},"font":{"type":"single_stroke_builtin","height":20},"_parse_source":"planner"}',
        ),
        (
            "vẽ hình tròn trong mặt phẳng hiện tại bán kính 5cm",
            '{"intent":"draw_shape","shape_type":"circle","units":"cm","frame_id":"base_link","workplane":{"mode":"tool"},"params":{"radius":5.0},"_parse_source":"planner"}',
        ),
        (
            "vẽ hình chữ nhật rộng 6cm cao 3cm",
            '{"intent":"draw_shape","shape_type":"rectangle","units":"cm","frame_id":"base_link","workplane":{"mode":"tool"},"params":{"width":6.0,"height":3.0},"_parse_source":"planner"}',
        ),
        (
            "vẽ chữ HELLO cao 2cm",
            '{"intent":"draw_text","text":"HELLO","units":"cm","frame_id":"base_link","workplane":{"mode":"tool"},"font":{"type":"single_stroke_builtin","height":2.0},"_parse_source":"planner"}',
        ),
    ],
)
def test_review_intent_draw_commands_use_planner_without_regex_validation(
    raw_text, expected_semantic_ir_json
):
    planner_result = LLMParser().parse(expected_semantic_ir_json)
    planner_result.pop("_parse_source", None)
    node, statuses, dispatched = _make_gateway_shell(
        planner_result
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
    expected = LLMParser().parse(expected_semantic_ir_json)
    expected.pop("_parse_source", None)
    semantic_ir = _review_payload(result)
    assert _semantic_core(semantic_ir) == expected
    _assert_planner_factory_task_review(semantic_ir)
    assert node._task_planner.user_text == raw_text
    assert node._task_planner.calls == [raw_text]
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
    assert '"_parse_source":"llm_factory_task"' in second.semantic_ir_json
    assert node._task_planner.calls == [
        "vẽ hình vuông cạnh 4cm",
        "vẽ hình vuông cạnh 4cm",
    ]
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
def test_review_intent_named_pose_motion_uses_llm_factory_task(
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
    semantic_ir = _review_payload(result)
    assert _semantic_core(semantic_ir) == {
        "intent": "move_named_pose",
        "pose_name": expected_pose_name,
    }
    assert semantic_ir["_parse_source"] == "llm_factory_task"
    _assert_planner_factory_task_review(semantic_ir)
    assert node._task_planner.user_text == raw_text
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_cartesian_motion_text_uses_llm_factory_task():
    raw_text = "move to Cartesian x 300 mm y 0 z 400"
    node, statuses, dispatched = _make_gateway_shell(
        {
            "intent": "absolute_move_ptp",
            "target_pose": {"position": {"x": 300.0, "y": 0.0, "z": 400.0}},
            "linear_unit": "mm",
            "reference_frame": "base_link",
            "keep_current_orientation": True,
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
    semantic_ir = _review_payload(result)
    assert _semantic_core(semantic_ir) == {
        "intent": "absolute_move_ptp",
        "target_pose": {"position": {"x": 300.0, "y": 0.0, "z": 400.0}},
        "linear_unit": "mm",
        "reference_frame": "base_link",
        "keep_current_orientation": True,
    }
    assert semantic_ir["_parse_source"] == "llm_factory_task"
    _assert_planner_factory_task_review(semantic_ir)
    assert node._task_planner.user_text == raw_text
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_cartesian_move_to_xyz_uses_llm_factory_task():
    raw_text = "move to x 0.3 y 0.3 z 0.3"
    node, statuses, dispatched = _make_gateway_shell(
        {
            "task_type": "factory_task",
            "version": "1.0",
            "task_id": "move-xyz",
            "root": {
                "type": "skill",
                "name": "move_cartesian",
                "args": {
                    "target_pose": {
                        "position": {"x": 0.3, "y": 0.3, "z": 0.3},
                    },
                    "reference_frame": "base_link",
                    "keep_current_orientation": True,
                },
            },
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
    payload = _review_payload(result)
    assert _semantic_core(payload) == {
        "intent": "absolute_move_ptp",
        "target_pose": {
            "position": {"x": 0.3, "y": 0.3, "z": 0.3},
        },
        "reference_frame": "base_link",
        "keep_current_orientation": True,
    }
    assert payload["_parse_source"] == "llm_factory_task"
    _assert_planner_factory_task_review(payload, task_id="move-xyz")
    assert node._task_planner.user_text == raw_text
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
    assert _semantic_core(semantic_ir) == {
        "intent": "sequence",
        "steps": [
            {"intent": "move_named_pose", "pose_name": "poseA"},
            {"intent": "go_home"},
        ],
    }
    assert semantic_ir["_parse_source"] == "llm_factory_task"
    _assert_planner_factory_task_review(semantic_ir)
    assert node._task_planner.user_text == "di ve toi poseA roi ve home"
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
    assert node._task_planner.user_text is None
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
    assert node._task_planner.user_text is None
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
    assert node._task_planner.user_text is None
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
    assert node._task_planner.user_text is None
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
    assert node._task_planner.user_text is None
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
    node._wait_for_future_without_spinning = lambda future, _timeout: (True, response)

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


def test_gateway_caches_get_pose_result_for_planner_tools():
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


def test_review_intent_vietnamese_draw_circle_uses_planner_path():
    """Vietnamese draw commands should be parsed by TaskPlanner, not regex validation."""
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
    assert '"_parse_source":"llm_factory_task"' in result.semantic_ir_json
    assert node._llm_client.calls == []
    assert node._task_planner.user_text == "vẽ hình tròn bán kính 5cm"
    assert dispatched == []


def test_review_intent_vietnamese_draw_circle_with_plane_suffix_uses_planner_path():
    """Trailing 'trong mặt phẳng hiện tại' must remain a TaskPlanner draw request."""
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
    assert '"_parse_source":"llm_factory_task"' in result.semantic_ir_json
    assert node._llm_client.calls == []
    assert node._task_planner.user_text == "vẽ hình tròn bán kính 5cm trong mặt phẳng hiện tại"
    assert dispatched == []


def test_gateway_planner_state_callbacks_update_state_injector():
    LLMGatewayNode = _gateway_node_type()
    node = object.__new__(LLMGatewayNode)
    node._state_injector = StateInjector()

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

    node._state_joint_state_callback(joint_msg)
    node._state_robot_status_callback(robot_status)

    snapshot = node._state_injector.snapshot()["robot_state"]
    assert snapshot["joint_names"] == joint_names
    assert snapshot["joints_rad"] == joint_positions
    assert node._latest_joint_positions_rad == joint_positions
    assert snapshot["mode"] == "MOVING"
    assert snapshot["active_alarms"] == ["42"]


def test_review_intent_factory_task_with_retry_fallback_produces_runtime_plan_sentinel():
    """A FactoryTask with retry/fallback roots (runtime-control) should return a
    runtime-execution sentinel IR, not fail with FactoryTaskError."""
    node, statuses, dispatched = _make_gateway_shell(
        _factory_task(
            "pick-with-fallback",
            {
                "type": "fallback",
                "children": [
                    {
                        "type": "retry",
                        "count": 2,
                        "children": [
                            {"type": "skill", "name": "pick_object", "args": {"object": "apple"}}
                        ],
                    },
                    {"type": "skill", "name": "go_home", "args": {}},
                ],
            },
        ),
        raw_planner_result=True,
    )
    request = SimpleNamespace(
        raw_text="pick the apple with fallback to home",
        runtime_mode="hardware",
        session_id="session-a",
        operator_id="operator-a",
        command_id="command-a",
        review_token="",
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True, f"Expected accepted; error: {result.error}"
    payload = LLMParser().parse(result.semantic_ir_json)
    metadata = payload.get("metadata") or {}
    assert "runtime_plan" in metadata, "Runtime-control FactoryTask must have runtime_plan in metadata"
    assert "factory_task" in metadata, "Runtime-control FactoryTask must expose factory_task info"
    runtime_plan = metadata["runtime_plan"]
    assert runtime_plan["type"] == "fallback", "Top-level node type must be preserved"
    assert metadata.get("_factory_task_runtime") is True or payload.get("_factory_task_runtime") is True, (
        "Sentinel must set _factory_task_runtime=True"
    )
    policy = metadata.get("policy_decisions", [])
    assert len(policy) >= 1
    assert policy[0]["decision"] == "allow"
    assert "retry/fallback/replan" in policy[0]["reason"]

class _ReadyValidateClient:
    def __init__(self, response):
        self.response = response
        self.requests = []

    def wait_for_service(self, timeout_sec):
        _ = timeout_sec
        return True

    def call_async(self, request):
        self.requests.append(request)
        return object()

class _ExecuteClientReady:
    def __init__(self, ready=True):
        self.ready = ready

    def server_is_ready(self):
        return self.ready

def test_confirm_execution_validates_factory_task_runtime_sentinel_skill_path():
    node, _statuses, _dispatched = _make_gateway_shell({"intent": "go_home"})
    validate_response = SimpleNamespace(valid=True, reason="", sanitized_json="")
    validate_client = _ReadyValidateClient(validate_response)
    node._validate_client = validate_client
    node._execute_client = _ExecuteClientReady(True)
    node._wait_for_future_without_spinning = lambda _future, _timeout: (
        True,
        validate_response,
    )
    sentinel = {
        "intent": FACTORY_TASK_RUNTIME_INTENT,
        "_factory_task_runtime": True,
        "metadata": {
            "factory_task": {
                "task_id": "home-runtime",
                "version": "1.0",
                "mode": "supervised_hardware",
            },
            "runtime_plan": {"type": "skill", "name": "go_home"},
            "policy_decisions": [],
        },
    }
    request = SimpleNamespace(
        command_id="command-a",
        plan_fingerprint="abcdef1234567890",
        operator_id="operator-a",
        session_id="session-a",
        lease_id="lease-a",
        parsed_intent_json=json.dumps(sentinel),
        requested_mode="hardware",
    )
    response = SimpleNamespace(
        accepted=False,
        reason="",
        execution_summary="",
        dispatched_to_ros=False,
    )

    result = node._on_confirm_execution(request, response)

    assert result.accepted is True
    assert result.reason == ""
    assert "FactoryTask home-runtime" in result.execution_summary
    assert result.dispatched_to_ros is False
    assert len(validate_client.requests) == 1
    assert validate_client.requests[0].primitive_type == "HOME"

def test_confirm_execution_rejects_factory_task_runtime_when_execute_motion_unavailable():
    node, _statuses, _dispatched = _make_gateway_shell({"intent": "go_home"})
    validate_response = SimpleNamespace(valid=True, reason="", sanitized_json="")
    node._validate_client = _ReadyValidateClient(validate_response)
    node._execute_client = _ExecuteClientReady(False)
    node._wait_for_future_without_spinning = lambda _future, _timeout: (
        True,
        validate_response,
    )
    sentinel = {
        "intent": FACTORY_TASK_RUNTIME_INTENT,
        "_factory_task_runtime": True,
        "metadata": {
            "factory_task": {"task_id": "home-runtime", "version": "1.0"},
            "runtime_plan": {"type": "skill", "name": "go_home"},
            "policy_decisions": [],
        },
    }
    request = SimpleNamespace(
        command_id="command-a",
        plan_fingerprint="abcdef1234567890",
        operator_id="operator-a",
        session_id="session-a",
        lease_id="lease-a",
        parsed_intent_json=json.dumps(sentinel),
        requested_mode="hardware",
    )
    response = SimpleNamespace(
        accepted=False,
        reason="",
        execution_summary="",
        dispatched_to_ros=False,
    )

    result = node._on_confirm_execution(request, response)

    assert result.accepted is False
    assert "ExecuteMotion action server unavailable" in result.reason


def test_review_cache_is_fully_removed():
    """Spec 2026-06-10 §5: không cache kết quả LLM giữa các lần submit."""
    LLMGatewayNode = _gateway_node_type()
    for legacy_symbol in (
        "_get_semantic" + "_review" + "_cache",
        "_store_semantic" + "_review" + "_cache",
        "_review" + "_cache_key",
    ):
        assert not hasattr(LLMGatewayNode, legacy_symbol), legacy_symbol


def test_generate_review_semantic_ir_reparses_every_submission():
    node, _statuses, _dispatched = _make_gateway_shell({"intent": "go_home"})
    inner_agent = node._task_planner
    calls = []

    class _CountingAgent:
        def plan(self, text):
            calls.append(text)
            return inner_agent.plan(text)

    node._task_planner = _CountingAgent()
    first = node._generate_review_semantic_ir("move to pose A")
    second = node._generate_review_semantic_ir("move to pose A")
    assert len(calls) == 2
    assert first.get("error") is None
    assert second.get("error") is None


def test_review_response_carries_code_version_and_parse_source():
    """Spec 2026-06-10 §5: review response đính parse_source + code_version."""
    node, _statuses, _dispatched = _make_gateway_shell({"intent": "go_home"})
    node._code_version = "abc1234"
    payload = node._generate_review_semantic_ir("về nhà")
    prepared = node._prepare_review_semantic_ir(payload)
    stamped = node._stamp_review_provenance(prepared)
    assert stamped["_parse_source"] in {"direct", "llm_factory_task", "llm_factory_task", "planner", "llm"}
    assert stamped["_code_version"] == "abc1234"


def test_resolve_code_version_returns_short_hash_or_unknown():
    LLMGatewayNode = _gateway_node_type()
    version = LLMGatewayNode._resolve_code_version()
    assert isinstance(version, str)
    assert version == "unknown" or (4 <= len(version) <= 16)


def test_direct_tier_handles_safe_commands_without_planner():
    """Tầng 1 trả lời stop/home/get_pose/alarm_reset/wait — TaskPlanner không được gọi."""
    node, _statuses, _dispatched = _make_gateway_shell({"intent": "go_home"})

    class _MustNotRun:
        def run(self, text):
            raise AssertionError("TaskPlanner must not run for tier-1 commands")

    node._task_planner = _MustNotRun()
    assert node._generate_review_semantic_ir("dừng lại")["intent"] == "stop"
    assert node._generate_review_semantic_ir("về nhà")["intent"] == "go_home"
    assert node._generate_review_semantic_ir("wait 2 s") == {
        "intent": "wait",
        "wait_duration_sec": 2.0,
        "_parse_source": "direct",
    }


def test_legacy_direct_review_shortcut_symbols_are_removed():
    import llm_gateway.llm_gateway_node as node_module

    LLMGatewayNode = _gateway_node_type()
    legacy_module_symbol = "_DIRECT_REVIEW" + "_SHORTCUTS"
    legacy_method_symbol = "_direct_review" + "_semantic_ir"
    assert not hasattr(node_module, legacy_module_symbol)
    assert not hasattr(LLMGatewayNode, legacy_method_symbol)
