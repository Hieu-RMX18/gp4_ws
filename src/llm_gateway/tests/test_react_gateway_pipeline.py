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

    def run(self, user_text):
        self.user_text = user_text
        return self.result


class _LegacyClientMustNotRun:
    def generate_response(self, user_text):
        raise AssertionError(f"legacy LLM path called for {user_text}")


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
    node._review_intent_token = "review-token"
    node._review_intent_requires_token = True
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

    statuses = []
    dispatched = []
    node.publish_status = statuses.append
    node._dispatch_normalized_command = (
        lambda command, intent_text, **_: dispatched.append(
            {"command": command, "intent": intent_text}
        )
    )
    node._reject = (
        lambda stage, reason, **_: (_ for _ in ()).throw(
            AssertionError(f"{stage}: {reason}")
        )
    )
    return node, statuses, dispatched


def _signed_review_token(
    *,
    raw_text: str,
    runtime_mode: str,
    session_id: str = "session-a",
    operator_id: str = "operator-a",
    command_id: str = "command-a",
) -> str:
    from llm_gateway.llm_gateway_node import build_review_intent_token

    return build_review_intent_token(
        shared_secret="review-token",
        raw_text=raw_text,
        runtime_mode=runtime_mode,
        session_id=session_id,
        operator_id=operator_id,
        command_id=command_id,
    )


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
        review_token=_signed_review_token(
            raw_text="move to ready",
            runtime_mode="hardware",
        ),
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is True
    assert result.error == ""
    assert result.semantic_ir_json == (
        '{"intent":"move_named_pose","pose_name":"ready"}'
    )
    assert node._react_agent.user_text == "move to ready"
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
        review_token=_signed_review_token(raw_text="go home", runtime_mode="sim"),
    )
    response = SimpleNamespace(accepted=False, error="", semantic_ir_json="")

    result = node._on_review_intent(request, response)

    assert result.accepted is False
    assert "runtime_mode mismatch" in result.error
    assert routed_modes == []
    assert node._react_agent.user_text is None
    assert dispatched == []
    assert "routed" not in statuses


def test_review_intent_rejects_missing_review_token_before_llm_call():
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

    assert result.accepted is False
    assert "review token" in result.error
    assert node._react_agent.user_text is None
    assert dispatched == []


def test_review_intent_rejects_plain_shared_secret_token_before_llm_call():
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

    assert result.accepted is False
    assert "review token mismatch" in result.error
    assert node._react_agent.user_text is None
    assert dispatched == []


def test_review_intent_requires_configured_token_even_in_sim_before_llm_call():
    node, _statuses, dispatched = _make_gateway_shell({"intent": "go_home"})
    node._runtime_mode = "sim"
    node._review_intent_token = ""
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

    assert result.accepted is False
    assert "review token" in result.error
    assert node._react_agent.user_text is None
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
    node._reject = (
        lambda stage, reason, **kwargs: rejections.append(
            {"stage": stage, "reason": reason, "intent": kwargs.get("intent_text")}
        )
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
    assert snapshot["mode"] == "MOVING"
    assert snapshot["active_alarms"] == ["42"]
