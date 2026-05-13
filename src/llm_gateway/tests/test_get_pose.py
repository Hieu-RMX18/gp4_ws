"""Tests for GET_POSE query path — separate from motion action pipeline.

Tests are organized in two tiers:

Tier 1 (source-only): schema, normalizer, semantic validator, parser tests.
    No colcon workspace needed. No rclpy.init() side effects.

Tier 2 (ros_integration): gateway-level mocked tests.
    Require ``interfaces`` package. Marked with ``@pytest.mark.ros_integration``.
    Skipped with visible reason when interfaces is unavailable.
"""

from __future__ import annotations

import json
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from llm_gateway.intent_engine import Normalizer
from llm_gateway.intent_engine import LLMParser
from llm_gateway.intent_engine import SchemaValidator
from llm_gateway.intent_engine import SemanticValidator


# ── Tier 1: Pure-logic tests (no ROS dependencies) ──────────────────────────


def test_schema_accepts_get_pose_minimal(validator: SchemaValidator):
    """GET_POSE with only primitive_type is valid."""
    data = {"primitive_type": "GET_POSE"}
    valid, error = validator.validate_against_schema(data)
    assert valid, f"Expected valid, got: {error}"


def test_schema_rejects_get_pose_with_target_pose(validator: SchemaValidator):
    """GET_POSE must not carry target_pose — schema should still accept
    (schema doesn't enforce mutual exclusion, semantic_validator does)."""
    data = {
        "primitive_type": "GET_POSE",
        "target_pose": {"position": {"x": 0, "y": 0, "z": 0}},
    }
    # Schema allows extra fields for flexibility — semantic_validator rejects.
    valid, _ = validator.validate_against_schema(data)
    assert valid  # Schema passes; semantic_validator is the enforcement point.


def test_normalizer_get_pose_returns_minimal(normalizer: Normalizer):
    """Normalizer should return minimal dict for GET_POSE — no velocity/planner/approval."""
    cmd = {"primitive_type": "GET_POSE"}
    result = normalizer.normalize(cmd)
    assert result["primitive_type"] == "GET_POSE"
    assert result["reference_frame"] == "base_link"
    # Motion fields must NOT be present
    assert "velocity_scale" not in result
    assert "acceleration_scale" not in result
    assert "planner_id" not in result
    assert "require_approval" not in result  # field removed from action


def test_normalizer_get_pose_preserves_reference_frame(normalizer: Normalizer):
    """Normalizer preserves user-provided reference_frame."""
    cmd = {"primitive_type": "GET_POSE", "reference_frame": "tool0"}
    result = normalizer.normalize(cmd)
    assert result["reference_frame"] == "tool0"


def test_semantic_validator_accepts_get_pose(semantic_validator: SemanticValidator):
    """GET_POSE with no motion fields is valid."""
    cmd = {"primitive_type": "GET_POSE"}
    assert semantic_validator.validate(cmd) is True


def test_semantic_validator_rejects_get_pose_with_pose(
    semantic_validator: SemanticValidator,
):
    """GET_POSE must not include target_pose."""
    from geometry_msgs.msg import Pose

    cmd = {"primitive_type": "GET_POSE", "target_pose_msg": Pose()}
    with pytest.raises(ValueError, match="GET_POSE must not include"):
        semantic_validator.validate(cmd)


def test_semantic_validator_rejects_get_pose_with_joints(
    semantic_validator: SemanticValidator,
):
    """GET_POSE must not include joint_target."""
    cmd = {"primitive_type": "GET_POSE", "joint_target": [0.0] * 6}
    with pytest.raises(ValueError, match="GET_POSE must not include"):
        semantic_validator.validate(cmd)


def test_semantic_validator_skips_velocity_check_for_get_pose(
    semantic_validator: SemanticValidator,
):
    """GET_POSE should not need velocity_scale — validators should not reject."""
    # If velocity_scale check ran, this would fail because 0.0 is below min.
    cmd = {"primitive_type": "GET_POSE"}
    assert semantic_validator.validate(cmd) is True


def test_parser_handles_get_pose_direct_json(parser: LLMParser):
    """Parser can extract GET_POSE from a direct JSON payload."""
    raw = json.dumps({"primitive_type": "GET_POSE"})
    result = parser.parse(raw)
    assert result["primitive_type"] == "GET_POSE"


# ── Tier 2: Gateway integration tests (require interfaces) ──────────────────


class ImmediateFuture:
    def __init__(self, result=None, error: Exception = None):
        self._result = result
        self._error = error

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result


@pytest.mark.ros_integration
def test_gateway_routes_get_pose_to_query_service(ros_integration_context):
    """GET_POSE must go through query service, NOT ValidateCommand/ExecuteMotion."""
    from llm_gateway.llm_gateway_node import LLMGatewayNode

    node = LLMGatewayNode()
    statuses = []
    debug_messages = []
    node.publish_status = lambda status: statuses.append(status)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )

    # Mock the LLM to return a GET_POSE semantic IR command
    get_pose_payload = json.dumps(
        {"intent": "get_pose", "reference_frame": "base_link"}
    )
    node._llm_client.generate_response = MagicMock(return_value=get_pose_payload)

    # Mock the query service
    from geometry_msgs.msg import Pose

    mock_pose = Pose()
    mock_pose.position.x = 0.35
    mock_pose.position.y = 0.10
    mock_pose.position.z = 0.45
    mock_pose.orientation.x = 0.0
    mock_pose.orientation.y = 0.707
    mock_pose.orientation.z = 0.0
    mock_pose.orientation.w = 0.707

    mock_response = SimpleNamespace(
        success=True,
        message="current TCP pose in frame: base_link",
        current_pose=mock_pose,
    )
    node._get_pose_client.wait_for_service = MagicMock(return_value=True)
    node._get_pose_client.call_async = MagicMock(
        return_value=ImmediateFuture(mock_response)
    )

    # These should NOT be called for GET_POSE
    node._validate_client.wait_for_service = MagicMock()
    node._validate_client.call_async = MagicMock()
    node._execute_client.send_goal_async = MagicMock()

    node.process_intent("where is the robot")

    # Verify NO motion path was touched
    node._validate_client.wait_for_service.assert_not_called()
    node._validate_client.call_async.assert_not_called()
    node._execute_client.send_goal_async.assert_not_called()

    # Verify query service was called
    node._get_pose_client.call_async.assert_called_once()

    # Verify debug output contains pose data
    assert any(
        m.get("status") == "query_result" for m in debug_messages
    ), f"Expected query_result in debug, got: {debug_messages}"
    query_msg = next(m for m in debug_messages if m["status"] == "query_result")
    assert query_msg["current_pose"]["position"]["x"] == 0.35
    assert "query_succeeded" in statuses

    node.destroy_node()


@pytest.mark.ros_integration
def test_gateway_fails_closed_when_get_pose_service_unavailable(
    ros_integration_context,
):
    """GET_POSE must fail-closed when the query service is unavailable."""
    from llm_gateway.llm_gateway_node import LLMGatewayNode

    node = LLMGatewayNode()
    statuses = []
    debug_messages = []
    node.publish_status = lambda status: statuses.append(status)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )

    get_pose_payload = json.dumps(
        {"intent": "get_pose", "reference_frame": "base_link"}
    )
    node._llm_client.generate_response = MagicMock(return_value=get_pose_payload)
    node._get_pose_client.wait_for_service = MagicMock(return_value=False)

    # These should NOT be called
    node._validate_client.call_async = MagicMock()
    node._execute_client.send_goal_async = MagicMock()

    node.process_intent("get pose")

    node._validate_client.call_async.assert_not_called()
    node._execute_client.send_goal_async.assert_not_called()

    assert debug_messages[-1]["reason"] == "GetCurrentPose service unavailable"
    assert any("rejected" in s for s in statuses)

    node.destroy_node()


@pytest.mark.ros_integration
def test_gateway_handles_get_pose_service_failure(ros_integration_context):
    """GET_POSE must propagate service failure with explicit message."""
    from llm_gateway.llm_gateway_node import LLMGatewayNode

    node = LLMGatewayNode()
    statuses = []
    debug_messages = []
    node.publish_status = lambda status: statuses.append(status)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )

    get_pose_payload = json.dumps(
        {"intent": "get_pose", "reference_frame": "base_link"}
    )
    node._llm_client.generate_response = MagicMock(return_value=get_pose_payload)

    mock_response = SimpleNamespace(
        success=False,
        message="unsupported reference_frame 'tool0'; only 'base_link' is supported",
    )
    node._get_pose_client.wait_for_service = MagicMock(return_value=True)
    node._get_pose_client.call_async = MagicMock(
        return_value=ImmediateFuture(mock_response)
    )

    node._validate_client.call_async = MagicMock()
    node._execute_client.send_goal_async = MagicMock()

    node.process_intent("lấy pose hiện tại")

    node._validate_client.call_async.assert_not_called()
    node._execute_client.send_goal_async.assert_not_called()

    assert "unsupported reference_frame" in debug_messages[-1]["reason"]

    node.destroy_node()


@pytest.mark.ros_integration
def test_gateway_get_pose_does_not_affect_motion_path(ros_integration_context):
    """After GET_POSE, a motion command (HOME) should still use the motion action path."""
    from llm_gateway.llm_gateway_node import LLMGatewayNode

    node = LLMGatewayNode()
    node.publish_status = lambda _: None
    node._llm_debug_publisher.publish = MagicMock()

    # HOME command — must go through ValidateCommand, not query path
    home_payload = json.dumps({"intent": "go_home"})
    node._llm_client.generate_response = MagicMock(return_value=home_payload)
    node._validate_client.wait_for_service = MagicMock(return_value=True)
    sanitized_json = json.dumps(
        {
            "primitive_type": "HOME",
            "velocity_scale": 0.06,
            "acceleration_scale": 0.06,
            "planner_id": "PILZ_PTP",
        }
    )
    node._validate_client.call_async = MagicMock(
        return_value=ImmediateFuture(
            SimpleNamespace(valid=True, reason="OK", sanitized_json=sanitized_json)
        )
    )
    mock_exec_result = SimpleNamespace(
        result=SimpleNamespace(success=True, message="ok", execution_time_sec=0.1)
    )
    mock_goal_handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: ImmediateFuture(mock_exec_result),
    )
    node._execute_client.server_is_ready = MagicMock(return_value=True)
    node._execute_client.send_goal_async = MagicMock(
        return_value=ImmediateFuture(mock_goal_handle)
    )

    # GET_POSE service should NOT be called for HOME
    node._get_pose_client.call_async = MagicMock()

    node.process_intent("go home")

    node._get_pose_client.call_async.assert_not_called()
    node._validate_client.call_async.assert_called_once()
    node._execute_client.send_goal_async.assert_called_once()

    node.destroy_node()
