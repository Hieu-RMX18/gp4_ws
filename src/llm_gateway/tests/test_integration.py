import json
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.parameter import Parameter

from llm_gateway.llm_gateway_node import LLMGatewayNode


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


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_gateway_full_flow_uses_sanitized_json(openai_payload):
    node = LLMGatewayNode()
    node.set_parameters([Parameter("auto_clear_unimplemented_approval", value=True)])
    statuses = []
    debug_messages = []
    node.publish_status = lambda status: statuses.append(status)
    node._llm_client.generate_response = MagicMock(return_value=openai_payload)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )

    sanitized_json = json.dumps(
        {
            "primitive_type": "LIN",
            "target_pose": {
                "position": {"x": 0.35, "y": 0.1, "z": 0.2},
                "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
            },
            "velocity_scale": 0.15,
            "acceleration_scale": 0.1,
            "planner_id": "PILZ_LIN",
            "require_approval": True,
        }
    )
    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        return_value=ImmediateFuture(
            SimpleNamespace(valid=True, reason="OK", sanitized_json=sanitized_json)
        )
    )
    node._execute_client.server_is_ready = MagicMock(return_value=True)
    node._execute_client.send_goal_async = MagicMock(return_value=ImmediateFuture(SimpleNamespace()))

    node.process_intent("di chuyển thẳng tới x 0.35 y 0.1 z 0.2")

    goal = node._execute_client.send_goal_async.call_args.args[0]
    assert goal.velocity_scale == 0.15
    assert goal.require_approval is False
    assert debug_messages[-1]["status"] == "validated"
    assert debug_messages[-1]["validated_command"]["velocity_scale"] == 0.15
    assert debug_messages[-1]["validated_command"]["require_approval"] is False
    assert statuses[-1] == "dispatched"

    node.destroy_node()


def test_gateway_preserves_require_approval_when_auto_clear_disabled(openai_payload):
    node = LLMGatewayNode()
    debug_messages = []
    node._llm_client.generate_response = MagicMock(return_value=openai_payload)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )

    sanitized_json = json.dumps(
        {
            "primitive_type": "HOME",
            "velocity_scale": 0.10,
            "acceleration_scale": 0.10,
            "planner_id": "PILZ_PTP",
            "require_approval": True,
        }
    )
    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        return_value=ImmediateFuture(
            SimpleNamespace(valid=True, reason="OK", sanitized_json=sanitized_json)
        )
    )
    node._execute_client.server_is_ready = MagicMock(return_value=True)
    node._execute_client.send_goal_async = MagicMock(return_value=ImmediateFuture(SimpleNamespace()))

    node.process_intent("di chuyển về home")

    goal = node._execute_client.send_goal_async.call_args.args[0]
    assert goal.require_approval is True
    assert debug_messages[-1]["validated_command"]["require_approval"] is True

    node.destroy_node()


def test_gateway_fails_closed_when_validate_service_unavailable(openai_payload):
    node = LLMGatewayNode()
    debug_messages = []
    node._llm_client.generate_response = MagicMock(return_value=openai_payload)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )
    node._validate_client.wait_for_service = MagicMock(return_value=False)
    node._execute_client.send_goal_async = MagicMock()

    node.process_intent("di chuyển thẳng tới x 0.35 y 0.1 z 0.2")

    node._execute_client.send_goal_async.assert_not_called()
    assert debug_messages[-1]["reason"] == "ValidateCommand service unavailable"

    node.destroy_node()


def test_gateway_fails_closed_when_execute_motion_unavailable(openai_payload):
    node = LLMGatewayNode()
    debug_messages = []
    node._llm_client.generate_response = MagicMock(return_value=openai_payload)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )
    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        return_value=ImmediateFuture(
            SimpleNamespace(valid=True, reason="OK", sanitized_json="")
        )
    )
    node._execute_client.server_is_ready = MagicMock(return_value=False)
    node._execute_client.send_goal_async = MagicMock()

    node.process_intent("di chuyển thẳng tới x 0.35 y 0.1 z 0.2")

    node._execute_client.send_goal_async.assert_not_called()
    assert debug_messages[-1]["reason"] == "ExecuteMotion action server unavailable"

    node.destroy_node()


def test_gateway_rejects_model_error_payload(model_error_payload):
    node = LLMGatewayNode()
    debug_messages = []
    node._llm_client.generate_response = MagicMock(return_value=model_error_payload)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )
    node._validate_client.call_async = MagicMock()
    node._execute_client.send_goal_async = MagicMock()

    node.process_intent("làm gì đó mơ hồ")

    node._validate_client.call_async.assert_not_called()
    node._execute_client.send_goal_async.assert_not_called()
    assert debug_messages[-1]["reason"] == "UNSUPPORTED_OR_AMBIGUOUS_COMMAND"

    node.destroy_node()
