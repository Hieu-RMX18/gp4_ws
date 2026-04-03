import json
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import rclpy

from llm_gateway.gateway_node import LLMGatewayNode


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


def test_gateway_full_flow_success(openai_payload):
    node = LLMGatewayNode()
    statuses = []
    node.publish_status = lambda status: statuses.append(status)

    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        return_value=ImmediateFuture(SimpleNamespace(valid=True, reason="OK"))
    )
    node._approval_flow.request_human_approval = MagicMock(return_value=True)
    node._execute_client.server_is_ready = MagicMock(return_value=True)
    node._execute_client.send_goal_async = MagicMock(return_value=ImmediateFuture(SimpleNamespace()))

    node.process_raw_command(openai_payload)

    node._validate_client.call_async.assert_called_once()
    node._execute_client.send_goal_async.assert_called_once()

    request = node._validate_client.call_async.call_args.args[0]
    command_json = json.loads(request.command_json)
    assert command_json["primitive_type"] == "LIN"
    assert "target_pose" in command_json
    assert statuses[-1] == "dispatched"

    node.destroy_node()


def test_gateway_rejects_when_safety_fails(openai_payload):
    node = LLMGatewayNode()
    statuses = []
    node.publish_status = lambda status: statuses.append(status)

    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        return_value=ImmediateFuture(SimpleNamespace(valid=False, reason="workspace_guard"))
    )
    node._execute_client.server_is_ready = MagicMock(return_value=True)
    node._execute_client.send_goal_async = MagicMock(return_value=ImmediateFuture(SimpleNamespace()))

    node.process_raw_command(openai_payload)

    node._validate_client.call_async.assert_called_once()
    node._execute_client.send_goal_async.assert_not_called()
    assert any(status.startswith("rejected_by_safety") for status in statuses)

    node.destroy_node()
