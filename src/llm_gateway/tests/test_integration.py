"""Gateway integration tests — require colcon-sourced workspace.

These tests exercise the full LLMGatewayNode with mocked ROS service/action
clients. They require the ``interfaces`` package to be on PYTHONPATH, which is
only available after ``colcon build && source install/setup.bash``.

In source-only mode, the entire module is collected but skipped with an
explicit reason visible in pytest output.
"""

import json
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytestmark = [
    pytest.mark.ros_integration,
    pytest.mark.usefixtures("ros_integration_context"),
]

# ── Import availability detection ────────────────────────────────────────────
_INTERFACES_AVAILABLE = False
try:
    import interfaces  # noqa: F401

    _INTERFACES_AVAILABLE = True
except ImportError:
    pass

_SKIP_REASON = "requires colcon-sourced workspace with built interfaces"


# Conditional imports — only available when interfaces is on PYTHONPATH.
# Tests are skipped before these are referenced when _INTERFACES_AVAILABLE is False.
if _INTERFACES_AVAILABLE:
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


def test_gateway_full_flow_uses_sanitized_json():
    node = LLMGatewayNode()
    node.set_parameters([Parameter("runtime_mode", value="sim")])
    statuses = []
    debug_messages = []
    command_messages = []
    node.publish_status = lambda status: statuses.append(status)
    semantic_ir_payload = json.dumps(
        {
            "intent": "absolute_move_lin",
            "target_pose": {"position": {"x": 0.30, "y": 0.0, "z": 0.30}},
            "reference_frame": "base_link",
        }
    )
    node._llm_client.generate_response = MagicMock(return_value=semantic_ir_payload)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )
    node._command_publisher.publish = MagicMock(
        side_effect=lambda msg: command_messages.append(json.loads(msg.data))
    )

    sanitized_json = json.dumps(
        {
            "primitive_type": "LIN",
            "target_pose": {
                "position": {"x": 0.30, "y": 0.0, "z": 0.30},
                "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
            },
            "velocity_scale": 0.06,
            "acceleration_scale": 0.06,
            "planner_id": "PILZ_LIN",
        }
    )
    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        return_value=ImmediateFuture(
            SimpleNamespace(valid=True, reason="OK", sanitized_json=sanitized_json)
        )
    )
    node._execute_client.server_is_ready = MagicMock(return_value=True)
    # Mock goal handle must have .accepted and .get_result_async() like a real rclpy goal handle
    mock_exec_result = SimpleNamespace(
        result=SimpleNamespace(success=True, message="ok", execution_time_sec=0.1)
    )
    mock_goal_handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: ImmediateFuture(mock_exec_result),
    )
    node._execute_client.send_goal_async = MagicMock(
        return_value=ImmediateFuture(mock_goal_handle)
    )

    node.process_intent("di chuyển thẳng tới x 0.30 y 0.0 z 0.30")

    goal = node._execute_client.send_goal_async.call_args.args[0]
    assert goal.velocity_scale == 0.06
    # Full flow now completes: debug_messages has both 'validated' and 'succeeded'
    validated_msg = next(m for m in debug_messages if m["status"] == "validated")
    assert validated_msg["validated_command"]["velocity_scale"] == 0.06
    assert "dispatched" in statuses
    assert command_messages[0]["primitive_type"] == "LIN"
    assert command_messages[0]["velocity_scale"] == 0.06

    node.destroy_node()


def test_gateway_plan_only_does_precheck_without_execution():
    node = LLMGatewayNode()
    debug_messages = []
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )
    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        return_value=ImmediateFuture(
            SimpleNamespace(
                valid=True,
                reason="",
                sanitized_json=json.dumps(
                    {
                        "primitive_type": "HOME",
                        "velocity_scale": 0.06,
                        "acceleration_scale": 0.06,
                        "planner_id": "PILZ_PTP",
                        "plan_only": True,
                    }
                ),
            )
        )
    )
    command_messages = []
    node._command_publisher.publish = MagicMock(
        side_effect=lambda msg: command_messages.append(json.loads(msg.data))
    )
    node._execute_client.send_goal_async = MagicMock()

    node.process_raw_command(
        json.dumps(
            {
                "primitive_type": "HOME",
                "velocity_scale": 0.06,
                "acceleration_scale": 0.06,
                "planner_id": "PILZ_PTP",
                "plan_only": True,
            }
        )
    )

    node._validate_client.call_async.assert_called_once()
    assert command_messages == []
    node._execute_client.send_goal_async.assert_not_called()
    # Look for the precheck success message in debug stream
    precheck_msgs = [m for m in debug_messages if m.get("stage") == "plan_only"]
    assert (
        precheck_msgs
    ), f"Expected plan_only stage in debug messages, got {debug_messages}"
    assert precheck_msgs[-1]["status"] == "plan_precheck_succeeded"

    node.destroy_node()


def test_gateway_fails_closed_when_validate_service_unavailable():
    node = LLMGatewayNode()
    debug_messages = []
    semantic_ir_payload = json.dumps(
        {
            "intent": "absolute_move_lin",
            "target_pose": {"position": {"x": 0.30, "y": 0.0, "z": 0.30}},
            "reference_frame": "base_link",
        }
    )
    node._llm_client.generate_response = MagicMock(return_value=semantic_ir_payload)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )
    node._validate_client.wait_for_service = MagicMock(return_value=False)
    node._execute_client.send_goal_async = MagicMock()

    node.process_intent("di chuyển thẳng tới x 0.30 y 0.0 z 0.30")

    node._execute_client.send_goal_async.assert_not_called()
    assert debug_messages[-1]["reason"] == "ValidateCommand service unavailable"

    node.destroy_node()


def test_gateway_fails_closed_when_execute_motion_unavailable():
    node = LLMGatewayNode()
    debug_messages = []
    semantic_ir_payload = json.dumps(
        {
            "intent": "absolute_move_lin",
            "target_pose": {"position": {"x": 0.30, "y": 0.0, "z": 0.30}},
            "reference_frame": "base_link",
        }
    )
    node._llm_client.generate_response = MagicMock(return_value=semantic_ir_payload)
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

    node.process_intent("di chuyển thẳng tới x 0.30 y 0.0 z 0.30")

    node._execute_client.send_goal_async.assert_not_called()
    assert debug_messages[-1]["reason"] == "ExecuteMotion action server unavailable"

    node.destroy_node()


def test_gateway_status_heartbeat_reuses_latest_status():
    node = LLMGatewayNode()
    published = []
    node._status_publisher.publish = MagicMock(
        side_effect=lambda msg: published.append(msg.data)
    )

    node.publish_status("parsed")
    node._publish_status_heartbeat()

    assert node._last_status == "parsed"
    assert published[-2:] == ["parsed", "parsed"]

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


def test_gateway_routes_semantic_ir_single_command():
    node = LLMGatewayNode()
    statuses = []
    command_messages = []
    debug_messages = []
    node.publish_status = lambda status: statuses.append(status)
    node._command_publisher.publish = MagicMock(
        side_effect=lambda msg: command_messages.append(json.loads(msg.data))
    )
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )

    semantic_ir_payload = json.dumps(
        {
            "choices": [
                {
                    "message": {
                        "role": "assistant",
                        "content": json.dumps(
                            {
                                "intent": "absolute_move_lin",
                                "target_pose": {
                                    "position": {"x": 0.30, "y": 0.00, "z": 0.30}
                                },
                                "reference_frame": "base_link",
                                "velocity_scale": 0.06,
                            }
                        ),
                    }
                }
            ]
        }
    )
    node._llm_client.generate_response = MagicMock(return_value=semantic_ir_payload)
    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        return_value=ImmediateFuture(
            SimpleNamespace(valid=True, reason="OK", sanitized_json="")
        )
    )
    node._execute_client.server_is_ready = MagicMock(return_value=True)
    mock_exec_result = SimpleNamespace(
        result=SimpleNamespace(success=True, message="ok", execution_time_sec=0.1)
    )
    mock_goal_handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: ImmediateFuture(mock_exec_result),
    )
    node._execute_client.send_goal_async = MagicMock(
        return_value=ImmediateFuture(mock_goal_handle)
    )

    node.process_intent("move linearly to x 0.30 y 0.00 z 0.30")

    assert "routed" in statuses
    goal = node._execute_client.send_goal_async.call_args.args[0]
    assert goal.primitive_type == "LIN"
    assert command_messages[0]["primitive_type"] == "LIN"
    assert command_messages[0]["reference_frame"] == "base_link"
    assert command_messages[0]["target_pose"]["orientation"] == {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 0.0,
    }
    validated_msg = next(m for m in debug_messages if m["status"] == "validated")
    assert validated_msg["validated_command"]["primitive_type"] == "LIN"

    node.destroy_node()


def test_gateway_executes_sequence_step_by_step():
    node = LLMGatewayNode()
    statuses = []
    command_messages = []
    debug_messages = []
    node.publish_status = lambda status: statuses.append(status)
    node._command_publisher.publish = MagicMock(
        side_effect=lambda msg: command_messages.append(json.loads(msg.data))
    )
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )

    semantic_ir_payload = json.dumps(
        {
            "choices": [
                {
                    "message": {
                        "role": "assistant",
                        "content": json.dumps(
                            {
                                "intent": "sequence",
                                "steps": [
                                    {"intent": "go_home"},
                                    {"intent": "wait", "wait_duration_sec": 1.0},
                                    {
                                        "intent": "absolute_move_lin",
                                        "target_pose": {
                                            "position": {
                                                "x": 0.30,
                                                "y": 0.00,
                                                "z": 0.30,
                                            }
                                        },
                                        "reference_frame": "base_link",
                                    },
                                ],
                            }
                        ),
                    }
                }
            ]
        }
    )
    node._llm_client.generate_response = MagicMock(return_value=semantic_ir_payload)
    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        side_effect=[
            ImmediateFuture(
                SimpleNamespace(valid=True, reason="OK", sanitized_json="")
            ),
            ImmediateFuture(
                SimpleNamespace(valid=True, reason="OK", sanitized_json="")
            ),
            ImmediateFuture(
                SimpleNamespace(valid=True, reason="OK", sanitized_json="")
            ),
        ]
    )
    node._execute_client.server_is_ready = MagicMock(return_value=True)
    goal_handles = [
        SimpleNamespace(
            accepted=True,
            get_result_async=lambda: ImmediateFuture(
                SimpleNamespace(
                    result=SimpleNamespace(
                        success=True, message="ok", execution_time_sec=0.1
                    )
                )
            ),
        )
        for _ in range(3)
    ]
    node._execute_client.send_goal_async = MagicMock(
        side_effect=[ImmediateFuture(goal_handle) for goal_handle in goal_handles]
    )

    node.process_intent("go home, wait, then move linearly")

    assert "routed" in statuses
    assert "sequence_valid" in statuses
    assert "sequence_step:1/3" in statuses
    assert "sequence_step:2/3" in statuses
    assert "sequence_step:3/3" in statuses
    assert "sequence_succeeded" in statuses
    assert node._validate_client.call_async.call_count == 3
    assert node._execute_client.send_goal_async.call_count == 3
    assert [
        goal.args[0].primitive_type
        for goal in node._execute_client.send_goal_async.call_args_list
    ] == [
        "HOME",
        "WAIT",
        "LIN",
    ]
    assert [command["primitive_type"] for command in command_messages] == [
        "HOME",
        "WAIT",
        "LIN",
    ]
    assert any(m.get("status") == "sequence_succeeded" for m in debug_messages)

    node.destroy_node()


def test_gateway_aborts_sequence_after_first_failed_step_and_marks_manual_recovery():
    node = LLMGatewayNode()
    statuses = []
    debug_messages = []
    node.publish_status = lambda status: statuses.append(status)
    node._llm_debug_publisher.publish = MagicMock(
        side_effect=lambda msg: debug_messages.append(json.loads(msg.data))
    )

    semantic_ir_payload = json.dumps(
        {
            "choices": [
                {
                    "message": {
                        "role": "assistant",
                        "content": json.dumps(
                            {
                                "intent": "sequence",
                                "steps": [
                                    {
                                        "intent": "io_set",
                                        "io_address": 10010,
                                        "io_value": 1,
                                    },
                                    {"intent": "go_home"},
                                    {
                                        "intent": "absolute_move_lin",
                                        "target_pose": {
                                            "position": {
                                                "x": 0.30,
                                                "y": 0.00,
                                                "z": 0.30,
                                            }
                                        },
                                        "reference_frame": "base_link",
                                    },
                                ],
                            }
                        ),
                    }
                }
            ]
        }
    )
    node._llm_client.generate_response = MagicMock(return_value=semantic_ir_payload)
    node._validate_client.wait_for_service = MagicMock(return_value=True)
    node._validate_client.call_async = MagicMock(
        side_effect=[
            ImmediateFuture(
                SimpleNamespace(valid=True, reason="OK", sanitized_json="")
            ),
            ImmediateFuture(
                SimpleNamespace(
                    valid=False, reason="blocked by safety gate", sanitized_json=""
                )
            ),
        ]
    )
    node._execute_client.server_is_ready = MagicMock(return_value=True)
    first_goal_handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: ImmediateFuture(
            SimpleNamespace(
                result=SimpleNamespace(
                    success=True, message="ok", execution_time_sec=0.1
                )
            )
        ),
    )
    node._execute_client.send_goal_async = MagicMock(
        return_value=ImmediateFuture(first_goal_handle)
    )

    node.process_intent("set io, go home, then move linearly")

    assert node._validate_client.call_async.call_count == 2
    assert node._execute_client.send_goal_async.call_count == 1
    assert "sequence_valid" in statuses
    assert "sequence_step:1/3" in statuses
    assert "sequence_step:2/3" in statuses
    assert "rejected:sequence_step_failed" in statuses
    assert not any(status == "sequence_step:3/3" for status in statuses)
    assert debug_messages[-1]["stage"] == "sequence_step_failed"
    assert debug_messages[-1]["manual_recovery_required"] is True
    assert debug_messages[-1]["failed_step_index"] == 1

    node.destroy_node()
