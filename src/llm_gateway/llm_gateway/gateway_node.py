#!/usr/bin/env python3

"""ROS2 gateway that parses LLM output and enforces safety-first dispatch."""

from __future__ import annotations

import json
import os
from typing import Any, Dict

import rclpy
from ament_index_python.packages import get_package_share_directory
from interfaces.action import ExecuteMotion
from interfaces.srv import ValidateCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from llm_gateway.approval_flow import ApprovalFlow
from llm_gateway.normalizer import Normalizer
from llm_gateway.parser import LLMParser
from llm_gateway.schema_validator import SchemaValidator


class LLMGatewayNode(Node):
    """LLM Gateway node for parse -> validate -> approve -> dispatch pipeline."""

    def __init__(self) -> None:
        super().__init__("llm_gateway_node")
        self._declare_parameters()

        schema_path = self.get_parameter("schema_path").get_parameter_value().string_value
        if not schema_path:
            schema_path = os.path.join(
                get_package_share_directory("llm_gateway"), "config", "command_schema.json"
            )

        self._auto_approve = self.get_parameter("auto_approve").get_parameter_value().bool_value
        default_velocity = (
            self.get_parameter("default_velocity_scale").get_parameter_value().double_value
        )
        default_acceleration = (
            self.get_parameter("default_acceleration_scale").get_parameter_value().double_value
        )
        self._safety_timeout_sec = (
            self.get_parameter("safety_service_timeout_sec").get_parameter_value().double_value
        )

        self._parser = LLMParser()
        self._schema_validator = SchemaValidator(schema_path)
        self._normalizer = Normalizer(default_velocity, default_acceleration)
        self._approval_flow = ApprovalFlow()

        callback_group = ReentrantCallbackGroup()

        self._raw_subscriber = self.create_subscription(
            String,
            "/llm_raw_command",
            self.raw_command_callback,
            10,
            callback_group=callback_group,
        )
        self._status_publisher = self.create_publisher(String, "/gateway_status", 10)
        self._validate_client = self.create_client(
            ValidateCommand,
            "/validate_command",
            callback_group=callback_group,
        )
        self._execute_client = ActionClient(
            self,
            ExecuteMotion,
            "/execute_motion",
            callback_group=callback_group,
        )

        self.get_logger().info("LLMGatewayNode ready.")

    def _declare_parameters(self) -> None:
        self.declare_parameter("schema_path", "")
        self.declare_parameter("default_velocity_scale", 0.1)
        self.declare_parameter("default_acceleration_scale", 0.1)
        self.declare_parameter("auto_approve", False)
        self.declare_parameter("safety_service_timeout_sec", 2.0)

    def publish_status(self, status: str) -> None:
        """Publish gateway status events for traceability."""
        self.get_logger().info(f"gateway_status={status}")
        self._status_publisher.publish(String(data=status))

    def raw_command_callback(self, msg: String) -> None:
        self.process_raw_command(msg.data)

    # Backward-compatible callback alias used by previous tests.
    def raw_command_cb(self, msg: String) -> None:  # pragma: no cover - compatibility shim
        self.raw_command_callback(msg)

    def process_raw_command(self, raw_command_text: str) -> None:
        """Core Phase-3 flow for one LLM command string."""
        self.publish_status("received")

        try:
            function_call = self._parser.parse(raw_command_text)
            self.publish_status("parsed")
            self._schema_validator.validate(function_call)
            self.publish_status("schema_valid")
            normalized_command = self._normalizer.normalize(function_call["arguments"])
            self.publish_status("normalized")
        except Exception as exc:
            self._reject(f"rejected:parse_or_schema:{exc}")
            return

        if not self._validate_client.wait_for_service(timeout_sec=self._safety_timeout_sec):
            self._reject("rejected:safety_service_unavailable")
            return

        request = self._build_validate_request(normalized_command)
        self.publish_status("safety_validation_requested")
        validation_future = self._validate_client.call_async(request)
        validation_future.add_done_callback(
            lambda future, cmd=normalized_command: self._on_validation_done(future, cmd)
        )

    def _build_validate_request(self, normalized_command: Dict[str, Any]) -> ValidateCommand.Request:
        request = ValidateCommand.Request()
        request.primitive_type = normalized_command["primitive_type"]
        request.velocity_scale = normalized_command["velocity_scale"]

        if "target_pose_msg" in normalized_command:
            request.target_pose = normalized_command["target_pose_msg"]

        safety_payload: Dict[str, Any] = {
            "primitive_type": normalized_command["primitive_type"],
            "velocity_scale": normalized_command["velocity_scale"],
            "acceleration_scale": normalized_command["acceleration_scale"],
            "planner_id": normalized_command["planner_id"],
            "require_approval": normalized_command["require_approval"],
        }
        if normalized_command.get("joint_target"):
            safety_payload["joint_target"] = normalized_command["joint_target"]
        if "target_pose_msg" in normalized_command:
            pose = normalized_command["target_pose_msg"]
            safety_payload["target_pose"] = {
                "position": {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                },
                "orientation": {
                    "x": pose.orientation.x,
                    "y": pose.orientation.y,
                    "z": pose.orientation.z,
                    "w": pose.orientation.w,
                },
            }

        request.command_json = json.dumps(safety_payload, separators=(",", ":"))
        return request

    def _on_validation_done(self, future: Any, normalized_command: Dict[str, Any]) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._reject(f"rejected:safety_call_failed:{exc}")
            return

        if not response.valid:
            self._reject(f"rejected_by_safety:{response.reason}")
            return

        self.publish_status("safety_approved")
        if normalized_command.get("require_approval", True) and not self._auto_approve:
            self.publish_status("approval_requested")
            approval_command = {
                "primitive_type": normalized_command["primitive_type"],
                "velocity_scale": normalized_command["velocity_scale"],
            }
            if not self._approval_flow.request_human_approval(approval_command):
                self._reject("rejected_by_human")
                return
        self.publish_status("approval_passed")
        self._dispatch_execute_motion(normalized_command)

    def _dispatch_execute_motion(self, normalized_command: Dict[str, Any]) -> None:
        goal = ExecuteMotion.Goal()
        goal.primitive_type = normalized_command["primitive_type"]
        goal.velocity_scale = normalized_command["velocity_scale"]
        goal.acceleration_scale = normalized_command["acceleration_scale"]
        goal.planner_id = normalized_command["planner_id"]
        goal.require_approval = False
        goal.joint_target = normalized_command.get("joint_target", [])

        if "target_pose_msg" in normalized_command:
            goal.target_pose = normalized_command["target_pose_msg"]

        if not self._execute_client.server_is_ready():
            # Phase 3 stub: action server may not exist yet.
            self.get_logger().info("/execute_motion unavailable, Phase 3 dispatch stub only.")
            self.publish_status("dispatched")
            return

        self._execute_client.send_goal_async(goal)
        self.publish_status("dispatched")

    def _reject(self, status: str) -> None:
        self.get_logger().warning(status)
        self.publish_status(status)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = LLMGatewayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
