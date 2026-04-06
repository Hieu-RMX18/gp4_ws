#!/usr/bin/env python3

"""ROS2 node for the phase-9 LLM gateway pipeline."""

from __future__ import annotations

import json
from typing import Any, Dict

import rclpy
from interfaces.action import ExecuteMotion
from interfaces.srv import GetCurrentPose, ValidateCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from llm_gateway.goal_mapper import GoalMapper
from llm_gateway.llm_client import OpenAICompatibleLLMClient
from llm_gateway.llm_config import load_llm_backend_config
from llm_gateway.normalizer import Normalizer
from llm_gateway.parser import LLMParser
from llm_gateway.schema_validator import SchemaValidator
from llm_gateway.semantic_validator import SemanticValidator


class LLMGatewayNode(Node):
    """Convert /llm_intent text into a validated ExecuteMotion goal."""

    def __init__(
        self,
        llm_client: OpenAICompatibleLLMClient | None = None,
        parser: LLMParser | None = None,
        schema_validator: SchemaValidator | None = None,
        normalizer: Normalizer | None = None,
        semantic_validator: SemanticValidator | None = None,
        goal_mapper: GoalMapper | None = None,
    ) -> None:
        super().__init__("llm_gateway_node")
        self._declare_parameters()

        schema_path = self.get_parameter("schema_path").get_parameter_value().string_value or None
        llm_config_path = (
            self.get_parameter("llm_config_path").get_parameter_value().string_value or None
        )
        self._default_velocity_scale = (
            self.get_parameter("default_velocity_scale").get_parameter_value().double_value
        )
        self._default_acceleration_scale = (
            self.get_parameter("default_acceleration_scale").get_parameter_value().double_value
        )
        self._safety_service_timeout_sec = (
            self.get_parameter("safety_service_timeout_sec").get_parameter_value().double_value
        )

        self._parser = parser or LLMParser()
        self._schema_validator = schema_validator or SchemaValidator(schema_path)
        self._normalizer = normalizer or Normalizer(
            self._default_velocity_scale, self._default_acceleration_scale
        )
        self._semantic_validator = semantic_validator or SemanticValidator()
        self._goal_mapper = goal_mapper or GoalMapper()
        llm_backend_config = load_llm_backend_config(llm_config_path)
        self._llm_client = llm_client or OpenAICompatibleLLMClient(
            llm_backend_config, self._schema_validator.schema_as_json()
        )

        callback_group = ReentrantCallbackGroup()
        self._intent_subscriber = self.create_subscription(
            String,
            "/llm_intent",
            self.intent_callback,
            10,
            callback_group=callback_group,
        )
        self._raw_subscriber = self.create_subscription(
            String,
            "/llm_raw_command",
            self.raw_command_callback,
            10,
            callback_group=callback_group,
        )
        self._llm_debug_publisher = self.create_publisher(String, "/llm_debug", 10)
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

        # GET_POSE: dedicated query service client — separate from motion path
        self._get_pose_client = self.create_client(
            GetCurrentPose,
            "/get_current_pose",
            callback_group=callback_group,
        )

        self.get_logger().info("LLMGatewayNode ready.")

    def _declare_parameters(self) -> None:
        self.declare_parameter("schema_path", "")
        self.declare_parameter("llm_config_path", "")
        self.declare_parameter("default_velocity_scale", 0.10)
        self.declare_parameter("default_acceleration_scale", 0.10)
        self.declare_parameter("safety_service_timeout_sec", 2.0)
        self.declare_parameter("auto_clear_unimplemented_approval", False)

    def publish_status(self, status: str) -> None:
        self.get_logger().info(f"gateway_status={status}")
        self._status_publisher.publish(String(data=status))

    def intent_callback(self, msg: String) -> None:
        self.process_intent(msg.data)

    def raw_command_callback(self, msg: String) -> None:
        self.process_raw_command(msg.data)

    def raw_command_cb(self, msg: String) -> None:  # pragma: no cover - compatibility shim
        self.raw_command_callback(msg)

    def process_intent(self, intent_text: str) -> None:
        self.publish_status("received")
        try:
            llm_response = self._llm_client.generate_response(intent_text)
        except Exception as exc:
            self._reject("llm_request_failed", str(exc), intent_text=intent_text)
            return

        self.publish_status("llm_response_received")
        self._process_llm_payload(intent_text, llm_response)

    def process_raw_command(self, raw_payload: str) -> None:
        self.publish_status("received")
        self._process_llm_payload("", raw_payload)

    def _process_llm_payload(self, intent_text: str, raw_payload: str) -> None:
        try:
            parsed_command = self._parser.parse(raw_payload)
        except Exception as exc:
            self._reject("llm_parse_failed", str(exc), intent_text=intent_text, raw_llm_output=raw_payload)
            return

        self.publish_status("parsed")
        if parsed_command.get("error"):
            self._reject(
                "unsupported_or_ambiguous",
                str(parsed_command["error"]),
                intent_text=intent_text,
                parsed_command=parsed_command,
            )
            return

        try:
            self._schema_validator.validate(parsed_command)
        except Exception as exc:
            self._reject(
                "schema_validation_failed",
                str(exc),
                intent_text=intent_text,
                parsed_command=parsed_command,
            )
            return

        self.publish_status("schema_valid")
        try:
            normalized_command = self._normalize_and_validate(parsed_command)
        except Exception as exc:
            self._reject(
                "semantic_validation_failed",
                str(exc),
                intent_text=intent_text,
                parsed_command=parsed_command,
            )
            return

        self.publish_status("semantic_valid")

        # Query-only commands bypass the safety gate and motion action pipeline.
        # GET_POSE routes to the dedicated query service — no motion side effects.
        primitive_type = normalized_command.get("primitive_type", "")
        if self._is_query_command(primitive_type):
            self._handle_get_pose_query(normalized_command, intent_text)
            return

        if not self._validate_client.wait_for_service(timeout_sec=self._safety_service_timeout_sec):
            self._reject(
                "validate_service_unavailable",
                "ValidateCommand service unavailable",
                intent_text=intent_text,
                validated_command=self._goal_mapper.to_command_payload(normalized_command),
            )
            return

        command_payload = self._goal_mapper.to_command_payload(normalized_command)
        request = self._build_validate_request(normalized_command, command_payload)
        self.publish_status("safety_validation_requested")
        validation_future = self._validate_client.call_async(request)
        validation_future.add_done_callback(
            lambda future, intent=intent_text, payload=command_payload: self._on_validation_done(
                future, intent, payload
            )
        )

    def _normalize_and_validate(self, command: Dict[str, Any]) -> Dict[str, Any]:
        normalized_command = self._normalizer.normalize(command)
        self._semantic_validator.validate(normalized_command)
        return normalized_command

    def _is_query_command(self, primitive_type: str) -> bool:
        """Return True for query-only commands that bypass the motion action pipeline."""
        return primitive_type == "GET_POSE"

    def _handle_get_pose_query(
        self, normalized_command: Dict[str, Any], intent_text: str
    ) -> None:
        """Route GET_POSE to the dedicated query service — NO motion action path."""
        if not self._get_pose_client.wait_for_service(timeout_sec=self._safety_service_timeout_sec):
            self._reject(
                "get_pose_service_unavailable",
                "GetCurrentPose service unavailable",
                intent_text=intent_text,
            )
            return

        request = GetCurrentPose.Request()
        request.reference_frame = str(normalized_command.get("reference_frame", "base_link"))

        self.publish_status("get_pose_requested")
        future = self._get_pose_client.call_async(request)
        future.add_done_callback(
            lambda f, intent=intent_text: self._on_get_pose_done(f, intent)
        )

    def _on_get_pose_done(self, future: Any, intent_text: str) -> None:
        """Handle GetCurrentPose service response."""
        try:
            response = future.result()
        except Exception as exc:
            self._reject(
                "get_pose_service_call_failed",
                str(exc),
                intent_text=intent_text,
            )
            return

        if not response.success:
            self._reject(
                "get_pose_failed",
                response.message,
                intent_text=intent_text,
            )
            return

        pose = response.current_pose
        pose_data = {
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }

        self.get_logger().info(
            f"GET_POSE result: pos=({pose.position.x:.4f}, "
            f"{pose.position.y:.4f}, {pose.position.z:.4f}), "
            f"orient=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
            f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f})"
        )
        self._publish_debug({
            "status": "query_result",
            "stage": "get_pose",
            "intent": intent_text,
            "message": response.message,
            "current_pose": pose_data,
        })
        self.publish_status("query_succeeded")

    def _build_validate_request(
        self, normalized_command: Dict[str, Any], command_payload: Dict[str, Any]
    ) -> ValidateCommand.Request:
        request = ValidateCommand.Request()
        request.command_json = json.dumps(command_payload, ensure_ascii=True, separators=(",", ":"))
        request.primitive_type = normalized_command["primitive_type"]
        request.velocity_scale = normalized_command.get("velocity_scale", 0.0)
        if "target_pose_msg" in normalized_command:
            request.target_pose = normalized_command["target_pose_msg"]
        return request

    def _on_validation_done(self, future: Any, intent_text: str, command_payload: Dict[str, Any]) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._reject(
                "validate_service_call_failed",
                str(exc),
                intent_text=intent_text,
                validated_command=command_payload,
            )
            return

        if not response.valid:
            self._reject(
                "rejected_by_validate_service",
                str(response.reason),
                intent_text=intent_text,
                validated_command=command_payload,
            )
            return

        try:
            validated_command = self._command_from_sanitized_json(response.sanitized_json, command_payload)
            normalized_command = self._normalize_and_validate(validated_command)
        except Exception as exc:
            self._reject(
                "sanitized_command_invalid",
                str(exc),
                intent_text=intent_text,
                validated_command=command_payload,
            )
            return

        execution_command = self._prepare_execution_command(normalized_command)
        goal_payload = self._goal_mapper.to_command_payload(execution_command)
        self._publish_debug(
            {
                "status": "validated",
                "stage": "validate_command",
                "intent": intent_text,
                "validated_command": goal_payload,
            }
        )
        self.publish_status("safety_approved")

        if not self._execute_client.server_is_ready():
            self._reject(
                "execute_motion_unavailable",
                "ExecuteMotion action server unavailable",
                intent_text=intent_text,
                validated_command=goal_payload,
            )
            return

        goal = self._goal_mapper.to_execute_motion_goal(execution_command)
        send_future = self._execute_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f, intent=intent_text, payload=goal_payload:
                self._on_goal_sent(f, intent, payload)
        )
        self.publish_status("dispatched")

    def _on_goal_sent(
        self, future: Any, intent_text: str, goal_payload: Dict[str, Any]
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._reject(
                "execute_motion_send_failed", str(exc),
                intent_text=intent_text, validated_command=goal_payload)
            return
        if not goal_handle or not goal_handle.accepted:
            self._reject(
                "execute_motion_rejected",
                "ExecuteMotion action server rejected goal",
                intent_text=intent_text, validated_command=goal_payload)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, intent=intent_text, payload=goal_payload:
                self._on_execution_done(f, intent, payload)
        )

    def _on_execution_done(
        self, future: Any, intent_text: str, goal_payload: Dict[str, Any]
    ) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._reject(
                "execute_motion_result_error", str(exc),
                intent_text=intent_text, validated_command=goal_payload)
            return
        if wrapped.result and wrapped.result.success:
            self.get_logger().info(
                f"Execution succeeded: {wrapped.result.message}")
            self.publish_status("succeeded")
            self._publish_debug({
                "status": "succeeded",
                "stage": "execute_motion",
                "intent": intent_text,
                "message": wrapped.result.message,
            })
        else:
            msg = wrapped.result.message if wrapped.result else "no result"
            self._reject(
                "execute_motion_failed", msg,
                intent_text=intent_text, validated_command=goal_payload)

    def _prepare_execution_command(self, normalized_command: Dict[str, Any]) -> Dict[str, Any]:
        auto_clear = (
            self.get_parameter("auto_clear_unimplemented_approval")
            .get_parameter_value()
            .bool_value
        )
        if not auto_clear:
            return normalized_command

        execution_command = dict(normalized_command)
        if execution_command.get("require_approval"):
            # Fake/sim Phase 9 still uses ValidateCommand as the safety gate, but
            # motion_core aborts goals with require_approval=true until that flow
            # is implemented. Keep this override launch-controlled and disabled
            # for future real-hardware phases unless the approval path is restored.
            self.get_logger().info(
                "Clearing require_approval for fake/sim compatibility after ValidateCommand approval."
            )
        self.get_logger().warn(
            "[DEFERRED] require_approval cleared by auto_clear_unimplemented_approval. "
            "Approval gate is deferred to a future phase.")
        execution_command["require_approval"] = False
        return execution_command

    def _command_from_sanitized_json(
        self, sanitized_json: str, fallback_payload: Dict[str, Any]
    ) -> Dict[str, Any]:
        if not sanitized_json:
            return fallback_payload
        loaded = json.loads(sanitized_json)
        if not isinstance(loaded, dict):
            raise ValueError("sanitized_json must decode to a JSON object.")
        self._schema_validator.validate(loaded)
        return loaded

    def _reject(
        self,
        stage: str,
        reason: str,
        *,
        intent_text: str = "",
        raw_llm_output: str | None = None,
        parsed_command: Dict[str, Any] | None = None,
        validated_command: Dict[str, Any] | None = None,
    ) -> None:
        self.get_logger().warning(f"{stage}: {reason}")
        self._publish_debug(
            {
                "status": "rejected",
                "stage": stage,
                "reason": reason,
                "intent": intent_text,
                "raw_llm_output": raw_llm_output,
                "parsed_command": parsed_command,
                "validated_command": validated_command,
            }
        )
        self.publish_status(f"rejected:{stage}")

    def _publish_debug(self, payload: Dict[str, Any]) -> None:
        compact_payload = {
            key: value for key, value in payload.items() if value not in (None, "", [], {})
        }
        self._llm_debug_publisher.publish(
            String(data=json.dumps(compact_payload, ensure_ascii=True, separators=(",", ":")))
        )


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
