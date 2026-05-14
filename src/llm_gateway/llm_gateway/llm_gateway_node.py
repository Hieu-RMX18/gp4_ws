#!/usr/bin/env python3

"""ROS2 node for the phase-9 LLM gateway pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import hmac
import json
import os
import re
import threading
from typing import Any, Dict, List

import rclpy
from industrial_msgs.msg import RobotStatus, TriState
from interfaces.action import ExecuteMotion
from interfaces.srv import (
    ConfirmExecution,
    GetCurrentPose,
    GetObjectPositions,
    GetPrimitiveConstants,
    HydrateWorkplane,
    ReviewIntent,
    ValidateCommand,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from llm_gateway.intent_engine import (
    GoalMapper,
    IntentRouter,
    LLMParser,
    Normalizer,
    SchemaValidator,
    SemanticValidator,
    SequenceValidator,
    command_from_sanitized_json as _pipeline_command_from_sanitized_json,
    hydrate_draw_workplane as _pipeline_hydrate_draw_workplane,
    prepare_execution_command as _pipeline_prepare_execution_command,
)
from llm_gateway.react_planner import (
    ComputeArcPointsTool,
    GetCurrentPoseTool,
    GripperCloseTool,
    GripperOpenTool,
    IterationBudget,
    OpenAICompatibleLLMClient,
    PlanMotionTool,
    QueryPerceptionTool,
    ReActAgent,
    SetSpeedTool,
    StateInjector,
    SubmitMotionTool,
    ToolRegistry,
    WaitForStateTool,
    load_llm_backend_config,
)
from llm_gateway.semantic_ir_contract import validate_semantic_ir_contract


EXECUTOR_SHUTDOWN_TIMEOUT_SEC = 2.0
_REVIEW_INTENT_HMAC_VERSION = "v1"
_DIRECT_STOP_REVIEW_TEXTS = {"stop", "stop motion", "cancel motion", "halt"}
_DIRECT_GET_POSE_REVIEW_TEXTS = {
    "get pose",
    "current pose",
    "where is robot",
    "where is tcp",
}
_DIRECT_WAIT_REVIEW_RE = re.compile(
    r"(?:wait|pause)\s+([+-]?\d+(?:\.\d+)?)\s*(?:s|sec|second|seconds)?"
)


def build_review_intent_token(
    *,
    shared_secret: str,
    raw_text: str,
    runtime_mode: str,
    session_id: str,
    operator_id: str,
    command_id: str,
) -> str:
    """Build the signed ReviewIntent token expected from the HMI backend."""
    normalized_fields = [
        str(raw_text or "").strip(),
        str(runtime_mode or "").strip(),
        str(session_id or "").strip(),
        str(operator_id or "").strip(),
        str(command_id or "").strip(),
    ]
    message = "\0".join(normalized_fields).encode("utf-8")
    digest = hmac.new(
        str(shared_secret or "").encode("utf-8"),
        message,
        hashlib.sha256,
    ).hexdigest()
    return f"{_REVIEW_INTENT_HMAC_VERSION}:{digest}"


@dataclass
class _SequenceExecutionState:
    intent_text: str
    normalized_commands: List[Dict[str, Any]]
    step_count: int
    diagnostics: List[str] = field(default_factory=list)
    start_joints_rad: List[float] = field(default_factory=list)
    current_step_index: int = 0
    executed_io_side_effects: bool = False


class LLMGatewayNode(Node):
    """Convert /llm_intent or /llm_text_input text into a validated ExecuteMotion goal."""

    def __init__(
        self,
        llm_client: OpenAICompatibleLLMClient | None = None,
        parser: LLMParser | None = None,
        schema_validator: SchemaValidator | None = None,
        normalizer: Normalizer | None = None,
        semantic_validator: SemanticValidator | None = None,
        goal_mapper: GoalMapper | None = None,
        intent_router: IntentRouter | None = None,
        sequence_validator: SequenceValidator | None = None,
    ) -> None:
        super().__init__("llm_gateway_node")
        self._declare_parameters()

        schema_path = (
            self.get_parameter("schema_path").get_parameter_value().string_value or None
        )
        llm_config_path = (
            self.get_parameter("llm_config_path").get_parameter_value().string_value
            or None
        )
        self._default_velocity_scale = (
            self.get_parameter("default_velocity_scale")
            .get_parameter_value()
            .double_value
        )
        self._default_acceleration_scale = (
            self.get_parameter("default_acceleration_scale")
            .get_parameter_value()
            .double_value
        )
        self._safety_service_timeout_sec = (
            self.get_parameter("safety_service_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        self._status_heartbeat_period_sec = (
            self.get_parameter("status_heartbeat_period_sec")
            .get_parameter_value()
            .double_value
        )
        self._direct_topic_execution_enabled = (
            self.get_parameter("allow_direct_topic_execution")
            .get_parameter_value()
            .bool_value
        )

        self._parser = parser or LLMParser()
        self._schema_validator = schema_validator or SchemaValidator(schema_path)
        self._normalizer = normalizer or Normalizer(
            self._default_velocity_scale, self._default_acceleration_scale
        )
        self._semantic_validator = semantic_validator or SemanticValidator()
        self._goal_mapper = goal_mapper or GoalMapper(
            default_velocity_scale=self._default_velocity_scale,
            default_acceleration_scale=self._default_acceleration_scale,
        )
        runtime_mode = self._resolve_runtime_mode()
        self._runtime_mode = runtime_mode
        self._review_intent_token = os.getenv("GP4_REVIEW_INTENT_TOKEN", "").strip()
        self._review_intent_requires_token = True
        self._intent_router = intent_router or IntentRouter(runtime_mode=runtime_mode)
        self._sequence_validator = sequence_validator or SequenceValidator(
            schema_validator=self._schema_validator,
            normalizer=self._normalizer,
            semantic_validator=self._semantic_validator,
        )
        self._latest_joint_positions_rad: List[float] = []
        llm_backend_config = load_llm_backend_config(llm_config_path)
        self._llm_client = llm_client or OpenAICompatibleLLMClient(
            llm_backend_config, self._schema_validator.schema_as_json()
        )

        # ── ReAct agent init (W3) ─────────────────────────────────────────────
        self._react_enabled = self._load_react_enabled()
        self._react_state_injector = StateInjector()
        if self._react_enabled:
            tool_registry = (
                ToolRegistry()
                .register(GetCurrentPoseTool())
                .register(PlanMotionTool())
                .register(SubmitMotionTool())
                .register(WaitForStateTool())
                .register(SetSpeedTool())
                .register(QueryPerceptionTool())
                .register(GripperOpenTool())
                .register(GripperCloseTool())
                .register(ComputeArcPointsTool())
            )
            budget = IterationBudget(
                max_total=5,
                max_motion=3,
                max_readonly=10,
                max_repair=1,
                wall_clock_timeout_s=30.0,
            )
            self._react_agent = ReActAgent(
                llm_client=self._llm_client,
                tool_registry=tool_registry,
                state_injector=self._react_state_injector,
                budget=budget,
                schema_validator=self._schema_validator,
                ros_node=self,
            )
            self._react_plan_cache: Dict[str, Any] = {}
            self._react_plan_cache_max_entries = 64
        else:
            self._react_agent = None

        callback_group = ReentrantCallbackGroup()
        self._intent_subscriber = self.create_subscription(
            String,
            "/llm_intent",
            self.intent_callback,
            10,
            callback_group=callback_group,
        )
        self._text_input_subscriber = self.create_subscription(
            String,
            "/llm_text_input",
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
        self._react_joint_state_subscriber = self.create_subscription(
            JointState,
            "/yaskawa/joint_states",
            self._react_joint_state_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self._react_robot_status_subscriber = self.create_subscription(
            RobotStatus,
            "/yaskawa/robot_status",
            self._react_robot_status_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self._llm_debug_publisher = self.create_publisher(String, "/llm_debug", 10)
        self._status_publisher = self.create_publisher(String, "/gateway_status", 10)
        self._command_publisher = self.create_publisher(String, "/llm_command", 10)
        self._last_status = "ready"
        self._status_heartbeat_timer = None
        if self._status_heartbeat_period_sec > 0.0:
            self._status_heartbeat_timer = self.create_timer(
                self._status_heartbeat_period_sec,
                self._publish_status_heartbeat,
                callback_group=callback_group,
            )
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
        self._get_object_positions_client = self.create_client(
            GetObjectPositions,
            "/perception/get_object_positions",
            callback_group=callback_group,
        )

        # W5.T2 — HMI consolidation service servers
        self._hydrate_workplane_srv = self.create_service(
            HydrateWorkplane,
            "/llm_gateway/hydrate_workplane",
            self._on_hydrate_workplane,
            callback_group=callback_group,
        )
        self._get_primitive_constants_srv = self.create_service(
            GetPrimitiveConstants,
            "/llm_gateway/get_primitive_constants",
            self._on_get_primitive_constants,
            callback_group=callback_group,
        )
        self._review_intent_srv = self.create_service(
            ReviewIntent,
            "/llm_gateway/review_intent",
            self._on_review_intent,
            callback_group=callback_group,
        )
        self._confirm_execution_srv = self.create_service(
            ConfirmExecution,
            "/supervisor/confirm_execution",
            self._on_confirm_execution,
            callback_group=callback_group,
        )

        self.publish_status(self._last_status)
        self.get_logger().info(f"LLMGatewayNode ready (runtime_mode={runtime_mode}).")

    def _declare_parameters(self) -> None:
        self.declare_parameter("schema_path", "")
        self.declare_parameter("llm_config_path", "")
        self.declare_parameter("runtime_mode", "")
        self.declare_parameter("default_velocity_scale", 0.06)
        self.declare_parameter("default_acceleration_scale", 0.06)
        self.declare_parameter("safety_service_timeout_sec", 2.0)
        self.declare_parameter("status_heartbeat_period_sec", 5.0)
        self.declare_parameter("allow_direct_topic_execution", False)

    def _resolve_runtime_mode(self) -> str:
        runtime_mode = (
            self.get_parameter("runtime_mode")
            .get_parameter_value()
            .string_value.strip()
            .lower()
        )
        return runtime_mode or "hardware"

    def _load_react_enabled(self) -> bool:
        """Read llm.react.enabled from safety_rules.yaml SSOT."""
        from pathlib import Path

        import yaml
        from ament_index_python.packages import get_package_share_directory

        try:
            pkg_share = get_package_share_directory("safety")
            path = os.path.join(pkg_share, "config", "safety_rules.yaml")
        except Exception:
            path = str(
                Path(__file__).resolve().parents[2]
                / "safety"
                / "config"
                / "safety_rules.yaml"
            )
        try:
            with open(path, "r", encoding="utf-8") as f:
                rules = yaml.safe_load(f) or {}
            llm = rules.get("llm", {})
            react = llm.get("react", {})
            return bool(react.get("enabled", False))
        except Exception:
            return False

    def publish_status(self, status: str) -> None:
        if status != self._last_status:
            self.get_logger().info(f"gateway_status={status}")
        self._last_status = status
        self._status_publisher.publish(String(data=status))

    def _publish_status_heartbeat(self) -> None:
        self._status_publisher.publish(String(data=self._last_status))

    def intent_callback(self, msg: String) -> None:
        if not self._direct_topic_execution_enabled:
            self._reject_direct_topic_execution(
                "direct_text_topic_disabled",
                msg.data,
            )
            return
        self.process_intent(msg.data)

    def raw_command_callback(self, msg: String) -> None:
        if not self._direct_topic_execution_enabled:
            self._reject_direct_topic_execution(
                "direct_raw_topic_disabled",
                msg.data,
            )
            return
        self.process_raw_command(msg.data)

    def _reject_direct_topic_execution(self, stage: str, intent_text: str) -> None:
        self._reject(
            stage,
            "Direct gateway topic execution is disabled. Submit natural-language "
            "text through /llm_gateway/review_intent and the HMI supervisor "
            "confirmation flow.",
            intent_text=intent_text,
        )

    @staticmethod
    def _direct_review_semantic_ir(intent_text: str) -> Dict[str, Any] | None:
        normalized = " ".join(str(intent_text or "").strip().lower().split())
        normalized = normalized.strip(" .!?")
        if normalized in _DIRECT_STOP_REVIEW_TEXTS:
            return {"intent": "stop"}
        if normalized in {"home", "go home", "return home"}:
            return {"intent": "go_home"}
        if normalized in _DIRECT_GET_POSE_REVIEW_TEXTS:
            return {"intent": "get_pose", "reference_frame": "base_link"}

        wait_match = _DIRECT_WAIT_REVIEW_RE.fullmatch(normalized)
        if wait_match:
            duration_sec = float(wait_match.group(1))
            if duration_sec < 0.0:
                return {
                    "intent": "error",
                    "error": "wait duration must be non-negative",
                }
            return {"intent": "wait", "wait_duration_sec": duration_sec}
        return None

    def _generate_review_semantic_ir(self, intent_text: str) -> Dict[str, Any]:
        direct_review = self._direct_review_semantic_ir(intent_text)
        if direct_review is not None:
            return direct_review

        if self._react_enabled and self._react_agent is not None:
            react_result = self._react_agent.run(intent_text)
            if not react_result.get("_handoff"):
                return react_result
            reason = react_result.get("reason", "unknown")
            return {
                "error": "REACT_HANDOFF",
                "message": f"ReAct could not resolve the request: {reason}.",
                "hint": "Rephrase the command with clearer intent or check that all required parameters are provided.",
            }

        llm_response = self._llm_client.generate_response(intent_text)
        return self._parser.parse(llm_response)

    def _authorize_review_intent(self, request: ReviewIntent.Request) -> str:
        """Return an error string when the review request is not HMI-originated."""
        required_metadata = {
            "session_id": str(getattr(request, "session_id", "") or "").strip(),
            "operator_id": str(getattr(request, "operator_id", "") or "").strip(),
            "command_id": str(getattr(request, "command_id", "") or "").strip(),
        }
        missing = [name for name, value in required_metadata.items() if not value]
        if missing:
            return "ReviewIntent requires HMI metadata: " + ", ".join(missing)

        if not self._review_intent_token:
            return (
                "ReviewIntent review token is required; "
                "set GP4_REVIEW_INTENT_TOKEN for both HMI backend and llm_gateway."
            )

        provided = str(getattr(request, "review_token", "") or "").strip()
        expected = build_review_intent_token(
            shared_secret=self._review_intent_token,
            raw_text=str(getattr(request, "raw_text", "") or ""),
            runtime_mode=str(getattr(request, "runtime_mode", "") or ""),
            session_id=required_metadata["session_id"],
            operator_id=required_metadata["operator_id"],
            command_id=required_metadata["command_id"],
        )
        if not hmac.compare_digest(provided, expected):
            return "ReviewIntent review token mismatch."

        return ""

    def _on_review_intent(
        self,
        request: ReviewIntent.Request,
        response: ReviewIntent.Response,
    ) -> ReviewIntent.Response:
        intent_text = str(getattr(request, "raw_text", "") or "").strip()
        if not intent_text:
            response.accepted = False
            response.error = "raw_text is required for intent review."
            response.semantic_ir_json = ""
            return response

        authorization_error = self._authorize_review_intent(request)
        if authorization_error:
            response.accepted = False
            response.error = authorization_error
            response.semantic_ir_json = ""
            return response

        effective_runtime_mode = self._runtime_mode
        if effective_runtime_mode not in {"sim", "hardware"}:
            response.accepted = False
            response.error = (
                "gateway runtime_mode must be 'sim' or 'hardware' for intent review."
            )
            response.semantic_ir_json = ""
            return response

        requested_runtime_mode = str(getattr(request, "runtime_mode", "") or "").strip()
        if requested_runtime_mode != effective_runtime_mode:
            response.accepted = False
            response.error = (
                "ReviewIntent runtime_mode mismatch: "
                f"request={requested_runtime_mode or '<empty>'}, "
                f"gateway={effective_runtime_mode}."
            )
            response.semantic_ir_json = ""
            return response

        try:
            semantic_ir = self._generate_review_semantic_ir(intent_text)
        except Exception as exc:
            response.accepted = False
            response.error = str(exc)
            response.semantic_ir_json = ""
            return response

        if not isinstance(semantic_ir, dict):
            response.accepted = False
            response.error = "review result must be a JSON object."
            response.semantic_ir_json = ""
            return response

        contract = validate_semantic_ir_contract(semantic_ir)
        if not contract.valid:
            response.accepted = False
            response.error = contract.reason
            response.semantic_ir_json = json.dumps(
                {
                    "error": "SEMANTIC_IR_CONTRACT_REJECTED",
                    "reason": contract.reason,
                    "hint": contract.hint,
                },
                separators=(",", ":"),
                ensure_ascii=True,
            )
            return response

        response.semantic_ir_json = json.dumps(
            semantic_ir,
            separators=(",", ":"),
            ensure_ascii=True,
        )
        if "error" in semantic_ir:
            response.accepted = False
            response.error = str(semantic_ir.get("message") or semantic_ir["error"])
            return response
        if "intent" not in semantic_ir:
            response.accepted = False
            response.error = "review result must be Semantic IR with an intent field."
            return response
        if not self._semantic_ir_contains_intent(semantic_ir, "return_to_start"):
            try:
                routed = IntentRouter(runtime_mode=effective_runtime_mode).route(
                    semantic_ir
                )
            except Exception as exc:
                response.accepted = False
                response.error = str(exc)
                return response
            if routed.route_type == "error":
                error_payload = routed.error_payload or {}
                response.accepted = False
                response.error = str(
                    error_payload.get("message")
                    or error_payload.get("error")
                    or "review result was rejected by the intent router."
                )
                return response

        response.accepted = True
        response.error = ""
        return response

    def _react_joint_state_callback(self, msg: JointState) -> None:
        self._latest_joint_positions_rad = [float(value) for value in msg.position]
        self._react_state_injector.update_joint_states(
            {
                "name": list(msg.name),
                "position": list(self._latest_joint_positions_rad),
            }
        )

    @staticmethod
    def _semantic_ir_contains_intent(payload: Any, target_intent: str) -> bool:
        if isinstance(payload, dict):
            if str(payload.get("intent") or "").strip() == target_intent:
                return True
            return any(
                LLMGatewayNode._semantic_ir_contains_intent(value, target_intent)
                for value in payload.values()
            )
        if isinstance(payload, list):
            return any(
                LLMGatewayNode._semantic_ir_contains_intent(value, target_intent)
                for value in payload
            )
        return False

    @staticmethod
    def _inject_return_to_start_joints(
        payload: Dict[str, Any], start_joints_rad: List[float]
    ) -> Dict[str, Any]:
        enriched = dict(payload)
        intent = str(enriched.get("intent") or "").strip()
        if intent == "return_to_start" and "joint_target" not in enriched:
            if start_joints_rad:
                enriched["joint_target"] = [float(value) for value in start_joints_rad]
            return enriched
        if intent == "sequence":
            steps = enriched.get("steps")
            if isinstance(steps, list):
                enriched["steps"] = [
                    LLMGatewayNode._inject_return_to_start_joints(
                        step, start_joints_rad
                    )
                    if isinstance(step, dict)
                    else step
                    for step in steps
                ]
        return enriched

    def _react_robot_status_callback(self, msg: RobotStatus) -> None:
        if self._tri_state_is_true(msg.in_error) or self._tri_state_is_true(
            msg.e_stopped
        ):
            mode = "FAULT"
        elif self._tri_state_is_true(msg.in_motion):
            mode = "MOVING"
        elif self._tri_state_is_true(msg.motion_possible):
            mode = "IDLE"
        else:
            mode = "FAULT"
        self._react_state_injector.update_robot_status(
            {
                "mode": mode,
                "active_alarms": [str(code) for code in msg.error_codes],
            }
        )

    @staticmethod
    def _tri_state_is_true(value: Any) -> bool:
        return int(getattr(value, "val", TriState.UNKNOWN)) == int(TriState.TRUE)

    def process_intent(self, intent_text: str) -> None:
        self.publish_status("received")

        payload: str
        if self._react_enabled and self._react_agent is not None:
            try:
                react_result = self._react_agent.run(intent_text)
            except Exception as exc:
                self._reject("react_agent_failed", str(exc), intent_text=intent_text)
                return
            if react_result.get("_handoff"):
                self._reject(
                    "react_handoff",
                    f"ReAct could not resolve the request: {react_result.get('reason', 'unknown')}.",
                    intent_text=intent_text,
                    hint="Rephrase the command with clearer intent or check that all required parameters are provided.",
                )
                return
            payload = json.dumps(react_result)
        else:
            try:
                llm_response = self._llm_client.generate_response(intent_text)
            except Exception as exc:
                self._reject("llm_request_failed", str(exc), intent_text=intent_text)
                return
            self.publish_status("llm_response_received")
            payload = llm_response

        self._process_llm_payload(intent_text, payload, enforce_contract=True)

    def process_raw_command(self, raw_payload: str) -> None:
        self.publish_status("received")
        self._process_llm_payload("", raw_payload, enforce_contract=False)

    def _process_llm_payload(
        self,
        intent_text: str,
        raw_payload: str,
        *,
        enforce_contract: bool = True,
    ) -> None:
        try:
            parsed_command = self._parser.parse(raw_payload)
        except Exception as exc:
            self._reject(
                "llm_parse_failed",
                str(exc),
                intent_text=intent_text,
                raw_llm_output=raw_payload,
            )
            return

        try:
            parsed_command = self._hydrate_draw_workplane(parsed_command)
        except Exception as exc:
            self._reject(
                "workplane_resolution_failed",
                str(exc),
                intent_text=intent_text,
                parsed_command=parsed_command,
            )
            return

        if enforce_contract:
            contract = validate_semantic_ir_contract(parsed_command)
            if not contract.valid:
                self._reject(
                    "semantic_ir_contract_rejected",
                    contract.reason,
                    intent_text=intent_text,
                    hint=contract.hint,
                    parsed_command=parsed_command,
                )
                return

        parsed_command = self._inject_return_to_start_joints(
            parsed_command, list(getattr(self, "_latest_joint_positions_rad", []))
        )

        self.publish_status("parsed")
        try:
            routed_result = self._intent_router.route(parsed_command)
        except Exception as exc:
            self._reject(
                "intent_routing_failed",
                str(exc),
                intent_text=intent_text,
                parsed_command=parsed_command,
            )
            return

        self.publish_status("routed")

        if routed_result.route_type == "error":
            error_payload = routed_result.error_payload or {}
            reason = (
                error_payload.get("message")
                or error_payload.get("error")
                or "LLM returned an error payload."
            )
            self._reject(
                "unsupported_or_ambiguous",
                str(reason),
                intent_text=intent_text,
                parsed_command=parsed_command,
                validated_command=error_payload,
            )
            return

        if routed_result.route_type == "sequence":
            self._process_sequence(intent_text, parsed_command, routed_result.commands)
            return

        if len(routed_result.commands) != 1:
            self._reject(
                "intent_routing_failed",
                f"Expected exactly one primitive command, got {len(routed_result.commands)}.",
                intent_text=intent_text,
                parsed_command=parsed_command,
            )
            return

        self._process_single_command(
            intent_text, parsed_command, routed_result.commands[0]
        )

    def _normalize_and_validate(self, command: Dict[str, Any]) -> Dict[str, Any]:
        normalized_command = self._normalizer.normalize(command)
        self._semantic_validator.validate(normalized_command)
        return normalized_command

    def _process_single_command(
        self,
        intent_text: str,
        parsed_command: Dict[str, Any],
        routed_command: Dict[str, Any],
    ) -> None:
        try:
            self._schema_validator.validate(routed_command)
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
            normalized_command = self._normalize_and_validate(routed_command)
        except Exception as exc:
            self._reject(
                "semantic_validation_failed",
                str(exc),
                intent_text=intent_text,
                parsed_command=parsed_command,
            )
            return

        self.publish_status("semantic_valid")
        self._dispatch_normalized_command(normalized_command, intent_text)

    def _process_sequence(
        self,
        intent_text: str,
        parsed_command: Dict[str, Any],
        routed_commands: List[Dict[str, Any]],
    ) -> None:
        try:
            sequence_result = self._sequence_validator.validate(routed_commands)
        except Exception as exc:
            self._reject(
                "sequence_validation_failed",
                str(exc),
                intent_text=intent_text,
                parsed_command=parsed_command,
            )
            return

        self.publish_status("sequence_valid")
        self._publish_debug(
            {
                "status": "sequence_valid",
                "stage": "sequence_validation",
                "intent": intent_text,
                "step_count": sequence_result.step_count,
                "validated_reference_frame": sequence_result.validated_reference_frame,
                "cumulative_move_rel_distance_m": sequence_result.cumulative_move_rel_distance_m,
                "estimated_duration_lower_bound_sec": sequence_result.estimated_duration_lower_bound_sec,
                "duration_estimate_is_lower_bound": sequence_result.duration_estimate_is_lower_bound,
                "has_io_side_effects": sequence_result.has_io_side_effects,
                "manual_recovery_required_on_failure": (
                    sequence_result.manual_recovery_required_on_failure
                ),
                "diagnostics": sequence_result.diagnostics,
            }
        )
        sequence_state = _SequenceExecutionState(
            intent_text=intent_text,
            normalized_commands=list(sequence_result.normalized_commands),
            step_count=sequence_result.step_count,
            diagnostics=list(sequence_result.diagnostics),
            start_joints_rad=list(getattr(self, "_latest_joint_positions_rad", [])),
        )
        self._dispatch_sequence_step(sequence_state)

    def _dispatch_sequence_step(self, sequence_state: _SequenceExecutionState) -> None:
        if sequence_state.current_step_index >= sequence_state.step_count:
            self.publish_status("sequence_succeeded")
            self._publish_debug(
                {
                    "status": "sequence_succeeded",
                    "stage": "sequence_execution",
                    "intent": sequence_state.intent_text,
                    "step_count": sequence_state.step_count,
                }
            )
            return

        current_step_number = sequence_state.current_step_index + 1
        self.publish_status(
            f"sequence_step:{current_step_number}/{sequence_state.step_count}"
        )
        normalized_command = sequence_state.normalized_commands[
            sequence_state.current_step_index
        ]
        self._dispatch_normalized_command(
            normalized_command,
            sequence_state.intent_text,
            sequence_state=sequence_state,
        )

    def _dispatch_normalized_command(
        self,
        normalized_command: Dict[str, Any],
        intent_text: str,
        *,
        sequence_state: _SequenceExecutionState | None = None,
    ) -> None:
        primitive_type = normalized_command.get("primitive_type", "")

        if sequence_state is None and self._is_query_command(primitive_type):
            self._publish_command(
                self._goal_mapper.to_command_payload(normalized_command)
            )
            self._handle_get_pose_query(normalized_command, intent_text)
            return

        if sequence_state is not None and self._is_query_command(primitive_type):
            self._reject_sequence_step(
                sequence_state,
                "GET_POSE is query-only and cannot execute inside sequences.",
                validated_command=self._goal_mapper.to_command_payload(
                    normalized_command
                ),
            )
            return

        command_payload = self._goal_mapper.to_command_payload(normalized_command)

        if not self._validate_client.wait_for_service(
            timeout_sec=self._safety_service_timeout_sec
        ):
            if sequence_state is None:
                self._reject(
                    "validate_service_unavailable",
                    "ValidateCommand service unavailable",
                    intent_text=intent_text,
                    validated_command=command_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    "ValidateCommand service unavailable",
                    validated_command=command_payload,
                )
            return

        request = self._build_validate_request(normalized_command, command_payload)
        if sequence_state is None:
            self.publish_status("safety_validation_requested")
        validation_future = self._validate_client.call_async(request)
        validation_future.add_done_callback(
            lambda future,
            intent=intent_text,
            payload=command_payload,
            sequence=sequence_state: (
                self._on_validation_done(future, intent, payload, sequence)
            )
        )

    def _is_query_command(self, primitive_type: str) -> bool:
        """Return True for query-only commands that bypass the motion action pipeline."""
        return primitive_type == "GET_POSE"

    def _handle_get_pose_query(
        self, normalized_command: Dict[str, Any], intent_text: str
    ) -> None:
        """Route GET_POSE to the dedicated query service — NO motion action path."""
        if not self._get_pose_client.wait_for_service(
            timeout_sec=self._safety_service_timeout_sec
        ):
            self._reject(
                "get_pose_service_unavailable",
                "GetCurrentPose service unavailable",
                intent_text=intent_text,
            )
            return

        request = GetCurrentPose.Request()
        request.reference_frame = str(
            normalized_command.get("reference_frame", "base_link")
        )

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
        self._publish_debug(
            {
                "status": "query_result",
                "stage": "get_pose",
                "intent": intent_text,
                "message": response.message,
                "current_pose": pose_data,
            }
        )
        self.publish_status("query_succeeded")

    def _build_validate_request(
        self, normalized_command: Dict[str, Any], command_payload: Dict[str, Any]
    ) -> ValidateCommand.Request:
        request = ValidateCommand.Request()
        request.command_json = json.dumps(
            command_payload, ensure_ascii=True, separators=(",", ":")
        )
        request.primitive_type = normalized_command["primitive_type"]
        request.velocity_scale = normalized_command.get("velocity_scale", 0.0)
        if "target_pose_msg" in normalized_command:
            request.target_pose = normalized_command["target_pose_msg"]
        return request

    def _on_validation_done(
        self,
        future: Any,
        intent_text: str,
        command_payload: Dict[str, Any],
        sequence_state: _SequenceExecutionState | None = None,
    ) -> None:
        try:
            response = future.result()
        except Exception as exc:
            if sequence_state is None:
                self._reject(
                    "validate_service_call_failed",
                    str(exc),
                    intent_text=intent_text,
                    validated_command=command_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    str(exc),
                    validated_command=command_payload,
                )
            return

        if not response.valid:
            if sequence_state is None:
                self._reject(
                    "rejected_by_validate_service",
                    str(response.reason),
                    intent_text=intent_text,
                    validated_command=command_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    str(response.reason),
                    validated_command=command_payload,
                )
            return

        try:
            validated_command = self._command_from_sanitized_json(
                response.sanitized_json, command_payload
            )
            normalized_command = self._normalize_and_validate(validated_command)
        except Exception as exc:
            if sequence_state is None:
                self._reject(
                    "sanitized_command_invalid",
                    str(exc),
                    intent_text=intent_text,
                    validated_command=command_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    str(exc),
                    validated_command=command_payload,
                )
            return

        if normalized_command.get("plan_only") or command_payload.get("plan_only"):
            precheck_payload = {
                "status": "plan_precheck_succeeded",
                "stage": "plan_only",
                "intent": intent_text,
                "validated_command": self._goal_mapper.to_command_payload(
                    normalized_command
                ),
            }
            if sequence_state is not None:
                precheck_payload["sequence_step_index"] = (
                    sequence_state.current_step_index
                )
                precheck_payload["sequence_step_count"] = sequence_state.step_count
            self._publish_debug(precheck_payload)
            if sequence_state is None:
                self.publish_status("plan_precheck_succeeded")
                return
            sequence_state.current_step_index += 1
            self._dispatch_sequence_step(sequence_state)
            return

        try:
            execution_command = self._prepare_execution_command(normalized_command)
        except Exception as exc:
            if sequence_state is None:
                self._reject(
                    "prepare_execution_failed",
                    str(exc),
                    intent_text=intent_text,
                    validated_command=command_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    str(exc),
                    validated_command=command_payload,
                )
            return

        goal_payload = self._goal_mapper.to_command_payload(execution_command)
        self._publish_command(goal_payload)
        validated_debug_payload = {
            "status": "validated",
            "stage": "validate_command",
            "intent": intent_text,
            "validated_command": goal_payload,
        }
        if sequence_state is not None:
            validated_debug_payload["sequence_step_index"] = (
                sequence_state.current_step_index
            )
            validated_debug_payload["sequence_step_count"] = sequence_state.step_count
        self._publish_debug(validated_debug_payload)

        if sequence_state is None:
            self.publish_status("safety_approved")

        if not self._execute_client.server_is_ready():
            if sequence_state is None:
                self._reject(
                    "execute_motion_unavailable",
                    "ExecuteMotion action server unavailable",
                    intent_text=intent_text,
                    validated_command=goal_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    "ExecuteMotion action server unavailable",
                    validated_command=goal_payload,
                )
            return

        goal = self._goal_mapper.to_execute_motion_goal(execution_command)
        send_future = self._execute_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f,
            intent=intent_text,
            payload=goal_payload,
            sequence=sequence_state: (self._on_goal_sent(f, intent, payload, sequence))
        )
        if sequence_state is None:
            self.publish_status("dispatched")

    def _on_goal_sent(
        self,
        future: Any,
        intent_text: str,
        goal_payload: Dict[str, Any],
        sequence_state: _SequenceExecutionState | None = None,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if sequence_state is None:
                self._reject(
                    "execute_motion_send_failed",
                    str(exc),
                    intent_text=intent_text,
                    validated_command=goal_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    str(exc),
                    validated_command=goal_payload,
                )
            return
        if not goal_handle or not goal_handle.accepted:
            if sequence_state is None:
                self._reject(
                    "execute_motion_rejected",
                    "ExecuteMotion action server rejected goal",
                    intent_text=intent_text,
                    validated_command=goal_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    "ExecuteMotion action server rejected goal",
                    validated_command=goal_payload,
                )
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f,
            intent=intent_text,
            payload=goal_payload,
            sequence=sequence_state: (
                self._on_execution_done(f, intent, payload, sequence)
            )
        )

    def _on_execution_done(
        self,
        future: Any,
        intent_text: str,
        goal_payload: Dict[str, Any],
        sequence_state: _SequenceExecutionState | None = None,
    ) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            if sequence_state is None:
                self._reject(
                    "execute_motion_result_error",
                    str(exc),
                    intent_text=intent_text,
                    validated_command=goal_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    str(exc),
                    validated_command=goal_payload,
                )
            return
        if wrapped.result and wrapped.result.success:
            result_message = wrapped.result.message or ""
            self.get_logger().info(f"Execution succeeded: {result_message}")
            if sequence_state is None:
                if "READY_FOR_CONFIRM" in result_message:
                    self.publish_status("ready_for_confirm")
                self.publish_status("succeeded")
                self._publish_debug(
                    {
                        "status": "succeeded",
                        "stage": "execute_motion",
                        "intent": intent_text,
                        "message": result_message,
                    }
                )
                return

            if goal_payload.get("primitive_type") == "IO_SET":
                sequence_state.executed_io_side_effects = True
            sequence_state.current_step_index += 1
            self._dispatch_sequence_step(sequence_state)
        else:
            msg = wrapped.result.message if wrapped.result else "no result"
            if sequence_state is None:
                self._reject(
                    "execute_motion_failed",
                    msg,
                    intent_text=intent_text,
                    validated_command=goal_payload,
                )
            else:
                self._reject_sequence_step(
                    sequence_state,
                    msg,
                    validated_command=goal_payload,
                )

    def _prepare_execution_command(
        self, normalized_command: Dict[str, Any]
    ) -> Dict[str, Any]:
        return _pipeline_prepare_execution_command(
            normalized_command,
            logger=self.get_logger(),
        )

    def _command_from_sanitized_json(
        self, sanitized_json: str, fallback_payload: Dict[str, Any]
    ) -> Dict[str, Any]:
        # Thin wrapper: supplies the node's schema_validator to the pure helper.
        return _pipeline_command_from_sanitized_json(
            sanitized_json, fallback_payload, self._schema_validator
        )

    def _wait_for_future_without_spinning(
        self, future: Any, timeout_sec: float
    ) -> tuple[bool, Any | None]:
        done = getattr(future, "done", None)
        if callable(done) and done():
            return True, future.result()

        completed = threading.Event()
        add_done_callback = getattr(future, "add_done_callback", None)
        if not callable(add_done_callback):
            return False, None

        add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout_sec):
            return False, None
        return True, future.result()

    def _query_perception_detections(self, args: Dict[str, Any]) -> Dict[str, Any]:
        client = getattr(self, "_get_object_positions_client", None)
        if client is None or not client.service_is_ready():
            return {
                "ok": False,
                "error": "perception_service_not_ready",
                "payload": None,
            }

        request = GetObjectPositions.Request()
        request.class_filter = str(args.get("class_filter") or "")
        future = client.call_async(request)
        done, response = self._wait_for_future_without_spinning(
            future, self._safety_service_timeout_sec
        )
        if not done or response is None:
            return {
                "ok": False,
                "error": "perception_service_timeout",
                "payload": None,
            }
        if not response.ok:
            return {
                "ok": False,
                "error": response.failure_reason or "perception_query_rejected",
                "payload": None,
            }
        calibration_valid = bool(getattr(response, "calibration_valid", False))
        depth_in_range = bool(getattr(response, "depth_in_range", False))
        depth_noise_mm_p95 = float(getattr(response, "depth_noise_mm_p95", 0.0))
        calibration_payload = {
            "valid": calibration_valid,
            "date": getattr(response, "calibration_date_iso", ""),
            "age_days": float(getattr(response, "calibration_age_days", 0.0)),
        }
        if not calibration_valid:
            return {
                "ok": False,
                "error": "calibration_invalid",
                "payload": {"calibration": calibration_payload},
            }
        if not depth_in_range:
            return {
                "ok": False,
                "error": "depth_quality_invalid",
                "payload": {
                    "calibration": calibration_payload,
                    "depth_in_range": depth_in_range,
                    "depth_noise_mm_p95": depth_noise_mm_p95,
                },
            }
        return {
            "ok": True,
            "error": None,
            "payload": {
                "detections": [
                    self._serialize_detection(detection)
                    for detection in response.detections
                ],
                "calibration": calibration_payload,
                "depth_in_range": depth_in_range,
                "depth_noise_mm_p95": depth_noise_mm_p95,
                "stamp": {
                    "sec": int(response.stamp.sec),
                    "nanosec": int(response.stamp.nanosec),
                },
            },
        }

    @staticmethod
    def _serialize_detection(detection: Any) -> Dict[str, Any]:
        result = detection.results[0] if getattr(detection, "results", []) else None
        hypothesis = getattr(result, "hypothesis", None)
        pose_container = getattr(result, "pose", None)
        pose = getattr(pose_container, "pose", pose_container)
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        size = getattr(getattr(detection, "bbox", None), "size", None)
        return {
            "class_id": str(getattr(hypothesis, "class_id", "")),
            "score": float(getattr(hypothesis, "score", 0.0)),
            "frame_id": str(
                getattr(getattr(detection, "header", None), "frame_id", "")
            ),
            "position": {
                "x": float(getattr(position, "x", 0.0)),
                "y": float(getattr(position, "y", 0.0)),
                "z": float(getattr(position, "z", 0.0)),
            },
            "orientation": {
                "x": float(getattr(orientation, "x", 0.0)),
                "y": float(getattr(orientation, "y", 0.0)),
                "z": float(getattr(orientation, "z", 0.0)),
                "w": float(getattr(orientation, "w", 1.0)),
            },
            "size": {
                "x": float(getattr(size, "x", 0.0)),
                "y": float(getattr(size, "y", 0.0)),
                "z": float(getattr(size, "z", 0.0)),
            },
        }

    def _reject_sequence_step(
        self,
        sequence_state: _SequenceExecutionState,
        reason: str,
        *,
        validated_command: Dict[str, Any] | None = None,
    ) -> None:
        failed_step_index = sequence_state.current_step_index
        failed_command = sequence_state.normalized_commands[failed_step_index]
        manual_recovery_required = sequence_state.executed_io_side_effects
        self.get_logger().warning(
            f"sequence_step_failed step={failed_step_index + 1}/{sequence_state.step_count}: {reason}"
        )
        self._publish_debug(
            {
                "status": "rejected",
                "stage": "sequence_step_failed",
                "reason": reason,
                "intent": sequence_state.intent_text,
                "failed_step_index": failed_step_index,
                "failed_primitive_type": failed_command.get("primitive_type", ""),
                "validated_command": validated_command,
                "manual_recovery_required": manual_recovery_required,
                "sequence_diagnostics": sequence_state.diagnostics,
            }
        )
        if manual_recovery_required:
            self.publish_status("manual_recovery_required")
        self.publish_status("rejected:sequence_step_failed")

    def _reject(
        self,
        stage: str,
        reason: str,
        *,
        intent_text: str = "",
        raw_llm_output: str | None = None,
        parsed_command: Dict[str, Any] | None = None,
        validated_command: Dict[str, Any] | None = None,
        hint: str = "",
    ) -> None:
        self.get_logger().warning(f"{stage}: {reason}")
        debug_payload: Dict[str, Any] = {
            "status": "rejected",
            "stage": stage,
            "reason": reason,
            "intent": intent_text,
            "raw_llm_output": raw_llm_output,
            "parsed_command": parsed_command,
            "validated_command": validated_command,
        }
        if hint:
            debug_payload["hint"] = hint
        self._publish_debug(debug_payload)
        self.publish_status(f"rejected:{stage}")

    def _publish_debug(self, payload: Dict[str, Any]) -> None:
        compact_payload = {
            key: value
            for key, value in payload.items()
            if value not in (None, "", [], {})
        }
        self._llm_debug_publisher.publish(
            String(
                data=json.dumps(
                    compact_payload, ensure_ascii=True, separators=(",", ":")
                )
            )
        )

    def _publish_command(self, command_payload: Dict[str, Any]) -> None:
        self._command_publisher.publish(
            String(
                data=json.dumps(
                    command_payload, ensure_ascii=True, separators=(",", ":")
                )
            )
        )

    def _hydrate_draw_workplane(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        # Thin wrapper: injects the ROS-coupled pose-snapshot fetcher.
        return _pipeline_hydrate_draw_workplane(
            payload,
            fetch_current_pose=self._request_current_pose_snapshot,
        )

    # ── W5.T2 service handlers ────────────────────────────────────────────

    def _on_hydrate_workplane(
        self,
        request: HydrateWorkplane.Request,
        response: HydrateWorkplane.Response,
    ) -> HydrateWorkplane.Response:
        try:
            payload = json.loads(request.payload_json)
        except json.JSONDecodeError as exc:
            response.success = False
            response.error = f"invalid payload_json: {exc}"
            return response

        try:
            hydrated = _pipeline_hydrate_draw_workplane(
                payload,
                fetch_current_pose=self._request_current_pose_snapshot,
            )
            response.success = True
            response.hydrated_payload_json = json.dumps(
                hydrated, ensure_ascii=True, separators=(",", ":")
            )
        except Exception as exc:
            response.success = False
            response.error = str(exc)

        return response

    def _on_get_primitive_constants(
        self,
        request: GetPrimitiveConstants.Request,
        response: GetPrimitiveConstants.Response,
    ) -> GetPrimitiveConstants.Response:
        # Static primitive constants — canonical source.
        # Mirrors hmi/backend/services/intent_constants.py.
        # When these change, update both this handler and intent_constants.py.
        _SUPPORTED_PRIMITIVES = sorted(
            [
                "HOME",
                "PTP",
                "LIN",
                "CIRC",
                "CARTESIAN_PATH",
                "MOVE_REL",
                "MOVE_JOINT",
                "MOVE_JOINTS",
                "WAIT",
                "STOP",
                "SET_SPEED",
                "IO_SET",
                "ALARM_RESET",
                "GET_POSE",
            ]
        )
        _PLANNER_DEFAULTS = {
            "HOME": "PILZ_PTP",
            "PTP": "PILZ_PTP",
            "LIN": "PILZ_LIN",
            "CIRC": "PILZ_CIRC",
            "CARTESIAN_PATH": "PILZ_LIN",
            "MOVE_REL": "PILZ_LIN",
            "MOVE_JOINT": "PILZ_PTP",
            "MOVE_JOINTS": "PILZ_PTP",
        }
        _ALLOWED_FIELDS_BY_PRIMITIVE = {
            "HOME": sorted(
                [
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                    "reference_frame",
                ]
            ),
            "PTP": sorted(
                [
                    "target_pose",
                    "joint_target",
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                    "reference_frame",
                ]
            ),
            "LIN": sorted(
                [
                    "target_pose",
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                    "reference_frame",
                ]
            ),
            "CIRC": sorted(
                [
                    "target_pose",
                    "waypoints",
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                    "reference_frame",
                ]
            ),
            "CARTESIAN_PATH": sorted(
                [
                    "waypoints",
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                    "reference_frame",
                ]
            ),
            "MOVE_REL": sorted(
                [
                    "delta_x",
                    "delta_y",
                    "delta_z",
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                    "reference_frame",
                ]
            ),
            "MOVE_JOINT": sorted(
                [
                    "joint_index",
                    "joint_angle",
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                ]
            ),
            "MOVE_JOINTS": sorted(
                [
                    "joint_target",
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                ]
            ),
            "WAIT": sorted(["wait_duration_sec", "reference_frame"]),
            "STOP": sorted(["reference_frame"]),
            "SET_SPEED": sorted(["velocity_scale"]),
            "IO_SET": sorted(["io_address", "io_value", "reference_frame"]),
            "ALARM_RESET": sorted(["reference_frame"]),
            "GET_POSE": sorted(["reference_frame"]),
        }
        _OLD_ACTIONS = {
            "move_home": "HOME",
            "home": "HOME",
            "stop": "STOP",
            "move_rel": "MOVE_REL",
            "move_cartesian_delta": "MOVE_REL",
            "move_joint": "MOVE_JOINT",
            "move_joint_delta": "MOVE_JOINT",
            "move_joints": "MOVE_JOINTS",
            "wait": "WAIT",
            "set_speed": "SET_SPEED",
            "io_set": "IO_SET",
            "alarm_reset": "ALARM_RESET",
            "get_pose": "GET_POSE",
            "ptp": "PTP",
            "lin": "LIN",
            "circ": "CIRC",
            "cartesian_path": "CARTESIAN_PATH",
        }

        try:
            constants = {
                "SUPPORTED_PRIMITIVES": _SUPPORTED_PRIMITIVES,
                "PLANNER_DEFAULTS": _PLANNER_DEFAULTS,
                "_ALLOWED_FIELDS_BY_PRIMITIVE": _ALLOWED_FIELDS_BY_PRIMITIVE,
                "_OLD_ACTIONS": _OLD_ACTIONS,
            }
            response.success = True
            response.constants_json = json.dumps(
                constants, ensure_ascii=True, separators=(",", ":")
            )
        except Exception as exc:
            response.success = False
            response.error = str(exc)

        return response

    def _on_confirm_execution(
        self,
        request: ConfirmExecution.Request,
        response: ConfirmExecution.Response,
    ) -> ConfirmExecution.Response:
        # Re-validate the parsed intent against current runtime state.
        # This is a stateless gate: the HMI owns the state machine;
        # this service only answers "is execution safe right now?"
        try:
            parsed_intent = json.loads(request.parsed_intent_json)
        except json.JSONDecodeError as exc:
            response.accepted = False
            response.reason = f"invalid parsed_intent_json: {exc}"
            return response

        if not self._validate_client.wait_for_service(
            timeout_sec=self._safety_service_timeout_sec
        ):
            response.accepted = False
            response.reason = "ValidateCommand service unavailable"
            return response

        command_payload = self._goal_mapper.to_command_payload(parsed_intent)
        validate_req = ValidateCommand.Request()
        validate_req.command_json = json.dumps(
            command_payload, ensure_ascii=True, separators=(",", ":")
        )
        validate_req.primitive_type = parsed_intent.get("primitive_type", "")
        validate_req.velocity_scale = parsed_intent.get("velocity_scale", 0.0)

        try:
            validate_future = self._validate_client.call_async(validate_req)
            done, validate_resp = self._wait_for_future_without_spinning(
                validate_future, self._safety_service_timeout_sec
            )
            if not done:
                response.accepted = False
                response.reason = "ValidateCommand service timed out"
                return response
        except Exception as exc:
            response.accepted = False
            response.reason = f"ValidateCommand call failed: {exc}"
            return response

        if not validate_resp.valid:
            response.accepted = False
            response.reason = str(validate_resp.reason)
            return response

        response.accepted = True
        response.execution_summary = (
            f"Command {request.command_id} confirmed by {request.operator_id}; "
            f"fingerprint {request.plan_fingerprint[:12]}..."
        )
        response.dispatched_to_ros = False
        return response

    def _request_current_pose_snapshot(
        self, reference_frame: str
    ) -> Dict[str, Any] | None:
        if not self._get_pose_client.wait_for_service(
            timeout_sec=self._safety_service_timeout_sec
        ):
            return None

        request = GetCurrentPose.Request()
        request.reference_frame = reference_frame
        future = self._get_pose_client.call_async(request)
        done, response = self._wait_for_future_without_spinning(
            future, self._safety_service_timeout_sec
        )

        if not done:
            return None
        if response is None or not response.success:
            return None

        pose = response.current_pose
        return {
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


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = LLMGatewayNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=EXECUTOR_SHUTDOWN_TIMEOUT_SEC)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
