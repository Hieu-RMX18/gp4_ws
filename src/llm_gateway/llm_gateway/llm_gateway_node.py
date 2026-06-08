#!/usr/bin/env python3

"""ROS2 node for the phase-9 LLM gateway pipeline."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import json
import math
import os
import time
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
    GP4_JOINT_NAMES,
    GoalMapper,
    IntentRouter,
    LLMParser,
    Normalizer,
    SchemaValidator,
    SemanticValidator,
    SequenceValidator,
    load_srdf_named_poses as _load_srdf_named_poses,
    command_from_sanitized_json as _pipeline_command_from_sanitized_json,
    hydrate_draw_workplane as _pipeline_hydrate_draw_workplane,
    prepare_execution_command as _pipeline_prepare_execution_command,
    prepare_semantic_ir_for_routing as _pipeline_prepare_semantic_ir_for_routing,
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
from llm_gateway.composite_tools import (
    EmitSequenceTool,
    GripperConfig,
    GripperIoAdapter,
    PickObjectTool,
    RefreshSceneTool,
)
from llm_gateway.semantic_ir_contract import validate_semantic_ir_contract


EXECUTOR_SHUTDOWN_TIMEOUT_SEC = 2.0
_DIRECT_STOP_REVIEW_TEXTS = {"stop", "stop motion", "cancel motion", "halt"}
_REVIEW_CACHE_VERSION = "react_semantic_review_v1"
_REVIEW_CACHE_MAX_ENTRIES = 128


@dataclass
class _SequenceExecutionState:
    intent_text: str
    normalized_commands: List[Dict[str, Any]]
    step_count: int
    diagnostics: List[str] = field(default_factory=list)
    start_joints_rad: List[float] = field(default_factory=list)
    current_step_index: int = 0
    executed_io_side_effects: bool = False


class _SceneSnapshotCache:
    def __init__(self, ttl_sec: float, now_fn=time.monotonic):
        self._ttl_sec = float(ttl_sec)
        self._now_fn = now_fn
        self._entries: Dict[tuple[str, str], tuple[float, Dict[str, Any]]] = {}

    def get(self, args: Dict[str, Any]) -> Dict[str, Any] | None:
        key = self._key(args)
        entry = self._entries.get(key)
        if entry is None:
            return None
        stamp, payload = entry
        if self._now_fn() - stamp > self._ttl_sec:
            self._entries.pop(key, None)
            return None
        cached = dict(payload)
        cached["cache_hit"] = True
        return cached

    def store(self, args: Dict[str, Any], payload: Dict[str, Any]) -> None:
        stored = dict(payload)
        stored["cache_hit"] = False
        self._entries[self._key(args)] = (self._now_fn(), stored)

    def invalidate(self) -> None:
        self._entries.clear()

    @staticmethod
    def _key(args: Dict[str, Any]) -> tuple[str, str]:
        return (
            str(args.get("class_filter") or ""),
            str(args.get("frame") or "base_link"),
        )


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
        self._gripper_adapter = GripperIoAdapter(
            config=GripperConfig.from_rules(
                getattr(self._semantic_validator, "_safety_rules", {})
            ),
            node=self,
            robot_mode_fn=self._current_react_robot_mode,
        )
        self._goal_mapper = goal_mapper or GoalMapper(
            default_velocity_scale=self._default_velocity_scale,
            default_acceleration_scale=self._default_acceleration_scale,
        )
        runtime_mode = self._resolve_runtime_mode()
        self._runtime_mode = runtime_mode
        self._review_intent_requires_token = False
        self._intent_router = intent_router or IntentRouter(runtime_mode=runtime_mode)
        self._sequence_validator = sequence_validator or SequenceValidator(
            schema_validator=self._schema_validator,
            normalizer=self._normalizer,
            semantic_validator=self._semantic_validator,
        )
        self._latest_joint_positions_rad: List[float] = []
        self._latest_joint_positions_by_name_rad: Dict[str, float] = {}
        self._latest_pose_by_frame: Dict[str, Dict[str, Any]] = {}
        self._current_pose_cache_ttl_sec = 5.0
        self._semantic_review_cache: Dict[str, Dict[str, Any]] = {}
        self._scene_snapshot_cache = _SceneSnapshotCache(ttl_sec=2.0)
        llm_backend_config = load_llm_backend_config(llm_config_path)
        self._llm_client = llm_client or OpenAICompatibleLLMClient(
            llm_backend_config, self._schema_validator.schema_as_json()
        )

        # ── ReAct agent init (W3) ─────────────────────────────────────────────
        self._react_enabled = self._load_react_enabled()
        self._react_state_injector = StateInjector()
        try:
            self._react_state_injector.set_available_named_poses(
                list(_load_srdf_named_poses().keys())
            )
        except Exception:
            pass
        if self._react_enabled:
            tool_registry = (
                ToolRegistry()
                .register(GetCurrentPoseTool())
                .register(PlanMotionTool())
                .register(SubmitMotionTool())
                .register(WaitForStateTool())
                .register(SetSpeedTool())
                .register(QueryPerceptionTool())
                .register(EmitSequenceTool())
                .register(RefreshSceneTool())
                .register(PickObjectTool())
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
        self._react_joint_state_fallback_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
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
    def _direct_review_semantic_ir(
        intent_text: str,
        *,
        allow_sequence: bool = True,
        runtime_mode: str = "",
    ) -> Dict[str, Any] | None:
        _ = allow_sequence, runtime_mode
        normalized = " ".join(str(intent_text or "").strip().lower().split())
        normalized = normalized.strip(" .!?")
        if normalized in _DIRECT_STOP_REVIEW_TEXTS:
            return {"intent": "stop"}
        return None

    @staticmethod
    def _validate_draw_params(semantic_ir: Dict[str, Any]) -> str | None:
        """Return an error string if required draw params are missing, None if valid."""
        intent = str(semantic_ir.get("intent", "")).strip()
        if intent == "draw_shape":
            shape = str(semantic_ir.get("shape_type", "")).strip().lower()
            params = semantic_ir.get("params") or {}
            if not isinstance(params, dict):
                return "draw_shape params must be an object."
            if shape == "circle":
                has_radius = any(k in params for k in ("radius", "radius_m"))
                has_diameter = any(k in params for k in ("diameter", "diameter_m", "size", "size_m"))
                if not has_radius and not has_diameter:
                    return "draw_shape circle requires params.radius or params.diameter"
            if shape in {"polygon", "polyline"}:
                has_points = any(k in params for k in ("points", "vertices"))
                has_size = any(
                    k in params
                    for k in ("n_sides", "sides", "radius", "radius_m", "side", "side_m", "side_m")
                )
                if not has_points and not has_size:
                    return f"draw_shape {shape} requires params with size or points"
                if shape == "polygon" and not has_points:
                    has_n_sides = any(k in params for k in ("n_sides", "sides"))
                    if not has_n_sides:
                        return "draw_shape polygon requires params.n_sides"
            if shape == "arc":
                has_radius = any(k in params for k in ("radius", "radius_m"))
                if not has_radius:
                    return "draw_shape arc requires params.radius"
            if shape in {"square", "rectangle", "triangle"}:
                has_explicit = any(k in params for k in ("points", "vertices"))
                if not has_explicit:
                    if shape == "rectangle":
                        has_width = any(k in params for k in ("width_m", "width"))
                        has_height = any(k in params for k in ("height_m", "height"))
                        if not has_width:
                            return "draw_shape rectangle requires params.width"
                        if not has_height:
                            return "draw_shape rectangle requires params.height"
                    else:
                        side_keys = ("side_m", "side", "size_m", "size")
                        has_side = any(k in params for k in side_keys)
                        if not has_side:
                            return f"draw_shape {shape} requires params.side or explicit points"
        elif intent == "draw_text":
            text = str(semantic_ir.get("text", "")).strip()
            if not text:
                return "draw_text requires a non-empty text string."
            font = semantic_ir.get("font") or {}
            if not isinstance(font, dict):
                return "draw_text font must be an object."
            height_keys = ("height_m", "height", "char_height_m")
            has_height = any(k in semantic_ir for k in height_keys) or any(
                k in font for k in height_keys
            )
            if not has_height:
                return "draw_text requires font.height."
        return None

    def _generate_review_semantic_ir(self, intent_text: str) -> Dict[str, Any]:
        cached_review = self._get_semantic_review_cache(intent_text)
        if cached_review is not None:
            return cached_review

        direct_review = self._direct_review_semantic_ir(
            intent_text, runtime_mode=self._runtime_mode
        )
        if direct_review is not None:
            self._emit_trace(
                "direct_pre_parsed",
                "parsing",
                source="direct_fast_path",
                summary=str(direct_review.get("intent") or "")[:80],
            )
            validated = dict(direct_review)
            validated["_parse_source"] = "direct_fast_path"
            return validated

        if self._react_enabled and self._react_agent is not None:
            react_result = self._react_agent.run(intent_text)
            if not react_result.get("_handoff"):
                enriched_result = dict(react_result)
                enriched_result["_parse_source"] = "react"
                return enriched_result
            reason = react_result.get("reason", "unknown")
            return {
                "error": "REACT_HANDOFF",
                "message": f"ReAct could not resolve the request: {reason}.",
                "hint": "Rephrase the command with clearer intent or check that all required parameters are provided.",
            }

        self._emit_trace("llm_request_started", "reasoning", source="llm")
        llm_response = self._llm_client.generate_response(intent_text)
        self._emit_trace("llm_response_received", "reasoning", source="llm")
        parsed = self._parser.parse(llm_response)
        parsed["_parse_source"] = "llm"
        self._emit_trace("parsed", "parsing", source="llm")
        return parsed

    def _review_cache_key(self, intent_text: str) -> str:
        normalized = " ".join(str(intent_text or "").strip().lower().split())
        return f"{_REVIEW_CACHE_VERSION}|{self._runtime_mode}|{normalized}"

    def _get_semantic_review_cache(self, intent_text: str) -> Dict[str, Any] | None:
        cache = getattr(self, "_semantic_review_cache", None)
        if not isinstance(cache, dict):
            return None
        cached = cache.get(self._review_cache_key(intent_text))
        if not isinstance(cached, dict):
            return None
        result = deepcopy(cached)
        result["_parse_source"] = "semantic_cache"
        self._emit_trace(
            "semantic_cache_hit",
            "parsing",
            source="semantic_cache",
            summary=str(result.get("intent") or "")[:80],
        )
        return result

    def _store_semantic_review_cache(
        self, intent_text: str, semantic_ir: Dict[str, Any]
    ) -> None:
        if not self._semantic_ir_cacheable(semantic_ir):
            return
        cache = getattr(self, "_semantic_review_cache", None)
        if not isinstance(cache, dict):
            cache = {}
            self._semantic_review_cache = cache
        stored = self._strip_metadata_fields(semantic_ir)
        cache[self._review_cache_key(intent_text)] = stored
        while len(cache) > _REVIEW_CACHE_MAX_ENTRIES:
            oldest_key = next(iter(cache))
            cache.pop(oldest_key, None)

    @staticmethod
    def _semantic_ir_cacheable(payload: Any) -> bool:
        if not isinstance(payload, dict) or "error" in payload:
            return False
        intent = str(payload.get("intent") or "").strip()
        if intent in {"draw_shape", "draw_text"}:
            return True
        if intent == "sequence":
            steps = payload.get("steps")
            return isinstance(steps, list) and all(
                LLMGatewayNode._semantic_ir_cacheable(step) for step in steps
            )
        return False

    @staticmethod
    def _strip_metadata_fields(payload: Any) -> Any:
        if isinstance(payload, dict):
            return {
                key: LLMGatewayNode._strip_metadata_fields(value)
                for key, value in payload.items()
                if not str(key).startswith("_")
            }
        if isinstance(payload, list):
            return [LLMGatewayNode._strip_metadata_fields(value) for value in payload]
        return deepcopy(payload)

    def _resolve_tool_relative_review_move(
        self, payload: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Convert tool0-relative Semantic IR nudges into base_link deltas.

        The safety and motion layers only accept MOVE_REL in base_link, so
        tool-frame relative movement must be grounded in the current tool0
        orientation before intent review succeeds.
        """
        intent = str(payload.get("intent", "")).strip()
        if intent == "sequence":
            steps = payload.get("steps")
            if isinstance(steps, list):
                resolved = dict(payload)
                resolved["steps"] = [
                    self._resolve_tool_relative_review_move(step)
                    if isinstance(step, dict)
                    else step
                    for step in steps
                ]
                return resolved
            return payload

        if intent != "move_relative" or payload.get("reference_frame") != "tool0":
            return payload

        current_pose = self._get_cached_current_pose_snapshot("base_link")
        if current_pose is None:
            request_pose = getattr(self, "_request_current_pose_snapshot", None)
            if callable(request_pose):
                current_pose = request_pose("base_link")
        if current_pose is None:
            return {
                "error": "CURRENT_POSE_UNAVAILABLE",
                "message": (
                    "current pose unavailable for tool0-relative move; "
                    "cannot transform forward/back/left/right into base_link"
                ),
                "hint": "Verify /get_current_pose is available and returns base_link pose.",
            }

        try:
            base_delta = self._rotate_tool_delta_to_base(
                payload.get("delta", {}), current_pose
            )
        except ValueError as exc:
            return {
                "error": "CURRENT_POSE_INVALID",
                "message": str(exc),
                "hint": "Verify /get_current_pose returns a finite unit quaternion.",
            }

        resolved = dict(payload)
        resolved["delta"] = base_delta
        resolved["reference_frame"] = "base_link"
        return resolved

    def _resolve_move_joint_delta(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Convert move_joint_delta (relative) to move_joint (absolute) using current state."""
        try:
            return _pipeline_prepare_semantic_ir_for_routing(
                payload,
                current_joint_positions_rad=list(
                    getattr(self, "_latest_joint_positions_rad", [])
                ),
                current_joint_positions_by_name=getattr(
                    self, "_latest_joint_positions_by_name_rad", {}
                ),
            )
        except ValueError as exc:
            message = str(exc)
            if "current joint positions unavailable" in message:
                error = "CURRENT_JOINT_STATE_UNAVAILABLE"
                hint = "Verify /yaskawa/joint_states or /joint_states is publishing."
            else:
                error = "MOVE_JOINT_DELTA_RESOLUTION_FAILED"
                hint = "Verify joint index/alias and delta_angle are valid."
            return {
                "error": error,
                "message": message,
                "hint": hint,
            }

    def _prepare_review_semantic_ir(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        prepared = self._canonicalize_semantic_ir_aliases(payload)
        if not isinstance(prepared, dict):
            return prepared
        prepared = self._resolve_tool_relative_review_move(prepared)
        if isinstance(prepared, dict) and "error" in prepared:
            return prepared
        return self._resolve_move_joint_delta(prepared)

    @staticmethod
    def _review_error_message(payload: Dict[str, Any]) -> str:
        if (
            payload.get("error") == "MISSING_SLOT"
            and payload.get("intent") == "move_relative"
        ):
            return "relative move requires direction and distance."
        reason = str(payload.get("message") or payload.get("error") or "")
        if reason == "move_relative requires delta.":
            return "relative move requires direction and distance."
        return reason

    @staticmethod
    def _review_exception_message(exc: Exception) -> str:
        message = str(exc)
        if message == "move_relative requires delta.":
            return "relative move requires direction and distance."
        return message

    @staticmethod
    def _rotate_tool_delta_to_base(
        delta: Any, current_pose: Dict[str, Any]
    ) -> Dict[str, float]:
        """Rotate tool-relative delta by current quaternion orientation.
        
        Uses standard quaternion rotation formula: v' = v + 2*cross(q_xyz, cross(q_xyz, v) + qw*v)
        """
        if not isinstance(delta, dict):
            raise ValueError("tool-relative move requires a delta object")
        orientation = current_pose.get("orientation")
        if not isinstance(orientation, dict):
            raise ValueError("current pose is missing orientation")
        vx = float(delta.get("x", 0.0))
        vy = float(delta.get("y", 0.0))
        vz = float(delta.get("z", 0.0))
        qx = float(orientation.get("x", 0.0))
        qy = float(orientation.get("y", 0.0))
        qz = float(orientation.get("z", 0.0))
        qw = float(orientation.get("w", 1.0))
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or norm <= 1e-9:
            raise ValueError("current pose orientation quaternion is invalid")
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        # Cross product: q_xyz x v
        cross_x = qy * vz - qz * vy
        cross_y = qz * vx - qx * vz
        cross_z = qx * vy - qy * vx

        # Add qw * v to cross product
        add_x = cross_x + qw * vx
        add_y = cross_y + qw * vy
        add_z = cross_z + qw * vz

        # Cross product: q_xyz x (cross + qw*v)
        cross2_x = qy * add_z - qz * add_y
        cross2_y = qz * add_x - qx * add_z
        cross2_z = qx * add_y - qy * add_x

        # Final: v' = v + 2 * cross2
        return {
            "x": vx + 2.0 * cross2_x,
            "y": vy + 2.0 * cross2_y,
            "z": vz + 2.0 * cross2_z,
        }

    def _authorize_review_intent(self, request: ReviewIntent.Request) -> str:
        """Return an error string when required HMI metadata is missing."""
        required_metadata = {
            "session_id": str(getattr(request, "session_id", "") or "").strip(),
            "operator_id": str(getattr(request, "operator_id", "") or "").strip(),
            "command_id": str(getattr(request, "command_id", "") or "").strip(),
        }
        missing = [name for name, value in required_metadata.items() if not value]
        if missing:
            return "ReviewIntent requires HMI metadata: " + ", ".join(missing)

        return ""

    def _on_review_intent(
        self,
        request: ReviewIntent.Request,
        response: ReviewIntent.Response,
    ) -> ReviewIntent.Response:
        self._active_command_id = str(getattr(request, "command_id", "") or "").strip()
        intent_text = str(getattr(request, "raw_text", "") or "").strip()
        self._emit_trace("prompt_received", "ingress", summary=intent_text[:80])
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

        semantic_ir = self._prepare_review_semantic_ir(semantic_ir)
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
            response.error = self._review_error_message(semantic_ir)
            return response
        if "intent" not in semantic_ir:
            response.accepted = False
            response.error = "review result must be Semantic IR with an intent field."
            return response
        draw_err = self._validate_draw_params(semantic_ir)
        if draw_err:
            response.accepted = False
            response.error = draw_err
            response.semantic_ir_json = json.dumps(
                {"error": "SEMANTIC_IR_DRAW_PARAMS_MISSING", "reason": draw_err},
                separators=(",", ":"),
                ensure_ascii=True,
            )
            return response
        if (
            not self._semantic_ir_contains_intent(semantic_ir, "return_to_start")
            and not self._semantic_ir_contains_any_intent(
                semantic_ir, {"draw_shape", "draw_text"}
            )
        ):
            try:
                routed = IntentRouter(runtime_mode=effective_runtime_mode).route(
                    semantic_ir
                )
            except Exception as exc:
                response.accepted = False
                response.error = self._review_exception_message(exc)
                return response
            if routed.route_type == "error":
                error_payload = routed.error_payload or {}
                response.accepted = False
                response.error = self._review_error_message(
                    {
                        "error": error_payload.get("error"),
                        "message": (
                            error_payload.get("message")
                            or "review result was rejected by the intent router."
                        ),
                        "intent": error_payload.get("intent"),
                    }
                )
                return response

        response.accepted = True
        response.error = ""
        self._store_semantic_review_cache(intent_text, semantic_ir)
        return response

    def _react_joint_state_callback(self, msg: JointState) -> None:
        raw_positions = [float(value) for value in msg.position]
        by_name = {
            str(name): raw_positions[index]
            for index, name in enumerate(msg.name)
            if index < len(raw_positions)
        }
        if all(joint_name in by_name for joint_name in GP4_JOINT_NAMES):
            self._latest_joint_positions_rad = [
                by_name[joint_name] for joint_name in GP4_JOINT_NAMES
            ]
        else:
            self._latest_joint_positions_rad = raw_positions
        self._latest_joint_positions_by_name_rad = dict(by_name)
        self._react_state_injector.update_joint_states(
            {
                "name": (
                    list(GP4_JOINT_NAMES)
                    if len(self._latest_joint_positions_rad) == len(GP4_JOINT_NAMES)
                    else list(msg.name)
                ),
                "position": list(self._latest_joint_positions_rad),
            }
        )

    @staticmethod
    def _canonicalize_semantic_ir_aliases(payload: Any) -> Any:
        if isinstance(payload, dict):
            canonical = {
                key: LLMGatewayNode._canonicalize_semantic_ir_aliases(value)
                for key, value in payload.items()
            }
            if canonical.get("intent") == "move_to_named_pose":
                canonical["intent"] = "move_named_pose"
            return canonical
        if isinstance(payload, list):
            return [LLMGatewayNode._canonicalize_semantic_ir_aliases(value) for value in payload]
        return payload

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
    def _semantic_ir_contains_any_intent(
        payload: Any, target_intents: set[str]
    ) -> bool:
        if isinstance(payload, dict):
            if str(payload.get("intent") or "").strip() in target_intents:
                return True
            return any(
                LLMGatewayNode._semantic_ir_contains_any_intent(value, target_intents)
                for value in payload.values()
            )
        if isinstance(payload, list):
            return any(
                LLMGatewayNode._semantic_ir_contains_any_intent(value, target_intents)
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
        if mode != "IDLE":
            self._invalidate_scene_cache()

    def _invalidate_scene_cache(self) -> None:
        self._get_scene_snapshot_cache().invalidate()

    def _get_scene_snapshot_cache(self) -> _SceneSnapshotCache:
        cache = getattr(self, "_scene_snapshot_cache", None)
        if cache is None:
            cache = _SceneSnapshotCache(ttl_sec=2.0)
            self._scene_snapshot_cache = cache
        return cache

    def _current_react_robot_mode(self) -> str:
        snapshot = self._react_state_injector.snapshot()
        robot_state = snapshot.get("robot_state", {}) if isinstance(snapshot, dict) else {}
        return str(robot_state.get("mode") or "IDLE")

    @staticmethod
    def _tri_state_is_true(value: Any) -> bool:
        return int(getattr(value, "val", TriState.UNKNOWN)) == int(TriState.TRUE)

    def _emit_trace(
        self,
        event: str,
        phase: str,
        level: str = "INFO",
        summary: str = "",
        command_id: str = "",
        source: str = "",
        **details,
    ) -> None:
        """Publish structured command trace event to /llm_debug."""
        import json
        import time
        trace_payload = {
            "t": "command_trace",
            "ts": time.time(),
            "cmd_id": command_id or getattr(self, "_active_command_id", ""),
            "layer": "llm_gateway",
            "phase": phase,
            "event": event,
            "level": level,
            "summary": summary,
            "details": details if details else None,
        }
        if source:
            trace_payload["source"] = source
        payload = json.dumps(trace_payload)
        if hasattr(self, "_llm_debug_publisher") and self._llm_debug_publisher is not None:
            self._llm_debug_publisher.publish(String(data=payload))

    def process_intent(self, intent_text: str) -> None:
        self.publish_status("received")
        self._emit_trace("prompt_received", "ingress", summary=intent_text[:80])

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
                self._emit_trace("llm_request_started", "reasoning")
                llm_response = self._llm_client.generate_response(intent_text)
                self._emit_trace("llm_response_received", "reasoning")
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
            self._emit_trace("parsed", "parsing",
                             summary=f"intent={parsed_command.get('intent', '?')} raw_len={len(raw_payload)}",
                             intent=str(parsed_command.get('intent', '')),
                             primitive_type=str(parsed_command.get('primitive_type', '')),
                             raw_preview=raw_payload[:120])
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

        parsed_command = self._prepare_review_semantic_ir(parsed_command)
        self._emit_trace("canonicalized", "parsing",
                         summary=f"Aliases resolved → intent={parsed_command.get('intent', '?')}")
        if isinstance(parsed_command, dict) and "error" in parsed_command:
            self._reject(
                "semantic_ir_preparation_failed",
                self._review_error_message(parsed_command),
                intent_text=intent_text,
                hint=str(parsed_command.get("hint", "")),
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
            self._emit_trace("contract_validated", "validation",
                             summary="Semantic IR contract check PASSED")

        parsed_command = self._inject_return_to_start_joints(
            parsed_command, list(getattr(self, "_latest_joint_positions_rad", []))
        )

        self.publish_status("parsed")
        self._emit_trace("schema_validated", "validation")
        try:
            routed_result = self._intent_router.route(parsed_command)
        except Exception as exc:
            self._reject(
                "intent_routing_failed",
                self._review_exception_message(exc),
                intent_text=intent_text,
                parsed_command=parsed_command,
            )
            return

        self.publish_status("routed")
        self._emit_trace("routed", "routing", summary=f"route={routed_result.route_type}")

        if routed_result.route_type == "error":
            error_payload = routed_result.error_payload or {}
            reason = (
                error_payload.get("message")
                or error_payload.get("error")
                or "LLM returned an error payload."
            )
            reason = self._review_error_message(
                {
                    "error": error_payload.get("error"),
                    "message": reason,
                    "intent": error_payload.get("intent"),
                }
            )
            self._reject(
                "unsupported_or_ambiguous",
                reason,
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
        # Emit detailed normalization trace
        prim = normalized_command.get('primitive_type', '?')
        vel = normalized_command.get('velocity_scale', '?')
        acc = normalized_command.get('acceleration_scale', '?')
        frame = normalized_command.get('reference_frame', '?')
        tp = normalized_command.get('target_pose', {})
        pos_str = ""
        if isinstance(tp, dict) and 'position' in tp:
            p = tp['position']
            pos_str = f"target=({p.get('x','?')}, {p.get('y','?')}, {p.get('z','?')})"
        delta_str = ""
        dx = normalized_command.get('delta_x')
        if dx is not None:
            dy = normalized_command.get('delta_y', 0)
            dz = normalized_command.get('delta_z', 0)
            delta_str = f"delta=({dx}, {dy}, {dz})"
        self._emit_trace(
            "normalized", "normalization",
            summary=f"{prim} vel={vel} acc={acc} frame={frame} {pos_str}{delta_str}",
            primitive_type=prim,
            velocity_scale=str(vel),
            acceleration_scale=str(acc),
            reference_frame=str(frame),
        )
        self._semantic_validator.validate(normalized_command)
        self._emit_trace("semantic_validated", "validation",
                         summary=f"Semantic checks PASSED for {prim} (units, frames, limits OK)")
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
        prim = command_payload.get('primitive_type', '?')
        self._emit_trace(
            "safety_gate_request", "validation",
            summary=f"Sending {prim} to /validate_command service",
            source="safety",
            primitive_type=prim,
            velocity_scale=str(command_payload.get('velocity_scale', '')),
            command_json_len=str(len(request.command_json)),
        )
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
            lambda f, intent=intent_text, frame=request.reference_frame: self._on_get_pose_done(
                f, intent, frame
            )
        )

    def _on_get_pose_done(
        self, future: Any, intent_text: str, reference_frame: str = "base_link"
    ) -> None:
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
        pose_data = self._cache_current_pose_snapshot(reference_frame, pose)

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

        self._emit_trace(
            "safety_validated",
            "validation",
            source="safety",
            summary="ValidateCommand accepted command",
            primitive_type=str(command_payload.get("primitive_type") or ""),
            has_sanitized_json=bool(getattr(response, "sanitized_json", "")),
            sequence_step_index=(
                sequence_state.current_step_index
                if sequence_state is not None
                else None
            ),
        )

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
        prim = goal_payload.get('primitive_type', '?')
        # Emit current pose at dispatch time for before/after comparison
        cur_pose_str = ""
        cached = self._get_cached_current_pose_snapshot("base_link")
        if cached:
            cp = cached
            cur_pose_str = f"current_pos=({cp.get('position',{}).get('x','?')}, {cp.get('position',{}).get('y','?')}, {cp.get('position',{}).get('z','?')})"
        self._emit_trace(
            "goal_dispatched", "dispatch",
            summary=f"Sending {prim} to ExecuteMotion action server {cur_pose_str}",
            source="motion_core",
            primitive_type=prim,
            planner_id=str(goal_payload.get('planner_id', '')),
        )
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
            self._emit_trace(
                "execution_result",
                "execution",
                source="motion_core",
                summary=result_message[:160],
                primitive_type=str(goal_payload.get("primitive_type") or ""),
                planner_id=str(goal_payload.get("planner_id") or ""),
                success=True,
            )
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
            self._emit_trace(
                "execution_result",
                "execution",
                level="ERROR",
                source="motion_core",
                summary=msg[:160],
                primitive_type=str(goal_payload.get("primitive_type") or ""),
                planner_id=str(goal_payload.get("planner_id") or ""),
                success=False,
            )
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

        add_done_callback = getattr(future, "add_done_callback", None)
        if not callable(add_done_callback):
            return False, None

        result_container: list[Any] = [None]
        def _resolve(f):
            try:
                result_container[0] = f.result()
            except Exception as exc:
                result_container[0] = exc

        add_done_callback(_resolve)
        # Use a short-poll loop so the executor callback that completes the
        # future can run on another thread while we yield the GIL.
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            if result_container[0] is not None:
                if isinstance(result_container[0], Exception):
                    return False, None
                return True, result_container[0]
            time.sleep(0.01)
        return False, None

    def _query_perception_detections(self, args: Dict[str, Any]) -> Dict[str, Any]:
        scene_cache = self._get_scene_snapshot_cache()
        cached_payload = scene_cache.get(args)
        if cached_payload is not None:
            return {"ok": True, "error": None, "payload": cached_payload}

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
        payload = {
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
        }
        scene_cache.store(args, payload)
        return {
            "ok": True,
            "error": None,
            "payload": payload,
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
        self._emit_trace(f"rejected_{stage}", "error", level="ERROR", summary=reason, error_code=stage, error_why=reason, error_next_action=hint)
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

    def _cache_current_pose_snapshot(
        self, reference_frame: str, pose: Any
    ) -> Dict[str, Any]:
        frame = str(reference_frame or "base_link")
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
        if not hasattr(self, "_latest_pose_by_frame"):
            self._latest_pose_by_frame = {}
        self._latest_pose_by_frame[frame] = {
            "pose": pose_data,
            "timestamp": time.monotonic(),
        }
        return pose_data

    def _get_cached_current_pose_snapshot(
        self, reference_frame: str
    ) -> Dict[str, Any] | None:
        frame = str(reference_frame or "base_link")
        cache = getattr(self, "_latest_pose_by_frame", {})
        entry = cache.get(frame) if isinstance(cache, dict) else None
        if not isinstance(entry, dict):
            return None
        timestamp = entry.get("timestamp")
        pose = entry.get("pose")
        ttl = float(getattr(self, "_current_pose_cache_ttl_sec", 5.0))
        if not isinstance(timestamp, (int, float)) or time.monotonic() - timestamp > ttl:
            return None
        if not isinstance(pose, dict):
            return None
        position = pose.get("position")
        orientation = pose.get("orientation")
        if not isinstance(position, dict) or not isinstance(orientation, dict):
            return None
        return {
            "position": dict(position),
            "orientation": dict(orientation),
        }

    def _request_current_pose_snapshot(
        self, reference_frame: str
    ) -> Dict[str, Any] | None:
        cached_pose = self._get_cached_current_pose_snapshot(reference_frame)
        if cached_pose is not None:
            return cached_pose
        if not self._get_pose_client.wait_for_service(
            timeout_sec=self._safety_service_timeout_sec
        ):
            self.get_logger().error(
                f"ReAct get_current_pose: service /get_current_pose "
                f"not available within {self._safety_service_timeout_sec}s"
            )
            return None

        request = GetCurrentPose.Request()
        request.reference_frame = reference_frame
        future = self._get_pose_client.call_async(request)
        done, response = self._wait_for_future_without_spinning(
            future, self._safety_service_timeout_sec
        )

        if not done:
            self.get_logger().error(
                f"ReAct get_current_pose: async call timed out "
                f"after {self._safety_service_timeout_sec}s"
            )
            return None
        if response is None or not response.success:
            self.get_logger().error(
                f"ReAct get_current_pose: response "
                f"{'is None' if response is None else 'not success'}"
            )
            return None

        return self._cache_current_pose_snapshot(reference_frame, response.current_pose)


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
