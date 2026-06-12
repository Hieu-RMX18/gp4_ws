#!/usr/bin/env python3

"""ROS2 node for the phase-9 LLM gateway pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import subprocess
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

from llm_gateway import direct_commands
from llm_gateway.factory_task import (
    GP4_JOINT_NAMES,
    GoalMapper,
    IntentRouter,
    LLMParser,
    Normalizer,
    SchemaValidator,
    SemanticValidator,
    SequenceValidator,
    canonicalize_named_pose,
    load_srdf_named_poses as _load_srdf_named_poses,
    command_from_sanitized_json as _pipeline_command_from_sanitized_json,
    hydrate_draw_workplane as _pipeline_hydrate_draw_workplane,
    prepare_execution_command as _pipeline_prepare_execution_command,
    prepare_semantic_ir_for_routing as _pipeline_prepare_semantic_ir_for_routing,
    RuntimeStepResult,
    TaskCompiler,
    TaskRuntime,
    WorldModel,
    is_factory_task,
    parse_factory_task,
)
from llm_gateway.task_planner import (
    OpenAICompatibleLLMClient,
    StateInjector,
    TaskPlanner,
    load_llm_backend_config,
)
from motoros2_interfaces.srv import ReadSingleIO, WriteSingleIO
from llm_gateway.gripper_adapter import (
    GripperConfig,
    GripperIoAdapter,
)
from llm_gateway.semantic_ir_contract import (
    FACTORY_TASK_RUNTIME_INTENT,
    is_factory_task_runtime_sentinel,
    validate_semantic_ir_contract,
)


EXECUTOR_SHUTDOWN_TIMEOUT_SEC = 2.0
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

    def snapshots(self) -> List[Dict[str, Any]]:
        snapshots: List[Dict[str, Any]] = []
        for class_filter, frame in list(self._entries.keys()):
            payload = self.get({"class_filter": class_filter, "frame": frame})
            if payload is not None:
                snapshots.append(payload)
        return snapshots

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
        self._motion_result_timeout_sec = (
            self.get_parameter("motion_result_timeout_sec")
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
            robot_mode_fn=self._current_planner_robot_mode,
        )
        self._goal_mapper = goal_mapper or GoalMapper(
            default_velocity_scale=self._default_velocity_scale,
            default_acceleration_scale=self._default_acceleration_scale,
        )
        runtime_mode = self._resolve_runtime_mode()
        self._runtime_mode = runtime_mode
        self._code_version = self._resolve_code_version()
        self._review_intent_requires_token = False
        self._station_scene_graph = self._load_station_scene_graph_safe()
        semantic_map = self._station_scene_graph.to_dict() if self._station_scene_graph else None
        self._intent_router = intent_router or IntentRouter(
            runtime_mode=runtime_mode,
            station_semantic_map=semantic_map
        )
        self._sequence_validator = sequence_validator or SequenceValidator(
            schema_validator=self._schema_validator,
            normalizer=self._normalizer,
            semantic_validator=self._semantic_validator,
        )
        self._latest_joint_positions_rad: List[float] = []
        self._latest_joint_positions_by_name_rad: Dict[str, float] = {}
        self._latest_pose_by_frame: Dict[str, Dict[str, Any]] = {}
        self._current_pose_cache_ttl_sec = 5.0
        self._scene_snapshot_cache = _SceneSnapshotCache(ttl_sec=2.0)
        llm_backend_config = load_llm_backend_config(llm_config_path)
        self._llm_client = llm_client or OpenAICompatibleLLMClient(
            llm_backend_config, self._schema_validator.schema_as_json()
        )

        # ── Task planner init ────────────────────────────────────────────────
        self._planner_enabled = self._load_planner_enabled()
        self._state_injector = StateInjector()
        poses = []
        try:
            poses.extend(list(_load_srdf_named_poses().keys()))
        except Exception:
            pass
        if self._station_scene_graph is not None:
            self._state_injector.set_semantic_map(self._station_scene_graph.to_dict())
            if hasattr(self._station_scene_graph, "_regions"):
                poses.extend(self._station_scene_graph._regions.keys())
        self._state_injector.set_available_named_poses(poses)
        if self._planner_enabled:
            self._task_planner = TaskPlanner(
                llm_client=self._llm_client,
                state_injector=self._state_injector,
                schema_validator=self._schema_validator,
                max_repair=1,
            )
        else:
            self._task_planner = None

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
        self._state_joint_state_subscriber = self.create_subscription(
            JointState,
            "/yaskawa/joint_states",
            self._state_joint_state_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self._state_joint_state_fallback_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self._state_joint_state_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self._state_robot_status_subscriber = self.create_subscription(
            RobotStatus,
            "/yaskawa/robot_status",
            self._state_robot_status_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self._llm_debug_publisher = self.create_publisher(String, "/llm_debug", 10)
        self._status_publisher = self.create_publisher(String, "/gateway_status", 10)
        self._command_publisher = self.create_publisher(String, "/llm_command", 10)
        self._task_events_pub = self.create_publisher(String, "/llm_gateway/task_events", 10)
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
        self._write_single_io_client = self.create_client(
            WriteSingleIO,
            self._gripper_adapter._config.write_single_io_service,
            callback_group=callback_group,
        )
        self._read_single_io_client = self.create_client(
            ReadSingleIO,
            self._gripper_adapter._config.read_single_io_service,
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

        self._init_runtime_stop_state()
        self.publish_status(self._last_status)
        self.get_logger().info(f"LLMGatewayNode ready (runtime_mode={runtime_mode}).")

    @staticmethod
    def _load_station_scene_graph_safe():
        try:
            from ament_index_python.packages import get_package_share_directory
            from llm_gateway.factory_task import StationSceneGraph
            pkg_share = get_package_share_directory("llm_gateway")
            path = os.path.join(pkg_share, "config", "station_semantic_map.yaml")
            return StationSceneGraph.from_file(path)
        except Exception:
            return None

    def _declare_parameters(self) -> None:
        self.declare_parameter("schema_path", "")
        self.declare_parameter("llm_config_path", "")
        self.declare_parameter("runtime_mode", "")
        self.declare_parameter("default_velocity_scale", 0.06)
        self.declare_parameter("default_acceleration_scale", 0.06)
        self.declare_parameter("safety_service_timeout_sec", 2.0)
        self.declare_parameter("motion_result_timeout_sec", 30.0)
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

    @staticmethod
    def _resolve_code_version() -> str:
        """Git short hash for the running code, or 'unknown' if unavailable."""
        try:
            result = subprocess.run(
                ["git", "rev-parse", "--short", "HEAD"],
                cwd=str(Path(__file__).resolve().parent),
                capture_output=True,
                text=True,
                timeout=2.0,
                check=True,
            )
            return result.stdout.strip() or "unknown"
        except Exception:
            return "unknown"

    def _stamp_review_provenance(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Attach parse source and code version to review payloads without mutating input."""
        if not isinstance(payload, dict):
            return payload
        stamped = dict(payload)
        stamped.setdefault("_parse_source", "unknown")
        stamped["_code_version"] = getattr(self, "_code_version", "unknown")
        return stamped

    def _load_planner_enabled(self) -> bool:
        """Read llm.react.enabled for backward-compatible planner gating."""
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
        direct_review = direct_commands.parse(intent_text)
        if direct_review is not None:
            self._emit_trace(
                "direct_pre_parsed",
                "parsing",
                source="direct",
                summary=str(direct_review.get("intent") or "")[:80],
            )
            validated = dict(direct_review)
            validated["_parse_source"] = "direct"
            return validated

        if self._planner_enabled and self._task_planner is not None:
            self._emit_trace("llm_request_started", "reasoning", source="task_planner")
            planner_result = self._task_planner.plan(intent_text)
            self._emit_trace("llm_response_received", "reasoning", source="task_planner")
            if is_factory_task(planner_result):
                return self._compile_factory_task_review_result(
                    planner_result, parse_source="llm_factory_task"
                )
            if "error" in planner_result:
                enriched_result = dict(planner_result)
                enriched_result["_parse_source"] = "llm"
                return enriched_result
            return {
                "error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND",
                "message": "planner returned neither FactoryTask nor error.",
                "hint": "Rephrase the command with clearer intent or required parameters.",
            }

        self._emit_trace("llm_request_started", "reasoning", source="llm")
        llm_response = self._llm_client.generate_response(intent_text)
        self._emit_trace("llm_response_received", "reasoning", source="llm")
        parsed = self._parser.parse(llm_response)
        if is_factory_task(parsed):
            return self._compile_factory_task_review_result(
                parsed, parse_source="llm_factory_task"
            )
        parsed["_parse_source"] = "llm"
        self._emit_trace("parsed", "parsing", source="llm")
        return parsed

    def _compile_factory_task_review_result(
        self, payload: Dict[str, Any], *, parse_source: str
    ) -> Dict[str, Any]:
        task = parse_factory_task(payload)
        runtime_plan = self._factory_task_runtime_plan(payload)
        semantic_ir: Dict[str, Any] = {
            "intent": FACTORY_TASK_RUNTIME_INTENT,
            "metadata": {
                "factory_task": {
                    "task_id": task.task_id,
                    "version": task.version,
                    "mode": task.mode,
                    "operator_summary": task.operator_summary,
                    "limits": dict(task.limits),
                    "replan_policy": dict(task.replan_policy),
                },
                "runtime_plan": runtime_plan,
                "policy_decisions": self._factory_task_review_policy_decisions(
                    runtime_plan
                ),
            },
            "_parse_source": parse_source,
            "_factory_task_runtime": True,
        }
        self._emit_trace(
            "factory_task_runtime_plan",
            "planning",
            source=parse_source,
            summary=task.task_id[:80],
            details_json=json.dumps(semantic_ir.get("metadata") or {}),
        )
        return semantic_ir

    @staticmethod
    def _factory_task_review_policy_decisions(
        runtime_plan: Dict[str, Any]
    ) -> List[Dict[str, Any]]:
        decisions: List[Dict[str, Any]] = []

        def _walk(node: Any, path: str) -> None:
            if not isinstance(node, dict):
                return
            node_type = str(node.get("type") or "")
            node_name = str(node.get("name") or node_type)
            decisions.append(
                {
                    "node_path": path,
                    "node_type": node_type,
                    "node_name": node_name,
                    "decision": "allow",
                    "reason": (
                        "review preserves FactoryTask for TaskRuntime; "
                        "runtime controls such as retry/fallback/replan remain live; "
                        "motion remains behind validation, operator confirmation, "
                        "and execution guards"
                    ),
                    "risk_level": "medium" if node_type == "skill" else "low",
                }
            )
            for index, child in enumerate(node.get("children") or []):
                _walk(child, f"{path}.children[{index}]")

        _walk(runtime_plan, "root")
        return decisions

    def _prime_factory_task_world_model(self, payload: Dict[str, Any]) -> None:
        for object_ref in self._factory_task_grounding_object_refs(payload):
            try:
                self._factory_task_world_model().object_pose(object_ref)
                continue
            except Exception:
                pass
            query = {"class_filter": object_ref, "frame": "base_link"}
            result = self._query_perception_detections(query)
            if result.get("ok") and isinstance(result.get("payload"), dict):
                self._get_scene_snapshot_cache().store(query, result["payload"])

    @staticmethod
    def _factory_task_grounding_object_refs(payload: Dict[str, Any]) -> tuple[str, ...]:
        refs: List[str] = []

        def _walk(node: Any) -> None:
            if not isinstance(node, dict):
                return
            args = node.get("args") if isinstance(node.get("args"), dict) else {}
            if node.get("type") == "skill" and node.get("name") in {
                "move_to_object", "pick_object", "pick_and_place",
            }:
                value = args.get("object_ref") or args.get("object") or args.get("object_id")
                if isinstance(value, str):
                    object_ref = value.strip()
                    if object_ref and not object_ref.startswith("$") and object_ref not in refs:
                        refs.append(object_ref)
            for child in node.get("children") or []:
                _walk(child)

        _walk(payload.get("root"))
        return tuple(refs)

    @staticmethod
    def _factory_task_runtime_plan(payload: Dict[str, Any]) -> Dict[str, Any]:
        """Extract the runtime plan tree from the FactoryTask payload for HMI visibility."""
        root = payload.get("root") or {}
        def _plan_node(node: Any) -> Dict[str, Any]:
            if not isinstance(node, dict):
                return {"type": "unknown"}
            plan: Dict[str, Any] = {"type": node.get("type", "")}
            if node.get("name"):
                plan["name"] = node["name"]
            if node.get("args"):
                plan["args"] = dict(node["args"])
            if node.get("count") is not None:
                plan["count"] = node["count"]
            if node.get("collection"):
                plan["collection"] = node["collection"]
                plan["item_name"] = node.get("item_name", "item")
            if node.get("replan_policy"):
                plan["replan_policy"] = dict(node["replan_policy"])
            children = node.get("children") or []
            if children:
                plan["children"] = [_plan_node(c) for c in children]
            return plan
        return _plan_node(root)

    def _factory_task_world_model(self) -> WorldModel:
        objects: Dict[str, Dict[str, Any]] = {}
        visible_objects: List[Dict[str, Any]] = []
        for snapshot in self._get_scene_snapshot_cache().snapshots():
            for detection in snapshot.get("detections", []):
                grounded = self._factory_task_object_from_detection(detection)
                if grounded is None:
                    continue
                visible_objects.append(grounded)
                for key in self._factory_task_object_keys(grounded):
                    objects.setdefault(key, grounded)
        return WorldModel(
            objects=objects,
            collections={"visible_objects": visible_objects},
        )

    @staticmethod
    def _factory_task_object_from_detection(
        detection: Dict[str, Any]
    ) -> Dict[str, Any] | None:
        if not isinstance(detection, dict):
            return None
        class_id = str(detection.get("class_id") or "").strip()
        if not class_id:
            return None
        position = detection.get("position")
        if not isinstance(position, dict):
            return None
        orientation = detection.get("orientation")
        if not isinstance(orientation, dict):
            orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        return {
            "id": class_id,
            "name": class_id,
            "label": str(detection.get("label") or class_id),
            "class_id": class_id,
            "frame_id": str(detection.get("frame_id") or "base_link"),
            "pose": {
                "position": dict(position),
                "orientation": dict(orientation),
            },
            "size": dict(detection.get("size") or {}),
            "score": float(detection.get("score") or 0.0),
        }

    @staticmethod
    def _factory_task_object_keys(grounded: Dict[str, Any]) -> tuple[str, ...]:
        keys: List[str] = []
        for field_name in ("id", "name", "label", "class_id"):
            value = str(grounded.get(field_name) or "").strip()
            if value and value not in keys:
                keys.append(value)
        return tuple(keys)

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
        semantic_ir = self._stamp_review_provenance(semantic_ir)
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
            not is_factory_task_runtime_sentinel(semantic_ir)
            and
            not self._semantic_ir_contains_intent(semantic_ir, "return_to_start")
            and not self._semantic_ir_contains_any_intent(
                semantic_ir, {"draw_shape", "draw_text"}
            )
        ):
            try:
                canonical = self._canonicalize_semantic_ir_aliases(semantic_ir)
                station_scene_graph = getattr(self, "_station_scene_graph", None)
                semantic_map = station_scene_graph.to_dict() if station_scene_graph else None
                routed = IntentRouter(
                    runtime_mode=effective_runtime_mode,
                    station_semantic_map=semantic_map
                ).route(canonical)
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
        return response

    def _state_joint_state_callback(self, msg: JointState) -> None:
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
        self._state_injector.update_joint_states(
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
            intent = str(canonical.get("intent") or "").strip()
            if intent == "move_to_named_pose":
                canonical["intent"] = "move_named_pose"
                intent = "move_named_pose"

            if intent == "move_named_pose":
                if "pose_name" not in canonical:
                    if "pose" in canonical:
                        canonical["pose_name"] = canonical.pop("pose")
                    elif "region" in canonical:
                        canonical["pose_name"] = canonical.pop("region")
            elif intent == "get_current_pose":
                canonical["intent"] = "get_pose"
            elif intent == "move_linear":
                canonical["intent"] = "absolute_move_lin"
            elif intent in {"move_joint", "move_joint_delta"}:
                if "joint_index" not in canonical and "joint" in canonical:
                    canonical["joint_index"] = canonical.pop("joint")
                if "joint_angle" not in canonical and "angle" in canonical:
                    canonical["joint_angle"] = canonical.pop("angle")
                if "delta_angle" not in canonical and "delta" in canonical:
                    canonical["delta_angle"] = canonical.pop("delta")
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

    def _emit_robot_status_event(self, status: Any) -> None:
        def _get_bool(attr):
            val = getattr(status, attr, False)
            return self._tri_state_is_true(val) if hasattr(val, "val") else bool(val)
        def _get_int(attr):
            val = getattr(status, attr, 0)
            return int(getattr(val, "val", val))

        in_error = _get_bool("in_error")
        e_stopped = _get_bool("e_stopped")
        servo_on = _get_bool("drives_powered") if hasattr(status, "drives_powered") else _get_bool("servo_on")
        mode = _get_int("mode")
        
        error_code = 0
        if hasattr(status, "error_code"):
            error_code = int(status.error_code)
        elif hasattr(status, "error_codes") and status.error_codes:
            error_code = int(status.error_codes[0])

        level = "ERR" if (in_error or e_stopped) else "INFO"
        
        fingerprint = f"{in_error}:{e_stopped}:{servo_on}:{mode}:{error_code}"
        last_fp = getattr(self, "_last_robot_status_fingerprint", None)
        if last_fp == fingerprint:
            return
        self._last_robot_status_fingerprint = fingerprint

        self._emit_task_event(
            "HARDWARE", "robot_status",
            f"alarm={error_code} estop={e_stopped} servo={servo_on}",
            level=level, source="hw_adapter",
            data={
                "error_code": error_code,
                "e_stopped": e_stopped, "in_error": in_error,
                "in_motion": _get_bool("in_motion"),
                "servo_on": servo_on,
                "mode": mode,
            },
        )

    def _state_robot_status_callback(self, msg: RobotStatus) -> None:
        self._emit_robot_status_event(msg)
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
        self._state_injector.update_robot_status(
            {
                "mode": mode,
                "active_alarms": [str(code) for code in msg.error_codes],
            }
        )
        if mode == "FAULT":
            self._set_runtime_stop(True)
        if mode != "IDLE":
            self._invalidate_scene_cache()

    def _invalidate_scene_cache(self) -> None:
        self._get_scene_snapshot_cache().invalidate()

    # ── Runtime STOP flag for FactoryTask executor ────────────────────────

    def _init_runtime_stop_state(self) -> None:
        self._runtime_stop_flag = False

    def _set_runtime_stop(self, value: bool) -> None:
        self._runtime_stop_flag = bool(value)

    def _runtime_is_stopped(self) -> bool:
        return bool(getattr(self, "_runtime_stop_flag", False))

    def _query_scene_for_verify(self) -> dict:
        # Bypass cache: always get a fresh snapshot for postcondition verification.
        self._invalidate_scene_cache()
        result = self._query_perception_detections({
            "class_filter": "",
            "frame": "base_link",
        })
        if result.get("ok"):
            return result.get("payload") or {}
        return {}


    def _read_gripper_feedback(self) -> bool:
        """Read gripper closed feedback; returns True if grasped, False otherwise."""
        config = self._gripper_adapter._config
        if not config.verified():
            return False
        client = getattr(self, "_read_single_io_client", None)
        if client is None or not client.service_is_ready():
            return False
        from motoros2_interfaces.srv import ReadSingleIO
        request = ReadSingleIO.Request()
        request.address = int(config.closed_input_address)
        future = client.call_async(request)
        done, response = self._wait_for_future_without_spinning(future, config.feedback_timeout_sec)
        if not done or response is None or not response.success:
            return False
        return int(response.value) == int(config.closed_input_active_value)
    def _get_scene_snapshot_cache(self) -> _SceneSnapshotCache:
        cache = getattr(self, "_scene_snapshot_cache", None)
        if cache is None:
            cache = _SceneSnapshotCache(ttl_sec=2.0)
            self._scene_snapshot_cache = cache
        return cache

    def _current_planner_robot_mode(self) -> str:
        snapshot = self._state_injector.snapshot()
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
        if self._planner_enabled and self._task_planner is not None:
            try:
                self._emit_trace("llm_request_started", "reasoning", source="task_planner")
                planner_result = self._task_planner.plan(intent_text)
                self._emit_trace("llm_response_received", "reasoning", source="task_planner")
            except Exception as exc:
                self._reject("task_planner_failed", str(exc), intent_text=intent_text)
                return
            if is_factory_task(planner_result):
                self._reject(
                    "factory_task_requires_review",
                    "FactoryTask motion requests must be reviewed through "
                    "/llm_gateway/review_intent and confirmed before execution.",
                    intent_text=intent_text,
                )
                return
            if "error" not in planner_result:
                self._reject(
                    "planner_contract_rejected",
                    "TaskPlanner returned neither FactoryTask nor error payload.",
                    intent_text=intent_text,
                    hint="Planner output must be FactoryTask; Semantic IR is internal to the gateway compiler.",
                )
                return
            payload = json.dumps(planner_result)
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
        send_future.add_done_callback(lambda f: None)
        if sequence_state is None:
            self.publish_status("dispatched")

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

    def _run_single_command_via_runtime(self, semantic_ir: Dict[str, Any]) -> bool:
        from llm_gateway.task_runtime import TaskRuntime, RuntimeStepResult
        from llm_gateway.factory_task import parse_factory_task
        self._set_runtime_stop(False)
        
        payload = {
            "task_type": "factory_task",
            "version": "1.0",
            "task_id": "tier1",
            "mode": "supervised_hardware",
            "operator_summary": semantic_ir.get("intent", "direct_command"),
            "limits": {},
            "replan_policy": {},
            "root": {
                "type": "skill",
                "name": semantic_ir.get("intent", "direct_command"),
                "arguments": semantic_ir,
            }
        }
        task = parse_factory_task(payload)

        def _execute_skill(name: str, args: Dict[str, Any]) -> RuntimeStepResult:
            try:
                return self._validate_runtime_semantic_ir(args)
            except Exception as exc:
                return RuntimeStepResult(success=False, reason=str(exc))

        runtime = TaskRuntime(
            world_model=self._factory_task_world_model(),
            is_stopped_fn=self._runtime_is_stopped,
            event_callback=getattr(self, "_runtime_event_sink", None),
        )
        report = runtime.run(task, _execute_skill)
        return report.success

    def _execute_tier1_command(self, parsed_intent: Dict[str, Any]) -> bool:
        return self._run_single_command_via_runtime(parsed_intent)

    @staticmethod
    def _factory_task_payload_from_runtime_sentinel(
        parsed_intent: Dict[str, Any]
    ) -> Dict[str, Any]:
        metadata = parsed_intent.get("metadata")
        if not isinstance(metadata, dict):
            raise ValueError("FactoryTask runtime sentinel requires metadata")
        factory_task = metadata.get("factory_task")
        runtime_plan = metadata.get("runtime_plan")
        if not isinstance(factory_task, dict) or not isinstance(runtime_plan, dict):
            raise ValueError(
                "FactoryTask runtime sentinel requires factory_task and runtime_plan metadata"
            )
        task_id = str(factory_task.get("task_id") or "").strip()
        if not task_id:
            raise ValueError("FactoryTask runtime sentinel requires factory_task.task_id")
        return {
            "task_type": "factory_task",
            "version": str(factory_task.get("version") or "1.0"),
            "task_id": task_id,
            "mode": str(factory_task.get("mode") or "supervised_hardware"),
            "operator_summary": str(factory_task.get("operator_summary") or ""),
            "limits": dict(factory_task.get("limits") or {}),
            "replan_policy": dict(factory_task.get("replan_policy") or {}),
            "root": dict(runtime_plan),
        }

    def _on_confirm_factory_task_runtime(
        self,
        parsed_intent: Dict[str, Any],
        request: ConfirmExecution.Request,
        response: ConfirmExecution.Response,
    ) -> ConfirmExecution.Response:
        try:
            task_payload = self._factory_task_payload_from_runtime_sentinel(
                parsed_intent
            )
            task = parse_factory_task(task_payload)
        except Exception as exc:
            response.accepted = False
            response.reason = f"invalid FactoryTask runtime payload: {exc}"
            return response

        # Reset stale stop from a previous task.
        self._set_runtime_stop(False)

        from llm_gateway.runtime_skill_executor import RuntimeSkillExecutor

        class _NodeDeps:
            def __init__(self, node, task_payload):
                self.node = node
                self.task_payload = task_payload
            def semantic_ir_for_skill(self, name, args):
                return self.node._semantic_ir_for_runtime_skill(self.task_payload, name, args)
            def validate_and_dispatch(self, semantic_ir):
                return self.node._validate_runtime_semantic_ir(semantic_ir)

        _execute_skill = RuntimeSkillExecutor(_NodeDeps(self, task_payload))

        def _replan(_report):
            """Re-plan once via the planner from the original operator summary."""
            try:
                planned = self._task_planner.plan(task.operator_summary) if self._task_planner else None
            except Exception:
                return None
            return parse_factory_task(planned) if is_factory_task(planned) else None

        runtime = TaskRuntime(
            world_model=self._factory_task_world_model(),
            is_stopped_fn=self._runtime_is_stopped,
            event_callback=getattr(self, "_runtime_event_sink", None),
            replan_handler=_replan,
            max_replans=1,
        )
        report = runtime.run(task, _execute_skill)

        if not report.success:
            response.accepted = False
            response.reason = report.reason or "FactoryTask runtime execution failed"
            response.dispatched_to_ros = False
            return response

        response.accepted = True
        response.reason = ""
        response.execution_summary = (
            f"FactoryTask {task.task_id} confirmed by {request.operator_id}; "
            f"executed {sum(report.attempts_by_skill.values())} runtime skill(s); "
            f"fingerprint {request.plan_fingerprint[:12]}..."
        )
        response.dispatched_to_ros = report.success
        return response

    def _runtime_event_sink(self, event: dict) -> None:
        pub = getattr(self, "_task_events_pub", None)
        if pub is None:
            return
        msg = String()
        msg.data = json.dumps(event, ensure_ascii=False, separators=(",", ":"))
        pub.publish(msg)

    def _emit_task_event(self, category, event, detail, *, level="INFO", source="gateway", data=None):
        import datetime
        self._runtime_event_sink({
            "ts": datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "level": level, "source": source, "category": category,
            "event": event, "detail": detail, "data": data or {},
        })

    def _semantic_ir_for_runtime_skill(
        self,
        task_payload: Dict[str, Any],
        name: str,
        args: Dict[str, Any],
    ) -> Dict[str, Any]:
        if name in {"pick_object", "place_object", "pick_and_place"}:
            config = getattr(getattr(self, "_gripper_adapter", None), "_config", None)
            if config is None or not config.verified():
                raise ValueError("gripper configuration is not verified")

        single_task_payload = dict(task_payload)
        single_task_payload["task_id"] = f"{task_payload['task_id']}:{name}"
        single_task_payload["root"] = {
            "type": "skill",
            "name": name,
            "args": dict(args or {}),
        }
        self._prime_factory_task_world_model(single_task_payload)
        compiled = TaskCompiler(world_model=self._factory_task_world_model()).compile(
            parse_factory_task(single_task_payload)
        )
        return dict(compiled.semantic_ir)

    def _validate_runtime_semantic_ir(
        self, semantic_ir: Dict[str, Any]
    ) -> RuntimeStepResult:
        if not isinstance(semantic_ir, dict):
            return RuntimeStepResult(
                success=False,
                reason="runtime skill produced a non-object command artifact",
            )
        if semantic_ir.get("intent") == "sequence":
            steps = semantic_ir.get("steps")
            if not isinstance(steps, list) or not steps:
                return RuntimeStepResult(
                    success=False,
                    reason="runtime sequence requires at least one step",
                )
            for step in steps:
                result = self._validate_runtime_semantic_ir(step)
                if not result.success:
                    return result
            return RuntimeStepResult(success=True)

        prepared = self._prepare_review_semantic_ir(semantic_ir)
        if isinstance(prepared, dict) and "error" in prepared:
            return RuntimeStepResult(
                success=False,
                reason=self._review_error_message(prepared),
            )
        contract = validate_semantic_ir_contract(prepared)
        if not contract.valid:
            return RuntimeStepResult(success=False, reason=contract.reason)

        try:
            routed = self._intent_router.route(
                self._canonicalize_semantic_ir_aliases(prepared)
            )
        except Exception as exc:
            return RuntimeStepResult(
                success=False,
                reason=self._review_exception_message(exc),
            )
        if routed.route_type == "error":
            error_payload = routed.error_payload or {}
            return RuntimeStepResult(
                success=False,
                reason=self._review_error_message(
                    {
                        "error": error_payload.get("error"),
                        "message": (
                            error_payload.get("message")
                            or "runtime skill was rejected by the intent router."
                        ),
                        "intent": error_payload.get("intent"),
                    }
                ),
            )
        commands = list(routed.commands or [])
        if not commands:
            return RuntimeStepResult(
                success=False,
                reason="runtime skill produced no routed command",
            )
        for command in commands:
            result = self._validate_runtime_command(command)
            if not result.success:
                return result
        return RuntimeStepResult(success=True)

    def _runtime_command_is_safe(
        self, command: Dict[str, Any]
    ) -> tuple[bool, str, Dict[str, Any] | None, Dict[str, Any] | None]:
        """Return (ok, reason, normalized_command, command_payload). No dispatch."""
        try:
            self._schema_validator.validate(command)
            normalized_command = self._normalize_and_validate(command)
        except Exception as exc:
            return False, str(exc), None, None

        primitive_type = str(normalized_command.get("primitive_type") or "")
        if self._is_query_command(primitive_type):
            pose_client = getattr(self, "_get_pose_client", None)
            if pose_client is None or not pose_client.wait_for_service(
                timeout_sec=self._safety_service_timeout_sec
            ):
                return False, "GetCurrentPose service unavailable", None, None
            return True, "", normalized_command, None  # query: no motion goal

        command_payload = self._goal_mapper.to_command_payload(normalized_command)
        if not self._validate_client.wait_for_service(
            timeout_sec=self._safety_service_timeout_sec
        ):
            return False, "ValidateCommand service unavailable", None, None
        validate_req = self._build_validate_request(normalized_command, command_payload)
        try:
            validate_future = self._validate_client.call_async(validate_req)
            done, validate_resp = self._wait_for_future_without_spinning(
                validate_future, self._safety_service_timeout_sec
            )
        except Exception as exc:
            return False, f"ValidateCommand call failed: {exc}", None, None
        if not done:
            return False, "ValidateCommand service timed out", None, None
        if not validate_resp.valid:
            return False, str(validate_resp.reason), None, None
        return True, "", normalized_command, command_payload

    def _dispatch_runtime_goal(
        self, normalized_command: Dict[str, Any]
    ) -> "DispatchOutcome":
        """Send the validated command to ExecuteMotion and await its result."""
        from llm_gateway.runtime_dispatch import dispatch_and_await
        goal = self._goal_mapper.to_execute_motion_goal(normalized_command)
        outcome = dispatch_and_await(
            getattr(self, "_execute_client", None),
            goal=goal,
            wait_fn=self._wait_for_future_without_spinning,
            is_stopped_fn=self._runtime_is_stopped,
            timeout_sec=self._motion_result_timeout_sec,
        )
        return outcome

    def _validate_runtime_command(
        self, command: Dict[str, Any]
    ) -> RuntimeStepResult:
        ok, reason, normalized_command, _payload = self._runtime_command_is_safe(command)
        if not ok:
            return RuntimeStepResult(success=False, reason=reason)
        # Query commands (GET_POSE) validated above carry no motion goal.
        if normalized_command is not None and self._is_query_command(
            str(normalized_command.get("primitive_type") or "")
        ):
            return RuntimeStepResult(success=True)
        if normalized_command is None:
            return RuntimeStepResult(success=True)
        outcome = self._dispatch_runtime_goal(normalized_command)
        return RuntimeStepResult(success=outcome.ok, reason=outcome.reason)

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

        if is_factory_task_runtime_sentinel(parsed_intent):
            return self._on_confirm_factory_task_runtime(
                parsed_intent, request, response
            )

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
                f"TaskPlanner get_current_pose: service /get_current_pose "
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
                f"TaskPlanner get_current_pose: async call timed out "
                f"after {self._safety_service_timeout_sec}s"
            )
            return None
        if response is None or not response.success:
            self.get_logger().error(
                f"TaskPlanner get_current_pose: response "
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
