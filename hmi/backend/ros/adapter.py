from __future__ import annotations

from datetime import datetime, timezone
import importlib.util
import json
import logging
from pathlib import Path
import sys
from threading import Lock, Thread
from typing import Any

from .command_dispatch import CommandDispatchMixin
from .jog_dispatch import JogDispatchMixin
from .telemetry_snapshot import (
    CONNECTION_FRESHNESS_SEC,
    TelemetrySnapshotMixin,
    _TelemetryState,
)


def _load_joint_state_type() -> Any:
    module_name = "sensor_msgs.msg._joint_state"
    for search_root in map(Path, sys.path):
        candidate = search_root / "sensor_msgs" / "msg" / "_joint_state.py"
        if not candidate.exists():
            continue
        spec = importlib.util.spec_from_file_location(module_name, candidate)
        if spec is None or spec.loader is None:
            continue
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        return module.JointState
    raise ImportError("Unable to locate sensor_msgs/msg/_joint_state.py on sys.path")


try:
    import rclpy
    from geometry_msgs.msg import Pose
    from diagnostic_msgs.msg import DiagnosticStatus
    from industrial_msgs.msg import RobotMode as IndustrialRobotMode
    from industrial_msgs.msg import RobotStatus as IndustrialRobotStatus
    from interfaces.action import ExecuteMotion
    from interfaces.msg import RobotReadiness as RobotReadinessMsg
    from interfaces.srv import GetCurrentPose, ValidateCommand
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    JointState = _load_joint_state_type()
except Exception as exc:  # pragma: no cover - depends on sourced ROS environment
    rclpy = None
    Pose = None
    DiagnosticStatus = None
    IndustrialRobotMode = None
    IndustrialRobotStatus = None
    ExecuteMotion = None
    RobotReadinessMsg = None
    GetCurrentPose = None
    ValidateCommand = None
    ActionClient = None
    SingleThreadedExecutor = None
    qos_profile_sensor_data = None
    JointState = None
    String = None
    _ROS_IMPORT_ERROR = str(exc)
else:  # pragma: no cover - trivial constant assignment
    _ROS_IMPORT_ERROR = None

try:
    from motoros2_interfaces.srv import StartTrajMode as StartTrajModeSrv
except Exception:  # pragma: no cover - optional MotoROS2 package
    StartTrajModeSrv = None
try:
    from std_srvs.srv import Trigger as TriggerSrv
except Exception:  # pragma: no cover
    TriggerSrv = None

# Keep supervisor defaults at-or-below current validation limits so
# sim execution fails closed less often on conservative profiles.
DEFAULT_MOTION_VELOCITY_SCALE = 0.06
DEFAULT_MOTION_ACCELERATION_SCALE = 0.06
DEFAULT_VALIDATE_TIMEOUT_SEC = 5.0
DEFAULT_ACTION_WAIT_TIMEOUT_SEC = 5.0
DEFAULT_EXECUTION_TIMEOUT_SEC = 120.0
LOGGER = logging.getLogger("uvicorn.error")


KNOWN_WORKSPACE_ENDPOINTS = {
    "read_only_topics": [
        "/gateway_status",
        "/llm_debug",
        "/llm_command",
        "/hw_adapter/ready",
        "/supervisor/alerts",
        "/yaskawa/joint_states",
        "/joint_states",
        "/yaskawa/robot_status",
    ],
    "write_capable_interfaces": [
        "/llm_text_input",
        "/validate_command",
        "/execute_motion",
    ],
}


class WorkspaceRosAdapter(
    TelemetrySnapshotMixin, CommandDispatchMixin, JogDispatchMixin
):
    """ROS adapter for HMI telemetry and supervisor-owned execution handoff.

    Telemetry remains subscription-driven and read-oriented. The only write-capable
    path this adapter opens is the supervisor-owned execution boundary:
    ValidateCommand service -> ExecuteMotion action.
    """

    def __init__(
        self,
        *,
        node_name: str = "gp4_hmi_readonly_bridge",
        gateway_status_topic: str = "/gateway_status",
        llm_debug_topic: str = "/llm_debug",
        llm_command_topic: str = "/llm_command",
        readiness_topic: str = "/hw_adapter/ready",
        supervisor_alert_topic: str = "/supervisor/alerts",
        robot_status_topic: str = "/yaskawa/robot_status",
        joint_state_topics: tuple[str, ...] = (
            "/yaskawa/joint_states",
            "/joint_states",
        ),
        preferred_joint_state_topic: str = "/yaskawa/joint_states",
        validate_command_service: str = "/validate_command",
        execute_motion_action: str = "/execute_motion",
        get_current_pose_service: str = "/get_current_pose",
    ) -> None:
        self._node_name = node_name
        self._gateway_status_topic = gateway_status_topic
        self._llm_debug_topic = llm_debug_topic
        self._llm_command_topic = llm_command_topic
        self._readiness_topic = readiness_topic
        self._supervisor_alert_topic = supervisor_alert_topic
        self._robot_status_topic = robot_status_topic
        self._joint_state_topics = joint_state_topics
        self._preferred_joint_state_topic = preferred_joint_state_topic
        self._validate_command_service = validate_command_service
        self._execute_motion_action = execute_motion_action
        self._get_current_pose_service = get_current_pose_service

        self._lock = Lock()
        self._state = _TelemetryState(start_error=_ROS_IMPORT_ERROR)
        self._industrial_robot_mode = IndustrialRobotMode
        self._context: Any = None
        self._node: Any = None
        self._executor: Any = None
        self._thread: Thread | None = None
        self._subscriptions: list[Any] = []
        self._validate_client: Any = None
        self._execute_client: Any = None
        self._get_pose_client: Any = None
        self._start_traj_client: Any = None
        self._stop_traj_client: Any = None
        self._goal_handles: dict[str, Any] = {}
        self._goal_correlation_ids: dict[str, str] = {}
        self._goal_lock = Lock()
        self._stop_requested = False
        self._command_interface_poll_period_sec = 0.5

    def start(self) -> None:
        if rclpy is None:
            return
        if self._thread is not None:
            return

        try:
            self._context = rclpy.context.Context()
            rclpy.init(args=None, context=self._context)
            self._node = rclpy.create_node(self._node_name, context=self._context)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._create_subscriptions()
            self._create_command_clients()
            with self._lock:
                self._state.ros_started_at = self._now()
                self._state.start_error = None
            self._stop_requested = False
            self._thread = Thread(
                target=self._spin, name=f"{self._node_name}-spin", daemon=True
            )
            self._thread.start()
        except Exception as exc:  # pragma: no cover - requires ROS runtime
            with self._lock:
                self._state.start_error = str(exc)
            self.stop()

    def stop(self) -> None:
        self._stop_requested = True
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=0.2)
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if self._context is not None:
            try:
                if self._context.ok():
                    rclpy.shutdown(context=self._context)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self._subscriptions = []
        with self._goal_lock:
            self._goal_handles.clear()
            self._goal_correlation_ids.clear()
        self._validate_client = None
        self._execute_client = None
        self._get_pose_client = None
        self._start_traj_client = None
        self._stop_traj_client = None
        self._executor = None
        self._node = None
        self._context = None

    def submit_text_for_review(
        self,
        *,
        raw_text: str,
        session_id: str,
        operator_id: str,
        command_id: str,
    ) -> dict[str, Any]:
        return {
            "accepted": True,
            "adapter": "workspace_stub",
            "summary": (
                "Supervisor retained intent for local parse/validation. "
                "No ROS write-capable review path was invoked."
            ),
            "rawText": raw_text,
            "sessionId": session_id,
            "operatorId": operator_id,
            "commandId": command_id,
        }

    def _trace(
        self,
        stage: str,
        *,
        command_id: str,
        correlation_id: str | None,
        **fields: Any,
    ) -> None:
        rendered_fields = [
            f"command_id={command_id}",
            f"correlation_id={correlation_id or 'n/a'}",
        ]
        for key, value in fields.items():
            if value is None:
                continue
            if isinstance(value, (dict, list, tuple)):
                rendered_value = json.dumps(
                    value, ensure_ascii=True, separators=(",", ":")
                )
            else:
                rendered_value = str(value)
            rendered_fields.append(f"{key}={rendered_value}")
        LOGGER.info("[HMI ROS] %s | %s", stage, " | ".join(rendered_fields))

    def get_current_pose(
        self, *, reference_frame: str = "base_link"
    ) -> dict[str, Any] | None:
        if (
            self._node is None
            or GetCurrentPose is None
            or self._get_pose_client is None
        ):
            return None
        if not self._get_pose_client.wait_for_service(
            timeout_sec=DEFAULT_VALIDATE_TIMEOUT_SEC
        ):
            return None

        request = GetCurrentPose.Request()
        request.reference_frame = reference_frame
        try:
            response = self._wait_for_future(
                self._get_pose_client.call_async(request),
                DEFAULT_VALIDATE_TIMEOUT_SEC,
            )
        except Exception:
            return None

        if response is None or not getattr(response, "success", False):
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

    def start_traj_mode(self) -> dict[str, Any]:
        if (
            self._node is None
            or StartTrajModeSrv is None
            or self._start_traj_client is None
        ):
            return {"accepted": False, "message": "start_traj_mode service unavailable"}
        if not self._start_traj_client.wait_for_service(timeout_sec=2.0):
            return {"accepted": False, "message": "start_traj_mode service not ready"}
        try:
            resp = self._wait_for_future(
                self._start_traj_client.call_async(StartTrajModeSrv.Request()),
                5.0,
            )
        except Exception as exc:
            return {"accepted": False, "message": str(exc)}
        if resp is None:
            return {"accepted": False, "message": "no response"}
        code = getattr(resp.result_code, "value", resp.result_code)
        return {"accepted": code == 1, "message": resp.message or f"result_code={code}"}

    def stop_motion(self) -> dict[str, Any]:
        if self._node is None or TriggerSrv is None or self._stop_traj_client is None:
            return {"accepted": False, "message": "stop_traj_mode service unavailable"}
        if not self._stop_traj_client.wait_for_service(timeout_sec=2.0):
            return {"accepted": False, "message": "stop_traj_mode service not ready"}
        try:
            resp = self._wait_for_future(
                self._stop_traj_client.call_async(TriggerSrv.Request()),
                5.0,
            )
        except Exception as exc:
            return {"accepted": False, "message": str(exc)}
        if resp is None:
            return {"accepted": False, "message": "no response"}
        return {
            "accepted": resp.success,
            "message": resp.message or ("stopped" if resp.success else "failed"),
        }

    def _create_subscriptions(self) -> None:
        assert self._node is not None
        assert String is not None
        assert RobotReadinessMsg is not None
        assert DiagnosticStatus is not None
        assert IndustrialRobotStatus is not None
        assert JointState is not None

        self._subscriptions = [
            self._node.create_subscription(
                String, self._gateway_status_topic, self._on_gateway_status, 10
            ),
            self._node.create_subscription(
                String, self._llm_debug_topic, self._on_llm_debug, 10
            ),
            self._node.create_subscription(
                String, self._llm_command_topic, self._on_llm_command, 10
            ),
            self._node.create_subscription(
                RobotReadinessMsg, self._readiness_topic, self._on_readiness, 10
            ),
            self._node.create_subscription(
                DiagnosticStatus,
                self._supervisor_alert_topic,
                self._on_supervisor_alert,
                10,
            ),
            self._node.create_subscription(
                IndustrialRobotStatus,
                self._robot_status_topic,
                self._on_robot_status,
                qos_profile_sensor_data,
            ),
        ]
        for topic in self._joint_state_topics:
            self._subscriptions.append(
                self._node.create_subscription(
                    JointState,
                    topic,
                    lambda msg, joint_topic=topic: self._on_joint_state(
                        joint_topic, msg
                    ),
                    10,
                )
            )

    def _create_command_clients(self) -> None:
        if (
            self._node is None
            or ValidateCommand is None
            or ExecuteMotion is None
            or ActionClient is None
        ):
            return
        self._validate_client = self._node.create_client(
            ValidateCommand,
            self._validate_command_service,
        )
        self._execute_client = ActionClient(
            self._node,
            ExecuteMotion,
            self._execute_motion_action,
        )
        if GetCurrentPose is not None:
            self._get_pose_client = self._node.create_client(
                GetCurrentPose,
                self._get_current_pose_service,
            )
        if StartTrajModeSrv is not None:
            self._start_traj_client = self._node.create_client(
                StartTrajModeSrv,
                "/yaskawa/start_traj_mode",
            )
        if TriggerSrv is not None:
            self._stop_traj_client = self._node.create_client(
                TriggerSrv,
                "/yaskawa/stop_traj_mode",
            )

    def _spin(self) -> None:  # pragma: no cover - requires ROS runtime
        assert self._executor is not None
        assert self._context is not None
        while not self._stop_requested and self._context.ok():
            self._executor.spin_once(timeout_sec=0.2)
            self._refresh_command_interface_state()

    def _refresh_command_interface_state(self) -> None:
        if (
            self._node is None
            or self._validate_client is None
            or self._execute_client is None
        ):
            return

        now = self._now()
        with self._lock:
            last_checked = self._state.command_interface_checked_at
            if (
                last_checked is not None
                and (now - last_checked).total_seconds()
                < self._command_interface_poll_period_sec
            ):
                return
            self._state.command_interface_checked_at = now

        validate_ready = self._validate_client.service_is_ready()
        execute_ready = self._execute_client.server_is_ready()

        with self._lock:
            self._state.validate_command_ready = validate_ready
            self._state.execute_motion_ready = execute_ready
            self._state.validate_command_detail = (
                f"ready at {self._validate_command_service}"
                if validate_ready
                else f"waiting for {self._validate_command_service}"
            )
            self._state.execute_motion_detail = (
                f"ready at {self._execute_motion_action}"
                if execute_ready
                else f"waiting for {self._execute_motion_action}"
            )
            if validate_ready:
                self._state.validate_command_ready_at = now
            if execute_ready:
                self._state.execute_motion_ready_at = now
            if validate_ready and execute_ready:
                self._state.command_interface_error = None

    def _command_interfaces_ready(self) -> bool:
        with self._lock:
            return (
                self._state.validate_command_ready and self._state.execute_motion_ready
            )

    def _command_interface_block_reason(self) -> str | None:
        with self._lock:
            if self._state.command_interface_error:
                return self._state.command_interface_error
            missing: list[str] = []
            if not self._state.validate_command_ready:
                missing.append(
                    self._state.validate_command_detail
                    or self._validate_command_service
                )
            if not self._state.execute_motion_ready:
                missing.append(
                    self._state.execute_motion_detail or self._execute_motion_action
                )
        if not missing:
            return None
        return "command interfaces not ready: " + "; ".join(missing)

    def _on_gateway_status(self, msg: Any) -> None:
        with self._lock:
            self._state.llm.gateway_status_at = self._now()
            self._state.llm.gateway_status_text = str(msg.data)

    def _on_llm_debug(self, msg: Any) -> None:
        _ = msg
        with self._lock:
            self._state.llm.debug_at = self._now()

    def _on_llm_command(self, msg: Any) -> None:
        _ = msg
        with self._lock:
            self._state.llm.command_at = self._now()

    def _on_readiness(self, msg: Any) -> None:
        with self._lock:
            self._state.readiness.received_at = self._now()
            self._state.readiness.ready = bool(msg.ready)
            self._state.readiness.status_message = str(msg.status_message)

    def _on_supervisor_alert(self, msg: Any) -> None:
        values = {str(item.key): str(item.value) for item in getattr(msg, "values", [])}
        with self._lock:
            self._state.supervisor_alert.received_at = self._now()
            self._state.supervisor_alert.level = self._coerce_int(msg.level)
            self._state.supervisor_alert.message = str(msg.message)
            self._state.supervisor_alert.values = values

    def _on_robot_status(self, msg: Any) -> None:
        with self._lock:
            self._state.robot_status.received_at = self._now()
            self._state.robot_status.mode = self._coerce_int(msg.mode.val)
            self._state.robot_status.e_stopped = self._tri_state_to_bool(
                msg.e_stopped.val
            )
            self._state.robot_status.drives_powered = self._tri_state_to_bool(
                msg.drives_powered.val
            )
            self._state.robot_status.motion_possible = self._tri_state_to_bool(
                msg.motion_possible.val
            )
            self._state.robot_status.in_motion = self._tri_state_to_bool(
                msg.in_motion.val
            )
            self._state.robot_status.in_error = self._tri_state_to_bool(
                msg.in_error.val
            )
            self._state.robot_status.error_codes = [
                int(code) for code in msg.error_codes
            ]

    def _on_joint_state(self, topic: str, msg: Any) -> None:
        joint_positions: dict[str, float] = {}
        for index, name in enumerate(getattr(msg, "name", [])):
            if index < len(getattr(msg, "position", [])):
                joint_positions[str(name)] = float(msg.position[index])
        with self._lock:
            if not self._should_accept_joint_update(topic):
                return
            self._state.joint_received_at = self._now()
            self._state.joint_source_topic = topic
            self._state.joint_topic_received_at[topic] = self._state.joint_received_at
            self._state.joint_positions_rad.update(joint_positions)

    def _should_accept_joint_update(self, topic: str) -> bool:
        current_source = self._state.joint_source_topic
        if topic == self._preferred_joint_state_topic:
            return True
        if current_source is None:
            return True
        if current_source != self._preferred_joint_state_topic:
            return True
        return not self._is_fresh(
            self._state.joint_received_at,
            CONNECTION_FRESHNESS_SEC["joint_states"],
        )

    def _tri_state_to_bool(self, value: int) -> bool | None:
        value = self._coerce_int(value)
        if value < 0:
            return None
        return value > 0

    def _coerce_int(self, value: Any) -> int:
        if isinstance(value, (bytes, bytearray)):
            return int.from_bytes(value, byteorder="little", signed=True)
        return int(value)

    def _is_fresh(self, timestamp: datetime | None, max_age_sec: float) -> bool:
        if timestamp is None:
            return False
        return (self._now() - timestamp).total_seconds() <= max_age_sec

    def _now(self) -> datetime:
        return datetime.now(timezone.utc)
