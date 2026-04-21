from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import importlib.util
import json
import logging
from math import degrees
from pathlib import Path
import sys
from threading import Lock, Thread
import time
from typing import Any

from ..domain.constants import GP4_JOINT_NAMES as DEFAULT_JOINT_NAMES
from ..domain.models import (
    BridgeConnection,
    ConnectionHealth,
    JointPosition,
    RobotStatusSnapshot,
    TelemetryFreshnessState,
    TelemetrySourceSnapshot,
    RuntimeMode,
    RuntimeSnapshot,
    SystemRuntimeState,
)
from .command_dispatch import CommandDispatchMixin
from .jog_dispatch import JogDispatchMixin


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
    from interfaces.srv import ValidateCommand
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
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
    ValidateCommand = None
    ActionClient = None
    SingleThreadedExecutor = None
    JointState = None
    String = None
    _ROS_IMPORT_ERROR = str(exc)
else:  # pragma: no cover - trivial constant assignment
    _ROS_IMPORT_ERROR = None


CONNECTION_FRESHNESS_SEC = {
    'ros': 3.0,
    'robot_status': 3.0,
    'readiness': 3.0,
    'joint_states': 3.0,
    'command_interface': 3.0,
    'llm': 30.0,
    'alerts': 5.0,
}
# Keep supervisor defaults at-or-below current validation limits so
# sim execution fails closed less often on conservative profiles.
DEFAULT_MOTION_VELOCITY_SCALE = 0.06
DEFAULT_MOTION_ACCELERATION_SCALE = 0.06
DEFAULT_VALIDATE_TIMEOUT_SEC = 5.0
DEFAULT_ACTION_WAIT_TIMEOUT_SEC = 5.0
DEFAULT_EXECUTION_TIMEOUT_SEC = 120.0
LOGGER = logging.getLogger("uvicorn.error")


@dataclass(slots=True)
class _RobotStatusState:
    received_at: datetime | None = None
    mode: int | None = None
    e_stopped: bool | None = None
    drives_powered: bool | None = None
    motion_possible: bool | None = None
    in_motion: bool | None = None
    in_error: bool | None = None
    error_codes: list[int] = field(default_factory=list)


@dataclass(slots=True)
class _ReadinessState:
    received_at: datetime | None = None
    ready: bool | None = None
    status_message: str = 'No readiness signal received.'


@dataclass(slots=True)
class _SupervisorAlertState:
    received_at: datetime | None = None
    level: int | None = None
    message: str = ''
    values: dict[str, str] = field(default_factory=dict)


@dataclass(slots=True)
class _LlmState:
    gateway_status_at: datetime | None = None
    gateway_status_text: str = ''
    debug_at: datetime | None = None
    command_at: datetime | None = None


@dataclass(slots=True)
class _TelemetryState:
    ros_started_at: datetime | None = None
    start_error: str | None = None
    robot_status: _RobotStatusState = field(default_factory=_RobotStatusState)
    readiness: _ReadinessState = field(default_factory=_ReadinessState)
    supervisor_alert: _SupervisorAlertState = field(default_factory=_SupervisorAlertState)
    llm: _LlmState = field(default_factory=_LlmState)
    joint_positions_rad: dict[str, float] = field(default_factory=dict)
    joint_received_at: datetime | None = None
    joint_source_topic: str | None = None
    joint_topic_received_at: dict[str, datetime] = field(default_factory=dict)
    validate_command_ready_at: datetime | None = None
    execute_motion_ready_at: datetime | None = None
    validate_command_ready: bool = False
    execute_motion_ready: bool = False
    validate_command_detail: str = ''
    execute_motion_detail: str = ''
    command_interface_checked_at: datetime | None = None
    command_interface_check_inflight: bool = False
    command_interface_error: str | None = None
    command_interface_thread: Thread | None = None
    command_interface_lock: Lock = field(default_factory=Lock)


KNOWN_WORKSPACE_ENDPOINTS = {
    'read_only_topics': [
        '/gateway_status',
        '/llm_debug',
        '/llm_command',
        '/hw_adapter/ready',
        '/supervisor/alerts',
        '/yaskawa/joint_states',
        '/joint_states',
        '/yaskawa/robot_status',
    ],
    'write_capable_interfaces': [
        '/llm_text_input',
        '/validate_command',
        '/execute_motion',
    ],
}


class WorkspaceRosAdapter(CommandDispatchMixin, JogDispatchMixin):
    """ROS adapter for HMI telemetry and supervisor-owned execution handoff.

    Telemetry remains subscription-driven and read-oriented. The only write-capable
    path this adapter opens is the supervisor-owned execution boundary:
    ValidateCommand service -> ExecuteMotion action.
    """

    def __init__(
        self,
        *,
        node_name: str = 'gp4_hmi_readonly_bridge',
        gateway_status_topic: str = '/gateway_status',
        llm_debug_topic: str = '/llm_debug',
        llm_command_topic: str = '/llm_command',
        readiness_topic: str = '/hw_adapter/ready',
        supervisor_alert_topic: str = '/supervisor/alerts',
        robot_status_topic: str = '/yaskawa/robot_status',
        joint_state_topics: tuple[str, ...] = ('/yaskawa/joint_states', '/joint_states'),
        preferred_joint_state_topic: str = '/yaskawa/joint_states',
        validate_command_service: str = '/validate_command',
        execute_motion_action: str = '/execute_motion',
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

        self._lock = Lock()
        self._state = _TelemetryState(start_error=_ROS_IMPORT_ERROR)
        self._context: Any = None
        self._node: Any = None
        self._executor: Any = None
        self._thread: Thread | None = None
        self._subscriptions: list[Any] = []
        self._validate_client: Any = None
        self._execute_client: Any = None
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
            self._thread = Thread(target=self._spin, name=f'{self._node_name}-spin', daemon=True)
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
                rendered_value = json.dumps(value, ensure_ascii=True, separators=(",", ":"))
            else:
                rendered_value = str(value)
            rendered_fields.append(f"{key}={rendered_value}")
        LOGGER.info("[HMI ROS] %s | %s", stage, " | ".join(rendered_fields))

    def read_connections(self) -> list[BridgeConnection]:
        with self._lock:
            snapshot = self._copy_state_locked()

        ros_health = self._derive_ros_health(snapshot)
        llm_health = self._derive_llm_health(snapshot, ros_health)
        moveit_health = self._derive_moveit_health(snapshot, ros_health)
        motoros2_health = self._derive_motoros2_health(snapshot, ros_health)

        return [
            BridgeConnection(name='ros2', label='ROS 2', health=ros_health),
            BridgeConnection(name='moveit2', label='MoveIt 2', health=moveit_health),
            BridgeConnection(name='llm', label='LLM', health=llm_health),
            BridgeConnection(name='motoros2', label='MotoROS2', health=motoros2_health),
        ]

    def read_runtime_snapshot(self) -> RuntimeSnapshot:
        with self._lock:
            snapshot = self._copy_state_locked()

        mode = self._derive_mode(snapshot)
        robot_status = self._derive_robot_status_snapshot(snapshot)
        runtime_state, status_text = self._derive_runtime_state(snapshot, robot_status)
        return RuntimeSnapshot(
            system_state=runtime_state,
            blocking=runtime_state in {
                SystemRuntimeState.FAULT,
                SystemRuntimeState.ESTOP,
                SystemRuntimeState.LOST_CONN,
                SystemRuntimeState.SAFETY_BLOCKED,
            },
            status_text=status_text,
            mode=mode,
            robot_status=robot_status,
        )

    def read_joint_positions(self) -> list[JointPosition]:
        with self._lock:
            snapshot = self._copy_state_locked()

        joint_positions: list[JointPosition] = []
        joint_data_is_fresh = self._is_fresh(snapshot.joint_received_at, CONNECTION_FRESHNESS_SEC['joint_states'])
        for joint_name in DEFAULT_JOINT_NAMES:
            position_rad = snapshot.joint_positions_rad.get(joint_name) if joint_data_is_fresh else None
            position_deg = degrees(position_rad) if position_rad is not None else None
            joint_positions.append(JointPosition(name=joint_name, position_deg=position_deg))
        return joint_positions

    def read_source_statuses(self) -> list[TelemetrySourceSnapshot]:
        with self._lock:
            snapshot = self._copy_state_locked()

        runtime_mode = self._derive_mode(snapshot)
        joint_fallback_topic = self._joint_state_topics[-1]
        preferred_joint_topic = (
            joint_fallback_topic if runtime_mode == RuntimeMode.SIM else self._preferred_joint_state_topic
        )

        statuses = [
            self._build_source_status(
                snapshot=snapshot,
                name='gateway_status',
                label='Gateway status',
                topic=self._gateway_status_topic,
                last_seen_at=snapshot.llm.gateway_status_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC['llm'],
                detail=snapshot.llm.gateway_status_text or None,
            ),
            self._build_source_status(
                snapshot=snapshot,
                name='llm_debug',
                label='LLM debug',
                topic=self._llm_debug_topic,
                last_seen_at=snapshot.llm.debug_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC['llm'],
                active=self._is_fresh(snapshot.llm.debug_at, CONNECTION_FRESHNESS_SEC['llm']),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name='llm_command',
                label='LLM command echo',
                topic=self._llm_command_topic,
                last_seen_at=snapshot.llm.command_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC['llm'],
                active=self._is_fresh(snapshot.llm.command_at, CONNECTION_FRESHNESS_SEC['llm']),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name='readiness',
                label='HW readiness',
                topic=self._readiness_topic,
                last_seen_at=snapshot.readiness.received_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC['readiness'],
                detail=snapshot.readiness.status_message,
            ),
            self._build_source_status(
                snapshot=snapshot,
                name='supervisor_alerts',
                label='Supervisor alerts',
                topic=self._supervisor_alert_topic,
                last_seen_at=snapshot.supervisor_alert.received_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC['alerts'],
                detail=snapshot.supervisor_alert.message or None,
            ),
            self._build_source_status(
                snapshot=snapshot,
                name='robot_status',
                label='Robot status',
                topic=self._robot_status_topic,
                last_seen_at=snapshot.robot_status.received_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC['robot_status'],
                active=runtime_mode != RuntimeMode.SIM,
                detail=(
                    'SIM mode uses /hw_adapter/ready instead of raw /yaskawa/robot_status.'
                    if runtime_mode == RuntimeMode.SIM else None
                ),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name='joint_states_primary',
                label='Joint states primary',
                topic=self._preferred_joint_state_topic,
                last_seen_at=snapshot.joint_topic_received_at.get(self._preferred_joint_state_topic),
                freshness_sec=CONNECTION_FRESHNESS_SEC['joint_states'],
                preferred=self._preferred_joint_state_topic == preferred_joint_topic,
                active=snapshot.joint_source_topic == self._preferred_joint_state_topic,
                detail=(
                    f'SIM mode prefers {joint_fallback_topic}.'
                    if runtime_mode == RuntimeMode.SIM and self._preferred_joint_state_topic != preferred_joint_topic
                    else None
                ),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name='joint_states_fallback',
                label='Joint states fallback',
                topic=joint_fallback_topic,
                last_seen_at=snapshot.joint_topic_received_at.get(joint_fallback_topic),
                freshness_sec=CONNECTION_FRESHNESS_SEC['joint_states'],
                preferred=joint_fallback_topic == preferred_joint_topic,
                active=snapshot.joint_source_topic == joint_fallback_topic,
            ),
        ]
        statuses.extend(self._command_interface_source_statuses(snapshot, runtime_mode))
        return statuses

    def _create_subscriptions(self) -> None:
        assert self._node is not None
        assert String is not None
        assert RobotReadinessMsg is not None
        assert DiagnosticStatus is not None
        assert IndustrialRobotStatus is not None
        assert JointState is not None

        self._subscriptions = [
            self._node.create_subscription(String, self._gateway_status_topic, self._on_gateway_status, 10),
            self._node.create_subscription(String, self._llm_debug_topic, self._on_llm_debug, 10),
            self._node.create_subscription(String, self._llm_command_topic, self._on_llm_command, 10),
            self._node.create_subscription(RobotReadinessMsg, self._readiness_topic, self._on_readiness, 10),
            self._node.create_subscription(DiagnosticStatus, self._supervisor_alert_topic, self._on_supervisor_alert, 10),
            self._node.create_subscription(IndustrialRobotStatus, self._robot_status_topic, self._on_robot_status, 10),
        ]
        for topic in self._joint_state_topics:
            self._subscriptions.append(
                self._node.create_subscription(
                    JointState,
                    topic,
                    lambda msg, joint_topic=topic: self._on_joint_state(joint_topic, msg),
                    10,
                )
            )

    def _create_command_clients(self) -> None:
        if self._node is None or ValidateCommand is None or ExecuteMotion is None or ActionClient is None:
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

    def _spin(self) -> None:  # pragma: no cover - requires ROS runtime
        assert self._executor is not None
        assert self._context is not None
        while not self._stop_requested and self._context.ok():
            self._executor.spin_once(timeout_sec=0.2)
            self._refresh_command_interface_state()

    def _refresh_command_interface_state(self) -> None:
        if self._node is None or self._validate_client is None or self._execute_client is None:
            return

        now = self._now()
        with self._lock:
            last_checked = self._state.command_interface_checked_at
            if last_checked is not None and (
                now - last_checked
            ).total_seconds() < self._command_interface_poll_period_sec:
                return
            self._state.command_interface_checked_at = now

        validate_ready = self._validate_client.service_is_ready()
        execute_ready = self._execute_client.server_is_ready()

        with self._lock:
            self._state.validate_command_ready = validate_ready
            self._state.execute_motion_ready = execute_ready
            self._state.validate_command_detail = (
                f"ready at {self._validate_command_service}" if validate_ready
                else f"waiting for {self._validate_command_service}"
            )
            self._state.execute_motion_detail = (
                f"ready at {self._execute_motion_action}" if execute_ready
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
            return self._state.validate_command_ready and self._state.execute_motion_ready

    def _command_interface_block_reason(self) -> str | None:
        with self._lock:
            if self._state.command_interface_error:
                return self._state.command_interface_error
            missing: list[str] = []
            if not self._state.validate_command_ready:
                missing.append(self._state.validate_command_detail or self._validate_command_service)
            if not self._state.execute_motion_ready:
                missing.append(self._state.execute_motion_detail or self._execute_motion_action)
        if not missing:
            return None
        return "command interfaces not ready: " + "; ".join(missing)

    def _command_interface_source_statuses(
        self,
        snapshot: _TelemetryState,
        runtime_mode: RuntimeMode,
    ) -> list[TelemetrySourceSnapshot]:
        active = runtime_mode in {RuntimeMode.SIM, RuntimeMode.HARDWARE}
        return [
            self._build_source_status(
                snapshot=snapshot,
                name='validate_command_service',
                label='ValidateCommand service',
                topic=self._validate_command_service,
                last_seen_at=snapshot.validate_command_ready_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC['command_interface'],
                active=active,
                detail=(
                    snapshot.validate_command_detail
                    if active
                    else 'Read-only outside command-capable modes.'
                ),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name='execute_motion_action',
                label='ExecuteMotion action',
                topic=self._execute_motion_action,
                last_seen_at=snapshot.execute_motion_ready_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC['command_interface'],
                active=active,
                detail=(
                    snapshot.execute_motion_detail
                    if active
                    else 'Read-only outside command-capable modes.'
                ),
            ),
        ]

    def _command_interface_health(self, snapshot: _TelemetryState, runtime_mode: RuntimeMode) -> ConnectionHealth:
        if snapshot.start_error:
            return ConnectionHealth.DOWN
        if runtime_mode not in {RuntimeMode.SIM, RuntimeMode.HARDWARE}:
            return ConnectionHealth.DOWN
        if snapshot.validate_command_ready and snapshot.execute_motion_ready:
            return ConnectionHealth.HEALTHY
        if snapshot.ros_started_at is not None:
            return ConnectionHealth.DEGRADED
        return ConnectionHealth.DOWN

    def _command_interface_active(self, runtime_mode: RuntimeMode) -> bool:
        return runtime_mode in {RuntimeMode.SIM, RuntimeMode.HARDWARE}

    def _command_interface_detail(self, snapshot: _TelemetryState, runtime_mode: RuntimeMode) -> str | None:
        if runtime_mode not in {RuntimeMode.SIM, RuntimeMode.HARDWARE}:
            return 'Command ingress stays read-only outside command-capable modes.'
        parts: list[str] = []
        if snapshot.validate_command_detail:
            parts.append(snapshot.validate_command_detail)
        if snapshot.execute_motion_detail:
            parts.append(snapshot.execute_motion_detail)
        return '; '.join(parts) if parts else None

    def _copy_state_locked(self) -> _TelemetryState:
        return _TelemetryState(
            ros_started_at=self._state.ros_started_at,
            start_error=self._state.start_error,
            robot_status=_RobotStatusState(
                received_at=self._state.robot_status.received_at,
                mode=self._state.robot_status.mode,
                e_stopped=self._state.robot_status.e_stopped,
                drives_powered=self._state.robot_status.drives_powered,
                motion_possible=self._state.robot_status.motion_possible,
                in_motion=self._state.robot_status.in_motion,
                in_error=self._state.robot_status.in_error,
                error_codes=list(self._state.robot_status.error_codes),
            ),
            readiness=_ReadinessState(
                received_at=self._state.readiness.received_at,
                ready=self._state.readiness.ready,
                status_message=self._state.readiness.status_message,
            ),
            supervisor_alert=_SupervisorAlertState(
                received_at=self._state.supervisor_alert.received_at,
                level=self._state.supervisor_alert.level,
                message=self._state.supervisor_alert.message,
                values=dict(self._state.supervisor_alert.values),
            ),
            llm=_LlmState(
                gateway_status_at=self._state.llm.gateway_status_at,
                gateway_status_text=self._state.llm.gateway_status_text,
                debug_at=self._state.llm.debug_at,
                command_at=self._state.llm.command_at,
            ),
            joint_positions_rad=dict(self._state.joint_positions_rad),
            joint_received_at=self._state.joint_received_at,
            joint_source_topic=self._state.joint_source_topic,
            joint_topic_received_at=dict(self._state.joint_topic_received_at),
            validate_command_ready_at=self._state.validate_command_ready_at,
            execute_motion_ready_at=self._state.execute_motion_ready_at,
            validate_command_ready=self._state.validate_command_ready,
            execute_motion_ready=self._state.execute_motion_ready,
            validate_command_detail=self._state.validate_command_detail,
            execute_motion_detail=self._state.execute_motion_detail,
            command_interface_checked_at=self._state.command_interface_checked_at,
            command_interface_check_inflight=False,
            command_interface_error=self._state.command_interface_error,
        )

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
        values = {str(item.key): str(item.value) for item in getattr(msg, 'values', [])}
        with self._lock:
            self._state.supervisor_alert.received_at = self._now()
            self._state.supervisor_alert.level = self._coerce_int(msg.level)
            self._state.supervisor_alert.message = str(msg.message)
            self._state.supervisor_alert.values = values

    def _on_robot_status(self, msg: Any) -> None:
        with self._lock:
            self._state.robot_status.received_at = self._now()
            self._state.robot_status.mode = self._coerce_int(msg.mode.val)
            self._state.robot_status.e_stopped = self._tri_state_to_bool(msg.e_stopped.val)
            self._state.robot_status.drives_powered = self._tri_state_to_bool(msg.drives_powered.val)
            self._state.robot_status.motion_possible = self._tri_state_to_bool(msg.motion_possible.val)
            self._state.robot_status.in_motion = self._tri_state_to_bool(msg.in_motion.val)
            self._state.robot_status.in_error = self._tri_state_to_bool(msg.in_error.val)
            self._state.robot_status.error_codes = [int(code) for code in msg.error_codes]

    def _on_joint_state(self, topic: str, msg: Any) -> None:
        joint_positions: dict[str, float] = {}
        for index, name in enumerate(getattr(msg, 'name', [])):
            if index < len(getattr(msg, 'position', [])):
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
            CONNECTION_FRESHNESS_SEC['joint_states'],
        )

    def _derive_mode(self, snapshot: _TelemetryState) -> RuntimeMode:
        readiness_text = snapshot.readiness.status_message.lower()
        if 'sim' in readiness_text:
            return RuntimeMode.SIM
        if snapshot.robot_status.received_at or snapshot.readiness.received_at:
            return RuntimeMode.HARDWARE
        return RuntimeMode.UNKNOWN

    def _derive_robot_status_snapshot(self, snapshot: _TelemetryState) -> RobotStatusSnapshot:
        if not self._is_fresh(snapshot.robot_status.received_at, CONNECTION_FRESHNESS_SEC['robot_status']):
            readiness_message = snapshot.readiness.status_message or 'Robot status topic is stale.'
            return RobotStatusSnapshot(readiness_message=readiness_message)

        drives_powered = snapshot.robot_status.drives_powered
        e_stopped = snapshot.robot_status.e_stopped
        in_error = snapshot.robot_status.in_error or bool(snapshot.robot_status.error_codes)

        return RobotStatusSnapshot(
            servo_state='ON' if drives_powered is True else 'OFF' if drives_powered is False else 'UNKNOWN',
            e_stop='ACTIVE' if e_stopped is True else 'CLEAR' if e_stopped is False else 'UNKNOWN',
            alarm_state='ACTIVE' if in_error else 'NONE' if in_error is False else 'UNKNOWN',
            motion_mode=self._robot_mode_to_string(snapshot.robot_status.mode),
            trajectory_points_used=None,
            trajectory_points_capacity=None,
            readiness_message=snapshot.readiness.status_message,
        )

    def _derive_runtime_state(
        self,
        snapshot: _TelemetryState,
        robot_status: RobotStatusSnapshot,
    ) -> tuple[SystemRuntimeState, str]:
        if snapshot.start_error:
            return SystemRuntimeState.LOST_CONN, f'ROS telemetry bridge unavailable: {snapshot.start_error}'

        ros_health = self._derive_ros_health(snapshot)
        if ros_health == ConnectionHealth.DOWN:
            return SystemRuntimeState.LOST_CONN, 'No fresh ROS telemetry received from configured read-only topics.'

        alert_text = ' '.join(
            filter(
                None,
                [
                    snapshot.supervisor_alert.message,
                    snapshot.supervisor_alert.values.get('reason', ''),
                    snapshot.supervisor_alert.values.get('message', ''),
                ],
            )
        ).lower()

        if snapshot.robot_status.e_stopped is True:
            return SystemRuntimeState.ESTOP, 'Emergency stop is active according to /yaskawa/robot_status.'

        if 'timeout' in alert_text:
            return SystemRuntimeState.TIMEOUT, snapshot.supervisor_alert.message or 'Supervisor reported a timeout condition.'

        if snapshot.robot_status.in_error is True or bool(snapshot.robot_status.error_codes):
            return SystemRuntimeState.FAULT, 'Robot controller reports an active fault condition.'

        if self._is_fresh(snapshot.supervisor_alert.received_at, CONNECTION_FRESHNESS_SEC['alerts']):
            alert_level = snapshot.supervisor_alert.level
            if alert_level is not None and alert_level >= 2:
                return SystemRuntimeState.FAULT, snapshot.supervisor_alert.message or 'Supervisor alert level indicates a fault.'
            if 'hold' in alert_text:
                return SystemRuntimeState.HOLD, snapshot.supervisor_alert.message or 'Supervisor reported HOLD state.'
            if 'blocked' in alert_text:
                return SystemRuntimeState.SAFETY_BLOCKED, snapshot.supervisor_alert.message or 'Supervisor reported safety blocked.'

        if self._is_fresh(snapshot.readiness.received_at, CONNECTION_FRESHNESS_SEC['readiness']):
            if snapshot.readiness.ready is False:
                return SystemRuntimeState.SAFETY_BLOCKED, snapshot.readiness.status_message

        return SystemRuntimeState.NORMAL, robot_status.readiness_message or 'Telemetry bridge connected and no blocking state is active.'

    def _derive_ros_health(self, snapshot: _TelemetryState) -> ConnectionHealth:
        if snapshot.start_error:
            return ConnectionHealth.DOWN
        if any(
            self._is_fresh(candidate, CONNECTION_FRESHNESS_SEC['ros'])
            for candidate in (
                snapshot.robot_status.received_at,
                snapshot.readiness.received_at,
                snapshot.joint_received_at,
                snapshot.supervisor_alert.received_at,
                snapshot.llm.gateway_status_at,
            )
        ):
            return ConnectionHealth.HEALTHY
        if snapshot.ros_started_at is not None:
            if self._is_fresh(snapshot.ros_started_at, CONNECTION_FRESHNESS_SEC['ros']):
                return ConnectionHealth.DEGRADED
            return ConnectionHealth.DOWN
        return ConnectionHealth.DOWN

    def _derive_llm_health(
        self,
        snapshot: _TelemetryState,
        ros_health: ConnectionHealth,
    ) -> ConnectionHealth:
        if snapshot.start_error:
            return ConnectionHealth.DOWN
        if any(
            self._is_fresh(candidate, CONNECTION_FRESHNESS_SEC['llm'])
            for candidate in (
                snapshot.llm.gateway_status_at,
                snapshot.llm.debug_at,
                snapshot.llm.command_at,
            )
        ):
            return ConnectionHealth.HEALTHY
        if ros_health != ConnectionHealth.DOWN:
            return ConnectionHealth.DEGRADED
        return ConnectionHealth.DOWN

    def _derive_moveit_health(
        self,
        snapshot: _TelemetryState,
        ros_health: ConnectionHealth,
    ) -> ConnectionHealth:
        if snapshot.start_error:
            return ConnectionHealth.DOWN
        if self._is_fresh(snapshot.readiness.received_at, CONNECTION_FRESHNESS_SEC['readiness']):
            return ConnectionHealth.HEALTHY if snapshot.readiness.ready else ConnectionHealth.DEGRADED
        if self._is_fresh(snapshot.supervisor_alert.received_at, CONNECTION_FRESHNESS_SEC['alerts']):
            return ConnectionHealth.DEGRADED
        if ros_health != ConnectionHealth.DOWN:
            return ConnectionHealth.DEGRADED
        return ConnectionHealth.DOWN

    def _derive_motoros2_health(
        self,
        snapshot: _TelemetryState,
        ros_health: ConnectionHealth,
    ) -> ConnectionHealth:
        if snapshot.start_error:
            return ConnectionHealth.DOWN
        if self._is_fresh(snapshot.robot_status.received_at, CONNECTION_FRESHNESS_SEC['robot_status']):
            return ConnectionHealth.HEALTHY
        if ros_health != ConnectionHealth.DOWN and self._derive_mode(snapshot) == RuntimeMode.SIM:
            return ConnectionHealth.DOWN
        if ros_health != ConnectionHealth.DOWN:
            return ConnectionHealth.DEGRADED
        return ConnectionHealth.DOWN

    def _build_source_status(
        self,
        *,
        snapshot: _TelemetryState,
        name: str,
        label: str,
        topic: str,
        last_seen_at: datetime | None,
        freshness_sec: float,
        preferred: bool = False,
        active: bool = True,
        detail: str | None = None,
    ) -> TelemetrySourceSnapshot:
        freshness_state = self._source_freshness_state(snapshot, last_seen_at, freshness_sec)
        return TelemetrySourceSnapshot(
            name=name,
            label=label,
            topic=topic,
            last_seen_at=last_seen_at,
            freshness_threshold_sec=freshness_sec,
            freshness_state=freshness_state,
            preferred=preferred,
            active=active,
            detail=detail,
        )

    def _source_freshness_state(
        self,
        snapshot: _TelemetryState,
        last_seen_at: datetime | None,
        freshness_sec: float,
    ) -> TelemetryFreshnessState:
        if snapshot.start_error:
            return TelemetryFreshnessState.UNAVAILABLE
        if last_seen_at is None:
            if snapshot.ros_started_at is None:
                return TelemetryFreshnessState.UNAVAILABLE
            return TelemetryFreshnessState.STALE
        if self._is_fresh(last_seen_at, freshness_sec):
            return TelemetryFreshnessState.FRESH
        return TelemetryFreshnessState.STALE

    def _robot_mode_to_string(self, mode: int | None) -> str | None:
        if mode is None or IndustrialRobotMode is None:
            return None
        if mode == IndustrialRobotMode.AUTO:
            return 'AUTO'
        if mode == IndustrialRobotMode.MANUAL:
            return 'MANUAL'
        return 'UNKNOWN'

    def _tri_state_to_bool(self, value: int) -> bool | None:
        value = self._coerce_int(value)
        if value < 0:
            return None
        return value > 0

    def _coerce_int(self, value: Any) -> int:
        if isinstance(value, (bytes, bytearray)):
            return int.from_bytes(value, byteorder='little', signed=True)
        return int(value)

    def _is_fresh(self, timestamp: datetime | None, max_age_sec: float) -> bool:
        if timestamp is None:
            return False
        return (self._now() - timestamp).total_seconds() <= max_age_sec

    def _now(self) -> datetime:
        return datetime.now(timezone.utc)
