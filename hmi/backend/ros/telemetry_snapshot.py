from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from math import degrees
from threading import Lock, Thread

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


CONNECTION_FRESHNESS_SEC = {
    "ros": 3.0,
    "robot_status": 3.0,
    "readiness": 3.0,
    "joint_states": 3.0,
    "command_interface": 3.0,
    "llm": 30.0,
    "alerts": 5.0,
}


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
    status_message: str = "No readiness signal received."


@dataclass(slots=True)
class _SupervisorAlertState:
    received_at: datetime | None = None
    level: int | None = None
    message: str = ""
    values: dict[str, str] = field(default_factory=dict)


@dataclass(slots=True)
class _LlmState:
    gateway_status_at: datetime | None = None
    gateway_status_text: str = ""
    debug_at: datetime | None = None
    command_at: datetime | None = None


@dataclass(slots=True)
class _TelemetryState:
    ros_started_at: datetime | None = None
    start_error: str | None = None
    robot_status: _RobotStatusState = field(default_factory=_RobotStatusState)
    readiness: _ReadinessState = field(default_factory=_ReadinessState)
    supervisor_alert: _SupervisorAlertState = field(
        default_factory=_SupervisorAlertState
    )
    llm: _LlmState = field(default_factory=_LlmState)
    joint_positions_rad: dict[str, float] = field(default_factory=dict)
    joint_received_at: datetime | None = None
    joint_source_topic: str | None = None
    joint_topic_received_at: dict[str, datetime] = field(default_factory=dict)
    validate_command_ready_at: datetime | None = None
    execute_motion_ready_at: datetime | None = None
    validate_command_ready: bool = False
    execute_motion_ready: bool = False
    validate_command_detail: str = ""
    execute_motion_detail: str = ""
    # W5.T4 — new HMI consolidation service readiness
    hydrate_workplane_ready_at: datetime | None = None
    get_primitive_constants_ready_at: datetime | None = None
    review_intent_ready_at: datetime | None = None
    confirm_execution_ready_at: datetime | None = None
    hydrate_workplane_ready: bool = False
    get_primitive_constants_ready: bool = False
    review_intent_ready: bool = False
    confirm_execution_ready: bool = False
    command_interface_checked_at: datetime | None = None
    command_interface_check_inflight: bool = False
    command_interface_error: str | None = None
    command_interface_thread: Thread | None = None
    command_interface_lock: Lock = field(default_factory=Lock)


class TelemetrySnapshotMixin:
    def read_connections(self) -> list[BridgeConnection]:
        with self._lock:
            snapshot = self._copy_state_locked()

        ros_health = self._derive_ros_health(snapshot)
        llm_health = self._derive_llm_health(snapshot, ros_health)
        moveit_health = self._derive_moveit_health(snapshot, ros_health)
        motoros2_health = self._derive_motoros2_health(snapshot, ros_health)

        return [
            BridgeConnection(name="ros2", label="ROS 2", health=ros_health),
            BridgeConnection(name="moveit2", label="MoveIt 2", health=moveit_health),
            BridgeConnection(name="llm", label="LLM", health=llm_health),
            BridgeConnection(name="motoros2", label="MotoROS2", health=motoros2_health),
        ]

    def read_runtime_snapshot(self) -> RuntimeSnapshot:
        with self._lock:
            snapshot = self._copy_state_locked()

        mode = self._derive_mode(snapshot)
        robot_status = self._derive_robot_status_snapshot(snapshot)
        runtime_state, status_text = self._derive_runtime_state(snapshot, robot_status)
        return RuntimeSnapshot(
            system_state=runtime_state,
            blocking=runtime_state
            in {
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
        joint_data_is_fresh = self._is_fresh(
            snapshot.joint_received_at, CONNECTION_FRESHNESS_SEC["joint_states"]
        )
        for joint_name in DEFAULT_JOINT_NAMES:
            position_rad = (
                snapshot.joint_positions_rad.get(joint_name)
                if joint_data_is_fresh
                else None
            )
            position_deg = degrees(position_rad) if position_rad is not None else None
            joint_positions.append(
                JointPosition(name=joint_name, position_deg=position_deg)
            )
        return joint_positions

    def read_source_statuses(self) -> list[TelemetrySourceSnapshot]:
        with self._lock:
            snapshot = self._copy_state_locked()

        runtime_mode = self._derive_mode(snapshot)
        joint_fallback_topic = self._joint_state_topics[-1]
        preferred_joint_topic = (
            joint_fallback_topic
            if runtime_mode == RuntimeMode.SIM
            else self._preferred_joint_state_topic
        )

        statuses = [
            self._build_source_status(
                snapshot=snapshot,
                name="gateway_status",
                label="Gateway status",
                topic=self._gateway_status_topic,
                last_seen_at=snapshot.llm.gateway_status_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC["llm"],
                detail=snapshot.llm.gateway_status_text or None,
            ),
            self._build_source_status(
                snapshot=snapshot,
                name="llm_debug",
                label="LLM debug",
                topic=self._llm_debug_topic,
                last_seen_at=snapshot.llm.debug_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC["llm"],
                active=self._is_fresh(
                    snapshot.llm.debug_at, CONNECTION_FRESHNESS_SEC["llm"]
                ),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name="llm_command",
                label="LLM command echo",
                topic=self._llm_command_topic,
                last_seen_at=snapshot.llm.command_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC["llm"],
                active=self._is_fresh(
                    snapshot.llm.command_at, CONNECTION_FRESHNESS_SEC["llm"]
                ),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name="readiness",
                label="HW readiness",
                topic=self._readiness_topic,
                last_seen_at=snapshot.readiness.received_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC["readiness"],
                detail=snapshot.readiness.status_message,
            ),
            self._build_source_status(
                snapshot=snapshot,
                name="supervisor_alerts",
                label="Supervisor alerts",
                topic=self._supervisor_alert_topic,
                last_seen_at=snapshot.supervisor_alert.received_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC["alerts"],
                detail=snapshot.supervisor_alert.message or None,
            ),
            self._build_source_status(
                snapshot=snapshot,
                name="robot_status",
                label="Robot status",
                topic=self._robot_status_topic,
                last_seen_at=snapshot.robot_status.received_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC["robot_status"],
                active=runtime_mode != RuntimeMode.SIM,
                detail=(
                    "SIM mode uses /hw_adapter/ready instead of raw /yaskawa/robot_status."
                    if runtime_mode == RuntimeMode.SIM
                    else None
                ),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name="joint_states_primary",
                label="Joint states primary",
                topic=self._preferred_joint_state_topic,
                last_seen_at=snapshot.joint_topic_received_at.get(
                    self._preferred_joint_state_topic
                ),
                freshness_sec=CONNECTION_FRESHNESS_SEC["joint_states"],
                preferred=self._preferred_joint_state_topic == preferred_joint_topic,
                active=snapshot.joint_source_topic == self._preferred_joint_state_topic,
                detail=(
                    f"SIM mode prefers {joint_fallback_topic}."
                    if runtime_mode == RuntimeMode.SIM
                    and self._preferred_joint_state_topic != preferred_joint_topic
                    else None
                ),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name="joint_states_fallback",
                label="Joint states fallback",
                topic=joint_fallback_topic,
                last_seen_at=snapshot.joint_topic_received_at.get(joint_fallback_topic),
                freshness_sec=CONNECTION_FRESHNESS_SEC["joint_states"],
                preferred=joint_fallback_topic == preferred_joint_topic,
                active=snapshot.joint_source_topic == joint_fallback_topic,
            ),
        ]
        statuses.extend(self._command_interface_source_statuses(snapshot, runtime_mode))
        return statuses

    def _command_interface_source_statuses(
        self,
        snapshot: _TelemetryState,
        runtime_mode: RuntimeMode,
    ) -> list[TelemetrySourceSnapshot]:
        active = runtime_mode in {RuntimeMode.SIM, RuntimeMode.HARDWARE}
        return [
            self._build_source_status(
                snapshot=snapshot,
                name="validate_command_service",
                label="ValidateCommand service",
                topic=self._validate_command_service,
                last_seen_at=snapshot.validate_command_ready_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC["command_interface"],
                active=active and snapshot.validate_command_ready,
                detail=(
                    snapshot.validate_command_detail
                    if active
                    else "Read-only outside command-capable modes."
                ),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name="execute_motion_action",
                label="ExecuteMotion action",
                topic=self._execute_motion_action,
                last_seen_at=snapshot.execute_motion_ready_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC["command_interface"],
                active=active and snapshot.execute_motion_ready,
                detail=(
                    snapshot.execute_motion_detail
                    if active
                    else "Read-only outside command-capable modes."
                ),
            ),
            self._build_source_status(
                snapshot=snapshot,
                name="review_intent_service",
                label="ReviewIntent service",
                topic=self._review_intent_service,
                last_seen_at=snapshot.review_intent_ready_at,
                freshness_sec=CONNECTION_FRESHNESS_SEC["command_interface"],
                active=active and snapshot.review_intent_ready,
                detail=(
                    "ready at " + self._review_intent_service
                    if snapshot.review_intent_ready
                    else "waiting for " + self._review_intent_service
                )
                if active
                else "Read-only outside command-capable modes.",
            ),
        ]

    def _command_interface_health(
        self, snapshot: _TelemetryState, runtime_mode: RuntimeMode
    ) -> ConnectionHealth:
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

    def _command_interface_detail(
        self, snapshot: _TelemetryState, runtime_mode: RuntimeMode
    ) -> str | None:
        if runtime_mode not in {RuntimeMode.SIM, RuntimeMode.HARDWARE}:
            return "Command ingress stays read-only outside command-capable modes."
        parts: list[str] = []
        if snapshot.validate_command_detail:
            parts.append(snapshot.validate_command_detail)
        if snapshot.execute_motion_detail:
            parts.append(snapshot.execute_motion_detail)
        if snapshot.review_intent_ready:
            parts.append(f"ready at {self._review_intent_service}")
        elif runtime_mode in {RuntimeMode.SIM, RuntimeMode.HARDWARE}:
            parts.append(f"waiting for {self._review_intent_service}")
        return "; ".join(parts) if parts else None

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
            review_intent_ready_at=self._state.review_intent_ready_at,
            validate_command_ready=self._state.validate_command_ready,
            execute_motion_ready=self._state.execute_motion_ready,
            review_intent_ready=self._state.review_intent_ready,
            validate_command_detail=self._state.validate_command_detail,
            execute_motion_detail=self._state.execute_motion_detail,
            hydrate_workplane_ready_at=self._state.hydrate_workplane_ready_at,
            get_primitive_constants_ready_at=self._state.get_primitive_constants_ready_at,
            confirm_execution_ready_at=self._state.confirm_execution_ready_at,
            hydrate_workplane_ready=self._state.hydrate_workplane_ready,
            get_primitive_constants_ready=self._state.get_primitive_constants_ready,
            confirm_execution_ready=self._state.confirm_execution_ready,
            command_interface_checked_at=self._state.command_interface_checked_at,
            command_interface_check_inflight=False,
            command_interface_error=self._state.command_interface_error,
        )

    def _derive_mode(self, snapshot: _TelemetryState) -> RuntimeMode:
        readiness_text = snapshot.readiness.status_message.lower()
        if "sim" in readiness_text:
            return RuntimeMode.SIM
        if snapshot.robot_status.received_at or snapshot.readiness.received_at:
            return RuntimeMode.HARDWARE
        return RuntimeMode.UNKNOWN

    def _derive_robot_status_snapshot(
        self, snapshot: _TelemetryState
    ) -> RobotStatusSnapshot:
        if not self._is_fresh(
            snapshot.robot_status.received_at, CONNECTION_FRESHNESS_SEC["robot_status"]
        ):
            readiness_message = (
                snapshot.readiness.status_message or "Robot status topic is stale."
            )
            return RobotStatusSnapshot(readiness_message=readiness_message)

        drives_powered = snapshot.robot_status.drives_powered
        e_stopped = snapshot.robot_status.e_stopped
        in_error = snapshot.robot_status.in_error or bool(
            snapshot.robot_status.error_codes
        )

        return RobotStatusSnapshot(
            servo_state="ON"
            if drives_powered is True
            else "OFF"
            if drives_powered is False
            else "UNKNOWN",
            e_stop="ACTIVE"
            if e_stopped is True
            else "CLEAR"
            if e_stopped is False
            else "UNKNOWN",
            alarm_state="ACTIVE"
            if in_error
            else "NONE"
            if in_error is False
            else "UNKNOWN",
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
            return (
                SystemRuntimeState.LOST_CONN,
                f"ROS telemetry bridge unavailable: {snapshot.start_error}",
            )

        ros_health = self._derive_ros_health(snapshot)
        if ros_health == ConnectionHealth.DOWN:
            return (
                SystemRuntimeState.LOST_CONN,
                "No fresh ROS telemetry received from configured read-only topics.",
            )

        alert_text = " ".join(
            filter(
                None,
                [
                    snapshot.supervisor_alert.message,
                    snapshot.supervisor_alert.values.get("reason", ""),
                    snapshot.supervisor_alert.values.get("message", ""),
                ],
            )
        ).lower()

        if snapshot.robot_status.e_stopped is True:
            return (
                SystemRuntimeState.ESTOP,
                "Emergency stop is active according to /yaskawa/robot_status.",
            )

        if "timeout" in alert_text:
            return (
                SystemRuntimeState.TIMEOUT,
                snapshot.supervisor_alert.message
                or "Supervisor reported a timeout condition.",
            )

        if snapshot.robot_status.in_error is True or bool(
            snapshot.robot_status.error_codes
        ):
            return (
                SystemRuntimeState.FAULT,
                "Robot controller reports an active fault condition.",
            )

        if self._is_fresh(
            snapshot.supervisor_alert.received_at, CONNECTION_FRESHNESS_SEC["alerts"]
        ):
            alert_level = snapshot.supervisor_alert.level
            if alert_level is not None and alert_level >= 2:
                return (
                    SystemRuntimeState.FAULT,
                    snapshot.supervisor_alert.message
                    or "Supervisor alert level indicates a fault.",
                )
            if "hold" in alert_text:
                return (
                    SystemRuntimeState.HOLD,
                    snapshot.supervisor_alert.message
                    or "Supervisor reported HOLD state.",
                )
            if "blocked" in alert_text:
                return (
                    SystemRuntimeState.SAFETY_BLOCKED,
                    snapshot.supervisor_alert.message
                    or "Supervisor reported safety blocked.",
                )

        if self._is_fresh(
            snapshot.readiness.received_at, CONNECTION_FRESHNESS_SEC["readiness"]
        ):
            if snapshot.readiness.ready is False:
                return (
                    SystemRuntimeState.SAFETY_BLOCKED,
                    snapshot.readiness.status_message,
                )

        return (
            SystemRuntimeState.NORMAL,
            robot_status.readiness_message
            or "Telemetry bridge connected and no blocking state is active.",
        )

    def _derive_ros_health(self, snapshot: _TelemetryState) -> ConnectionHealth:
        if snapshot.start_error:
            return ConnectionHealth.DOWN
        if any(
            self._is_fresh(candidate, CONNECTION_FRESHNESS_SEC["ros"])
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
            if self._is_fresh(snapshot.ros_started_at, CONNECTION_FRESHNESS_SEC["ros"]):
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
            self._is_fresh(candidate, CONNECTION_FRESHNESS_SEC["llm"])
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
        if self._is_fresh(
            snapshot.readiness.received_at, CONNECTION_FRESHNESS_SEC["readiness"]
        ):
            return (
                ConnectionHealth.HEALTHY
                if snapshot.readiness.ready
                else ConnectionHealth.DEGRADED
            )
        if self._is_fresh(
            snapshot.supervisor_alert.received_at, CONNECTION_FRESHNESS_SEC["alerts"]
        ):
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
        if self._is_fresh(
            snapshot.robot_status.received_at, CONNECTION_FRESHNESS_SEC["robot_status"]
        ):
            return ConnectionHealth.HEALTHY
        if (
            ros_health != ConnectionHealth.DOWN
            and self._derive_mode(snapshot) == RuntimeMode.SIM
        ):
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
        freshness_state = self._source_freshness_state(
            snapshot, last_seen_at, freshness_sec
        )
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
        industrial_robot_mode = getattr(self, "_industrial_robot_mode", None)
        if mode is None or industrial_robot_mode is None:
            return None
        if mode == industrial_robot_mode.AUTO:
            return "AUTO"
        if mode == industrial_robot_mode.MANUAL:
            return "MANUAL"
        return "UNKNOWN"
