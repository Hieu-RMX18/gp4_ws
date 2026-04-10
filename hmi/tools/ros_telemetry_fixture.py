#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path
import sys
import time
from dataclasses import dataclass
from typing import Any


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
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from industrial_msgs.msg import RobotMode, RobotStatus, TriState
    from interfaces.msg import RobotReadiness
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
    JointState = _load_joint_state_type()
except Exception as exc:  # pragma: no cover - depends on sourced ROS environment
    print(f"ros_telemetry_fixture import error: {exc}", file=sys.stderr)
    raise


JOINT_NAMES = (
    "joint_1_s",
    "joint_2_l",
    "joint_3_u",
    "joint_4_r",
    "joint_5_b",
    "joint_6_t",
)


@dataclass(frozen=True)
class Phase:
    name: str
    duration_sec: float
    publish_gateway_status: bool = True
    publish_llm_debug: bool = True
    publish_llm_command: bool = True
    publish_readiness: bool = True
    publish_supervisor_alert: bool = True
    publish_robot_status: bool = True
    publish_joint_primary: bool = True
    publish_joint_fallback: bool = False
    status_rate_hz: float = 5.0
    joint_rate_hz: float = 20.0
    gateway_text: str = "gateway connected"
    llm_debug_text: str = "llm debug"
    llm_command_text: str = "llm echo"
    readiness_ready: bool = True
    readiness_message: str = "hardware ready"
    alert_level: int = DiagnosticStatus.OK
    alert_message: str = "normal telemetry"
    alert_reason: str = "normal"
    alert_state: str = "IDLE"
    robot_mode: int = RobotMode.AUTO
    e_stopped: int = TriState.FALSE
    drives_powered: int = TriState.TRUE
    motion_possible: int = TriState.TRUE
    in_motion: int = TriState.FALSE
    in_error: int = TriState.FALSE
    error_codes: tuple[int, ...] = ()
    primary_joint_offset_deg: float = 0.0
    fallback_joint_offset_deg: float = 35.0
    joint_velocity_deg_per_sec: float = 12.0


def build_scenarios() -> dict[str, list[Phase]]:
    return {
        "normal": [
            Phase(
                name="normal",
                duration_sec=8.0,
                publish_joint_fallback=True,
                readiness_message="hardware ready",
                alert_message="normal telemetry",
                alert_reason="normal",
            ),
        ],
        "safety_blocked": [
            Phase(
                name="safety_blocked",
                duration_sec=8.0,
                readiness_ready=False,
                readiness_message="fixture safety blocked",
                alert_level=DiagnosticStatus.WARN,
                alert_message="fixture safety blocked",
                alert_reason="blocked",
                alert_state="BLOCKED",
                publish_joint_fallback=True,
            ),
        ],
        "fault": [
            Phase(
                name="fault",
                duration_sec=8.0,
                in_error=TriState.TRUE,
                error_codes=(42,),
                alert_level=DiagnosticStatus.ERROR,
                alert_message="fixture controller fault",
                alert_reason="fault",
                alert_state="FAULT",
                publish_joint_fallback=True,
            ),
        ],
        "estop_over_fault_and_blocked": [
            Phase(
                name="estop_over_fault_and_blocked",
                duration_sec=8.0,
                e_stopped=TriState.TRUE,
                in_error=TriState.TRUE,
                error_codes=(42,),
                readiness_ready=False,
                readiness_message="fixture blocked while estop active",
                alert_level=DiagnosticStatus.ERROR,
                alert_message="fixture safety blocked while estop active",
                alert_reason="blocked",
                alert_state="ESTOP",
                publish_joint_fallback=True,
            ),
        ],
        "fault_over_blocked": [
            Phase(
                name="fault_over_blocked",
                duration_sec=8.0,
                in_error=TriState.TRUE,
                error_codes=(42,),
                readiness_ready=False,
                readiness_message="fixture blocked while fault active",
                alert_level=DiagnosticStatus.WARN,
                alert_message="fixture safety blocked while fault active",
                alert_reason="blocked",
                alert_state="FAULT",
                publish_joint_fallback=True,
            ),
        ],
        "hold_alert": [
            Phase(
                name="hold_alert",
                duration_sec=8.0,
                alert_level=DiagnosticStatus.WARN,
                alert_message="fixture hold active",
                alert_reason="hold",
                alert_state="HOLD",
                publish_joint_fallback=True,
            ),
        ],
        "timeout_alert": [
            Phase(
                name="timeout_alert",
                duration_sec=8.0,
                alert_level=DiagnosticStatus.WARN,
                alert_message="fixture timeout active",
                alert_reason="timeout",
                alert_state="TIMEOUT",
                publish_joint_fallback=True,
            ),
        ],
        "partial_stale_readiness": [
            Phase(
                name="warmup",
                duration_sec=2.0,
                publish_joint_fallback=True,
                readiness_message="warmup ready",
            ),
            Phase(
                name="readiness_stale",
                duration_sec=4.5,
                publish_readiness=False,
                publish_joint_fallback=True,
                alert_message="readiness paused",
                alert_reason="normal",
            ),
        ],
        "joint_failover": [
            Phase(
                name="primary_preferred",
                duration_sec=2.0,
                publish_joint_primary=True,
                publish_joint_fallback=True,
                readiness_message="joint failover warmup",
            ),
            Phase(
                name="fallback_only",
                duration_sec=4.5,
                publish_joint_primary=False,
                publish_joint_fallback=True,
                readiness_message="joint failover running",
            ),
        ],
        "lost_conn_recover": [
            Phase(
                name="connected",
                duration_sec=2.0,
                publish_joint_fallback=True,
                readiness_message="fixture connected",
            ),
            Phase(
                name="all_topics_paused",
                duration_sec=6.0,
                publish_gateway_status=False,
                publish_llm_debug=False,
                publish_llm_command=False,
                publish_readiness=False,
                publish_supervisor_alert=False,
                publish_robot_status=False,
                publish_joint_primary=False,
                publish_joint_fallback=False,
            ),
            Phase(
                name="recovered",
                duration_sec=3.0,
                publish_joint_fallback=True,
                readiness_message="fixture recovered",
            ),
        ],
        "burst_joint_states": [
            Phase(
                name="burst_joint_states",
                duration_sec=10.0,
                status_rate_hz=20.0,
                joint_rate_hz=200.0,
                publish_joint_fallback=True,
                readiness_message="burst mode ready",
                joint_velocity_deg_per_sec=80.0,
            ),
        ],
    }


class TelemetryFixtureNode(Node):
    def __init__(self, scenario_name: str, phases: list[Phase]) -> None:
        super().__init__("gp4_hmi_ros_telemetry_fixture")
        self._scenario_name = scenario_name
        self._phases = phases
        self._started_at = time.monotonic()
        self._last_publish_at: dict[str, float] = {}
        self._active_phase_name: str | None = None
        self._done = False

        self._gateway_pub = self.create_publisher(String, "/gateway_status", 10)
        self._llm_debug_pub = self.create_publisher(String, "/llm_debug", 10)
        self._llm_command_pub = self.create_publisher(String, "/llm_command", 10)
        self._readiness_pub = self.create_publisher(RobotReadiness, "/hw_adapter/ready", 10)
        self._alerts_pub = self.create_publisher(DiagnosticStatus, "/supervisor/alerts", 10)
        self._robot_status_pub = self.create_publisher(RobotStatus, "/yaskawa/robot_status", 10)
        self._joint_primary_pub = self.create_publisher(JointState, "/yaskawa/joint_states", 10)
        self._joint_fallback_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.create_timer(0.01, self._tick)

    @property
    def done(self) -> bool:
        return self._done

    def _tick(self) -> None:
        phase, phase_started_at = self._phase_at(time.monotonic())
        if phase is None:
            if not self._done:
                self.get_logger().info(
                    f"Scenario '{self._scenario_name}' completed. Fixture will stop."
                )
                self._done = True
            return

        if phase.name != self._active_phase_name:
            self._active_phase_name = phase.name
            self.get_logger().info(
                f"Scenario '{self._scenario_name}' entering phase '{phase.name}'."
            )

        self._maybe_publish_string("gateway_status", phase.publish_gateway_status, phase.status_rate_hz, self._gateway_pub, phase.gateway_text)
        self._maybe_publish_string("llm_debug", phase.publish_llm_debug, phase.status_rate_hz, self._llm_debug_pub, phase.llm_debug_text)
        self._maybe_publish_string("llm_command", phase.publish_llm_command, phase.status_rate_hz, self._llm_command_pub, phase.llm_command_text)

        if phase.publish_readiness:
            self._maybe_publish(
                "readiness",
                phase.status_rate_hz,
                lambda: self._readiness_pub.publish(self._build_readiness(phase)),
            )
        if phase.publish_supervisor_alert:
            self._maybe_publish(
                "supervisor_alerts",
                phase.status_rate_hz,
                lambda: self._alerts_pub.publish(self._build_alert(phase)),
            )
        if phase.publish_robot_status:
            self._maybe_publish(
                "robot_status",
                phase.status_rate_hz,
                lambda: self._robot_status_pub.publish(self._build_robot_status(phase)),
            )
        if phase.publish_joint_primary:
            self._maybe_publish(
                "joint_primary",
                phase.joint_rate_hz,
                lambda: self._joint_primary_pub.publish(
                    self._build_joint_state(
                        phase,
                        phase_started_at=phase_started_at,
                        offset_deg=phase.primary_joint_offset_deg,
                    )
                ),
            )
        if phase.publish_joint_fallback:
            self._maybe_publish(
                "joint_fallback",
                phase.joint_rate_hz,
                lambda: self._joint_fallback_pub.publish(
                    self._build_joint_state(
                        phase,
                        phase_started_at=phase_started_at,
                        offset_deg=phase.fallback_joint_offset_deg,
                    )
                ),
            )

    def _phase_at(self, now_monotonic: float) -> tuple[Phase | None, float]:
        elapsed = now_monotonic - self._started_at
        phase_start = self._started_at
        for phase in self._phases:
            if elapsed <= phase.duration_sec:
                return phase, phase_start
            elapsed -= phase.duration_sec
            phase_start += phase.duration_sec
        return None, phase_start

    def _maybe_publish_string(
        self,
        key: str,
        enabled: bool,
        rate_hz: float,
        publisher: Any,
        text: str,
    ) -> None:
        if not enabled:
            return
        self._maybe_publish(key, rate_hz, lambda: publisher.publish(String(data=text)))

    def _maybe_publish(self, key: str, rate_hz: float, publish_fn) -> None:
        now_monotonic = time.monotonic()
        interval = 1.0 / max(rate_hz, 0.1)
        last_published = self._last_publish_at.get(key)
        if last_published is not None and (now_monotonic - last_published) < interval:
            return
        publish_fn()
        self._last_publish_at[key] = now_monotonic

    def _build_readiness(self, phase: Phase) -> RobotReadiness:
        message = RobotReadiness()
        message.ready = phase.readiness_ready
        message.status_message = phase.readiness_message
        return message

    def _build_alert(self, phase: Phase) -> DiagnosticStatus:
        alert = DiagnosticStatus()
        alert.level = phase.alert_level
        alert.name = "hmi/telemetry_fixture"
        alert.message = phase.alert_message
        alert.hardware_id = "fixture"
        alert.values = [
            KeyValue(key="reason", value=phase.alert_reason),
            KeyValue(key="state", value=phase.alert_state),
            KeyValue(key="active_goal_id", value=""),
            KeyValue(key="consecutive_failure_count", value="0"),
            KeyValue(key="velocity_scale", value="0.0"),
            KeyValue(key="expected_duration_sec", value="0.0"),
            KeyValue(key="allowed_duration_sec", value="0.0"),
            KeyValue(key="last_execution_time_sec", value="0.0"),
        ]
        return alert

    def _build_robot_status(self, phase: Phase) -> RobotStatus:
        status = RobotStatus()
        status.mode.val = phase.robot_mode
        status.e_stopped.val = phase.e_stopped
        status.drives_powered.val = phase.drives_powered
        status.motion_possible.val = phase.motion_possible
        status.in_motion.val = phase.in_motion
        status.in_error.val = phase.in_error
        status.error_codes = list(phase.error_codes)
        status.header.stamp = self.get_clock().now().to_msg()
        return status

    def _build_joint_state(
        self,
        phase: Phase,
        *,
        phase_started_at: float,
        offset_deg: float,
    ) -> JointState:
        elapsed = time.monotonic() - phase_started_at
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(JOINT_NAMES)

        base_deg = offset_deg + elapsed * phase.joint_velocity_deg_per_sec
        joint_state.position = [
            math.radians(base_deg + (index * 7.5))
            for index, _joint_name in enumerate(JOINT_NAMES)
        ]
        return joint_state


def build_parser() -> argparse.ArgumentParser:
    scenarios = build_scenarios()
    parser = argparse.ArgumentParser(description="Read-only ROS telemetry fixture for HMI validation.")
    parser.add_argument(
        "--scenario",
        choices=sorted(scenarios.keys()),
        help="Fixture scenario to publish.",
    )
    parser.add_argument(
        "--list-scenarios",
        action="store_true",
        help="List available scenarios and exit.",
    )
    return parser


def main() -> int:
    scenarios = build_scenarios()
    parser = build_parser()
    args = parser.parse_args()

    if args.list_scenarios:
        for name, phases in sorted(scenarios.items()):
            total_duration = sum(phase.duration_sec for phase in phases)
            print(f"{name}: {total_duration:.1f}s")
        return 0

    if not args.scenario:
        parser.error("--scenario is required unless --list-scenarios is used.")

    phases = scenarios[args.scenario]
    rclpy.init()
    node = TelemetryFixtureNode(args.scenario, phases)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Telemetry fixture interrupted by user.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
