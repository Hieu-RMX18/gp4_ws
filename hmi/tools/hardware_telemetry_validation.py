#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
import sys
import time
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
    from diagnostic_msgs.msg import DiagnosticStatus
    from industrial_msgs.msg import RobotMode, RobotStatus, TriState
    from interfaces.msg import RobotReadiness
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    JointState = _load_joint_state_type()
except Exception as exc:  # pragma: no cover - depends on sourced ROS environment
    print(f"hardware_telemetry_validation import error: {exc}", file=sys.stderr)
    raise


DEFAULT_THRESHOLDS_SEC = {
    "gateway_status": 30.0,
    "readiness": 3.0,
    "supervisor_alerts": 5.0,
    "robot_status": 3.0,
    "joint_states_primary": 3.0,
    "joint_states_fallback": 3.0,
}


def utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def tri_state_name(value: int) -> str:
    if value == TriState.TRUE:
        return "TRUE"
    if value == TriState.FALSE:
        return "FALSE"
    return "UNKNOWN"


def robot_mode_name(value: int) -> str:
    if value == RobotMode.AUTO:
        return "AUTO"
    if value == RobotMode.MANUAL:
        return "MANUAL"
    if value == RobotMode.UNKNOWN:
        return "UNKNOWN"
    return str(value)


def numeric_ros_value(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError(f"expected single-byte ROS value, got {len(value)} bytes")
        return value[0]
    return int(value)


@dataclass(slots=True)
class SourceMetric:
    name: str
    topic: str
    threshold_sec: float
    message_count: int = 0
    first_seen_at: str | None = None
    last_seen_at: str | None = None
    last_seen_monotonic: float | None = None
    min_interval_sec: float | None = None
    max_interval_sec: float | None = None
    interval_sum_sec: float = 0.0
    interval_count: int = 0
    max_gap_sec: float = 0.0
    freshness_state: str = "unavailable"
    transitions: list[dict[str, Any]] = field(default_factory=list)

    def record_message(self, observed_at_monotonic: float) -> None:
        observed_at_wall = utcnow_iso()
        if self.message_count == 0:
            self.first_seen_at = observed_at_wall
        if self.last_seen_monotonic is not None:
            interval_sec = max(0.0, observed_at_monotonic - self.last_seen_monotonic)
            self.min_interval_sec = interval_sec if self.min_interval_sec is None else min(self.min_interval_sec, interval_sec)
            self.max_interval_sec = interval_sec if self.max_interval_sec is None else max(self.max_interval_sec, interval_sec)
            self.interval_sum_sec += interval_sec
            self.interval_count += 1
            self.max_gap_sec = max(self.max_gap_sec, interval_sec)
        self.message_count += 1
        self.last_seen_at = observed_at_wall
        self.last_seen_monotonic = observed_at_monotonic

    def refresh_freshness(self, now_monotonic: float) -> None:
        if self.message_count == 0 or self.last_seen_monotonic is None:
            next_state = "unavailable"
            age_sec = None
        else:
            age_sec = max(0.0, now_monotonic - self.last_seen_monotonic)
            next_state = "fresh" if age_sec <= self.threshold_sec else "stale"
            self.max_gap_sec = max(self.max_gap_sec, age_sec)

        if next_state != self.freshness_state:
            self.transitions.append(
                {
                    "timestamp": utcnow_iso(),
                    "from": self.freshness_state,
                    "to": next_state,
                    "ageSec": round(age_sec, 6) if age_sec is not None else None,
                }
            )
            self.freshness_state = next_state

    def to_dict(self) -> dict[str, Any]:
        mean_interval_sec = (
            self.interval_sum_sec / self.interval_count if self.interval_count > 0 else None
        )
        mean_rate_hz = (
            1.0 / mean_interval_sec if mean_interval_sec and mean_interval_sec > 0.0 else None
        )
        return {
            "topic": self.topic,
            "thresholdSec": self.threshold_sec,
            "messageCount": self.message_count,
            "firstSeenAt": self.first_seen_at,
            "lastSeenAt": self.last_seen_at,
            "freshnessState": self.freshness_state,
            "minIntervalSec": round(self.min_interval_sec, 6) if self.min_interval_sec is not None else None,
            "maxIntervalSec": round(self.max_interval_sec, 6) if self.max_interval_sec is not None else None,
            "meanIntervalSec": round(mean_interval_sec, 6) if mean_interval_sec is not None else None,
            "meanRateHz": round(mean_rate_hz, 6) if mean_rate_hz is not None else None,
            "maxGapSec": round(self.max_gap_sec, 6),
            "thresholdMarginSec": (
                round(self.threshold_sec - self.max_gap_sec, 6)
                if self.message_count > 0
                else None
            ),
            "transitions": self.transitions,
        }


class HardwareTelemetryValidationNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("gp4_hmi_hardware_telemetry_validation")
        self._duration_sec = max(args.duration_sec, 1.0)
        self._started_monotonic = time.monotonic()
        self._started_at_utc = utcnow_iso()
        self._deadline_monotonic = self._started_monotonic + self._duration_sec
        self._done = False

        self._metrics: dict[str, SourceMetric] = {
            "gateway_status": SourceMetric(
                name="gateway_status",
                topic=args.gateway_status_topic,
                threshold_sec=DEFAULT_THRESHOLDS_SEC["gateway_status"],
            ),
            "readiness": SourceMetric(
                name="readiness",
                topic=args.readiness_topic,
                threshold_sec=DEFAULT_THRESHOLDS_SEC["readiness"],
            ),
            "supervisor_alerts": SourceMetric(
                name="supervisor_alerts",
                topic=args.supervisor_alert_topic,
                threshold_sec=DEFAULT_THRESHOLDS_SEC["supervisor_alerts"],
            ),
            "robot_status": SourceMetric(
                name="robot_status",
                topic=args.robot_status_topic,
                threshold_sec=DEFAULT_THRESHOLDS_SEC["robot_status"],
            ),
            "joint_states_primary": SourceMetric(
                name="joint_states_primary",
                topic=args.joint_primary_topic,
                threshold_sec=DEFAULT_THRESHOLDS_SEC["joint_states_primary"],
            ),
            "joint_states_fallback": SourceMetric(
                name="joint_states_fallback",
                topic=args.joint_fallback_topic,
                threshold_sec=DEFAULT_THRESHOLDS_SEC["joint_states_fallback"],
            ),
        }

        self._readiness_counts: Counter[str] = Counter()
        self._last_readiness_message: str | None = None
        self._alert_level_counts: Counter[str] = Counter()
        self._alert_reason_counts: Counter[str] = Counter()
        self._last_alert_message: str | None = None
        self._robot_mode_counts: Counter[str] = Counter()
        self._robot_estop_counts: Counter[str] = Counter()
        self._robot_drives_counts: Counter[str] = Counter()
        self._robot_motion_possible_counts: Counter[str] = Counter()
        self._robot_in_motion_counts: Counter[str] = Counter()
        self._robot_in_error_counts: Counter[str] = Counter()
        self._robot_error_code_samples: list[list[int]] = []
        self._joint_source_transitions: list[dict[str, Any]] = []
        self._active_joint_source: str | None = None

        self.create_subscription(String, args.gateway_status_topic, self._on_gateway_status, 10)
        self.create_subscription(RobotReadiness, args.readiness_topic, self._on_readiness, 10)
        self.create_subscription(DiagnosticStatus, args.supervisor_alert_topic, self._on_supervisor_alert, 10)
        self.create_subscription(RobotStatus, args.robot_status_topic, self._on_robot_status, 10)
        self.create_subscription(JointState, args.joint_primary_topic, self._on_joint_primary, 10)
        self.create_subscription(JointState, args.joint_fallback_topic, self._on_joint_fallback, 10)
        self.create_timer(0.1, self._on_timer)

    @property
    def done(self) -> bool:
        return self._done

    def build_report(self) -> dict[str, Any]:
        return {
            "reportGeneratedAt": utcnow_iso(),
            "captureStartedAt": self._started_at_utc,
            "durationSec": self._duration_sec,
            "sources": {
                name: metric.to_dict()
                for name, metric in self._metrics.items()
            },
            "jointSourcePrecedence": {
                "expectedHardwarePreferredSource": "joint_states_primary",
                "expectedFallbackSource": "joint_states_fallback",
                "activeSourceAtEnd": self._active_joint_source,
                "transitions": self._joint_source_transitions,
            },
            "semantics": {
                "readiness": {
                    "readyCounts": dict(self._readiness_counts),
                    "lastStatusMessage": self._last_readiness_message,
                },
                "supervisorAlerts": {
                    "levelCounts": dict(self._alert_level_counts),
                    "reasonCounts": dict(self._alert_reason_counts),
                    "lastMessage": self._last_alert_message,
                },
                "robotStatus": {
                    "modeCounts": dict(self._robot_mode_counts),
                    "eStoppedCounts": dict(self._robot_estop_counts),
                    "drivesPoweredCounts": dict(self._robot_drives_counts),
                    "motionPossibleCounts": dict(self._robot_motion_possible_counts),
                    "inMotionCounts": dict(self._robot_in_motion_counts),
                    "inErrorCounts": dict(self._robot_in_error_counts),
                    "errorCodeSamples": self._robot_error_code_samples[:10],
                },
            },
            "phaseAStatus": {
                "pass": False,
                "reason": (
                    "Read-only capture completed. Human review is still required to prove "
                    "real MotoROS2 timing, precedence, and safety semantics before any "
                    "hardware execution gate may be enabled."
                ),
            },
        }

    def _touch_source(self, source_name: str) -> None:
        self._metrics[source_name].record_message(time.monotonic())

    def _on_gateway_status(self, msg: String) -> None:
        _ = msg
        self._touch_source("gateway_status")

    def _on_readiness(self, msg: RobotReadiness) -> None:
        self._touch_source("readiness")
        self._readiness_counts["ready" if bool(msg.ready) else "not_ready"] += 1
        self._last_readiness_message = str(msg.status_message)

    def _on_supervisor_alert(self, msg: DiagnosticStatus) -> None:
        self._touch_source("supervisor_alerts")
        level = numeric_ros_value(msg.level)
        level_name = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARN",
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE",
        }.get(level, str(level))
        self._alert_level_counts[level_name] += 1
        values = {str(item.key): str(item.value) for item in getattr(msg, "values", [])}
        if values.get("reason"):
            self._alert_reason_counts[values["reason"]] += 1
        self._last_alert_message = str(msg.message)

    def _on_robot_status(self, msg: RobotStatus) -> None:
        self._touch_source("robot_status")
        self._robot_mode_counts[robot_mode_name(numeric_ros_value(msg.mode.val))] += 1
        self._robot_estop_counts[tri_state_name(numeric_ros_value(msg.e_stopped.val))] += 1
        self._robot_drives_counts[tri_state_name(numeric_ros_value(msg.drives_powered.val))] += 1
        self._robot_motion_possible_counts[tri_state_name(numeric_ros_value(msg.motion_possible.val))] += 1
        self._robot_in_motion_counts[tri_state_name(numeric_ros_value(msg.in_motion.val))] += 1
        self._robot_in_error_counts[tri_state_name(numeric_ros_value(msg.in_error.val))] += 1
        if msg.error_codes:
            self._robot_error_code_samples.append([int(code) for code in msg.error_codes])

    def _on_joint_primary(self, msg: JointState) -> None:
        _ = msg
        self._touch_source("joint_states_primary")

    def _on_joint_fallback(self, msg: JointState) -> None:
        _ = msg
        self._touch_source("joint_states_fallback")

    def _on_timer(self) -> None:
        now_monotonic = time.monotonic()
        for metric in self._metrics.values():
            metric.refresh_freshness(now_monotonic)

        next_active_joint_source: str | None = None
        if self._metrics["joint_states_primary"].freshness_state == "fresh":
            next_active_joint_source = "joint_states_primary"
        elif self._metrics["joint_states_fallback"].freshness_state == "fresh":
            next_active_joint_source = "joint_states_fallback"

        if next_active_joint_source != self._active_joint_source:
            self._joint_source_transitions.append(
                {
                    "timestamp": utcnow_iso(),
                    "from": self._active_joint_source,
                    "to": next_active_joint_source,
                }
            )
            self._active_joint_source = next_active_joint_source

        if now_monotonic >= self._deadline_monotonic:
            self._done = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only hardware telemetry validation capture for GP4 HMI Phase A."
    )
    parser.add_argument("--duration-sec", type=float, default=120.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--gateway-status-topic", default="/gateway_status")
    parser.add_argument("--readiness-topic", default="/hw_adapter/ready")
    parser.add_argument("--supervisor-alert-topic", default="/supervisor/alerts")
    parser.add_argument("--robot-status-topic", default="/yaskawa/robot_status")
    parser.add_argument("--joint-primary-topic", default="/yaskawa/joint_states")
    parser.add_argument("--joint-fallback-topic", default="/joint_states")
    return parser


def run_capture(args: argparse.Namespace) -> int:
    rclpy.init()
    node = HardwareTelemetryValidationNode(args)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        report = node.build_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    payload = json.dumps(report, indent=2, sort_keys=True)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return run_capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
