#!/usr/bin/env python3
"""GP4 Runtime Console — standalone terminal logger for thesis/demo.

Subscribes to ROS2 topics and prints a live feed of:
  - Robot pose (xyz + rpy), joint positions
  - LLM pipeline stage, intent, safety checks
  - Motion execution progress
  - Errors with hints

Usage (in a separate terminal):
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=0
  cd ~/gp4_ws && python3 tools/runtime_console.py [--mode both|snapshot|timeline]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any

# ROS2 imports (fail gracefully if env not sourced)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from industrial_msgs.msg import RobotStatus
    from diagnostic_msgs.msg import DiagnosticStatus
    from action_msgs.msg import GoalStatusArray
except Exception as exc:
    print(f"[FATAL] ROS2 environment not sourced: {exc}")
    print("Hint: source /opt/ros/humble/setup.bash")
    sys.exit(1)

# ANSI colours for terminal output
_C = {
    "reset": "\033[0m",
    "bold": "\033[1m",
    "dim": "\033[2m",
    "red": "\033[31m",
    "green": "\033[32m",
    "yellow": "\033[33m",
    "blue": "\033[34m",
    "magenta": "\033[35m",
    "cyan": "\033[36m",
}


def _fmt(key: str, value: Any, colour: str = "cyan") -> str:
    return f"{_C['bold']}{key}:{_C['reset']} {_C[colour]}{value}{_C['reset']}"


def _quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Convert quaternion to roll/pitch/yaw (ZYX convention)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RuntimeConsole(Node):
    def __init__(self, mode: str = "both") -> None:
        super().__init__("runtime_console")
        self._mode = mode
        self._joints: dict[str, float] = {}
        self._pose: PoseStamped | None = None
        self._robot_mode = "UNKNOWN"
        self._last_llm_stage = ""
        self._last_cmd_stage = ""
        self._error_active = False
        self._last_print_time = 0.0

        self._event_timeline: list[dict] = []
        self._active_command_id: str = ""
        self._active_command_start_time = 0.0
        self._command_events = 0

        self.create_subscription(JointState, "/yaskawa/joint_states", self._joints_callback, qos_profile_sensor_data)
        self.create_subscription(RobotStatus, "/yaskawa/robot_status", self._status_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/yaskawa/current_pose", self._pose_callback, qos_profile_sensor_data)
        self.create_subscription(String, "/gateway_status", self._llm_status_callback, 10)
        self.create_subscription(String, "/llm_debug", self._llm_debug_callback, 10)

        # New subscriptions for execution timeline
        self.create_subscription(DiagnosticStatus, "/supervisor/alerts", self._alerts_callback, 10)
        self.create_subscription(GoalStatusArray, "/execute_motion/_action/status", self._goal_status_callback, 10)

        if self._mode in ("snapshot", "both"):
            self._timer = self.create_timer(2.0, self._print_snapshot)

        self._print_banner()

    def _print_banner(self) -> None:
        print()
        print("=" * 70)
        print(f"{_C['bold']}{_C['blue']}  GP4 Runtime Console (Mode: {self._mode}){_C['reset']}")
        print(f"  ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID', '0')}")
        print("  Waiting for telemetry...")
        print("=" * 70)
        print()

    def _joints_callback(self, msg: JointState) -> None:
        self._joints = {name: math.degrees(float(pos)) for name, pos in zip(msg.name, msg.position)}

    def _status_callback(self, msg: RobotStatus) -> None:
        if msg.e_stopped == 1 or msg.in_error == 1:
            self._robot_mode = "FAULT/ESTOP"
            self._error_active = True
        elif msg.drives_powered == 1 and msg.motion_possible == 1:
            self._robot_mode = "READY"
            self._error_active = False
        elif msg.drives_powered == 1:
            self._robot_mode = "IDLE"
        else:
            self._robot_mode = "OFF"

    def _pose_callback(self, msg: PoseStamped) -> None:
        self._pose = msg

    def _llm_status_callback(self, msg: String) -> None:
        self._last_llm_stage = msg.data
        if self._mode in ("snapshot", "both"):
            self._print_snapshot(force=True)

    def _llm_debug_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if data.get("t") == "command_trace":
                self._append_event(data)
                return

            stage = data.get("stage", "debug")
            status = data.get("status", "")
            entry = f"[{stage}] {status}"
            if "error" in data:
                entry += f" | ERROR: {data['error']}"
            self._last_cmd_stage = entry
        except Exception:
            self._last_cmd_stage = msg.data

        if self._mode in ("snapshot", "both"):
            self._print_snapshot(force=True)

    def _alerts_callback(self, msg: DiagnosticStatus):
        if "heartbeat" in msg.message.lower() or msg.message.lower() == "idle":
            return

        level_val = msg.level[0] if isinstance(msg.level, bytes) else msg.level
        level = "ERROR" if level_val >= 2 else "WARN" if level_val == 1 else "INFO"
        event = {
            "t": "command_trace",
            "ts": time.time(),
            "cmd_id": self._active_command_id,
            "layer": "supervisor",
            "phase": "execution_monitor",
            "event": msg.message,
            "level": level,
            "summary": msg.message,
            "error_why": "",
            "error_where": "",
            "error_next_action": ""
        }

        for kv in msg.values:
            if kv.key == "reason":
                event["error_why"] = kv.value
            elif kv.key == "active_goal_id" and kv.value:
                event["cmd_id"] = kv.value
                self._active_command_id = kv.value

        if level in ("ERROR", "WARN"):
            event["error_where"] = "supervisor/alerts"

        self._append_event(event)

    def _goal_status_callback(self, msg: GoalStatusArray):
        for status in msg.status_list:
            # 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            goal_id_hex = "".join([f"{b:02x}" for b in status.goal_info.goal_id.uuid])

            if status.status == 4:
                self._append_event({
                    "t": "command_trace",
                    "ts": time.time(),
                    "cmd_id": goal_id_hex,
                    "layer": "motion_core",
                    "phase": "execution",
                    "event": "COMPLETED",
                    "level": "INFO",
                    "summary": "Motion goal completed successfully."
                })
                self._print_summary(goal_id_hex, success=True)
            elif status.status in (5, 6):
                ev_type = "CANCELED" if status.status == 5 else "ABORTED"
                self._append_event({
                    "t": "command_trace",
                    "ts": time.time(),
                    "cmd_id": goal_id_hex,
                    "layer": "motion_core",
                    "phase": "execution",
                    "event": ev_type,
                    "level": "ERROR",
                    "summary": f"Motion goal {ev_type.lower()}.",
                    "error_why": "Execution failed or was stopped.",
                    "error_where": "motion_core"
                })
                self._print_summary(goal_id_hex, success=False)

    def _append_event(self, event: dict):
        if event.get("cmd_id") and event.get("cmd_id") != self._active_command_id:
            self._active_command_id = event["cmd_id"]
            self._active_command_start_time = event.get("ts", time.time())
            self._command_events = 0

        self._event_timeline.append(event)
        self._command_events += 1
        if len(self._event_timeline) > 200:
            self._event_timeline.pop(0)

        if self._mode in ("timeline", "both"):
            self._print_event_line(event)

    def _print_event_line(self, event: dict):
        ts = time.strftime("%H:%M:%S", time.localtime(event.get("ts", time.time())))
        level = event.get("level", "INFO")

        icon = "🟢" if level == "INFO" else "🟡" if level == "WARN" else "🔴"
        if event.get("event") in ("llm_request_started", "llm_response_received", "DISPATCH_SENT", "EXECUTING", "validate_command_called"):
            icon = "🔵"

        layer = str(event.get("layer", "")).ljust(15)
        cmd_id = str(event.get("cmd_id", ""))
        if not cmd_id:
            cmd_id = "unknown"
        elif len(cmd_id) > 8:
            cmd_id = cmd_id[:8]
        cmd_id = cmd_id.ljust(8)

        ev_name = str(event.get("event", "")).ljust(22)
        summary = str(event.get("summary", ""))

        print(f"[{ts}] {icon} {ev_name} │ {layer} │ {cmd_id} │ {summary}")

        if level in ("WARN", "ERROR") and (event.get("error_why") or event.get("error_where")):
            print(f"           │ {_C['bold']}WHY:{_C['reset']}    {event.get('error_why', '')}")
            if event.get("error_where"):
                print(f"           │ {_C['bold']}WHERE:{_C['reset']}  {event.get('error_where', '')}")
            if event.get("error_next_action"):
                print(f"           │ {_C['bold']}ACTION:{_C['reset']} {event.get('error_next_action', '')}")

    def _print_summary(self, cmd_id: str, success: bool):
        duration = time.time() - self._active_command_start_time
        res = "✓ SUCCESS" if success else "✗ REJECTED/FAILED"
        res_color = _C['green'] if success else _C['red']

        print(f"{_C['bold']}{'─' * 75}{_C['reset']}")
        print(f" 📊 {cmd_id[:8]} │ {self._command_events} events │ {duration:.1f}s total │ {res_color}{res}{_C['reset']}")
        print(f"{_C['bold']}{'─' * 75}{_C['reset']}\n")

    def _print_snapshot(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_print_time < 2.0:
            return
        self._last_print_time = now

        # Skip snapshot clearing if we are heavily into timeline mode, to prevent destroying the timeline history.
        # But if mode is snapshot only, clear it.
        if self._mode == "snapshot":
            print("\n" * 2)
            print(f"{_C['bold']}{'─' * 70}{_C['reset']}")
            ts = time.strftime("%H:%M:%S")
            print(f"{_C['bold']}  [{ts}] GP4 Runtime Snapshot{_C['reset']}")
            print(f"{_C['bold']}{'─' * 70}{_C['reset']}")

            mode_colour = "green" if self._robot_mode == "READY" else "red" if self._error_active else "yellow"
            print(f"  {_fmt('MODE', self._robot_mode, mode_colour)}")

            if self._pose is not None:
                p = self._pose.pose.position
                o = self._pose.pose.orientation
                r, py, y = _quat_to_rpy(o.x, o.y, o.z, o.w)
                pose_str = (
                    f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} | "
                    f"roll={math.degrees(r):.1f}° pitch={math.degrees(py):.1f}° yaw={math.degrees(y):.1f}°"
                )
                print(f"  {_fmt('POSE', pose_str, 'cyan')}")
            else:
                print(f"  {_fmt('POSE', 'waiting for /yaskawa/current_pose ...', 'dim')}")

            if self._joints:
                joints_str = " | ".join(f"{k}={v:.1f}°" for k, v in sorted(self._joints.items()))
                print(f"  {_fmt('JOINTS', joints_str, 'magenta')}")
            else:
                print(f"  {_fmt('JOINTS', 'waiting for /yaskawa/joint_states ...', 'dim')}")

            if self._last_llm_stage:
                print(f"  {_fmt('LLM', self._last_llm_stage, 'yellow')}")
            if self._last_cmd_stage:
                print(f"  {_fmt('HMI', self._last_cmd_stage, 'blue')}")

            if self._error_active:
                print(f"  {_C['red']}  HINT: Robot is in FAULT/ESTOP. Run alarm_reset or check controller.{_C['reset']}")
            elif not self._joints:
                print(f"  {_C['dim']}  HINT: No joint states — is MotoROS2 running? Check /yaskawa/joint_states.{_C['reset']}")
            elif self._pose is None:
                print(f"  {_C['dim']}  HINT: No pose — is motion_core_node running? Check /yaskawa/current_pose.{_C['reset']}")

            print(f"{_C['bold']}{'─' * 70}{_C['reset']}")


def main() -> None:
    parser = argparse.ArgumentParser(description="GP4 Runtime Console")
    parser.add_argument("--mode", choices=["both", "snapshot", "timeline"], default="both",
                        help="Console display mode (both, snapshot, timeline)")
    args = parser.parse_args()

    rclpy.init()
    node = RuntimeConsole(mode=args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
