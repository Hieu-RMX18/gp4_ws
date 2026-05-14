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
  cd ~/gp4_ws && python3 tools/runtime_console.py
"""

from __future__ import annotations

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
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RuntimeConsole(Node):
    def __init__(self) -> None:
        super().__init__("runtime_console")
        self._joints: dict[str, float] = {}
        self._pose: PoseStamped | None = None
        self._robot_mode = "UNKNOWN"
        self._last_llm_stage = ""
        self._last_cmd_stage = ""
        self._error_active = False
        self._last_print_time = 0.0

        self.create_subscription(
            JointState,
            "/yaskawa/joint_states",
            self._joints_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RobotStatus,
            "/yaskawa/robot_status",
            self._status_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/yaskawa/current_pose",
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/gateway_status",
            self._llm_status_callback,
            10,
        )
        self.create_subscription(
            String,
            "/llm_debug",
            self._llm_debug_callback,
            10,
        )

        self._timer = self.create_timer(1.0, self._print_state)
        self._print_banner()

    def _print_banner(self) -> None:
        print()
        print("=" * 70)
        print(f"{_C['bold']}{_C['blue']}  GP4 Runtime Console {_C['reset']}")
        print(f"  ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID', '0')}")
        print("  Waiting for telemetry...")
        print("=" * 70)
        print()

    def _joints_callback(self, msg: JointState) -> None:
        self._joints = {
            name: math.degrees(float(pos))
            for name, pos in zip(msg.name, msg.position)
        }

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
        self._print_state(force=True)

    def _llm_debug_callback(self, msg: String) -> None:
        import json
        try:
            data = json.loads(msg.data)
            stage = data.get("stage", "debug")
            status = data.get("status", "")
            entry = f"[{stage}] {status}"
            if "error" in data:
                entry += f" | ERROR: {data['error']}"
            self._last_cmd_stage = entry
        except Exception:
            self._last_cmd_stage = msg.data
        self._print_state(force=True)

    def _print_state(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_print_time < 2.0:
            return
        self._last_print_time = now

        # Clear screen-ish (scroll up)
        print("\n" * 2)
        print(f"{_C['bold']}{'─' * 70}{_C['reset']}")
        ts = time.strftime("%H:%M:%S")
        print(f"{_C['bold']}  [{ts}] GP4 Runtime Snapshot{_C['reset']}")
        print(f"{_C['bold']}{'─' * 70}{_C['reset']}")

        # Robot mode
        mode_colour = "green" if self._robot_mode == "READY" else "red" if self._error_active else "yellow"
        print(f"  {_fmt('MODE', self._robot_mode, mode_colour)}")

        # Pose
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

        # Joints
        if self._joints:
            joints_str = " | ".join(
                f"{k}={v:.1f}°" for k, v in sorted(self._joints.items())
            )
            print(f"  {_fmt('JOINTS', joints_str, 'magenta')}")
        else:
            print(f"  {_fmt('JOINTS', 'waiting for /yaskawa/joint_states ...', 'dim')}")

        # Pipeline stages
        if self._last_llm_stage:
            print(f"  {_fmt('LLM', self._last_llm_stage, 'yellow')}")
        if self._last_cmd_stage:
            print(f"  {_fmt('HMI', self._last_cmd_stage, 'blue')}")

        # Hints
        if self._error_active:
            print(f"  {_C['red']}  HINT: Robot is in FAULT/ESTOP. Run alarm_reset or check controller.{_C['reset']}")
        elif not self._joints:
            print(f"  {_C['dim']}  HINT: No joint states — is MotoROS2 running? Check /yaskawa/joint_states.{_C['reset']}")
        elif self._pose is None:
            print(f"  {_C['dim']}  HINT: No pose — is motion_core_node running? Check /yaskawa/current_pose.{_C['reset']}")

        print(f"{_C['bold']}{'─' * 70}{_C['reset']}")


def main() -> None:
    rclpy.init()
    node = RuntimeConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
