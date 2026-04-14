#!/usr/bin/env python3
# Copyright 2026 hieu2
# SPDX-License-Identifier: Apache-2.0

"""
jog_input_node.py — Experimental jog pendant input translator.

Subscribes to:
  /web_jog_command  (interfaces/msg/JogCommand)

Publishes to:
  ~/delta_joint_cmds  (control_msgs/msg/JointJog) — MoveIt Servo input

Behavior:
  - continuous mode: hold-to-run, velocity commands while heartbeat active
  - discrete mode:   single-step per command
  - watchdog > 200ms without heartbeat → publish halt

This is a READ-ONLY bridge from HMI to Servo. It does NOT send points
directly to MotoROS2.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, Reliability

from control_msgs.msg import JointJog
from interfaces.msg import JogCommand

# GP4 joint names in order (0-based index)
GP4_JOINT_NAMES = [
    'joint_1_s',
    'joint_2_l',
    'joint_3_u',
    'joint_4_r',
    'joint_5_b',
    'joint_5_b',   # typo in name, but following actual codebase
    'joint_6_t',
]

# Actual joint names from ros2_controllers.yaml
GP4_JOINT_NAMES_CORRECT = [
    'joint_1_s',
    'joint_2_l',
    'joint_3_u',
    'joint_4_r',
    'joint_5_b',
    'joint_6_t',
]

# Joint velocity limits from joint_limits.yaml (rad/s)
# Conservative defaults for experimental jog
GP4_JOINT_MAX_VELOCITY_RAD_S = {
    'joint_1_s': 8.116,
    'joint_2_l': 8.116,
    'joint_3_u': 9.163,
    'joint_4_r': 9.861,
    'joint_5_b': 9.861,
    'joint_6_t': 17.453,
}


class JogInputNode(Node):
    """
    Translates web jog commands into MoveIt Servo JointJog messages.

    State machine:
      IDLE → (heartbeat + valid command) → ACTIVE → (no heartbeat 200ms) → HALTING → IDLE
    """

    def __init__(self) -> None:
        super().__init__('jog_input_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('joint_names', GP4_JOINT_NAMES_CORRECT)
        self.declare_parameter('max_velocity_scale', 0.3)
        self.declare_parameter('min_velocity_scale', 0.01)
        self.declare_parameter('max_step_degrees', 10.0)
        self.declare_parameter('min_step_degrees', 0.01)
        self.declare_parameter('watchdog_timeout_ms', 200)
        self.declare_parameter('default_velocity_scale', 0.05)
        self.declare_parameter('servo_cmd_topic', 'delta_joint_cmds')
        self.declare_parameter('joint_max_velocity_rad_s', list(GP4_JOINT_MAX_VELOCITY_RAD_S.values()))

        self._joint_names: list[str] = self.get_parameter('joint_names').value
        self._max_velocity_scale = self.get_parameter('max_velocity_scale').value
        self._min_velocity_scale = self.get_parameter('min_velocity_scale').value
        self._max_step_degrees = self.get_parameter('max_step_degrees').value
        self._min_step_degrees = self.get_parameter('min_step_degrees').value
        self._watchdog_timeout_ms = self.get_parameter('watchdog_timeout_ms').value
        self._default_velocity_scale = self.get_parameter('default_velocity_scale').value
        self._servo_cmd_topic = self.get_parameter('servo_cmd_topic').value
        self._joint_max_velocity: list[float] = self.get_parameter('joint_max_velocity_rad_s').value

        if len(self._joint_max_velocity) != len(self._joint_names):
            self.get_logger().warn(
                f'joint_max_velocity count ({len(self._joint_max_velocity)}) != '
                f'joint_names count ({len(self._joint_names)}); using defaults'
            )
            self._joint_max_velocity = [
                GP4_JOINT_MAX_VELOCITY_RAD_S.get(name, 1.0)
                for name in self._joint_names
            ]

        # ── State ──────────────────────────────────────────────────
        self._state = 'IDLE'
        self._active_joint_index: Optional[int] = None
        self._active_direction: int = 0
        self._active_velocity_scale: float = self._default_velocity_scale
        self._active_mode: str = 'continuous'
        self._last_heartbeat_sec: float = self.get_clock().now().nanoseconds / 1e9

        self._state_lock = threading.Lock()

        # ── Subscriptions ───────────────────────────────────────────
        self._qos = QoSProfile(depth=10, reliability=Reliability.RELIABLE)

        self._jog_sub = self.create_subscription(
            JogCommand,
            'web_jog_command',
            self._on_jog_command,
            self._qos,
        )

        # ── Publishers ─────────────────────────────────────────────
        self._jog_pub = self.create_publisher(
            JointJog,
            self._servo_cmd_topic,
            self._qos,
        )

        # ── Watchdog timer ─────────────────────────────────────────
        watchdog_period_ms = self._watchdog_timeout_ms // 2
        self._watchdog_timer = self.create_timer(
            float(watchdog_timeout_ms := self._watchdog_timeout_ms) / 1000.0 * 0.5,
            self._on_watchdog,
        )

        self.get_logger().info(
            f'jog_input_node started. Servo topic: {self._servo_cmd_topic}, '
            f'joints: {self._joint_names}, watchdog: {self._watchdog_timeout_ms}ms'
        )

    # ── Subscribers ───────────────────────────────────────────────────────

    def _on_jog_command(self, msg: JogCommand) -> None:
        """Process incoming web jog command."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self._last_heartbeat_sec = now_sec

        # ── Parse validation ────────────────────────────────────────
        if msg.joint_index < 0 or msg.joint_index >= len(self._joint_names):
            self.get_logger().warn(
                f'rejected: invalid joint_index={msg.joint_index} '
                f'(valid: 0-{len(self._joint_names) - 1})'
            )
            return

        if msg.direction not in (-1, 1):
            self.get_logger().warn(f'rejected: invalid direction={msg.direction} (must be -1 or +1)')
            return

        # ── Parse mode ──────────────────────────────────────────────
        mode = msg.mode.strip().lower() if msg.mode else 'continuous'
        if mode not in ('continuous', 'discrete'):
            self.get_logger().warn(f'invalid mode "{msg.mode}"; defaulting to continuous')
            mode = 'continuous'

        # ── Parse velocity scale ────────────────────────────────────
        raw_vel_scale = msg.velocity_scale
        if raw_vel_scale <= 0.0:
            raw_vel_scale = self._default_velocity_scale
        velocity_scale = max(
            self._min_velocity_scale,
            min(self._max_velocity_scale, raw_vel_scale),
        )

        # ── Parse step size ─────────────────────────────────────────
        step_degrees = max(
            self._min_step_degrees,
            min(self._max_step_degrees, max(0.0, msg.step_degrees)),
        )

        joint_name = self._joint_names[msg.joint_index]

        if mode == 'discrete':
            self._publish_discrete(joint_name, msg.joint_index, msg.direction, step_degrees, velocity_scale)
        else:
            self._publish_continuous(
                joint_name, msg.joint_index, msg.direction, velocity_scale
            )

    # ── Watchdog ─────────────────────────────────────────────────────────

    def _on_watchdog(self) -> None:
        """Hard watchdog: if no heartbeat for > watchdog_timeout_ms, halt."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed_ms = (now_sec - self._last_heartbeat_sec) * 1000.0

        if elapsed_ms > self._watchdog_timeout_ms:
            with self._state_lock:
                if self._state != 'IDLE':
                    self.get_logger().warn(
                        f'watchdog: no heartbeat for {elapsed_ms:.0f}ms > {self._watchdog_timeout_ms}ms; '
                        'halting motion'
                    )
                    self._halt_locked()
                    self._state = 'IDLE'
                    self._active_joint_index = None
                    self._active_direction = 0

    # ── Publishers ───────────────────────────────────────────────────────

    def _publish_continuous(
        self,
        joint_name: str,
        joint_index: int,
        direction: int,
        velocity_scale: float,
    ) -> None:
        """Publish continuous velocity command."""
        joint_max_vel = self._joint_max_velocity[joint_index]
        velocity_rad_s = direction * joint_max_vel * velocity_scale

        cmd = JointJog()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.joint_names = [joint_name]
        cmd.displacements = []          # use velocities, not displacements
        cmd.velocities = [velocity_rad_s]
        cmd.duration = 0.0            # 0 = indefinite; Servo's incoming_command_timeout is the guard

        self._jog_pub.publish(cmd)

        with self._state_lock:
            self._state = 'ACTIVE'
            self._active_joint_index = joint_index
            self._active_direction = direction
            self._active_velocity_scale = velocity_scale
            self._active_mode = 'continuous'

        self.get_logger().debug(
            f'continuous: {joint_name} dir={direction} vel_scale={velocity_scale:.3f} '
            f'(vel={velocity_rad_s:.4f} rad/s)'
        )

    def _publish_discrete(
        self,
        joint_name: str,
        joint_index: int,
        direction: int,
        step_degrees: float,
        velocity_scale: float,
    ) -> None:
        """
        Publish single-step jog command.

        Uses JointJog displacements + velocities + duration for profiled single move.
        """
        step_rad = math.radians(step_degrees)
        joint_max_vel = self._joint_max_velocity[joint_index]
        # Duration: how long to hold the velocity for a single step
        # duration = step / (max_vel * scale) — time to travel the step at target speed
        duration = step_rad / (joint_max_vel * velocity_scale)
        # Clamp duration to reasonable bounds (0.05s - 5s)
        duration = max(0.05, min(5.0, duration))

        # Velocity for the profiled motion
        velocity_rad_s = direction * joint_max_vel * velocity_scale

        cmd = JointJog()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.joint_names = [joint_name]
        cmd.displacements = [direction * step_rad]
        cmd.velocities = [velocity_rad_s]
        cmd.duration = duration

        self._jog_pub.publish(cmd)

        self.get_logger().info(
            f'discrete: {joint_name} dir={direction} step={step_degrees:.2f}deg '
            f'duration={duration:.2f}s'
        )

        with self._state_lock:
            # After a discrete move, go back to idle and wait for next command.
            # The Servo's incoming_command_timeout will handle stopping.
            self._state = 'IDLE'
            self._active_joint_index = None
            self._active_direction = 0

    def _halt_locked(self) -> None:
        """Publish a halt (zero velocity) command."""
        if self._active_joint_index is None:
            return

        joint_name = self._joint_names[self._active_joint_index]
        cmd = JointJog()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.joint_names = [joint_name]
        cmd.displacements = []
        cmd.velocities = [0.0]
        cmd.duration = 0.0

        self._jog_pub.publish(cmd)
        self.get_logger().info(f'halt published for {joint_name}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JogInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
