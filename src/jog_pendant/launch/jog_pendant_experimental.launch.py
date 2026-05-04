# Copyright 2026 hieu2
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
jog_pendant_experimental.launch.py — Experimental jog pendant launch.

WARNING: This is an EXPERIMENTAL feature branch.
This launch does NOT replace the mainline FJT execution path.

Requires: MotoROS2 hardware bringup must be running.
Does NOT start: hw_adapter, motion_core, safety, llm_gateway.

Architecture:
  HMI Web Pendant
       ↓ /web_jog_command
  jog_input_node (Python)
       ↓ ~/delta_joint_cmds  → remap → /servo_node/delta_joint_cmds
  MoveIt Servo (joint jog only)
       ↓ /servo_node/command_out
  servo_bridge_node
       ↓ /yaskawa/queue_traj_point
  MotoROS2
       ↓ UDP
  YRC1000micro
       ↓
  GP4

Safety notes:
  - jog_input_node has 200ms dead-man watchdog
  - servo_bridge_node validates joint limits before forwarding
  - No hard stop service for point-queue mode
  - Bridge deactivation = soft stop by input withdrawal
  - FJT path remains blocked while bridge is active (MotoROS2 enforces this)
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ── Warnings ─────────────────────────────────────────────────────────────
    print("=" * 70)
    print("WARNING: jog_pendant_experimental.launch.py")
    print("  This is an EXPERIMENTAL feature branch.")
    print("  DO NOT use this launch for production.")
    print("  This does NOT start hw_adapter, motion_core, safety, or llm_gateway.")
    print("  Joint jogging only — no Cartesian motion.")
    print("  No hard stop service for point-queue mode.")
    print("=" * 70)

    # ── Shared paths ──────────────────────────────────────────────────────
    gp4_moveit_share = get_package_share_directory("gp4_moveit_config")
    jog_pendant_share = get_package_share_directory("jog_pendant")

    # ── Load MoveIt configs for the robot ───────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("motoman_gp4", package_name="gp4_moveit_config")
        .robot_description(
            mappings={
                "use_fake_hardware": "false",
            }
        )
        .to_moveit_configs()
    )

    # ── Servo config for GP4 joint jogging ───────────────────────────────
    # This is the experimental Servo config created in Phase 3.
    # joint_topic is set to /joint_states because the experimental launch
    # does not include the move_group launch (which owns the planning scene).
    # The servo node will use /joint_states directly from the controller.
    servo_config_path = os.path.join(gp4_moveit_share, "config", "servo_gp4_jog.yaml")
    # MoveIt Servo reads params from "moveit_servo" namespace (see servo_parameters.h).
    # Must wrap YAML contents under {"moveit_servo": ...} like the official launch.
    with open(servo_config_path, "r") as f:
        servo_yaml = yaml.safe_load(f)
    # Strip the /**/ros__parameters wrapper if present — the dict params handle that.
    if "/**" in servo_yaml and "ros__parameters" in servo_yaml["/**"]:
        servo_yaml = servo_yaml["/**"]["ros__parameters"]
    servo_params = {"moveit_servo": servo_yaml}

    # ── Shared params for jog_pendant ───────────────────────────────────
    jog_params_path = os.path.join(
        jog_pendant_share, "config", "jog_pendant_params.yaml"
    )

    # ── Joint state topic ────────────────────────────────────────────────
    # Use /joint_states because MotoROS2 remaps /yaskawa/joint_states → /joint_states
    # in hw.launch.py (via joint_state_publisher source_list).
    joint_states_topic = LaunchConfiguration(
        "joint_states_topic", default="/joint_states"
    )

    # ════════════════════════════════════════════════════════════════════════
    # Nodes
    # ════════════════════════════════════════════════════════════════════════

    # ── MoveIt Servo ────────────────────────────────────────────────────
    # Namespace: /servo_node
    # Input: /servo_node/delta_joint_cmds  (from jog_input_node, remapped)
    # Output: /servo_node/command_out  (to servo_bridge_node)
    # Status: /servo_node/status  (from Servo, to servo_bridge_node)
    #
    # NOTE: is_primary_planning_scene_monitor=false because we don't launch
    # move_group in this experimental compose. Collision checking is enabled
    # but may be degraded without a full planning scene.
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        namespace="/servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "moveit_servo": {
                    # Override joint_topic to use /joint_states directly
                    # (no move_group remapping in this experimental compose)
                    "joint_topic": joint_states_topic,
                    # Must override — MoveIt Servo defaults to panda_arm
                    "move_group_name": "gp4_arm",
                    # Enable collision checking for safety
                    "check_collisions": True,
                },
            },
        ],
        # Remap: jog_input_node publishes to /servo_node/delta_joint_cmds
        # without needing namespace on jog_input_node itself.
        # This is the standard pattern for connecting two nodes via remap.
        remappings=[
            # name=servo_node + ns=/servo_node → fully qualified /servo_node/servo_node.
            # ~/X resolves to /servo_node/servo_node/X, but downstream nodes
            # expect /servo_node/X. Remap using absolute source names.
            ("/servo_node/servo_node/delta_joint_cmds", "/servo_node/delta_joint_cmds"),
            ("/servo_node/servo_node/status", "/servo_node/status"),
            ("/servo_node/servo_node/command_out", "/servo_node/command_out"),
        ],
    )

    # ── jog_input_node (Python) ─────────────────────────────────────────
    # This node translates HMI web jog commands into MoveIt Servo JointJog.
    # Runs at ROOT namespace (no remapping needed for web_jog_command).
    # Publishes to 'delta_joint_cmds' (root ns) → remapped to /servo_node/delta_joint_cmds
    # Watchdog: 200ms no heartbeat → halt.
    jog_input_node = Node(
        package="jog_pendant",
        executable="jog_input_node.py",
        name="jog_input_node",
        output="screen",
        parameters=[
            {
                # jog_input_node publishes to this topic.
                # Remapped by servo_node to /servo_node/delta_joint_cmds.
                "servo_cmd_topic": "delta_joint_cmds",
            }
        ],
    )

    # ── servo_bridge_node (C++) ─────────────────────────────────────────
    # Subscribes to:
    #   /servo_node/command_out  — Servo trajectory output
    #   /servo_node/status       — Servo status (std_msgs/Int8)
    #   /yaskawa/robot_status   — Robot ready check
    #   /joint_states           — Joint position cache
    # Calls:
    #   /yaskawa/start_point_queue_mode
    #   /yaskawa/queue_traj_point
    # Publishes:
    #   /servo_bridge/status
    # Services:
    #   /servo_bridge/activate
    #   /servo_bridge/deactivate
    servo_bridge_node = Node(
        package="jog_pendant",
        executable="servo_bridge_node",
        name="servo_bridge_node",
        output="screen",
        parameters=[
            jog_params_path,
            {
                "joint_states_topic": joint_states_topic,
                # Servo trajectory topic (namespaced by servo_node)
                "servo_trajectory_topic": "/servo_node/command_out",
                # Servo status topic
                "servo_status_topic": "/servo_node/status",
            },
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            LogInfo(msg="[EXPERIMENTAL] jog_pendant_experimental.launch.py loaded"),
            LogInfo(
                msg="[EXPERIMENTAL] WARNING: This is NOT the mainline execution path"
            ),
            LogInfo(msg="[EXPERIMENTAL] DO NOT use this for production"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description="Joint states topic for Servo (usually /joint_states in hw bringup)",
            ),
            servo_node,
            jog_input_node,
            servo_bridge_node,
        ]
    )
