# DEPRECATED (2026-04-21): Superseded by gp4_bringup/launch/moveit_only.launch.py.
# Will be removed in a future cleanup once external users migrate.
"""
gp4_start.launch.py
────────────────────
Launch file hoàn chỉnh cho Yaskawa GP4 với MotoROS2 driver.

Kiến trúc:
  - MotoROS2 driver chạy trên robot (cung cấp /yaskawa/follow_joint_trajectory
    và /yaskawa/joint_states)
  - File này khởi động: move_group + RViz + robot_state_publisher +
    joint_state_publisher + static TF

Cách dùng:
  1. Đảm bảo MotoROS2 driver đang chạy trên robot
  2. ros2 launch gp4_moveit_config gp4_start.launch.py
  3. Dùng RViz để plan & execute, robot thật sẽ chạy theo
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    # ── MoveIt config ────────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("motoman_gp4", package_name="gp4_moveit_config")
        .robot_description(file_path="config/motoman_gp4.urdf.xacro")
        .robot_description_semantic(file_path="config/motoman_gp4.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # ── move_group node ──────────────────────────────────────────────────
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_monitor_parameters,
        ],
        ros_arguments=['--log-level', 'class_loader.ClassLoader:=FATAL'],
        remappings=[
            ('/joint_states', '/yaskawa/joint_states'),
        ],
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz_config_path = os.path.join(
        get_package_share_directory("gp4_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # ── Robot State Publisher (publish TF từ URDF) ───────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ── Joint State Publisher ────────────────────────────────────────────
    #    Lắng nghe /yaskawa/joint_states từ MotoROS2 driver
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {
                "rate": 43,
                "source_list": ["/yaskawa/joint_states"],
            },
        ],
    )

    # ── Static TF (world -> base_link) ───────────────────────────────────
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0",
                   "0.0", "0.0", "world", "base_link"],
    )

    return LaunchDescription(
        [
            db_arg,
            static_tf,
            robot_state_publisher,
            joint_state_publisher,
            run_move_group_node,
            rviz_node,
        ]
    )
