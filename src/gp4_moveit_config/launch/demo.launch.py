import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt Config (urdf, srdf, kinematics, etc.)
    moveit_config = MoveItConfigsBuilder("motoman_gp4", package_name="gp4_moveit_config").to_moveit_configs()

    # Create MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    # Create RViz Node that EXPLICITLY receives the robot_description_semantic
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    # ROS 2 Control Node MUST be named "controller_manager" for spawner stability in Humble
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen",
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # GP4 Arm Controller Spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gp4_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Prevent Race Conditions: Spawn JSB after ros2_control starts, then Arm Controller after JSB
    delay_jsb = TimerAction(
        period=1.5,
        actions=[joint_state_broadcaster_spawner]
    )

    delay_arm = TimerAction(
        period=3.0,
        actions=[arm_controller_spawner]
    )

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            delay_jsb,
            delay_arm,
            move_group_node,
            rviz_node,
        ]
    )
