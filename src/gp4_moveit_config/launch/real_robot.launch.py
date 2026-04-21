# DEPRECATED (2026-04-21): Superseded by gp4_bringup/launch/hw.launch.py.
# Will be removed in a future cleanup once external users migrate.
"""
real_robot.launch.py
────────────────────
Khởi động controller_manager + MotoROS2 hardware interface
cho Yaskawa GP4 / YRC1000micro.

Chạy TRƯỚC khi bật RViz:
  ros2 launch gp4_moveit_config real_robot.launch.py

Sau đó bật RViz riêng:
  ros2 launch gp4_moveit_config demo.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ── Tham số có thể override từ command line ──────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.33",
        description="IP address của YRC1000micro"
    )
    use_fake_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="true = mock/simulation, false = robot thật"
    )

    robot_ip        = LaunchConfiguration("robot_ip")
    use_fake        = LaunchConfiguration("use_fake_hardware")

    # ── Load MoveIt config với tham số hardware ───────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("motoman_gp4", package_name="gp4_moveit_config")
        .robot_description(
            mappings={
                "use_fake_hardware": use_fake,
                "robot_ip":          robot_ip,
            }
        )
        .to_moveit_configs()
    )

    # ── Node 1: ros2_control_node (controller_manager) ───────────────────
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[
            ('follow_joint_trajectory', '/yaskawa/follow_joint_trajectory'),
        ],
        output="screen",
    )

    # ── Node 2: robot_state_publisher ────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # ── Node 3 & 4: Spawn controllers (chờ controller_manager sẵn sàng) ──
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gp4_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn controllers sau 2 giây để controller_manager kịp khởi động
    delayed_spawners = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner, arm_controller_spawner],
    )

    return LaunchDescription([
        robot_ip_arg,
        use_fake_arg,
        rsp_node,
        control_node,
        delayed_spawners,
    ])
