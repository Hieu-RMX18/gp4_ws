import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    llm_provider_arg = DeclareLaunchArgument(
        "llm_provider",
        default_value="9router_local",
        description="Logical LLM provider label exported to the llm_gateway process environment.",
    )
    audit_log_path_arg = DeclareLaunchArgument(
        "audit_log_path",
        default_value="/tmp/gp4_audit",
        description="Directory used by supervisor audit_logger for rosbag2 and JSONL output.",
    )

    llm_provider = LaunchConfiguration("llm_provider")
    audit_log_path = LaunchConfiguration("audit_log_path")
    gp4_bringup_share = get_package_share_directory("gp4_bringup")

    sim_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gp4_bringup_share, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "audit_log_path": audit_log_path,
        }.items(),
    )

    llm_gateway_node = Node(
        package="llm_gateway",
        executable="llm_gateway_node",
        name="llm_gateway",
        output="screen",
        parameters=[
            {
                # Fake/sim Phase 9 uses ValidateCommand as the active safety gate,
                # while motion_core still aborts goals that request approval.
                "auto_clear_unimplemented_approval": True,
            }
        ],
        additional_env={
            "LLM_PROVIDER": llm_provider,
        },
    )

    return LaunchDescription(
        [
            llm_provider_arg,
            audit_log_path_arg,
            sim_stack,
            llm_gateway_node,
        ]
    )
