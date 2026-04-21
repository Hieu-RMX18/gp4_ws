# DEPRECATED (2026-04-21): This launch file is a thin wrapper around sim.launch.py.
# Use sim.launch.py directly. Will be removed in a future cleanup.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    audit_log_path_arg = DeclareLaunchArgument(
        "audit_log_path",
        default_value="/tmp/gp4_audit",
        description="Directory used by supervisor audit_logger for rosbag2 and JSONL output.",
    )

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

    return LaunchDescription(
        [
            audit_log_path_arg,
            sim_stack,
        ]
    )
