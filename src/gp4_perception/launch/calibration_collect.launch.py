"""Launch nodes needed for calibration data collection."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial", default_value="<RUNTIME>"),
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="camera",
                namespace="camera",
                parameters=[
                    {
                        "depth_module.profile": "848x480x30",
                        "align_depth.enable": True,
                        "enable_sync": True,
                        "pointcloud.enable": True,
                        "emitter_enabled": True,
                        "serial_no": LaunchConfiguration("serial"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="gp4_perception",
                executable="calibration_service",
                name="calibration_service",
                output="screen",
            ),
        ]
    )
