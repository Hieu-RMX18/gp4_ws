"""Launch RealSense D435i with GP4 workspace depth profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("depth_profile", default_value="848x480x30"),
            DeclareLaunchArgument("align_depth", default_value="true"),
            DeclareLaunchArgument("enable_sync", default_value="true"),
            DeclareLaunchArgument("pointcloud", default_value="true"),
            DeclareLaunchArgument("emitter_enabled", default_value="true"),
            DeclareLaunchArgument("serial", default_value="<RUNTIME>"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("realsense2_camera"),
                        "/launch",
                        "/rs_launch.py",
                    ]
                ),
                launch_arguments={
                    "depth_module.profile": LaunchConfiguration("depth_profile"),
                    "align_depth.enable": LaunchConfiguration("align_depth"),
                    "enable_sync": LaunchConfiguration("enable_sync"),
                    "pointcloud.enable": LaunchConfiguration("pointcloud"),
                    "emitter_enabled": LaunchConfiguration("emitter_enabled"),
                    "serial_no": LaunchConfiguration("serial"),
                }.items(),
            ),
        ]
    )
