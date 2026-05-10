"""Launch RealSense D435i with GP4 workspace depth profile.

Uses realsense-ros ros2-master parameter conventions:
  - depth_module.depth_profile (replaces old depth_width/height/fps)
  - rgb_camera.color_profile
  - align_depth.enable (replaces align_depth)
  - pointcloud.enable (replaces pointcloud_enable)
  - spatial_filter.enable, temporal_filter.enable (replaces filters string)

Reference: https://github.com/realsenseai/realsense-ros (ros2-master branch)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # --- Configurable launch arguments ---
            DeclareLaunchArgument(
                "depth_profile",
                default_value="848,480,30",
                description="Depth stream resolution and FPS (W,H,FPS)",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="1280,720,30",
                description="Color stream resolution and FPS (W,H,FPS)",
            ),
            DeclareLaunchArgument(
                "align_depth",
                default_value="true",
                description="Align depth to color frame",
            ),
            DeclareLaunchArgument(
                "enable_sync",
                default_value="true",
                description="Synchronize depth and color frames",
            ),
            DeclareLaunchArgument(
                "pointcloud",
                default_value="true",
                description="Enable point cloud generation",
            ),
            DeclareLaunchArgument(
                "emitter_enabled",
                default_value="1",
                description="IR emitter: 1=on, 0=off (disable if IR cross-talk)",
            ),
            DeclareLaunchArgument(
                "serial",
                default_value="",
                description="Camera serial number (empty = auto-discover)",
            ),
            DeclareLaunchArgument(
                "spatial_filter",
                default_value="true",
                description="Enable spatial edge-preserving filter",
            ),
            DeclareLaunchArgument(
                "temporal_filter",
                default_value="true",
                description="Enable temporal consistency filter",
            ),
            DeclareLaunchArgument(
                "depth_qos",
                default_value="SENSOR_DATA",
                description="QoS for depth topics (SENSOR_DATA = BEST_EFFORT)",
            ),
            DeclareLaunchArgument(
                "color_qos",
                default_value="SENSOR_DATA",
                description="QoS for color topics (SENSOR_DATA = BEST_EFFORT)",
            ),
            # --- Include upstream RealSense launch ---
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("realsense2_camera"),
                        "/launch",
                        "/rs_launch.py",
                    ]
                ),
                launch_arguments={
                    # Camera identity
                    "camera_namespace": "",
                    "camera_name": "camera",
                    "serial_no": LaunchConfiguration("serial"),
                    # Stream profiles (ros2-master format)
                    "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
                    "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                    # Alignment and sync
                    "align_depth.enable": LaunchConfiguration("align_depth"),
                    "enable_sync": LaunchConfiguration("enable_sync"),
                    # Point cloud
                    "pointcloud.enable": LaunchConfiguration("pointcloud"),
                    # IR emitter
                    "depth_module.emitter_enabled": LaunchConfiguration(
                        "emitter_enabled"
                    ),
                    # Post-processing filters
                    "spatial_filter.enable": LaunchConfiguration("spatial_filter"),
                    "temporal_filter.enable": LaunchConfiguration("temporal_filter"),
                    # QoS — must match subscriber side (BEST_EFFORT / SENSOR_DATA)
                    "depth_qos": LaunchConfiguration("depth_qos"),
                    "color_qos": LaunchConfiguration("color_qos"),
                    # TF — publish internal camera frame TFs as static
                    "publish_tf": "true",
                    "tf_publish_rate": "0.0",
                }.items(),
            ),
        ]
    )
