"""Launch nodes needed for calibration data collection.

Starts:
  1. RealSense D435i camera node (depth + color, no point cloud)
  2. CalibrationService node (ArUco detection + sample buffering)

Usage:
  ros2 launch gp4_perception calibration_collect.launch.py
  # Then jog robot through 12-24 diverse poses with ArUco board visible.
  # Finally call:
  #   ros2 service call /perception/calibrate_hand_eye \
  #     interfaces/srv/CalibrateHandEye "{fiducial_id: 'board_5x7', min_samples: 12}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial",
                default_value="",
                description="Camera serial number (empty = auto-discover)",
            ),
            DeclareLaunchArgument(
                "depth_profile",
                default_value="848x480x30",
                description="Depth stream resolution for calibration",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="1280x720x30",
                description="Color stream resolution for ArUco detection",
            ),
            # Camera via rs_launch.py — same namespace strategy as camera.launch.py
            # so topics appear at /camera/color/image_raw (not /camera/camera/...)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("realsense2_camera"),
                        "/launch",
                        "/rs_launch.py",
                    ]
                ),
                launch_arguments={
                    "camera_namespace": "",
                    "camera_name": "camera",
                    "serial_no": LaunchConfiguration("serial"),
                    "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
                    "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    # Point cloud not needed for calibration (saves CPU)
                    "pointcloud.enable": "false",
                    "depth_module.emitter_enabled": "1",
                    # Filters off during calibration — raw depth is fine
                    "spatial_filter.enable": "false",
                    "temporal_filter.enable": "false",
                    # QoS
                    "depth_qos": "SENSOR_DATA",
                    "color_qos": "SENSOR_DATA",
                }.items(),
            ),
            # Calibration service node
            Node(
                package="gp4_perception",
                executable="calibration_service",
                name="calibration_service",
                output="screen",
            ),
        ]
    )
