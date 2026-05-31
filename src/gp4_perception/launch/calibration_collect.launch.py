"""Launch nodes needed for calibration data collection.

Starts:
  1. RealSense D435i camera node (depth + color, no point cloud)
  2. CalibrationService node (Charuco detection + sample buffering)

Usage:
  ros2 launch gp4_perception calibration_collect.launch.py
  # Then jog robot through 12-24 diverse poses with the Charuco board visible.
  # Finally call:
  #   ros2 service call /perception/calibrate_hand_eye \
  #     interfaces/srv/CalibrateHandEye \
  #     "{fiducial_id: 'charuco_10x11_20mm_15mm', min_samples: 12}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    serial_no = ParameterValue(LaunchConfiguration("serial"), value_type=str)

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
                description="Color stream resolution for Charuco detection",
            ),
            # Camera node — same namespace strategy as camera.launch.py
            # so topics appear at /camera/color/image_raw (not /camera/camera/...)
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace="",
                name="camera",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "camera_namespace": "",
                        "camera_name": "camera",
                        "serial_no": serial_no,
                        "enable_color": True,
                        "enable_depth": True,
                        "enable_infra": False,
                        "enable_infra1": False,
                        "enable_infra2": False,
                        "enable_gyro": False,
                        "enable_accel": False,
                        "depth_module.depth_profile": LaunchConfiguration(
                            "depth_profile"
                        ),
                        "rgb_camera.color_profile": LaunchConfiguration(
                            "color_profile"
                        ),
                        "align_depth.enable": True,
                        "enable_sync": True,
                        # Point cloud not needed for calibration (saves CPU)
                        "pointcloud.enable": False,
                        # Filters off during calibration - raw depth is fine
                        "spatial_filter.enable": False,
                        "temporal_filter.enable": False,
                        "publish_tf": True,
                        "tf_publish_rate": 0.0,
                    }
                ],
                arguments=["--ros-args", "--log-level", "info"],
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
