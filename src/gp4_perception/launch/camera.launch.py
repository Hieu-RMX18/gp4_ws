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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    serial_no = ParameterValue(LaunchConfiguration("serial"), value_type=str)
    align_depth = ParameterValue(LaunchConfiguration("align_depth"), value_type=bool)
    enable_sync = ParameterValue(LaunchConfiguration("enable_sync"), value_type=bool)
    pointcloud = ParameterValue(LaunchConfiguration("pointcloud"), value_type=bool)
    spatial_filter = ParameterValue(
        LaunchConfiguration("spatial_filter"), value_type=bool
    )
    temporal_filter = ParameterValue(
        LaunchConfiguration("temporal_filter"), value_type=bool
    )
    hole_filling_filter = ParameterValue(
        LaunchConfiguration("hole_filling_filter"), value_type=bool
    )
    pointcloud_stream_filter = ParameterValue(
        LaunchConfiguration("pointcloud_stream_filter"), value_type=int
    )
    pointcloud_stream_index_filter = ParameterValue(
        LaunchConfiguration("pointcloud_stream_index_filter"), value_type=int
    )

    return LaunchDescription(
        [
            # --- Configurable launch arguments ---
            DeclareLaunchArgument(
                "depth_profile",
                default_value="848x480x30",
                description="Depth stream resolution and FPS (W,H,FPS)",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="1280x720x30",
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
                "hole_filling_filter",
                default_value="true",
                description="Enable hole-filling filter (reduces depth dropouts/noise)",
            ),
            DeclareLaunchArgument(
                "pointcloud_stream_filter",
                default_value="2",
                description="RealSense pointcloud texture stream (2=color on realsense-ros Humble)",
            ),
            DeclareLaunchArgument(
                "pointcloud_stream_index_filter",
                default_value="0",
                description="RealSense pointcloud texture stream index",
            ),
            # --- RealSense camera node ---
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace="",
                name="camera",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        # Camera identity
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
                        # Stream profiles (ros2-master format)
                        "depth_module.depth_profile": LaunchConfiguration(
                            "depth_profile"
                        ),
                        "rgb_camera.color_profile": LaunchConfiguration(
                            "color_profile"
                        ),
                        # Alignment and sync
                        "align_depth.enable": align_depth,
                        "enable_sync": enable_sync,
                        # Point cloud. stream_filter=2 selects the color stream
                        # as texture source for packed RGB in PointCloud2.
                        "pointcloud.enable": pointcloud,
                        "pointcloud.stream_filter": pointcloud_stream_filter,
                        "pointcloud.stream_index_filter": pointcloud_stream_index_filter,
                        # Post-processing filters
                        "spatial_filter.enable": spatial_filter,
                        "temporal_filter.enable": temporal_filter,
                        "hole_filling_filter.enable": hole_filling_filter,
                        # TF - publish internal camera frame TFs as static
                        "publish_tf": True,
                        "tf_publish_rate": 0.0,
                    }
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
