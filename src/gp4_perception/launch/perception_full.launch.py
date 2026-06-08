"""Launch full perception stack: camera + scene processor + TF + viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial", default_value="943222073917"),
            DeclareLaunchArgument(
                "enable_bbox_filter",
                default_value="true",
                description="Enable workspace bounding-box filter (set false to see all detections)",
            ),
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
                "use_unified_gui",
                default_value="true",
                description="Use the single-window visualizer; false launches legacy viewers",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("gp4_perception"),
                        "/launch",
                        "/camera.launch.py",
                    ]
                ),
                launch_arguments={
                    "serial": LaunchConfiguration("serial"),
                    "depth_profile": LaunchConfiguration("depth_profile"),
                    "color_profile": LaunchConfiguration("color_profile"),
                }.items(),
            ),
            Node(
                package="gp4_perception",
                executable="scene_processor",
                name="scene_processor",
                output="screen",
                parameters=[
                    {"enable_bbox_filter": LaunchConfiguration("enable_bbox_filter")},
                ],
            ),
            Node(
                package="gp4_perception",
                executable="tf_publisher",
                name="tf_publisher",
                output="screen",
            ),
            Node(
                package="gp4_perception",
                executable="unified_visualizer",
                name="unified_visualizer",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_unified_gui")),
            ),
            Node(
                package="gp4_perception",
                executable="detection_visualizer",
                name="detection_visualizer",
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("use_unified_gui")),
            ),
            Node(
                package="gp4_perception",
                executable="preprocessing_visualizer",
                name="preprocessing_visualizer",
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("use_unified_gui")),
            ),
        ]
    )
