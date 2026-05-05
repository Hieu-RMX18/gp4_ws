"""Launch full perception stack: camera + scene processor + TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial", default_value="<RUNTIME>"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("gp4_perception"),
                        "/launch",
                        "/camera.launch.py",
                    ]
                ),
                launch_arguments={"serial": LaunchConfiguration("serial")}.items(),
            ),
            Node(
                package="gp4_perception",
                executable="scene_processor",
                name="scene_processor",
                output="screen",
            ),
            Node(
                package="gp4_perception",
                executable="tf_publisher",
                name="tf_publisher",
                output="screen",
            ),
        ]
    )
