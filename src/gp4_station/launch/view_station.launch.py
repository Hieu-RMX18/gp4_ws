from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    station_share = Path(get_package_share_directory("gp4_station"))
    xacro_path = station_share / "urdf" / "gp4_on_station.urdf.xacro"
    rviz_config = station_share / "rviz" / "station.rviz"

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(
                                [FindExecutable(name="xacro"), " ", str(xacro_path)]
                            ),
                            value_type=str,
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(rviz_config)],
                output="screen",
            ),
        ]
    )
