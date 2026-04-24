import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gp4_bringup_share = get_package_share_directory('gp4_bringup')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for MoveIt visualization in demo bringup.',
    )
    use_rviz = LaunchConfiguration('use_rviz')

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

    moveit_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gp4_bringup_share, 'launch', 'moveit_only.launch.py')
        ),
        launch_arguments={
            'use_fake_hardware': 'true',
            'use_rviz': use_rviz,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        use_rviz_arg,
        static_tf,
        moveit_only,
    ])
