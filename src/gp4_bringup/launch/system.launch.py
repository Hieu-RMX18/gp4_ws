import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='true launches the software-only stack; false launches the real hardware stack.',
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.33',
        description='Robot IP address forwarded to hw.launch.py when real hardware is selected.',
    )
    agent_ip_arg = DeclareLaunchArgument(
        'agent_ip',
        default_value='192.168.1.99',
        description='micro-ROS Agent host IP address forwarded to hw.launch.py.',
    )

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    agent_ip = LaunchConfiguration('agent_ip')

    gp4_bringup_share = get_package_share_directory('gp4_bringup')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gp4_bringup_share, 'launch', 'sim.launch.py')
        ),
        condition=IfCondition(use_fake_hardware),
    )

    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gp4_bringup_share, 'launch', 'hw.launch.py')
        ),
        condition=UnlessCondition(use_fake_hardware),
        launch_arguments={
            'robot_ip': robot_ip,
            'agent_ip': agent_ip,
        }.items(),
    )

    llm_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gp4_bringup_share, 'launch', 'llm_stack.launch.py')
        ),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        use_fake_hardware_arg,
        robot_ip_arg,
        agent_ip_arg,
        sim_launch,
        hw_launch,
        llm_stack_launch,
    ])
