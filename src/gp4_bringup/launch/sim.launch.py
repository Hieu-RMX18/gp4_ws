import os

from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gp4_bringup_share = get_package_share_directory('gp4_bringup')
    supervisor_share = get_package_share_directory('supervisor')
    use_fake_hardware_value = 'true'
    audit_log_path_arg = DeclareLaunchArgument(
        'audit_log_path',
        default_value='/tmp/gp4_audit',
        description='Directory used by supervisor audit_logger for rosbag2 and JSONL output.',
    )
    audit_log_path = LaunchConfiguration('audit_log_path')
    supervisor_params = os.path.join(
        supervisor_share,
        'config',
        'supervisor_defaults.yaml',
    )
    moveit_config = (
        MoveItConfigsBuilder('motoman_gp4', package_name='gp4_moveit_config')
        .robot_description(
            mappings={
                'use_fake_hardware': use_fake_hardware_value,
            }
        )
        .to_moveit_configs()
    )

    moveit_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gp4_bringup_share, 'launch', 'moveit_only.launch.py')
        ),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware_value,
            'use_rviz': 'true',
        }.items(),
    )

    safety_node = Node(
        package='safety',
        executable='safety_manager',
        name='safety',
        output='screen',
    )

    motion_core_node = Node(
        package='motion_core',
        executable='motion_core_node',
        name='motion_core_node',
        output='screen',
        parameters=[moveit_config.to_dict()],
        remappings=[
            ('/yaskawa/joint_states', '/joint_states'),
        ],
    )

    supervisor_node = Node(
        package='supervisor',
        executable='supervisor_node',
        name='supervisor_node',
        output='screen',
        parameters=[
            supervisor_params,
            {
                'audit_log_path': ParameterValue(audit_log_path, value_type=str),
            },
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        audit_log_path_arg,
        moveit_only,
        safety_node,
        motion_core_node,
        supervisor_node,
    ])
