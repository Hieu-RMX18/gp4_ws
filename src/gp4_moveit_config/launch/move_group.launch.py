from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _move_group_remappings(use_fake_hardware_value: str):
    if use_fake_hardware_value.lower() == 'true':
        return []
    return [
        ('/joint_states', '/yaskawa/joint_states'),
        ('/tf', '/yaskawa/tf'),
        ('/tf_static', '/yaskawa/tf_static'),
    ]


def _normalized_move_group_parameters(moveit_config):
    parameters = moveit_config.to_dict()
    parameters['planning_plugin'] = 'pilz_industrial_motion_planner/CommandPlanner'
    parameters['default_planning_pipeline'] = 'pilz_industrial_motion_planner'
    parameters.setdefault('ompl', {})['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    parameters.setdefault('pilz_industrial_motion_planner', {})[
        'planning_plugin'
    ] = 'pilz_industrial_motion_planner/CommandPlanner'
    parameters.setdefault('chomp', {})['planning_plugin'] = 'chomp_interface/CHOMPPlanner'
    return parameters


def _launch_setup(context):
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    moveit_config = (
        MoveItConfigsBuilder('motoman_gp4', package_name='gp4_moveit_config')
        .robot_description(
            mappings={
                'use_fake_hardware': use_fake_hardware,
            }
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            _normalized_move_group_parameters(moveit_config),
        ],
        remappings=_move_group_remappings(use_fake_hardware),
    )

    return [move_group_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Disable /yaskawa remaps when using fake hardware.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
