from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os


def _move_group_remappings(use_fake_hardware_value: str):
    if use_fake_hardware_value.lower() == 'true':
        return []
    return [
        ('/joint_states', '/yaskawa/joint_states'),
        ('/tf', '/yaskawa/tf'),
        ('/tf_static', '/yaskawa/tf_static'),
    ]


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

    # Build normalized parameters
    params = moveit_config.to_dict()
    params['planning_plugin'] = 'pilz_industrial_motion_planner/CommandPlanner'
    params['default_planning_pipeline'] = 'pilz_industrial_motion_planner'

    # Load kinematics.yaml explicitly (fixes "No kinematics plugins defined" warning)
    kinematics_config_path = os.path.join(
        FindPackageShare('gp4_moveit_config').perform(context),
        'config', 'kinematics.yaml'
    )
    import yaml
    with open(kinematics_config_path, 'r') as f:
        kinematics_params = yaml.safe_load(f)
    params.update(kinematics_params)

    # Load moveit_controllers.yaml to fix "No controller_names specified" error
    controllers_config_path = os.path.join(
        FindPackageShare('gp4_moveit_config').perform(context),
        'config', 'moveit_controllers.yaml'
    )
    with open(controllers_config_path, 'r') as f:
        controllers_params = yaml.safe_load(f)
    # Extract only the controller manager settings
    if 'moveit_simple_controller_manager' in controllers_params:
        params['moveit_simple_controller_manager'] = controllers_params['moveit_simple_controller_manager']
    if 'moveit_controller_manager' in controllers_params:
        params['moveit_controller_manager'] = controllers_params['moveit_controller_manager']
    if 'trajectory_execution' in controllers_params:
        params['trajectory_execution'] = controllers_params['trajectory_execution']

    # Load Pilz planner config
    pilz_config_path = os.path.join(
        FindPackageShare('gp4_moveit_config').perform(context),
        'config', 'pilz_industrial_motion_planner_planning.yaml'
    )
    with open(pilz_config_path, 'r') as f:
        pilz_params = yaml.safe_load(f)
    params.setdefault('pilz_industrial_motion_planner', {}).update(pilz_params)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[params],
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
