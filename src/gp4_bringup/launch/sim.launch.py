import os

import yaml
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
    analyzers_config = os.path.join(
        supervisor_share,
        'config',
        'diagnostics_analyzers.yaml',
    )
    use_fake_hardware_value = 'true'
    audit_log_path_arg = DeclareLaunchArgument(
        'audit_log_path',
        default_value='/tmp/gp4_audit',
        description='Directory used by supervisor audit_logger for rosbag2 and JSONL output.',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for MoveIt visualization in sim bringup.',
    )
    audit_log_path = LaunchConfiguration('audit_log_path')
    use_rviz = LaunchConfiguration('use_rviz')
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
            'use_rviz': use_rviz,
        }.items(),
    )

    safety_node = Node(
        package='safety',
        executable='safety_manager',
        name='safety',
        output='screen',
        parameters=[{
            'sim_mode': True,
        }],
    )

    # Compatibility only: direct llm_gateway dispatch now always sends
    # require_approval=false for executable commands. The parameter is kept
    # stable for older launch/test tooling.
    llm_gateway_node = Node(
        package='llm_gateway',
        executable='llm_gateway_node',
        name='llm_gateway',
        output='screen',
        parameters=[{
            'auto_clear_unimplemented_approval': True,
        }],
    )

    # V4 A1: hw_adapter is the only execution backend, even in simulation.
    # In sim mode, it connects to the fake controller's FJT action.
    # SIM MODE: bypass robot_status readiness for RViz simulation only.
    hw_adapter_node = Node(
        package='hw_adapter',
        executable='hw_adapter_node',
        name='hw_adapter_node',
        output='screen',
        parameters=[{
            # Fake hardware exposes FollowJointTrajectory on /controller_manager.
            # Route hw_adapter there so confirmed sim commands execute end-to-end.
            "follow_joint_trajectory_action": "/controller_manager/follow_joint_trajectory",
            "dispatch_action_name": "/hw_adapter/dispatch_trajectory",
            "robot_status_topic": "/yaskawa/robot_status",
            # Fake ros2_control publishes /joint_states directly in sim.
            "joint_states_topic": "/joint_states",
            "start_traj_mode_service": "",
            "reset_error_service": "",
            "sim_mode": True,
        }],
    )

    # V4 Phase 1: motion_core is plan-only. It sends validated trajectories
    # to hw_adapter via DispatchTrajectory, not directly to FJT.
    # Load motion limits from safety_rules.yaml (single source of truth).
    _safety_yaml_path = os.path.join(
        get_package_share_directory('safety'), 'config', 'safety_rules.yaml')
    _safety_rules = {}
    if os.path.exists(_safety_yaml_path):
        with open(_safety_yaml_path, 'r') as _f:
            _safety_rules = yaml.safe_load(_f) or {}
    _motion_limits = _safety_rules.get('motion_limits', {})

    motion_core_node = Node(
        package='motion_core',
        executable='motion_core_node',
        name='motion_core_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                # motion_core dispatches to hw_adapter, no direct FJT client
                "dispatch_action_name": "/hw_adapter/dispatch_trajectory",
                "dispatch_timeout_sec": 30.0,
                # V4 J9: planning scene collision objects
                "scene_objects_path": os.path.join(gp4_bringup_share, 'config', 'scene_objects.yaml'),
                # Motion limits from safety_rules.yaml
                "max_velocity_scale": _motion_limits.get('max_velocity_scale', 0.06),
                "max_acceleration_scale": _motion_limits.get('max_acceleration_scale', 0.06),
            },
        ],
        remappings=[
            ('/yaskawa/joint_states', '/joint_states'),
        ],
    )

    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[analyzers_config],
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
        use_rviz_arg,
        moveit_only,
        safety_node,
        llm_gateway_node,
        hw_adapter_node,
        motion_core_node,
        diagnostic_aggregator,
        supervisor_node,
    ])
