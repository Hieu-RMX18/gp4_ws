import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    llm_provider_arg = DeclareLaunchArgument(
        'llm_provider',
        default_value='9router_local',
        description='Logical LLM provider label exported to the llm_gateway process environment.',
    )
    audit_log_path_arg = DeclareLaunchArgument(
        'audit_log_path',
        default_value='/tmp/gp4_audit',
        description='Directory used by supervisor audit_logger for rosbag2 and JSONL output.',
    )
    # SAFETY: approval auto-clear is ONLY enabled for sim/fake-hardware.
    # Real hardware MUST require explicit approval. This argument is forwarded
    # from system.launch.py so that the auto_clear parameter tracks
    # the hardware mode consistently.
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='When true, enables auto_clear_unimplemented_approval for sim compatibility.',
    )

    llm_provider = LaunchConfiguration('llm_provider')
    audit_log_path = LaunchConfiguration('audit_log_path')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    supervisor_share = get_package_share_directory('supervisor')
    analyzers_config = os.path.join(
        supervisor_share,
        'config',
        'diagnostics_analyzers.yaml',
    )
    supervisor_params = os.path.join(
        supervisor_share,
        'config',
        'supervisor_defaults.yaml',
    )

    safety_node = Node(
        package='safety',
        executable='safety_manager',
        name='safety',
        output='screen',
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

    # SAFETY: auto_clear_unimplemented_approval is gated by use_fake_hardware.
    # When use_fake_hardware is false (real hardware), approval is NEVER auto-cleared.
    auto_clear_value = PythonExpression(
        ["'", use_fake_hardware, "'.lower() == 'true'"]
    )

    llm_gateway_node = Node(
        package='llm_gateway',
        executable='llm_gateway_node',
        name='llm_gateway',
        output='screen',
        additional_env={
            'LLM_PROVIDER': llm_provider,
        },
        parameters=[{
            'auto_clear_unimplemented_approval': auto_clear_value,
        }],
    )

    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[analyzers_config],
    )

    return LaunchDescription([
        llm_provider_arg,
        audit_log_path_arg,
        use_fake_hardware_arg,
        safety_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=safety_node,
                on_start=[supervisor_node],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=supervisor_node,
                on_start=[llm_gateway_node, diagnostic_aggregator],
            )
        ),
    ])
