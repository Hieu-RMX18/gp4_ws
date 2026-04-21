# PRE-CONDITION: micro-ROS Agent must be running before launching.
# Start it manually: docker run --rm -it --net=host microros/micro-ros-agent:humble
#   udp4 --port 8888 -v6
# This launch file does NOT start the agent automatically.

import os
import subprocess
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(file_path: Path):
    with open(file_path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _normalized_move_group_parameters(moveit_config):
    parameters = moveit_config.to_dict()
    controllers_config = _load_yaml(moveit_config.package_path / 'config' / 'moveit_controllers.yaml')
    manager_key = 'moveit_simple_controller_manager'
    # Real hardware path: execute directly against MotoROS2 FollowJointTrajectory.
    # Avoid dependency on ros2_control hardware plugin in companion PC workspace.
    manager_config = {
        'controller_names': ['yaskawa'],
        'yaskawa': {
            'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory',
            'default': True,
            'joints': [
                'joint_1_s',
                'joint_2_l',
                'joint_3_u',
                'joint_4_r',
                'joint_5_b',
                'joint_6_t',
            ],
        },
    }

    parameters['trajectory_execution'] = controllers_config.get(
        'trajectory_execution',
        parameters.get('trajectory_execution', {}),
    )
    parameters['moveit_controller_manager'] = controllers_config.get(
        'moveit_controller_manager',
        parameters.get('moveit_controller_manager'),
    )
    parameters[manager_key] = manager_config
    # Real hardware safety boundary: move_group may plan and publish scene state,
    # but must not execute trajectories directly. All execution must flow through
    # execution_gate -> motion_core -> hw_adapter.
    parameters['allow_trajectory_execution'] = False
    # Match the robust planner wiring used by moveit_only.launch.py.
    # Without explicit planning_plugin per pipeline, MoveIt may pick CHOMP
    # for all pipelines and abort simple pose goals.
    parameters['planning_plugin'] = 'pilz_industrial_motion_planner/CommandPlanner'
    parameters['default_planning_pipeline'] = 'pilz_industrial_motion_planner'
    parameters.setdefault('ompl', {})['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    parameters.setdefault('pilz_industrial_motion_planner', {})[
        'planning_plugin'
    ] = 'pilz_industrial_motion_planner/CommandPlanner'
    parameters.setdefault('chomp', {})['planning_plugin'] = 'chomp_interface/CHOMPPlanner'
    return parameters


def _check_rmw_and_agent(_context):
    """V4 A8 + H1-Layer0: verify RMW is FastDDS and micro-ROS Agent is reachable."""
    rmw = os.environ.get('RMW_IMPLEMENTATION', '')
    if rmw != 'rmw_fastrtps_cpp':
        return [
            LogInfo(msg=f'RMW_IMPLEMENTATION is "{rmw}", expected "rmw_fastrtps_cpp".'),
            EmitEvent(event=Shutdown(
                reason='V4 A8: RMW_IMPLEMENTATION must be rmw_fastrtps_cpp for MotoROS2 compatibility.')),
        ]

    try:
        subprocess.run(
            ['bash', '-lc', "ss -lun | grep -q ':8888'"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError:
        return [
            LogInfo(
                msg='micro-ROS Agent readiness check failed: no local UDP listener detected on port 8888.'
            ),
            EmitEvent(event=Shutdown(reason='micro-ROS Agent must be started before hw.launch.py')),
        ]

    return [
        LogInfo(msg='RMW and micro-ROS Agent readiness checks passed.'),
    ]


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.33',
        description='YRC1000micro MotoROS2 controller IP address.',
    )
    agent_ip_arg = DeclareLaunchArgument(
        'agent_ip',
        default_value='192.168.1.99',
        description='External micro-ROS Agent host IP address (documented for operator traceability).',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz alongside the real hardware stack.',
    )

    robot_ip = LaunchConfiguration('robot_ip')
    agent_ip = LaunchConfiguration('agent_ip')
    use_rviz = LaunchConfiguration('use_rviz')
    gp4_bringup_share = get_package_share_directory('gp4_bringup')

    moveit_config = (
        MoveItConfigsBuilder('motoman_gp4', package_name='gp4_moveit_config')
        .robot_description(
            mappings={
                'use_fake_hardware': 'false',
                'robot_ip': robot_ip,
            }
        )
        .to_moveit_configs()
    )
    move_group_parameters = _normalized_move_group_parameters(moveit_config)
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # Required for external MoveGroupInterface clients to resolve robot model.
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # Bridge MotoROS2 joint states into the conventional /joint_states topic
    # expected by MoveGroupInterface clients started via `ros2 run`.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {
                # Use local robot_description parameter (not topic wait mode).
                'use_robot_description_topic': False,
                'rate': 50,
                'source_list': ['/yaskawa/joint_states'],
            },
        ],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[move_group_parameters, planning_scene_monitor_parameters],
        remappings=[
            ('/joint_states', '/yaskawa/joint_states'),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', str(moveit_config.package_path / 'config/moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # V4 A1/B1: hw_adapter is the only execution backend.
    hw_adapter_node = Node(
        package='hw_adapter',
        executable='hw_adapter_node',
        name='hw_adapter_node',
        output='screen',
        parameters=[{
            # Real hardware: MotoROS2 FJT action
            "follow_joint_trajectory_action": "/yaskawa/follow_joint_trajectory",
            "dispatch_action_name": "/hw_adapter/dispatch_trajectory",
        }],
        additional_env={
            'GP4_AGENT_IP': agent_ip,
        },
    )

    # V4 Phase 1: motion_core MUST be launched on real hardware path too.
    # Same /execute_motion contract as simulation.
    motion_core_node = Node(
        package='motion_core',
        executable='motion_core_node',
        name='motion_core_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                # motion_core sends validated trajectories to hw_adapter (not directly to controller)
                "dispatch_action_name": "/hw_adapter/dispatch_trajectory",
                "dispatch_timeout_sec": 60.0,
                "scene_objects_path": os.path.join(gp4_bringup_share, 'config', 'scene_objects.yaml'),
                "require_planning_scene": True,
            },
        ],
    )

    return LaunchDescription([
        LogInfo(
            msg=(
                "[launch hygiene] hw.launch.py starts only the hardware motion stack. "
                "Use system.launch.py for the full guarded hardware path "
                "(safety + supervisor + llm_gateway)."
            )
        ),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        robot_ip_arg,
        agent_ip_arg,
        use_rviz_arg,
        OpaqueFunction(function=_check_rmw_and_agent),
        robot_state_publisher,
        joint_state_publisher,
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher,
                on_start=[hw_adapter_node, move_group, rviz, motion_core_node],
            )
        ),
    ])
