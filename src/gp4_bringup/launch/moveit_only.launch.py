from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(file_path: Path):
    with open(file_path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _normalized_move_group_parameters(moveit_config):
    parameters = moveit_config.to_dict()
    controllers_config = _load_yaml(
        moveit_config.package_path / "config" / "moveit_controllers.yaml"
    )
    manager_key = "moveit_simple_controller_manager"
    manager_config = dict(controllers_config.get(manager_key, {}))
    controller_names = list(manager_config.get("controller_names", []))

    for controller_name in controller_names:
        if controller_name in manager_config:
            continue
        controller_config = controllers_config.get(controller_name)
        if controller_config is not None:
            manager_config[controller_name] = controller_config
            parameters.pop(controller_name, None)

    parameters["trajectory_execution"] = controllers_config.get(
        "trajectory_execution",
        parameters.get("trajectory_execution", {}),
    )
    parameters["moveit_controller_manager"] = controllers_config.get(
        "moveit_controller_manager",
        parameters.get("moveit_controller_manager"),
    )
    parameters[manager_key] = manager_config
    parameters["planning_plugin"] = "pilz_industrial_motion_planner/CommandPlanner"
    parameters["default_planning_pipeline"] = "pilz_industrial_motion_planner"
    parameters.setdefault("ompl", {})["planning_plugin"] = "ompl_interface/OMPLPlanner"
    parameters.setdefault("pilz_industrial_motion_planner", {})["planning_plugin"] = (
        "pilz_industrial_motion_planner/CommandPlanner"
    )
    parameters.setdefault("chomp", {})["planning_plugin"] = (
        "chomp_interface/CHOMPPlanner"
    )
    return parameters


def _move_group_remappings(use_fake_hardware_value: str):
    if use_fake_hardware_value.lower() == "true":
        return []
    return [
        ("/joint_states", "/yaskawa/joint_states"),
        ("/tf", "/yaskawa/tf"),
        ("/tf_static", "/yaskawa/tf_static"),
    ]


def _create_move_group(context, *, move_group_parameters):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[move_group_parameters],
            remappings=_move_group_remappings(use_fake_hardware),
        )
    ]


def generate_launch_description():
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use ros2_control mock_components/GenericSystem for MoveIt-only bringup.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz for MoveIt visualization.",
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_rviz = LaunchConfiguration("use_rviz")

    moveit_config = (
        MoveItConfigsBuilder("motoman_gp4", package_name="gp4_moveit_config")
        .robot_description(
            mappings={
                "use_fake_hardware": use_fake_hardware,
            }
        )
        .to_moveit_configs()
    )
    gp4_bringup_share = Path(get_package_share_directory("gp4_bringup"))
    ros2_controllers_file = str(
        moveit_config.package_path / "config" / "ros2_controllers.yaml"
    )
    arm_controller_params_file = str(
        gp4_bringup_share / "config" / "gp4_arm_controller_spawner.yaml"
    )
    move_group_parameters = _normalized_move_group_parameters(moveit_config)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_file,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    gp4_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gp4_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--param-file",
            arm_controller_params_file,
        ],
        output="screen",
    )

    move_group = OpaqueFunction(
        function=_create_move_group,
        kwargs={
            "move_group_parameters": move_group_parameters,
        },
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            use_fake_hardware_arg,
            use_rviz_arg,
            robot_state_publisher,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=robot_state_publisher,
                    on_start=[controller_manager],
                )
            ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=controller_manager,
                    on_start=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[gp4_arm_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gp4_arm_controller_spawner,
                    on_exit=[move_group, rviz],
                )
            ),
        ]
    )
