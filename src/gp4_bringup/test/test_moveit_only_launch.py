import importlib.util
import os
import signal
import subprocess
from types import SimpleNamespace
from pathlib import Path

import yaml
from launch import LaunchContext


os.environ.setdefault("ROS_LOG_DIR", "/tmp/gp4_bringup_launch_test_logs")

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "moveit_only.launch.py"
SIM_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "sim.launch.py"
E2E_PATH = WORKSPACE_ROOT / "tools" / "e2e" / "test_full_pipeline.py"
SAFETY_RULES_PATH = WORKSPACE_ROOT / "src" / "safety" / "config" / "safety_rules.yaml"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("moveit_only_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_sim_launch_module():
    spec = importlib.util.spec_from_file_location("sim_launch", SIM_LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_e2e_module():
    spec = importlib.util.spec_from_file_location("gp4_full_pipeline_e2e", E2E_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _substitution_text(values):
    return "".join(getattr(value, "text", "") for value in values)


def _node_signature(node):
    return (
        node.node_package,
        node.node_executable,
    )


def test_move_group_shutdown_timeout_exceeds_default_launch_grace():
    module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations["use_fake_hardware"] = "true"

    move_group_nodes = module._create_move_group(context, move_group_parameters={})

    assert len(move_group_nodes) == 1
    move_group = move_group_nodes[0]
    sigterm_timeout = _substitution_text(
        getattr(move_group, "_ExecuteLocal__sigterm_timeout")
    )
    assert float(sigterm_timeout) >= 20.0


def test_launch_description_constructs_without_runtime_ros(tmp_path, monkeypatch):
    module = _load_launch_module()

    class FakeMoveItConfig:
        package_path = tmp_path
        robot_description = {}
        robot_description_semantic = {}
        robot_description_kinematics = {}
        planning_pipelines = {}

        def to_dict(self):
            return {}

    class FakeMoveItConfigsBuilder:
        def __init__(self, *args, **kwargs):
            pass

        def robot_description(self, *args, **kwargs):
            return self

        def to_moveit_configs(self):
            return FakeMoveItConfig()

    monkeypatch.setattr(module, "get_package_share_directory", lambda package: tmp_path)
    monkeypatch.setattr(module, "MoveItConfigsBuilder", FakeMoveItConfigsBuilder)

    launch_description = module.generate_launch_description()

    assert launch_description.entities


def test_failed_controller_spawner_aborts_launch_instead_of_advancing():
    module = _load_launch_module()
    next_action = object()

    actions = module._continue_after_successful_spawner_exit(
        next_actions=[next_action],
        spawner_label="joint_state_broadcaster",
    )(SimpleNamespace(returncode=1), LaunchContext())

    assert next_action not in actions
    assert any(isinstance(action, module.EmitEvent) for action in actions)
    assert any(
        isinstance(getattr(action, "_EmitEvent__event", None), module.Shutdown)
        for action in actions
    )


def test_successful_controller_spawner_advances_launch():
    module = _load_launch_module()
    next_action = object()

    actions = module._continue_after_successful_spawner_exit(
        next_actions=[next_action],
        spawner_label="joint_state_broadcaster",
    )(SimpleNamespace(returncode=0), LaunchContext())

    assert actions == [next_action]


def test_move_group_launch_disables_moveit_owned_trajectory_execution(tmp_path):
    module = _load_launch_module()
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    (config_dir / "moveit_controllers.yaml").write_text(
        "\n".join(
            [
                "trajectory_execution:",
                "  allowed_execution_duration_scaling: 1.5",
                "moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_simple_controller_manager:",
                "  controller_names: []",
            ]
        )
    )

    fake_moveit_config = SimpleNamespace(
        package_path=tmp_path,
        trajectory_execution={
            "moveit_manage_controllers": True,
            "moveit_simple_controller_manager": {
                "controller_names": ["gp4_arm_controller"]
            },
            "gp4_arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
            },
        },
        to_dict=lambda: {
            "moveit_manage_controllers": True,
            "trajectory_execution": {
                "allowed_execution_duration_scaling": 1.5,
            },
            "moveit_controller_manager": (
                "moveit_simple_controller_manager/MoveItSimpleControllerManager"
            ),
            "moveit_simple_controller_manager": {
                "controller_names": ["gp4_arm_controller"]
            },
            "gp4_arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
            },
            "gp4_arm_controller.type": "FollowJointTrajectory",
            "gp4_arm_controller.action_ns": "follow_joint_trajectory",
        },
    )

    parameters = module._normalized_move_group_parameters(fake_moveit_config)

    assert parameters["allow_trajectory_execution"] is False
    assert (
        "move_group/MoveGroupExecuteTrajectoryAction"
        in parameters["disable_capabilities"]
    )
    assert "move_group/MoveGroupExecuteService" in parameters["disable_capabilities"]
    assert parameters["capabilities"] == module.PILZ_SEQUENCE_ACTION_CAPABILITY
    assert "trajectory_execution" not in parameters
    assert "moveit_controller_manager" not in parameters
    assert "moveit_simple_controller_manager" not in parameters
    assert parameters["moveit_manage_controllers"] is False
    assert "gp4_arm_controller" not in parameters
    assert "gp4_arm_controller.type" not in parameters
    assert "gp4_arm_controller.action_ns" not in parameters


def test_e2e_waits_longer_than_move_group_sigterm_grace():
    launch_module = _load_launch_module()
    e2e_module = _load_e2e_module()

    assert e2e_module.LAUNCH_SHUTDOWN_TIMEOUT_SEC > float(
        launch_module.MOVE_GROUP_SIGTERM_TIMEOUT_SEC
    )


def test_sim_motion_core_shutdown_timeout_exceeds_move_group_teardown(
    tmp_path, monkeypatch
):
    sim_module = _load_sim_launch_module()
    moveit_module = _load_launch_module()
    e2e_module = _load_e2e_module()

    class FakeMoveItConfig:
        package_path = tmp_path
        robot_description = {}

        def to_dict(self):
            return {}

    class FakeMoveItConfigsBuilder:
        def __init__(self, *args, **kwargs):
            pass

        def robot_description(self, *args, **kwargs):
            return self

        def to_moveit_configs(self):
            return FakeMoveItConfig()

    monkeypatch.setattr(
        sim_module, "get_package_share_directory", lambda package: tmp_path
    )
    monkeypatch.setattr(sim_module, "MoveItConfigsBuilder", FakeMoveItConfigsBuilder)

    launch_description = sim_module.generate_launch_description()
    motion_core_nodes = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, sim_module.Node)
        and _node_signature(entity) == ("motion_core", "motion_core_node")
    ]

    assert len(motion_core_nodes) == 1
    motion_core = motion_core_nodes[0]
    motion_core_sigterm = float(
        _substitution_text(getattr(motion_core, "_ExecuteLocal__sigterm_timeout"))
    )
    motion_core_sigkill = float(
        _substitution_text(getattr(motion_core, "_ExecuteLocal__sigkill_timeout"))
    )
    move_group_teardown = float(moveit_module.MOVE_GROUP_SIGTERM_TIMEOUT_SEC) + float(
        moveit_module.MOVE_GROUP_SIGKILL_TIMEOUT_SEC
    )

    assert motion_core_sigterm > move_group_teardown
    assert e2e_module.LAUNCH_SHUTDOWN_TIMEOUT_SEC > (
        motion_core_sigterm + motion_core_sigkill
    )


def test_e2e_rejects_crashed_launch_child_after_shutdown(tmp_path):
    e2e_module = _load_e2e_module()
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "\n".join(
            [
                "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                "[ERROR] [move_group-11]: process has died [pid 165, exit code -11, cmd '/opt/ros/humble/lib/moveit_ros_move_group/move_group'].",
            ]
        )
    )

    try:
        e2e_module._assert_launch_child_exit_records_clean(launch_log)
    except RuntimeError as exc:
        assert "move_group-11" in str(exc)
        assert "exit code -11" in str(exc)
    else:
        raise AssertionError("crashed launch child exit was not rejected")


def test_e2e_allows_move_group_teardown_segfault_after_success(tmp_path):
    e2e_module = _load_e2e_module()
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "\n".join(
            [
                "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                "[ERROR] [move_group-11]: process has died [pid 165, exit code -11, cmd '/opt/ros/humble/lib/moveit_ros_move_group/move_group'].",
            ]
        )
    )

    e2e_module._assert_launch_child_exit_records_clean(
        launch_log,
        scenario_completed=True,
    )


def test_e2e_allows_ros2_control_teardown_segfault_after_success(tmp_path):
    e2e_module = _load_e2e_module()
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "\n".join(
            [
                "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                "[ERROR] [ros2_control_node-8]: process has died [pid 58, exit code -11, cmd '/opt/ros/humble/lib/controller_manager/ros2_control_node'].",
            ]
        )
    )

    e2e_module._assert_launch_child_exit_records_clean(
        launch_log,
        scenario_completed=True,
    )


def test_e2e_rejects_ros2_control_segfault_before_scenario_success(tmp_path):
    e2e_module = _load_e2e_module()
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "\n".join(
            [
                "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                "[ERROR] [ros2_control_node-8]: process has died [pid 58, exit code -11, cmd '/opt/ros/humble/lib/controller_manager/ros2_control_node'].",
            ]
        )
    )

    try:
        e2e_module._assert_launch_child_exit_records_clean(
            launch_log,
            scenario_completed=False,
        )
    except RuntimeError as exc:
        assert "ros2_control_node-8" in str(exc)
        assert "exit code -11" in str(exc)
    else:
        raise AssertionError("pre-success ros2_control crash was not rejected")


def test_e2e_allows_sigint_launch_child_exit_after_shutdown(tmp_path):
    e2e_module = _load_e2e_module()
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "\n".join(
            [
                "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                "[ERROR] [move_group-11]: process has died [pid 165, exit code -2, cmd '/opt/ros/humble/lib/moveit_ros_move_group/move_group'].",
            ]
        )
    )

    e2e_module._assert_launch_child_exit_records_clean(launch_log)


def test_e2e_allows_move_group_sigterm_after_shutdown_escalation(tmp_path):
    e2e_module = _load_e2e_module()
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "\n".join(
            [
                "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                "[ERROR] [move_group-11]: process[move_group-11] failed to terminate '20.0' seconds after receiving 'SIGINT', escalating to 'SIGTERM'",
                "[ERROR] [move_group-11]: process has died [pid 165, exit code -15, cmd '/opt/ros/humble/lib/moveit_ros_move_group/move_group'].",
            ]
        )
    )

    e2e_module._assert_launch_child_exit_records_clean(launch_log)


def test_e2e_allows_move_group_sigkill_after_success_shutdown(tmp_path):
    e2e_module = _load_e2e_module()
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "\n".join(
            [
                "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                "[ERROR] [move_group-11]: process[move_group-11] failed to terminate '25.0' seconds after receiving 'SIGTERM', escalating to 'SIGKILL'",
                "[ERROR] [move_group-11]: process has died [pid 172, exit code -9, cmd '/opt/ros/humble/lib/moveit_ros_move_group/move_group'].",
            ]
        )
    )

    e2e_module._assert_launch_child_exit_records_clean(
        launch_log,
        scenario_completed=True,
    )


def test_e2e_allows_ros2_control_sigterm_after_success_shutdown(tmp_path):
    e2e_module = _load_e2e_module()
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "\n".join(
            [
                "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                "[ERROR] [ros2_control_node-8]: process[ros2_control_node-8] failed to terminate '10.0' seconds after receiving 'SIGTERM', escalating to 'SIGKILL'",
                "[ERROR] [ros2_control_node-8]: process has died [pid 64, exit code -15, cmd '/opt/ros/humble/lib/controller_manager/ros2_control_node'].",
            ]
        )
    )

    e2e_module._assert_launch_child_exit_records_clean(
        launch_log,
        scenario_completed=True,
    )


def test_e2e_rejects_non_move_group_sigterm_after_shutdown(tmp_path):
    e2e_module = _load_e2e_module()
    launch_log = tmp_path / "launch.log"
    launch_log.write_text(
        "\n".join(
            [
                "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
                "[ERROR] [motion_core_node-5]: process has died [pid 165, exit code -15, cmd '/home/hieu2/gp4_ws/install/motion_core/lib/motion_core/motion_core_node'].",
            ]
        )
    )

    try:
        e2e_module._assert_launch_child_exit_records_clean(launch_log)
    except RuntimeError as exc:
        assert "motion_core_node-5" in str(exc)
        assert "exit code -15" in str(exc)
    else:
        raise AssertionError("non-move_group SIGTERM exit was not rejected")


def test_e2e_requests_shutdown_from_launch_parent_first():
    e2e_module = _load_e2e_module()

    class FakeLaunchProcess:
        pid = 4242

        def __init__(self):
            self.signals = []
            self.wait_timeouts = []

        def send_signal(self, shutdown_signal):
            self.signals.append(shutdown_signal)

        def wait(self, timeout):
            self.wait_timeouts.append(timeout)
            return 0

    launch_process = FakeLaunchProcess()

    e2e_module._shutdown_launch_process(launch_process)

    assert launch_process.signals == [signal.SIGINT]
    assert launch_process.wait_timeouts == [e2e_module.LAUNCH_SHUTDOWN_TIMEOUT_SEC]


def test_e2e_uses_process_group_sigterm_only_after_shutdown_timeout(monkeypatch):
    e2e_module = _load_e2e_module()
    events = []

    class FakeLaunchProcess:
        pid = 4242

        def send_signal(self, shutdown_signal):
            events.append(("send_signal", shutdown_signal))

        def wait(self, timeout):
            events.append(("wait", timeout))
            if len([event for event in events if event[0] == "wait"]) == 1:
                raise subprocess.TimeoutExpired("ros2 launch", timeout)
            return 0

    def record_killpg(pid, shutdown_signal):
        events.append(("killpg", pid, shutdown_signal))

    monkeypatch.setattr(e2e_module.os, "killpg", record_killpg)

    e2e_module._shutdown_launch_process(FakeLaunchProcess())

    assert events == [
        ("send_signal", signal.SIGINT),
        ("wait", e2e_module.LAUNCH_SHUTDOWN_TIMEOUT_SEC),
        ("killpg", 4242, signal.SIGTERM),
        ("wait", 10.0),
    ]


def test_e2e_staging_joint_target_is_inside_operational_limits():
    e2e_module = _load_e2e_module()
    safety_rules = yaml.safe_load(SAFETY_RULES_PATH.read_text()) or {}
    joint_limits = safety_rules["operational_joint_limits"]
    target = e2e_module.SOFTWARE_STAGING_JOINT_TARGET

    assert len(target) == 6
    for joint_name, value in zip(e2e_module.GP4_JOINT_NAMES, target):
        limit = joint_limits[joint_name]
        if "default" in limit:
            limit = limit["default"]
        assert limit["min"] <= value <= limit["max"]

    home_j5 = e2e_module.HOME_JOINT_TARGET[4]
    staging_j5 = target[4]
    assert abs(staging_j5 - home_j5) >= 0.5


def test_e2e_move_rel_delta_returns_inside_workspace_from_low_pose():
    e2e_module = _load_e2e_module()
    safety_rules = {
        "workspace_bounds": {
            "z_min": 0.15,
            "z_max": 1.0,
        },
        "motion_limits": {
            "max_move_rel_translation": 0.05,
        },
    }
    pose = SimpleNamespace(
        position=SimpleNamespace(
            z=0.10,
        )
    )

    _, _, dz = e2e_module._move_rel_delta(pose, safety_rules)

    assert dz > 0.0
    assert abs(dz) <= safety_rules["motion_limits"]["max_move_rel_translation"]
    assert pose.position.z + dz >= safety_rules["workspace_bounds"]["z_min"]
