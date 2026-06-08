from types import SimpleNamespace

from llm_gateway.composite_tools import GripperConfig, GripperIoAdapter
from llm_gateway.react_planner import GripperCloseTool, GripperOpenTool


def test_gripper_config_requires_verified_values():
    config = GripperConfig.from_rules({"gripper": {"open_output_address": "VERIFY_CONFIG"}})

    result = GripperIoAdapter(config=config, node=None, robot_mode_fn=lambda: "IDLE").open()

    assert result.ok is False
    assert result.error == "verify_config_required"


def test_gripper_adapter_rejects_motion_state_before_io():
    config = GripperConfig(
        write_single_io_service="/io_set",
        read_single_io_service="/read_single_io",
        open_output_address=10010,
        open_output_value=1,
        close_output_address=10010,
        close_output_value=0,
        closed_input_address=20010,
        closed_input_active_value=1,
        feedback_timeout_sec=1.0,
    )

    result = GripperIoAdapter(config=config, node=None, robot_mode_fn=lambda: "MOVING").close()

    assert result.ok is False
    assert result.error == "robot_not_idle"


def test_react_gripper_tools_delegate_to_adapter():
    config = GripperConfig(
        write_single_io_service="/io_set",
        read_single_io_service="/read_single_io",
        open_output_address=10010,
        open_output_value=1,
        close_output_address=10010,
        close_output_value=0,
        closed_input_address=20010,
        closed_input_active_value=1,
        feedback_timeout_sec=1.0,
    )
    adapter = GripperIoAdapter(config=config, node=None, robot_mode_fn=lambda: "MOVING")
    context = SimpleNamespace(ros_node=SimpleNamespace(_gripper_adapter=adapter))

    open_result = GripperOpenTool().invoke({}, context)
    close_result = GripperCloseTool().invoke({}, context)

    assert open_result.ok is False
    assert open_result.error == "robot_not_idle"
    assert close_result.ok is False
    assert close_result.error == "robot_not_idle"
