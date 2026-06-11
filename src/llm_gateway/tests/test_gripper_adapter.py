from types import SimpleNamespace

from llm_gateway.composite_tools import GripperConfig, GripperIoAdapter


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


def test_gripper_adapter_open_and_close_reject_motion_state():
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

    open_result = adapter.open()
    close_result = adapter.close()

    assert open_result.ok is False
    assert open_result.error == "robot_not_idle"
    assert close_result.ok is False
    assert close_result.error == "robot_not_idle"


def test_gripper_adapter_calls_write_single_io_when_verified():
    """Verify the adapter calls WriteSingleIO service when config is verified."""
    from llm_gateway.composite_tools import GripperConfig, GripperIoAdapter
    from types import SimpleNamespace

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

    class _FakeNode:
        _write_single_io_client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda req: None)
        def _wait_for_future_without_spinning(self, future, _timeout):
            # Simulate successful write
            response = SimpleNamespace(success=True, message="ok")
            return (True, response)

    adapter = GripperIoAdapter(config=config, node=_FakeNode(), robot_mode_fn=lambda: "IDLE")
    result = adapter.open()

    # With a mocked successful service, it should succeed
    assert result.ok is True
