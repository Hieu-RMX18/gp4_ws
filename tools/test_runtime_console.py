from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path


class _FakeNode:
    def __init__(self, *args, **kwargs):
        pass

    def create_subscription(self, *args, **kwargs):
        return None

    def create_timer(self, *args, **kwargs):
        return None

    def destroy_node(self):
        return None


class _FakeMsg:
    def __init__(self, data: str = "") -> None:
        self.data = data


def _install_fake_ros_modules() -> None:
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.spin = lambda node: None
    rclpy.shutdown = lambda: None

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _FakeNode

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = _FakeMsg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = _FakeMsg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = _FakeMsg

    industrial_msgs = types.ModuleType("industrial_msgs")
    industrial_msgs_msg = types.ModuleType("industrial_msgs.msg")
    industrial_msgs_msg.RobotStatus = _FakeMsg

    diagnostic_msgs = types.ModuleType("diagnostic_msgs")
    diagnostic_msgs_msg = types.ModuleType("diagnostic_msgs.msg")
    diagnostic_msgs_msg.DiagnosticStatus = _FakeMsg

    action_msgs = types.ModuleType("action_msgs")
    action_msgs_msg = types.ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatusArray = _FakeMsg

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": rclpy_node,
            "rclpy.qos": rclpy_qos,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
            "industrial_msgs": industrial_msgs,
            "industrial_msgs.msg": industrial_msgs_msg,
            "diagnostic_msgs": diagnostic_msgs,
            "diagnostic_msgs.msg": diagnostic_msgs_msg,
            "action_msgs": action_msgs,
            "action_msgs.msg": action_msgs_msg,
        }
    )


def _load_runtime_console_module():
    _install_fake_ros_modules()
    module_path = Path(__file__).with_name("runtime_console.py")
    spec = importlib.util.spec_from_file_location("runtime_console_under_test", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _console(module):
    console = module.RuntimeConsole.__new__(module.RuntimeConsole)
    console._mode = "timeline"
    console._event_timeline = []
    console._active_command_id = "cmd-1"
    console._active_command_start_time = 100.0
    console._command_events = 2
    return console


def test_print_event_line_uses_ros_like_node_format_and_details(capsys):
    module = _load_runtime_console_module()
    console = _console(module)

    console._print_event_line(
        {
            "ts": 1778839092.4929056,
            "level": "INFO",
            "layer": "llm_gateway",
            "event": "llm_validation_result",
            "summary": "validated MOVE_REL",
            "source": "regex+llm_validated",
            "details": {"planner_id": "PILZ_LIN", "cartesian_fraction": 0.73},
        }
    )

    output = capsys.readouterr().out
    assert "llm_gateway" in output
    assert "llm_validation_result" in output
    assert "validated MOVE_REL" in output
    assert "source=regex+llm_validated" in output
    assert "planner_id=PILZ_LIN" in output
    assert "cartesian_fraction=0.73" in output


def test_print_summary_includes_source_planner_and_fraction(capsys, monkeypatch):
    module = _load_runtime_console_module()
    console = _console(module)
    console._event_timeline = [
        {
            "cmd_id": "cmd-1",
            "source": "regex+llm_validated",
            "details": {"planner_id": "PILZ_LIN", "cartesian_fraction": 0.73},
        }
    ]
    monkeypatch.setattr(module.time, "time", lambda: 104.0)

    console._print_summary("cmd-1", success=True)

    output = capsys.readouterr().out
    assert "src=regex+llm_validated" in output
    assert "planner=PILZ_LIN" in output
    assert "fraction=0.73" in output


def test_llm_debug_callback_preserves_source_for_non_trace_messages():
    module = _load_runtime_console_module()
    console = _console(module)

    console._llm_debug_callback(
        _FakeMsg('{"stage":"review","status":"accepted","source":"llm_gateway"}')
    )

    assert console._last_cmd_stage == "[review] accepted (source=llm_gateway)"
