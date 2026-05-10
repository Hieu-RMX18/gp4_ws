"""Tests for ReAct state injector."""

import ast
from pathlib import Path

from llm_gateway.react_planner import StateInjector


def _subscription_qos_arg(constructor: ast.FunctionDef, topic: str) -> ast.AST:
    for node in ast.walk(constructor):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "create_subscription"
            and len(node.args) >= 4
            and isinstance(node.args[1], ast.Constant)
            and node.args[1].value == topic
        ):
            continue
        return node.args[3]
    raise AssertionError(f"missing {topic} subscription")


def test_react_state_subscriptions_use_sensor_qos():
    node_path = (
        Path(__file__).resolve().parents[1]
        / "llm_gateway"
        / "llm_gateway_node.py"
    )
    module = ast.parse(node_path.read_text(encoding="utf-8"))
    node_class = next(
        item
        for item in module.body
        if isinstance(item, ast.ClassDef) and item.name == "LLMGatewayNode"
    )
    constructor = next(
        item
        for item in node_class.body
        if isinstance(item, ast.FunctionDef) and item.name == "__init__"
    )

    for topic in ("/yaskawa/joint_states", "/yaskawa/robot_status"):
        qos_arg = _subscription_qos_arg(constructor, topic)
        assert isinstance(qos_arg, ast.Name)
        assert qos_arg.id == "qos_profile_sensor_data"


def test_default_snapshot():
    inj = StateInjector()
    snap = inj.snapshot()
    rs = snap["robot_state"]
    assert rs["joints_rad"] == [0.0] * 6
    assert rs["joint_names"] == [
        "joint_1_s",
        "joint_2_l",
        "joint_3_u",
        "joint_4_r",
        "joint_5_b",
        "joint_6_t",
    ]
    assert rs["mode"] == "IDLE"
    assert rs["active_alarms"] == []
    assert rs["velocity_scale_active"] == 0.06
    assert rs["capabilities"] == {"gripper": False, "perception": False}


def test_update_joint_states():
    inj = StateInjector()
    inj.update_joint_states({"position": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]})
    snap = inj.snapshot()
    assert snap["robot_state"]["joints_rad"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_update_robot_status():
    inj = StateInjector()
    inj.update_robot_status({"mode": "MOVING", "active_alarms": ["alarm1"]})
    snap = inj.snapshot()
    assert snap["robot_state"]["mode"] == "MOVING"
    assert snap["robot_state"]["active_alarms"] == ["alarm1"]


def test_set_velocity_scale():
    inj = StateInjector()
    inj.set_velocity_scale(0.25)
    assert inj.snapshot()["robot_state"]["velocity_scale_active"] == 0.25


def test_set_capabilities():
    inj = StateInjector()
    inj.set_capabilities(gripper=True, perception=False)
    caps = inj.snapshot()["robot_state"]["capabilities"]
    assert caps["gripper"] is True
    assert caps["perception"] is False
