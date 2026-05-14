"""Tests for ExecutionGate._validate_blended_sequence (W4)."""

import json
from unittest.mock import MagicMock

import pytest
from geometry_msgs.msg import Pose

from safety.command_validator import CommandValidator
from safety.execution_gate import ExecutionGate
from safety.workspace_guard import WorkspaceGuard


def _mock_node():
    node = MagicMock()
    node.get_logger.return_value = MagicMock(
        info=MagicMock(), warn=MagicMock(), error=MagicMock()
    )
    return node


def _make_gate(safety_rules):
    node = _mock_node()
    validator = CommandValidator(safety_rules)
    guard = WorkspaceGuard(safety_rules)
    gate = ExecutionGate(node, validator, guard, safety_manager=None)
    return gate


def _make_request():
    """Return a mock ValidateCommand request with a target_pose factory."""
    req = MagicMock()
    req.target_pose = Pose()
    req.target_pose.__class__ = Pose
    return req


def _valid_step(
    primitive_type="LIN",
    goal_type=0,
    blend_radius_m=0.0,
    velocity_scale=None,
    acceleration_scale=None,
    x=0.15,
    y=-0.15,
    z=0.30,
):
    step = {
        "primitive_type": primitive_type,
        "goal_type": goal_type,
        "blend_radius_m": blend_radius_m,
        "target_pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }
    if velocity_scale is not None:
        step["velocity_scale"] = velocity_scale
    if acceleration_scale is not None:
        step["acceleration_scale"] = acceleration_scale
    return step


def _joints_step(joint_target, blend_radius_m=0.0, primitive_type="PTP"):
    return {
        "primitive_type": primitive_type,
        "goal_type": 1,
        "blend_radius_m": blend_radius_m,
        "joint_target": list(joint_target),
    }


def _named_step(named_target, blend_radius_m=0.0, primitive_type="PTP"):
    return {
        "primitive_type": primitive_type,
        "goal_type": 2,
        "blend_radius_m": blend_radius_m,
        "named_target": named_target,
    }


# ── Basic acceptance / rejection ──


def test_valid_two_step_pose_sequence(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _valid_step(blend_radius_m=0.01),
            _valid_step(blend_radius_m=0.0, x=0.20),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is True
    assert reason == ""


def test_rejects_single_step(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [_valid_step(blend_radius_m=0.0)],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "at least 2 sequence_steps" in reason


def test_rejects_negative_blend_radius(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _valid_step(blend_radius_m=-0.01),
            _valid_step(blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "blend_radius must be >= 0.0" in reason


def test_rejects_last_step_nonzero_blend(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _valid_step(blend_radius_m=0.0),
            _valid_step(blend_radius_m=0.01),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "last step blend_radius must be 0.0" in reason


def test_rejects_unsupported_primitive_type(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _valid_step(primitive_type="CIRC", blend_radius_m=0.0),
            _valid_step(primitive_type="LIN", blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "unsupported primitive_type" in reason


# ── Scale cap checks ──


def test_rejects_step_velocity_above_max(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _valid_step(blend_radius_m=0.0, velocity_scale=0.09),
            _valid_step(blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "velocity_scale" in reason and "exceeds max allowed" in reason


def test_rejects_step_acceleration_non_positive(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _valid_step(blend_radius_m=0.0, acceleration_scale=0.0),
            _valid_step(blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "acceleration_scale" in reason and "must be positive" in reason


# ── Workspace bounds for POSE steps ──


def test_rejects_pose_step_outside_workspace(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _valid_step(blend_radius_m=0.0, x=10.0, y=0.0, z=0.30),
            _valid_step(blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "out of bounds" in reason


# ── GOAL_JOINTS checks ──


def test_valid_joints_step(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _joints_step([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], blend_radius_m=0.01),
            _valid_step(blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is True


def test_rejects_joints_step_wrong_count(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _joints_step([0.0, 0.0, 0.0], blend_radius_m=0.01),
            _valid_step(blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "exactly 6 joint_target values" in reason


def test_rejects_joints_step_outside_limits(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _joints_step([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], blend_radius_m=0.01),
            # J5 limit max is 1.603 rad; use 2.0 rad to trigger reject
            _joints_step([0.0, 0.0, 0.0, 0.0, 2.0, 0.0], blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "joint_5_b" in reason and "above limit" in reason


# ── GOAL_NAMED checks ──


def test_valid_named_step_home(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _named_step("home", blend_radius_m=0.01),
            _valid_step(blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is True


def test_rejects_named_step_not_in_allowlist(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            _named_step("unknown_target", blend_radius_m=0.01),
            _valid_step(blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "named_target" in reason and "not in allowed set" in reason


def test_rejects_named_step_empty(safety_rules):
    gate = _make_gate(safety_rules)
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            {"primitive_type": "PTP", "goal_type": 2, "blend_radius_m": 0.01},
            _valid_step(blend_radius_m=0.0),
        ],
    }
    ok, reason = gate._validate_blended_sequence(cmd, _make_request())
    assert ok is False
    assert "GOAL_NAMED requires named_target" in reason
