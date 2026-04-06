import json
import pytest
from safety.command_validator import CommandValidator

def test_valid_home_command(safety_rules):
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({"primitive_type": "HOME", "velocity_scale": 0.2})
    valid, reason = validator.validate(cmd)
    assert valid is True
    assert reason == ""

def test_velocity_scale_exceeds(safety_rules):
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({"primitive_type": "HOME", "velocity_scale": 0.6})
    valid, reason = validator.validate(cmd)
    assert valid is False
    assert "exceeds max allowed" in reason

def test_malformed_json(safety_rules):
    validator = CommandValidator(safety_rules)
    cmd = "{invalid_json:"
    valid, reason = validator.validate(cmd)
    assert valid is False
    assert "Invalid JSON format" in reason


# ── SET_SPEED tests ──

def test_set_speed_valid_at_015(safety_rules):
    """SET_SPEED with velocity_scale 0.15 passes safety check."""
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({"primitive_type": "SET_SPEED", "velocity_scale": 0.15})
    valid, reason = validator.validate(cmd)
    assert valid is True

def test_set_speed_rejected_above_030(safety_rules):
    """SET_SPEED with velocity_scale 0.80 exceeds max_velocity_scale 0.3."""
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({"primitive_type": "SET_SPEED", "velocity_scale": 0.80})
    valid, reason = validator.validate(cmd)
    assert valid is False
    assert "exceeds max allowed" in reason


# ── Non-motion primitives bypass velocity checks ──

def test_wait_bypasses_velocity_check(safety_rules):
    """WAIT passes without velocity_scale."""
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({"primitive_type": "WAIT", "wait_duration_sec": 2.5})
    valid, reason = validator.validate(cmd)
    assert valid is True

def test_stop_bypasses_velocity_check(safety_rules):
    """STOP passes without velocity_scale."""
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({"primitive_type": "STOP"})
    valid, reason = validator.validate(cmd)
    assert valid is True

def test_alarm_reset_bypasses_velocity_check(safety_rules):
    """ALARM_RESET passes without velocity_scale."""
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({"primitive_type": "ALARM_RESET"})
    valid, reason = validator.validate(cmd)
    assert valid is True

def test_io_set_bypasses_velocity_check(safety_rules):
    """IO_SET passes without velocity_scale."""
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({"primitive_type": "IO_SET", "io_address": 10010, "io_value": 1})
    valid, reason = validator.validate(cmd)
    assert valid is True


# ── Motion primitives require valid velocity_scale ──

def test_move_joint_requires_velocity(safety_rules):
    """MOVE_JOINT is a motion command — velocity_scale defaults to 1.0 and exceeds max."""
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({"primitive_type": "MOVE_JOINT", "joint_index": 2, "joint_angle": 0.5})
    valid, reason = validator.validate(cmd)
    # velocity_scale defaults to 1.0 (>0.3), so rejected
    assert valid is False
    assert "exceeds max allowed" in reason

def test_move_joint_valid_with_velocity(safety_rules):
    """MOVE_JOINT passes with valid velocity_scale."""
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({
        "primitive_type": "MOVE_JOINT",
        "joint_index": 2, "joint_angle": 0.5,
        "velocity_scale": 0.15
    })
    valid, reason = validator.validate(cmd)
    assert valid is True

def test_move_joints_requires_velocity(safety_rules):
    """MOVE_JOINTS is a motion command — velocity_scale defaults to 1.0 and exceeds max."""
    validator = CommandValidator(safety_rules)
    cmd = json.dumps({
        "primitive_type": "MOVE_JOINTS",
        "joint_target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    })
    valid, reason = validator.validate(cmd)
    # velocity_scale defaults to 1.0 (>0.3), so rejected
    assert valid is False
    assert "exceeds max allowed" in reason

