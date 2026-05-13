"""Tests for disabled extended-mode policy in CommandValidator."""

import json

import pytest
from safety.command_validator import CommandValidator


@pytest.fixture
def validator():
    """Validator with default-only joint_6_t config."""
    rules = {
        "motion_limits": {"max_velocity_scale": 0.06, "max_acceleration_scale": 0.06},
        "operational_joint_limits": {
            "joint_1_s": {"min": -2.967, "max": 2.967},
            "joint_2_l": {"min": -1.920, "max": 2.269},
            "joint_3_u": {"min": -1.134, "max": 3.491},
            "joint_4_r": {"min": -2.443, "max": 2.443},
            "joint_5_b": {"min": -1.603, "max": 1.603},
            "joint_6_t": {"default": {"min": -3.142, "max": 3.142}},
        },
    }
    return CommandValidator(rules)


def test_default_mode_passes(validator):
    cmd = json.dumps({"primitive_type": "HOME", "velocity_scale": 0.06})
    valid, reason = validator.validate(cmd)
    assert valid is True
    assert reason == ""


def test_motion_without_extended_tokens_passes(validator):
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "velocity_scale": 0.06,
        }
    )
    valid, reason = validator.validate(cmd)
    assert valid is True
    assert reason == ""


def test_extended_mode_is_disabled_with_clear_reason(validator):
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": True,
            "velocity_scale": 0.06,
        }
    )
    valid, reason = validator.validate(cmd)
    assert valid is False
    assert "extended_mode disabled" in reason
    assert "default joint_6_t envelope" in reason


def test_default_mode_ignores_extended_fields(validator):
    """Default mode ignores extended_mode tokens even if present."""
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": False,
            "velocity_scale": 0.06,
        }
    )
    valid, reason = validator.validate(cmd)
    assert valid is True
