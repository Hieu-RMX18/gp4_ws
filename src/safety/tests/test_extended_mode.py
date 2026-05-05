"""W7.T6: Tests for extended-mode precondition gate in CommandValidator."""

import json

import pytest
from safety.command_validator import CommandValidator, _extended_runs


@pytest.fixture
def validator():
    """Validator with tiered joint_6_t config."""
    rules = {
        "motion_limits": {"max_velocity_scale": 0.06, "max_acceleration_scale": 0.06},
        "operational_joint_limits": {
            "joint_1_s": {"min": -2.967, "max": 2.967},
            "joint_2_l": {"min": -1.920, "max": 2.269},
            "joint_3_u": {"min": -1.134, "max": 3.491},
            "joint_4_r": {"min": -2.443, "max": 2.443},
            "joint_5_b": {"min": -1.603, "max": 1.603},
            "joint_6_t": {
                "default": {"min": -3.142, "max": 3.142},
                "extended": {"min": -7.941, "max": 7.941},
                "extended_preconditions": {
                    "cable_inspection_signed_off": True,
                    "max_velocity_scale": 0.10,
                    "requires_operator_confirm": True,
                    "max_continuous_extended_time_s": 30,
                    "cool_down_s_between_runs": 60,
                },
            },
        },
    }
    return CommandValidator(rules)


@pytest.fixture(autouse=True)
def reset_cooldown():
    """Reset cooldown tracker between tests."""
    _extended_runs._last_end_time = None
    yield


def test_default_mode_passes(validator):
    cmd = json.dumps({"primitive_type": "HOME", "velocity_scale": 0.06})
    valid, reason = validator.validate(cmd)
    assert valid is True
    assert reason == ""


def test_extended_mode_missing_operator_confirm(validator):
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": True,
            "cable_inspection_signed_off_token": "abc123",
            "velocity_scale": 0.06,
        }
    )
    valid, reason = validator.validate(cmd)
    assert valid is False
    assert "operator_confirm_token" in reason


def test_extended_mode_velocity_exceeds_cap(validator):
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": True,
            "cable_inspection_signed_off_token": "abc123",
            "operator_confirm_token": "xyz456",
            "velocity_scale": 0.15,
        }
    )
    valid, reason = validator.validate(cmd)
    assert valid is False
    assert "velocity_scale=0.15 > cap 0.1" in reason


def test_extended_mode_all_preconditions_satisfied(validator):
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": True,
            "cable_inspection_signed_off_token": "abc123",
            "operator_confirm_token": "xyz456",
            "velocity_scale": 0.06,
            "estimated_duration_s": 10,
        }
    )
    valid, reason = validator.validate(cmd)
    assert valid is True
    assert reason == ""


def test_extended_mode_missing_cable_inspection(validator):
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": True,
            "operator_confirm_token": "xyz456",
            "velocity_scale": 0.06,
        }
    )
    valid, reason = validator.validate(cmd)
    assert valid is False
    assert "cable_inspection_signed_off_token" in reason


def test_extended_mode_duration_exceeds_cap(validator):
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": True,
            "cable_inspection_signed_off_token": "abc123",
            "operator_confirm_token": "xyz456",
            "velocity_scale": 0.06,
            "estimated_duration_s": 45,
        }
    )
    valid, reason = validator.validate(cmd)
    assert valid is False
    assert "estimated_duration 45.0s > cap 30.0s" in reason


def test_cooldown_rejects_immediate_retry(validator):
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": True,
            "cable_inspection_signed_off_token": "abc123",
            "operator_confirm_token": "xyz456",
            "velocity_scale": 0.06,
            "estimated_duration_s": 10,
        }
    )
    valid, _ = validator.validate(cmd)
    assert valid is True
    validator.record_extended_run_end()

    valid2, reason2 = validator.validate(cmd)
    assert valid2 is False
    assert "cooldown not elapsed" in reason2


def test_cooldown_allows_after_wait(validator):
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": True,
            "cable_inspection_signed_off_token": "abc123",
            "operator_confirm_token": "xyz456",
            "velocity_scale": 0.06,
            "estimated_duration_s": 10,
        }
    )
    valid, _ = validator.validate(cmd)
    assert valid is True
    validator.record_extended_run_end()

    # Simulate 61s elapsed by manipulating the tracker
    from datetime import datetime, timedelta, timezone

    _extended_runs._last_end_time = datetime.now(timezone.utc) - timedelta(seconds=61)

    valid2, reason2 = validator.validate(cmd)
    assert valid2 is True


def test_extended_mode_no_tiered_config(validator):
    """Extended mode rejected when config has no tiered joint_6_t."""
    validator.safety_rules["operational_joint_limits"]["joint_6_t"] = {
        "min": -3.142,
        "max": 3.142,
    }
    cmd = json.dumps(
        {
            "primitive_type": "PTP",
            "extended_mode": True,
            "velocity_scale": 0.06,
        }
    )
    valid, reason = validator.validate(cmd)
    assert valid is False
    assert "no extended tier" in reason


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
