"""Tests for safety.policy_loader — single-source policy loading."""

from safety.policy_loader import (
    _FAILSAFE_MOTION_LIMITS,
    get_motion_limits,
    load_safety_rules,
)


def test_load_safety_rules_returns_dict():
    rules = load_safety_rules()
    assert isinstance(rules, dict)


def test_get_motion_limits_from_loaded_rules():
    rules = load_safety_rules()
    limits = get_motion_limits(rules)
    assert "max_velocity_scale" in limits
    assert "max_acceleration_scale" in limits
    assert "max_move_rel_translation" in limits
    assert all(isinstance(v, float) for v in limits.values())


def test_get_motion_limits_failsafe_on_empty():
    """Empty rules dict falls back to fail-safe constants."""
    limits = get_motion_limits({})
    assert limits["max_velocity_scale"] == _FAILSAFE_MOTION_LIMITS["max_velocity_scale"]
    assert (
        limits["max_acceleration_scale"]
        == _FAILSAFE_MOTION_LIMITS["max_acceleration_scale"]
    )
    assert (
        limits["max_move_rel_translation"]
        == _FAILSAFE_MOTION_LIMITS["max_move_rel_translation"]
    )


def test_get_motion_limits_reads_from_motion_limits_key():
    rules = {
        "motion_limits": {"max_velocity_scale": 0.10, "max_acceleration_scale": 0.08}
    }
    limits = get_motion_limits(rules)
    assert limits["max_velocity_scale"] == 0.10
    assert limits["max_acceleration_scale"] == 0.08


def test_get_motion_limits_legacy_fallback():
    """Legacy joint_limits_override key works as fallback."""
    rules = {"joint_limits_override": {"max_velocity_scale": 0.12}}
    limits = get_motion_limits(rules)
    assert limits["max_velocity_scale"] == 0.12
