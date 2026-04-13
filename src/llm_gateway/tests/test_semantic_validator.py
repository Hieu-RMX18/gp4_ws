import copy

import pytest


def test_semantic_validator_accepts_in_bounds_lin(normalizer, semantic_validator, canonical_command):
    normalized = normalizer.normalize(canonical_command)

    assert semantic_validator.validate(normalized) is True


def test_semantic_validator_rejects_out_of_bounds_pose(
    normalizer, semantic_validator, canonical_command
):
    invalid = copy.deepcopy(canonical_command)
    # Must exceed safety_rules.yaml x_max (0.38), not hardcoded _DEFAULT_BOUNDS (0.6)
    invalid["target_pose"]["position"]["x"] = 0.39
    normalized = normalizer.normalize(invalid)

    with pytest.raises(ValueError, match="target_pose.position.x"):
        semantic_validator.validate(normalized)


def test_semantic_validator_rejects_home_with_pose(normalizer, semantic_validator):
    normalized = normalizer.normalize(
        {
            "primitive_type": "HOME",
            "target_pose": {
                "position": {"x": 0.3, "y": 0.0, "z": 0.4},
                "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
            },
        }
    )

    with pytest.raises(ValueError, match="HOME must not include target_pose or joint_target"):
        semantic_validator.validate(normalized)


# ── MOVE_REL tests ──


def test_semantic_validator_accepts_valid_move_rel(normalizer, semantic_validator):
    """Valid MOVE_REL with non-zero delta passes validation."""
    cmd = {
        "primitive_type": "MOVE_REL",
        "delta_x": 0.0,
        "delta_y": 0.0,
        "delta_z": 0.03,
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    assert semantic_validator.validate(normalized) is True


def test_semantic_validator_accepts_move_rel_multi_axis(normalizer, semantic_validator):
    """MOVE_REL with multiple non-zero deltas passes."""
    cmd = {
        "primitive_type": "MOVE_REL",
        "delta_x": 0.02,
        "delta_y": 0.0,
        "delta_z": 0.01,
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    assert semantic_validator.validate(normalized) is True


def test_semantic_validator_rejects_move_rel_zero_delta(normalizer, semantic_validator):
    """MOVE_REL with all-zero deltas is rejected."""
    cmd = {
        "primitive_type": "MOVE_REL",
        "delta_x": 0.0,
        "delta_y": 0.0,
        "delta_z": 0.0,
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    with pytest.raises(ValueError, match="at least one delta component must be non-zero"):
        semantic_validator.validate(normalized)


def test_semantic_validator_rejects_move_rel_oversized_delta(normalizer, semantic_validator):
    """MOVE_REL with delta norm > 0.03m is rejected."""
    cmd = {
        "primitive_type": "MOVE_REL",
        "delta_x": 0.03,
        "delta_y": 0.02,
        "delta_z": 0.0,
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    with pytest.raises(ValueError, match="delta norm"):
        semantic_validator.validate(normalized)


def test_semantic_validator_rejects_move_rel_unsupported_frame(normalizer, semantic_validator):
    """MOVE_REL with unsupported reference_frame is rejected."""
    cmd = {
        "primitive_type": "MOVE_REL",
        "delta_x": 0.0,
        "delta_y": 0.0,
        "delta_z": 0.03,
        "reference_frame": "tool0",
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    with pytest.raises(ValueError, match="unsupported reference_frame"):
        semantic_validator.validate(normalized)


def test_semantic_validator_rejects_move_rel_missing_delta(semantic_validator):
    """MOVE_REL with missing delta field is rejected."""
    cmd = {
        "primitive_type": "MOVE_REL",
        "delta_x": 0.0,
        "delta_z": 0.03,
        "velocity_scale": 0.06,
        "acceleration_scale": 0.06,
        "planner_id": "PILZ_LIN",
        "require_approval": True,
    }
    with pytest.raises(ValueError, match="MOVE_REL requires delta_y"):
        semantic_validator.validate(cmd)


def test_semantic_validator_rejects_move_rel_with_target_pose(normalizer, semantic_validator):
    """MOVE_REL must not include target_pose."""
    cmd = {
        "primitive_type": "MOVE_REL",
        "delta_x": 0.0,
        "delta_y": 0.0,
        "delta_z": 0.03,
        "target_pose": {
            "position": {"x": 0.3, "y": 0.0, "z": 0.4},
            "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
        },
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    with pytest.raises(ValueError, match="MOVE_REL must not include target_pose"):
        semantic_validator.validate(normalized)


# ── SET_SPEED tests ──


def test_set_speed_valid_at_015(semantic_validator):
    """SET_SPEED with valid velocity_scale 0.06 passes."""
    cmd = {"primitive_type": "SET_SPEED", "velocity_scale": 0.06}
    assert semantic_validator.validate(cmd) is True


def test_set_speed_rejected_above_030(semantic_validator):
    """SET_SPEED with velocity_scale above 0.06 is rejected."""
    cmd = {"primitive_type": "SET_SPEED", "velocity_scale": 0.08}
    with pytest.raises(ValueError, match="SET_SPEED.*velocity_scale"):
        semantic_validator.validate(cmd)


def test_set_speed_rejected_below_min(semantic_validator):
    """SET_SPEED with velocity_scale below 0.05 is rejected."""
    cmd = {"primitive_type": "SET_SPEED", "velocity_scale": 0.01}
    with pytest.raises(ValueError, match="SET_SPEED.*velocity_scale"):
        semantic_validator.validate(cmd)


# ── WAIT tests ──


def test_wait_valid_at_2_5(semantic_validator):
    """WAIT with duration 2.5s passes."""
    cmd = {"primitive_type": "WAIT", "wait_duration_sec": 2.5}
    assert semantic_validator.validate(cmd) is True


def test_wait_valid_at_zero(semantic_validator):
    """WAIT with duration 0.0 passes (immediate return)."""
    cmd = {"primitive_type": "WAIT", "wait_duration_sec": 0.0}
    assert semantic_validator.validate(cmd) is True


def test_wait_rejected_negative(semantic_validator):
    """WAIT with negative duration is rejected."""
    cmd = {"primitive_type": "WAIT", "wait_duration_sec": -1.0}
    with pytest.raises(ValueError, match="wait_duration_sec must be >= 0"):
        semantic_validator.validate(cmd)


# ── STOP tests ──


def test_stop_valid_empty(semantic_validator):
    """STOP with no extra fields passes."""
    cmd = {"primitive_type": "STOP"}
    assert semantic_validator.validate(cmd) is True


def test_stop_rejected_with_target_pose(normalizer, semantic_validator):
    """STOP must not include target_pose."""
    cmd = {
        "primitive_type": "STOP",
        "target_pose": {
            "position": {"x": 0.3, "y": 0.0, "z": 0.4},
            "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
        },
    }
    normalized = normalizer.normalize(cmd)
    with pytest.raises(ValueError, match="STOP must not include target_pose"):
        semantic_validator.validate(normalized)


# ── ALARM_RESET tests ──


def test_alarm_reset_valid_no_velocity(semantic_validator):
    """ALARM_RESET passes with no velocity_scale."""
    cmd = {"primitive_type": "ALARM_RESET"}
    assert semantic_validator.validate(cmd) is True


# ── IO_SET tests ──


def test_io_set_valid_address_value_1(semantic_validator):
    """IO_SET with valid address and value=1 passes."""
    cmd = {"primitive_type": "IO_SET", "io_address": 10010, "io_value": 1}
    assert semantic_validator.validate(cmd) is True


def test_io_set_valid_address_value_0(semantic_validator):
    """IO_SET with valid address and value=0 passes."""
    cmd = {"primitive_type": "IO_SET", "io_address": 10010, "io_value": 0}
    assert semantic_validator.validate(cmd) is True


def test_io_set_reject_invalid_value(semantic_validator):
    """IO_SET with invalid io_value is rejected."""
    cmd = {"primitive_type": "IO_SET", "io_address": 10010, "io_value": 5}
    with pytest.raises(ValueError, match="io_value must be 0 or 1"):
        semantic_validator.validate(cmd)


def test_io_set_reject_missing_address(semantic_validator):
    """IO_SET without io_address is rejected."""
    cmd = {"primitive_type": "IO_SET", "io_value": 1}
    with pytest.raises(ValueError, match="IO_SET requires io_address"):
        semantic_validator.validate(cmd)


# ── MOVE_JOINT tests ──


def test_move_joint_valid_in_range(semantic_validator):
    """MOVE_JOINT with valid joint_index [0..5] and finite angle passes."""
    cmd = {"primitive_type": "MOVE_JOINT", "joint_index": 2, "joint_angle": 0.5}
    assert semantic_validator.validate(cmd) is True


def test_move_joint_reject_out_of_range_index(semantic_validator):
    """MOVE_JOINT with joint_index >= 6 is rejected."""
    cmd = {"primitive_type": "MOVE_JOINT", "joint_index": 6, "joint_angle": 0.5}
    with pytest.raises(ValueError, match="joint_index 6 out of range"):
        semantic_validator.validate(cmd)


def test_move_joint_reject_negative_index(semantic_validator):
    """MOVE_JOINT with negative joint_index is rejected."""
    cmd = {"primitive_type": "MOVE_JOINT", "joint_index": -1, "joint_angle": 0.5}
    with pytest.raises(ValueError, match="joint_index -1 out of range"):
        semantic_validator.validate(cmd)


def test_move_joint_reject_infinite_angle(semantic_validator):
    """MOVE_JOINT with infinite angle is rejected."""
    cmd = {"primitive_type": "MOVE_JOINT", "joint_index": 0, "joint_angle": float("inf")}
    with pytest.raises(ValueError, match="joint_angle must be a finite number"):
        semantic_validator.validate(cmd)


# ── MOVE_JOINTS tests ──


def test_move_joints_valid_6_values(semantic_validator):
    """MOVE_JOINTS with 6-value joint_target passes."""
    cmd = {
        "primitive_type": "MOVE_JOINTS",
        "joint_target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
    assert semantic_validator.validate(cmd) is True


def test_move_joints_reject_wrong_length(semantic_validator):
    """MOVE_JOINTS with wrong-length joint_target is rejected."""
    cmd = {
        "primitive_type": "MOVE_JOINTS",
        "joint_target": [0.0, 0.0, 0.0],
    }
    with pytest.raises(ValueError, match="joint_target must have exactly 6 elements"):
        semantic_validator.validate(cmd)


def test_move_joints_reject_missing_joint_target(semantic_validator):
    """MOVE_JOINTS without joint_target is rejected."""
    cmd = {"primitive_type": "MOVE_JOINTS"}
    with pytest.raises(ValueError, match="MOVE_JOINTS requires joint_target"):
        semantic_validator.validate(cmd)


def test_semantic_validator_accepts_cartesian_path(normalizer, semantic_validator):
    normalized = normalizer.normalize(
        {
            "primitive_type": "CARTESIAN_PATH",
            "reference_frame": "base_link",
            "waypoints": [
                {"position": {"x": 0.30, "y": 0.00, "z": 0.30}},
                {"position": {"x": 0.32, "y": 0.02, "z": 0.30}},
            ],
        }
    )

    assert semantic_validator.validate(normalized) is True
