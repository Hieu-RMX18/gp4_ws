import copy

import pytest
import yaml
from pathlib import Path

from llm_gateway.intent_engine import SemanticValidator


def _load_workspace_bounds() -> dict:
    safety_yaml = (
        Path(__file__).resolve().parents[2] / "safety" / "config" / "safety_rules.yaml"
    )
    safety_rules = yaml.safe_load(safety_yaml.read_text()) or {}
    return safety_rules["workspace_bounds"]


def test_semantic_validator_fallback_bounds_match_safety_rules():
    bounds = _load_workspace_bounds()
    fallback = SemanticValidator._DEFAULT_BOUNDS  # pylint: disable=protected-access
    assert fallback["x"] == pytest.approx((bounds["x_min"], bounds["x_max"]))
    assert fallback["y"] == pytest.approx((bounds["y_min"], bounds["y_max"]))
    assert fallback["z"] == pytest.approx((bounds["z_min"], bounds["z_max"]))


def test_semantic_validator_accepts_in_bounds_lin(
    normalizer, semantic_validator, canonical_command
):
    normalized = normalizer.normalize(canonical_command)

    assert semantic_validator.validate(normalized) is True


def test_semantic_validator_rejects_out_of_bounds_pose(
    normalizer, semantic_validator, canonical_command
):
    invalid = copy.deepcopy(canonical_command)
    x_upper = float(semantic_validator._workspace_bounds["x"][1])  # pylint: disable=protected-access
    invalid["target_pose"]["position"]["x"] = x_upper + 0.01
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

    with pytest.raises(
        ValueError, match="HOME must not include target_pose or joint_target"
    ):
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
    with pytest.raises(
        ValueError, match="at least one delta component must be non-zero"
    ):
        semantic_validator.validate(normalized)


def test_semantic_validator_rejects_move_rel_oversized_delta(
    normalizer, semantic_validator
):
    """MOVE_REL with delta norm above configured safety limit is rejected."""
    # norm = sqrt(0.16^2 + 0.16^2) ≈ 0.2263 > 0.21
    cmd = {
        "primitive_type": "MOVE_REL",
        "delta_x": 0.16,
        "delta_y": 0.16,
        "delta_z": 0.0,
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    with pytest.raises(ValueError, match="delta norm"):
        semantic_validator.validate(normalized)


def test_semantic_validator_rejects_move_rel_unsupported_frame(
    normalizer, semantic_validator
):
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
    }
    with pytest.raises(ValueError, match="MOVE_REL requires delta_y"):
        semantic_validator.validate(cmd)


def test_semantic_validator_rejects_move_rel_with_target_pose(
    normalizer, semantic_validator
):
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
    cmd = {"primitive_type": "SET_SPEED", "velocity_scale": 0.07}
    with pytest.raises(ValueError, match="SET_SPEED.*velocity_scale"):
        semantic_validator.validate(cmd)


def test_set_speed_rejected_below_min(semantic_validator):
    """SET_SPEED with velocity_scale below 0.01 is rejected."""
    cmd = {"primitive_type": "SET_SPEED", "velocity_scale": 0.0}
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
    cmd = {
        "primitive_type": "MOVE_JOINT",
        "joint_index": 0,
        "joint_angle": float("inf"),
    }
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


# ── CIRC tests ──


def test_semantic_validator_accepts_valid_circ(normalizer, semantic_validator):
    cmd = {
        "primitive_type": "CIRC",
        "target_pose": {
            "position": {"x": 0.30, "y": 0.00, "z": 0.40},
            "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
        },
        "waypoints": [
            {
                "position": {"x": 0.32, "y": 0.05, "z": 0.42},
                "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
            }
        ],
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    assert semantic_validator.validate(normalized) is True


def test_semantic_validator_rejects_circ_auxiliary_out_of_workspace(
    normalizer, semantic_validator
):
    cmd = {
        "primitive_type": "CIRC",
        "target_pose": {
            "position": {"x": 0.30, "y": 0.00, "z": 0.40},
            "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
        },
        "waypoints": [
            # x beyond workspace x_max (currently 0.45 from safety_rules)
            {
                "position": {"x": 0.90, "y": 0.00, "z": 0.40},
                "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
            }
        ],
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    with pytest.raises(ValueError, match="CIRC auxiliary waypoint"):
        semantic_validator.validate(normalized)


def test_semantic_validator_rejects_circ_missing_target_pose_msg(semantic_validator):
    # Normalizer would fail first; simulate a command that skipped normalization.
    cmd = {
        "primitive_type": "CIRC",
        "waypoints_msg": [object()],  # presence only; target_pose_msg absent
    }
    with pytest.raises(ValueError, match="CIRC requires target_pose_msg"):
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


# ── BLENDED_SEQUENCE tests (W2.T7) ──


def _make_pose_msg(x, y, z):
    """Create a minimal Pose-like object for semantic validator tests."""

    class _Position:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class _Orientation:
        def __init__(self):
            self.x, self.y, self.z, self.w = 0.0, 0.0, 0.0, 1.0

    class _Pose:
        def __init__(self, pos):
            self.position = pos
            self.orientation = _Orientation()

    return _Pose(_Position(x, y, z))


def test_semantic_validator_accepts_valid_blended_sequence(semantic_validator):
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.30, 0.00, 0.30),
                "blend_radius_m": 0.0,
            },
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.32, 0.02, 0.30),
                "blend_radius_m": 0.008,
            },
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.34, 0.00, 0.30),
                "blend_radius_m": 0.0,
            },
        ],
    }
    assert semantic_validator.validate(cmd) is True


def test_semantic_validator_rejects_blended_sequence_1_step(semantic_validator):
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.30, 0.00, 0.30),
                "blend_radius_m": 0.0,
            },
        ],
    }
    with pytest.raises(ValueError, match="at least 2"):
        semantic_validator.validate(cmd)


def test_semantic_validator_rejects_blended_sequence_first_blend_nonzero(
    semantic_validator,
):
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.30, 0.00, 0.30),
                "blend_radius_m": 0.008,
            },
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.32, 0.02, 0.30),
                "blend_radius_m": 0.0,
            },
        ],
    }
    with pytest.raises(ValueError, match="first step"):
        semantic_validator.validate(cmd)


def test_semantic_validator_rejects_blended_sequence_last_blend_nonzero(
    semantic_validator,
):
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.30, 0.00, 0.30),
                "blend_radius_m": 0.0,
            },
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.32, 0.02, 0.30),
                "blend_radius_m": 0.008,
            },
        ],
    }
    with pytest.raises(ValueError, match="last step"):
        semantic_validator.validate(cmd)


def test_semantic_validator_rejects_blended_sequence_step_out_of_workspace(
    semantic_validator,
):
    cmd = {
        "primitive_type": "BLENDED_SEQUENCE",
        "sequence_steps": [
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.30, 0.00, 0.30),
                "blend_radius_m": 0.0,
            },
            {
                "primitive_type": "LIN",
                "target_pose_msg": _make_pose_msg(0.90, 0.00, 0.30),
                "blend_radius_m": 0.0,
            },
        ],
    }
    with pytest.raises(ValueError, match="BLENDED_SEQUENCE step"):
        semantic_validator.validate(cmd)


# ── CIRC degenerate arc tests (W2.T6) ──


def test_semantic_validator_rejects_circ_degenerate_colinear(
    normalizer, semantic_validator
):
    """CIRC with colinear start, aux, goal should be rejected."""
    cmd = {
        "primitive_type": "CIRC",
        "target_pose": {
            "position": {"x": 0.34, "y": 0.00, "z": 0.30},
            "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
        },
        "waypoints": [
            {
                "position": {"x": 0.32, "y": 0.00, "z": 0.30},
                "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
            }
        ],
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    normalized["start_pose_msg"] = _make_pose_msg(0.30, 0.00, 0.30)
    with pytest.raises(ValueError, match="degenerate CIRC"):
        semantic_validator.validate(normalized)


def test_semantic_validator_accepts_circ_non_degenerate_with_start(
    normalizer, semantic_validator
):
    """CIRC with non-colinear start, aux, goal should pass."""
    cmd = {
        "primitive_type": "CIRC",
        "target_pose": {
            "position": {"x": 0.30, "y": 0.00, "z": 0.40},
            "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
        },
        "waypoints": [
            {
                "position": {"x": 0.32, "y": 0.05, "z": 0.42},
                "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
            }
        ],
        "velocity_scale": 0.06,
    }
    normalized = normalizer.normalize(cmd)
    normalized["start_pose_msg"] = _make_pose_msg(0.28, 0.00, 0.38)
    assert semantic_validator.validate(normalized) is True
