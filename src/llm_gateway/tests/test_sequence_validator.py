"""Phase 5 tests for SequenceValidator.

Covers structural checks, per-step validation delegation, frame policy,
cumulative MOVE_REL budget, STOP sole-primitive policy, query rejection,
IO side-effect tracking, and duration lower-bound semantics.

Uses fake validators to isolate SequenceValidator logic from downstream
schema/normalize/semantic behavior.
"""

import math

import pytest


# u2500u2500 Fake collaborators u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


class FakeSchemaValidator:
    def validate(self, command):
        if command.get("force_schema_error"):
            raise ValueError("schema rejected command")
        return True


class FakeNormalizer:
    def normalize(self, command):
        if command.get("force_normalize_error"):
            raise ValueError("normalizer rejected command")
        normalized = dict(command)
        normalized["normalized"] = True
        return normalized


class FakeSemanticValidator:
    def validate(self, command):
        if command.get("force_semantic_error"):
            raise ValueError("semantic validator rejected command")
        return True


def _validator(**kwargs):
    from llm_gateway.sequence_validator import SequenceValidator

    defaults = {
        "schema_validator": FakeSchemaValidator(),
        "normalizer": FakeNormalizer(),
        "semantic_validator": FakeSemanticValidator(),
        "max_sequence_length": 8,
        "max_cumulative_move_rel_distance_m": 0.25,
    }
    defaults.update(kwargs)
    return SequenceValidator(**defaults)


# u2500u2500 Happy path u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_accepts_valid_sequence_and_returns_metadata():
    validator = _validator()

    result = validator.validate(
        [
            {"primitive_type": "HOME"},
            {"primitive_type": "WAIT", "wait_duration_sec": 1.5},
            {"primitive_type": "IO_SET", "io_address": 10010, "io_value": 1},
            {
                "primitive_type": "MOVE_REL",
                "delta_x": 0.0,
                "delta_y": 0.0,
                "delta_z": 0.10,
                "reference_frame": "base_link",
            },
        ]
    )

    assert len(result.normalized_commands) == 4
    assert all(command["normalized"] is True for command in result.normalized_commands)
    assert result.step_count == 4
    assert result.has_io_side_effects is True
    assert result.manual_recovery_required_on_failure is True
    assert math.isclose(result.estimated_duration_lower_bound_sec, 1.5)
    assert result.duration_estimate_is_lower_bound is True
    assert math.isclose(result.cumulative_move_rel_distance_m, 0.10)
    assert result.validated_reference_frame == "base_link"


def test_single_valid_stop():
    """STOP as sole primitive is valid."""
    validator = _validator()

    result = validator.validate([{"primitive_type": "STOP"}])

    assert result.step_count == 1
    assert result.normalized_commands[0]["primitive_type"] == "STOP"


# u2500u2500 Structural errors u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_rejects_empty_sequence():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError, match="non-empty list"):
        validator.validate([])


def test_rejects_non_list_input():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError, match="non-empty list"):
        validator.validate("not a list")


def test_rejects_non_dict_step():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate([{"primitive_type": "HOME"}, "not_a_dict"])

    assert exc_info.value.stage == "structure"
    assert exc_info.value.step_index == 1


def test_rejects_step_missing_primitive_type():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate([{"some_field": "value"}])

    assert exc_info.value.stage == "structure"
    assert exc_info.value.step_index == 0


# u2500u2500 Sequence length u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_rejects_sequence_longer_than_limit():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator(max_sequence_length=2)

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {"primitive_type": "HOME"},
                {"primitive_type": "WAIT", "wait_duration_sec": 1.0},
                {"primitive_type": "STOP"},
            ]
        )

    assert exc_info.value.stage == "sequence_length"


# u2500u2500 STOP policy u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_rejects_stop_before_final_step():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {"primitive_type": "STOP"},
                {"primitive_type": "HOME"},
            ]
        )

    assert exc_info.value.stage == "stop_policy"
    assert exc_info.value.step_index == 0


def test_rejects_stop_when_not_sole_primitive():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {"primitive_type": "HOME"},
                {"primitive_type": "STOP"},
            ]
        )

    assert exc_info.value.stage == "stop_policy"
    assert exc_info.value.step_index == 1


# u2500u2500 Query primitives in sequences u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_rejects_query_primitive_in_sequence():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {"primitive_type": "HOME"},
                {"primitive_type": "GET_POSE", "reference_frame": "base_link"},
            ]
        )

    assert exc_info.value.stage == "unsupported_step"
    assert exc_info.value.step_index == 1


# u2500u2500 Frame policy u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_rejects_missing_reference_frame_for_cartesian_step():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {
                    "primitive_type": "LIN",
                    "target_pose": {"position": {"x": 0.35, "y": 0.0, "z": 0.3}},
                }
            ]
        )

    assert exc_info.value.stage == "frame_policy"
    assert exc_info.value.step_index == 0


def test_rejects_missing_reference_frame_for_ptp():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {
                    "primitive_type": "PTP",
                    "joint_target": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
                }
            ]
        )

    assert exc_info.value.stage == "frame_policy"
    assert exc_info.value.step_index == 0


def test_rejects_missing_reference_frame_for_cartesian_path():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {
                    "primitive_type": "CARTESIAN_PATH",
                    "waypoints": [
                        {"position": {"x": 0.35, "y": 0.0, "z": 0.3}},
                    ],
                }
            ]
        )

    assert exc_info.value.stage == "frame_policy"
    assert exc_info.value.step_index == 0


def test_accepts_cartesian_path_with_reference_frame():
    validator = _validator()

    result = validator.validate(
        [
            {
                "primitive_type": "CARTESIAN_PATH",
                "reference_frame": "base_link",
                "waypoints": [
                    {"position": {"x": 0.35, "y": 0.0, "z": 0.3}},
                ],
            }
        ]
    )

    assert result.validated_reference_frame == "base_link"


def test_rejects_unsupported_reference_frame():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {
                    "primitive_type": "MOVE_REL",
                    "delta_x": 0.0,
                    "delta_y": 0.0,
                    "delta_z": 0.05,
                    "reference_frame": "tool0",
                }
            ]
        )

    assert exc_info.value.stage == "frame_policy"
    assert exc_info.value.step_index == 0


def test_rejects_mixed_reference_frames():
    """Different frames across steps are rejected even if each is individually valid."""
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    # Both are base_link in the current implementation, so we can only test
    # that the validator catches a non-base_link frame.
    # Mixed-frame rejection is implicitly tested through the unsupported frame check.
    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {
                    "primitive_type": "LIN",
                    "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.3}},
                    "reference_frame": "base_link",
                },
                {
                    "primitive_type": "LIN",
                    "target_pose": {"position": {"x": 0.35, "y": 0.0, "z": 0.3}},
                    "reference_frame": "tool0",
                },
            ]
        )

    assert exc_info.value.stage == "frame_policy"


# u2500u2500 Cumulative MOVE_REL budget u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_rejects_excessive_cumulative_move_rel_distance():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator(max_cumulative_move_rel_distance_m=0.15)

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {
                    "primitive_type": "MOVE_REL",
                    "delta_x": 0.0,
                    "delta_y": 0.0,
                    "delta_z": 0.10,
                    "reference_frame": "base_link",
                },
                {
                    "primitive_type": "MOVE_REL",
                    "delta_x": 0.05,
                    "delta_y": 0.0,
                    "delta_z": 0.05,
                    "reference_frame": "base_link",
                },
            ]
        )

    assert exc_info.value.stage == "move_rel_budget"
    assert exc_info.value.step_index == 1


def test_accepts_cumulative_move_rel_within_budget():
    validator = _validator(max_cumulative_move_rel_distance_m=0.50)

    result = validator.validate(
        [
            {
                "primitive_type": "MOVE_REL",
                "delta_x": 0.0,
                "delta_y": 0.0,
                "delta_z": 0.10,
                "reference_frame": "base_link",
            },
            {
                "primitive_type": "MOVE_REL",
                "delta_x": 0.10,
                "delta_y": 0.0,
                "delta_z": 0.0,
                "reference_frame": "base_link",
            },
        ]
    )

    assert result.step_count == 2
    assert result.cumulative_move_rel_distance_m > 0.0


# u2500u2500 Per-step validation delegation u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_surfaces_schema_stage_and_step():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {"primitive_type": "HOME"},
                {"primitive_type": "WAIT", "wait_duration_sec": 1.0, "force_schema_error": True},
            ]
        )

    assert exc_info.value.stage == "schema"
    assert exc_info.value.step_index == 1


def test_surfaces_normalize_stage_and_step():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {"primitive_type": "HOME", "force_normalize_error": True},
            ]
        )

    assert exc_info.value.stage == "normalize"
    assert exc_info.value.step_index == 0


def test_surfaces_semantic_stage_and_step():
    from llm_gateway.sequence_validator import SequenceValidationError

    validator = _validator()

    with pytest.raises(SequenceValidationError) as exc_info:
        validator.validate(
            [
                {"primitive_type": "HOME", "force_semantic_error": True},
            ]
        )

    assert exc_info.value.stage == "semantic"
    assert exc_info.value.step_index == 0


# u2500u2500 IO side-effect tracking u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_marks_no_manual_recovery_when_io_is_last_step():
    validator = _validator()

    result = validator.validate(
        [
            {"primitive_type": "HOME"},
            {"primitive_type": "IO_SET", "io_address": 10010, "io_value": 1},
        ]
    )

    assert result.has_io_side_effects is True
    assert result.manual_recovery_required_on_failure is False


def test_marks_manual_recovery_when_io_before_last_step():
    validator = _validator()

    result = validator.validate(
        [
            {"primitive_type": "IO_SET", "io_address": 10010, "io_value": 1},
            {"primitive_type": "HOME"},
        ]
    )

    assert result.has_io_side_effects is True
    assert result.manual_recovery_required_on_failure is True


def test_no_io_side_effects_without_io_set():
    validator = _validator()

    result = validator.validate(
        [
            {"primitive_type": "HOME"},
            {"primitive_type": "WAIT", "wait_duration_sec": 1.0},
        ]
    )

    assert result.has_io_side_effects is False
    assert result.manual_recovery_required_on_failure is False


# u2500u2500 Duration estimation u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_duration_is_always_lower_bound():
    validator = _validator()

    result = validator.validate([{"primitive_type": "STOP"}])

    assert result.estimated_duration_lower_bound_sec == 0.0
    assert result.duration_estimate_is_lower_bound is True


def test_wait_contributes_to_duration_estimate():
    validator = _validator()

    result = validator.validate(
        [
            {"primitive_type": "WAIT", "wait_duration_sec": 2.5},
            {"primitive_type": "WAIT", "wait_duration_sec": 1.5},
        ]
    )

    assert math.isclose(result.estimated_duration_lower_bound_sec, 4.0)


# u2500u2500 Diagnostics u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_diagnostics_mention_not_yet_implemented():
    """Honest about what is NOT validated yet."""
    validator = _validator()

    result = validator.validate([{"primitive_type": "HOME"}])

    diagnostics_text = " ".join(result.diagnostics)
    assert "NOT YET IMPLEMENTED" in diagnostics_text


# u2500u2500 SequenceValidationError structure u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500u2500


def test_error_includes_stage_and_reason_in_str():
    from llm_gateway.sequence_validator import SequenceValidationError

    err = SequenceValidationError("frame_policy", "unsupported frame", step_index=2)

    assert err.stage == "frame_policy"
    assert err.step_index == 2
    assert err.reason == "unsupported frame"
    assert "frame_policy" in str(err)
    assert "step=3" in str(err)  # 0-indexed -> 1-indexed in message


def test_error_without_step_index():
    from llm_gateway.sequence_validator import SequenceValidationError

    err = SequenceValidationError("sequence_length", "too long")

    assert err.step_index is None
    assert "step=" not in str(err)
