import copy

import pytest


def test_schema_valid_payload(validator, canonical_command):
    assert validator.validate(canonical_command) is True


def test_schema_rejects_hallucinated_argument(validator, canonical_command):
    invalid = copy.deepcopy(canonical_command)
    invalid["hallucinated_field"] = "must_not_exist"
    with pytest.raises(ValueError, match="Additional properties"):
        validator.validate(invalid)


def test_schema_rejects_missing_required_pose_for_lin(validator, canonical_command):
    invalid = copy.deepcopy(canonical_command)
    del invalid["target_pose"]
    with pytest.raises(ValueError, match="required property"):
        validator.validate(invalid)


def test_schema_accepts_cartesian_path_payload(validator):
    assert validator.validate(
        {
            "primitive_type": "CARTESIAN_PATH",
            "reference_frame": "base_link",
            "waypoints": [
                {"position": {"x": 0.30, "y": 0.00, "z": 0.30}},
                {"position": {"x": 0.32, "y": 0.02, "z": 0.30}},
            ],
        }
    ) is True
