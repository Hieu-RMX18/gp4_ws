import copy

import pytest


def test_semantic_validator_accepts_in_bounds_lin(normalizer, semantic_validator, canonical_command):
    normalized = normalizer.normalize(canonical_command)

    assert semantic_validator.validate(normalized) is True


def test_semantic_validator_rejects_out_of_bounds_pose(
    normalizer, semantic_validator, canonical_command
):
    invalid = copy.deepcopy(canonical_command)
    invalid["target_pose"]["position"]["x"] = 0.75
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
