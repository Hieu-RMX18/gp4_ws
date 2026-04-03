import copy

import pytest


def test_schema_valid_payload(validator, canonical_function_call):
    assert validator.validate(canonical_function_call) is True


def test_schema_rejects_hallucinated_argument(validator, canonical_function_call):
    invalid = copy.deepcopy(canonical_function_call)
    invalid["arguments"]["hallucinated_field"] = "must_not_exist"
    with pytest.raises(ValueError, match="Additional properties"):
        validator.validate(invalid)


def test_schema_rejects_missing_required_pose_for_lin(validator, canonical_function_call):
    invalid = copy.deepcopy(canonical_function_call)
    del invalid["arguments"]["target_pose"]
    with pytest.raises(ValueError, match="required property"):
        validator.validate(invalid)
