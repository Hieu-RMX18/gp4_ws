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
