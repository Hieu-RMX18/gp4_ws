"""B2: Contract consistency test.

Ensures the public primitive set is identical across:
  - llm_schema.yaml (schema validator)
  - semantic_validator.py (allowed primitives)
  - normalizer.py (planner defaults)
  - prompt_builder.py (system prompt)

If any layer drifts, this test fails immediately, preventing
partial contract exposure.
"""

from pathlib import Path

import pytest
import yaml

from llm_gateway.normalizer import Normalizer
from llm_gateway.prompt_builder import build_system_prompt
from llm_gateway.semantic_validator import SemanticValidator


# Frozen public primitive set for the current sprint contract.
_FROZEN_PUBLIC_PRIMITIVES = {
    "HOME",
    "PTP",
    "LIN",
    "MOVE_REL",
    "GET_POSE",
    "SET_SPEED",
    "WAIT",
    "STOP",
    "MOVE_JOINT",
    "MOVE_JOINTS",
    "IO_SET",
    "ALARM_RESET",
}
# These primitives are operational/query commands and do not need planner defaults.
_NON_PLANNING_PRIMITIVES = {
    "GET_POSE",
    "SET_SPEED",
    "WAIT",
    "STOP",
    "IO_SET",
    "ALARM_RESET",
}


@pytest.fixture(scope="module")
def schema_primitives() -> set:
    """Extract primitive_type enum from llm_schema.yaml."""
    schema_path = Path(__file__).resolve().parents[1] / "config" / "llm_schema.yaml"
    with open(schema_path, "r", encoding="utf-8") as f:
        schema = yaml.safe_load(f)
    return set(schema["properties"]["primitive_type"]["enum"])


def test_schema_matches_frozen_shortlist(schema_primitives):
    """llm_schema.yaml primitive_type enum must equal frozen shortlist."""
    assert schema_primitives == _FROZEN_PUBLIC_PRIMITIVES, (
        f"Schema drift: schema has {schema_primitives}, "
        f"frozen shortlist has {_FROZEN_PUBLIC_PRIMITIVES}"
    )


def test_semantic_validator_matches_frozen_shortlist():
    """SemanticValidator._ALLOWED_PRIMITIVES must equal frozen shortlist."""
    assert SemanticValidator._ALLOWED_PRIMITIVES == _FROZEN_PUBLIC_PRIMITIVES, (
        f"SemanticValidator drift: has {SemanticValidator._ALLOWED_PRIMITIVES}, "
        f"frozen shortlist has {_FROZEN_PUBLIC_PRIMITIVES}"
    )


def test_normalizer_planner_defaults_cover_public_primitives():
    """Normalizer._PLANNER_DEFAULTS must have a default for each motion primitive."""
    defaults = Normalizer._PLANNER_DEFAULTS
    motion_primitives = _FROZEN_PUBLIC_PRIMITIVES - _NON_PLANNING_PRIMITIVES
    for prim in motion_primitives:
        assert prim in defaults, (
            f"Normalizer missing planner default for motion primitive '{prim}'"
        )


def test_prompt_builder_mentions_all_public_primitives():
    """The system prompt must mention every frozen public primitive."""
    prompt = build_system_prompt("{}")
    for prim in _FROZEN_PUBLIC_PRIMITIVES:
        assert prim in prompt, (
            f"Prompt builder does not mention public primitive '{prim}'"
        )


def test_no_deprecated_schema_loaded_at_runtime():
    """command_schema.json must not exist (deprecated to .DEPRECATED)."""
    deprecated_path = Path(__file__).resolve().parents[1] / "config" / "command_schema.json"
    assert not deprecated_path.exists(), (
        "command_schema.json should have been renamed to .DEPRECATED"
    )
