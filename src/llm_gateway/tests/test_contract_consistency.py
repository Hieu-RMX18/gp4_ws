"""B2: Contract consistency test.

Ensures the public primitive set is identical across:
  - llm_schema.yaml (schema validator)
  - semantic_validator.py (allowed primitives)
  - normalizer.py (planner defaults)
  - prompt_builder.py (system prompt — via semantic intents)
  - intent_router.py (IntentRouter producible primitives)

If any layer drifts, this test fails immediately, preventing
partial contract exposure.
"""

import re
from pathlib import Path

import pytest
import yaml

from llm_gateway.normalizer import Normalizer
from llm_gateway.prompt_builder import FROZEN_SEMANTIC_INTENTS, build_system_prompt
from llm_gateway.semantic_validator import SemanticValidator


# Frozen public primitive set for the current sprint contract.
_FROZEN_PUBLIC_PRIMITIVES = {
    "HOME",
    "PTP",
    "LIN",
    "CIRC",
    "CARTESIAN_PATH",
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

# Mapping from semantic intents to the primitive_type(s) they produce.
# draw_shape and draw_text produce PTP + CARTESIAN_PATH macros; all others are 1:1.
_INTENT_TO_PRIMITIVES = {
    "go_home": {"HOME"},
    "stop": {"STOP"},
    "alarm_reset": {"ALARM_RESET"},
    "get_pose": {"GET_POSE"},
    "set_speed": {"SET_SPEED"},
    "wait": {"WAIT"},
    "move_relative": {"MOVE_REL"},
    "absolute_move_ptp": {"PTP"},
    "absolute_move_lin": {"LIN"},
    "circular_move": {"CIRC"},
    "move_joint": {"MOVE_JOINT"},
    "move_joints": {"MOVE_JOINTS"},
    "io_set": {"IO_SET"},
    "draw_shape": {"PTP", "CARTESIAN_PATH"},  # macro expander
    "draw_text": {"PTP", "CARTESIAN_PATH"},  # macro expander
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


def test_prompt_builder_mentions_all_semantic_intents():
    """The system prompt must mention every frozen semantic intent.

    v2.1 change: prompt outputs Semantic IR with 'intent' field, not
    direct primitive_type. We verify intent names instead of primitive names.
    """
    prompt = build_system_prompt("{}")
    for intent_name in FROZEN_SEMANTIC_INTENTS:
        assert intent_name in prompt, (
            f"Prompt builder does not mention semantic intent '{intent_name}'"
        )


def test_intent_to_primitive_mapping_covers_all_primitives():
    """Every frozen public primitive must be producible by at least one intent.

    Primitives in _DIRECT_ONLY_PRIMITIVES are valid public primitives but only
    reachable via the raw command path (/llm_raw_command), not through a
    semantic intent.  They are excluded from intent-coverage checks.
    """
    # All public primitives are now reachable via a semantic intent.
    _DIRECT_ONLY_PRIMITIVES: set = set()
    producible = set()
    for primitive_set in _INTENT_TO_PRIMITIVES.values():
        producible |= primitive_set
    intent_required = _FROZEN_PUBLIC_PRIMITIVES - _DIRECT_ONLY_PRIMITIVES
    assert producible >= intent_required, (
        f"Intent-to-primitive mapping does not cover all primitives.\n"
        f"  Uncovered: {intent_required - producible}"
    )


def test_intent_to_primitive_mapping_matches_frozen_intents():
    """_INTENT_TO_PRIMITIVES keys must equal FROZEN_SEMANTIC_INTENTS."""
    mapping_intents = set(_INTENT_TO_PRIMITIVES.keys())
    assert mapping_intents == FROZEN_SEMANTIC_INTENTS, (
        f"Intent-to-primitive mapping out of sync with FROZEN_SEMANTIC_INTENTS.\n"
        f"  Extra in mapping: {mapping_intents - FROZEN_SEMANTIC_INTENTS}\n"
        f"  Missing from mapping: {FROZEN_SEMANTIC_INTENTS - mapping_intents}"
    )


def test_intent_router_covers_all_frozen_intents():
    """IntentRouter must handle every intent in FROZEN_SEMANTIC_INTENTS."""
    from llm_gateway.intent_router import IntentRouter
    import inspect

    router = IntentRouter(
        macro_policy_path=str(
            Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml"
        )
    )

    # Extract intent names from _route_single_intent source
    source = inspect.getsource(router._route_single_intent)
    single_intents = set(re.findall(r'intent\s*==\s*"(\w+)"', source))

    # Extract macro intents from route() source
    route_source = inspect.getsource(router.route)
    meta_intents = set(re.findall(r'normalized_intent\s*==\s*"(\w+)"', route_source))
    # "sequence" is a meta-intent, not in FROZEN_SEMANTIC_INTENTS
    router_intents = single_intents | (meta_intents - {"sequence"})

    assert router_intents == FROZEN_SEMANTIC_INTENTS, (
        f"IntentRouter intent coverage does not match FROZEN_SEMANTIC_INTENTS.\n"
        f"  Extra in router: {router_intents - FROZEN_SEMANTIC_INTENTS}\n"
        f"  Missing from router: {FROZEN_SEMANTIC_INTENTS - router_intents}"
    )


def test_no_deprecated_schema_loaded_at_runtime():
    """command_schema.json must not exist (deprecated to .DEPRECATED)."""
    deprecated_path = Path(__file__).resolve().parents[1] / "config" / "command_schema.json"
    assert not deprecated_path.exists(), (
        "command_schema.json should have been renamed to .DEPRECATED"
    )


def test_macro_policy_declares_draw_shape_and_draw_text():
    policy_path = Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml"
    with open(policy_path, "r", encoding="utf-8") as policy_file:
        policy = yaml.safe_load(policy_file)

    macros = policy["macros"]
    assert "draw_shape" in macros
    assert "draw_text" in macros
