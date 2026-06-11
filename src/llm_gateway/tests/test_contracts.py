"""Consolidated tests; original source sections are marked below."""



# ---- test_contract_consistency.py ----
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

from llm_gateway.factory_task import Normalizer
from llm_gateway.task_planner import (
    FROZEN_SEMANTIC_INTENTS,
    FROZEN_TOP_LEVEL_OUTPUT_INTENTS,
    build_system_prompt,
)
from llm_gateway.factory_task import SchemaValidator
from llm_gateway.factory_task import SemanticValidator


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
    "BLENDED_SEQUENCE",
    "MACRO",
}
# These primitives are operational/query commands and do not need planner defaults.
_NON_PLANNING_PRIMITIVES = {
    "GET_POSE",
    "SET_SPEED",
    "WAIT",
    "STOP",
    "IO_SET",
    "ALARM_RESET",
    "MACRO",
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
    "move_named_pose": {"PTP"},
    "absolute_move_lin": {"LIN"},
    "circular_move": {"CIRC"},
    "move_joint": {"MOVE_JOINT"},
    "move_joint_delta": {"MOVE_JOINT"},
    "move_joints": {"MOVE_JOINTS"},
    "io_set": {"IO_SET"},
    "draw_shape": {"PTP", "CARTESIAN_PATH", "BLENDED_SEQUENCE", "MACRO"},
    "draw_text": {"PTP", "CARTESIAN_PATH", "BLENDED_SEQUENCE", "MACRO"},
    "return_to_start": {"MOVE_JOINTS"},
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
        assert (
            prim in defaults
        ), f"Normalizer missing planner default for motion primitive '{prim}'"


def test_prompt_builder_uses_factory_task_contract_not_semantic_ir():
    """The LLM-facing prompt must request FactoryTask, not frozen Semantic IR."""
    prompt = build_system_prompt("{}")
    assert "FactoryTask" in prompt
    assert '"task_type": "factory_task"' in prompt
    assert "Semantic IR (normal path)" not in prompt
    assert '"intent":"sequence"' not in prompt


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
    from llm_gateway.factory_task import IntentRouter
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


def test_top_level_output_intents_include_sequence():
    assert FROZEN_TOP_LEVEL_OUTPUT_INTENTS == (FROZEN_SEMANTIC_INTENTS | {"sequence"})


def test_no_deprecated_schema_loaded_at_runtime():
    """command_schema.json must not exist (legacy, removal_date=2026-04-01)."""
    deprecated_path = (
        Path(__file__).resolve().parents[1] / "config" / "command_schema.json"
    )
    assert (
        not deprecated_path.exists()
    ), "command_schema.json should have been removed (removal_date=2026-04-01)"


def test_macro_policy_declares_draw_shape_and_draw_text():
    policy_path = Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml"
    with open(policy_path, "r", encoding="utf-8") as policy_file:
        policy = yaml.safe_load(policy_file)

    macros = policy["macros"]
    assert "draw_shape" in macros
    assert "draw_text" in macros


def test_schema_declares_explicit_unit_hints():
    schema_path = Path(__file__).resolve().parents[1] / "config" / "llm_schema.yaml"
    with open(schema_path, "r", encoding="utf-8") as schema_file:
        schema = yaml.safe_load(schema_file)

    assert schema["properties"]["linear_unit"]["enum"] == ["m", "cm", "mm"]
    assert schema["properties"]["angular_unit"]["enum"] == ["rad", "deg"]


def test_router_output_with_explicit_units_survives_schema_and_normalizer():
    from llm_gateway.factory_task import IntentRouter

    router = IntentRouter()
    routed = router.route(
        {
            "intent": "move_relative",
            "delta": {"x": 0.0, "y": 0.0, "z": 5.0},
            "linear_unit": "cm",
            "reference_frame": "base_link",
        }
    ).commands[0]

    SchemaValidator().validate(routed)
    normalized = Normalizer().normalize(routed)

    assert normalized["primitive_type"] == "MOVE_REL"
    assert normalized["delta_z"] == pytest.approx(0.05)
    assert "linear_unit" not in normalized


# ---- test_schema_validator.py ----
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
    assert (
        validator.validate(
            {
                "primitive_type": "CARTESIAN_PATH",
                "reference_frame": "base_link",
                "waypoints": [
                    {"position": {"x": 0.30, "y": 0.00, "z": 0.30}},
                    {"position": {"x": 0.32, "y": 0.02, "z": 0.30}},
                ],
            }
        )
        is True
    )


def test_schema_accepts_blended_sequence_3_steps(validator):
    assert (
        validator.validate(
            {
                "primitive_type": "BLENDED_SEQUENCE",
                "reference_frame": "base_link",
                "sequence_steps": [
                    {
                        "primitive_type": "LIN",
                        "target_pose": {"position": {"x": 0.30, "y": 0.00, "z": 0.30}},
                        "blend_radius_m": 0.0,
                        "velocity_scale": 0.06,
                    },
                    {
                        "primitive_type": "LIN",
                        "target_pose": {"position": {"x": 0.32, "y": 0.02, "z": 0.30}},
                        "blend_radius_m": 0.008,
                        "velocity_scale": 0.06,
                    },
                    {
                        "primitive_type": "LIN",
                        "target_pose": {"position": {"x": 0.34, "y": 0.00, "z": 0.30}},
                        "blend_radius_m": 0.0,
                        "velocity_scale": 0.06,
                    },
                ],
            }
        )
        is True
    )


def test_schema_rejects_blended_sequence_1_step(validator):
    with pytest.raises(ValueError):
        validator.validate(
            {
                "primitive_type": "BLENDED_SEQUENCE",
                "reference_frame": "base_link",
                "sequence_steps": [
                    {
                        "primitive_type": "LIN",
                        "target_pose": {"position": {"x": 0.30, "y": 0.00, "z": 0.30}},
                        "blend_radius_m": 0.0,
                    },
                ],
            }
        )


def test_schema_rejects_blended_sequence_missing_steps(validator):
    with pytest.raises(ValueError):
        validator.validate(
            {
                "primitive_type": "BLENDED_SEQUENCE",
                "reference_frame": "base_link",
            }
        )


def test_schema_rejects_blended_sequence_blend_radius_over_max(validator):
    with pytest.raises(ValueError):
        validator.validate(
            {
                "primitive_type": "BLENDED_SEQUENCE",
                "sequence_steps": [
                    {
                        "primitive_type": "LIN",
                        "target_pose": {"position": {"x": 0.30, "y": 0.00, "z": 0.30}},
                        "blend_radius_m": 0.0,
                    },
                    {
                        "primitive_type": "LIN",
                        "target_pose": {"position": {"x": 0.32, "y": 0.02, "z": 0.30}},
                        "blend_radius_m": 0.020,
                    },
                ],
            }
        )


# ---- test_semantic_ir_contract.py ----
"""Tests for the strict Semantic IR contract gate."""



from llm_gateway.semantic_ir_contract import validate_semantic_ir_contract


class TestAcceptsValidSemanticIR:
    def test_go_home(self):
        result = validate_semantic_ir_contract({"intent": "go_home"})
        assert result.valid is True
        assert result.reason == ""

    def test_sequence(self):
        result = validate_semantic_ir_contract(
            {"intent": "sequence", "steps": [{"intent": "go_home"}]}
        )
        assert result.valid is True

    def test_return_to_start(self):
        result = validate_semantic_ir_contract(
            {"intent": "sequence", "steps": [{"intent": "return_to_start"}]}
        )
        assert result.valid is True

    def test_factory_task_runtime_sentinel(self):
        result = validate_semantic_ir_contract(
            {
                "intent": "factory_task_runtime",
                "_factory_task_runtime": True,
                "metadata": {
                    "runtime_plan": {"type": "fallback", "children": []},
                    "factory_task": {"task_id": "fallback-home"},
                },
            }
        )
        assert result.valid is True


class TestRejectsPrimitiveTypeLeakage:
    def test_rejects_primitive_type_field(self):
        result = validate_semantic_ir_contract(
            {"intent": "go_home", "primitive_type": "HOME"}
        )
        assert result.valid is False
        assert "primitive_type" in result.reason
        assert "hint" in result.hint.lower() or "backward" in result.hint.lower()

    def test_rejects_primitive_only_payload(self):
        result = validate_semantic_ir_contract({"primitive_type": "HOME"})
        assert result.valid is False
        assert "primitive_type" in result.reason

    def test_rejects_primitive_type_inside_sequence_step(self):
        result = validate_semantic_ir_contract(
            {"intent": "sequence", "steps": [{"primitive_type": "HOME"}]}
        )
        assert result.valid is False
        assert "$.steps[0].primitive_type" in result.reason
        assert "primitive_type" in result.reason


class TestRejectsRawTextLeakage:
    def test_rejects_raw_text_field(self):
        result = validate_semantic_ir_contract(
            {"intent": "go_home", "raw_text": "go home"}
        )
        assert result.valid is False
        assert "raw_text" in result.reason

    def test_rejects_raw_text_inside_sequence_step(self):
        result = validate_semantic_ir_contract(
            {
                "intent": "sequence",
                "steps": [{"intent": "go_home", "raw_text": "go home"}],
            }
        )
        assert result.valid is False
        assert "$.steps[0].raw_text" in result.reason
        assert "raw_text" in result.reason


class TestRejectsUnknownIntent:
    def test_rejects_fly_to_moon(self):
        result = validate_semantic_ir_contract({"intent": "fly_to_moon"})
        assert result.valid is False
        assert "Unsupported semantic intent" in result.reason
        assert "fly_to_moon" in result.reason
        assert "go_home" in result.hint

    def test_rejects_empty_intent(self):
        result = validate_semantic_ir_contract({"intent": ""})
        assert result.valid is False
        assert "non-empty 'intent' field" in result.reason

    def test_rejects_missing_intent(self):
        result = validate_semantic_ir_contract({"delta": {"x": 0.1}})
        assert result.valid is False
        assert "non-empty 'intent' field" in result.reason

    def test_rejects_top_level_return_to_start(self):
        result = validate_semantic_ir_contract({"intent": "return_to_start"})
        assert result.valid is False
        assert "only valid inside a sequence" in result.reason

    def test_rejects_bare_factory_task_runtime_sentinel(self):
        result = validate_semantic_ir_contract({"intent": "factory_task_runtime"})
        assert result.valid is False
        assert "Unsupported semantic intent" in result.reason


class TestAcceptsErrorPayloads:
    def test_missing_slot_error(self):
        result = validate_semantic_ir_contract(
            {
                "error": "MISSING_SLOT",
                "intent": "move_relative",
                "missing_fields": ["delta"],
            }
        )
        assert result.valid is True

    def test_unsupported_or_ambiguous_error(self):
        result = validate_semantic_ir_contract(
            {"error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND"}
        )
        assert result.valid is True

    def test_unsupported_error_with_message_and_hint(self):
        result = validate_semantic_ir_contract(
            {
                "error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND",
                "message": "planner returned neither FactoryTask nor error",
                "hint": "try again",
            }
        )
        assert result.valid is True


class TestRejectsNonDictPayload:
    def test_rejects_string(self):
        result = validate_semantic_ir_contract("go home")
        assert result.valid is False
        assert "JSON object" in result.reason

    def test_rejects_list(self):
        result = validate_semantic_ir_contract([{"intent": "go_home"}])
        assert result.valid is False

    def test_rejects_none(self):
        result = validate_semantic_ir_contract(None)
        assert result.valid is False
