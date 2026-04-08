"""Prompt contract tests for v2.1 Semantic IR prompt builder.

Validates:
  1. Prompt mentions every frozen semantic intent name
  2. Prompt uses correct IR field name ("intent", not "intent_type")
  3. Prompt uses correct slot names matching IntentRouter expectations
  4. Prompt does NOT mention direct primitive_type output format
  5. Prompt includes all three output format options (A, B, C)
  6. Prompt includes workspace, unit, and velocity constraints
  7. FROZEN_SEMANTIC_INTENTS set matches IntentRouter coverage
  8. draw_shape is marked sim-only in the prompt
  9. Absolute motion does not default to tool-down
  10. Frozen intent list is exactly correct
"""

import re
from pathlib import Path

import pytest

from llm_gateway.prompt_builder import FROZEN_SEMANTIC_INTENTS, build_system_prompt


@pytest.fixture(scope="module")
def prompt() -> str:
    """Build the system prompt with a placeholder schema."""
    return build_system_prompt("{}")


# ── 1. Frozen intent list is exactly correct ──────────────────────────────────

_EXPECTED_INTENTS = frozenset({
    "go_home",
    "stop",
    "alarm_reset",
    "get_pose",
    "set_speed",
    "wait",
    "move_relative",
    "absolute_move_ptp",
    "absolute_move_lin",
    "move_joint",
    "move_joints",
    "io_set",
    "draw_shape",
    "draw_text",
})


def test_frozen_intent_list_matches_expected():
    """FROZEN_SEMANTIC_INTENTS must equal the expected set exactly."""
    assert FROZEN_SEMANTIC_INTENTS == _EXPECTED_INTENTS, (
        f"Frozen intents drifted.\n"
        f"  Extra: {FROZEN_SEMANTIC_INTENTS - _EXPECTED_INTENTS}\n"
        f"  Missing: {_EXPECTED_INTENTS - FROZEN_SEMANTIC_INTENTS}"
    )


# ── 2. Prompt mentions every frozen intent ────────────────────────────────────

@pytest.mark.parametrize("intent_name", sorted(_EXPECTED_INTENTS))
def test_prompt_mentions_intent(prompt: str, intent_name: str):
    """Every frozen semantic intent must appear in the prompt text."""
    assert intent_name in prompt, (
        f"Prompt does not mention semantic intent '{intent_name}'"
    )


# ── 3. Prompt uses "intent" field, not "intent_type" ─────────────────────────

def test_prompt_uses_intent_field_not_intent_type(prompt: str):
    """Prompt IR examples must use 'intent', matching IntentRouter's field name."""
    # The prompt should contain the pattern {"intent": everywhere
    assert '"intent":' in prompt, "Prompt does not contain '\"intent\":'"
    # The prompt must NOT use intent_type as the IR key
    # (it may appear in descriptive text but not in JSON examples)
    intent_type_in_json = re.findall(r'\{"intent_type":', prompt)
    assert not intent_type_in_json, (
        f"Prompt uses 'intent_type' in JSON output examples — "
        f"should use 'intent' to match IntentRouter"
    )


# ── 4. Prompt does NOT instruct direct primitive_type output ──────────────────

def test_prompt_does_not_instruct_primitive_type_output(prompt: str):
    """Normal-path prompt must not tell LLM to output primitive_type directly.

    The primitive_type contract is internal to the pipeline. The LLM outputs
    Semantic IR with 'intent' field; IntentRouter converts to primitives.
    The prompt may mention primitive_type in the schema reference section
    but must not instruct the LLM to output it.
    """
    # Check that the output format section (A/B/C) does not mention primitive_type
    output_section_match = re.search(
        r"OUTPUT FORMAT.*?AVAILABLE INTENTS", prompt, re.DOTALL
    )
    assert output_section_match is not None, "Could not find OUTPUT FORMAT section"
    output_section = output_section_match.group(0)
    assert "primitive_type" not in output_section, (
        "OUTPUT FORMAT section mentions 'primitive_type' — LLM should output "
        "'intent', not 'primitive_type'"
    )


# ── 5. Prompt includes all three output formats ──────────────────────────────

def test_prompt_includes_semantic_ir_format(prompt: str):
    assert '"intent":' in prompt

def test_prompt_includes_missing_slot_format(prompt: str):
    assert "MISSING_SLOT" in prompt

def test_prompt_includes_unsupported_error_format(prompt: str):
    assert "UNSUPPORTED_OR_AMBIGUOUS_COMMAND" in prompt


# ── 6. Prompt includes workspace and velocity constraints ─────────────────────

def test_prompt_includes_workspace_limits(prompt: str):
    assert "0.0" in prompt and "0.6" in prompt, "Workspace x-limits missing"
    assert "-0.3" in prompt and "0.3" in prompt, "Workspace y-limits missing"
    assert "0.2" in prompt, "Workspace z-min missing"

def test_prompt_includes_velocity_scale_range(prompt: str):
    assert "0.05" in prompt and "0.30" in prompt, "Velocity scale range missing"

def test_prompt_includes_unit_conversions(prompt: str):
    assert "0.01" in prompt or "cm" in prompt, "Unit conversion rules missing"


# ── 7. Prompt mentions correct slot names for key intents ─────────────────────

def test_prompt_move_relative_uses_delta_object(prompt: str):
    """move_relative must use delta object with x/y/z, not axis/direction/distance_m."""
    assert '"delta":' in prompt, (
        "move_relative must use 'delta' slot matching IntentRouter._route_move_relative"
    )


def test_prompt_absolute_move_uses_target_pose(prompt: str):
    """absolute_move_ptp/lin must use target_pose.position, not target_x/y/z."""
    assert '"target_pose":' in prompt, (
        "absolute_move must use 'target_pose' slot matching IntentRouter._route_absolute_move"
    )


def test_prompt_wait_uses_wait_duration_sec(prompt: str):
    """wait intent must use wait_duration_sec slot, not duration_sec."""
    assert "wait_duration_sec" in prompt, (
        "wait must use 'wait_duration_sec' slot matching IntentRouter._route_single_intent"
    )


def test_prompt_move_joints_uses_joint_target(prompt: str):
    """move_joints must use joint_target slot, not joint_values."""
    assert "joint_target" in prompt, (
        "move_joints must use 'joint_target' slot matching IntentRouter._route_move_joints"
    )


# ── 8. draw_shape is sim-only ─────────────────────────────────────────────────

def test_prompt_draw_shape_marked_sim_only(prompt: str):
    """draw_shape description must contain sim-only marker per v2.1 policy."""
    # Find the draw_shape section
    draw_section_match = re.search(
        r"draw_shape.*?═{10,}", prompt, re.DOTALL
    )
    assert draw_section_match is not None, "Could not find draw_shape section"
    draw_section = draw_section_match.group(0)
    assert "sim" in draw_section.lower() or "SIM" in draw_section, (
        "draw_shape section must mention sim-only status"
    )


def test_prompt_draw_text_marked_sim_only(prompt: str):
    draw_text_section_match = re.search(
        r"draw_text.*?═{10,}", prompt, re.DOTALL
    )
    assert draw_text_section_match is not None, "Could not find draw_text section"
    draw_text_section = draw_text_section_match.group(0)
    assert "sim" in draw_text_section.lower() or "SIM" in draw_text_section, (
        "draw_text section must mention sim-only status"
    )


# ── 9. Absolute motion does not default to tool-down ──────────────────────────

def test_prompt_absolute_move_no_default_tool_down(prompt: str):
    """v2.1 correction: generic absolute motions must NOT default to tool-down.

    The prompt should instruct the LLM to omit orientation unless the user
    explicitly requests it. The keep_current_orientation policy handles the rest.
    """
    # Find the absolute_move_ptp section
    ptp_section_match = re.search(
        r"absolute_move_ptp.*?absolute_move_lin",
        prompt,
        re.DOTALL,
    )
    assert ptp_section_match is not None, "Could not find absolute_move_ptp section"
    ptp_section = ptp_section_match.group(0)
    # Should mention keeping current orientation, not defaulting to tool-down
    assert "keep_current_orientation" in ptp_section or "OMIT orientation" in ptp_section, (
        "absolute_move_ptp section should mention keep_current_orientation or "
        "instruct to omit orientation, not default to tool-down"
    )
    # The few-shot example for generic PTP should NOT include orientation_preset
    ptp_example_match = re.search(
        r'"intent":\s*"absolute_move_ptp"[^}]+}[^}]*}',
        prompt,
    )
    assert ptp_example_match is not None, "Could not find absolute_move_ptp example"
    # First example (without orientation keyword) should not have orientation_preset
    generic_example = re.search(
        r'User:.*?move to x=0\.3 y=0 z=0\.4"\s*\n→\s*(\{[^→]+)',
        prompt,
        re.DOTALL,
    )
    if generic_example:
        assert "orientation_preset" not in generic_example.group(1), (
            "Generic PTP example must NOT include orientation_preset"
        )


# ── 10. FROZEN_SEMANTIC_INTENTS matches IntentRouter coverage ─────────────────

def test_frozen_intents_match_intent_router():
    """FROZEN_SEMANTIC_INTENTS must cover exactly the intents in IntentRouter."""
    from llm_gateway.intent_router import IntentRouter

    router = IntentRouter(
        macro_policy_path=str(
            Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml"
        )
    )

    # Extract intent names from _route_single_intent by examining the source code.
    # This is fragile but is the ground-truth contract test.
    import inspect
    source = inspect.getsource(router._route_single_intent)
    # Pattern: if intent == "<name>":
    router_intents = set(re.findall(r'intent\s*==\s*"(\w+)"', source))

    # Also check draw_shape and sequence in the route() method
    route_source = inspect.getsource(router.route)
    meta_intents = set(re.findall(r'normalized_intent\s*==\s*"(\w+)"', route_source))
    # sequence is meta-only, not in FROZEN_SEMANTIC_INTENTS
    router_intents |= (meta_intents - {"sequence"})

    assert FROZEN_SEMANTIC_INTENTS == router_intents, (
        f"FROZEN_SEMANTIC_INTENTS out of sync with IntentRouter.\n"
        f"  Extra in frozen: {FROZEN_SEMANTIC_INTENTS - router_intents}\n"
        f"  Missing from frozen: {router_intents - FROZEN_SEMANTIC_INTENTS}"
    )


# ── 11. Schema placeholder is injected ────────────────────────────────────────

def test_schema_json_injected():
    """build_system_prompt must inject the schema JSON into the template."""
    marker = '{"test_schema": true}'
    prompt = build_system_prompt(marker)
    assert marker in prompt, "Schema JSON was not injected into prompt"
    assert "__JSON_SCHEMA__" not in prompt, "Placeholder was not replaced"


# ── 12. Prompt does not mention deprecated field names ────────────────────────

def test_prompt_does_not_mention_deprecated_slots(prompt: str):
    """Prompt must not use plan-draft slot names that differ from IntentRouter."""
    # These are plan-draft names that do NOT match the IntentRouter
    deprecated_pairs = [
        ("intent_type", "Should use 'intent'"),
        ("distance_m", "Should use 'delta' object"),
        ("joint_values", "Should use 'joint_target'"),
        ("duration_sec", "Should use 'wait_duration_sec'"),
    ]
    for deprecated_name, reason in deprecated_pairs:
        # Allow the name in descriptive text but not in JSON examples
        json_uses = re.findall(
            rf'"{deprecated_name}"\s*:', prompt
        )
        assert not json_uses, (
            f"Prompt uses deprecated slot '{deprecated_name}' in JSON. {reason}"
        )


# ── 13. Bilingual support ────────────────────────────────────────────────────

def test_prompt_mentions_vietnamese_support(prompt: str):
    """Prompt must mention Vietnamese language support."""
    assert "tiếng Việt" in prompt or "Vietnamese" in prompt
