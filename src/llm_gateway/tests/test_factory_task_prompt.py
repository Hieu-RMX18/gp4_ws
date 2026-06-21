"""Prompt contract tests for the FactoryTask LLM-facing prompt builder."""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

from llm_gateway.task_planner import build_system_prompt


@pytest.fixture(scope="module")
def prompt() -> str:
    return build_system_prompt("{}")


def _load_workspace_bounds() -> dict:
    safety_yaml = (
        Path(__file__).resolve().parents[2] / "safety" / "config" / "safety_rules.yaml"
    )
    safety_rules = yaml.safe_load(safety_yaml.read_text()) or {}
    return safety_rules["workspace_bounds"]


def _section(prompt: str, start: str, end: str) -> str:
    match = re.search(start + r".*?" + end, prompt, re.DOTALL)
    assert match is not None, f"Could not find prompt section {start!r}"
    return match.group(0)


def test_prompt_declares_factory_task_as_normal_output(prompt: str) -> None:
    assert "FactoryTask" in prompt
    assert '"task_type": "factory_task"' in prompt
    assert '"version": "1.0"' in prompt
    assert '"root"' in prompt


def test_prompt_output_section_does_not_expose_semantic_ir_or_primitives(
    prompt: str,
) -> None:
    output_section = _section(prompt, "OUTPUT FORMAT", "FACTORY TASK NODE TYPES")
    assert "Semantic IR (normal path)" not in output_section
    assert '"intent": "sequence"' not in output_section
    assert "primitive_type" not in output_section
    assert "Do not output final Semantic IR" in output_section


@pytest.mark.parametrize(
    "node_type",
    ["sequence", "skill", "repeat", "for_each", "until", "if", "retry", "fallback", "observe", "wait_until"],
)
def test_prompt_lists_factory_task_node_types(prompt: str, node_type: str) -> None:
    assert node_type in prompt


@pytest.mark.parametrize(
    "skill_name",
    [
        "go_home",
        "wait",
        "move_named_pose",
        "move_to_region",
        "move_to_object",
        "pick_object",
        "place_object",
        "place_relative",
        "verify_scene",
    ],
)
def test_prompt_lists_factory_task_skills(prompt: str, skill_name: str) -> None:
    assert skill_name in prompt


def test_prompt_preserves_runtime_loop_semantics(prompt: str) -> None:
    assert "never expand loops into long static sequences" in prompt
    assert "retry" in prompt
    assert "fallback" in prompt
    assert "replan_policy" in prompt


def test_prompt_includes_workspace_limits(prompt: str) -> None:
    bounds = _load_workspace_bounds()
    expected_line = (
        f"WORKSPACE LIMITS (meters): "
        f"x: {bounds['x_min']:.2f}–{bounds['x_max']:.2f}, "
        f"y: {bounds['y_min']:.2f}–{bounds['y_max']:.2f}, "
        f"z: {bounds['z_min']:.2f}–{bounds['z_max']:.2f}"
    )
    assert expected_line in prompt


def test_prompt_includes_units_and_velocity_constraints(prompt: str) -> None:
    assert "0.01" in prompt and "0.06" in prompt
    assert "1 phân = 1 cm = 0.01 m" in prompt
    assert "1 mm = 0.001 m" in prompt
    assert "linear_unit" in prompt
    assert "angular_unit" in prompt


def test_prompt_keeps_safety_boundary_visible(prompt: str) -> None:
    assert "does not execute motion" in prompt
    assert "supervisor validation" in prompt
    assert "operator confirmation" in prompt
    assert "collision" in prompt.lower()


def test_prompt_prefers_runtime_observe_for_ungrounded_world_facts(prompt: str) -> None:
    assert "ALWAYS generate a FactoryTask with an observe" in prompt
    assert "Only return MISSING_SLOT if the operator's command is fundamentally incomplete" in prompt
    assert "must produce MISSING_SLOT or a FactoryTask observe step" not in prompt


def test_prompt_includes_bilingual_operator_examples(prompt: str) -> None:
    assert "tiếng Việt" in prompt or "Vietnamese" in prompt
    assert "về nhà" in prompt
    assert "move to pose A" in prompt
    assert "nhặt" in prompt or "pick" in prompt


def test_prompt_absolute_motion_uses_factory_task_skill_args(prompt: str) -> None:
    assert "target_pose" in prompt
    assert "keep_current_orientation" in prompt
    assert "orientation_preset" in prompt


def test_schema_json_injected() -> None:
    marker = '{"test_schema": true}'
    prompt = build_system_prompt(marker)
    assert marker in prompt
    assert "__JSON_SCHEMA__" not in prompt


def test_prompt_does_not_use_deprecated_json_slots(prompt: str) -> None:
    deprecated_names = ["intent_type", "distance_m", "joint_values", "duration_sec"]
    for deprecated_name in deprecated_names:
        assert not re.findall(rf'"{deprecated_name}"\s*:', prompt)
