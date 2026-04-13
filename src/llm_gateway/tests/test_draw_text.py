"""Tests for DRAW_TEXT routing and stroke compilation."""

from pathlib import Path

import pytest


def _macro_policy_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


def _router(runtime_mode: str = "hardware"):
    from llm_gateway.intent_router import IntentRouter

    return IntentRouter(macro_policy_path=_macro_policy_path(), runtime_mode=runtime_mode)


def _draw_text_payload(text: str = "GP4", **overrides) -> dict:
    base = {
        "intent": "draw_text",
        "text": text,
        "units": "m",
        "frame_id": "base_link",
        "workplane": {
            "mode": "base",
            "origin": {
                "position": {"x": 0.30, "y": 0.00, "z": 0.30},
            },
        },
        "font": {
            "type": "single_stroke_builtin",
            "height_m": 0.02,
        },
    }
    base.update(overrides)
    return base


def test_macro_policy_declares_draw_text_contract():
    from llm_gateway.intent_router import load_macro_policy

    policy = load_macro_policy(_macro_policy_path())
    draw_text = policy["macros"]["draw_text"]

    assert draw_text["availability"] == "all"
    assert "base" in draw_text["supported_workplane_modes"]
    assert "A" in draw_text["supported_characters"]
    assert " " in draw_text["supported_characters"]


def test_accepts_draw_text_in_hardware_mode():
    router = _router(runtime_mode="hardware")

    result = router.route(_draw_text_payload())

    assert result.route_type == "sequence"
    assert result.metadata["macro_name"] == "draw_text"
    assert result.metadata["text"] == "GP4"


def test_rejects_unsupported_character():
    router = _router()

    with pytest.raises(ValueError, match="unsupported_font_glyph"):
        router.route(_draw_text_payload(text="@@@"))


def test_rejects_invalid_font_height():
    router = _router()

    with pytest.raises(ValueError, match="text height"):
        router.route(_draw_text_payload(font={"type": "single_stroke_builtin", "height_m": 0.0}))


def test_rejects_non_builtin_font_type():
    router = _router()

    with pytest.raises(ValueError, match="single_stroke_builtin"):
        router.route(_draw_text_payload(font={"type": "hershey", "height_m": 0.02}))


def test_draw_text_routes_to_motion_sequence_with_approach_and_retract():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_text_payload(text="A"))
    primitive_types = [command["primitive_type"] for command in result.commands]

    assert primitive_types.count("PTP") >= 1
    assert primitive_types.count("LIN") >= 2
    assert any(primitive in {"LIN", "CARTESIAN_PATH"} for primitive in primitive_types)


def test_draw_text_uppercases_input_automatically():
    router = _router()

    result = router.route(_draw_text_payload(text="gp4"))

    assert result.metadata["text"] == "GP4"


def test_draw_text_alignment_and_spacing_fields_are_supported():
    router = _router()

    result = router.route(
        _draw_text_payload(
            text="HELLO",
            font={
                "type": "single_stroke_builtin",
                "height": 20,
                "char_spacing": 3,
                "line_spacing": 10,
                "alignment": "center",
            },
            units="mm",
        )
    )

    assert result.route_type == "sequence"
    assert result.metadata["summary"]["draw_stroke_count"] >= 1


def test_draw_text_plan_only_marks_all_commands_for_confirmation():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_text_payload(execution_mode="plan_only"))

    for command in result.commands:
        assert command["require_approval"] is True
        assert command["plan_only"] is True


def test_draw_text_carries_reference_frame_and_orientation():
    router = _router()

    result = router.route(_draw_text_payload(text="HI"))

    for command in result.commands:
        assert command["reference_frame"] == "base_link"
        pose = command.get("target_pose")
        if pose is not None:
            assert set(pose["orientation"].keys()) == {"x", "y", "z", "w"}


def test_draw_text_supports_newline_multiline():
    router = _router()

    result = router.route(_draw_text_payload(text="HI\nGP4"))

    assert result.route_type == "sequence"
    assert result.metadata["summary"]["draw_stroke_count"] >= 2
