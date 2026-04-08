"""Tests for draw_text macro routing."""

from pathlib import Path

import pytest


def _macro_policy_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


def _router(runtime_mode: str):
    from llm_gateway.intent_router import IntentRouter

    return IntentRouter(macro_policy_path=_macro_policy_path(), runtime_mode=runtime_mode)


def _draw_text_payload(text: str = "GP4", **overrides) -> dict:
    base = {
        "intent": "draw_text",
        "text": text,
        "height_m": 0.02,
        "plane": "xy",
        "reference_frame": "base_link",
        "start_pose": {
            "position": {"x": 0.30, "y": 0.00, "z": 0.30},
        },
    }
    base.update(overrides)
    return base


def test_macro_policy_declares_draw_text_sim_only():
    from llm_gateway.intent_router import load_macro_policy

    policy = load_macro_policy(_macro_policy_path())
    draw_text = policy["macros"]["draw_text"]

    assert draw_text["availability"] == "sim_only"
    assert draw_text["requires_current_pose"] is False
    assert draw_text["supported_frames"] == ["base_link"]
    assert draw_text["supported_planes"] == ["xy"]
    assert "A" in draw_text["supported_characters"]
    assert " " in draw_text["supported_characters"]


def test_rejects_draw_text_in_hardware_mode():
    router = _router(runtime_mode="hardware")

    with pytest.raises(ValueError, match="sim-only"):
        router.route(_draw_text_payload())


def test_rejects_unsupported_character():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="unsupported characters"):
        router.route(_draw_text_payload(text="@"))


def test_rejects_missing_start_pose():
    router = _router(runtime_mode="sim")
    payload = _draw_text_payload()
    del payload["start_pose"]

    with pytest.raises(ValueError, match="start_pose"):
        router.route(payload)


def test_rejects_invalid_height():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="height_m"):
        router.route(_draw_text_payload(height_m=0.0))


def test_draw_text_a_routes_to_ptp_and_cartesian_path_per_stroke():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_text_payload(text="A"))

    assert result.route_type == "sequence"
    assert result.metadata["macro_name"] == "draw_text"
    assert result.metadata["text"] == "A"
    assert [command["primitive_type"] for command in result.commands] == [
        "PTP",
        "CARTESIAN_PATH",
        "PTP",
        "CARTESIAN_PATH",
        "PTP",
        "CARTESIAN_PATH",
    ]


def test_draw_text_approach_moves_stay_above_draw_plane():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_text_payload(text="A"))
    ptp_commands = [command for command in result.commands if command["primitive_type"] == "PTP"]
    cartesian_commands = [command for command in result.commands if command["primitive_type"] == "CARTESIAN_PATH"]

    assert all(command["target_pose"]["position"]["z"] == pytest.approx(0.31) for command in ptp_commands)
    assert all(
        waypoint["position"]["z"] == pytest.approx(0.30)
        for command in cartesian_commands
        for waypoint in command["waypoints"]
    )


def test_draw_text_all_commands_carry_reference_frame_and_orientation():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_text_payload(text="GP4"))

    for command in result.commands:
        assert command["reference_frame"] == "base_link"
        if command["primitive_type"] == "PTP":
            assert command["target_pose"]["orientation"] == {
                "x": 0.0,
                "y": 1.0,
                "z": 0.0,
                "w": 0.0,
            }
        else:
            for waypoint in command["waypoints"]:
                assert waypoint["orientation"] == {
                    "x": 0.0,
                    "y": 1.0,
                    "z": 0.0,
                    "w": 0.0,
                }


def test_draw_text_space_creates_horizontal_offset():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_text_payload(text="A A"))
    ptp_commands = [command for command in result.commands if command["primitive_type"] == "PTP"]

    assert ptp_commands[0]["target_pose"]["position"]["x"] < ptp_commands[-1]["target_pose"]["position"]["x"]


def test_draw_text_gp4_stays_within_sequence_limit_budget():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_text_payload(text="GP4"))

    assert len(result.commands) < 40
