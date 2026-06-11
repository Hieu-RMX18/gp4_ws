"""Phase 5 tests for IntentRouter.

Covers all semantic intent → primitive routing, passthrough paths,
error handling, and field validation. Does NOT test runtime integration.
"""

from pathlib import Path

import pytest


def _macro_policy_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


def _router(runtime_mode: str = "hardware"):
    from llm_gateway.factory_task import IntentRouter

    return IntentRouter(
        macro_policy_path=_macro_policy_path(), runtime_mode=runtime_mode
    )


# ── Passthrough paths ────────────────────────────────────────────────────────


def test_passthrough_direct_primitive_command():
    router = _router()
    payload = {"primitive_type": "HOME"}

    result = router.route(payload)

    assert result.route_type == "primitive"
    assert result.commands == [payload]
    assert result.metadata["source"] == "primitive_passthrough"


def test_passthrough_error_payload():
    router = _router()
    payload = {
        "error": "MISSING_SLOT",
        "missing_slots": ["target_pose"],
        "message": "target pose is required",
    }

    result = router.route(payload)

    assert result.route_type == "error"
    assert result.error_payload == payload
    assert not result.commands


def test_passthrough_preserves_all_primitive_fields():
    """Extra fields on a direct primitive are preserved through passthrough."""
    router = _router()
    payload = {
        "primitive_type": "SET_SPEED",
        "velocity_scale": 0.2,
        "extra_field": "should_survive",
    }

    result = router.route(payload)

    assert result.commands[0]["extra_field"] == "should_survive"


# ── Input validation ─────────────────────────────────────────────────────────


def test_rejects_non_dict_payload():
    router = _router()

    with pytest.raises(ValueError, match="must be an object"):
        router.route("not a dict")


def test_rejects_missing_intent_field():
    router = _router()

    with pytest.raises(ValueError, match="non-empty intent field"):
        router.route({"some_key": "some_value"})


def test_rejects_empty_intent_field():
    router = _router()

    with pytest.raises(ValueError, match="non-empty intent field"):
        router.route({"intent": ""})


def test_rejects_unsupported_intent():
    router = _router()

    with pytest.raises(ValueError, match="unsupported semantic intent"):
        router.route({"intent": "fly_to_moon"})


# ── Simple intents (no required fields) ──────────────────────────────────────


def test_go_home():
    router = _router()

    result = router.route({"intent": "go_home"})

    assert result.route_type == "primitive"
    assert len(result.commands) == 1
    assert result.commands[0]["primitive_type"] == "HOME"


def test_move_named_pose_routes_srdf_group_state_to_existing_ptp_joint_target():
    router = _router()

    result = router.route({"intent": "move_named_pose", "pose_name": "ready"})

    assert result.route_type == "primitive"
    assert len(result.commands) == 1
    command = result.commands[0]
    assert command["primitive_type"] == "PTP"
    assert command["planner_id"] == "PILZ_PTP"
    assert command["reference_frame"] == "base_link"
    assert command["joint_target"] == pytest.approx(
        [
            1.938101818035138,
            0.0903533622099061,
            -0.15852595742235326,
            0.0,
            -1.1752774274713826,
            0.05333592949720888,
        ]
    )
    assert "pose_name" not in command
    assert "named_target" not in command


def test_move_named_pose_rejects_unknown_srdf_group_state():
    router = _router()

    with pytest.raises(ValueError, match="unknown named pose"):
        router.route({"intent": "move_named_pose", "pose_name": "park_safe"})


@pytest.mark.parametrize(
    "raw_pose_name",
    ["poseA", "pose A", "pose a", "A", "a", "point A", "điểm A", "diem a"],
)
def test_move_named_pose_canonicalizes_aliases_to_poseA(raw_pose_name):
    router = _router()

    result = router.route({"intent": "move_named_pose", "pose_name": raw_pose_name})

    assert result.route_type == "primitive"
    command = result.commands[0]
    assert command["primitive_type"] == "PTP"
    assert command["planner_id"] == "PILZ_PTP"
    assert len(command["joint_target"]) == 6


@pytest.mark.parametrize(
    "raw_pose_name",
    ["poseB", "pose B", "B", "point B", "điểm B"],
)
def test_move_named_pose_canonicalizes_aliases_to_poseB(raw_pose_name):
    router = _router()

    result = router.route({"intent": "move_named_pose", "pose_name": raw_pose_name})

    assert result.route_type == "primitive"
    assert result.commands[0]["primitive_type"] == "PTP"


def test_move_named_pose_rejects_unknown_alias():
    router = _router()

    with pytest.raises(ValueError, match="unknown named pose.*available"):
        router.route({"intent": "move_named_pose", "pose_name": "poseZ"})


def test_return_to_start_requires_captured_joint_target():
    router = _router()

    with pytest.raises(ValueError, match="captured joint_target"):
        router.route({"intent": "return_to_start"})


def test_return_to_start_routes_captured_joint_target_to_move_joints():
    router = _router()

    result = router.route(
        {"intent": "return_to_start", "joint_target": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]}
    )

    assert result.route_type == "primitive"
    assert result.commands[0]["primitive_type"] == "MOVE_JOINTS"
    assert result.commands[0]["joint_target"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_stop():
    router = _router()

    result = router.route({"intent": "stop"})

    assert result.commands[0]["primitive_type"] == "STOP"


def test_alarm_reset():
    router = _router()

    result = router.route({"intent": "alarm_reset"})

    assert result.commands[0]["primitive_type"] == "ALARM_RESET"


def test_get_pose_defaults_to_base_link():
    router = _router()

    result = router.route({"intent": "get_pose"})

    assert result.commands[0]["primitive_type"] == "GET_POSE"
    assert result.commands[0]["reference_frame"] == "base_link"


def test_get_pose_explicit_frame():
    router = _router()

    result = router.route({"intent": "get_pose", "reference_frame": "base_link"})

    assert result.commands[0]["reference_frame"] == "base_link"


def test_get_pose_rejects_unsupported_frame():
    router = _router()

    with pytest.raises(ValueError, match="unsupported reference_frame"):
        router.route({"intent": "get_pose", "reference_frame": "tool0"})


# ── set_speed ────────────────────────────────────────────────────────────────


def test_set_speed():
    router = _router()

    result = router.route({"intent": "set_speed", "velocity_scale": 0.15})

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "SET_SPEED"
    assert cmd["velocity_scale"] == 0.15


def test_set_speed_missing_velocity_raises():
    router = _router()

    with pytest.raises(ValueError, match="velocity_scale"):
        router.route({"intent": "set_speed"})


# ── wait ─────────────────────────────────────────────────────────────────────


def test_wait():
    router = _router()

    result = router.route({"intent": "wait", "wait_duration_sec": 3.0})

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "WAIT"
    assert cmd["wait_duration_sec"] == 3.0


def test_wait_missing_duration_defaults_to_two_seconds():
    """When LLM omits wait_duration_sec, the router defaults to 2.0 s."""
    router = _router()

    result = router.route({"intent": "wait"})

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "WAIT"
    assert cmd["wait_duration_sec"] == 2.0


# ── move_relative ────────────────────────────────────────────────────────────


def test_move_relative():
    router = _router()

    result = router.route(
        {
            "intent": "move_relative",
            "delta": {"x": 0.0, "y": 0.0, "z": 0.05},
            "reference_frame": "base_link",
        }
    )

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "MOVE_REL"
    assert cmd["delta_z"] == 0.05
    assert cmd["reference_frame"] == "base_link"


def test_move_relative_preserves_explicit_linear_unit():
    router = _router()

    result = router.route(
        {
            "intent": "move_relative",
            "delta": {"x": 0.0, "y": 0.0, "z": 5.0},
            "linear_unit": "cm",
            "reference_frame": "base_link",
        }
    )

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "MOVE_REL"
    assert cmd["delta_z"] == 5.0
    assert cmd["linear_unit"] == "cm"


def test_move_relative_missing_delta_raises():
    router = _router()

    with pytest.raises(
        ValueError, match="relative move requires direction and distance"
    ):
        router.route({"intent": "move_relative", "reference_frame": "base_link"})


def test_move_relative_defaults_missing_axes_to_zero():
    router = _router()

    result = router.route(
        {
            "intent": "move_relative",
            "delta": {"z": 0.03},
            "reference_frame": "base_link",
        }
    )

    cmd = result.commands[0]
    assert cmd["delta_x"] == 0.0
    assert cmd["delta_y"] == 0.0
    assert cmd["delta_z"] == 0.03


# ── absolute_move_ptp ────────────────────────────────────────────────────────


def test_absolute_move_ptp_with_orientation_preset():
    router = _router()

    result = router.route(
        {
            "intent": "absolute_move_ptp",
            "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
            "orientation_preset": "tool_forward",
        }
    )

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "PTP"
    assert cmd["target_pose"]["orientation"] == {
        "x": 0.0,
        "y": 0.707,
        "z": 0.0,
        "w": 0.707,
    }


def test_absolute_move_ptp_without_orientation_keeps_current():
    """v2.1: omitting orientation means keep_current_orientation semantics."""
    router = _router()

    result = router.route(
        {
            "intent": "absolute_move_ptp",
            "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
        }
    )

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "PTP"
    assert "orientation" not in cmd["target_pose"]


def test_absolute_move_ptp_missing_target_pose_raises():
    router = _router()

    with pytest.raises(ValueError, match="target_pose"):
        router.route({"intent": "absolute_move_ptp"})


def test_absolute_move_ptp_rejects_unsupported_frame():
    router = _router()

    with pytest.raises(ValueError, match="unsupported reference_frame"):
        router.route(
            {
                "intent": "absolute_move_ptp",
                "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
                "reference_frame": "tool0",
            }
        )


# ── absolute_move_lin ────────────────────────────────────────────────────────


def test_absolute_move_lin():
    router = _router()

    result = router.route(
        {
            "intent": "absolute_move_lin",
            "target_pose": {"position": {"x": 0.35, "y": 0.1, "z": 0.25}},
            "reference_frame": "base_link",
            "velocity_scale": 0.15,
        }
    )

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "LIN"
    assert cmd["target_pose"]["position"] == {"x": 0.35, "y": 0.1, "z": 0.25}
    assert cmd["reference_frame"] == "base_link"
    assert cmd["velocity_scale"] == 0.15


def test_absolute_move_lin_without_orientation():
    """v2.1 fix: LIN without orientation does NOT default to tool-down."""
    router = _router()

    result = router.route(
        {
            "intent": "absolute_move_lin",
            "target_pose": {"position": {"x": 0.35, "y": 0.1, "z": 0.25}},
        }
    )

    cmd = result.commands[0]
    assert "orientation" not in cmd["target_pose"]


# ── move_joint ───────────────────────────────────────────────────────────────


def test_move_joint():
    router = _router()

    result = router.route(
        {
            "intent": "move_joint",
            "joint_index": 2,
            "joint_angle": 0.524,
        }
    )

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "MOVE_JOINT"
    assert cmd["joint_index"] == 2
    assert cmd["joint_angle"] == 0.524


def test_move_joint_missing_joint_index_raises():
    router = _router()

    with pytest.raises(ValueError, match="joint_index"):
        router.route({"intent": "move_joint", "joint_angle": 0.5})


def test_move_joint_missing_joint_angle_raises():
    router = _router()

    with pytest.raises(ValueError, match="joint_angle"):
        router.route({"intent": "move_joint", "joint_index": 2})


# ── move_joints ──────────────────────────────────────────────────────────────


def test_move_joints():
    router = _router()
    joint_values = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]

    result = router.route(
        {
            "intent": "move_joints",
            "joint_target": joint_values,
        }
    )

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "MOVE_JOINTS"
    assert cmd["joint_target"] == joint_values


def test_move_joints_missing_joint_target_raises():
    router = _router()

    with pytest.raises((ValueError, KeyError, TypeError)):
        router.route({"intent": "move_joints"})


# ── io_set ───────────────────────────────────────────────────────────────────


def test_io_set():
    router = _router()

    result = router.route(
        {
            "intent": "io_set",
            "io_address": 10010,
            "io_value": 1,
        }
    )

    cmd = result.commands[0]
    assert cmd["primitive_type"] == "IO_SET"
    assert cmd["io_address"] == 10010
    assert cmd["io_value"] == 1


def test_io_set_missing_io_address_raises():
    router = _router()

    with pytest.raises(ValueError, match="io_address"):
        router.route({"intent": "io_set", "io_value": 1})


def test_io_set_missing_io_value_raises():
    router = _router()

    with pytest.raises(ValueError, match="io_value"):
        router.route({"intent": "io_set", "io_address": 10010})


# ── sequence ─────────────────────────────────────────────────────────────────


def test_sequence_flattens_intents():
    router = _router()

    result = router.route(
        {
            "intent": "sequence",
            "steps": [
                {"intent": "go_home"},
                {"intent": "wait", "wait_duration_sec": 1.5},
                {
                    "intent": "absolute_move_lin",
                    "target_pose": {"position": {"x": 0.35, "y": 0.1, "z": 0.25}},
                },
            ],
        }
    )

    assert result.route_type == "sequence"
    assert [c["primitive_type"] for c in result.commands] == ["HOME", "WAIT", "LIN"]


def test_sequence_empty_steps_raises():
    router = _router()

    with pytest.raises(ValueError, match="non-empty steps"):
        router.route({"intent": "sequence", "steps": []})


def test_sequence_non_list_steps_raises():
    router = _router()

    with pytest.raises(ValueError, match="non-empty steps"):
        router.route({"intent": "sequence", "steps": "not a list"})


def test_sequence_rejects_error_step():
    router = _router()

    with pytest.raises(ValueError, match="error payload"):
        router.route(
            {
                "intent": "sequence",
                "steps": [
                    {"intent": "go_home"},
                    {"error": "MISSING_SLOT", "message": "test"},
                ],
            }
        )


def test_sequence_non_dict_step_raises():
    router = _router()

    with pytest.raises(ValueError, match="must be an object"):
        router.route(
            {
                "intent": "sequence",
                "steps": [{"intent": "go_home"}, "not_a_dict"],
            }
        )


# ── Optional motion fields ───────────────────────────────────────────────────


def test_optional_motion_fields_forwarded():
    """Motion metadata, including explicit unit hints, passes through."""
    router = _router()

    result = router.route(
        {
            "intent": "go_home",
            "velocity_scale": 0.2,
            "acceleration_scale": 0.1,
            "planner_id": "PILZ_PTP",
            "linear_unit": "mm",
            "angular_unit": "deg",
        }
    )

    cmd = result.commands[0]
    assert cmd["velocity_scale"] == 0.2
    assert cmd["acceleration_scale"] == 0.1
    assert cmd["planner_id"] == "PILZ_PTP"
    assert cmd["linear_unit"] == "mm"
    assert cmd["angular_unit"] == "deg"


# ── Semantic intent naming consistency ───────────────────────────────────────


def test_all_documented_intents_are_routable():
    """Every intent name used in the prompt/plan must be accepted by the router."""
    router = _router(runtime_mode="sim")  # sim to allow draw_shape routing
    documented_intents = [
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
        "sequence",
    ]
    # Verify none of these raise "unsupported semantic intent"
    for intent_name in documented_intents:
        # Build minimal valid payload per intent
        payload = _minimal_payload_for(intent_name)
        result = router.route(payload)
        assert result.route_type in (
            "primitive",
            "sequence",
            "error",
        ), f"Intent '{intent_name}' returned unexpected route_type: {result.route_type}"


def _minimal_payload_for(intent_name: str) -> dict:
    """Build the minimal valid Semantic IR payload for each intent."""
    base = {"intent": intent_name}
    extras = {
        "set_speed": {"velocity_scale": 0.1},
        "wait": {"wait_duration_sec": 1.0},
        "move_relative": {"delta": {"z": 0.01}, "reference_frame": "base_link"},
        "absolute_move_ptp": {
            "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
        },
        "absolute_move_lin": {
            "target_pose": {"position": {"x": 0.3, "y": 0.0, "z": 0.4}},
        },
        "move_joint": {"joint_index": 0, "joint_angle": 0.0},
        "move_joints": {"joint_target": [0.0] * 6},
        "io_set": {"io_address": 10010, "io_value": 1},
        "draw_shape": {
            "shape_type": "square",
            "units": "m",
            "frame_id": "base_link",
            "workplane": {
                "mode": "base",
                "origin": {"position": {"x": 0.3, "y": 0.0, "z": 0.3}},
            },
            "params": {"side_m": 0.05},
        },
        "draw_text": {
            "text": "GP4",
            "units": "m",
            "frame_id": "base_link",
            "workplane": {
                "mode": "base",
                "origin": {"position": {"x": 0.3, "y": 0.0, "z": 0.3}},
            },
            "font": {"type": "single_stroke_builtin", "height_m": 0.02},
        },
        "sequence": {
            "steps": [{"intent": "go_home"}],
        },
    }
    base.update(extras.get(intent_name, {}))
    return base


# ── circular_move intent (CIRC) ─────────────────────────────────────────────


def test_circular_move_routes_to_circ():
    router = _router()

    result = router.route(
        {
            "intent": "circular_move",
            "target_pose": {"position": {"x": 0.30, "y": 0.00, "z": 0.40}},
            "auxiliary_pose": {"position": {"x": 0.32, "y": 0.05, "z": 0.42}},
        }
    )

    assert result.route_type == "primitive"
    assert len(result.commands) == 1
    cmd = result.commands[0]
    assert cmd["primitive_type"] == "CIRC"
    assert cmd["reference_frame"] == "base_link"
    assert "target_pose" in cmd
    assert isinstance(cmd["waypoints"], list) and len(cmd["waypoints"]) == 1


def test_circular_move_missing_auxiliary_pose_rejects():
    router = _router()

    with pytest.raises(ValueError, match="auxiliary_pose"):
        router.route(
            {
                "intent": "circular_move",
                "target_pose": {"position": {"x": 0.30, "y": 0.00, "z": 0.40}},
            }
        )


def test_circular_move_missing_target_pose_rejects():
    router = _router()

    with pytest.raises(ValueError, match="target_pose"):
        router.route(
            {
                "intent": "circular_move",
                "auxiliary_pose": {"position": {"x": 0.32, "y": 0.05, "z": 0.42}},
            }
        )
