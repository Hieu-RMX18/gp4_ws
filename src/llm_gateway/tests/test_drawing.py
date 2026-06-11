"""Consolidated tests; original source sections are marked below."""



# ---- test_drawing_geometry.py ----
"""Pure geometry tests for deterministic drawing compiler."""

import math

import pytest

from llm_gateway.drawing_geometry import (
    compile_strokes_to_commands,
    generate_arc_path,
    generate_circle_path,
    generate_polygon_path,
    generate_rectangle_path,
    generate_text_stroke_segments,
    generate_triangle_path,
    resolve_workplane,
)


def _base_workplane():
    return resolve_workplane(
        mode="base",
        frame_id="base_link",
        anchor_position=(0.30, 0.00, 0.30),
        origin_pose={
            "position": {"x": 0.30, "y": 0.00, "z": 0.30},
        },
        default_orientation={"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
    )


def test_circle_path_radius_correctness():
    strokes = generate_circle_path(
        radius_m=0.03, max_chord_error_m=0.0005, max_segment_angle_rad=math.radians(10)
    )
    points = strokes[0].points_2d

    center_x = 0.03
    center_y = 0.0
    for x_coord, y_coord in points:
        radius = math.sqrt((x_coord - center_x) ** 2 + (y_coord - center_y) ** 2)
        assert math.isclose(radius, 0.03, abs_tol=2e-3)


def test_rectangle_dimensions():
    strokes = generate_rectangle_path(width_m=0.05, height_m=0.08)
    points = strokes[0].points_2d

    assert points[0] == (0.0, 0.0)
    assert points[1] == (0.05, 0.0)
    assert points[2] == (0.05, 0.08)
    assert points[3] == (0.0, 0.08)
    assert points[4] == (0.0, 0.0)


def test_polygon_vertex_count_and_closure():
    strokes = generate_polygon_path(n_sides=6, radius_m=0.04)
    points = strokes[0].points_2d

    assert len(points) == 7
    assert points[0][0] == pytest.approx(points[-1][0], abs=1e-12)
    assert points[0][1] == pytest.approx(points[-1][1], abs=1e-12)


def test_arc_sweep_behavior():
    strokes = generate_arc_path(
        radius_m=0.03,
        sweep_rad=math.radians(180),
        max_chord_error_m=0.0005,
        max_segment_angle_rad=math.radians(10),
    )
    points = strokes[0].points_2d

    assert points[0] != points[-1]
    assert math.isclose(points[0][0], 0.0, abs_tol=1e-9)
    assert math.isclose(points[-1][0], 0.06, abs_tol=1e-6)


def test_text_glyph_scaling_and_spacing():
    segments = generate_text_stroke_segments(
        text="A A",
        height_m=0.02,
        char_spacing_m=0.004,
        line_spacing_m=0.01,
        alignment="left",
    )

    draw_points = [
        point
        for segment in segments
        if segment.kind == "draw"
        for point in segment.points_2d
    ]
    min_y = min(point[1] for point in draw_points)
    max_y = max(point[1] for point in draw_points)

    assert math.isclose(min_y, 0.0, abs_tol=1e-9)
    assert math.isclose(max_y, 0.02, abs_tol=1e-9)

    first_draw_start = [segment for segment in segments if segment.kind == "draw"][
        0
    ].points_2d[0][0]
    second_draw_start = [segment for segment in segments if segment.kind == "draw"][
        3
    ].points_2d[0][0]
    assert second_draw_start > first_draw_start + 0.02


@pytest.mark.parametrize(
    "glyph,expected_draw_strokes",
    [
        ("A", 3),
        ("H", 3),
        ("K", 3),
        ("4", 2),
    ],
)
def test_disconnected_stroke_count(glyph: str, expected_draw_strokes: int):
    segments = generate_text_stroke_segments(
        text=glyph,
        height_m=0.02,
        char_spacing_m=0.004,
        line_spacing_m=0.01,
        alignment="left",
    )
    draw_strokes = [segment for segment in segments if segment.kind == "draw"]
    assert len(draw_strokes) == expected_draw_strokes


def test_compile_strokes_generates_chunk_metadata():
    workplane = _base_workplane()
    strokes = generate_triangle_path(side_m=0.05)

    compiled = compile_strokes_to_commands(
        strokes=strokes,
        workplane=workplane,
        reference_frame="base_link",
        approach_distance_m=0.01,
        retract_distance_m=0.01,
        drawing_speed_scale=0.1,
        travel_speed_scale=0.2,
        plan_only=True,
        max_waypoints_per_chunk=2,
    )

    assert compiled.summary["draw_stroke_count"] == 1
    assert compiled.summary["chunk_count"] >= 1
    assert any(
        command["primitive_type"] in {"LIN", "CARTESIAN_PATH"}
        for command in compiled.commands
    )


# ---- test_draw_shape.py ----
"""Tests for DRAW_SHAPE routing and deterministic geometry compilation."""

import math
from pathlib import Path

import pytest


def _macro_policy_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


def _router(runtime_mode: str = "hardware"):
    from llm_gateway.factory_task import IntentRouter

    return IntentRouter(
        macro_policy_path=_macro_policy_path(), runtime_mode=runtime_mode
    )


def _draw_payload(shape_type: str = "square", **overrides) -> dict:
    base = {
        "intent": "draw_shape",
        "shape_type": shape_type,
        "units": "m",
        "frame_id": "base_link",
        "workplane": {
            "mode": "base",
            "origin": {
                "position": {"x": 0.30, "y": 0.00, "z": 0.30},
            },
        },
        "params": {
            "side_m": 0.05,
        },
    }
    base.update(overrides)
    return base


def _all_draw_positions(result) -> list[dict]:
    positions = []
    for command in result.commands:
        primitive = command["primitive_type"]
        if primitive in {"PTP", "LIN"}:
            positions.append(command["target_pose"]["position"])
        elif primitive == "CARTESIAN_PATH":
            positions.extend(waypoint["position"] for waypoint in command["waypoints"])
        elif primitive == "BLENDED_SEQUENCE":
            for step in command["sequence_steps"]:
                positions.append(step["target_pose"]["position"])
    return positions


def _collect_draw_line_points(result) -> list[dict]:
    points = []
    for command in result.commands:
        if command["primitive_type"] == "LIN":
            points.append(command["target_pose"]["position"])
        elif command["primitive_type"] == "CARTESIAN_PATH":
            points.extend(waypoint["position"] for waypoint in command["waypoints"])
        elif command["primitive_type"] == "BLENDED_SEQUENCE":
            for step in command["sequence_steps"]:
                points.append(step["target_pose"]["position"])
    return points


def _distance(p1: dict, p2: dict) -> float:
    return math.sqrt(
        (p2["x"] - p1["x"]) ** 2 + (p2["y"] - p1["y"]) ** 2 + (p2["z"] - p1["z"]) ** 2
    )


def test_macro_policy_declares_draw_shape_contract():
    from llm_gateway.factory_task import load_macro_policy

    policy = load_macro_policy(_macro_policy_path())
    draw_shape = policy["macros"]["draw_shape"]

    assert draw_shape["availability"] == "all"
    assert "base" in draw_shape["supported_workplane_modes"]
    assert "tool" in draw_shape["supported_workplane_modes"]
    assert "explicit_pose" in draw_shape["supported_workplane_modes"]
    assert draw_shape["default_units"] == "m"


def test_accepts_draw_shape_in_hardware_mode():
    router = _router(runtime_mode="hardware")

    result = router.route(_draw_payload("square"))

    assert result.route_type == "sequence"
    assert result.metadata["macro_name"] == "draw_shape"
    assert result.metadata["shape_type"] == "square"
    assert result.metadata["summary"]["draw_stroke_count"] >= 1


def test_rejects_unsupported_shape_type():
    router = _router()

    with pytest.raises(ValueError, match="unsupported_shape_type"):
        router.route(_draw_payload(shape_type="star"))


def test_rejects_invalid_polygon_sides():
    router = _router()

    with pytest.raises(ValueError, match="invalid_polygon_sides"):
        router.route(
            _draw_payload(
                shape_type="polygon",
                params={"n_sides": 2, "radius_m": 0.03},
            )
        )


def test_square_draws_closed_path_with_expected_edge_lengths():
    router = _router()

    result = router.route(_draw_payload("square", params={"side_m": 0.05}))
    points = _collect_draw_line_points(result)

    # Last retract LIN is above draw plane, so use first five in-plane points.
    in_plane = [
        point for point in points if math.isclose(point["z"], 0.30, abs_tol=1e-9)
    ]
    assert len(in_plane) >= 5
    assert in_plane[0] == in_plane[-1]

    edges = [
        _distance(in_plane[index], in_plane[index + 1])
        for index in range(len(in_plane) - 1)
    ]
    # Keep first 4 polygon edges.
    for edge in edges[:4]:
        assert math.isclose(edge, 0.05, abs_tol=1e-6)


def test_circle_points_stay_on_declared_radius():
    router = _router()
    radius_m = 0.03

    result = router.route(
        _draw_payload(
            "circle",
            params={"radius_m": radius_m},
        )
    )

    draw_points = [
        point
        for point in _collect_draw_line_points(result)
        if math.isclose(point["z"], 0.30, abs_tol=1e-9)
    ]
    center_x = 0.30 + radius_m
    center_y = 0.00

    for point in draw_points:
        distance = math.sqrt(
            (point["x"] - center_x) ** 2 + (point["y"] - center_y) ** 2
        )
        assert math.isclose(distance, radius_m, abs_tol=2e-3)


def test_arc_uses_non_closed_path_with_requested_sweep():
    router = _router()

    result = router.route(
        _draw_payload(
            "arc",
            params={"radius_m": 0.03, "sweep_deg": 180.0},
        )
    )
    draw_points = [
        point
        for point in _collect_draw_line_points(result)
        if math.isclose(point["z"], 0.30, abs_tol=1e-9)
    ]

    assert draw_points[0] != draw_points[-1]
    assert math.isclose(draw_points[0]["x"], 0.30, abs_tol=1e-6)
    assert math.isclose(draw_points[-1]["x"], 0.36, abs_tol=1e-3)


def test_polyline_preserves_point_ordering():
    router = _router()
    result = router.route(
        _draw_payload(
            "polyline",
            params={
                "points": [
                    {"x": 0.0, "y": 0.0},
                    {"x": 0.02, "y": 0.01},
                    {"x": 0.05, "y": 0.03},
                ],
            },
        )
    )

    draw_points = [
        point
        for point in _collect_draw_line_points(result)
        if math.isclose(point["z"], 0.30, abs_tol=1e-9)
    ]
    assert draw_points[0]["x"] == pytest.approx(0.30)
    assert draw_points[1]["x"] == pytest.approx(0.32)
    assert draw_points[2]["x"] == pytest.approx(0.35)


def test_tool_workplane_requires_origin_or_start_pose():
    router = _router()

    with pytest.raises(ValueError, match="missing_workplane"):
        router.route(
            {
                "intent": "draw_shape",
                "shape_type": "square",
                "units": "m",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "params": {"side_m": 0.05},
            }
        )


def test_plan_only_marks_segments_without_requiring_approval():
    router = _router(runtime_mode="sim")

    result = router.route(
        _draw_payload(
            "rectangle",
            execution_mode="plan_only",
            params={"width_m": 0.05, "height_m": 0.08},
        )
    )

    for command in result.commands:
        assert command["plan_only"] is True


def test_dense_circle_is_chunked_into_multiple_segments():
    router = _router()

    result = router.route(
        _draw_payload(
            "circle",
            params={"radius_m": 0.10},
            stroke={"drawing_speed_scale": 0.10, "travel_speed_scale": 0.15},
        )
    )

    seq_commands = [
        command
        for command in result.commands
        if command["primitive_type"] in ("CARTESIAN_PATH", "BLENDED_SEQUENCE")
    ]
    assert len(seq_commands) >= 1
    for command in seq_commands:
        assert command["stroke_index"] >= 1
        if command["primitive_type"] == "BLENDED_SEQUENCE":
            assert len(command["sequence_steps"]) >= 2
            assert command["sequence_steps"][0]["blend_radius_m"] == 0.0
            assert command["sequence_steps"][-1]["blend_radius_m"] == 0.0


def test_all_generated_commands_carry_reference_frame():
    router = _router()

    result = router.route(_draw_payload("triangle", params={"side_m": 0.05}))

    for command in result.commands:
        assert command["reference_frame"] == "base_link"


# ---- test_draw_text.py ----
"""Tests for DRAW_TEXT routing and stroke compilation."""

from pathlib import Path

import pytest


def _macro_policy_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


def _router(runtime_mode: str = "hardware"):
    from llm_gateway.factory_task import IntentRouter

    return IntentRouter(
        macro_policy_path=_macro_policy_path(), runtime_mode=runtime_mode
    )


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
    from llm_gateway.factory_task import load_macro_policy

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
        router.route(
            _draw_text_payload(font={"type": "single_stroke_builtin", "height_m": 0.0})
        )


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


def test_draw_text_plan_only_marks_commands_without_requiring_approval():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_text_payload(execution_mode="plan_only"))

    for command in result.commands:
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


# ---- test_stroke_font.py ----
"""Pure geometry tests for the draw_text stroke font."""

import math

import pytest

from llm_gateway.stroke_font import GLYPHS, SUPPORTED_GLYPHS, generate_text_strokes


def test_supported_glyph_inventory_is_complete():
    assert len(SUPPORTED_GLYPHS) == 42
    assert {"A", "Z", "0", "9", ".", ",", "-", "_", "/", " "} <= SUPPORTED_GLYPHS
    assert set(GLYPHS.keys()) == SUPPORTED_GLYPHS


def test_generate_text_strokes_scales_to_requested_height():
    segments = generate_text_strokes("L", height_m=0.02, char_spacing_m=0.004)

    draw_points = [
        point
        for segment in segments
        if segment.kind == "draw"
        for point in segment.points_2d
    ]
    ys = [point[1] for point in draw_points]

    assert math.isclose(min(ys), 0.0, abs_tol=1e-9)
    assert math.isclose(max(ys), 0.02, abs_tol=1e-9)


def test_generate_text_strokes_for_a_preserves_stroke_breaks():
    segments = generate_text_strokes("A", height_m=0.02, char_spacing_m=0.004)

    kinds = [segment.kind for segment in segments]
    assert kinds == ["draw", "travel", "draw", "travel", "draw"]


def test_generate_text_strokes_marks_closed_glyph_loop():
    segments = generate_text_strokes("O", height_m=0.02, char_spacing_m=0.004)

    draw_segments = [segment for segment in segments if segment.kind == "draw"]
    assert len(draw_segments) == 1
    assert draw_segments[0].closed is True


def test_generate_text_strokes_space_inserts_horizontal_gap():
    segments = generate_text_strokes("A A", height_m=0.02, char_spacing_m=0.004)
    draw_segments = [segment for segment in segments if segment.kind == "draw"]

    first_a_start_x = draw_segments[0].points_2d[0][0]
    second_a_start_x = draw_segments[3].points_2d[0][0]

    assert second_a_start_x > first_a_start_x + 0.02


def test_generate_text_strokes_rejects_unsupported_glyph():
    with pytest.raises(ValueError, match="Unsupported glyph"):
        generate_text_strokes("@", height_m=0.02, char_spacing_m=0.004)


def test_generate_text_strokes_rejects_invalid_dimensions():
    with pytest.raises(ValueError, match="height_m"):
        generate_text_strokes("A", height_m=0.0, char_spacing_m=0.004)

    with pytest.raises(ValueError, match="char_spacing_m"):
        generate_text_strokes("A", height_m=0.02, char_spacing_m=-0.001)
