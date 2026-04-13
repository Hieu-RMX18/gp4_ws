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
    strokes = generate_circle_path(radius_m=0.03, max_chord_error_m=0.0005, max_segment_angle_rad=math.radians(10))
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

    draw_points = [point for segment in segments if segment.kind == "draw" for point in segment.points_2d]
    min_y = min(point[1] for point in draw_points)
    max_y = max(point[1] for point in draw_points)

    assert math.isclose(min_y, 0.0, abs_tol=1e-9)
    assert math.isclose(max_y, 0.02, abs_tol=1e-9)

    first_draw_start = [segment for segment in segments if segment.kind == "draw"][0].points_2d[0][0]
    second_draw_start = [segment for segment in segments if segment.kind == "draw"][3].points_2d[0][0]
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
        require_approval=True,
        plan_only=True,
        max_waypoints_per_chunk=2,
    )

    assert compiled.summary["draw_stroke_count"] == 1
    assert compiled.summary["chunk_count"] >= 1
    assert any(command["primitive_type"] in {"LIN", "CARTESIAN_PATH"} for command in compiled.commands)
