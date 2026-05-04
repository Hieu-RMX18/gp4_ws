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
