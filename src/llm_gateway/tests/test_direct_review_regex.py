"""Hợp đồng tầng direct (đã chuyển sang llm_gateway.direct_commands) + draw params."""

from __future__ import annotations

import pytest


@pytest.fixture
def direct_review():
    from llm_gateway import direct_commands

    return direct_commands.parse


class TestDirectReviewDeterministicSafety:
    @pytest.mark.parametrize(
        ("raw_text", "expected"),
        [
            ("stop", {"intent": "stop"}),
            ("alarm_reset", {"intent": "alarm_reset"}),
            ("get_pose", {"intent": "get_pose", "reference_frame": "base_link"}),
        ],
    )
    def test_exact_safety_and_read_only_shortcuts_stay_direct(
        self, direct_review, raw_text, expected
    ):
        assert direct_review(raw_text) == expected

    @pytest.mark.parametrize(
        "raw_text",
        [
            "move to pose A",
            "move to Cartesian x 300 mm y 0 z 400",
            "move down 2 cm",
            "repeat 2 times move to pose A then home",
            "đi đến gá phôi",
            "xoay khớp số 3 +15 độ",
            "draw circle radius 5cm",
            "vẽ hình tròn bán kính 5cm",
        ],
    )
    def test_free_form_language_goes_to_planner_or_llm(self, direct_review, raw_text):
        assert direct_review(raw_text) is None


class TestDrawParamsValidation:
    """Unit tests for LLMGatewayNode._validate_draw_params."""

    @staticmethod
    def _validate(payload: dict) -> str | None:
        from llm_gateway.validation import validate_draw_params
        return validate_draw_params(payload)

    def test_valid_circle_passes(self):
        result = self._validate({
            "intent": "draw_shape",
            "shape_type": "circle",
            "params": {"radius": 5.0},
        })
        assert result is None

    def test_missing_radius_rejected(self):
        result = self._validate({
            "intent": "draw_shape",
            "shape_type": "circle",
            "params": {},
        })
        assert result is not None
        assert "circle" in result.lower()

    def test_diameter_accepted(self):
        result = self._validate({
            "intent": "draw_shape",
            "shape_type": "circle",
            "params": {"diameter": 10.0},
        })
        assert result is None

    def test_circle_no_params_at_all(self):
        result = self._validate({
            "intent": "draw_shape",
            "shape_type": "circle",
        })
        assert result is not None

    def test_square_missing_side_rejected(self):
        result = self._validate({
            "intent": "draw_shape",
            "shape_type": "square",
            "params": {},
        })
        assert result is not None

    def test_square_with_side_passes(self):
        result = self._validate({
            "intent": "draw_shape",
            "shape_type": "square",
            "params": {"side": 5.0},
        })
        assert result is None

    def test_rectangle_missing_dims_rejected(self):
        result = self._validate({
            "intent": "draw_shape",
            "shape_type": "rectangle",
            "params": {},
        })
        assert result is not None

    def test_rectangle_missing_height_rejected(self):
        result = self._validate({
            "intent": "draw_shape",
            "shape_type": "rectangle",
            "params": {"width": 50},
        })
        assert result is not None
        assert "height" in result

    def test_rectangle_with_size_passes(self):
        result = self._validate({
            "intent": "draw_shape",
            "shape_type": "rectangle",
            "params": {"width": 50, "height": 80},
        })
        assert result is None

    def test_missing_text_rejected(self):
        result = self._validate({
            "intent": "draw_text",
            "text": "",
        })
        assert result is not None

    def test_valid_text_passes(self):
        result = self._validate({
            "intent": "draw_text",
            "text": "HELLO",
            "font": {"height": 20},
        })
        assert result is None

    def test_text_missing_height_rejected(self):
        result = self._validate({
            "intent": "draw_text",
            "text": "HELLO",
        })
        assert result is not None
        assert "height" in result

    def test_non_draw_intent_returns_none(self):
        result = self._validate({
            "intent": "move_relative",
            "delta": {"x": 0.0, "y": 0.0, "z": 5.0},
        })
        assert result is None
