"""Unit tests for the narrowed direct review path.

Free-form motion language belongs to the ReAct/LLM path. The only raw-text
direct review shortcut left in llm_gateway is the protective stop command.
"""

from __future__ import annotations

import pytest


@pytest.fixture
def direct_review():
    from llm_gateway.llm_gateway_node import LLMGatewayNode

    return LLMGatewayNode._direct_review_semantic_ir


class TestDirectReviewProtectiveOnly:
    @pytest.mark.parametrize(
        "raw_text",
        ["stop", "stop motion", "cancel motion", "halt"],
    )
    def test_protective_stop_stays_direct(self, direct_review, raw_text):
        assert direct_review(raw_text) == {"intent": "stop"}

    @pytest.mark.parametrize(
        "raw_text",
        [
            "move down 2 cm",
            "move delta down 2 cm",
            "rotate joint s +15 degrees",
            "draw circle radius 5cm",
            "vẽ hình tròn bán kính 5cm",
            "go home",
            "get pose",
            "wait 2 s",
        ],
    )
    def test_motion_and_utility_language_goes_to_react_or_llm(
        self, direct_review, raw_text
    ):
        assert direct_review(raw_text) is None


class TestDrawParamsValidation:
    """Unit tests for LLMGatewayNode._validate_draw_params."""

    @staticmethod
    def _validate(payload: dict) -> str | None:
        from llm_gateway.llm_gateway_node import LLMGatewayNode
        return LLMGatewayNode._validate_draw_params(payload)

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
