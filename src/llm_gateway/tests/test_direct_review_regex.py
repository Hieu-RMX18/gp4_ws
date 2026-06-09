"""Unit tests for the narrowed direct review path.

Free-form motion language belongs to the ReAct/LLM path. Direct review only
keeps protective stop and safety-critical deterministic commands whose wrong
parse could move the wrong joint or enter station geometry.
"""

from __future__ import annotations

import pytest


@pytest.fixture
def direct_review():
    from llm_gateway.llm_gateway_node import LLMGatewayNode

    return LLMGatewayNode._direct_review_semantic_ir


class TestDirectReviewDeterministicSafety:
    @pytest.mark.parametrize(
        "raw_text",
        ["stop", "stop motion", "cancel motion", "halt"],
    )
    def test_protective_stop_stays_direct(self, direct_review, raw_text):
        assert direct_review(raw_text) == {"intent": "stop"}

    @pytest.mark.parametrize(
        ("raw_text", "expected"),
        [
            (
                "xoay khớp số 3 +15 độ",
                {
                    "intent": "move_joint_delta",
                    "joint_index": 2,
                    "delta_angle": 15.0,
                    "angular_unit": "deg",
                },
            ),
            (
                "xoay khớp 3 thêm 15 độ",
                {
                    "intent": "move_joint_delta",
                    "joint_index": 2,
                    "delta_angle": 15.0,
                    "angular_unit": "deg",
                },
            ),
            (
                "xoay khớp số 3 sang góc 45 độ",
                {
                    "intent": "move_joint",
                    "joint_index": 2,
                    "joint_angle": 45.0,
                    "angular_unit": "deg",
                },
            ),
            (
                "rotate joint 3 by -20 degrees",
                {
                    "intent": "move_joint_delta",
                    "joint_index": 2,
                    "delta_angle": -20.0,
                    "angular_unit": "deg",
                },
            ),
        ],
    )
    def test_obvious_single_joint_commands_are_direct_and_indexed(
        self, direct_review, raw_text, expected
    ):
        assert direct_review(raw_text) == expected

    def test_station_navigation_uses_top_surface_clearance(self, direct_review):
        class _SceneGraph:
            def resolve_region(self, query):
                assert query == "gá phôi"
                return type(
                    "Result",
                    (),
                    {
                        "ok": True,
                        "payload": {
                            "geometry": {
                                "center": {"x": 0.28, "y": 0.18, "z": 0.12},
                                "size": {"x": 0.22, "y": 0.12, "z": 0.10},
                            },
                            "zones": {"grasp_zone": {"default_clearance_m": 0.08}},
                        },
                    },
                )()

        result = direct_review("đi đến gá phôi", station_scene_graph=_SceneGraph())

        assert result == {
            "intent": "absolute_move_ptp",
            "target_pose": {
                "position": {"x": 0.28, "y": 0.18, "z": 0.25},
            },
        }

    @pytest.mark.parametrize(
        "raw_text",
        [
            "move down 2 cm",
            "move delta down 2 cm",
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
