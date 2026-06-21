"""Unit tests for :mod:`supervisor_sequence` pure helpers.

These cover the static helpers that do not require a full SupervisorService
instance. Impure methods (``_parse_sequence_steps``, ``_prepare_sequence_request``,
``_current_pose_snapshot``) are exercised indirectly via the end-to-end
:mod:`test_supervisor_service` tests.
"""

from __future__ import annotations

import unittest

from hmi.backend.services.supervisor_sequence import SupervisorSequenceMixin


class IsSequenceRequestTests(unittest.TestCase):
    def test_structured_intent_sequence_is_sequence(self) -> None:
        self.assertTrue(
            SupervisorSequenceMixin._is_sequence_request(
                {"intent": "sequence"}, sequence_segments=[]
            )
        )

    def test_structured_intent_sequence_case_insensitive(self) -> None:
        self.assertTrue(
            SupervisorSequenceMixin._is_sequence_request(
                {"intent": "  Sequence  "}, sequence_segments=[]
            )
        )

    def test_multiple_segments_is_sequence(self) -> None:
        self.assertTrue(SupervisorSequenceMixin._is_sequence_request(None, ["a", "b"]))

    def test_single_segment_not_sequence(self) -> None:
        self.assertFalse(
            SupervisorSequenceMixin._is_sequence_request(None, ["move to home"])
        )

    def test_empty_not_sequence(self) -> None:
        self.assertFalse(SupervisorSequenceMixin._is_sequence_request(None, []))

    def test_non_sequence_structured_intent_only(self) -> None:
        self.assertFalse(
            SupervisorSequenceMixin._is_sequence_request(
                {"intent": "move_to_home"}, sequence_segments=["x"]
            )
        )


class SplitSequenceTextTests(unittest.TestCase):
    def test_empty_returns_empty_list(self) -> None:
        self.assertEqual(SupervisorSequenceMixin._split_sequence_text(""), [])

    def test_semicolon_separator(self) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text("move home; open gripper"),
            ["move home", "open gripper"],
        )

    def test_newline_separator(self) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text("move home\nopen gripper"),
            ["move home", "open gripper"],
        )

    def test_and_then_separator_strips_connector(self) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text(
                "move home and then open gripper"
            ),
            ["move home", "open gripper"],
        )

    def test_then_separator_strips_connector(self) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text("go to A then go to B"),
            ["go to A", "go to B"],
        )

    def test_comma_before_letter_separator(self) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text("move home, open gripper"),
            ["move home", "open gripper"],
        )

    def test_coordinate_commas_do_not_create_sequence_segments(self) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text(
                "move linearly to x 0.3, y 0.0, z 0.3"
            ),
            ["move linearly to x 0.3, y 0.0, z 0.3"],
        )

    def test_coordinate_commas_with_equals_do_not_create_sequence_segments(
        self,
    ) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text("move to x=0.3, y=0.0, z=0.4"),
            ["move to x=0.3, y=0.0, z=0.4"],
        )

    def test_no_separator_single_segment(self) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text("move to point A"),
            ["move to point A"],
        )

    def test_trailing_whitespace_stripped(self) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text("  move home  "),
            ["move home"],
        )


class SequenceSummaryLabelTests(unittest.TestCase):
    def test_draw_shape_uses_shape_type(self) -> None:
        label = SupervisorSequenceMixin._sequence_summary_label(
            parsed_steps=[],
            route_metadata={"macro_name": "draw_shape", "shape_type": "circle"},
            raw_text="",
            structured_intent=None,
        )
        self.assertEqual(label, "Draw circle")

    def test_draw_shape_default_when_shape_type_empty(self) -> None:
        label = SupervisorSequenceMixin._sequence_summary_label(
            parsed_steps=[],
            route_metadata={"macro_name": "draw_shape", "shape_type": ""},
            raw_text="",
            structured_intent=None,
        )
        self.assertEqual(label, "Draw shape")

    def test_draw_text_uppercases(self) -> None:
        label = SupervisorSequenceMixin._sequence_summary_label(
            parsed_steps=[],
            route_metadata={"macro_name": "draw_text", "text": "hello"},
            raw_text="",
            structured_intent=None,
        )
        self.assertEqual(label, "Draw text 'HELLO'")

    def test_draw_text_empty_falls_back(self) -> None:
        label = SupervisorSequenceMixin._sequence_summary_label(
            parsed_steps=[],
            route_metadata={"macro_name": "draw_text", "text": ""},
            raw_text="",
            structured_intent=None,
        )
        self.assertEqual(label, "Draw text")

    def test_parsed_steps_joined_with_arrow(self) -> None:
        steps = [
            {"targetSummary": "HOME"},
            {"targetSummary": "PICK"},
            {"targetSummary": "PLACE"},
        ]
        label = SupervisorSequenceMixin._sequence_summary_label(
            parsed_steps=steps,
            route_metadata=None,
            raw_text="",
            structured_intent=None,
        )
        self.assertEqual(label, "HOME -> PICK -> PLACE")

    def test_raw_text_fallback(self) -> None:
        label = SupervisorSequenceMixin._sequence_summary_label(
            parsed_steps=[],
            route_metadata=None,
            raw_text="move here then move there",
            structured_intent=None,
        )
        self.assertEqual(label, "move here then move there")

    def test_structured_intent_fallback(self) -> None:
        label = SupervisorSequenceMixin._sequence_summary_label(
            parsed_steps=[],
            route_metadata=None,
            raw_text="",
            structured_intent={"intent": "custom"},
        )
        self.assertIn("custom", label)

    def test_default_label(self) -> None:
        label = SupervisorSequenceMixin._sequence_summary_label(
            parsed_steps=[],
            route_metadata=None,
            raw_text="",
            structured_intent=None,
        )
        self.assertEqual(label, "structured sequence")

    def test_label_truncated_to_120_chars(self) -> None:
        long_text = "x" * 500
        label = SupervisorSequenceMixin._sequence_summary_label(
            parsed_steps=[],
            route_metadata=None,
            raw_text=long_text,
            structured_intent=None,
        )
        self.assertEqual(len(label), 120)


class ShouldEmitBlendedSequenceTests(unittest.TestCase):
    def test_two_lin_steps_eligible(self) -> None:
        steps = [
            {"action": "LIN", "targetSummary": "A"},
            {"action": "LIN", "targetSummary": "B"},
        ]
        self.assertTrue(
            SupervisorSequenceMixin._should_emit_blended_sequence(steps, None)
        )

    def test_mixed_ptp_lin_home_not_eligible(self) -> None:
        steps = [
            {"action": "PTP", "targetSummary": "A"},
            {"action": "LIN", "targetSummary": "B"},
            {"action": "HOME", "targetSummary": "C"},
        ]
        self.assertFalse(
            SupervisorSequenceMixin._should_emit_blended_sequence(steps, None)
        )

    def test_single_step_not_eligible(self) -> None:
        steps = [
            {"action": "LIN", "targetSummary": "A"},
        ]
        self.assertFalse(
            SupervisorSequenceMixin._should_emit_blended_sequence(steps, None)
        )

    def test_wait_step_makes_ineligible(self) -> None:
        steps = [
            {"action": "LIN", "targetSummary": "A"},
            {"action": "WAIT", "targetSummary": "B"},
        ]
        self.assertFalse(
            SupervisorSequenceMixin._should_emit_blended_sequence(steps, None)
        )

    def test_draw_shape_macro_ineligible(self) -> None:
        steps = [
            {"action": "LIN", "targetSummary": "A"},
            {"action": "LIN", "targetSummary": "B"},
        ]
        self.assertFalse(
            SupervisorSequenceMixin._should_emit_blended_sequence(
                steps, {"macro_name": "draw_shape"}
            )
        )

    def test_named_pose_with_joint_target_ineligible(self) -> None:
        """Named poses resolve to joint_target; blending them creates GOAL_JOINTS rejected by execution_gate."""
        steps = [
            {
                "action": "PTP",
                "targetSummary": "A",
                "normalizedCommand": {
                    "primitive_type": "PTP",
                    "joint_target": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                },
            },
            {
                "action": "PTP",
                "targetSummary": "B",
                "normalizedCommand": {
                    "primitive_type": "PTP",
                    "joint_target": [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                },
            },
        ]
        self.assertFalse(
            SupervisorSequenceMixin._should_emit_blended_sequence(steps, None)
        )

    def test_home_primitive_ineligible(self) -> None:
        steps = [
            {
                "action": "LIN",
                "targetSummary": "A",
                "normalizedCommand": {
                    "primitive_type": "LIN",
                    "target_pose": {
                        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                    },
                },
            },
            {
                "action": "HOME",
                "targetSummary": "HOME",
                "normalizedCommand": {"primitive_type": "HOME"},
            },
        ]
        self.assertFalse(
            SupervisorSequenceMixin._should_emit_blended_sequence(steps, None)
        )


class BuildBlendedSequenceStepTests(unittest.TestCase):
    def test_two_pose_steps_collapsed(self) -> None:
        steps = [
            {
                "action": "LIN",
                "targetSummary": "A",
                "normalizedCommand": {
                    "primitive_type": "LIN",
                    "target_pose": {
                        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                    },
                },
            },
            {
                "action": "LIN",
                "targetSummary": "B",
                "normalizedCommand": {
                    "primitive_type": "LIN",
                    "target_pose": {
                        "position": {"x": 0.2, "y": 0.3, "z": 0.4},
                        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                    },
                },
            },
        ]
        blended = SupervisorSequenceMixin._build_blended_sequence_step(
            steps, raw_text="go to A then B", structured_intent=None
        )
        self.assertEqual(blended["action"], "BLENDED_SEQUENCE")
        seq_steps = blended["parameters"]["sequence_steps"]
        self.assertEqual(len(seq_steps), 2)
        self.assertEqual(seq_steps[0]["blend_radius_m"], 0.01)
        self.assertEqual(seq_steps[1]["blend_radius_m"], 0.0)
        self.assertEqual(seq_steps[0]["goal_type"], 0)
        self.assertEqual(seq_steps[1]["goal_type"], 0)

    def test_home_step_becomes_named_goal(self) -> None:
        steps = [
            {
                "action": "LIN",
                "targetSummary": "A",
                "normalizedCommand": {
                    "primitive_type": "LIN",
                    "target_pose": {
                        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                    },
                },
            },
            {
                "action": "HOME",
                "targetSummary": "HOME",
                "normalizedCommand": {"primitive_type": "HOME"},
            },
        ]
        blended = SupervisorSequenceMixin._build_blended_sequence_step(
            steps, raw_text="go to A then home", structured_intent=None
        )
        seq_steps = blended["parameters"]["sequence_steps"]
        self.assertEqual(seq_steps[1]["goal_type"], 2)
        self.assertEqual(seq_steps[1]["named_target"], "home")
        self.assertEqual(seq_steps[1]["blend_radius_m"], 0.0)

    def test_target_summary_joins_with_arrow(self) -> None:
        steps = [
            {"action": "LIN", "targetSummary": "A"},
            {"action": "LIN", "targetSummary": "B"},
        ]
        blended = SupervisorSequenceMixin._build_blended_sequence_step(
            steps, raw_text="", structured_intent=None
        )
        self.assertEqual(blended["targetSummary"], "A -> B")


class RuntimePlanRepeatNodeTests(unittest.TestCase):
    """Adapter must expand a FactoryTask runtime `repeat` node into N flat steps."""

    def _adapter(self) -> SupervisorSequenceMixin:
        # _runtime_plan_to_semantic_steps is a regular method (uses self only
        # via current_pose_loader, which we pass as a no-op).
        return SupervisorSequenceMixin()

    def test_repeat_node_expands_children_n_times(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "repeat",
            "count": 2,
            "body": {
                "type": "sequence",
                "children": [
                    {"type": "skill", "name": "go_home"},
                    {"type": "skill", "name": "stop"},
                ],
            },
        }
        steps = adapter._runtime_plan_to_semantic_steps(
            runtime_plan,
            current_pose_loader=lambda: None,
        )
        intents = [step["intent"] for step in steps]
        self.assertEqual(
            intents,
            ["go_home", "stop", "go_home", "stop"],
        )

    def test_repeat_node_count_one_returns_single_expansion(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "repeat",
            "count": 1,
            "body": {
                "type": "sequence",
                "children": [{"type": "skill", "name": "go_home"}],
            },
        }
        steps = adapter._runtime_plan_to_semantic_steps(
            runtime_plan,
            current_pose_loader=lambda: None,
        )
        self.assertEqual([step["intent"] for step in steps], ["go_home"])

    def test_repeat_node_with_zero_count_rejected(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "repeat",
            "count": 0,
            "body": {"type": "sequence", "children": []},
        }
        from hmi.backend.services.supervisor_validation import (
            IntentResolutionError,
        )

        with self.assertRaises(IntentResolutionError) as ctx:
            adapter._runtime_plan_to_semantic_steps(
                runtime_plan,
                current_pose_loader=lambda: None,
            )
        self.assertIn("count", str(ctx.exception).lower())

    def test_repeat_node_with_oversized_count_rejected(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "repeat",
            "count": 10_000,
            "body": {
                "type": "sequence",
                "children": [{"type": "skill", "name": "go_home"}],
            },
        }
        from hmi.backend.services.supervisor_validation import (
            IntentResolutionError,
        )

        with self.assertRaises(IntentResolutionError) as ctx:
            adapter._runtime_plan_to_semantic_steps(
                runtime_plan,
                current_pose_loader=lambda: None,
            )
        self.assertIn("100", str(ctx.exception))

    def test_repeat_node_missing_body_rejected(self) -> None:
        adapter = self._adapter()
        runtime_plan = {"type": "repeat", "count": 2}
        from hmi.backend.services.supervisor_validation import (
            IntentResolutionError,
        )

        with self.assertRaises(IntentResolutionError) as ctx:
            adapter._runtime_plan_to_semantic_steps(
                runtime_plan,
                current_pose_loader=lambda: None,
            )
        self.assertIn("body", str(ctx.exception).lower())

    def test_repeat_node_nested_inside_sequence(self) -> None:
        adapter = self._adapter()
        runtime_plan = {
            "type": "sequence",
            "children": [
                {"type": "skill", "name": "go_home"},
                {
                    "type": "repeat",
                    "count": 3,
                    "body": {
                        "type": "sequence",
                        "children": [{"type": "skill", "name": "stop"}],
                    },
                },
            ],
        }
        steps = adapter._runtime_plan_to_semantic_steps(
            runtime_plan,
            current_pose_loader=lambda: None,
        )
        intents = [step["intent"] for step in steps]
        self.assertEqual(intents, ["go_home", "stop", "stop", "stop"])


if __name__ == "__main__":
    unittest.main()
