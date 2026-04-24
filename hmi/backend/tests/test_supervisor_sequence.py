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
        self.assertTrue(
            SupervisorSequenceMixin._is_sequence_request(None, ["a", "b"])
        )

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

    def test_coordinate_commas_with_equals_do_not_create_sequence_segments(self) -> None:
        self.assertEqual(
            SupervisorSequenceMixin._split_sequence_text(
                "move to x=0.3, y=0.0, z=0.4"
            ),
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


if __name__ == "__main__":
    unittest.main()
