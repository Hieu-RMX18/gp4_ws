"""Sequence-handling mixin for :class:`SupervisorService`.

Groups the sequence-specific parsing, routing, summarising, and current-pose
bootstrapping methods. Peer of :class:`SupervisorViewsMixin`,
:class:`SupervisorValidationMixin`, :class:`SupervisorExecutionMixin`.

This module is **not** a standalone service — it must be mixed into
:class:`SupervisorService` because several methods depend on the host class
for ``self._intent_resolution``, ``self._ros``, ``self._parse_intent``, etc.
"""

from __future__ import annotations

import json
import re
from typing import Any

from ..domain.models import RuntimeMode
from .intent_resolution import (
    IntentResolutionError,
    IntentRouter,
    SequenceValidationError,
    SequenceValidator,
)


# Strong sequence separators only. Commas are handled separately so coordinate
# lists like "x 0.3, y 0.0, z 0.3" do not get misclassified as multi-step
# sequences.
_SEQUENCE_SPLIT_PATTERN = re.compile(
    r"(?:\s*;\s*|\s*\n+\s*|\s+(?:and\s+then|then)\s+)",
    re.IGNORECASE,
)

_COMMA_SEQUENCE_PREFIXES = {
    "alarm",
    "bật",
    "bat",
    "clear",
    "close",
    "cho",
    "di",
    "dich",
    "doi",
    "dong",
    "dừng",
    "dung",
    "draw",
    "go",
    "ha",
    "home",
    "lift",
    "lower",
    "mở",
    "mo",
    "move",
    "open",
    "pause",
    "quay",
    "reset",
    "rotate",
    "set",
    "stop",
    "tat",
    "tắt",
    "về",
    "vẽ",
    "ve",
    "wait",
    "write",
    "xoay",
    "đi",
    "đợi",
    "đóng",
    "dịch",
    "nâng",
    "hạ",
    "chờ",
}


class SupervisorSequenceMixin:
    """Sequence parsing, routing, and summary helpers for SupervisorService."""

    # ── Sequence intake ──────────────────────────────────────────────────

    @staticmethod
    def _is_sequence_request(
        structured_intent: dict[str, Any] | None,
        sequence_segments: list[str],
    ) -> bool:
        """Return True when the input should be handled as a multi-step sequence."""
        if (
            structured_intent is not None
            and str(structured_intent.get("intent") or "").strip().lower() == "sequence"
        ):
            return True
        return len(sequence_segments) > 1

    @staticmethod
    def _split_sequence_text(raw_text: str) -> list[str]:
        """Split a free-form instruction into cleaned sequence segments."""
        if not raw_text:
            return []
        segments: list[str] = []
        for segment in _SEQUENCE_SPLIT_PATTERN.split(raw_text):
            cleaned = re.sub(
                r"^(?:and\s+then|then)\s+",
                "",
                segment.strip(" ,;"),
                flags=re.IGNORECASE,
            )
            if not cleaned:
                continue

            comma_parts = [part.strip(" ,;") for part in cleaned.split(",")]
            current = comma_parts[0]
            for part in comma_parts[1:]:
                if not part:
                    continue
                if SupervisorSequenceMixin._starts_new_comma_clause(part):
                    if current:
                        segments.append(current)
                    current = part
                else:
                    current = f"{current}, {part}" if current else part
            if current:
                segments.append(current)
        return segments

    @staticmethod
    def _starts_new_comma_clause(fragment: str) -> bool:
        """Return True when a comma-separated fragment starts a new command."""
        words = fragment.strip().split()
        if not words:
            return False
        return words[0].lower() in _COMMA_SEQUENCE_PREFIXES

    # ── Sequence planning (impure — host state needed) ───────────────────

    def _prepare_sequence_request(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        requested_mode: RuntimeMode,
    ) -> dict[str, Any] | None:
        return self._intent_resolution.prepare_sequence_submission(
            raw_text=raw_text,
            structured_intent=structured_intent,
            runtime_mode=requested_mode.value,
            current_joints=self._current_joints(),
            current_pose_loader=self._current_pose_snapshot,
        )

    def _current_pose_snapshot(self) -> dict[str, Any] | None:
        reader = getattr(self._ros, "get_current_pose", None)
        if not callable(reader):
            return None
        return reader(reference_frame="base_link")

    def _parse_sequence_steps(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        mode: RuntimeMode,
        sequence_segments: list[str],
    ) -> tuple[
        list[dict[str, Any]] | None, list[str], str | None, dict[str, Any] | None
    ]:
        diagnostics: list[str] = []
        parsed_steps: list[dict[str, Any]] = []
        if (
            structured_intent is not None
            and str(structured_intent.get("intent") or "").strip().lower() == "sequence"
        ):
            if IntentRouter is None or SequenceValidator is None:
                return (
                    None,
                    diagnostics,
                    "sequence routing is unavailable because llm_gateway sequence helpers are not importable.",
                    {"intent": "sequence"},
                )
            try:
                routed = IntentRouter(runtime_mode=mode.value).route(structured_intent)
                if routed.route_type != "sequence":
                    return (
                        None,
                        diagnostics,
                        "structured sequence did not resolve to a sequence route.",
                        {"intent": "sequence"},
                    )
                validation = SequenceValidator().validate(routed.commands)
                diagnostics.extend(validation.diagnostics)
                for command in routed.commands:
                    parsed_steps.append(
                        self._intent_resolution.resolve(
                            raw_text="",
                            structured_intent=command,
                            runtime_mode=mode.value,
                            current_joints=self._current_joints(),
                        )
                    )
            except (IntentResolutionError, SequenceValidationError, ValueError) as exc:
                return None, diagnostics, str(exc), {"intent": "sequence"}
            return parsed_steps, diagnostics, None, dict(routed.metadata)

        for segment in sequence_segments:
            parsed_step, parse_error = self._parse_intent(segment, None, mode=mode)
            if parsed_step is None:
                return None, diagnostics, parse_error, None
            parsed_steps.append(parsed_step)
        if SequenceValidator is not None:
            try:
                validation = SequenceValidator().validate(
                    [parsed_step["normalizedCommand"] for parsed_step in parsed_steps]
                )
                diagnostics.extend(validation.diagnostics)
            except SequenceValidationError as exc:
                return None, diagnostics, str(exc), None
        return parsed_steps, diagnostics, None, None

    # ── Sequence summarisation (pure) ────────────────────────────────────

    @staticmethod
    def _sequence_summary_label(
        *,
        parsed_steps: list[dict[str, Any]],
        route_metadata: dict[str, Any] | None,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
    ) -> str:
        macro_name = str((route_metadata or {}).get("macro_name") or "").strip().lower()
        if macro_name == "draw_shape":
            shape_type = (
                str((route_metadata or {}).get("shape_type") or "shape").strip().lower()
                or "shape"
            )
            return f"Draw {shape_type}"[:120]
        if macro_name == "draw_text":
            text = str((route_metadata or {}).get("text") or "").strip().upper()
            return f"Draw text '{text}'"[:120] if text else "Draw text"
        if parsed_steps:
            return " -> ".join(step["targetSummary"] for step in parsed_steps)[:120]
        if raw_text:
            return raw_text[:120]
        if structured_intent is not None:
            return json.dumps(
                structured_intent, separators=(",", ":"), ensure_ascii=True
            )[:120]
        return "structured sequence"

    def _sequence_plan_summary(
        self,
        *,
        raw_text: str,
        structured_intent: dict[str, Any] | None,
        parsed_steps: list[dict[str, Any]],
        diagnostics: list[str],
        route_metadata: dict[str, Any] | None,
        requires_confirmation: bool,
    ) -> dict[str, Any]:
        summary = {
            "normalizedIntent": (
                raw_text
                or (
                    json.dumps(
                        structured_intent, separators=(",", ":"), ensure_ascii=True
                    )
                    if structured_intent is not None
                    else ""
                )
            ),
            "parsedAction": "SEQUENCE",
            "targetSummary": self._sequence_summary_label(
                parsed_steps=parsed_steps,
                route_metadata=route_metadata,
                raw_text=raw_text,
                structured_intent=structured_intent,
            ),
            "requiresConfirmation": requires_confirmation,
            "stepCount": len(parsed_steps),
        }
        if diagnostics:
            summary["diagnostics"] = list(diagnostics)
        macro_name = str((route_metadata or {}).get("macro_name") or "").strip().lower()
        if macro_name:
            summary["macroName"] = macro_name
        if macro_name == "draw_shape":
            summary["shapeType"] = (route_metadata or {}).get("shape_type")
        if macro_name == "draw_text":
            summary["text"] = (route_metadata or {}).get("text")
        if isinstance((route_metadata or {}).get("summary"), dict):
            summary["macroSummary"] = dict((route_metadata or {})["summary"])
        if (route_metadata or {}).get("execution_mode") is not None:
            summary["executionMode"] = (route_metadata or {}).get("execution_mode")
        return summary
