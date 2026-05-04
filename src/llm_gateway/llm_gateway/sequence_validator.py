"""Full-sequence prevalidation for routed primitive command sequences."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List


_QUERY_PRIMITIVES = {"GET_POSE"}
_FRAME_REQUIRED_PRIMITIVES = {"PTP", "LIN", "MOVE_REL", "CARTESIAN_PATH"}
_SUPPORTED_SEQUENCE_FRAMES = {"base_link"}


@dataclass(frozen=True)
class SequenceValidationResult:
    normalized_commands: List[Dict[str, Any]]
    step_count: int
    validated_reference_frame: str | None
    cumulative_move_rel_distance_m: float
    estimated_duration_lower_bound_sec: float
    duration_estimate_is_lower_bound: bool
    has_io_side_effects: bool
    manual_recovery_required_on_failure: bool
    diagnostics: List[str] = field(default_factory=list)


class SequenceValidationError(ValueError):
    """Structured sequence prevalidation failure with stage and step context."""

    def __init__(self, stage: str, reason: str, *, step_index: int | None = None):
        self.stage = stage
        self.step_index = step_index
        self.reason = reason
        prefix = f"sequence_validation:{stage}"
        if step_index is not None:
            prefix = f"{prefix}:step={step_index + 1}"
        super().__init__(f"{prefix}: {reason}")


class SequenceValidator:
    """Prevalidate a full routed primitive sequence before any dispatch occurs."""

    def __init__(
        self,
        schema_validator: Any | None = None,
        normalizer: Any | None = None,
        semantic_validator: Any | None = None,
        *,
        max_sequence_length: int = 40,
        max_cumulative_move_rel_distance_m: float = 0.40,
    ) -> None:
        if schema_validator is None:
            from llm_gateway.schema_validator import SchemaValidator

            schema_validator = SchemaValidator()
        if normalizer is None:
            from llm_gateway.normalizer import Normalizer

            normalizer = Normalizer()
        if semantic_validator is None:
            from llm_gateway.semantic_validator import SemanticValidator

            semantic_validator = SemanticValidator()

        self._schema_validator = schema_validator
        self._normalizer = normalizer
        self._semantic_validator = semantic_validator
        self._max_sequence_length = int(max_sequence_length)
        self._max_cumulative_move_rel_distance_m = float(
            max_cumulative_move_rel_distance_m
        )

    def validate(self, commands: List[Dict[str, Any]]) -> SequenceValidationResult:
        if not isinstance(commands, list) or not commands:
            raise SequenceValidationError(
                "structure", "sequence must be a non-empty list of primitive commands."
            )
        if len(commands) > self._max_sequence_length:
            raise SequenceValidationError(
                "sequence_length",
                f"sequence has {len(commands)} steps, limit is {self._max_sequence_length}.",
            )

        normalized_commands: List[Dict[str, Any]] = []
        validated_frame: str | None = None
        cumulative_move_rel_distance_m = 0.0
        estimated_duration_lower_bound_sec = 0.0
        first_io_step_index: int | None = None

        for step_index, command in enumerate(commands):
            if not isinstance(command, dict):
                raise SequenceValidationError(
                    "structure",
                    "each sequence step must be an object.",
                    step_index=step_index,
                )

            primitive_type = str(command.get("primitive_type", ""))
            if not primitive_type:
                raise SequenceValidationError(
                    "structure",
                    "sequence step is missing primitive_type.",
                    step_index=step_index,
                )

            if primitive_type in _QUERY_PRIMITIVES:
                raise SequenceValidationError(
                    "unsupported_step",
                    f"{primitive_type} is query-only and is not supported inside sequences.",
                    step_index=step_index,
                )

            if primitive_type == "STOP" and len(commands) != 1:
                raise SequenceValidationError(
                    "stop_policy",
                    "STOP must be the sole primitive in a sequence.",
                    step_index=step_index,
                )

            try:
                self._schema_validator.validate(command)
            except Exception as exc:
                raise SequenceValidationError(
                    "schema", str(exc), step_index=step_index
                ) from exc

            try:
                normalized_command = self._normalizer.normalize(command)
            except Exception as exc:
                raise SequenceValidationError(
                    "normalize", str(exc), step_index=step_index
                ) from exc

            try:
                self._semantic_validator.validate(normalized_command)
            except Exception as exc:
                raise SequenceValidationError(
                    "semantic", str(exc), step_index=step_index
                ) from exc

            step_frame = self._resolve_step_frame(normalized_command, step_index)
            if step_frame is not None:
                if validated_frame is None:
                    validated_frame = step_frame
                elif validated_frame != step_frame:
                    raise SequenceValidationError(
                        "frame_policy",
                        f"mixed frames are not supported; saw '{validated_frame}' and '{step_frame}'.",
                        step_index=step_index,
                    )

            if primitive_type == "MOVE_REL":
                cumulative_move_rel_distance_m += self._move_rel_distance(
                    normalized_command
                )
                if (
                    cumulative_move_rel_distance_m
                    > self._max_cumulative_move_rel_distance_m
                ):
                    raise SequenceValidationError(
                        "move_rel_budget",
                        "cumulative MOVE_REL distance exceeds sequence limit "
                        f"{self._max_cumulative_move_rel_distance_m:.3f} m.",
                        step_index=step_index,
                    )

            if primitive_type == "WAIT":
                estimated_duration_lower_bound_sec += float(
                    normalized_command.get("wait_duration_sec", 0.0)
                )

            if primitive_type == "IO_SET" and first_io_step_index is None:
                first_io_step_index = step_index

            normalized_commands.append(normalized_command)

        has_io_side_effects = first_io_step_index is not None
        manual_recovery_required_on_failure = (
            has_io_side_effects and first_io_step_index < len(commands) - 1
        )

        diagnostics = [
            "Validated checks: structure, max length, STOP sole-primitive policy, per-step schema, "
            "normalization, semantic validation, explicit frame policy, cumulative MOVE_REL budget."
        ]
        diagnostics.append(
            "Duration estimate is a heuristic lower bound only; WAIT contributes directly and all other "
            "primitive timing remains conservative/unknown at prevalidation time."
        )
        diagnostics.append(
            "NOT YET IMPLEMENTED: kinematic reachability across the full sequence, collision continuity, "
            "controller timing feasibility, IO rollback analysis, and current-pose-aware macro validation."
        )

        return SequenceValidationResult(
            normalized_commands=normalized_commands,
            step_count=len(normalized_commands),
            validated_reference_frame=validated_frame,
            cumulative_move_rel_distance_m=cumulative_move_rel_distance_m,
            estimated_duration_lower_bound_sec=estimated_duration_lower_bound_sec,
            duration_estimate_is_lower_bound=True,
            has_io_side_effects=has_io_side_effects,
            manual_recovery_required_on_failure=manual_recovery_required_on_failure,
            diagnostics=diagnostics,
        )

    def _resolve_step_frame(
        self, normalized_command: Dict[str, Any], step_index: int
    ) -> str | None:
        primitive_type = str(normalized_command.get("primitive_type", ""))
        requires_frame = primitive_type in _FRAME_REQUIRED_PRIMITIVES

        if not requires_frame:
            return None

        if "reference_frame" not in normalized_command:
            raise SequenceValidationError(
                "frame_policy",
                f"{primitive_type} requires explicit reference_frame in sequences; no silent fallback is allowed.",
                step_index=step_index,
            )

        step_frame = str(normalized_command.get("reference_frame", "")).strip()
        if step_frame not in _SUPPORTED_SEQUENCE_FRAMES:
            raise SequenceValidationError(
                "frame_policy",
                f"unsupported reference_frame '{step_frame}'; only 'base_link' is supported in v2.1.",
                step_index=step_index,
            )
        return step_frame

    @staticmethod
    def _move_rel_distance(normalized_command: Dict[str, Any]) -> float:
        dx = float(normalized_command.get("delta_x", 0.0))
        dy = float(normalized_command.get("delta_y", 0.0))
        dz = float(normalized_command.get("delta_z", 0.0))
        return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))


__all__ = [
    "SequenceValidator",
    "SequenceValidationError",
    "SequenceValidationResult",
]
