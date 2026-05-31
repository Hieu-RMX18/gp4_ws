"""Strict Semantic IR contract gate.

Enforced before IntentRouter in both review and runtime paths.
Rejects raw LLM leakage (raw_text, primitive_type) and unknown intents
with structured reasons and actionable fix hints.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

# Must match react_planner.FROZEN_TOP_LEVEL_OUTPUT_INTENTS exactly.
# Duplicated here so the contract module has no import-time dependency
# on the heavy react_planner stack.
_FROZEN_SEMANTIC_INTENTS = frozenset(
    {
        "go_home",
        "stop",
        "alarm_reset",
        "get_pose",
        "set_speed",
        "wait",
        "move_relative",
        "absolute_move_ptp",
        "move_named_pose",
        "absolute_move_lin",
        "circular_move",
        "move_joint",
        "move_joint_delta",
        "move_joints",
        "io_set",
        "draw_shape",
        "draw_text",
        "sequence",
        "return_to_start",
    }
)


@dataclass(frozen=True)
class ContractResult:
    valid: bool
    reason: str = ""
    hint: str = ""


def validate_semantic_ir_contract(
    payload: Any, *, _in_sequence_step: bool = False
) -> ContractResult:
    """Validate that *payload* is a clean Semantic IR dict suitable for IntentRouter.

    Reject criteria:
      - Not a dict.
      - Contains ``primitive_type`` (raw primitive leakage).
      - Contains ``raw_text`` (unparsed natural-language leakage).
      - Missing or empty ``intent`` field.
      - ``intent`` not in the frozen semantic intent set.
    """
    if not isinstance(payload, dict):
        return ContractResult(
            valid=False,
            reason="Semantic IR must be a JSON object.",
            hint="The LLM output was not a valid JSON object.",
        )

    forbidden_path = _find_forbidden_field(payload)
    if forbidden_path is not None:
        field_name = forbidden_path.rsplit(".", 1)[-1]
        if field_name == "primitive_type":
            return ContractResult(
                valid=False,
                reason=f"Semantic IR must not contain 'primitive_type' at {forbidden_path}.",
                hint="Direct primitive payloads are only accepted via the /llm_raw_command backward-compatibility path. "
                "Ensure the LLM outputs Semantic IR with an 'intent' field, not raw primitives.",
            )
        if field_name == "raw_text":
            return ContractResult(
                valid=False,
                reason=f"Semantic IR must not contain 'raw_text' at {forbidden_path}.",
                hint="The LLM echoed raw user text instead of producing structured output. "
                "Retry with clearer instructions or check LLM temperature/prompt.",
            )

    # Explicit error payloads from the LLM (MISSING_SLOT, UNSUPPORTED_OR_AMBIGUOUS, etc.)
    # are valid responses even though they lack an 'intent' field.
    if isinstance(payload.get("error"), str) and payload["error"]:
        return ContractResult(valid=True)

    intent = payload.get("intent")
    if not isinstance(intent, str) or not intent.strip():
        return ContractResult(
            valid=False,
            reason="Semantic IR must contain a non-empty 'intent' field.",
            hint="The LLM output is missing the required 'intent' field. "
            "Verify the prompt schema and LLM JSON mode are active.",
        )

    if intent not in _FROZEN_SEMANTIC_INTENTS:
        return ContractResult(
            valid=False,
            reason=f"Unsupported semantic intent: '{intent}'.",
            hint=f"Intent '{intent}' is not in the frozen semantic intent set. "
            f"Available intents: {', '.join(sorted(_FROZEN_SEMANTIC_INTENTS))}.",
        )

    if intent == "return_to_start" and not _in_sequence_step:
        return ContractResult(
            valid=False,
            reason="return_to_start is only valid inside a sequence.",
            hint="Use return_to_start only as a sequence step so the supervisor can capture the start joint snapshot.",
        )

    if intent == "sequence":
        steps = payload.get("steps")
        if not isinstance(steps, list) or not steps:
            return ContractResult(
                valid=False,
                reason="sequence intent requires a non-empty 'steps' list.",
                hint="Output sequence Semantic IR as {'intent':'sequence','steps':[...semantic steps...]}.",
            )
        for index, step in enumerate(steps):
            step_result = validate_semantic_ir_contract(step, _in_sequence_step=True)
            if not step_result.valid:
                return ContractResult(
                    valid=False,
                    reason=f"sequence step {index} failed Semantic IR contract: {step_result.reason}",
                    hint=step_result.hint,
                )

    return ContractResult(valid=True)


def _find_forbidden_field(value: Any, path: str = "$") -> str | None:
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = f"{path}.{key}"
            if key in {"primitive_type", "raw_text"}:
                return child_path
            found = _find_forbidden_field(child, child_path)
            if found is not None:
                return found
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found = _find_forbidden_field(child, f"{path}[{index}]")
            if found is not None:
                return found
    return None
