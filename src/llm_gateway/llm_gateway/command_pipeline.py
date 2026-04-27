"""Pure command-transformation helpers extracted from LLMGatewayNode.

These helpers have no ROS2 dependencies and are unit-testable in isolation.
The node calls them with the right policy flags / injected dependencies.

Extracting them shrinks the node's surface area and clarifies which logic
is pure data transformation versus which logic is ROS-coupled orchestration.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Callable, Dict, Optional, Protocol


class _SchemaValidatorLike(Protocol):
    """Minimal protocol matching llm_gateway.schema_validator.SchemaValidator."""

    def validate(self, command: Dict[str, Any]) -> None: ...


def prepare_execution_command(
    normalized_command: Dict[str, Any],
    logger: Optional[logging.Logger] = None,
) -> Dict[str, Any]:
    """Return the command payload to dispatch to motion_core.

    ``plan_only`` is metadata only and is rejected instead of being converted
    into execution. Human approval is owned by the HMI supervisor layer.
    """
    del logger

    if normalized_command.get("plan_only"):
        raise ValueError(
            "plan_only_not_executable: plan_only commands are not executable by "
            "/execute_motion; use a plan-only review workflow instead."
        )

    execution_command = dict(normalized_command)
    return execution_command


def command_from_sanitized_json(
    sanitized_json: str,
    fallback_payload: Dict[str, Any],
    schema_validator: _SchemaValidatorLike,
) -> Dict[str, Any]:
    """Decode and re-validate a supervisor-sanitized JSON payload.

    Returns ``fallback_payload`` when ``sanitized_json`` is empty.
    Raises ``ValueError`` on non-object JSON or schema violations.
    """
    if not sanitized_json:
        return fallback_payload
    loaded = json.loads(sanitized_json)
    if not isinstance(loaded, dict):
        raise ValueError("sanitized_json must decode to a JSON object.")
    schema_validator.validate(loaded)
    return loaded


def hydrate_draw_workplane(
    payload: Dict[str, Any],
    fetch_current_pose: Callable[[str], Optional[Dict[str, Any]]],
) -> Dict[str, Any]:
    """Ensure tool-mode drawing payloads carry an explicit workplane origin.

    Only applies to ``draw_shape`` / ``draw_text`` intents with workplane
    mode ``tool``. The ``fetch_current_pose`` callable isolates ROS service
    calls so this function stays unit-testable.

    Raises ``ValueError`` when tool-mode hydration is required but the
    pose service is unavailable.
    """
    if not isinstance(payload, dict):
        return payload
    intent = str(payload.get("intent", "")).strip().lower()
    if intent not in {"draw_shape", "draw_text"}:
        return payload

    working_payload = dict(payload)
    workplane = working_payload.get("workplane")
    if workplane is None:
        workplane = {"mode": "tool"}
        working_payload["workplane"] = workplane
    if not isinstance(workplane, dict):
        return working_payload

    mode = str(workplane.get("mode", "base")).strip().lower()
    if mode != "tool":
        return working_payload
    if isinstance(workplane.get("origin"), dict):
        return working_payload
    if isinstance(working_payload.get("start_pose"), dict):
        return working_payload

    current_pose = fetch_current_pose("base_link")
    if current_pose is None:
        raise ValueError(
            "missing_workplane: tool mode requires current pose, "
            "but /get_current_pose is unavailable"
        )

    hydrated_workplane = dict(workplane)
    hydrated_workplane["origin"] = current_pose
    working_payload["workplane"] = hydrated_workplane
    return working_payload
