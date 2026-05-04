"""Wait for robot state tool — polls robot status topic."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext


class WaitForStateTool(Tool):
    name = "wait_for_state"
    description = "Wait until the robot reaches a given state (IDLE, MOVING, PLANNING, FAULT)."
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "state": {
                "type": "string",
                "enum": ["IDLE", "MOVING", "PLANNING", "FAULT"],
            },
            "timeout_s": {
                "type": "number",
                "minimum": 0.0,
                "maximum": 60.0,
            },
        },
        "required": ["state", "timeout_s"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        target = str(args["state"])
        timeout_s = float(args["timeout_s"])
        # Single-shot snapshot check.  Full polling with blocking wait
        # belongs in the ROS executor thread; tools must not time.sleep.
        snapshot = context.state_injector.snapshot()
        current = snapshot.get("robot_state", {}).get("mode", "IDLE")
        if current == target:
            return ToolResult(
                ok=True,
                payload={"reached": True, "current_state": current, "elapsed_s": 0.0},
            )
        return ToolResult(
            ok=False,
            error="state_not_reached",
            payload={
                "reached": False,
                "current_state": current,
                "timeout_s": timeout_s,
            },
        )
