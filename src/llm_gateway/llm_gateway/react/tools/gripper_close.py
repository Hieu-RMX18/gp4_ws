"""Gripper close tool — stub until gripper capability is wired."""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext


class GripperCloseTool(Tool):
    name = "gripper_close"
    description = "Close the robot gripper."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "force": {"type": "number", "description": "Optional closing force (N)."},
        },
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        return ToolResult(
            ok=False,
            error="capability_unavailable",
            payload={"capability": "gripper"},
        )
