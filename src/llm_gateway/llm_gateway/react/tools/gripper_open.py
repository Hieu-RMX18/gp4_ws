"""Gripper open tool — stub until gripper capability is wired."""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext


class GripperOpenTool(Tool):
    name = "gripper_open"
    description = "Open the robot gripper."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {},
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        return ToolResult(
            ok=False,
            error="capability_unavailable",
            payload={"capability": "gripper"},
        )
