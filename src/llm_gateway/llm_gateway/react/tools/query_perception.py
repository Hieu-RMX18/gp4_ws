"""Perception query tool — stub for W4."""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext


class QueryPerceptionTool(Tool):
    name = "query_perception"
    description = "Query the perception system for object detections. (Stub — W4 will implement.)"
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "class_filter": {"type": "string", "description": "Optional object class to filter."},
        },
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        return ToolResult(
            ok=False,
            error="perception_not_yet_implemented",
            payload={"wave": "W4"},
        )
