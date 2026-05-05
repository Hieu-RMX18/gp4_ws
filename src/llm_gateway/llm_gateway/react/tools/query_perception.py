"""Perception query tool — delegates to gp4_perception.query_perception_tool (W4)."""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext

# Attempt W4 import; fall back to stub error if gp4_perception is not built/installed.
try:
    from gp4_perception.query_perception_tool import query_perception

    _W4_AVAILABLE = True  # type: ignore[var-annotated]
except Exception:
    _W4_AVAILABLE = False


class QueryPerceptionTool(Tool):
    name = "query_perception"
    description = "Query the perception system for object detections."
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "class_filter": {
                "type": "string",
                "description": "Optional object class to filter.",
            },
        },
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        if not _W4_AVAILABLE:
            return ToolResult(
                ok=False,
                error="perception_not_available",
                payload={"hint": "gp4_perception package is not installed or built"},
            )
        result = query_perception(
            args=args,
            context_state=context.state_injector.snapshot(),
        )
        return ToolResult(
            ok=result["ok"],
            error=result.get("error"),
            payload=result.get("payload"),
        )
