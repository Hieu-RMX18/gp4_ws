"""Set speed tool — validates and records velocity_scale for future motion."""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext


class SetSpeedTool(Tool):
    name = "set_speed"
    description = "Set the global velocity scale (0.0–1.0). Affects future motion commands."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "velocity_scale": {
                "type": "number",
                "minimum": 0.0,
                "maximum": 1.0,
            },
        },
        "required": ["velocity_scale"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        velocity_scale = float(args["velocity_scale"])
        if not (0.0 <= velocity_scale <= 1.0):
            return ToolResult(
                ok=False,
                error=f"velocity_scale {velocity_scale} out of range [0.0, 1.0]",
            )
        context.state_injector.set_velocity_scale(velocity_scale)
        return ToolResult(
            ok=True,
            payload={"applied": True, "velocity_scale": velocity_scale},
        )
