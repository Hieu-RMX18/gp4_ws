"""Submit motion tool — sends an ExecuteMotion action goal."""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext


class SubmitMotionTool(Tool):
    name = "submit_motion"
    description = (
        "Submit a previously planned motion for execution. "
        "Requires a plan_id returned by plan_motion."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "plan_id": {"type": "string"},
        },
        "required": ["plan_id"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = getattr(context, "ros_node", None)
        if node is None:
            # Offline / test path
            return ToolResult(
                ok=True,
                payload={
                    "status": "SUBMITTED",
                    "goal_id": "test-goal-" + str(args["plan_id"]),
                },
            )

        action_client = getattr(node, "_execute_client", None)
        if action_client is None:
            return ToolResult(
                ok=False,
                error="execute_motion action client not available",
            )

        from interfaces.action import ExecuteMotion

        goal = ExecuteMotion.Goal()
        # In a real implementation we would look up the stored plan by plan_id.
        # For W3, we assume the node has a plan cache populated by plan_motion.
        plan_cache = getattr(node, "_react_plan_cache", {})
        stored = plan_cache.get(args["plan_id"])
        if stored is None:
            return ToolResult(
                ok=False,
                error=f"Unknown plan_id: {args['plan_id']}",
            )

        # Populate goal fields from the stored command dict
        for key, value in stored.items():
            if hasattr(goal, key):
                setattr(goal, key, value)

        # Send asynchronously
        future = action_client.send_goal_async(goal)
        return ToolResult(
            ok=True,
            payload={
                "status": "SUBMITTED",
                "goal_id": str(future),
            },
        )
