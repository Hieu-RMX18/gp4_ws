"""Plan motion tool — validates a motion target through /validate_command."""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext


class PlanMotionTool(Tool):
    name = "plan_motion"
    description = (
        "Validate a planned motion target (PoseStamped or joint positions). "
        "Returns a plan_id if valid. Does NOT execute."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "target": {
                "oneOf": [
                    {
                        "type": "object",
                        "required": ["header", "pose"],
                        "properties": {
                            "header": {"type": "object"},
                            "pose": {"type": "object"},
                        },
                    },
                    {
                        "type": "object",
                        "required": ["joint_target"],
                        "properties": {
                            "joint_target": {
                                "type": "array",
                                "items": {"type": "number"},
                            },
                        },
                    },
                ],
            },
            "planner": {"type": "string"},
            "velocity_scale": {"type": "number", "minimum": 0.0, "maximum": 1.0},
            "acceleration_scale": {"type": "number", "minimum": 0.0, "maximum": 1.0},
        },
        "required": ["target"],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        node = getattr(context, "ros_node", None)
        if node is None:
            # Offline / test path: synthesize a plan_id without ROS.
            return ToolResult(
                ok=True,
                payload={
                    "plan_id": "test-plan-001",
                    "valid": True,
                    "estimated_duration_s": 2.0,
                },
            )

        client = getattr(node, "_validate_client", None)
        if client is None or not client.service_is_ready():
            return ToolResult(
                ok=False,
                error="validate_command service not available",
            )

        # Build a synthetic command dict for validation
        target = args["target"]
        command = {
            "primitive_type": "PTP" if "joint_target" in target else "LIN",
            "target_pose": target if "header" in target else None,
            "joint_target": target.get("joint_target"),
            "velocity_scale": float(args.get("velocity_scale", 0.1)),
            "acceleration_scale": float(args.get("acceleration_scale", 0.1)),
        }

        req = node._validate_client.RequestType()
        req.command = command  # type: ignore[attr-defined]
        future = client.call_async(req)
        rclpy = __import__("rclpy")
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if not future.done():
            return ToolResult(ok=False, error="validate_command timeout")

        resp = future.result()
        if resp is None or not getattr(resp, "valid", False):
            return ToolResult(
                ok=False,
                error="validate_command rejected the plan",
            )

        return ToolResult(
            ok=True,
            payload={
                "plan_id": "plan-" + str(hash(str(command)) % 10000),
                "valid": True,
                "estimated_duration_s": 2.0,
            },
        )
