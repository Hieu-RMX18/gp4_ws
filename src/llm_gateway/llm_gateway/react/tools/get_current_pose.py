"""Get current robot pose tool — calls existing ROS service."""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext


class GetCurrentPoseTool(Tool):
    name = "get_current_pose"
    description = "Get the current robot end-effector pose in the base_link frame."
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {},
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        # When running under a real ROS node, the pose client lives on the node.
        node = getattr(context, "ros_node", None)
        if node is None:
            return ToolResult(
                ok=False,
                error="ros_node not available in AgentContext",
            )

        client = getattr(node, "_get_pose_client", None)
        if client is None or not client.service_is_ready():
            return ToolResult(
                ok=False,
                error="get_current_pose service not available",
            )

        req = node._get_pose_client.RequestType()
        future = client.call_async(req)
        # Synchronous wait — acceptable for ReAct tool (no time.sleep).
        rclpy = __import__("rclpy")
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if not future.done():
            return ToolResult(ok=False, error="get_current_pose timeout")

        resp = future.result()
        if resp is None:
            return ToolResult(ok=False, error="get_current_pose returned None")

        pose = {
            "header": {"frame_id": "base_link"},
            "pose": {
                "position": {
                    "x": resp.pose.position.x,
                    "y": resp.pose.position.y,
                    "z": resp.pose.position.z,
                },
                "orientation": {
                    "x": resp.pose.orientation.x,
                    "y": resp.pose.orientation.y,
                    "z": resp.pose.orientation.z,
                    "w": resp.pose.orientation.w,
                },
            },
        }
        return ToolResult(ok=True, payload={"pose": pose})
