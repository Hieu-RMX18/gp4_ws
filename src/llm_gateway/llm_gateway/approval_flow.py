"""Human-in-the-loop approval step for LLM commands."""

from __future__ import annotations

from typing import Any, Callable, Optional

from rclpy.node import Node


class ApprovalFlow:
    """
    CLI approval flow.
    A topic-based approval channel can be added in later phases.
    """

    def __init__(self, node: Optional[Node] = None, input_fn: Callable[[str], str] = input):
        self._node = node
        self._input_fn = input_fn

    def request_human_approval(self, command: Any) -> bool:
        if isinstance(command, dict):
            primitive_type = command.get("primitive_type", "UNKNOWN")
            velocity = command.get("velocity_scale", "N/A")
        else:
            primitive_type = str(command)
            velocity = "N/A"
        prompt = f"Approve {primitive_type} at velocity {velocity}? (y/n): "
        try:
            response = self._input_fn(prompt).strip().lower()
        except EOFError:
            return False
        return response in {"y", "yes"}
