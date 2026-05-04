"""State injector — pulls live robot state from ROS topics for ReAct prompt context."""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

_LOGGER = logging.getLogger(__name__)


class StateInjector:
    """Maintains a snapshot of robot state from ROS subscriptions.

    When ROS is not running (e.g. unit tests), the snapshot returns a
    minimal safe fallback so the ReAct agent can still reason.
    """

    _DEFAULT_JOINT_NAMES: List[str] = [
        "joint_1_s",
        "joint_2_l",
        "joint_3_u",
        "joint_4_r",
        "joint_5_b",
        "joint_6_t",
    ]

    def __init__(self) -> None:
        self._last_joint_states: Optional[Dict[str, Any]] = None
        self._last_robot_status: Optional[Dict[str, Any]] = None
        self._velocity_scale_active: float = 0.06
        self._gripper_available: bool = False
        self._perception_available: bool = False

    def update_joint_states(self, msg: Dict[str, Any]) -> None:
        self._last_joint_states = msg

    def update_robot_status(self, msg: Dict[str, Any]) -> None:
        self._last_robot_status = msg

    def set_velocity_scale(self, value: float) -> None:
        self._velocity_scale_active = float(value)

    def set_capabilities(self, *, gripper: bool, perception: bool) -> None:
        self._gripper_available = gripper
        self._perception_available = perception

    def snapshot(self) -> Dict[str, Any]:
        """Return a structured state dict for inclusion in the LLM prompt."""
        joints = self._last_joint_states or {}
        position = joints.get("position", [])
        if not position:
            position = [0.0] * len(self._DEFAULT_JOINT_NAMES)

        status = self._last_robot_status or {}
        mode = status.get("mode", "IDLE")
        active_alarms = status.get("active_alarms", [])

        return {
            "robot_state": {
                "joints_rad": list(position),
                "joint_names": list(self._DEFAULT_JOINT_NAMES),
                "mode": str(mode),
                "active_alarms": list(active_alarms),
                "last_action": {
                    "tool": "",
                    "status": "",
                    "error": None,
                },
                "velocity_scale_active": float(self._velocity_scale_active),
                "capabilities": {
                    "gripper": bool(self._gripper_available),
                    "perception": bool(self._perception_available),
                },
            }
        }
