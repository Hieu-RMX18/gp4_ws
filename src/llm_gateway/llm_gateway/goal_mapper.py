"""Mapping helpers from validated command dictionaries to ROS interfaces."""

from __future__ import annotations

from typing import Any, Dict

from geometry_msgs.msg import Pose
from interfaces.action import ExecuteMotion


class GoalMapper:
    """Create action goals and JSON-safe payloads from normalized commands."""

    def to_execute_motion_goal(self, command: Dict[str, Any]) -> ExecuteMotion.Goal:
        goal = ExecuteMotion.Goal()
        goal.primitive_type = str(command["primitive_type"])
        goal.velocity_scale = float(command["velocity_scale"])
        goal.acceleration_scale = float(command["acceleration_scale"])
        goal.planner_id = str(command["planner_id"])
        goal.require_approval = bool(command["require_approval"])
        goal.joint_target = list(command.get("joint_target", []))
        goal.target_pose = command.get("target_pose_msg", Pose())
        return goal

    def to_command_payload(self, command: Dict[str, Any]) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "primitive_type": str(command["primitive_type"]),
            "velocity_scale": float(command["velocity_scale"]),
            "acceleration_scale": float(command["acceleration_scale"]),
            "planner_id": str(command["planner_id"]),
            "require_approval": bool(command["require_approval"]),
        }
        if command.get("joint_target"):
            payload["joint_target"] = [float(value) for value in command["joint_target"]]
        if "target_pose_msg" in command:
            pose = command["target_pose_msg"]
            payload["target_pose"] = {
                "position": {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                },
                "orientation": {
                    "x": float(pose.orientation.x),
                    "y": float(pose.orientation.y),
                    "z": float(pose.orientation.z),
                    "w": float(pose.orientation.w),
                },
            }
        return payload
