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
        goal.velocity_scale = float(command.get("velocity_scale", 0.0))
        goal.acceleration_scale = float(command.get("acceleration_scale", 0.0))
        goal.planner_id = str(command.get("planner_id", ""))
        goal.require_approval = bool(command.get("require_approval", True))
        goal.joint_target = list(command.get("joint_target", []))
        goal.target_pose = command.get("target_pose_msg", Pose())

        # MOVE_REL delta fields
        goal.delta_x = float(command.get("delta_x", 0.0))
        goal.delta_y = float(command.get("delta_y", 0.0))
        goal.delta_z = float(command.get("delta_z", 0.0))
        goal.reference_frame = str(command.get("reference_frame", ""))
        goal.wait_duration_sec = float(command.get("wait_duration_sec", 0.0))
        goal.joint_index = int(command.get("joint_index", 0))
        goal.joint_angle = float(command.get("joint_angle", 0.0))
        goal.io_address = int(command.get("io_address", 0))
        goal.io_value = int(command.get("io_value", 0))

        return goal

    def to_command_payload(self, command: Dict[str, Any]) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "primitive_type": str(command["primitive_type"]),
        }
        if "velocity_scale" in command:
            payload["velocity_scale"] = float(command["velocity_scale"])
        if "acceleration_scale" in command:
            payload["acceleration_scale"] = float(command["acceleration_scale"])
        if "planner_id" in command:
            payload["planner_id"] = str(command["planner_id"])
        if "require_approval" in command:
            payload["require_approval"] = bool(command["require_approval"])
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

        # MOVE_REL delta fields
        if command.get("primitive_type") == "MOVE_REL":
            for field in ("delta_x", "delta_y", "delta_z"):
                if field in command:
                    payload[field] = float(command[field])
            if "reference_frame" in command:
                payload["reference_frame"] = str(command["reference_frame"])

        if command.get("primitive_type") == "WAIT" and "wait_duration_sec" in command:
            payload["wait_duration_sec"] = float(command["wait_duration_sec"])

        if command.get("primitive_type") == "MOVE_JOINT":
            if "joint_index" in command:
                payload["joint_index"] = int(command["joint_index"])
            if "joint_angle" in command:
                payload["joint_angle"] = float(command["joint_angle"])

        if command.get("primitive_type") == "IO_SET":
            if "io_address" in command:
                payload["io_address"] = int(command["io_address"])
            if "io_value" in command:
                payload["io_value"] = int(command["io_value"])

        return payload
