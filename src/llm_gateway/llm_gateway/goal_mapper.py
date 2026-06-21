"""Goal mapper from normalized commands to ROS action goals or command payloads.

Extracted from factory_task.py (R1) — behavior-preserving relocation.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional
from llm_gateway.normalization import _import_geometry_msgs


class GoalMapper:
    """Create action goals and JSON-safe payloads from normalized commands."""

    def __init__(
        self,
        *,
        default_velocity_scale: float = 0.06,
        default_acceleration_scale: float = 0.06,
    ) -> None:
        self._default_velocity_scale = float(default_velocity_scale)
        self._default_acceleration_scale = float(default_acceleration_scale)

    @staticmethod
    def _pose_to_payload(pose: Any) -> Dict[str, Any]:
        return {
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

    def to_execute_motion_goal(self, command: Dict[str, Any]) -> Any:
        from interfaces.action import ExecuteMotion

        Pose, _ = _import_geometry_msgs()

        goal = ExecuteMotion.Goal()
        goal.primitive_type = str(command["primitive_type"])
        goal.velocity_scale = float(
            command.get("velocity_scale", self._default_velocity_scale)
        )
        goal.acceleration_scale = float(
            command.get("acceleration_scale", self._default_acceleration_scale)
        )
        goal.planner_id = str(command.get("planner_id", ""))
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

        # CARTESIAN_PATH: multi-waypoint smooth trajectory
        if command.get("waypoints_msg"):
            goal.waypoints = list(command["waypoints_msg"])

        # BLENDED_SEQUENCE: typed sequence steps (W2.T4)
        if command.get("primitive_type") == "BLENDED_SEQUENCE" and command.get(
            "sequence_steps"
        ):
            from interfaces.msg import SequenceStep

            goal.sequence_steps = []
            for step in command["sequence_steps"]:
                seq = SequenceStep()
                seq.primitive_type = str(step.get("primitive_type", "LIN"))
                if "target_pose_msg" in step:
                    seq.target_pose = step["target_pose_msg"]
                seq.blend_radius_m = float(step.get("blend_radius_m", 0.0))
                seq.planner_id = str(step.get("planner_id", "PILZ_LIN"))
                seq.velocity_scale = float(
                    step.get("velocity_scale", self._default_velocity_scale)
                )
                seq.acceleration_scale = float(
                    step.get("acceleration_scale", self._default_acceleration_scale)
                )
                goal.sequence_steps.append(seq)

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
        if "plan_only" in command:
            payload["plan_only"] = bool(command["plan_only"])
        if "chunk_index" in command:
            payload["chunk_index"] = int(command["chunk_index"])
        if "stroke_index" in command:
            payload["stroke_index"] = int(command["stroke_index"])
        if "reference_frame" in command:
            payload["reference_frame"] = str(command["reference_frame"])
        if command.get("joint_target"):
            payload["joint_target"] = [
                float(value) for value in command["joint_target"]
            ]
        if "target_pose_msg" in command:
            payload["target_pose"] = self._pose_to_payload(command["target_pose_msg"])

        # MOVE_REL delta fields
        if command.get("primitive_type") == "MOVE_REL":
            for field in ("delta_x", "delta_y", "delta_z"):
                if field in command:
                    payload[field] = float(command[field])

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

        # CIRC safety path requires the auxiliary waypoint in command_json so
        # safety.execution_gate can validate it against WorkspaceGuard.
        if command.get("primitive_type") == "CIRC" and command.get("waypoints_msg"):
            payload["waypoints"] = [
                self._pose_to_payload(pose) for pose in command["waypoints_msg"]
            ]

        # CARTESIAN_PATH waypoints are required by safety.execution_gate for
        # waypoint-by-waypoint workspace validation.
        if command.get("primitive_type") == "CARTESIAN_PATH" and command.get(
            "waypoints_msg"
        ):
            payload["waypoints"] = [
                self._pose_to_payload(pose) for pose in command["waypoints_msg"]
            ]
            payload["waypoints_count"] = len(command["waypoints_msg"])

        # BLENDED_SEQUENCE: include sequence_steps for safety gate (W2.T4)
        if command.get("primitive_type") == "BLENDED_SEQUENCE" and command.get(
            "sequence_steps"
        ):
            payload["sequence_steps"] = []
            for step in command["sequence_steps"]:
                step_payload: Dict[str, Any] = {
                    "primitive_type": str(step.get("primitive_type", "LIN")),
                    "blend_radius_m": float(step.get("blend_radius_m", 0.0)),
                }
                if "target_pose_msg" in step:
                    step_payload["target_pose"] = self._pose_to_payload(
                        step["target_pose_msg"]
                    )
                if "planner_id" in step:
                    step_payload["planner_id"] = str(step["planner_id"])
                if "velocity_scale" in step:
                    step_payload["velocity_scale"] = float(step["velocity_scale"])
                if "acceleration_scale" in step:
                    step_payload["acceleration_scale"] = float(
                        step["acceleration_scale"]
                    )
                payload["sequence_steps"].append(step_payload)

        return payload
