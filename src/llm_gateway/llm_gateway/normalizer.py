"""Unit normalizer for poses and joints."""

from __future__ import annotations

import math
from typing import Any, Dict, List

from geometry_msgs.msg import Pose, Quaternion


def _to_float(value: Any, field: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be numeric.") from exc


def _is_likely_mm(position_xyz: List[float]) -> bool:
    # If any component is larger than 10 units, treat as millimeters.
    return any(abs(value) > 10.0 for value in position_xyz)


def _is_likely_degrees(angles: List[float]) -> bool:
    # If any angle magnitude exceeds 2*pi, treat all as degrees.
    return any(abs(value) > (2.0 * math.pi) for value in angles)


def _rpy_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Quaternion:
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def _normalize_orientation(raw_orientation: Dict[str, Any]) -> Quaternion:
    if not isinstance(raw_orientation, dict):
        raise ValueError("target_pose.orientation must be an object.")

    quaternion_fields = ("x", "y", "z", "w")
    if all(field in raw_orientation for field in quaternion_fields):
        return Quaternion(
            x=_to_float(raw_orientation.get("x"), "target_pose.orientation.x"),
            y=_to_float(raw_orientation.get("y"), "target_pose.orientation.y"),
            z=_to_float(raw_orientation.get("z"), "target_pose.orientation.z"),
            w=_to_float(raw_orientation.get("w"), "target_pose.orientation.w"),
        )

    rpy_fields = ("roll", "pitch", "yaw")
    if all(field in raw_orientation for field in rpy_fields):
        roll = _to_float(raw_orientation.get("roll"), "target_pose.orientation.roll")
        pitch = _to_float(raw_orientation.get("pitch"), "target_pose.orientation.pitch")
        yaw = _to_float(raw_orientation.get("yaw"), "target_pose.orientation.yaw")

        if _is_likely_degrees([roll, pitch, yaw]):
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)

        return _rpy_to_quaternion(roll, pitch, yaw)

    raise ValueError("target_pose.orientation must provide x/y/z/w or roll/pitch/yaw.")


def normalize_pose(raw_pose: Dict[str, Any]) -> Pose:
    """Convert position/orientation into geometry_msgs/Pose with unit detection."""
    if not isinstance(raw_pose, dict):
        raise ValueError("target_pose must be an object.")

    position = raw_pose.get("position")
    orientation = raw_pose.get("orientation")
    if not isinstance(position, dict):
        raise ValueError("target_pose.position must be an object.")
    if orientation is None:
        # LLM position-only intents can omit orientation; encode this as a
        # zero-quaternion sentinel so motion_core can preserve current tool
        # orientation deterministically at execution time.
        orientation = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "w": 0.0,
        }
    if not isinstance(orientation, dict):
        raise ValueError("target_pose.orientation must be an object.")

    x = _to_float(position.get("x"), "target_pose.position.x")
    y = _to_float(position.get("y"), "target_pose.position.y")
    z = _to_float(position.get("z"), "target_pose.position.z")

    if _is_likely_mm([x, y, z]):
        x /= 1000.0
        y /= 1000.0
        z /= 1000.0

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = _normalize_orientation(orientation)
    return pose


def normalize_joints(raw_joints: List[Any]) -> List[float]:
    """Normalize joint list; degrees are converted to radians when detected."""
    if not isinstance(raw_joints, list):
        raise ValueError("joint_target must be a list.")

    joints = [_to_float(value, f"joint_target[{index}]") for index, value in enumerate(raw_joints)]
    if _is_likely_degrees(joints):
        return [math.radians(value) for value in joints]
    return joints


class Normalizer:
    """Class wrapper used by node code."""

    _PLANNER_DEFAULTS = {
        "LIN": "PILZ_LIN",
        "PTP": "PILZ_PTP",
        "HOME": "PILZ_PTP",
    }

    def __init__(self, default_velocity_scale: float = 0.1, default_acceleration_scale: float = 0.1):
        self.default_velocity_scale = float(default_velocity_scale)
        self.default_acceleration_scale = float(default_acceleration_scale)

    def normalize_pose(self, raw_pose: Dict[str, Any]) -> Pose:
        return normalize_pose(raw_pose)

    def normalize_joints(self, raw_joints: List[Any]) -> List[float]:
        return normalize_joints(raw_joints)

    def normalize(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Compatibility helper: normalize optional fields in a command dict."""
        if not isinstance(command, dict):
            raise ValueError("command must be an object.")
        if "primitive_type" not in command:
            raise ValueError("Missing required field: primitive_type.")

        normalized = dict(command)
        if "target_pose" in normalized:
            normalized["target_pose_msg"] = normalize_pose(normalized["target_pose"])
        if "joint_target" in normalized:
            normalized["joint_target"] = normalize_joints(normalized["joint_target"])

        normalized.setdefault("velocity_scale", self.default_velocity_scale)
        normalized.setdefault("acceleration_scale", self.default_acceleration_scale)
        primitive_type = normalized["primitive_type"]
        normalized.setdefault(
            "planner_id",
            self._PLANNER_DEFAULTS.get(primitive_type, "OMPL_RRTConnect"),
        )
        # Default to explicit downstream approval because llm_gateway is not
        # the final safety authority for real hardware execution.
        normalized.setdefault("require_approval", True)
        return normalized
