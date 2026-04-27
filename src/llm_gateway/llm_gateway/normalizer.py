"""Unit normalizer for poses and joints."""

from __future__ import annotations

import logging
import math
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import Pose, Quaternion

_LOGGER = logging.getLogger(__name__)

# Explicit unit vocabulary accepted at the LLM/raw-command boundary.
_VALID_LINEAR_UNITS = {"m", "cm", "mm"}
_VALID_ANGULAR_UNITS = {"rad", "deg"}

# DEPRECATED: legacy implicit unit guessing. Set True ONLY for backward-compat
# testing or migration. Production must use explicit unit fields.
_COMPAT_UNIT_HEURISTIC = False


def _to_float(value: Any, field: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be numeric.") from exc


def _convert_linear(value: float, unit: Optional[str], field: str) -> float:
    """Convert a linear value to meters using an explicit unit hint."""
    if unit is None:
        return value  # already SI (meters)
    if unit == "m":
        return value
    if unit == "cm":
        return value / 100.0
    if unit == "mm":
        return value / 1000.0
    raise ValueError(f"{field}: unsupported linear_unit '{unit}'. Use one of {sorted(_VALID_LINEAR_UNITS)}.")


def _convert_angular(value: float, unit: Optional[str], field: str) -> float:
    """Convert an angular value to radians using an explicit unit hint."""
    if unit is None:
        return value  # already SI (radians)
    if unit == "rad":
        return value
    if unit == "deg":
        return math.radians(value)
    raise ValueError(f"{field}: unsupported angular_unit '{unit}'. Use one of {sorted(_VALID_ANGULAR_UNITS)}.")


def _is_likely_mm(position_xyz: List[float]) -> bool:
    # DEPRECATED: implicit heuristic. Only used when _COMPAT_UNIT_HEURISTIC is True.
    return any(abs(value) > 10.0 for value in position_xyz)


def _is_likely_degrees(angles: List[float]) -> bool:
    # DEPRECATED: implicit heuristic. Only used when _COMPAT_UNIT_HEURISTIC is True.
    return any(abs(value) > (2.0 * math.pi) for value in angles)


def _wrap_to_pi(angle_rad: float) -> float:
    """Normalize angle to (-pi, pi]. Example: 450deg -> 90deg."""
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    wrapped -= math.pi
    if wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def _normalize_single_joint_angle(
    raw_value: Any, field: str, angular_unit: Optional[str] = None,
) -> float:
    angle = _to_float(raw_value, field)
    if angular_unit is not None:
        angle = _convert_angular(angle, angular_unit, field)
    elif _COMPAT_UNIT_HEURISTIC and _is_likely_degrees([angle]):
        _LOGGER.warning("DEPRECATED heuristic: treating %s=%.4f as degrees.", field, angle)
        angle = math.radians(angle)
    return _wrap_to_pi(angle)


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


def _normalize_orientation(
    raw_orientation: Dict[str, Any], angular_unit: Optional[str] = None,
) -> Quaternion:
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

        if angular_unit is not None:
            roll = _convert_angular(roll, angular_unit, "orientation.roll")
            pitch = _convert_angular(pitch, angular_unit, "orientation.pitch")
            yaw = _convert_angular(yaw, angular_unit, "orientation.yaw")
        elif _COMPAT_UNIT_HEURISTIC and _is_likely_degrees([roll, pitch, yaw]):
            _LOGGER.warning("DEPRECATED heuristic: treating RPY as degrees.")
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)

        return _rpy_to_quaternion(roll, pitch, yaw)

    raise ValueError("target_pose.orientation must provide x/y/z/w or roll/pitch/yaw.")


def normalize_pose(
    raw_pose: Dict[str, Any],
    linear_unit: Optional[str] = None,
    angular_unit: Optional[str] = None,
) -> Pose:
    """Convert position/orientation into geometry_msgs/Pose.

    When linear_unit/angular_unit are provided, explicit conversion is used.
    When absent, values are assumed SI (meters/radians).
    Legacy heuristic only activates if _COMPAT_UNIT_HEURISTIC is True.
    """
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

    if linear_unit is not None:
        x = _convert_linear(x, linear_unit, "target_pose.position.x")
        y = _convert_linear(y, linear_unit, "target_pose.position.y")
        z = _convert_linear(z, linear_unit, "target_pose.position.z")
    elif _COMPAT_UNIT_HEURISTIC and _is_likely_mm([x, y, z]):
        _LOGGER.warning("DEPRECATED heuristic: treating position as mm.")
        x /= 1000.0
        y /= 1000.0
        z /= 1000.0

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = _normalize_orientation(orientation, angular_unit)
    return pose


def normalize_joints(
    raw_joints: List[Any], angular_unit: Optional[str] = None,
) -> List[float]:
    """Normalize joint list to radians in (-pi, pi].

    When angular_unit is provided, explicit conversion is used.
    When absent, values are assumed radians (SI).
    Legacy heuristic only activates if _COMPAT_UNIT_HEURISTIC is True.
    """
    if not isinstance(raw_joints, list):
        raise ValueError("joint_target must be a list.")

    joints = [_to_float(value, f"joint_target[{index}]") for index, value in enumerate(raw_joints)]
    if angular_unit is not None:
        joints = [_convert_angular(v, angular_unit, f"joint_target[{i}]") for i, v in enumerate(joints)]
    elif _COMPAT_UNIT_HEURISTIC and _is_likely_degrees(joints):
        _LOGGER.warning("DEPRECATED heuristic: treating joint_target as degrees.")
        joints = [math.radians(value) for value in joints]
    return [_wrap_to_pi(value) for value in joints]


class Normalizer:
    """Class wrapper used by node code."""

    _PLANNER_DEFAULTS = {
        "LIN": "PILZ_LIN",
        "PTP": "PILZ_PTP",
        "CIRC": "PILZ_CIRC",
        "CARTESIAN_PATH": "PILZ_LIN",
        "HOME": "PILZ_PTP",
        "MOVE_REL": "PILZ_LIN",
        "MOVE_JOINT": "PILZ_PTP",
        "MOVE_JOINTS": "PILZ_PTP",
    }

    def __init__(
        self,
        default_velocity_scale: float = 0.06,
        default_acceleration_scale: float = 0.06,
    ):
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
        primitive_type = normalized["primitive_type"]
        if "plan_only" in normalized:
            normalized["plan_only"] = bool(normalized["plan_only"])

        # Extract explicit unit hints from command boundary early. These are
        # consumed here and NOT propagated downstream (internal is SI-only).
        linear_unit = normalized.pop("linear_unit", None)
        angular_unit = normalized.pop("angular_unit", None)
        if linear_unit is not None and linear_unit not in _VALID_LINEAR_UNITS:
            raise ValueError(
                f"Unsupported linear_unit '{linear_unit}'. "
                f"Use one of {sorted(_VALID_LINEAR_UNITS)}.")
        if angular_unit is not None and angular_unit not in _VALID_ANGULAR_UNITS:
            raise ValueError(
                f"Unsupported angular_unit '{angular_unit}'. "
                f"Use one of {sorted(_VALID_ANGULAR_UNITS)}.")

        if primitive_type == "WAIT" and "wait_duration_sec" in normalized:
            normalized["wait_duration_sec"] = float(normalized["wait_duration_sec"])
        if primitive_type == "IO_SET":
            if "io_address" in normalized:
                normalized["io_address"] = int(normalized["io_address"])
            if "io_value" in normalized:
                normalized["io_value"] = int(normalized["io_value"])
        if primitive_type == "MOVE_JOINT":
            if "joint_index" in normalized:
                normalized["joint_index"] = int(normalized["joint_index"])
            if "joint_angle" in normalized:
                normalized["joint_angle"] = _normalize_single_joint_angle(
                    normalized["joint_angle"], "joint_angle",
                    angular_unit=angular_unit,
                )

        # Non-motion primitives bypass planner and velocity defaults.
        if primitive_type in {"ALARM_RESET", "STOP", "WAIT", "IO_SET", "GET_POSE"}:
            normalized.setdefault("reference_frame", "base_link")
            return normalized

        if "target_pose" in normalized:
            normalized["target_pose_msg"] = normalize_pose(
                normalized["target_pose"], linear_unit, angular_unit)
        if "joint_target" in normalized:
            normalized["joint_target"] = normalize_joints(
                normalized["joint_target"], angular_unit)

        # CARTESIAN_PATH: normalize each waypoint dict to Pose msg
        if primitive_type == "CARTESIAN_PATH" and "waypoints" in normalized:
            waypoints_msg = []
            for wp in normalized["waypoints"]:
                waypoints_msg.append(normalize_pose(wp, linear_unit, angular_unit))
            normalized["waypoints_msg"] = waypoints_msg

        # CIRC: normalize target_pose and auxiliary waypoint (exactly 1 required)
        if primitive_type == "CIRC":
            if "target_pose" not in normalized:
                raise ValueError("CIRC requires target_pose.")
            normalized["target_pose_msg"] = normalize_pose(
                normalized["target_pose"], linear_unit, angular_unit)
            if "waypoints" not in normalized:
                raise ValueError("CIRC requires waypoints.")
            if not isinstance(normalized["waypoints"], list):
                raise ValueError("CIRC waypoints must be a list.")
            if len(normalized["waypoints"]) != 1:
                raise ValueError("CIRC requires exactly 1 auxiliary waypoint.")
            waypoints_msg = [normalize_pose(normalized["waypoints"][0], linear_unit, angular_unit)]
            normalized["waypoints_msg"] = waypoints_msg

        # MOVE_REL: explicit linear_unit applies to delta fields when present.
        if normalized["primitive_type"] == "MOVE_REL":
            for field in ("delta_x", "delta_y", "delta_z"):
                if field in normalized:
                    normalized[field] = _to_float(normalized[field], field)
                    if linear_unit is not None:
                        normalized[field] = _convert_linear(normalized[field], linear_unit, field)
            normalized.setdefault("reference_frame", "base_link")

        normalized.setdefault("velocity_scale", self.default_velocity_scale)
        normalized.setdefault("acceleration_scale", self.default_acceleration_scale)
        normalized.setdefault(
            "planner_id",
            self._PLANNER_DEFAULTS.get(primitive_type, "OMPL_RRTConnect"),
        )
        return normalized
