"""Mixin providing _normalize_command and related helpers for IntentResolutionService."""

from __future__ import annotations

import math
import re
from typing import Any

from ..domain.constants import GP4_JOINT_NAMES as JOINT_NAMES
from .intent_constants import (
    _ALLOWED_FIELDS_BY_PRIMITIVE,
    HARDWARE_WHITELIST,
    MOTION_PRIMITIVES,
    PLANNER_DEFAULTS,
    ROUTED_DRAW_METADATA_FIELDS,
    SUPPORTED_PRIMITIVES,
)


def _to_float(value: Any, field: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        from .intent_resolution import IntentResolutionError

        raise IntentResolutionError(f"{field} must be numeric.") from exc


def _to_int(value: Any, field: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        from .intent_resolution import IntentResolutionError

        raise IntentResolutionError(f"{field} must be an integer.") from exc


def _wrap_to_pi(angle_rad: float) -> float:
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    wrapped -= math.pi
    if wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def _normalize_angle(value: Any, field: str) -> float:
    raw = _to_float(value, field)
    if abs(raw) > 2.0 * math.pi:
        raw = math.radians(raw)
    return _wrap_to_pi(raw)


def _rpy_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> dict[str, float]:
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class IntentNormalizationMixin:
    """Normalization methods extracted from IntentResolutionService."""

    def _normalize_command(
        self,
        *,
        command: dict[str, Any],
        runtime_mode: str,
        allow_routed_draw_metadata: bool = False,
    ) -> tuple[dict[str, Any], list[str]]:
        from .intent_resolution import IntentResolutionError

        if allow_routed_draw_metadata:
            command = {
                key: value
                for key, value in command.items()
                if key not in ROUTED_DRAW_METADATA_FIELDS
            }
        primitive = str(command.get("primitive_type") or "").strip().upper()
        if not primitive:
            raise IntentResolutionError("missing primitive_type", missing_slots=["primitive_type"])
        if primitive not in SUPPORTED_PRIMITIVES:
            raise IntentResolutionError(
                f"primitive_type {primitive!r} is outside the supported ExecuteMotion policy.",
                rejected_fields=["primitive_type"],
            )
        if runtime_mode == "hardware" and primitive not in HARDWARE_WHITELIST:
            raise IntentResolutionError(
                f"primitive_type {primitive!r} is blocked by the hardware primitive whitelist.",
                rejected_fields=["primitive_type"],
            )

        allowed_fields = _ALLOWED_FIELDS_BY_PRIMITIVE[primitive]
        rejected_fields = sorted(key for key in command if key not in allowed_fields and key != "primitive_type")
        if rejected_fields:
            raise IntentResolutionError(
                f"{primitive} payload contains unsupported fields.",
                rejected_fields=rejected_fields,
            )

        normalized: dict[str, Any] = {"primitive_type": primitive}
        notes: list[str] = []

        if primitive in MOTION_PRIMITIVES:
            velocity_scale = _to_float(
                command.get("velocity_scale", self._default_velocity_scale),
                "velocity_scale",
            )
            velocity_clamped = _clamp(velocity_scale, self._min_velocity_scale, self._max_velocity_scale)
            if velocity_clamped != velocity_scale:
                notes.append(
                    f"velocity_scale clamped from {velocity_scale:.4f} to {velocity_clamped:.4f}."
                )
            normalized["velocity_scale"] = velocity_clamped

            acceleration_scale = _to_float(
                command.get("acceleration_scale", self._default_acceleration_scale),
                "acceleration_scale",
            )
            acceleration_clamped = _clamp(acceleration_scale, self._min_velocity_scale, self._max_acceleration_scale)
            if acceleration_clamped != acceleration_scale:
                notes.append(
                    f"acceleration_scale clamped from {acceleration_scale:.4f} to {acceleration_clamped:.4f}."
                )
            normalized["acceleration_scale"] = acceleration_clamped
            normalized["planner_id"] = str(command.get("planner_id") or PLANNER_DEFAULTS.get(primitive, "PILZ_PTP"))

        if primitive in {"HOME", "PTP", "LIN", "CIRC", "CARTESIAN_PATH", "MOVE_REL", "GET_POSE", "WAIT", "STOP", "IO_SET", "ALARM_RESET"}:
            normalized["reference_frame"] = str(command.get("reference_frame") or "base_link")
            if normalized["reference_frame"] != "base_link":
                raise IntentResolutionError(
                    f"{primitive} requires reference_frame='base_link'.",
                    rejected_fields=["reference_frame"],
                )

        if primitive in {"PTP", "LIN", "CIRC"}:
            if "target_pose" not in command:
                raise IntentResolutionError(
                    f"{primitive} requires target_pose.",
                    missing_slots=["target_pose"],
                )
            normalized["target_pose"] = self._normalize_pose(command["target_pose"], field_name="target_pose")

        if primitive == "PTP" and "joint_target" in command:
            normalized["joint_target"] = self._normalize_joint_target(command["joint_target"])
        if primitive == "PTP" and "target_pose" not in normalized and "joint_target" not in normalized:
            raise IntentResolutionError(
                "PTP requires target_pose or joint_target.",
                missing_slots=["target_pose|joint_target"],
            )

        if primitive == "CIRC":
            if "waypoints" not in command:
                raise IntentResolutionError("CIRC requires waypoints.", missing_slots=["waypoints[0]"])
            waypoints = command.get("waypoints")
            if not isinstance(waypoints, list) or len(waypoints) != 1:
                raise IntentResolutionError("CIRC requires exactly one auxiliary waypoint.", missing_slots=["waypoints[0]"])
            normalized["waypoints"] = [
                self._normalize_pose(waypoints[0], field_name="waypoints[0]")
            ]

        if primitive == "CARTESIAN_PATH":
            waypoints = command.get("waypoints")
            if not isinstance(waypoints, list) or not waypoints:
                raise IntentResolutionError(
                    "CARTESIAN_PATH requires non-empty waypoints.",
                    missing_slots=["waypoints[0..n]"],
                )
            normalized["waypoints"] = [
                self._normalize_pose(waypoint, field_name=f"waypoints[{index}]")
                for index, waypoint in enumerate(waypoints)
            ]

        if primitive == "MOVE_REL":
            missing_delta = [
                field
                for field in ("delta_x", "delta_y", "delta_z")
                if field not in command
            ]
            if missing_delta:
                raise IntentResolutionError(
                    "MOVE_REL requires delta_x, delta_y, and delta_z.",
                    missing_slots=missing_delta,
                )
            normalized["delta_x"] = _to_float(command.get("delta_x"), "delta_x")
            normalized["delta_y"] = _to_float(command.get("delta_y"), "delta_y")
            normalized["delta_z"] = _to_float(command.get("delta_z"), "delta_z")
            if (
                normalized["delta_x"] == 0.0
                and normalized["delta_y"] == 0.0
                and normalized["delta_z"] == 0.0
            ):
                raise IntentResolutionError("MOVE_REL requires at least one non-zero delta component.")

        if primitive == "MOVE_JOINT":
            if "joint_index" not in command:
                raise IntentResolutionError("MOVE_JOINT requires joint_index.", missing_slots=["joint_index"])
            if "joint_angle" not in command:
                raise IntentResolutionError("MOVE_JOINT requires joint_angle.", missing_slots=["joint_angle"])
            joint_index = _to_int(command.get("joint_index"), "joint_index")
            if joint_index < 0 or joint_index >= len(JOINT_NAMES):
                raise IntentResolutionError("MOVE_JOINT joint_index must be between 0 and 5.", rejected_fields=["joint_index"])
            normalized["joint_index"] = joint_index
            normalized["joint_angle"] = _normalize_angle(command.get("joint_angle"), "joint_angle")

        if primitive == "MOVE_JOINTS":
            if "joint_target" not in command:
                raise IntentResolutionError("MOVE_JOINTS requires joint_target.", missing_slots=["joint_target"])
            normalized["joint_target"] = self._normalize_joint_target(command.get("joint_target"))

        if primitive == "WAIT":
            if "wait_duration_sec" not in command:
                raise IntentResolutionError("WAIT requires wait_duration_sec.", missing_slots=["wait_duration_sec"])
            wait_duration_sec = _to_float(command.get("wait_duration_sec"), "wait_duration_sec")
            if wait_duration_sec < 0.0:
                raise IntentResolutionError("WAIT wait_duration_sec must be >= 0.", rejected_fields=["wait_duration_sec"])
            if wait_duration_sec > 60.0:
                raise IntentResolutionError("WAIT wait_duration_sec must be <= 60.", rejected_fields=["wait_duration_sec"])
            normalized["wait_duration_sec"] = wait_duration_sec

        if primitive == "SET_SPEED":
            if "velocity_scale" not in command:
                raise IntentResolutionError("SET_SPEED requires velocity_scale.", missing_slots=["velocity_scale"])
            speed_raw = _to_float(command.get("velocity_scale"), "velocity_scale")
            speed_clamped = _clamp(speed_raw, self._min_velocity_scale, self._max_velocity_scale)
            if speed_clamped != speed_raw:
                notes.append(f"SET_SPEED velocity_scale clamped from {speed_raw:.4f} to {speed_clamped:.4f}.")
            normalized["velocity_scale"] = speed_clamped

        if primitive == "IO_SET":
            missing_io = [field for field in ("io_address", "io_value") if field not in command]
            if missing_io:
                raise IntentResolutionError("IO_SET requires io_address and io_value.", missing_slots=missing_io)
            io_address = _to_int(command.get("io_address"), "io_address")
            io_value = _to_int(command.get("io_value"), "io_value")
            if io_address < 0:
                raise IntentResolutionError("IO_SET io_address must be >= 0.", rejected_fields=["io_address"])
            if io_value not in {0, 1}:
                raise IntentResolutionError("IO_SET io_value must be 0 or 1.", rejected_fields=["io_value"])
            normalized["io_address"] = io_address
            normalized["io_value"] = io_value

        return normalized, notes

    def _normalize_joint_target(self, value: Any) -> list[float]:
        from .intent_resolution import IntentResolutionError

        if not isinstance(value, list):
            raise IntentResolutionError("joint_target must be an array of 6 values.", missing_slots=["joint_target"])
        if len(value) != len(JOINT_NAMES):
            raise IntentResolutionError(
                "joint_target must include exactly 6 values.",
                missing_slots=["joint_target[0..5]"],
            )
        return [_normalize_angle(entry, f"joint_target[{index}]") for index, entry in enumerate(value)]

    def _normalize_pose(self, value: Any, *, field_name: str) -> dict[str, Any]:
        from .intent_resolution import IntentResolutionError

        if not isinstance(value, dict):
            raise IntentResolutionError(f"{field_name} must be an object.")
        position = value.get("position")
        if not isinstance(position, dict):
            raise IntentResolutionError(f"{field_name}.position must be an object.", missing_slots=[f"{field_name}.position"])

        x = _to_float(position.get("x"), f"{field_name}.position.x")
        y = _to_float(position.get("y"), f"{field_name}.position.y")
        z = _to_float(position.get("z"), f"{field_name}.position.z")
        if any(abs(component) > 10.0 for component in (x, y, z)):
            x /= 1000.0
            y /= 1000.0
            z /= 1000.0

        orientation_payload = value.get("orientation")
        if orientation_payload is None:
            orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
        elif not isinstance(orientation_payload, dict):
            raise IntentResolutionError(f"{field_name}.orientation must be an object.")
        elif {"x", "y", "z", "w"}.issubset(set(orientation_payload.keys())):
            orientation = {
                "x": _to_float(orientation_payload.get("x"), f"{field_name}.orientation.x"),
                "y": _to_float(orientation_payload.get("y"), f"{field_name}.orientation.y"),
                "z": _to_float(orientation_payload.get("z"), f"{field_name}.orientation.z"),
                "w": _to_float(orientation_payload.get("w"), f"{field_name}.orientation.w"),
            }
        elif {"roll", "pitch", "yaw"}.issubset(set(orientation_payload.keys())):
            roll = _to_float(orientation_payload.get("roll"), f"{field_name}.orientation.roll")
            pitch = _to_float(orientation_payload.get("pitch"), f"{field_name}.orientation.pitch")
            yaw = _to_float(orientation_payload.get("yaw"), f"{field_name}.orientation.yaw")
            if any(abs(component) > (2.0 * math.pi) for component in (roll, pitch, yaw)):
                roll = math.radians(roll)
                pitch = math.radians(pitch)
                yaw = math.radians(yaw)
            orientation = _rpy_to_quaternion(roll, pitch, yaw)
        else:
            raise IntentResolutionError(
                f"{field_name}.orientation must provide quaternion (x,y,z,w) or RPY (roll,pitch,yaw)."
            )

        return {
            "position": {"x": x, "y": y, "z": z},
            "orientation": orientation,
        }

    def _resolve_joint_index(self, parameters: dict[str, Any]) -> int | None:
        raw_zero_based = parameters.get("jointIndexZeroBased")
        if raw_zero_based is not None:
            index = _to_int(raw_zero_based, "jointIndexZeroBased")
            if 0 <= index < len(JOINT_NAMES):
                return index

        raw_joint_index = parameters.get("jointIndex")
        if raw_joint_index is not None:
            index = _to_int(raw_joint_index, "jointIndex")
            if 0 <= index < len(JOINT_NAMES):
                return index
            if 1 <= index <= len(JOINT_NAMES):
                return index - 1

        raw_joint_name = str(parameters.get("jointNameResolved") or parameters.get("joint") or "").strip().lower()
        if raw_joint_name:
            if raw_joint_name in JOINT_NAMES:
                return JOINT_NAMES.index(raw_joint_name)
            match = re.fullmatch(r"joint[_\s-]*([1-6])(?:[_\s-].+)?", raw_joint_name)
            if match:
                return int(match.group(1)) - 1

        return None

    def _read_joint_deg(self, *, current_joints: list[Any], joint_index: int) -> float | None:
        target_name = JOINT_NAMES[joint_index]
        for joint in current_joints:
            if getattr(joint, "name", None) == target_name:
                position_deg = getattr(joint, "position_deg", None)
                if position_deg is None:
                    return None
                return float(position_deg)
        return None

    def _target_summary(self, normalized_command: dict[str, Any]) -> str:
        primitive = normalized_command["primitive_type"]
        if primitive == "HOME":
            return "Return robot to configured home pose."
        if primitive == "STOP":
            return "Request supervised stop handling."
        if primitive == "MOVE_REL":
            return (
                "Relative translation in base_link: "
                f"dx={float(normalized_command.get('delta_x', 0.0)) * 1000.0:.1f} mm "
                f"dy={float(normalized_command.get('delta_y', 0.0)) * 1000.0:.1f} mm "
                f"dz={float(normalized_command.get('delta_z', 0.0)) * 1000.0:.1f} mm."
            )
        if primitive == "MOVE_JOINT":
            return (
                "Move single joint target: "
                f"joint_index={normalized_command.get('joint_index')} "
                f"joint_angle={normalized_command.get('joint_angle'):.4f} rad."
            )
        if primitive == "MOVE_JOINTS":
            return "Move all six joints to absolute targets."
        if primitive == "WAIT":
            return f"Pause execution for {normalized_command.get('wait_duration_sec', 0.0):.2f} s."
        if primitive == "SET_SPEED":
            return f"Set default velocity scale to {normalized_command.get('velocity_scale', 0.0):.4f}."
        if primitive == "IO_SET":
            return (
                f"Set controller IO address {normalized_command.get('io_address')} "
                f"to {normalized_command.get('io_value')}."
            )
        if primitive == "ALARM_RESET":
            return "Request alarm reset at execution boundary."
        if primitive == "GET_POSE":
            return "Query current robot TCP pose."
        if primitive == "LIN":
            return "Linear motion to target pose."
        if primitive == "PTP":
            return "Point-to-point motion to target pose or joint target."
        if primitive == "CIRC":
            return "Circular motion using auxiliary waypoint."
        if primitive == "CARTESIAN_PATH":
            return "Cartesian multi-waypoint path execution."
        return f"Execute primitive {primitive}."
