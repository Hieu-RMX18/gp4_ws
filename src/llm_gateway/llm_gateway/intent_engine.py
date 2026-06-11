"""Intent routing, normalization, and validation helpers."""

from __future__ import annotations

from llm_gateway.llm_payload_parser import LLMParser, parse_llm_output

"""Unit normalizer for poses and joints."""


import json
import logging
import math
from typing import List, Optional

_LOGGER = logging.getLogger(__name__)


def _import_geometry_msgs():
    """Lazy import geometry_msgs types.

    geometry_msgs is a ROS 2 package that is unavailable when
    llm_gateway is imported from the source tree without a full
    ROS 2 environment (e.g. HMI backend tests with PYTHONPATH=hmi/backend).

    When the real types are unavailable this returns SimpleNamespace-based
    stubs so normalizers and validators can still construct pose objects.
    """
    try:
        from geometry_msgs.msg import Pose as _Pose, Quaternion as _Quaternion
    except ImportError:
        from types import SimpleNamespace

        def _make_quaternion(*, x=0.0, y=0.0, z=0.0, w=1.0):
            return SimpleNamespace(x=x, y=y, z=z, w=w)

        def _make_pose():
            return SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=_make_quaternion(),
            )

        return _make_pose, _make_quaternion

    return _Pose, _Quaternion

# Explicit unit vocabulary accepted at the LLM/raw-command boundary.
_VALID_LINEAR_UNITS = {"m", "cm", "mm"}
_VALID_ANGULAR_UNITS = {"rad", "deg"}

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
    raise ValueError(
        f"{field}: unsupported linear_unit '{unit}'. Use one of {sorted(_VALID_LINEAR_UNITS)}."
    )


def _convert_angular(value: float, unit: Optional[str], field: str) -> float:
    """Convert an angular value to radians using an explicit unit hint."""
    if unit is None:
        return value  # already SI (radians)
    if unit == "rad":
        return value
    if unit == "deg":
        return math.radians(value)
    raise ValueError(
        f"{field}: unsupported angular_unit '{unit}'. Use one of {sorted(_VALID_ANGULAR_UNITS)}."
    )


def _is_likely_mm(position_xyz: List[float]) -> bool:
    return any(abs(value) > 10.0 for value in position_xyz)


def _is_likely_degrees(angles: List[float]) -> bool:
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
    raw_value: Any,
    field: str,
    angular_unit: Optional[str] = None,
) -> float:
    angle = _to_float(raw_value, field)
    if angular_unit is not None:
        angle = _convert_angular(angle, angular_unit, field)
    elif _COMPAT_UNIT_HEURISTIC and _is_likely_degrees([angle]):
        _LOGGER.warning("Legacy heuristic: treating %s=%.4f as degrees.", field, angle)
        angle = math.radians(angle)
    return _wrap_to_pi(angle)


def _rpy_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> "Quaternion":
    _, Quaternion = _import_geometry_msgs()
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
    raw_orientation: Dict[str, Any],
    angular_unit: Optional[str] = None,
) -> "Quaternion":
    _, Quaternion = _import_geometry_msgs()
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
            _LOGGER.warning("Legacy heuristic: treating RPY as degrees.")
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)

        return _rpy_to_quaternion(roll, pitch, yaw)

    raise ValueError("target_pose.orientation must provide x/y/z/w or roll/pitch/yaw.")


def normalize_pose(
    raw_pose: Dict[str, Any],
    linear_unit: Optional[str] = None,
    angular_unit: Optional[str] = None,
) -> "Pose":
    """Convert position/orientation into geometry_msgs/Pose.

    When linear_unit/angular_unit are provided, explicit conversion is used.
    When absent, values are assumed SI (meters/radians).
    Legacy heuristic only activates if _COMPAT_UNIT_HEURISTIC is True.
    """
    Pose, _ = _import_geometry_msgs()
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
        _LOGGER.warning("Legacy heuristic: treating position as mm.")
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
    raw_joints: List[Any],
    angular_unit: Optional[str] = None,
) -> List[float]:
    """Normalize joint list to radians in (-pi, pi].

    When angular_unit is provided, explicit conversion is used.
    When absent, values are assumed radians (SI).
    Legacy heuristic only activates if _COMPAT_UNIT_HEURISTIC is True.
    """
    if not isinstance(raw_joints, list):
        raise ValueError("joint_target must be a list.")

    joints = [
        _to_float(value, f"joint_target[{index}]")
        for index, value in enumerate(raw_joints)
    ]
    if angular_unit is not None:
        joints = [
            _convert_angular(v, angular_unit, f"joint_target[{i}]")
            for i, v in enumerate(joints)
        ]
    elif _COMPAT_UNIT_HEURISTIC and _is_likely_degrees(joints):
        _LOGGER.warning("Legacy heuristic: treating joint_target as degrees.")
        joints = [math.radians(value) for value in joints]
    return [_wrap_to_pi(value) for value in joints]


class Normalizer:
    """Class wrapper used by node code."""

    _PLANNER_DEFAULTS = {
        "LIN": "PILZ_LIN",
        "PTP": "PILZ_PTP",
        "CIRC": "PILZ_CIRC",
        "CARTESIAN_PATH": "PILZ_LIN",
        "BLENDED_SEQUENCE": "PILZ_LIN",
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

    def normalize_pose(self, raw_pose: Dict[str, Any]) -> "Pose":
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
                f"Use one of {sorted(_VALID_LINEAR_UNITS)}."
            )
        if angular_unit is not None and angular_unit not in _VALID_ANGULAR_UNITS:
            raise ValueError(
                f"Unsupported angular_unit '{angular_unit}'. "
                f"Use one of {sorted(_VALID_ANGULAR_UNITS)}."
            )

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
                    normalized["joint_angle"],
                    "joint_angle",
                    angular_unit=angular_unit,
                )

        # Non-motion primitives bypass planner and velocity defaults.
        if primitive_type in {
            "ALARM_RESET",
            "STOP",
            "WAIT",
            "IO_SET",
            "GET_POSE",
            "MACRO",
        }:
            normalized.setdefault("reference_frame", "base_link")
            return normalized

        if "target_pose" in normalized:
            normalized["target_pose_msg"] = normalize_pose(
                normalized["target_pose"], linear_unit, angular_unit
            )
        if "joint_target" in normalized:
            normalized["joint_target"] = normalize_joints(
                normalized["joint_target"], angular_unit
            )

        # CARTESIAN_PATH: normalize each waypoint dict to Pose msg
        if primitive_type == "CARTESIAN_PATH" and "waypoints" in normalized:
            waypoints_msg = []
            for wp in normalized["waypoints"]:
                waypoints_msg.append(normalize_pose(wp, linear_unit, angular_unit))
            normalized["waypoints_msg"] = waypoints_msg

        # BLENDED_SEQUENCE: normalize each step's target_pose (W2.T3)
        if primitive_type == "BLENDED_SEQUENCE" and "sequence_steps" in normalized:
            norm_steps = []
            for step in normalized["sequence_steps"]:
                ns = dict(step)
                if "target_pose" in ns:
                    ns["target_pose_msg"] = normalize_pose(
                        ns["target_pose"], linear_unit, angular_unit
                    )
                step_ptype = ns.get("primitive_type", "LIN")
                ns.setdefault("velocity_scale", self.default_velocity_scale)
                ns.setdefault("acceleration_scale", self.default_acceleration_scale)
                ns.setdefault(
                    "planner_id",
                    self._PLANNER_DEFAULTS.get(step_ptype, "PILZ_LIN"),
                )
                norm_steps.append(ns)
            normalized["sequence_steps"] = norm_steps

        # CIRC: normalize target_pose and auxiliary waypoint (exactly 1 required)
        if primitive_type == "CIRC":
            if "target_pose" not in normalized:
                raise ValueError("CIRC requires target_pose.")
            normalized["target_pose_msg"] = normalize_pose(
                normalized["target_pose"], linear_unit, angular_unit
            )
            if "waypoints" not in normalized:
                raise ValueError("CIRC requires waypoints.")
            if not isinstance(normalized["waypoints"], list):
                raise ValueError("CIRC waypoints must be a list.")
            if len(normalized["waypoints"]) != 1:
                raise ValueError("CIRC requires exactly 1 auxiliary waypoint.")
            waypoints_msg = [
                normalize_pose(normalized["waypoints"][0], linear_unit, angular_unit)
            ]
            normalized["waypoints_msg"] = waypoints_msg

        # MOVE_REL: explicit linear_unit applies to delta fields when present.
        if normalized["primitive_type"] == "MOVE_REL":
            for field in ("delta_x", "delta_y", "delta_z"):
                if field in normalized:
                    normalized[field] = _to_float(normalized[field], field)
                    if linear_unit is not None:
                        normalized[field] = _convert_linear(
                            normalized[field], linear_unit, field
                        )
            normalized.setdefault("reference_frame", "base_link")

        normalized.setdefault("velocity_scale", self.default_velocity_scale)
        normalized.setdefault("acceleration_scale", self.default_acceleration_scale)
        normalized.setdefault(
            "planner_id",
            self._PLANNER_DEFAULTS.get(primitive_type, "OMPL_RRTConnect"),
        )
        return normalized


"""JSON schema validator for LLM command payloads."""


import os
from pathlib import Path
from typing import Tuple

import jsonschema
import yaml
try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None  # type: ignore[assignment]


def _default_schema_path() -> str:
    """Resolve llm_schema.yaml from installed package or local source tree."""
    try:
        pkg_share = get_package_share_directory("llm_gateway")
        return os.path.join(pkg_share, "config", "llm_schema.yaml")
    except Exception:
        # Fallback for direct source-tree execution in tests/tools.
        return str(Path(__file__).resolve().parents[1] / "config" / "llm_schema.yaml")


def _load_schema(schema_path: str) -> Dict[str, Any]:
    with open(schema_path, "r", encoding="utf-8") as schema_file:
        if schema_path.endswith((".yaml", ".yml")):
            schema = yaml.safe_load(schema_file)
        else:
            schema = json.load(schema_file)
    if not isinstance(schema, dict):
        raise ValueError("Schema root must be a JSON object.")
    return schema


class SchemaValidator:
    """Load and validate command dicts against the phase-9 LLM schema."""

    def __init__(self, schema_path: str | None = None):
        self._schema_path = schema_path or _default_schema_path()
        self._schema = _load_schema(self._schema_path)

    def validate_against_schema(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        """Return (True, '') when valid, otherwise (False, detailed_error)."""
        try:
            jsonschema.validate(instance=data, schema=self._schema)
            return True, ""
        except jsonschema.ValidationError as exc:
            path = ".".join(str(item) for item in exc.path)
            if path:
                return False, f"{path}: {exc.message}"
            return False, exc.message
        except Exception as exc:
            return False, str(exc)

    def validate(self, data: Dict[str, Any]) -> bool:
        """Compatibility helper for code paths expecting exceptions on failure."""
        valid, error = self.validate_against_schema(data)
        if not valid:
            raise ValueError(error)
        return True

    def schema_as_json(self) -> str:
        """Return the loaded schema in compact JSON form for prompt injection."""
        return json.dumps(self._schema, ensure_ascii=True, separators=(",", ":"))


"""Semantic validation for phase-9 LLM motion commands."""


import logging

import numpy as np

# Failsafe motion limits — hardcoded safety boundary. These mirror the identical
# values in safety.policy_loader._FAILSAFE_MOTION_LIMITS so intent_engine stays
# importable when the safety package is not on the path (e.g. HMI backend tests
# running outside a colcon workspace).
_FAILSAFE_MOTION_LIMITS: dict[str, float] = {
    "max_velocity_scale": 0.06,
    "max_acceleration_scale": 0.06,
    "max_move_rel_translation": 0.21,
}


def _load_safety_rules() -> dict:
    """Lazy-load the full safety rules dict from the safety package.

    Returns an empty dict when the safety package is not importable so that
    SemanticValidator and drawing routers use their internal defaults.
    """
    try:
        from safety.policy_loader import load_safety_rules as _loader
    except ImportError:
        return {}
    return _loader()


_LOGGER = logging.getLogger(__name__)


class SemanticValidator:
    """Enforce phase-specific primitive, workspace, and scaling constraints."""

    _ALLOWED_PRIMITIVES = {
        "HOME",
        "PTP",
        "LIN",
        "CIRC",
        "MOVE_REL",
        "GET_POSE",
        "CARTESIAN_PATH",
        "SET_SPEED",
        "WAIT",
        "STOP",
        "MOVE_JOINT",
        "MOVE_JOINTS",
        "IO_SET",
        "ALARM_RESET",
        "BLENDED_SEQUENCE",
        "MACRO",
    }
    _MIN_VELOCITY_SCALE = 0.01
    # Fail-safe only — active policy loaded from safety_rules.yaml at construction.
    _MAX_VELOCITY_SCALE = _FAILSAFE_MOTION_LIMITS["max_velocity_scale"]
    _MAX_MOVE_REL_DELTA = _FAILSAFE_MOTION_LIMITS["max_move_rel_translation"]

    # GP4 has 6 joints (0..5)
    _NUM_JOINTS = 6

    # Fallback bounds — overridden by safety_rules.yaml at construction
    _DEFAULT_BOUNDS = {
        "x": (-0.45, 0.45),
        "y": (-0.16, 0.52),
        "z": (0.15, 0.65),
    }

    def __init__(self, safety_rules: dict | None = None):
        if safety_rules is None:
            safety_rules = _load_safety_rules()
        self._safety_rules = dict(safety_rules)
        circ_cfg = safety_rules.get("circ", {})
        self._circ_degenerate_tolerance = float(
            circ_cfg.get("degenerate_tolerance", 1e-3)
        )
        motion_limits = safety_rules.get("motion_limits", {})
        legacy_limits = safety_rules.get("joint_limits_override", {})
        self._max_velocity_scale = float(
            motion_limits.get(
                "max_velocity_scale",
                legacy_limits.get("max_velocity_scale", self._MAX_VELOCITY_SCALE),
            )
        )
        self._max_move_rel_delta = float(
            motion_limits.get("max_move_rel_translation", self._MAX_MOVE_REL_DELTA)
        )
        workspace_bounds = self._DEFAULT_BOUNDS
        ws = safety_rules.get("workspace_bounds")
        if not isinstance(ws, dict) or not ws:
            ws = safety_rules.get("workspace", {})
        self._workspace_bounds = {
            "x": (
                float(ws.get("x_min", workspace_bounds["x"][0])),
                float(ws.get("x_max", workspace_bounds["x"][1])),
            ),
            "y": (
                float(ws.get("y_min", workspace_bounds["y"][0])),
                float(ws.get("y_max", workspace_bounds["y"][1])),
            ),
            "z": (
                float(ws.get("z_min", workspace_bounds["z"][0])),
                float(ws.get("z_max", workspace_bounds["z"][1])),
            ),
        }

    # _load_safety_rules() removed — use safety.policy_loader.load_safety_rules()

    def validate(self, command: Dict[str, Any]) -> bool:
        if not isinstance(command, dict):
            raise ValueError("command must be an object.")

        primitive_type = command.get("primitive_type")
        if primitive_type not in self._ALLOWED_PRIMITIVES:
            raise ValueError(
                f"primitive_type must be one of {sorted(self._ALLOWED_PRIMITIVES)}."
            )

        # ── GET_POSE is a query-only command — no motion targets, no velocity/scaling needed. ──
        if primitive_type == "GET_POSE":
            if (
                command.get("target_pose_msg")
                or command.get("target_pose")
                or command.get("joint_target")
            ):
                raise ValueError(
                    "GET_POSE must not include target_pose or joint_target."
                )
            return True

        # ── STOP: no fields needed ──
        if primitive_type == "STOP":
            if (
                command.get("target_pose_msg")
                or command.get("target_pose")
                or command.get("joint_target")
            ):
                raise ValueError("STOP must not include target_pose or joint_target.")
            return True

        # ── ALARM_RESET: no fields needed ──
        if primitive_type == "ALARM_RESET":
            return True

        # ── SET_SPEED: velocity_scale in bounds, stateless ──
        if primitive_type == "SET_SPEED":
            vs = float(command.get("velocity_scale", 0.0))
            if not (self._MIN_VELOCITY_SCALE <= vs <= self._max_velocity_scale):
                raise ValueError(
                    f"SET_SPEED: velocity_scale {vs:.2f} must be within "
                    f"[{self._MIN_VELOCITY_SCALE:.2f}, {self._max_velocity_scale:.2f}]."
                )
            return True

        # ── WAIT: duration must be >= 0 ──
        if primitive_type == "WAIT":
            duration = float(command.get("wait_duration_sec", -1.0))
            if duration < 0:
                raise ValueError("WAIT: wait_duration_sec must be >= 0.")
            return True

        if primitive_type == "MACRO":
            steps = command.get("steps")
            if not isinstance(steps, list) or not steps:
                raise ValueError("MACRO requires non-empty steps.")
            if len(steps) > 10:
                raise ValueError("MACRO supports at most 10 steps.")
            return True

        # ── IO_SET: address required, value must be 0 or 1 ──
        if primitive_type == "IO_SET":
            if "io_address" not in command:
                raise ValueError("IO_SET requires io_address.")
            io_val = command.get("io_value")
            if io_val is None:
                raise ValueError("IO_SET requires io_value.")
            if int(io_val) not in (0, 1):
                raise ValueError(f"IO_SET: io_value must be 0 or 1, got {io_val}.")
            return True

        # ── BLENDED_SEQUENCE: multi-step blended LIN sequence (W2) ──
        if primitive_type == "BLENDED_SEQUENCE":
            steps = command.get("sequence_steps")
            if not steps or len(steps) < 2:
                raise ValueError("BLENDED_SEQUENCE requires at least 2 sequence_steps.")
            first_br = steps[0].get("blend_radius_m", 0.0)
            if first_br != 0.0:
                raise ValueError(
                    "BLENDED_SEQUENCE: first step blend_radius_m must be 0.0"
                )
            last_br = steps[-1].get("blend_radius_m", 0.0)
            if last_br != 0.0:
                raise ValueError(
                    "BLENDED_SEQUENCE: last step blend_radius_m must be 0.0"
                )
            for i, step in enumerate(steps):
                if "target_pose_msg" not in step:
                    raise ValueError(
                        f"BLENDED_SEQUENCE step[{i}] requires target_pose_msg."
                    )
                try:
                    self._validate_pose(step["target_pose_msg"])
                except ValueError as exc:
                    raise ValueError(f"BLENDED_SEQUENCE step[{i}]: {exc}") from exc
            return True

        # ── CIRC: circular motion via Pilz — target_pose + 1 auxiliary waypoint ──
        if primitive_type == "CIRC":
            waypoints = command.get("waypoints_msg")
            if not waypoints:
                raise ValueError(
                    "CIRC requires non-empty waypoints (exactly 1 auxiliary pose)."
                )
            if len(waypoints) != 1:
                raise ValueError(
                    f"CIRC requires exactly 1 auxiliary waypoint, got {len(waypoints)}."
                )
            if "target_pose_msg" not in command:
                raise ValueError("CIRC requires target_pose_msg (final pose).")
            self._validate_pose(command["target_pose_msg"])
            try:
                self._validate_pose(waypoints[0])
            except ValueError as e:
                raise ValueError(f"CIRC auxiliary waypoint[0]: {e}") from e
            # W2.T6: degenerate arc check
            self._check_circ_degenerate(command)
            return True

        # ── MOVE_JOINT: validate joint_index and joint_angle ──
        if primitive_type == "MOVE_JOINT":
            if "joint_index" not in command:
                raise ValueError("MOVE_JOINT requires joint_index.")
            if "joint_angle" not in command:
                raise ValueError("MOVE_JOINT requires joint_angle.")
            idx = int(command["joint_index"])
            if idx < 0 or idx >= self._NUM_JOINTS:
                raise ValueError(
                    f"MOVE_JOINT: joint_index {idx} out of range "
                    f"[0, {self._NUM_JOINTS - 1}]."
                )
            angle = float(command["joint_angle"])
            if not math.isfinite(angle):
                raise ValueError("MOVE_JOINT: joint_angle must be a finite number.")
            return True

        # ── MOVE_JOINTS: validate joint_target length ──
        if primitive_type == "MOVE_JOINTS":
            jt = command.get("joint_target")
            if not jt or not isinstance(jt, list):
                raise ValueError("MOVE_JOINTS requires joint_target as a list.")
            if len(jt) != self._NUM_JOINTS:
                raise ValueError(
                    f"MOVE_JOINTS: joint_target must have exactly "
                    f"{self._NUM_JOINTS} elements, got {len(jt)}."
                )
            return True

        # ── MOVE_REL: translation-only relative motion ──
        if primitive_type == "MOVE_REL":
            return self._validate_move_rel(command)

        # ── Motion primitives (HOME, PTP, LIN) require velocity_scale ──
        velocity_scale = float(command.get("velocity_scale", 0.0))
        if not (self._MIN_VELOCITY_SCALE <= velocity_scale <= self._max_velocity_scale):
            raise ValueError(
                "velocity_scale must be within "
                f"[{self._MIN_VELOCITY_SCALE:.2f}, {self._max_velocity_scale:.2f}]."
            )

        has_pose = "target_pose_msg" in command
        has_joints = bool(command.get("joint_target"))

        if primitive_type == "HOME":
            if has_pose or has_joints:
                raise ValueError("HOME must not include target_pose or joint_target.")
            return True

        # CARTESIAN_PATH: multi-waypoint smooth trajectory
        if primitive_type == "CARTESIAN_PATH":
            waypoints = command.get("waypoints_msg")
            if not waypoints:
                raise ValueError("CARTESIAN_PATH requires non-empty waypoints.")
            for i, wp in enumerate(waypoints):
                try:
                    self._validate_pose(wp)
                except ValueError as e:
                    raise ValueError(f"CARTESIAN_PATH waypoint[{i}]: {e}") from e
            return True

        if primitive_type == "LIN" and not has_pose:
            raise ValueError("LIN requires target_pose.")

        if primitive_type == "PTP" and not (has_pose or has_joints):
            raise ValueError("PTP requires target_pose or joint_target.")

        if has_pose:
            self._validate_pose(command["target_pose_msg"])

        return True

    def _check_circ_degenerate(self, command: Dict[str, Any]) -> None:
        """Reject CIRC if start, auxiliary, goal are colinear (W2.T6)."""
        target = command["target_pose_msg"]
        aux = command["waypoints_msg"][0]
        # start is the current pose — not available here; use aux vs target only
        # when a start_pose is provided in the command, use that.
        start_pose = command.get("start_pose_msg")
        if start_pose is None:
            # Cannot check colinearity without start pose; skip.
            return
        start = np.array(
            [start_pose.position.x, start_pose.position.y, start_pose.position.z]
        )
        aux_pt = np.array([aux.position.x, aux.position.y, aux.position.z])
        goal = np.array([target.position.x, target.position.y, target.position.z])
        v1 = aux_pt - start
        v2 = goal - start
        cross_norm = float(np.linalg.norm(np.cross(v1, v2)))
        denom = float(np.linalg.norm(v1) * np.linalg.norm(v2))
        if denom < 1e-9:
            raise ValueError(
                "degenerate CIRC: zero-length segment between start, aux, or goal"
            )
        ratio = cross_norm / denom
        if ratio < self._circ_degenerate_tolerance:
            raise ValueError(
                f"degenerate CIRC: aux is colinear with start-goal "
                f"(cross/|v1||v2| = {ratio:.6f})"
            )

    def _validate_move_rel(self, command: Dict[str, Any]) -> bool:
        """Validate MOVE_REL: translation-only relative motion."""
        if command.get("target_pose_msg") or command.get("joint_target"):
            raise ValueError("MOVE_REL must not include target_pose or joint_target.")

        for field in ("delta_x", "delta_y", "delta_z"):
            if field not in command:
                raise ValueError(f"MOVE_REL requires {field}.")
            if not math.isfinite(float(command[field])):
                raise ValueError(f"MOVE_REL: {field} must be a finite number.")

        dx = float(command["delta_x"])
        dy = float(command["delta_y"])
        dz = float(command["delta_z"])

        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            raise ValueError("MOVE_REL: at least one delta component must be non-zero.")

        delta_norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if delta_norm > self._max_move_rel_delta:
            raise ValueError(
                f"MOVE_REL: delta norm {delta_norm:.4f} m exceeds "
                f"limit {self._max_move_rel_delta} m."
            )

        ref_frame = command.get("reference_frame", "base_link")
        if ref_frame and ref_frame != "base_link":
            raise ValueError(
                f"MOVE_REL: unsupported reference_frame '{ref_frame}'; "
                f"only 'base_link' is supported."
            )

        return True

    def _validate_pose(self, pose: Any) -> None:
        for axis, value in (
            ("x", pose.position.x),
            ("y", pose.position.y),
            ("z", pose.position.z),
        ):
            lower, upper = self._workspace_bounds[axis]  # ← instance attr
            if not (lower <= value <= upper):
                raise ValueError(
                    f"target_pose.position.{axis}={value:.4f} is outside "
                    f"configured workspace [{lower}, {upper}]."
                )

        quaternion_norm = math.sqrt(
            (pose.orientation.x * pose.orientation.x)
            + (pose.orientation.y * pose.orientation.y)
            + (pose.orientation.z * pose.orientation.z)
            + (pose.orientation.w * pose.orientation.w)
        )
        if not math.isfinite(quaternion_norm):
            raise ValueError("target_pose.orientation must be finite.")
        if quaternion_norm <= 1e-9:
            # Zero quaternion is a supported sentinel for position-only intents.
            # motion_core resolves this to current tool orientation before IK.
            return


"""Full-sequence prevalidation for routed primitive command sequences."""


from dataclasses import dataclass, field


_QUERY_PRIMITIVES = {"GET_POSE"}
_FRAME_REQUIRED_PRIMITIVES = {"PTP", "LIN", "MOVE_REL", "CARTESIAN_PATH"}
_SUPPORTED_SEQUENCE_FRAMES = {"base_link"}


@dataclass(frozen=True)
class SequenceValidationResult:
    normalized_commands: List[Dict[str, Any]]
    step_count: int
    validated_reference_frame: str | None
    cumulative_move_rel_distance_m: float
    estimated_duration_lower_bound_sec: float
    duration_estimate_is_lower_bound: bool
    has_io_side_effects: bool
    manual_recovery_required_on_failure: bool
    diagnostics: List[str] = field(default_factory=list)


class SequenceValidationError(ValueError):
    """Structured sequence prevalidation failure with stage and step context."""

    def __init__(self, stage: str, reason: str, *, step_index: int | None = None):
        self.stage = stage
        self.step_index = step_index
        self.reason = reason
        prefix = f"sequence_validation:{stage}"
        if step_index is not None:
            prefix = f"{prefix}:step={step_index + 1}"
        super().__init__(f"{prefix}: {reason}")


class SequenceValidator:
    """Prevalidate a full routed primitive sequence before any dispatch occurs."""

    def __init__(
        self,
        schema_validator: Any | None = None,
        normalizer: Any | None = None,
        semantic_validator: Any | None = None,
        *,
        max_sequence_length: int = 40,
        max_cumulative_move_rel_distance_m: float = 0.40,
    ) -> None:
        if schema_validator is None:
            schema_validator = SchemaValidator()
        if normalizer is None:
            normalizer = Normalizer()
        if semantic_validator is None:
            semantic_validator = SemanticValidator()

        self._schema_validator = schema_validator
        self._normalizer = normalizer
        self._semantic_validator = semantic_validator
        self._max_sequence_length = int(max_sequence_length)
        self._max_cumulative_move_rel_distance_m = float(
            max_cumulative_move_rel_distance_m
        )

    def validate(self, commands: List[Dict[str, Any]]) -> SequenceValidationResult:
        if not isinstance(commands, list) or not commands:
            raise SequenceValidationError(
                "structure", "sequence must be a non-empty list of primitive commands."
            )
        if len(commands) > self._max_sequence_length:
            raise SequenceValidationError(
                "sequence_length",
                f"sequence has {len(commands)} steps, limit is {self._max_sequence_length}.",
            )

        normalized_commands: List[Dict[str, Any]] = []
        validated_frame: str | None = None
        cumulative_move_rel_distance_m = 0.0
        estimated_duration_lower_bound_sec = 0.0
        first_io_step_index: int | None = None

        for step_index, command in enumerate(commands):
            if not isinstance(command, dict):
                raise SequenceValidationError(
                    "structure",
                    "each sequence step must be an object.",
                    step_index=step_index,
                )

            primitive_type = str(command.get("primitive_type", ""))
            if not primitive_type:
                raise SequenceValidationError(
                    "structure",
                    "sequence step is missing primitive_type.",
                    step_index=step_index,
                )

            if primitive_type in _QUERY_PRIMITIVES:
                raise SequenceValidationError(
                    "unsupported_step",
                    f"{primitive_type} is query-only and is not supported inside sequences.",
                    step_index=step_index,
                )

            if primitive_type == "STOP" and len(commands) != 1:
                raise SequenceValidationError(
                    "stop_policy",
                    "STOP must be the sole primitive in a sequence.",
                    step_index=step_index,
                )

            try:
                self._schema_validator.validate(command)
            except Exception as exc:
                raise SequenceValidationError(
                    "schema", str(exc), step_index=step_index
                ) from exc

            try:
                normalized_command = self._normalizer.normalize(command)
            except Exception as exc:
                raise SequenceValidationError(
                    "normalize", str(exc), step_index=step_index
                ) from exc

            try:
                self._semantic_validator.validate(normalized_command)
            except Exception as exc:
                raise SequenceValidationError(
                    "semantic", str(exc), step_index=step_index
                ) from exc

            step_frame = self._resolve_step_frame(normalized_command, step_index)
            if step_frame is not None:
                if validated_frame is None:
                    validated_frame = step_frame
                elif validated_frame != step_frame:
                    raise SequenceValidationError(
                        "frame_policy",
                        f"mixed frames are not supported; saw '{validated_frame}' and '{step_frame}'.",
                        step_index=step_index,
                    )

            if primitive_type == "MOVE_REL":
                cumulative_move_rel_distance_m += self._move_rel_distance(
                    normalized_command
                )
                if (
                    cumulative_move_rel_distance_m
                    > self._max_cumulative_move_rel_distance_m
                ):
                    raise SequenceValidationError(
                        "move_rel_budget",
                        "cumulative MOVE_REL distance exceeds sequence limit "
                        f"{self._max_cumulative_move_rel_distance_m:.3f} m.",
                        step_index=step_index,
                    )

            if primitive_type == "WAIT":
                estimated_duration_lower_bound_sec += float(
                    normalized_command.get("wait_duration_sec", 0.0)
                )

            if primitive_type == "IO_SET" and first_io_step_index is None:
                first_io_step_index = step_index

            normalized_commands.append(normalized_command)

        has_io_side_effects = first_io_step_index is not None
        manual_recovery_required_on_failure = (
            has_io_side_effects and first_io_step_index < len(commands) - 1
        )

        diagnostics = [
            "Validated checks: structure, max length, STOP sole-primitive policy, per-step schema, "
            "normalization, semantic validation, explicit frame policy, cumulative MOVE_REL budget."
        ]
        diagnostics.append(
            "Duration estimate is a heuristic lower bound only; WAIT contributes directly and all other "
            "primitive timing remains conservative/unknown at prevalidation time."
        )
        diagnostics.append(
            "NOT YET IMPLEMENTED: kinematic reachability across the full sequence, collision continuity, "
            "controller timing feasibility, IO rollback analysis, and current-pose-aware macro validation."
        )

        return SequenceValidationResult(
            normalized_commands=normalized_commands,
            step_count=len(normalized_commands),
            validated_reference_frame=validated_frame,
            cumulative_move_rel_distance_m=cumulative_move_rel_distance_m,
            estimated_duration_lower_bound_sec=estimated_duration_lower_bound_sec,
            duration_estimate_is_lower_bound=True,
            has_io_side_effects=has_io_side_effects,
            manual_recovery_required_on_failure=manual_recovery_required_on_failure,
            diagnostics=diagnostics,
        )

    def _resolve_step_frame(
        self, normalized_command: Dict[str, Any], step_index: int
    ) -> str | None:
        primitive_type = str(normalized_command.get("primitive_type", ""))
        requires_frame = primitive_type in _FRAME_REQUIRED_PRIMITIVES

        if not requires_frame:
            return None

        if "reference_frame" not in normalized_command:
            raise SequenceValidationError(
                "frame_policy",
                f"{primitive_type} requires explicit reference_frame in sequences; no silent fallback is allowed.",
                step_index=step_index,
            )

        step_frame = str(normalized_command.get("reference_frame", "")).strip()
        if step_frame not in _SUPPORTED_SEQUENCE_FRAMES:
            raise SequenceValidationError(
                "frame_policy",
                f"unsupported reference_frame '{step_frame}'; only 'base_link' is supported in v2.1.",
                step_index=step_index,
            )
        return step_frame

    @staticmethod
    def _move_rel_distance(normalized_command: Dict[str, Any]) -> float:
        dx = float(normalized_command.get("delta_x", 0.0))
        dy = float(normalized_command.get("delta_y", 0.0))
        dz = float(normalized_command.get("delta_z", 0.0))
        return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))


__all__ = [
    "SequenceValidator",
    "SequenceValidationError",
    "SequenceValidationResult",
]
"""Mapping helpers from validated command dictionaries to ROS interfaces."""


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
    def _pose_to_payload(pose: "Pose") -> Dict[str, Any]:
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


"""Pure command-transformation helpers extracted from LLMGatewayNode.

These helpers have no ROS2 dependencies and are unit-testable in isolation.
The node calls them with the right policy flags / injected dependencies.

Extracting them shrinks the node's surface area and clarifies which logic
is pure data transformation versus which logic is ROS-coupled orchestration.
"""


import logging
from typing import Callable, Protocol

_LOGGER = logging.getLogger(__name__)


class _SchemaValidatorLike(Protocol):
    """Minimal protocol matching intent_engine.SchemaValidator."""

    def validate(self, command: Dict[str, Any]) -> None: ...


def prepare_execution_command(
    normalized_command: Dict[str, Any],
    logger: Optional[logging.Logger] = None,
) -> Dict[str, Any]:
    """Return the command payload to dispatch to motion_core.

    ``plan_only`` is metadata only and is rejected instead of being converted
    into execution. Human approval is owned by the HMI supervisor layer.
    """
    del logger

    if normalized_command.get("plan_only"):
        raise ValueError(
            "plan_only_not_executable: plan_only commands are not executable by "
            "/execute_motion; use a plan-only review workflow instead."
        )

    execution_command = dict(normalized_command)
    return execution_command


def command_from_sanitized_json(
    sanitized_json: str,
    fallback_payload: Dict[str, Any],
    schema_validator: _SchemaValidatorLike,
) -> Dict[str, Any]:
    """Decode and re-validate a supervisor-sanitized JSON payload.

    Returns ``fallback_payload`` when ``sanitized_json`` is empty.
    Raises ``ValueError`` on non-object JSON or schema violations.
    """
    if not sanitized_json:
        return fallback_payload
    loaded = json.loads(sanitized_json)
    if not isinstance(loaded, dict):
        raise ValueError("sanitized_json must decode to a JSON object.")
    schema_validator.validate(loaded)
    return loaded


def hydrate_draw_workplane(
    payload: Dict[str, Any],
    fetch_current_pose: Callable[[str], Optional[Dict[str, Any]]],
) -> Dict[str, Any]:
    """Ensure tool-mode drawing payloads carry an explicit workplane origin.

    Only applies to ``draw_shape`` / ``draw_text`` intents with workplane
    mode ``tool``. The ``fetch_current_pose`` callable isolates ROS service
    calls so this function stays unit-testable.

    Raises ``ValueError`` when tool-mode hydration is required but the
    pose service is unavailable.
    """
    if not isinstance(payload, dict):
        return payload
    intent = str(payload.get("intent", "")).strip().lower()
    if intent not in {"draw_shape", "draw_text"}:
        return payload

    working_payload = dict(payload)
    workplane = working_payload.get("workplane")
    if workplane is None:
        workplane = {"mode": "tool"}
        working_payload["workplane"] = workplane
    if not isinstance(workplane, dict):
        return working_payload

    mode = str(workplane.get("mode", "base")).strip().lower()
    if mode != "tool":
        return working_payload
    if isinstance(workplane.get("origin"), dict):
        return working_payload
    if isinstance(working_payload.get("start_pose"), dict):
        return working_payload

    current_pose = fetch_current_pose("base_link")
    if current_pose is None:
        # W2.T5: fallback to base-frame workplane from SSOT instead of hard error.
        _LOGGER.warning(
            "hydrate_draw_workplane: /get_current_pose unavailable; "
            "falling back to base-frame workplane from safety_rules.yaml"
        )
        try:
            from safety.policy_loader import load_safety_rules

            drawing_cfg = load_safety_rules().get("drawing", {})
            fb = drawing_cfg.get("fallback_workplane", {})
            fb_pose = fb.get("pose", {})
            current_pose = {
                "position": fb_pose.get("position", {"x": 0.30, "y": 0.0, "z": 0.20}),
                "orientation": fb_pose.get(
                    "orientation", {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
                ),
            }
        except Exception:
            raise ValueError(
                "missing_workplane: tool mode requires current pose, "
                "but /get_current_pose is unavailable"
            )

    hydrated_workplane = dict(workplane)
    hydrated_workplane["origin"] = current_pose
    working_payload["workplane"] = hydrated_workplane
    return working_payload


"""Draw shape and draw text routing extracted from intent_router."""


from copy import deepcopy

# _load_safety_rules is defined earlier in this module as a lazy loader;
# when the safety package is importable this import replaces it with the
# direct function (identical call signature).
try:
    from safety.policy_loader import load_safety_rules as _load_safety_rules
except ImportError:
    pass

from llm_gateway.drawing_geometry import (
    DrawingGeometryError,
    compile_strokes_to_commands,
    generate_arc_path,
    generate_circle_path,
    generate_polygon_path,
    generate_polyline_path,
    generate_rectangle_path,
    generate_square_path,
    generate_text_stroke_segments,
    generate_triangle_path,
    lift_points_to_poses,
    parse_position_dict,
    parse_vector_dict,
    resolve_workplane,
    supported_glyphs,
    to_meters,
    to_radians,
)


_FRAME_BASE_LINK = "base_link"


def _build_route_result(*args: Any, **kwargs: Any) -> Any:
    # Late import avoids circular dependency: intent_router imports this module.

    return RouteResult(*args, **kwargs)


class DrawRouterMixin:
    """Mixin providing draw shape/text routing for IntentRouter."""

    def _route_draw_shape(self, payload: Dict[str, Any]) -> RouteResult:
        policy = self._macro_policy["macros"].get("draw_shape")
        if not isinstance(policy, dict):
            raise ValueError("Macro policy missing draw_shape configuration.")

        availability = str(policy.get("availability", "all")).strip().lower()
        if availability == "sim_only" and self._runtime_mode != "sim":
            raise ValueError(
                "draw_shape is sim-only and is unavailable in hardware mode."
            )
        if availability == "disabled":
            raise ValueError("draw_shape capability_unavailable")

        shape = str(payload.get("shape_type", payload.get("shape", ""))).strip().lower()
        supported_shapes = {
            str(item).strip().lower() for item in policy.get("supported_shapes", [])
        }
        if shape not in supported_shapes:
            raise ValueError(
                f"unsupported_shape_type: '{shape}'; supported: {sorted(supported_shapes)}"
            )

        reference_frame = payload.get(
            "frame_id", payload.get("reference_frame", _FRAME_BASE_LINK)
        )
        self._validate_reference_frame(
            reference_frame,
            allowed_frames={str(item) for item in policy.get("supported_frames", [])},
        )

        units = (
            str(payload.get("units", policy.get("default_units", "m"))).strip().lower()
        )
        params = payload.get("params") or {}
        if not isinstance(params, dict):
            raise ValueError("draw_shape params must be an object.")

        try:
            workplane = self._resolve_draw_workplane(
                payload=payload,
                policy=policy,
                reference_frame=reference_frame,
            )
            execution_policy = self._resolve_execution_policy(payload, policy)
            stroke_config = self._resolve_stroke_config(payload, policy, units=units)
            max_chord_error_m = float(policy.get("max_chord_error_m", 0.0005))
            max_segment_angle_rad = math.radians(
                float(policy.get("max_segment_angle_deg", 10.0))
            )

            if shape == "square":
                side_m = self._extract_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("side_m", "side", "size_m", "size"),
                    error_label="square side",
                )
                strokes = generate_square_path(side_m=side_m)
            elif shape == "rectangle":
                width_m = self._extract_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("width_m", "width"),
                    error_label="rectangle width",
                )
                height_m = self._extract_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("height_m", "height"),
                    error_label="rectangle height",
                )
                strokes = generate_rectangle_path(width_m=width_m, height_m=height_m)
            elif shape == "triangle":
                explicit_points = self._extract_points_2d(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("points", "vertices"),
                    required=False,
                )
                if explicit_points is not None:
                    strokes = generate_triangle_path(
                        side_m=1.0, points_2d=explicit_points
                    )
                else:
                    side_m = self._extract_positive_length(
                        payload=payload,
                        params=params,
                        units=units,
                        candidates=("side_m", "side", "size_m", "size"),
                        error_label="triangle side",
                    )
                    strokes = generate_triangle_path(side_m=side_m)
            elif shape == "circle":
                radius_m = self._extract_circle_radius_m(
                    payload=payload, params=params, units=units
                )
                strokes = generate_circle_path(
                    radius_m=radius_m,
                    max_chord_error_m=max_chord_error_m,
                    max_segment_angle_rad=max_segment_angle_rad,
                )
            elif shape == "arc":
                radius_m = self._extract_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("radius_m", "radius"),
                    error_label="arc radius",
                )
                sweep_rad = self._extract_sweep_radians(payload=payload, params=params)
                strokes = generate_arc_path(
                    radius_m=radius_m,
                    sweep_rad=sweep_rad,
                    max_chord_error_m=max_chord_error_m,
                    max_segment_angle_rad=max_segment_angle_rad,
                )
            elif shape == "polygon":
                n_sides = self._extract_polygon_sides(
                    payload=payload, params=params, policy=policy
                )
                radius_candidate = self._extract_optional_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("radius_m", "radius"),
                )
                side_candidate = self._extract_optional_positive_length(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("side_m", "side", "size_m", "size"),
                )
                strokes = generate_polygon_path(
                    n_sides=n_sides,
                    radius_m=radius_candidate,
                    side_m=side_candidate,
                )
            else:  # polyline
                points_2d = self._extract_points_2d(
                    payload=payload,
                    params=params,
                    units=units,
                    candidates=("points", "vertices"),
                    required=True,
                )
                strokes = generate_polyline_path(points_2d=points_2d)

            drawing_cfg = _load_safety_rules().get("drawing", {})
            compiled = compile_strokes_to_commands(
                strokes=strokes,
                workplane=workplane,
                reference_frame=reference_frame,
                approach_distance_m=stroke_config["approach_distance_m"],
                retract_distance_m=stroke_config["retract_distance_m"],
                drawing_speed_scale=stroke_config["drawing_speed_scale"],
                travel_speed_scale=stroke_config["travel_speed_scale"],
                plan_only=execution_policy["plan_only"],
                max_waypoints_per_chunk=int(policy.get("max_waypoints_per_chunk", 80)),
                use_blended_sequence=bool(
                    drawing_cfg.get("use_blended_sequence", True)
                ),
                blend_radius_m=float(drawing_cfg.get("blend_radius_m", 0.008)),
            )
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

        return _build_route_result(
            route_type="sequence",
            commands=[deepcopy(command) for command in compiled.commands],
            metadata={
                "source": "semantic_ir",
                "macro_name": "draw_shape",
                "requires_current_pose": bool(
                    policy.get("requires_current_pose", False)
                ),
                "shape_type": shape,
                "execution_mode": execution_policy["execution_mode"],
                "summary": dict(compiled.summary),
            },
        )

    def _route_draw_text(self, payload: Dict[str, Any]) -> RouteResult:
        policy = self._macro_policy["macros"].get("draw_text")
        if not isinstance(policy, dict):
            raise ValueError("Macro policy missing draw_text configuration.")

        availability = str(policy.get("availability", "all")).strip().lower()
        # Removed sim-only guard to enable draw on hardware
        if availability == "disabled":
            raise ValueError("draw_text capability_unavailable")

        reference_frame = payload.get(
            "frame_id", payload.get("reference_frame", _FRAME_BASE_LINK)
        )
        self._validate_reference_frame(
            reference_frame,
            allowed_frames={str(item) for item in policy.get("supported_frames", [])},
        )

        text = payload.get("text")
        if not isinstance(text, str) or not text.strip():
            raise ValueError("draw_text requires a non-empty text string.")
        normalized_text = text.upper()

        supported_characters = {
            str(item) for item in policy.get("supported_characters", [])
        }
        if not supported_characters:
            supported_characters = set(supported_glyphs())
        unsupported_characters = sorted(
            {
                character
                for character in normalized_text
                if character not in supported_characters and character != "\n"
            }
        )
        if unsupported_characters:
            raise ValueError(
                "unsupported_font_glyph: "
                f"{unsupported_characters}; supported: {sorted(supported_characters)}"
            )

        units = (
            str(payload.get("units", policy.get("default_units", "m"))).strip().lower()
        )
        font = payload.get("font") or {}
        if not isinstance(font, dict):
            raise ValueError("draw_text font must be an object.")
        font_type = str(font.get("type", "single_stroke_builtin")).strip().lower()
        if font_type != "single_stroke_builtin":
            raise ValueError(
                "capability_unavailable: only single_stroke_builtin font is supported."
            )

        try:
            workplane = self._resolve_draw_workplane(
                payload=payload,
                policy=policy,
                reference_frame=reference_frame,
            )
            execution_policy = self._resolve_execution_policy(payload, policy)
            stroke_config = self._resolve_stroke_config(payload, policy, units=units)

            height_value = self._extract_any(
                payload=payload,
                params=font,
                candidates=("height_m", "height", "char_height_m"),
            )
            if height_value is None:
                raise DrawingGeometryError("missing_size: text height is required")
            height_m = to_meters(height_value, units, field_name="font.height")
            if height_m <= 0.0:
                raise DrawingGeometryError("invalid_size: text height must be > 0")

            default_char_spacing = (
                float(policy.get("default_char_spacing_ratio", 0.20)) * height_m
            )
            default_line_spacing = (
                float(policy.get("default_line_spacing_ratio", 0.50)) * height_m
            )

            char_spacing_raw = self._extract_any(
                payload=payload,
                params=font,
                candidates=("char_spacing_m", "char_spacing"),
            )
            line_spacing_raw = self._extract_any(
                payload=payload,
                params=font,
                candidates=("line_spacing_m", "line_spacing"),
            )
            char_spacing_m = (
                default_char_spacing
                if char_spacing_raw is None
                else to_meters(char_spacing_raw, units, field_name="font.char_spacing")
            )
            line_spacing_m = (
                default_line_spacing
                if line_spacing_raw is None
                else to_meters(line_spacing_raw, units, field_name="font.line_spacing")
            )

            alignment = (
                str(font.get("alignment", payload.get("alignment", "left")))
                .strip()
                .lower()
            )
            direction = (
                str(font.get("direction", payload.get("direction", "+x")))
                .strip()
                .lower()
            )
            if direction not in {"+x", "x", "local+x"}:
                raise DrawingGeometryError(
                    "capability_unavailable: only +X text direction is supported"
                )

            stroke_segments = generate_text_stroke_segments(
                text=normalized_text,
                height_m=height_m,
                char_spacing_m=char_spacing_m,
                line_spacing_m=line_spacing_m,
                alignment=alignment,
            )
            if not any(segment.kind == "draw" for segment in stroke_segments):
                raise DrawingGeometryError(
                    "unsupported_font_glyph: text has no drawable glyphs"
                )

            drawing_cfg = _load_safety_rules().get("drawing", {})
            compiled = compile_strokes_to_commands(
                strokes=stroke_segments,
                workplane=workplane,
                reference_frame=reference_frame,
                approach_distance_m=stroke_config["approach_distance_m"],
                retract_distance_m=stroke_config["retract_distance_m"],
                drawing_speed_scale=stroke_config["drawing_speed_scale"],
                travel_speed_scale=stroke_config["travel_speed_scale"],
                plan_only=execution_policy["plan_only"],
                max_waypoints_per_chunk=int(policy.get("max_waypoints_per_chunk", 80)),
                use_blended_sequence=bool(
                    drawing_cfg.get("use_blended_sequence", True)
                ),
                blend_radius_m=float(drawing_cfg.get("blend_radius_m", 0.008)),
            )
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

        return _build_route_result(
            route_type="sequence",
            commands=[deepcopy(command) for command in compiled.commands],
            metadata={
                "source": "semantic_ir",
                "macro_name": "draw_text",
                "requires_current_pose": bool(
                    policy.get("requires_current_pose", False)
                ),
                "text": normalized_text,
                "execution_mode": execution_policy["execution_mode"],
                "summary": dict(compiled.summary),
            },
        )

    def _resolve_draw_workplane(
        self,
        *,
        payload: Dict[str, Any],
        policy: Dict[str, Any],
        reference_frame: str,
    ):
        workplane_payload = payload.get("workplane") or {}
        if not isinstance(workplane_payload, dict):
            raise ValueError("workplane must be an object.")

        supported_modes = {
            str(item).strip().lower()
            for item in policy.get("supported_workplane_modes", ["base"])
        }
        mode = (
            str(
                workplane_payload.get(
                    "mode",
                    payload.get(
                        "plane_mode", policy.get("default_workplane_mode", "base")
                    ),
                )
            )
            .strip()
            .lower()
        )
        if mode not in supported_modes:
            raise ValueError(
                f"missing_workplane: unsupported mode '{mode}'; supported: {sorted(supported_modes)}"
            )

        start_pose = payload.get("start_pose")
        anchor_position: Tuple[float, float, float] | None = None
        if isinstance(start_pose, dict) and isinstance(
            start_pose.get("position"), dict
        ):
            try:
                anchor_position = parse_position_dict(
                    start_pose["position"], field_name="start_pose.position"
                )
            except DrawingGeometryError as exc:
                raise ValueError(str(exc)) from exc

        origin_pose = workplane_payload.get("origin")
        if origin_pose is None and mode == "tool" and isinstance(start_pose, dict):
            origin_pose = start_pose
        if origin_pose is None and mode == "explicit_pose":
            raise ValueError(
                "missing_workplane: explicit_pose requires workplane.origin"
            )

        normal = None
        x_axis_hint = None
        if isinstance(workplane_payload.get("normal"), dict):
            try:
                normal = parse_vector_dict(
                    workplane_payload["normal"], field_name="workplane.normal"
                )
            except DrawingGeometryError as exc:
                raise ValueError(str(exc)) from exc
        if isinstance(workplane_payload.get("x_axis_hint"), dict):
            try:
                x_axis_hint = parse_vector_dict(
                    workplane_payload["x_axis_hint"],
                    field_name="workplane.x_axis_hint",
                )
            except DrawingGeometryError as exc:
                raise ValueError(str(exc)) from exc

        default_orientation = self._orientation_from_preset(
            policy.get("default_orientation_preset", "tool-down")
        )
        try:
            return resolve_workplane(
                mode=mode,
                frame_id=reference_frame,
                anchor_position=anchor_position,
                origin_pose=origin_pose if isinstance(origin_pose, dict) else None,
                default_orientation=default_orientation,
                normal=normal,
                x_axis_hint=x_axis_hint,
            )
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

    def _resolve_execution_policy(
        self, payload: Dict[str, Any], policy: Dict[str, Any]
    ) -> Dict[str, Any]:
        execution_mode = (
            str(
                payload.get(
                    "execution_mode", policy.get("default_execution_mode", "execute")
                )
            )
            .strip()
            .lower()
        )
        if execution_mode not in {"execute", "plan_only"}:
            raise ValueError("execution_mode must be one of: execute, plan_only")

        return {
            "execution_mode": execution_mode,
            "plan_only": execution_mode == "plan_only",
        }

    def _resolve_stroke_config(
        self,
        payload: Dict[str, Any],
        policy: Dict[str, Any],
        *,
        units: str,
    ) -> Dict[str, float]:
        stroke_payload = payload.get("stroke") or {}
        if not isinstance(stroke_payload, dict):
            raise ValueError("stroke must be an object.")

        def _field(*names: str) -> Any:
            for name in names:
                if name in stroke_payload:
                    return stroke_payload[name]
                if name in payload:
                    return payload[name]
            return None

        try:
            approach_raw = _field("approach_distance_m", "approach_distance")
            retract_raw = _field("retract_distance_m", "retract_distance")
            approach_distance_m = to_meters(
                approach_raw
                if approach_raw is not None
                else policy.get("approach_distance_m", 0.01),
                units,
                field_name="stroke.approach_distance",
            )
            retract_distance_m = to_meters(
                retract_raw
                if retract_raw is not None
                else policy.get("retract_distance_m", 0.01),
                units,
                field_name="stroke.retract_distance",
            )
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

        drawing_speed_scale = float(
            _field("drawing_speed_scale")
            or policy.get("drawing_speed_scale", payload.get("velocity_scale", 0.06))
        )
        travel_speed_scale = float(
            _field("travel_speed_scale") or policy.get("travel_speed_scale", 0.06)
        )
        if drawing_speed_scale <= 0.0 or travel_speed_scale <= 0.0:
            raise ValueError(
                "invalid_size: drawing_speed_scale and travel_speed_scale must be > 0"
            )

        return {
            "approach_distance_m": approach_distance_m,
            "retract_distance_m": retract_distance_m,
            "drawing_speed_scale": drawing_speed_scale,
            "travel_speed_scale": travel_speed_scale,
        }

    def _extract_any(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        candidates: Sequence[str],
    ) -> Any:
        for candidate in candidates:
            if candidate in params:
                return params[candidate]
            if candidate in payload:
                return payload[candidate]
        return None

    def _extract_positive_length(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        units: str,
        candidates: Sequence[str],
        error_label: str,
    ) -> float:
        raw_value = self._extract_any(
            payload=payload, params=params, candidates=candidates
        )
        if raw_value is None:
            raise ValueError(f"missing_size: {error_label} is required")
        try:
            length_m = to_meters(raw_value, units, field_name=error_label)
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc
        if length_m <= 0.0:
            raise ValueError(f"invalid_size: {error_label} must be > 0")
        return length_m

    def _extract_optional_positive_length(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        units: str,
        candidates: Sequence[str],
    ) -> float | None:
        raw_value = self._extract_any(
            payload=payload, params=params, candidates=candidates
        )
        if raw_value is None:
            return None
        try:
            length_m = to_meters(raw_value, units, field_name=candidates[0])
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc
        if length_m <= 0.0:
            raise ValueError(f"invalid_size: {candidates[0]} must be > 0")
        return length_m

    def _extract_circle_radius_m(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        units: str,
    ) -> float:
        radius_m = self._extract_optional_positive_length(
            payload=payload,
            params=params,
            units=units,
            candidates=("radius_m", "radius"),
        )
        if radius_m is not None:
            return radius_m

        diameter_m = self._extract_optional_positive_length(
            payload=payload,
            params=params,
            units=units,
            candidates=("diameter_m", "diameter", "size_m", "size"),
        )
        if diameter_m is None:
            raise ValueError("missing_size: circle radius is required")
        return diameter_m * 0.5

    def _extract_sweep_radians(
        self, *, payload: Dict[str, Any], params: Mapping[str, Any]
    ) -> float:
        sweep_rad_raw = self._extract_any(
            payload=payload, params=params, candidates=("sweep_rad",)
        )
        sweep_deg_raw = self._extract_any(
            payload=payload, params=params, candidates=("sweep_deg", "sweep")
        )

        try:
            if sweep_rad_raw is not None:
                sweep_rad = to_radians(
                    sweep_rad_raw, field_name="arc sweep_rad", units="rad"
                )
            elif sweep_deg_raw is not None:
                sweep_rad = to_radians(
                    sweep_deg_raw, field_name="arc sweep_deg", units="deg"
                )
            else:
                sweep_rad = math.radians(180.0)
        except DrawingGeometryError as exc:
            raise ValueError(str(exc)) from exc

        if math.isclose(sweep_rad, 0.0, abs_tol=1e-12):
            raise ValueError("invalid_size: arc sweep must be non-zero")
        return sweep_rad

    def _extract_polygon_sides(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        policy: Mapping[str, Any],
    ) -> int:
        raw_value = self._extract_any(
            payload=payload, params=params, candidates=("n_sides", "sides")
        )
        if raw_value is None:
            raw_value = policy.get("polygon_default_sides", 6)
        try:
            n_sides = int(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "invalid_polygon_sides: n_sides must be an integer"
            ) from exc

        min_sides = int(policy.get("polygon_min_sides", 3))
        max_sides = int(policy.get("polygon_max_sides", 12))
        if n_sides < min_sides or n_sides > max_sides:
            raise ValueError(
                f"invalid_polygon_sides: n_sides must be in [{min_sides}, {max_sides}]"
            )
        return n_sides

    def _extract_points_2d(
        self,
        *,
        payload: Dict[str, Any],
        params: Mapping[str, Any],
        units: str,
        candidates: Sequence[str],
        required: bool,
    ) -> List[Tuple[float, float]] | None:
        raw_points = self._extract_any(
            payload=payload, params=params, candidates=candidates
        )
        if raw_points is None:
            if required:
                raise ValueError("invalid_size: points are required")
            return None
        if not isinstance(raw_points, list):
            raise ValueError("invalid_size: points must be an array")
        if required and len(raw_points) < 2:
            raise ValueError("invalid_size: polyline requires at least 2 points")

        points_2d: List[Tuple[float, float]] = []
        for index, point in enumerate(raw_points):
            if not isinstance(point, dict):
                raise ValueError(f"invalid_size: points[{index}] must be an object")
            if "x" not in point or "y" not in point:
                raise ValueError(f"invalid_size: points[{index}] requires x and y")
            try:
                x_m = to_meters(point["x"], units, field_name=f"points[{index}].x")
                y_m = to_meters(point["y"], units, field_name=f"points[{index}].y")
            except DrawingGeometryError as exc:
                raise ValueError(str(exc)) from exc
            points_2d.append((x_m, y_m))
        return points_2d

    def _build_square_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        size_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        points = [
            {"x": position["x"], "y": position["y"], "z": position["z"]},
            {"x": position["x"] + size_m, "y": position["y"], "z": position["z"]},
            {
                "x": position["x"] + size_m,
                "y": position["y"] + size_m,
                "z": position["z"],
            },
            {"x": position["x"], "y": position["y"] + size_m, "z": position["z"]},
            {"x": position["x"], "y": position["y"], "z": position["z"]},
        ]
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_triangle_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        size_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        """Equilateral triangle on XY plane. size_m = side length."""
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        sx, sy, sz = position["x"], position["y"], position["z"]
        points = [
            {"x": sx, "y": sy, "z": sz},
            {"x": sx + size_m, "y": sy, "z": sz},
            {"x": sx + size_m / 2.0, "y": sy + size_m * math.sqrt(3) / 2.0, "z": sz},
            {"x": sx, "y": sy, "z": sz},  # close path
        ]
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_circle_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        size_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
        segments: int = 32,
    ) -> List[Dict[str, Any]]:
        """Circle on XY plane approximated by N cartesian waypoints. size_m = diameter."""
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        radius = size_m / 2.0
        # Center offset so circle starts at start_pose
        center_x = position["x"] + radius
        center_y = position["y"]
        sz = position["z"]

        points: List[Dict[str, float]] = []
        for i in range(segments + 1):
            theta = 2.0 * math.pi * i / segments
            points.append(
                {
                    "x": center_x - radius * math.cos(theta),
                    "y": center_y + radius * math.sin(theta),
                    "z": sz,
                }
            )
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_polygon_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        size_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
        sides: int = 6,
    ) -> List[Dict[str, Any]]:
        """Regular polygon on XY plane. size_m = circumscribed diameter."""
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        radius = size_m / 2.0
        # Center offset so polygon starts at start_pose
        center_x = position["x"] + radius
        center_y = position["y"]
        sz = position["z"]

        points: List[Dict[str, float]] = []
        for i in range(sides + 1):  # +1 to close path
            theta = 2.0 * math.pi * i / sides
            points.append(
                {
                    "x": center_x - radius * math.cos(theta),
                    "y": center_y + radius * math.sin(theta),
                    "z": sz,
                }
            )
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_rectangle_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        width_m: float,
        height_m: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        points = [
            {"x": position["x"], "y": position["y"], "z": position["z"]},
            {"x": position["x"] + width_m, "y": position["y"], "z": position["z"]},
            {
                "x": position["x"] + width_m,
                "y": position["y"] + height_m,
                "z": position["z"],
            },
            {"x": position["x"], "y": position["y"] + height_m, "z": position["z"]},
            {"x": position["x"], "y": position["y"], "z": position["z"]},
        ]
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_arc_sequence(
        self,
        *,
        start_pose: Dict[str, Any],
        radius_m: float,
        sweep_deg: float,
        reference_frame: str,
        motion_fields: Dict[str, Any],
        segments: int,
    ) -> List[Dict[str, Any]]:
        position = start_pose["position"]
        orientation = deepcopy(start_pose.get("orientation"))
        center_x = position["x"] + radius_m
        center_y = position["y"]
        sweep_rad = math.radians(sweep_deg)
        points: List[Dict[str, float]] = []
        for index in range(segments + 1):
            theta = sweep_rad * index / segments
            points.append(
                {
                    "x": center_x - radius_m * math.cos(theta),
                    "y": center_y + radius_m * math.sin(theta),
                    "z": position["z"],
                }
            )
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_polyline_sequence(
        self,
        *,
        points: List[Dict[str, float]],
        orientation: Dict[str, float] | None,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        return self._build_cartesian_path_sequence(
            points=points,
            orientation=orientation,
            reference_frame=reference_frame,
            motion_fields=motion_fields,
        )

    def _build_cartesian_path_sequence(
        self,
        *,
        points: List[Dict[str, float]],
        orientation: Dict[str, float] | None,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        if len(points) < 2:
            raise ValueError("CARTESIAN_PATH sequences require at least 2 points.")
        return [
            self._build_ptp_command(
                position=points[0],
                orientation=orientation,
                reference_frame=reference_frame,
                motion_fields=motion_fields,
            ),
            self._build_cartesian_path_command(
                points=points[1:],
                orientation=orientation,
                reference_frame=reference_frame,
                motion_fields=motion_fields,
            ),
        ]

    def _build_ptp_command(
        self,
        *,
        position: Dict[str, float],
        orientation: Dict[str, float] | None,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> Dict[str, Any]:
        return {
            "primitive_type": "PTP",
            "reference_frame": reference_frame,
            "target_pose": self._pose_from_position(position, orientation),
            **deepcopy(motion_fields),
        }

    def _build_cartesian_path_command(
        self,
        *,
        points: List[Dict[str, float]],
        orientation: Dict[str, float] | None,
        reference_frame: str,
        motion_fields: Dict[str, Any],
    ) -> Dict[str, Any]:
        if not points:
            raise ValueError("CARTESIAN_PATH requires at least one waypoint.")
        return {
            "primitive_type": "CARTESIAN_PATH",
            "reference_frame": reference_frame,
            "waypoints": [
                self._pose_from_position(point, orientation) for point in points
            ],
            **deepcopy(motion_fields),
        }

    def _pose_from_position(
        self, position: Dict[str, float], orientation: Dict[str, float] | None
    ) -> Dict[str, Any]:
        pose = {"position": deepcopy(position)}
        if orientation is not None:
            pose["orientation"] = deepcopy(orientation)
        return pose

    def _resolve_macro_start_pose(
        self,
        payload: Dict[str, Any],
        macro_name: str,
        policy: Dict[str, Any],
    ) -> Dict[str, Any]:
        start_pose = payload.get("start_pose")
        if not isinstance(start_pose, dict):
            raise ValueError(
                f"{macro_name} requires start_pose (legacy path; use draw_shape/draw_text with workplane.mode=tool for auto-pose)."
            )
        return self._resolve_target_pose(
            target_pose=start_pose,
            orientation_preset=policy.get("default_orientation_preset"),
            keep_current_orientation=False,
            allow_orientation_default=True,
        )


"""Route Semantic IR into public primitive commands without executing motion."""


from dataclasses import dataclass, field
import xml.etree.ElementTree as ET


try:  # pragma: no cover - optional in source-only test environments
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - exercised implicitly in tests
    get_package_share_directory = None


_ORIENTATION_PRESETS: Dict[str, Dict[str, float]] = {
    "tool-down": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
    "tool-forward": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
    "tool-up": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
}
_FRAME_BASE_LINK = "base_link"
_GP4_PLANNING_GROUP = "gp4_arm"
_GP4_JOINT_NAMES = (
    "joint_1_s",
    "joint_2_l",
    "joint_3_u",
    "joint_4_r",
    "joint_5_b",
    "joint_6_t",
)
GP4_JOINT_NAMES = _GP4_JOINT_NAMES
_GP4_JOINT_ALIAS_TO_INDEX: Dict[str, int] = {
    "1": 0,
    "s": 0,
    "joint1": 0,
    "joint1s": 0,
    "joint_1_s": 0,
    "2": 1,
    "l": 1,
    "joint2": 1,
    "joint2l": 1,
    "joint_2_l": 1,
    "3": 2,
    "u": 2,
    "joint3": 2,
    "joint3u": 2,
    "joint_3_u": 2,
    "4": 3,
    "r": 3,
    "joint4": 3,
    "joint4r": 3,
    "joint_4_r": 3,
    "5": 4,
    "b": 4,
    "joint5": 4,
    "joint5b": 4,
    "joint_5_b": 4,
    "6": 5,
    "t": 5,
    "joint6": 5,
    "joint6t": 5,
    "joint_6_t": 5,
}

def _copy_semantic_ir(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _copy_semantic_ir(child) for key, child in value.items()}
    if isinstance(value, list):
        return [_copy_semantic_ir(child) for child in value]
    return value

def _joint_alias_key(value: Any) -> str:
    return "".join(
        char for char in str(value or "").strip().lower() if char.isalnum()
    )

def resolve_gp4_joint_index(value: Any) -> int:
    if isinstance(value, int):
        index = value
    elif isinstance(value, float) and value.is_integer():
        index = int(value)
    else:
        text = str(value or "").strip().lower()
        if text in _GP4_JOINT_ALIAS_TO_INDEX:
            return _GP4_JOINT_ALIAS_TO_INDEX[text]
        key = _joint_alias_key(text)
        if key in _GP4_JOINT_ALIAS_TO_INDEX:
            return _GP4_JOINT_ALIAS_TO_INDEX[key]
        raise ValueError(f"unknown GP4 joint alias '{value}'")
    if 0 <= index < len(_GP4_JOINT_NAMES):
        return index
    if 1 <= index <= len(_GP4_JOINT_NAMES):
        return index - 1
    raise ValueError(f"joint_index {index} out of range [0, 5]")

def _current_joint_positions_for_delta(
    *,
    current_joint_positions_rad: Sequence[float] | None,
    current_joint_positions_by_name: Mapping[str, float] | None,
) -> list[float]:
    if current_joint_positions_by_name:
        by_name = {
            str(name): float(position)
            for name, position in current_joint_positions_by_name.items()
        }
        if all(joint_name in by_name for joint_name in _GP4_JOINT_NAMES):
            return [by_name[joint_name] for joint_name in _GP4_JOINT_NAMES]
    if current_joint_positions_rad is not None:
        positions = [float(value) for value in current_joint_positions_rad]
        if len(positions) >= len(_GP4_JOINT_NAMES):
            return positions[: len(_GP4_JOINT_NAMES)]
    raise ValueError(
        "current joint positions unavailable for relative joint move; "
        "cannot compute absolute target"
    )

def prepare_semantic_ir_for_routing(
    payload: Dict[str, Any],
    *,
    current_joint_positions_rad: Sequence[float] | None = None,
    current_joint_positions_by_name: Mapping[str, float] | None = None,
) -> Dict[str, Any]:
    """Resolve state-dependent Semantic IR before passing it to IntentRouter."""
    if not isinstance(payload, dict):
        return payload
    intent = str(payload.get("intent") or "").strip()
    if intent == "sequence":
        prepared = _copy_semantic_ir(payload)
        steps = prepared.get("steps")
        if isinstance(steps, list):
            prepared["steps"] = [
                prepare_semantic_ir_for_routing(
                    step,
                    current_joint_positions_rad=current_joint_positions_rad,
                    current_joint_positions_by_name=current_joint_positions_by_name,
                )
                if isinstance(step, dict)
                else step
                for step in steps
            ]
        return prepared
    if intent != "move_joint_delta":
        return _copy_semantic_ir(payload)

    positions = _current_joint_positions_for_delta(
        current_joint_positions_rad=current_joint_positions_rad,
        current_joint_positions_by_name=current_joint_positions_by_name,
    )
    raw_joint = (
        payload.get("joint_index")
        if "joint_index" in payload
        else payload.get("joint_name", payload.get("joint", payload.get("axis")))
    )
    joint_index = resolve_gp4_joint_index(raw_joint)
    if "delta_angle" not in payload:
        raise ValueError("move_joint_delta requires delta_angle")
    delta_rad = _convert_angular(
        _to_float(payload["delta_angle"], "delta_angle"),
        payload.get("angular_unit"),
        "delta_angle",
    )

    prepared = _copy_semantic_ir(payload)
    prepared["intent"] = "move_joint"
    prepared["joint_index"] = joint_index
    prepared["joint_angle"] = positions[joint_index] + delta_rad
    prepared["angular_unit"] = "rad"
    prepared.pop("delta_angle", None)
    prepared.pop("joint", None)
    prepared.pop("joint_name", None)
    prepared.pop("axis", None)
    return prepared


def _default_macro_policy_path() -> str:
    """Resolve macro_policy.yaml from the installed package or local source tree."""
    if get_package_share_directory is not None:
        try:
            pkg_share = get_package_share_directory("llm_gateway")
            candidate = os.path.join(pkg_share, "config", "macro_policy.yaml")
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


def _default_named_pose_srdf_path() -> str:
    """Resolve the GP4 MoveIt SRDF from install space or the local workspace."""
    if get_package_share_directory is not None:
        try:
            pkg_share = get_package_share_directory("gp4_moveit_config")
            candidate = os.path.join(pkg_share, "config", "motoman_gp4.srdf")
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass
    return str(
        Path(__file__).resolve().parents[2]
        / "gp4_moveit_config"
        / "config"
        / "motoman_gp4.srdf"
    )


def load_srdf_named_poses(srdf_path: str | None = None) -> Dict[str, List[float]]:
    """Load gp4_arm group_state joint targets from the MoveIt SRDF."""
    resolved_path = srdf_path or _default_named_pose_srdf_path()
    root = ET.parse(resolved_path).getroot()
    named_poses: Dict[str, List[float]] = {}
    for group_state in root.findall("group_state"):
        if group_state.attrib.get("group") != _GP4_PLANNING_GROUP:
            continue
        pose_name = str(group_state.attrib.get("name", "")).strip()
        if not pose_name:
            continue
        joints_by_name: Dict[str, float] = {}
        for joint in group_state.findall("joint"):
            joint_name = joint.attrib.get("name")
            joint_value = joint.attrib.get("value")
            if joint_name and joint_value is not None:
                joints_by_name[joint_name] = float(joint_value)
        if all(joint_name in joints_by_name for joint_name in _GP4_JOINT_NAMES):
            named_poses[pose_name] = [
                joints_by_name[joint_name] for joint_name in _GP4_JOINT_NAMES
            ]
    return named_poses


# ── Named-pose alias canonicalizer ──────────────────────────────────────────
# Maps common operator variations to the exact SRDF group_state name.
# Unknown inputs return None so callers can fail-closed.

_NAMED_POSE_ALIASES: Dict[str, str] = {
    # exact SRDF names (identity)
    "home": "home",
    "ready": "ready",
    "posea": "poseA",
    "poseb": "poseB",
    # English aliases
    "pose a": "poseA",
    "pose b": "poseB",
    "a": "poseA",
    "b": "poseB",
    "first": "poseA",
    "first pose": "poseA",
    "the first pose": "poseA",
    "point a": "poseA",
    "point b": "poseB",
    # Vietnamese aliases
    "điểm a": "poseA",
    "điểm b": "poseB",
    "diem a": "poseA",
    "diem b": "poseB",
    # Vietnamese "về X" (return to / go to X)
    "về a": "poseA",
    "về b": "poseB",
    "ve a": "poseA",
    "ve b": "poseB",
    "về home": "home",
    "về ready": "ready",
    "ve home": "home",
    "ve ready": "ready",
}


def canonicalize_named_pose(
    raw_name: str, available_poses: Dict[str, Any] | None = None
) -> str | None:
    """Return the canonical SRDF pose name for *raw_name*, or None if unknown.

    Lookup order:
      1. Exact match against *available_poses* keys (case-sensitive).
      2. Case-insensitive alias lookup via ``_NAMED_POSE_ALIASES``.
    """
    stripped = str(raw_name or "").strip()
    if not stripped:
        return None
    if available_poses is not None and stripped in available_poses:
        return stripped
    normalized = stripped.lower()
    if available_poses is not None and normalized in available_poses:
        return normalized
    return _NAMED_POSE_ALIASES.get(normalized)


def load_macro_policy(policy_path: str | None = None) -> Dict[str, Any]:
    resolved_path = policy_path or _default_macro_policy_path()
    with open(resolved_path, "r", encoding="utf-8") as policy_file:
        policy = yaml.safe_load(policy_file) or {}
    if not isinstance(policy, dict):
        raise ValueError("Macro policy root must be an object.")
    macros = policy.get("macros")
    if not isinstance(macros, dict):
        raise ValueError("Macro policy must define a macros object.")
    return policy


@dataclass(frozen=True)
class RouteResult:
    route_type: str
    commands: List[Dict[str, Any]]
    error_payload: Dict[str, Any] | None = None
    diagnostics: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SkillCall:
    name: str
    args: Dict[str, Any] = field(default_factory=dict)


def compile_goal(
    goal_dsl: Dict[str, Any],
    *,
    scene_graph: Any,
    live_scene: Dict[str, Any] | None = None,
) -> List[SkillCall]:
    if not isinstance(goal_dsl, dict):
        return [SkillCall("needs_clarification", {"field": "goal", "query": "non_object"})]
    action = str(goal_dsl.get("action") or "").strip()
    if action != "pick_and_place":
        return [SkillCall("capability_unavailable", {"action": action})]

    object_query = str(goal_dsl.get("object") or "").strip()
    destination_query = str(goal_dsl.get("destination") or "").strip()
    object_result = scene_graph.resolve_object(object_query, live_scene=live_scene)
    if not object_result.ok:
        return [
            SkillCall("needs_clarification", {"field": "object", "query": object_query})
        ]
    destination_result = scene_graph.resolve_region(destination_query)
    if not destination_result.ok:
        return [
            SkillCall(
                "needs_clarification",
                {"field": "destination", "query": destination_query},
            )
        ]

    return [
        SkillCall("refresh_scene"),
        SkillCall("approach_object", {"object_id": object_result.name}),
        SkillCall("pick_object", {"object_id": object_result.name}),
        SkillCall(
            "place_object",
            {"object_id": object_result.name, "destination": destination_result.name},
        ),
        SkillCall(
            "verify_postcondition",
            {"object_id": object_result.name, "destination": destination_result.name},
        ),
    ]


class IntentRouter(DrawRouterMixin):
    """Translate Semantic IR into the frozen public primitive set."""

    def __init__(
        self,
        macro_policy_path: str | None = None,
        runtime_mode: str = "hardware",
        named_pose_srdf_path: str | None = None,
        station_semantic_map: Dict[str, Any] | None = None,
    ) -> None:
        self._macro_policy = load_macro_policy(macro_policy_path)
        self._runtime_mode = str(runtime_mode).strip().lower() or "hardware"
        self._named_pose_targets = load_srdf_named_poses(named_pose_srdf_path)
        self._station_semantic_map = station_semantic_map

    def route(self, payload: Dict[str, Any]) -> RouteResult:
        if not isinstance(payload, dict):
            raise ValueError("Semantic IR payload must be an object.")

        if "primitive_type" in payload:
            return RouteResult(
                route_type="primitive",
                commands=[deepcopy(payload)],
                metadata={"source": "primitive_passthrough"},
            )

        if "error" in payload:
            return RouteResult(
                route_type="error",
                commands=[],
                error_payload=deepcopy(payload),
                diagnostics=[str(payload["error"])],
                metadata={"source": "error_passthrough"},
            )

        intent = payload.get("intent")
        if not isinstance(intent, str) or not intent.strip():
            raise ValueError("Semantic IR requires a non-empty intent field.")

        normalized_intent = intent.strip()
        if normalized_intent == "sequence":
            return self._route_sequence(payload)
        if normalized_intent == "draw_shape":
            return self._route_draw_shape(payload)
        if normalized_intent == "draw_text":
            return self._route_draw_text(payload)

        return RouteResult(
            route_type="primitive",
            commands=[self._route_single_intent(payload)],
            metadata={"source": "semantic_ir", "intent": normalized_intent},
        )

    def _route_sequence(self, payload: Dict[str, Any]) -> RouteResult:
        steps = payload.get("steps")
        if not isinstance(steps, list) or not steps:
            raise ValueError("sequence intent requires a non-empty steps list.")

        commands: List[Dict[str, Any]] = []
        for index, step in enumerate(steps):
            if not isinstance(step, dict):
                raise ValueError(f"sequence step {index} must be an object.")
            routed_step = self.route(step)
            if routed_step.route_type == "error":
                error_code = routed_step.error_payload.get("error", "unknown_error")
                raise ValueError(
                    f"sequence step {index} is an error payload: {error_code}"
                )
            commands.extend(deepcopy(routed_step.commands))

        return RouteResult(
            route_type="sequence",
            commands=commands,
            metadata={"source": "semantic_ir", "intent": "sequence"},
        )

    def _route_single_intent(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        intent = str(payload["intent"])

        if intent == "go_home":
            return self._base_command("HOME", payload)
        if intent == "absolute_move_ptp":
            return self._route_absolute_move(payload, primitive_type="PTP")
        if intent == "move_named_pose":
            return self._route_named_pose(payload)
        if intent == "absolute_move_lin":
            return self._route_absolute_move(payload, primitive_type="LIN")
        if intent == "move_relative":
            return self._route_move_relative(payload)
        if intent == "move_joint":
            return self._route_move_joint(payload)
        if intent == "move_joint_delta":
            raise ValueError(
                "move_joint_delta must be resolved to move_joint before routing; "
                "current joint positions were unavailable."
            )
        if intent == "move_joints":
            return self._route_move_joints(payload)
        if intent == "set_speed":
            if "velocity_scale" not in payload:
                raise ValueError("set_speed requires velocity_scale.")
            return {
                "primitive_type": "SET_SPEED",
                "velocity_scale": float(payload["velocity_scale"]),
            }
        if intent == "wait":
            duration = payload.get("wait_duration_sec")
            if duration is None:
                # Default to 2.0 s when the LLM omits the field but the intent
                # is clearly "wait".  Matches the system prompt contract:
                # "default 2.0 if unspecified but clear intent".
                duration = 2.0
            return {
                "primitive_type": "WAIT",
                "wait_duration_sec": float(duration),
            }
        if intent == "stop":
            return {"primitive_type": "STOP"}
        if intent == "io_set":
            if "io_address" not in payload:
                raise ValueError("io_set requires io_address.")
            if "io_value" not in payload:
                raise ValueError("io_set requires io_value.")
            return {
                "primitive_type": "IO_SET",
                "io_address": int(payload["io_address"]),
                "io_value": int(payload["io_value"]),
            }
        if intent == "alarm_reset":
            return {"primitive_type": "ALARM_RESET"}
        if intent == "get_pose":
            command = {"primitive_type": "GET_POSE"}
            reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
            self._validate_reference_frame(reference_frame)
            command["reference_frame"] = reference_frame
            return command
        if intent == "return_to_start":
            return self._route_return_to_start(payload)
        if intent == "circular_move":
            return self._route_circular_move(payload)

        raise ValueError(f"unsupported semantic intent '{intent}'")

    def _route_named_pose(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        raw_pose = str(payload.get("pose_name") or payload.get("name") or "").strip()
        if not raw_pose:
            raise ValueError("move_named_pose requires pose_name.")

        # 1. Check SRDF named poses first (Joint targets)
        pose_name = canonicalize_named_pose(raw_pose, self._named_pose_targets)
        if pose_name is not None and pose_name in self._named_pose_targets:
            joint_target = self._named_pose_targets.get(pose_name)
            command = self._base_command("PTP", payload)
            command["joint_target"] = list(joint_target)
            command["planner_id"] = str(command.get("planner_id") or "PILZ_PTP")
            command["reference_frame"] = _FRAME_BASE_LINK
            return command

        # 2. Check semantic map regions (Cartesian targets)
        if self._station_semantic_map is not None:
            from llm_gateway.factory_task import StationSceneGraph
            sg = StationSceneGraph(self._station_semantic_map)
            region_res = sg.resolve_region(raw_pose)
            if region_res.ok and isinstance(region_res.payload, dict):
                region_data = region_res.payload
                center = region_data.get("geometry", {}).get("center", {})
                size = region_data.get("geometry", {}).get("size", {})

                # Calculate safe approach point: Top of the bounding box + 10cm clearance
                z_clearance = 0.10
                safe_z = float(center.get("z", 0.0)) + float(size.get("z", 0.0)) / 2.0 + z_clearance

                command = self._base_command("PTP", payload)
                command["reference_frame"] = str(region_data.get("frame_id", _FRAME_BASE_LINK))
                command["target_pose"] = {
                    "position": {
                        "x": float(center.get("x", 0.0)),
                        "y": float(center.get("y", 0.0)),
                        "z": safe_z
                    }
                }
                # Keep current orientation to point downwards
                command["keep_current_orientation"] = True
                command["planner_id"] = str(command.get("planner_id") or "PILZ_PTP")
                return command

        raise ValueError(
            f"unknown named pose or region '{raw_pose}'; "
            f"available joint poses: {sorted(self._named_pose_targets)}"
        )

    def _route_absolute_move(
        self, payload: Dict[str, Any], *, primitive_type: str
    ) -> Dict[str, Any]:
        target_pose = payload.get("target_pose")
        if not isinstance(target_pose, dict):
            raise ValueError(f"{payload['intent']} requires target_pose.")

        reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
        self._validate_reference_frame(reference_frame)

        command = self._base_command(primitive_type, payload)
        command["reference_frame"] = reference_frame
        command["target_pose"] = self._resolve_target_pose(
            target_pose=target_pose,
            orientation_preset=payload.get("orientation_preset"),
            keep_current_orientation=payload.get("keep_current_orientation"),
        )
        return command

    def _route_circular_move(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Convert circular_move semantic intent to CIRC primitive.

        CIRC requires:
          - target_pose: final arc endpoint
          - waypoints: [auxiliary_pose] — the arc via-point
        """
        target_pose = payload.get("target_pose")
        if not isinstance(target_pose, dict):
            raise ValueError("circular_move requires target_pose.")

        auxiliary_pose = payload.get("auxiliary_pose")
        if not isinstance(auxiliary_pose, dict):
            raise ValueError("circular_move requires auxiliary_pose (arc via-point).")

        reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
        self._validate_reference_frame(reference_frame)

        resolved_target = self._resolve_target_pose(
            target_pose=target_pose,
            orientation_preset=payload.get("orientation_preset"),
            keep_current_orientation=payload.get("keep_current_orientation"),
        )
        resolved_auxiliary = self._resolve_target_pose(
            target_pose=auxiliary_pose,
            orientation_preset=payload.get("orientation_preset"),
            keep_current_orientation=payload.get("keep_current_orientation"),
        )

        command = self._base_command("CIRC", payload)
        command["reference_frame"] = reference_frame
        command["target_pose"] = resolved_target
        command["waypoints"] = [resolved_auxiliary]
        return command

    def _route_move_relative(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        delta = payload.get("delta")
        if not isinstance(delta, dict):
            raise ValueError("relative move requires direction and distance.")

        reference_frame = payload.get("reference_frame", _FRAME_BASE_LINK)
        self._validate_reference_frame(reference_frame)
        return {
            "primitive_type": "MOVE_REL",
            "delta_x": float(delta.get("x", 0.0)),
            "delta_y": float(delta.get("y", 0.0)),
            "delta_z": float(delta.get("z", 0.0)),
            "reference_frame": reference_frame,
            **self._optional_motion_fields(payload),
        }

    def _route_move_joint(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if "joint_index" not in payload:
            raise ValueError("move_joint requires joint_index.")
        if "joint_angle" not in payload:
            raise ValueError("move_joint requires joint_angle.")
        return {
            "primitive_type": "MOVE_JOINT",
            "joint_index": int(payload["joint_index"]),
            "joint_angle": float(payload["joint_angle"]),
            **self._optional_motion_fields(payload),
        }

    def _route_move_joints(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        joint_target = payload.get("joint_target")
        if not isinstance(joint_target, list):
            raise ValueError("move_joints requires joint_target.")
        return {
            "primitive_type": "MOVE_JOINTS",
            "joint_target": [float(value) for value in joint_target],
            **self._optional_motion_fields(payload),
        }

    def _route_return_to_start(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Convert return_to_start semantic intent to captured MOVE_JOINTS.

        If the payload carries a captured ``joint_target`` (e.g. from a
        sequence pose snapshot), use it directly. Without that snapshot,
        fail closed instead of guessing that HOME is the original start.
        """
        joint_target = payload.get("joint_target")
        if isinstance(joint_target, list) and len(joint_target) == 6:
            return {
                "primitive_type": "MOVE_JOINTS",
                "joint_target": [float(v) for v in joint_target],
                **self._optional_motion_fields(payload),
            }
        raise ValueError("return_to_start requires a captured joint_target.")

    def _resolve_position(
        self, raw_position: Any, *, field_prefix: str
    ) -> Dict[str, float]:
        if not isinstance(raw_position, dict):
            raise ValueError(f"{field_prefix} must be an object with x/y/z.")
        resolved_position: Dict[str, float] = {}
        for axis in ("x", "y", "z"):
            if axis not in raw_position:
                raise ValueError(f"{field_prefix}.{axis} is required.")
            resolved_position[axis] = float(raw_position[axis])
        return resolved_position

    def _require_positive_float(
        self,
        payload: Dict[str, Any],
        field_name: str,
        error_message: str,
    ) -> float:
        value = float(payload.get(field_name, 0.0))
        if value <= 0.0:
            raise ValueError(error_message)
        return value

    def _base_command(
        self, primitive_type: str, payload: Dict[str, Any]
    ) -> Dict[str, Any]:
        return {
            "primitive_type": primitive_type,
            **self._optional_motion_fields(payload),
        }

    def _optional_motion_fields(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command: Dict[str, Any] = {}
        for field in (
            "velocity_scale",
            "acceleration_scale",
            "planner_id",
            "linear_unit",
            "angular_unit",
        ):
            if field in payload:
                command[field] = payload[field]
        return command

    def _resolve_target_pose(
        self,
        *,
        target_pose: Dict[str, Any],
        orientation_preset: Any,
        keep_current_orientation: Any,
        allow_orientation_default: bool = False,
    ) -> Dict[str, Any]:
        resolved_pose = deepcopy(target_pose)
        position = resolved_pose.get("position")
        if not isinstance(position, dict):
            raise ValueError("target_pose.position must be an object.")

        if "orientation" in resolved_pose and orientation_preset is not None:
            raise ValueError(
                "Provide either target_pose.orientation or orientation_preset, not both."
            )

        if "orientation" in resolved_pose:
            return resolved_pose

        if orientation_preset is not None:
            resolved_pose["orientation"] = self._orientation_from_preset(
                orientation_preset
            )
            return resolved_pose

        if keep_current_orientation not in (None, True, False):
            raise ValueError(
                "keep_current_orientation must be a boolean when provided."
            )

        if allow_orientation_default:
            return resolved_pose

        if keep_current_orientation is False:
            raise ValueError(
                "Explicit orientation is required when keep_current_orientation is false."
            )
        return resolved_pose

    def _orientation_from_preset(self, preset_name: Any) -> Dict[str, float]:
        normalized_name = str(preset_name).strip().lower().replace("_", "-")
        preset = _ORIENTATION_PRESETS.get(normalized_name)
        if preset is None:
            raise ValueError(
                f"unsupported orientation_preset '{preset_name}'; "
                f"supported presets: {sorted(_ORIENTATION_PRESETS)}"
            )
        return deepcopy(preset)

    def _validate_reference_frame(
        self,
        reference_frame: Any,
        *,
        allowed_frames: set[str] | None = None,
    ) -> None:
        normalized_frame = str(reference_frame).strip()
        frames = allowed_frames if allowed_frames is not None else {_FRAME_BASE_LINK}
        if normalized_frame not in frames:
            raise ValueError(
                f"unsupported reference_frame '{normalized_frame}'; "
                f"supported frames: {sorted(frames)}"
            )


__all__ = [
    "GoalMapper",
    "IntentRouter",
    "LLMParser",
    "Normalizer",
    "RouteResult",
    "SchemaValidator",
    "SemanticValidator",
    "SequenceValidationError",
    "SequenceValidationResult",
    "SequenceValidator",
    "canonicalize_named_pose",
    "command_from_sanitized_json",
    "compile_strokes_to_commands",
    "hydrate_draw_workplane",
    "lift_points_to_poses",
    "load_macro_policy",
    "load_srdf_named_poses",
    "normalize_joints",
    "normalize_pose",
    "parse_llm_output",
    "prepare_execution_command",
]
