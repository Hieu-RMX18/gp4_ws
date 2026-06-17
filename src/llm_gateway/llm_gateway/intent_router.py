"""IntentRouter and Semantic IR routing helpers.

Extracted from factory_task.py (R1) — behavior-preserving relocation.
"""
from __future__ import annotations

import logging
import os
import xml.etree.ElementTree as ET
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

import yaml

from llm_gateway.drawing_router import DrawRouterMixin, RouteResult
from llm_gateway.normalization import _convert_angular, _to_float

_LOGGER = logging.getLogger(__name__)

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


def _default_station_semantic_map_path() -> str:
    """Resolve station_semantic_map.yaml from installed package or local source tree."""
    if get_package_share_directory is not None:
        try:
            pkg_share = get_package_share_directory("llm_gateway")
            candidate = os.path.join(pkg_share, "config", "station_semantic_map.yaml")
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass
    return str(Path(__file__).resolve().parents[1] / "config" / "station_semantic_map.yaml")



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
        if station_semantic_map is None:
            try:
                path = _default_station_semantic_map_path()
                if os.path.exists(path):
                    with open(path, "r", encoding="utf-8") as f:
                        station_semantic_map = yaml.safe_load(f) or {}
            except Exception:
                pass
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

        pose_name = canonicalize_named_pose(raw_pose, self._named_pose_targets)
        if pose_name is not None and pose_name in self._named_pose_targets:
            joint_target = self._named_pose_targets.get(pose_name)
            command = self._base_command("PTP", payload)
            command["joint_target"] = list(joint_target)
            command["planner_id"] = str(command.get("planner_id") or "PILZ_PTP")
            command["reference_frame"] = _FRAME_BASE_LINK
            return command

        if self._station_semantic_map is not None:
            from llm_gateway.factory_task import StationSceneGraph
            sg = StationSceneGraph(self._station_semantic_map)
            region_res = sg.resolve_region(raw_pose)
            if region_res.ok and isinstance(region_res.payload, dict):
                region_data = region_res.payload
                center = region_data.get("geometry", {}).get("center", {})
                size = region_data.get("geometry", {}).get("size", {})

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
