"""Schema + semantic + sequence validation for llm_gateway commands.

Extracted from factory_task.py (R1) — behavior-preserving relocation.
"""
from __future__ import annotations

import json
import logging
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import jsonschema
import numpy as np
import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

from llm_gateway.normalization import Normalizer

_LOGGER = logging.getLogger(__name__)

_QUERY_PRIMITIVES = {"GET_POSE"}
_FRAME_REQUIRED_PRIMITIVES = {"PTP", "LIN", "MOVE_REL", "CARTESIAN_PATH"}
_SUPPORTED_SEQUENCE_FRAMES = {"base_link"}

# Failsafe motion limits — hardcoded safety boundary. These mirror the identical
# values in safety.policy_loader._FAILSAFE_MOTION_LIMITS so factory_task stays
# importable when the safety package is not on the path (e.g. HMI backend tests
# running outside a colcon workspace).
_FAILSAFE_MOTION_LIMITS: dict[str, float] = {
    "max_velocity_scale": 0.06,
    "max_acceleration_scale": 0.06,
    "max_move_rel_translation": 0.21,
}


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
