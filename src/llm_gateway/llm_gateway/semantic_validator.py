"""Semantic validation for phase-9 LLM motion commands."""

from __future__ import annotations

import math
import os
from typing import Any, Dict
import yaml
from ament_index_python.packages import get_package_share_directory


class SemanticValidator:
    """Enforce phase-specific primitive, workspace, and scaling constraints."""

    _ALLOWED_PRIMITIVES = {
        "HOME", "PTP", "LIN", "CIRC", "MOVE_REL", "GET_POSE", "CARTESIAN_PATH",
        "SET_SPEED", "WAIT", "STOP", "MOVE_JOINT", "MOVE_JOINTS",
        "IO_SET", "ALARM_RESET",
    }
    _MIN_VELOCITY_SCALE = 0.05
    _MAX_VELOCITY_SCALE = 0.06

    # MOVE_REL: max single-command translation norm (meters).
    # Fallback; runtime loads from safety_rules.yaml motion_limits.max_move_rel_translation.
    # This pass raises fallback from 0.03 to 0.08.
    _MAX_MOVE_REL_DELTA = 0.08

    # GP4 has 6 joints (0..5)
    _NUM_JOINTS = 6

    # Fallback bounds — overridden by safety_rules.yaml at construction
    _DEFAULT_BOUNDS = {
        "x": (-0.25, 0.38),
        "y": (-0.25, 0.38),
        "z": (0.20, 0.56),
    }

    def __init__(self, safety_rules: dict | None = None):
        if safety_rules is None:
            safety_rules = self._load_safety_rules()
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

    @staticmethod
    def _load_safety_rules() -> dict:
        try:
            pkg_share = get_package_share_directory('safety')
            yaml_path = os.path.join(pkg_share, 'config', 'safety_rules.yaml')
            with open(yaml_path, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception:
            return {}

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
            if command.get("target_pose_msg") or command.get("target_pose") or command.get("joint_target"):
                raise ValueError("GET_POSE must not include target_pose or joint_target.")
            return True

        # ── STOP: no fields needed ──
        if primitive_type == "STOP":
            if command.get("target_pose_msg") or command.get("target_pose") or command.get("joint_target"):
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

        # ── IO_SET: address required, value must be 0 or 1 ──
        if primitive_type == "IO_SET":
            if "io_address" not in command:
                raise ValueError("IO_SET requires io_address.")
            io_val = command.get("io_value")
            if io_val is None:
                raise ValueError("IO_SET requires io_value.")
            if int(io_val) not in (0, 1):
                raise ValueError(
                    f"IO_SET: io_value must be 0 or 1, got {io_val}."
                )
            return True

        # ── CIRC: circular motion via Pilz — target_pose + 1 auxiliary waypoint ──
        if primitive_type == "CIRC":
            waypoints = command.get("waypoints_msg")
            if not waypoints:
                raise ValueError("CIRC requires non-empty waypoints (exactly 1 auxiliary pose).")
            if len(waypoints) != 1:
                raise ValueError(f"CIRC requires exactly 1 auxiliary waypoint, got {len(waypoints)}.")
            if "target_pose_msg" not in command:
                raise ValueError("CIRC requires target_pose_msg (final pose).")
            self._validate_pose(command["target_pose_msg"])
            try:
                self._validate_pose(waypoints[0])
            except ValueError as e:
                raise ValueError(f"CIRC auxiliary waypoint[0]: {e}") from e
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
                    raise ValueError(
                        f"CARTESIAN_PATH waypoint[{i}]: {e}"
                    ) from e
            return True

        if primitive_type == "LIN" and not has_pose:
            raise ValueError("LIN requires target_pose.")

        if primitive_type == "PTP" and not (has_pose or has_joints):
            raise ValueError("PTP requires target_pose or joint_target.")

        if has_pose:
            self._validate_pose(command["target_pose_msg"])

        return True

    def _validate_move_rel(self, command: Dict[str, Any]) -> bool:
        """Validate MOVE_REL: translation-only relative motion."""
        if command.get("target_pose_msg") or command.get("joint_target"):
            raise ValueError(
                "MOVE_REL must not include target_pose or joint_target."
            )

        for field in ("delta_x", "delta_y", "delta_z"):
            if field not in command:
                raise ValueError(f"MOVE_REL requires {field}.")
            if not math.isfinite(float(command[field])):
                raise ValueError(f"MOVE_REL: {field} must be a finite number.")

        dx = float(command["delta_x"])
        dy = float(command["delta_y"])
        dz = float(command["delta_z"])

        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            raise ValueError(
                "MOVE_REL: at least one delta component must be non-zero."
            )

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
            lower, upper = self._workspace_bounds[axis]   # ← instance attr
            if not (lower <= value <= upper):
                raise ValueError(
                    f"target_pose.position.{axis}={value:.4f} is outside "
                    f"configured workspace [{lower}, {upper}].")

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
