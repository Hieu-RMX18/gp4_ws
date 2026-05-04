"""Semantic validation for phase-9 LLM motion commands."""

from __future__ import annotations

import logging
import math
from typing import Any, Dict

import numpy as np

from safety.policy_loader import _FAILSAFE_MOTION_LIMITS, load_safety_rules


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
        "z": (0.23, 0.56),
    }

    def __init__(self, safety_rules: dict | None = None):
        if safety_rules is None:
            safety_rules = load_safety_rules()
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
