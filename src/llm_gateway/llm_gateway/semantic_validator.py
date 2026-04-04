"""Semantic validation for phase-9 LLM motion commands."""

from __future__ import annotations

import math
import os
from typing import Any, Dict
import yaml
from ament_index_python.packages import get_package_share_directory


class SemanticValidator:
    """Enforce phase-specific primitive, workspace, and scaling constraints."""

    _ALLOWED_PRIMITIVES = {"HOME", "PTP", "LIN"}
    _MIN_VELOCITY_SCALE = 0.05
    _MAX_VELOCITY_SCALE = 0.30

    # Fallback bounds — overridden by safety_rules.yaml at construction
    _DEFAULT_BOUNDS = {
        "x": (0.0, 0.6),
        "y": (-0.3, 0.3),
        "z": (0.2, 0.6),
    }

    def __init__(self, safety_rules: dict | None = None):
        if safety_rules is None:
            safety_rules = self._load_safety_rules()
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

        velocity_scale = float(command.get("velocity_scale", 0.0))
        if not (self._MIN_VELOCITY_SCALE <= velocity_scale <= self._MAX_VELOCITY_SCALE):
            raise ValueError(
                "velocity_scale must be within "
                f"[{self._MIN_VELOCITY_SCALE:.2f}, {self._MAX_VELOCITY_SCALE:.2f}]."
            )

        has_pose = "target_pose_msg" in command
        has_joints = bool(command.get("joint_target"))

        if primitive_type == "HOME":
            if has_pose or has_joints:
                raise ValueError("HOME must not include target_pose or joint_target.")
            return True

        if primitive_type == "LIN" and not has_pose:
            raise ValueError("LIN requires target_pose.")

        if primitive_type == "PTP" and not (has_pose or has_joints):
            raise ValueError("PTP requires target_pose or joint_target.")

        if has_pose:
            self._validate_pose(command["target_pose_msg"])

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
