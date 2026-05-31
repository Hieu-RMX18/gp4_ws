from __future__ import annotations

from typing import Any

from ..domain.constants import GP4_JOINT_NAMES as DEFAULT_JOINT_NAMES
from ..domain.joint_utils import read_joint_position_deg, resolve_joint_target


class JogDispatchMixin:
    """Jog-related command helpers for WorkspaceRosAdapter.

    This mixin owns MOVE_JOINT delta payload normalization and joint-target
    resolution helpers. Transport-level jog pendant publish/subscribe remains
    in hmi.backend.services.jog_service.
    """

    def _build_move_cartesian_delta_payload(
        self, parameters: dict[str, Any]
    ) -> dict[str, Any]:
        frame = str(parameters.get("frame") or "base_link")
        if frame not in {"", "base_link"}:
            raise ValueError(
                f"Unsupported MOVE_REL reference frame '{frame}' for supervisor execution."
            )
        return {
            "primitive_type": "MOVE_REL",
            "delta_x": float(parameters.get("xMm", 0.0)) / 1000.0,
            "delta_y": float(parameters.get("yMm", 0.0)) / 1000.0,
            "delta_z": float(parameters.get("zMm", 0.0)) / 1000.0,
            "reference_frame": "base_link",
            "velocity_scale": 0.06,
            "acceleration_scale": 0.06,
            "planner_id": "PILZ_LIN",
        }

    def _build_move_joint_delta_payload(
        self, parameters: dict[str, Any]
    ) -> dict[str, Any]:
        joint_index, _joint_name = self._resolve_joint_target(parameters)
        if joint_index is None:
            raise ValueError(
                "Joint delta command did not resolve to a valid GP4 joint."
            )
        target_deg = parameters.get("resolvedTargetDeg")
        if target_deg is None:
            target_deg = self._resolve_joint_target_deg(joint_index, parameters)
        return {
            "primitive_type": "MOVE_JOINT",
            "joint_index": joint_index,
            "joint_angle": float(target_deg) * 3.141592653589793 / 180.0,
            "velocity_scale": 0.06,
            "acceleration_scale": 0.06,
            "planner_id": "PILZ_PTP",
        }

    def _resolve_joint_target(
        self,
        parameters: dict[str, Any],
    ) -> tuple[int | None, str | None]:
        return resolve_joint_target(parameters, DEFAULT_JOINT_NAMES)

    def _resolve_joint_target_deg(
        self, joint_index: int, parameters: dict[str, Any]
    ) -> float:
        current_position_deg = parameters.get("currentPositionDeg")
        if current_position_deg is None:
            joint_name = DEFAULT_JOINT_NAMES[joint_index]
            current_position_deg = self._read_joint_position_deg(joint_name)
        if current_position_deg is None:
            raise ValueError(
                f"Fresh joint position for {DEFAULT_JOINT_NAMES[joint_index]} is unavailable."
            )
        delta_deg = float(parameters.get("deltaDeg", 0.0))
        return float(current_position_deg) + delta_deg

    def _read_joint_position_deg(self, joint_name: str) -> float | None:
        return read_joint_position_deg(joint_name, self.read_joint_positions())
