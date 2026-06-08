from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from llm_gateway.station_scene_graph import map_contains_verify_config


@dataclass(frozen=True)
class CandidatePoseRequest:
    purpose: str
    region: dict[str, Any]
    safety_rules: dict[str, Any]
    tcp_offset_m: float = 0.0
    approach_axis: str = "+z_base"


@dataclass(frozen=True)
class CandidatePoseResult:
    ok: bool
    poses: list[dict[str, Any]] = field(default_factory=list)
    error: str = ""
    rejected: list[str] = field(default_factory=list)


def generate_candidate_poses(request: CandidatePoseRequest) -> CandidatePoseResult:
    geometry = request.region.get("geometry", {}) if isinstance(request.region, dict) else {}
    if map_contains_verify_config(geometry):
        return CandidatePoseResult(ok=False, error="verify_config_required")
    center = geometry.get("center") if isinstance(geometry, dict) else None
    if not isinstance(center, dict):
        return CandidatePoseResult(ok=False, error="needs_clarification")

    pose = {
        "position": {
            "x": float(center.get("x", 0.0)),
            "y": float(center.get("y", 0.0)),
            "z": float(center.get("z", 0.0)),
        },
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }
    _apply_axis_offset_once(pose["position"], request.approach_axis, request.tcp_offset_m)
    reason = _workspace_rejection(pose["position"], request.safety_rules)
    if reason:
        return CandidatePoseResult(ok=False, error="safety_rejected", rejected=[reason])
    return CandidatePoseResult(ok=True, poses=[pose])


def _apply_axis_offset_once(position: dict[str, float], axis: str, offset_m: float) -> None:
    if offset_m == 0.0:
        return
    axis_map = {
        "+x_base": ("x", 1.0),
        "-x_base": ("x", -1.0),
        "+y_base": ("y", 1.0),
        "-y_base": ("y", -1.0),
        "+z_base": ("z", 1.0),
        "-z_base": ("z", -1.0),
    }
    field, sign = axis_map.get(axis, ("z", 1.0))
    position[field] = round(float(position[field]) + sign * float(offset_m), 6)


def _workspace_rejection(position: dict[str, float], safety_rules: dict[str, Any]) -> str:
    bounds = safety_rules.get("workspace_bounds", {}) if isinstance(safety_rules, dict) else {}
    checks = (("x", "x_min", "x_max"), ("y", "y_min", "y_max"), ("z", "z_min", "z_max"))
    for axis, low_key, high_key in checks:
        low = float(bounds.get(low_key, float("-inf")))
        high = float(bounds.get(high_key, float("inf")))
        value = float(position[axis])
        if not (low <= value <= high):
            return f"{axis}={value:.4f} outside [{low:.4f}, {high:.4f}]"
    return ""
