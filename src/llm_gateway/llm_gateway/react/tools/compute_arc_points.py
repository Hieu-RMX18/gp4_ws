"""Compute arc points — local geometry tool for CIRC auxiliary poses."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, ClassVar

from ..tool_registry import Tool, ToolResult

if TYPE_CHECKING:
    from ..agent import AgentContext


def _normalize(v: list[float]) -> list[float]:
    norm = math.sqrt(sum(x * x for x in v))
    if norm < 1e-6:
        raise ValueError("zero vector")
    return [x / norm for x in v]


def _cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _quaternion_from_vectors(forward: list[float], up: list[float]) -> dict:
    """Build a quaternion that rotates +Z to `up` and +X to `forward`."""
    fx, fy, fz = forward
    ux, uy, uz = up
    # Rotation matrix: columns are forward, up cross forward, up
    cx = _normalize(_cross(up, forward))
    cy = forward
    cz = up
    # Convert to quaternion (x, y, z, w)
    w = math.sqrt(max(0.0, 1.0 + cx[0] + cy[1] + cz[2])) / 2.0
    x = math.sqrt(max(0.0, 1.0 + cx[0] - cy[1] - cz[2])) / 2.0
    y = math.sqrt(max(0.0, 1.0 - cx[0] + cy[1] - cz[2])) / 2.0
    z = math.sqrt(max(0.0, 1.0 - cx[0] - cy[1] + cz[2])) / 2.0
    # Correct signs
    if cx[1] < cy[0]:
        x = -x
    if cx[2] < cz[0]:
        y = -y
    if cy[2] < cz[1]:
        z = -z
    return {"x": x, "y": y, "z": z, "w": w}


class ComputeArcPointsTool(Tool):
    name = "compute_arc_points"
    description = (
        "Compute start, auxiliary, and target poses for a circular arc (CIRC)."
    )
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "center": {
                "type": "object",
                "required": ["x", "y", "z"],
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                },
            },
            "radius_m": {"type": "number"},
            "start_angle_rad": {"type": "number"},
            "sweep_angle_rad": {"type": "number"},
            "plane_normal": {
                "type": "object",
                "required": ["x", "y", "z"],
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"},
                },
            },
        },
        "required": [
            "center",
            "radius_m",
            "start_angle_rad",
            "sweep_angle_rad",
            "plane_normal",
        ],
    }

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        center = args["center"]
        radius_m = float(args["radius_m"])
        start_angle = float(args["start_angle_rad"])
        sweep = float(args["sweep_angle_rad"])
        n_raw = args["plane_normal"]

        if radius_m <= 0.0:
            return ToolResult(ok=False, error="radius_m must be > 0")
        if sweep == 0.0:
            return ToolResult(ok=False, error="sweep_angle_rad must be non-zero")
        if abs(sweep) > 2.0 * math.pi:
            return ToolResult(ok=False, error="|sweep_angle_rad| must not exceed 2*pi")

        n = _normalize([n_raw["x"], n_raw["y"], n_raw["z"]])
        if math.sqrt(sum(x * x for x in n)) < 1e-6:
            return ToolResult(ok=False, error="plane_normal is degenerate")

        if abs(n[2]) > 0.9:
            u = [1.0, 0.0, 0.0]
        elif abs(n[1]) > 0.9:
            u = [1.0, 0.0, 0.0]
        else:
            u = _normalize(_cross([0.0, 0.0, 1.0], n))
        v = _cross(n, u)

        def _pose_at(angle: float) -> dict:
            x = center["x"] + radius_m * (
                u[0] * math.cos(angle) + v[0] * math.sin(angle)
            )
            y = center["y"] + radius_m * (
                u[1] * math.cos(angle) + v[1] * math.sin(angle)
            )
            z = center["z"] + radius_m * (
                u[2] * math.cos(angle) + v[2] * math.sin(angle)
            )
            # Tangent vector = derivative w.r.t angle (counter-clockwise when sweep>0)
            tx = -u[0] * math.sin(angle) + v[0] * math.cos(angle)
            ty = -u[1] * math.sin(angle) + v[1] * math.cos(angle)
            tz = -u[2] * math.sin(angle) + v[2] * math.cos(angle)
            forward = _normalize([tx, ty, tz])
            up = n
            q = _quaternion_from_vectors(forward, up)
            return {
                "header": {"frame_id": "base_link"},
                "pose": {
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": q,
                },
            }

        aux_angle = start_angle + sweep / 2.0
        end_angle = start_angle + sweep
        return ToolResult(
            ok=True,
            payload={
                "start_pose": _pose_at(start_angle),
                "auxiliary_pose": _pose_at(aux_angle),
                "target_pose": _pose_at(end_angle),
            },
        )
