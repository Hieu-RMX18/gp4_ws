from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, ClassVar

import jsonschema

from llm_gateway.semantic_ir_contract import validate_semantic_ir_contract
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


@dataclass
class ToolResult:
    ok: bool
    payload: dict | None = None
    error: str | None = None

    def to_observation(self) -> str:
        if self.ok:
            return json.dumps({"ok": True, "payload": self.payload})
        return json.dumps({"ok": False, "error": self.error})


class _CompositeTool:
    name: ClassVar[str] = ""
    description: ClassVar[str] = ""
    input_schema: ClassVar[dict] = {}
    is_motion: ClassVar[bool] = False
    is_readonly: ClassVar[bool] = False

    def validate_input(self, args: dict) -> None:
        if self.input_schema:
            jsonschema.validate(instance=args, schema=self.input_schema)


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


class EmitSequenceTool(_CompositeTool):
    name = "emit_sequence"
    description = "Build a validated Semantic IR sequence from child Semantic IR steps."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {"steps": {"type": "array", "items": {"type": "object"}}},
        "required": ["steps"],
    }

    def invoke(self, args: dict, context) -> ToolResult:
        semantic_ir = {
            "intent": "sequence",
            "steps": list(args["steps"]),
            "metadata": {"source": "emit_sequence"},
        }
        contract = validate_semantic_ir_contract(semantic_ir)
        if not contract.valid:
            return ToolResult(ok=False, error=contract.reason)
        return ToolResult(ok=True, payload={"semantic_ir": semantic_ir})


class RefreshSceneTool(_CompositeTool):
    name = "refresh_scene"
    description = "Invalidate cached perception so the next scene query is fresh."
    is_readonly = True
    input_schema: ClassVar[dict] = {"type": "object", "properties": {}}

    def invoke(self, args: dict, context) -> ToolResult:
        invalidate = getattr(getattr(context, "ros_node", None), "_invalidate_scene_cache", None)
        if callable(invalidate):
            invalidate()
        return ToolResult(ok=True, payload={"scene_cache_invalidated": True})


class PickObjectTool(_CompositeTool):
    name = "pick_object"
    description = "Emit a fail-closed composite pick sequence for an already resolved object."
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {"object_id": {"type": "string"}},
        "required": ["object_id"],
    }

    def invoke(self, args: dict, context) -> ToolResult:
        object_id = str(args["object_id"])
        semantic_ir = {
            "intent": "sequence",
            "metadata": {
                "source": "composite_pick",
                "tool_changed_world": True,
                "object_id": object_id,
            },
            "steps": [
                {
                    "intent": "io_set",
                    "io_address": 0,
                    "io_value": 1,
                    "metadata": {"requires_gripper_config": True},
                },
            ],
        }
        contract = validate_semantic_ir_contract(semantic_ir)
        if not contract.valid:
            return ToolResult(ok=False, error=contract.reason)
        return ToolResult(ok=True, payload={"semantic_ir": semantic_ir})


@dataclass(frozen=True)
class GripperConfig:
    write_single_io_service: str
    read_single_io_service: str
    open_output_address: int | str
    open_output_value: int | str
    close_output_address: int | str
    close_output_value: int | str
    closed_input_address: int | str
    closed_input_active_value: int | str
    feedback_timeout_sec: float

    @classmethod
    def from_rules(cls, rules: dict[str, Any]) -> "GripperConfig":
        raw = rules.get("gripper", {}) if isinstance(rules, dict) else {}
        return cls(
            write_single_io_service=str(raw.get("write_single_io_service", "/io_set")),
            read_single_io_service=str(raw.get("read_single_io_service", "/read_single_io")),
            open_output_address=raw.get("open_output_address", "VERIFY_CONFIG"),
            open_output_value=raw.get("open_output_value", "VERIFY_CONFIG"),
            close_output_address=raw.get("close_output_address", "VERIFY_CONFIG"),
            close_output_value=raw.get("close_output_value", "VERIFY_CONFIG"),
            closed_input_address=raw.get("closed_input_address", "VERIFY_CONFIG"),
            closed_input_active_value=raw.get("closed_input_active_value", "VERIFY_CONFIG"),
            feedback_timeout_sec=float(raw.get("feedback_timeout_sec", 1.0)),
        )

    def verified(self) -> bool:
        values = (
            self.open_output_address,
            self.open_output_value,
            self.close_output_address,
            self.close_output_value,
            self.closed_input_address,
            self.closed_input_active_value,
        )
        return all(value != "VERIFY_CONFIG" for value in values)


@dataclass(frozen=True)
class GripperResult:
    ok: bool
    error: str = ""


class GripperIoAdapter:
    def __init__(self, *, config: GripperConfig, node: Any, robot_mode_fn):
        self._config = config
        self._node = node
        self._robot_mode_fn = robot_mode_fn

    def open(self) -> GripperResult:
        return self._write_guarded(
            self._config.open_output_address, self._config.open_output_value
        )

    def close(self) -> GripperResult:
        return self._write_guarded(
            self._config.close_output_address, self._config.close_output_value
        )

    def _write_guarded(self, address: int | str, value: int | str) -> GripperResult:
        if not self._config.verified():
            return GripperResult(ok=False, error="verify_config_required")
        if self._robot_mode_fn() != "IDLE":
            return GripperResult(ok=False, error="robot_not_idle")
        return GripperResult(ok=False, error="runtime_unavailable")
