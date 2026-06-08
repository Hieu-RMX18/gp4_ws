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
        
        # Call WriteSingleIO service when config is verified and robot is idle
        client = getattr(self._node, "_write_single_io_client", None)
        if client is None or not client.service_is_ready():
            return GripperResult(ok=False, error="runtime_unavailable")
        
        try:
            from motoros2_interfaces.srv import WriteSingleIO
            request = WriteSingleIO.Request()
            request.address = int(address)
            request.value = int(value)
            future = client.call_async(request)
            # Synchronous wait (node has _wait_for_future_without_spinning)
            wait_fn = getattr(self._node, "_wait_for_future_without_spinning", None)
            if not callable(wait_fn):
                return GripperResult(ok=False, error="runtime_unavailable")
            done, response = wait_fn(future, self._config.feedback_timeout_sec)
            if not done or response is None:
                return GripperResult(ok=False, error="io_timeout")
            if not response.success:
                return GripperResult(ok=False, error=f"io_failed: {response.message}")
            return GripperResult(ok=True)
        except Exception as exc:
            return GripperResult(ok=False, error=f"io_exception: {exc}")


@dataclass(frozen=True)
class VerificationResult:
    ok: bool
    error: str = ""


class PostconditionVerifier:
    def verify_place(
        self, *, object_id: str, destination: str, scene: dict[str, Any]
    ) -> VerificationResult:
        detections = scene.get("detections", []) if isinstance(scene, dict) else []
        for detection in detections:
            if detection.get("class_id") == object_id and detection.get("region") == destination:
                return VerificationResult(ok=True)
        return VerificationResult(ok=False, error="postcondition_failed")


class ApproachObjectTool(_CompositeTool):
    name = "approach_object"
    description = (
        "Emit a validated LIN approach sequence to the pre-grasp position above a resolved object. "
        "Requires the scene to have been refreshed beforehand."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {"object_id": {"type": "string"}},
        "required": ["object_id"],
    }

    def invoke(self, args: dict, context) -> ToolResult:
        object_id = str(args["object_id"])
        # Approach is a relative lift-clear move toward the object.
        # Actual target pose comes from perception at execution time via motion_core.
        semantic_ir = {
            "intent": "sequence",
            "metadata": {
                "source": "composite_approach",
                "tool_changed_world": False,
                "object_id": object_id,
            },
            "steps": [
                {
                    "intent": "move_relative",
                    "delta": {"x": 0.0, "y": 0.0, "z": -0.08},
                    "reference_frame": "base_link",
                    "metadata": {"purpose": "approach_object", "object_id": object_id},
                },
            ],
        }
        contract = validate_semantic_ir_contract(semantic_ir)
        if not contract.valid:
            return ToolResult(ok=False, error=contract.reason)
        return ToolResult(ok=True, payload={"semantic_ir": semantic_ir, "object_id": object_id})


class PlaceObjectTool(_CompositeTool):
    name = "place_object"
    description = (
        "Emit a validated composite place sequence: descend to destination, release, lift clear. "
        "Destination must already be resolved to a known region."
    )
    is_motion = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "object_id": {"type": "string"},
            "destination": {"type": "string"},
        },
        "required": ["object_id", "destination"],
    }

    def invoke(self, args: dict, context) -> ToolResult:
        object_id = str(args["object_id"])
        destination = str(args["destination"])
        semantic_ir = {
            "intent": "sequence",
            "metadata": {
                "source": "composite_place",
                "tool_changed_world": True,
                "object_id": object_id,
                "destination": destination,
            },
            "steps": [
                {
                    "intent": "move_relative",
                    "delta": {"x": 0.0, "y": 0.0, "z": -0.05},
                    "reference_frame": "base_link",
                    "metadata": {"purpose": "place_descend"},
                },
                {
                    "intent": "io_set",
                    "io_address": 0,
                    "io_value": 0,
                    "metadata": {"requires_gripper_config": True, "purpose": "release"},
                },
                {
                    "intent": "move_relative",
                    "delta": {"x": 0.0, "y": 0.0, "z": 0.08},
                    "reference_frame": "base_link",
                    "metadata": {"purpose": "place_lift_clear"},
                },
            ],
        }
        contract = validate_semantic_ir_contract(semantic_ir)
        if not contract.valid:
            return ToolResult(ok=False, error=contract.reason)
        return ToolResult(
            ok=True,
            payload={"semantic_ir": semantic_ir, "object_id": object_id, "destination": destination},
        )


class VerifyPostconditionTool(_CompositeTool):
    name = "verify_postcondition"
    description = (
        "Query perception to confirm the object is detected in the expected destination region. "
        "Returns ok=False with error='postcondition_failed' if the object is not confirmed there."
    )
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {
            "object_id": {"type": "string"},
            "destination": {"type": "string"},
        },
        "required": ["object_id", "destination"],
    }

    def invoke(self, args: dict, context) -> ToolResult:
        object_id = str(args["object_id"])
        destination = str(args["destination"])
        node = getattr(context, "ros_node", None)
        scene_fn = getattr(node, "_query_scene_for_verify", None)
        if callable(scene_fn):
            scene = scene_fn()
        else:
            # No live scene available; fail closed so the agent can retry or escalate.
            return ToolResult(ok=False, error="capability_unavailable")
        verifier = PostconditionVerifier()
        result = verifier.verify_place(
            object_id=object_id, destination=destination, scene=scene
        )
        return ToolResult(ok=result.ok, error=result.error if not result.ok else None)


def mtc_select(
    *,
    mtc_service_ready_fn=None,
    object_pose_known: bool = False,
    destination_known: bool = False,
    gripper_config_verified: bool = False,
) -> str:
    """Choose execution path: 'mtc' when all prerequisites are met, else 'primitive'.

    Returns 'capability_unavailable' when no path is viable.
    This function is pure and deterministic given its inputs.
    """
    all_prereqs = object_pose_known and destination_known and gripper_config_verified
    if all_prereqs and callable(mtc_service_ready_fn) and mtc_service_ready_fn():
        return "mtc"
    if all_prereqs:
        return "primitive"
    return "capability_unavailable"


class VerifyGraspTool(_CompositeTool):
    name = "verify_grasp"
    description = (
        "Query the gripper feedback input to confirm an object is grasped. "
        "Fails closed when gripper config is unverified or feedback times out."
    )
    is_readonly = True
    input_schema: ClassVar[dict] = {
        "type": "object",
        "properties": {"object_id": {"type": "string"}},
        "required": ["object_id"],
    }

    def invoke(self, args: dict, context) -> ToolResult:
        object_id = str(args["object_id"])
        node = getattr(context, "ros_node", None)
        adapter = getattr(node, "_gripper_adapter", None)
        if adapter is None:
            return ToolResult(ok=False, error="capability_unavailable")
        config = getattr(adapter, "_config", None)
        if config is None or not config.verified():
            return ToolResult(ok=False, error="verify_config_required")
        # At runtime the adapter would read the IO feedback pin.
        # Without a live node, fail closed so tests and simulation work correctly.
        read_fn = getattr(node, "_read_gripper_feedback", None)
        if not callable(read_fn):
            return ToolResult(ok=False, error="runtime_unavailable")
        grasped = read_fn()
        if grasped:
            return ToolResult(ok=True, payload={"object_id": object_id, "grasped": True})
        return ToolResult(ok=False, error="grasp_not_confirmed")
