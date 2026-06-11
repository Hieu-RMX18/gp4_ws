from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import yaml


FACTORY_TASK_VERSION = "1.0"

_SUPPORTED_NODE_TYPES = frozenset(
    {
        "sequence",
        "skill",
        "repeat",
        "for_each",
        "until",
        "if",
        "retry",
        "fallback",
        "observe",
        "wait_until",
    }
)

_CHILD_NODE_TYPES = frozenset(
    {"sequence", "repeat", "for_each", "until", "if", "retry", "fallback"}
)
_RUNTIME_CONTROL_NODE_TYPES = frozenset(
    {"repeat", "for_each", "until", "if", "retry", "fallback", "wait_until"}
)


class FactoryTaskError(ValueError):
    """Raised when a FactoryTask payload cannot be executed safely."""


VERIFY_CONFIG = "VERIFY_CONFIG"


@dataclass(frozen=True)
class ResolveResult:
    ok: bool
    name: str = ""
    payload: dict[str, Any] | None = None
    error: str = ""
    candidates: tuple[str, ...] = ()


def load_station_semantic_map(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError("station semantic map root must be a mapping")
    return data


def map_contains_verify_config(value: Any) -> bool:
    if value == VERIFY_CONFIG:
        return True
    if isinstance(value, dict):
        return any(map_contains_verify_config(child) for child in value.values())
    if isinstance(value, list):
        return any(map_contains_verify_config(child) for child in value)
    return False


class StationSceneGraph:
    """Static station semantic map plus dynamic perception object resolver."""

    def __init__(self, data: dict[str, Any]):
        self._data = data
        self._regions = data.get("regions") if isinstance(data.get("regions"), dict) else {}
        self._objects = data.get("objects") if isinstance(data.get("objects"), dict) else {}

    def to_dict(self) -> dict[str, Any]:
        return self._data

    @classmethod
    def from_file(cls, path: str | Path) -> "StationSceneGraph":
        return cls(load_station_semantic_map(path))

    def resolve_region(self, query: str) -> ResolveResult:
        return self._resolve_named(query, self._regions)

    def resolve_zone(self, region_name: str, query: str) -> ResolveResult:
        region = self._regions.get(region_name)
        zones = region.get("zones") if isinstance(region, dict) else None
        if not isinstance(zones, dict):
            return ResolveResult(ok=False, error="needs_clarification")
        return self._resolve_named(query, zones)

    def resolve_object(
        self, query: str, live_scene: dict[str, Any] | None = None
    ) -> ResolveResult:
        static_result = self._resolve_named(query, self._objects)
        if static_result.ok:
            return static_result
        if isinstance(live_scene, dict):
            detections = live_scene.get("detections", [])
            if isinstance(detections, list):
                normalized_query = _normalize_scene_name(query)
                for detection in detections:
                    if not isinstance(detection, dict):
                        continue
                    class_id = str(detection.get("class_id", ""))
                    if not class_id:
                        continue
                    if _normalize_scene_name(class_id) == normalized_query:
                        return ResolveResult(
                            ok=True,
                            name=class_id,
                            payload={
                                "dynamic": True,
                                "class_id": class_id,
                                "detection": detection,
                            },
                        )
        return static_result

    def runtime_geometry_ready(self, region_name: str) -> bool:
        region = self._regions.get(region_name)
        return isinstance(region, dict) and not map_contains_verify_config(region.get("geometry"))

    def runtime_block_reason(self, region_name: str) -> str:
        return "" if self.runtime_geometry_ready(region_name) else "verify_config_required"

    def nearest_free_cell(
        self, region_name: str, object_size: dict[str, float] | None = None
    ) -> ResolveResult:
        _ = object_size
        region = self._regions.get(region_name)
        if not isinstance(region, dict):
            return ResolveResult(ok=False, error="needs_clarification")
        if not self.runtime_geometry_ready(region_name):
            return ResolveResult(ok=False, name=region_name, error="verify_config_required")
        return ResolveResult(ok=False, name=region_name, error="capability_unavailable")

    def _resolve_named(self, query: str, collection: dict[str, Any]) -> ResolveResult:
        normalized = _normalize_scene_name(query)
        matches: list[str] = []
        for name, payload in collection.items():
            aliases = payload.get("aliases", []) if isinstance(payload, dict) else []
            names = [name, *[str(alias) for alias in aliases]]
            if normalized in {_normalize_scene_name(candidate) for candidate in names}:
                matches.append(name)
        if len(matches) == 1:
            name = matches[0]
            return ResolveResult(ok=True, name=name, payload=collection[name])
        if len(matches) > 1:
            return ResolveResult(
                ok=False, error="needs_clarification", candidates=tuple(sorted(matches))
            )
        return ResolveResult(ok=False, error="needs_clarification")


def _normalize_scene_name(value: str) -> str:
    return " ".join(str(value).strip().lower().replace("_", " ").split())


@dataclass(frozen=True)
class SkillCall:
    name: str
    args: dict[str, Any] = field(default_factory=dict)


def compile_goal(
    goal_dsl: dict[str, Any],
    *,
    scene_graph: Any,
    live_scene: dict[str, Any] | None = None,
) -> list[SkillCall]:
    if not isinstance(goal_dsl, dict):
        return [SkillCall("needs_clarification", {"field": "goal", "query": "non_object"})]
    action = str(goal_dsl.get("action") or "").strip()
    if action != "pick_and_place":
        return [SkillCall("capability_unavailable", {"action": action})]

    object_query = str(goal_dsl.get("object") or "").strip()
    destination_query = str(goal_dsl.get("destination") or "").strip()
    object_result = scene_graph.resolve_object(object_query, live_scene=live_scene)
    if not object_result.ok:
        return [
            SkillCall("needs_clarification", {"field": "object", "query": object_query})
        ]
    destination_result = scene_graph.resolve_region(destination_query)
    if not destination_result.ok:
        return [
            SkillCall(
                "needs_clarification",
                {"field": "destination", "query": destination_query},
            )
        ]

    return [
        SkillCall("refresh_scene"),
        SkillCall("approach_object", {"object_id": object_result.name}),
        SkillCall("pick_object", {"object_id": object_result.name}),
        SkillCall(
            "place_object",
            {"object_id": object_result.name, "destination": destination_result.name},
        ),
        SkillCall(
            "verify_postcondition",
            {"object_id": object_result.name, "destination": destination_result.name},
        ),
    ]


@dataclass(frozen=True)
class TaskNode:
    type: str
    name: str = ""
    args: dict[str, Any] = field(default_factory=dict)
    children: tuple["TaskNode", ...] = ()
    condition: dict[str, Any] | str | None = None
    count: int | None = None
    collection: str = ""
    item_name: str = "item"
    until: dict[str, Any] | str | None = None
    on_failure: dict[str, Any] | str | None = None


@dataclass(frozen=True)
class FactoryTask:
    task_id: str
    root: TaskNode
    mode: str = "supervised_hardware"
    version: str = FACTORY_TASK_VERSION
    operator_summary: str = ""
    limits: dict[str, Any] = field(default_factory=dict)
    replan_policy: dict[str, Any] = field(default_factory=dict)

@dataclass(frozen=True)
class PolicyDecision:
    node_path: str
    node_type: str
    node_name: str
    decision: str
    reason: str
    risk_level: str = "medium"

    def to_dict(self) -> dict[str, Any]:
        return {
            "node_path": self.node_path,
            "node_type": self.node_type,
            "node_name": self.node_name,
            "decision": self.decision,
            "reason": self.reason,
            "risk_level": self.risk_level,
        }

@dataclass(frozen=True)
class CompiledTask:
    task: FactoryTask
    semantic_ir: dict[str, Any]
    runtime_plan: dict[str, Any]
    policy_decisions: tuple[PolicyDecision, ...]

@dataclass(frozen=True)
class RuntimeStepResult:
    success: bool
    reason: str = ""
    requests_replan: bool = False
    observation: dict[str, Any] | None = None

@dataclass(frozen=True)
class TaskRuntimeReport:
    success: bool
    reason: str
    attempts_by_skill: dict[str, int]
    fallback_count: int
    replan_count: int
    policy_decisions: list[dict[str, Any]]

class WorldModel:
    """Grounding facts available to the task compiler/runtime."""

    def __init__(
        self,
        *,
        objects: dict[str, dict[str, Any]] | None = None,
        regions: dict[str, dict[str, Any]] | None = None,
        collections: dict[str, list[Any]] | None = None,
    ) -> None:
        self._objects = {str(key): dict(value) for key, value in (objects or {}).items()}
        self._regions = {str(key): dict(value) for key, value in (regions or {}).items()}
        self._collections = {
            str(key): list(value) for key, value in (collections or {}).items()
        }

    def object_pose(self, object_ref: Any) -> dict[str, Any]:
        key = self._object_key(object_ref)
        candidate = self._objects.get(key)
        if isinstance(object_ref, dict):
            candidate = object_ref
        if not isinstance(candidate, dict):
            raise FactoryTaskError(f"world model has no grounded pose for object '{key}'")
        pose = candidate.get("pose") or candidate.get("target_pose")
        if not isinstance(pose, dict):
            raise FactoryTaskError(f"world model has no grounded pose for object '{key}'")
        return dict(pose)

    def collection(self, name: str) -> list[Any]:
        key = str(name or "").strip()
        if key in self._collections:
            return list(self._collections[key])
        if key == "visible_objects":
            return [dict(value) for value in self._objects.values()]
        raise FactoryTaskError(f"world model collection is unavailable: {key}")

    @staticmethod
    def _object_key(object_ref: Any) -> str:
        if isinstance(object_ref, dict):
            for field_name in ("id", "name", "label"):
                value = str(object_ref.get(field_name) or "").strip()
                if value:
                    return value
            return "<object>"
        return str(object_ref or "").strip()

class PolicyEngine:
    """Visible policy decisions for FactoryTask planning.

    This is intentionally conservative: it records why a node may proceed, but
    final motion still goes through Semantic IR routing, validation, supervisor
    confirmation, and execution guards.
    """

    def evaluate_node(self, node: TaskNode, *, path: str) -> PolicyDecision:
        if node.type == "skill":
            reason = "skill compiles to guarded Semantic IR and remains behind supervisor validation"
            risk_level = "medium" if node.name not in {"wait", "observe_station"} else "low"
        elif node.type in {"repeat", "retry", "fallback", "for_each", "until", "if"}:
            reason = "runtime control is preserved and not statically expanded"
            risk_level = "medium"
        else:
            reason = "task node allowed for supervised runtime planning before supervisor validation"
            risk_level = "low"
        return PolicyDecision(
            node_path=path,
            node_type=node.type,
            node_name=node.name,
            decision="allow",
            reason=reason,
            risk_level=risk_level,
        )

class TaskCompiler:
    """Compile FactoryTask into the existing guarded downstream command form."""

    def __init__(
        self,
        *,
        world_model: WorldModel | None = None,
        policy_engine: PolicyEngine | None = None,
    ) -> None:
        self._world_model = world_model or WorldModel()
        self._policy_engine = policy_engine or PolicyEngine()

    def compile(self, task: FactoryTask) -> CompiledTask:
        decisions: list[PolicyDecision] = []
        steps = self._compile_node(task.root, path="root", decisions=decisions)
        if len(steps) == 1:
            semantic_ir = dict(steps[0])
        else:
            semantic_ir = {"intent": "sequence", "steps": steps}
        runtime_plan = self._node_to_runtime_plan(task.root)
        semantic_ir["metadata"] = {
            "factory_task": self._task_metadata(task),
            "runtime_plan": runtime_plan,
            "policy_decisions": [decision.to_dict() for decision in decisions],
        }
        return CompiledTask(
            task=task,
            semantic_ir=semantic_ir,
            runtime_plan=runtime_plan,
            policy_decisions=tuple(decisions),
        )

    def _compile_node(
        self,
        node: TaskNode,
        *,
        path: str,
        decisions: list[PolicyDecision],
    ) -> list[dict[str, Any]]:
        decisions.append(self._policy_engine.evaluate_node(node, path=path))
        if node.type in _RUNTIME_CONTROL_NODE_TYPES:
            raise FactoryTaskError(
                f"{node.type} at {path} requires TaskRuntime; static Semantic IR review cannot preserve runtime control flow"
            )
        if node.type == "skill":
            return [self._compile_skill(node, path=path)]
        if node.type == "observe":
            return []
        steps: list[dict[str, Any]] = []
        for index, child in enumerate(node.children):
            steps.extend(
                self._compile_node(
                    child,
                    path=f"{path}.children[{index}]",
                    decisions=decisions,
                )
            )
        return steps

    def _compile_skill(self, node: TaskNode, *, path: str) -> dict[str, Any]:
        args = dict(node.args)
        if node.name == "go_home":
            return {"intent": "go_home"}
        if node.name in {"stop", "alarm_reset"}:
            return {"intent": node.name}
        if node.name == "get_pose":
            return _payload_from_args(
                "get_pose",
                args,
                optional=("reference_frame",),
            )
        if node.name == "set_speed":
            return _payload_from_args(
                "set_speed",
                args,
                required=("velocity_scale",),
                optional=("acceleration_scale",),
            )
        if node.name == "wait":
            return {
                "intent": "wait",
                "wait_duration_sec": float(args.get("wait_duration_sec", 2.0)),
            }
        if node.name == "move_relative":
            return _payload_from_args(
                "move_relative",
                args,
                required=("delta",),
                optional=(
                    "reference_frame",
                    "linear_unit",
                    "velocity_scale",
                    "acceleration_scale",
                ),
            )
        if node.name == "move_cartesian":
            intent = (
                "absolute_move_lin"
                if args.get("motion_type") == "lin"
                else "absolute_move_ptp"
            )
            return _payload_from_args(
                intent,
                args,
                required=("target_pose",),
                optional=(
                    "reference_frame",
                    "linear_unit",
                    "orientation_preset",
                    "keep_current_orientation",
                    "velocity_scale",
                    "acceleration_scale",
                    "planner_id",
                ),
            )
        if node.name == "move_joint":
            return _payload_from_args(
                "move_joint",
                args,
                required=("joint_index", "joint_angle"),
                optional=("angular_unit", "velocity_scale", "acceleration_scale"),
            )
        if node.name == "move_joint_delta":
            return _payload_from_args(
                "move_joint_delta",
                args,
                required=("joint_index", "delta_angle"),
                optional=("angular_unit", "velocity_scale", "acceleration_scale"),
            )
        if node.name == "move_joints":
            return _payload_from_args(
                "move_joints",
                args,
                required=("joint_target",),
                optional=("angular_unit", "velocity_scale", "acceleration_scale"),
            )
        if node.name == "draw_shape":
            return _payload_from_args(
                "draw_shape",
                args,
                required=("shape_type", "params"),
                optional=("units", "frame_id", "workplane"),
            )
        if node.name == "draw_text":
            return _payload_from_args(
                "draw_text",
                args,
                required=("text", "font"),
                optional=("units", "frame_id", "workplane"),
            )
        if node.name in {"move_named_pose", "move_to_region"}:
            pose_name = str(
                args.get("pose_name") or args.get("pose") or args.get("region") or ""
            ).strip()
            if not pose_name:
                raise FactoryTaskError(f"{node.name} at {path} requires pose_name or region")
            return {"intent": "move_named_pose", "pose_name": pose_name}
        if node.name == "move_to_object":
            object_ref = args.get("object_ref") or args.get("object")
            if object_ref is None:
                raise FactoryTaskError(f"move_to_object at {path} requires object_ref")
            return {
                "intent": "absolute_move_ptp",
                "target_pose": self._world_model.object_pose(object_ref),
                "reference_frame": str(args.get("reference_frame") or "base_link"),
            }
        if node.name in {"pick_object", "place_object", "pick_and_place"}:
            # Complex manipulator skills compile to a gripper I/O step as the static review
            # representative. The actual approach/descent/lift sequence is owned by TaskRuntime.
            # The review shows the world-model-resolved object pose and planned gripper state.
            object_ref = (
                args.get("object_ref")
                or args.get("object")
                or args.get("object_id")
            )
            if node.name in {"pick_object", "pick_and_place"} and object_ref is None:
                raise FactoryTaskError(f"{node.name} at {path} requires object_ref")
            io_intent = "io_set"
            if node.name == "place_object":
                # Place: open gripper to release
                io_step: dict[str, Any] = {
                    "intent": io_intent,
                    "io_address": 0,
                    "io_value": 0,
                    "metadata": {
                        "purpose": f"{node.name}_gripper_open",
                        "requires_gripper_config": True,
                        "destination": str(args.get("destination") or ""),
                    },
                }
            else:
                # Pick: close gripper to grasp
                io_step = {
                    "intent": io_intent,
                    "io_address": 0,
                    "io_value": 1,
                    "metadata": {
                        "purpose": f"{node.name}_gripper_close",
                        "requires_gripper_config": True,
                        "object_ref": str(object_ref or ""),
                    },
                }
            steps_for_review: list[dict[str, Any]] = []
            if object_ref is not None:
                try:
                    approach_pose = self._world_model.object_pose(object_ref)
                    steps_for_review.append({
                        "intent": "absolute_move_ptp",
                        "target_pose": approach_pose,
                        "reference_frame": str(args.get("reference_frame") or "base_link"),
                        "metadata": {"purpose": f"{node.name}_approach"},
                    })
                except FactoryTaskError:
                    if node.name in {"pick_object", "pick_and_place"}:
                        raise
                    # Object pose not yet grounded — static review can't plan;
                    # TaskRuntime will resolve place destinations at execution time.
                    pass
            steps_for_review.append(io_step)
            if len(steps_for_review) == 1:
                return steps_for_review[0]
            return {
                "intent": "sequence",
                "steps": steps_for_review,
                "metadata": {"source": f"factory_task.{node.name}", "tool_changed_world": True},
            }
        if node.name == "verify_scene":
            # Postcondition check — no motion Semantic IR, TaskRuntime handles verification.
            return {
                "intent": "get_pose",
                "metadata": {
                    "purpose": "verify_scene",
                    "object": str(args.get("object") or args.get("object_ref") or ""),
                    "expected": str(args.get("expected") or "visible"),
                },
            }
        if node.name == "place_relative":
            object_rel = args.get("object") or args.get("object_ref") or ""
            delta = args.get("delta") or {}
            return _payload_from_args(
                "move_relative",
                {"delta": delta, "reference_frame": args.get("reference_frame", "base_link")},
                required=("delta",),
                optional=("reference_frame", "linear_unit", "velocity_scale", "acceleration_scale"),
            )
        if node.name in {"observe_station", "refresh_scene"}:
            # Pure observation / cache invalidation — no motion Semantic IR step.
            return {
                "intent": "get_pose",
                "metadata": {
                    "purpose": node.name,
                    "region": str(args.get("region") or ""),
                },
            }
        raise FactoryTaskError(
            f"unsupported FactoryTask skill at {path}: {node.name}; provide a runtime executor or compiler mapping"
        )

    @staticmethod
    def _task_metadata(task: FactoryTask) -> dict[str, Any]:
        return {
            "task_id": task.task_id,
            "version": task.version,
            "mode": task.mode,
            "operator_summary": task.operator_summary,
            "limits": dict(task.limits),
            "replan_policy": dict(task.replan_policy),
        }

    def _node_to_runtime_plan(self, node: TaskNode) -> dict[str, Any]:
        payload: dict[str, Any] = {"type": node.type}
        if node.name:
            payload["name"] = node.name
        if node.args:
            payload["args"] = dict(node.args)
        if node.count is not None:
            payload["count"] = node.count
        if node.collection:
            payload["collection"] = node.collection
            payload["item_name"] = node.item_name
        if node.condition is not None:
            payload["condition"] = node.condition
        if node.until is not None:
            payload["until"] = node.until
        if node.on_failure is not None:
            payload["on_failure"] = node.on_failure
        if node.children:
            payload["children"] = [
                self._node_to_runtime_plan(child) for child in node.children
            ]
        return payload

class _RuntimeState:
    def __init__(self) -> None:
        self.attempts_by_skill: dict[str, int] = {}
        self.fallback_count = 0
        self.replan_count = 0
        self.policy_decisions: list[dict[str, Any]] = []

class TaskRuntime:
    """Execute FactoryTask control flow through an injected safe skill executor."""

    def __init__(
        self,
        *,
        world_model: WorldModel | None = None,
        replan_handler: Callable[[TaskRuntimeReport], FactoryTask | None] | None = None,
        max_replans: int = 1,
    ) -> None:
        self._world_model = world_model or WorldModel()
        self._replan_handler = replan_handler
        self._max_replans = max(0, int(max_replans))

    def run(
        self,
        task: FactoryTask,
        skill_executor: Callable[[str, dict[str, Any]], RuntimeStepResult | bool],
    ) -> TaskRuntimeReport:
        state = _RuntimeState()
        result = self._run_node(task.root, skill_executor, state, bindings={})
        if result.requests_replan and self._replan_handler is not None:
            max_replans = self._max_replans_for_task(task)
            while state.replan_count < max_replans:
                state.replan_count += 1
                state.policy_decisions.append(
                    self._runtime_decision("root", "replan", result.reason or "skill requested replan")
                )
                next_task = self._replan_handler(
                    self._report(False, result.reason or "replan requested", state)
                )
                if next_task is None:
                    break
                result = self._run_node(next_task.root, skill_executor, state, bindings={})
                if not result.requests_replan:
                    break
        return self._report(result.success, result.reason, state)

    def _max_replans_for_task(self, task: FactoryTask) -> int:
        raw_value = task.replan_policy.get("max_replans")
        if raw_value is None:
            return self._max_replans
        try:
            return max(0, int(raw_value))
        except (TypeError, ValueError):
            raise FactoryTaskError("replan_policy.max_replans must be an integer") from None

    def _run_node(
        self,
        node: TaskNode,
        skill_executor: Callable[[str, dict[str, Any]], RuntimeStepResult | bool],
        state: _RuntimeState,
        *,
        bindings: dict[str, Any],
    ) -> RuntimeStepResult:
        if node.type in {"skill", "observe"}:
            return self._run_skill(node, skill_executor, state, bindings=bindings)
        if node.type == "sequence":
            return self._run_children(node.children, skill_executor, state, bindings=bindings)
        if node.type == "repeat":
            for _ in range(int(node.count or 0)):
                result = self._run_children(node.children, skill_executor, state, bindings=bindings)
                if not result.success:
                    return result
            return RuntimeStepResult(success=True)
        if node.type == "retry":
            attempts = int(node.count or 0)
            last_result = RuntimeStepResult(success=False, reason="retry had no attempts")
            for _ in range(attempts):
                last_result = self._run_children(node.children, skill_executor, state, bindings=bindings)
                if last_result.success or last_result.requests_replan:
                    return last_result
            state.policy_decisions.append(
                self._runtime_decision("retry", "retry_exhausted", last_result.reason or "all retry attempts failed")
            )
            return last_result
        if node.type == "fallback":
            for index, child in enumerate(node.children):
                result = self._run_node(child, skill_executor, state, bindings=bindings)
                if result.success:
                    if index > 0:
                        state.fallback_count += 1
                        state.policy_decisions.append(
                            self._runtime_decision("fallback", "fallback_selected", child.name or child.type)
                        )
                    return result
                if result.requests_replan:
                    return result
            return RuntimeStepResult(success=False, reason="all fallback branches failed")
        if node.type == "for_each":
            for item in self._world_model.collection(node.collection):
                scoped_bindings = dict(bindings)
                scoped_bindings[node.item_name] = item
                result = self._run_children(
                    node.children, skill_executor, state, bindings=scoped_bindings
                )
                if not result.success:
                    return result
            return RuntimeStepResult(success=True)
        if node.type in {"if", "until", "wait_until"}:
            return RuntimeStepResult(
                success=False,
                reason=f"{node.type} requires an explicit condition evaluator",
            )
        return RuntimeStepResult(success=False, reason=f"unsupported runtime node: {node.type}")

    def _run_children(
        self,
        children: tuple[TaskNode, ...],
        skill_executor: Callable[[str, dict[str, Any]], RuntimeStepResult | bool],
        state: _RuntimeState,
        *,
        bindings: dict[str, Any],
    ) -> RuntimeStepResult:
        for child in children:
            result = self._run_node(child, skill_executor, state, bindings=bindings)
            if not result.success:
                return result
        return RuntimeStepResult(success=True)

    def _run_skill(
        self,
        node: TaskNode,
        skill_executor: Callable[[str, dict[str, Any]], RuntimeStepResult | bool],
        state: _RuntimeState,
        *,
        bindings: dict[str, Any],
    ) -> RuntimeStepResult:
        name = node.name
        state.attempts_by_skill[name] = state.attempts_by_skill.get(name, 0) + 1
        result = skill_executor(name, self._resolve_bindings(node.args, bindings))
        if isinstance(result, RuntimeStepResult):
            return result
        return RuntimeStepResult(success=bool(result))

    def _resolve_bindings(self, value: Any, bindings: dict[str, Any]) -> Any:
        if isinstance(value, str) and value.startswith("$"):
            key = value[1:]
            if key not in bindings:
                raise FactoryTaskError(f"runtime binding is unavailable: {value}")
            return bindings[key]
        if isinstance(value, dict):
            return {key: self._resolve_bindings(item, bindings) for key, item in value.items()}
        if isinstance(value, list):
            return [self._resolve_bindings(item, bindings) for item in value]
        return value

    @staticmethod
    def _runtime_decision(node_path: str, decision: str, reason: str) -> dict[str, Any]:
        return {
            "node_path": node_path,
            "decision": decision,
            "reason": reason,
            "risk_level": "medium",
        }

    @staticmethod
    def _report(success: bool, reason: str, state: _RuntimeState) -> TaskRuntimeReport:
        return TaskRuntimeReport(
            success=success,
            reason=reason,
            attempts_by_skill=dict(state.attempts_by_skill),
            fallback_count=state.fallback_count,
            replan_count=state.replan_count,
            policy_decisions=list(state.policy_decisions),
        )


def parse_factory_task(payload: dict[str, Any]) -> FactoryTask:
    """Parse and validate the LLM-facing task tree contract."""
    if not isinstance(payload, dict):
        raise FactoryTaskError("FactoryTask payload must be an object")
    if payload.get("task_type") != "factory_task":
        raise FactoryTaskError("FactoryTask payload requires task_type='factory_task'")

    version = str(payload.get("version") or FACTORY_TASK_VERSION)
    if version != FACTORY_TASK_VERSION:
        raise FactoryTaskError(f"unsupported FactoryTask version: {version}")

    task_id = str(payload.get("task_id") or "").strip()
    if not task_id:
        raise FactoryTaskError("FactoryTask requires non-empty task_id")

    root = _parse_node(payload.get("root"), path="root")
    return FactoryTask(
        task_id=task_id,
        root=root,
        mode=str(payload.get("mode") or "supervised_hardware"),
        version=version,
        operator_summary=str(payload.get("operator_summary") or ""),
        limits=_dict_or_empty(payload.get("limits"), field_name="limits"),
        replan_policy=_dict_or_empty(
            payload.get("replan_policy"), field_name="replan_policy"
        ),
    )


def is_factory_task(payload: Any) -> bool:
    return isinstance(payload, dict) and payload.get("task_type") == "factory_task"


def count_task_nodes(node: TaskNode) -> int:
    return 1 + sum(count_task_nodes(child) for child in node.children)


def _parse_node(payload: Any, *, path: str) -> TaskNode:
    if not isinstance(payload, dict):
        raise FactoryTaskError(f"{path} must be an object")

    node_type = str(payload.get("type") or "").strip()
    if node_type not in _SUPPORTED_NODE_TYPES:
        raise FactoryTaskError(f"unsupported node type at {path}: {node_type}")

    count = _optional_positive_int(payload.get("count"), path=path, node_type=node_type)
    children = tuple(
        _parse_node(child, path=f"{path}.children[{index}]")
        for index, child in enumerate(_children_payload(payload, node_type, path))
    )

    name = str(payload.get("name") or "").strip()
    if node_type in {"skill", "observe"} and not name:
        raise FactoryTaskError(f"{node_type} node at {path} requires name")

    collection = str(payload.get("collection") or "").strip()
    if node_type == "for_each" and not collection:
        raise FactoryTaskError(f"for_each at {path} requires collection")

    return TaskNode(
        type=node_type,
        name=name,
        args=_dict_or_empty(payload.get("args"), field_name=f"{path}.args"),
        children=children,
        condition=payload.get("condition"),
        count=count,
        collection=collection,
        item_name=str(payload.get("item_name") or "item"),
        until=payload.get("until"),
        on_failure=payload.get("on_failure"),
    )


def _children_payload(payload: dict[str, Any], node_type: str, path: str) -> list[Any]:
    raw_children = payload.get("children", [])
    if not isinstance(raw_children, list):
        raise FactoryTaskError(f"{path}.children must be a list")
    if node_type in _CHILD_NODE_TYPES and not raw_children:
        raise FactoryTaskError(f"{node_type} at {path} requires at least one child")
    if node_type not in _CHILD_NODE_TYPES and raw_children:
        raise FactoryTaskError(f"{node_type} at {path} cannot have children")
    return raw_children


def _optional_positive_int(value: Any, *, path: str, node_type: str) -> int | None:
    if node_type not in {"repeat", "retry"}:
        return None
    if not isinstance(value, int) or value <= 0:
        raise FactoryTaskError(f"{node_type} requires positive integer count at {path}")
    return value


def _dict_or_empty(value: Any, *, field_name: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise FactoryTaskError(f"{field_name} must be an object")
    return dict(value)

def _payload_from_args(
    intent: str,
    args: dict[str, Any],
    *,
    required: tuple[str, ...] = (),
    optional: tuple[str, ...] = (),
) -> dict[str, Any]:
    payload: dict[str, Any] = {"intent": intent}
    for field_name in required:
        if field_name not in args:
            raise FactoryTaskError(f"{intent} requires {field_name}")
        payload[field_name] = args[field_name]
    for field_name in optional:
        if field_name in args:
            payload[field_name] = args[field_name]
    return payload
