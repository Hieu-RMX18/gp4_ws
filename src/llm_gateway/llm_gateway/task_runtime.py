from __future__ import annotations

import datetime
import logging
from dataclasses import dataclass
from typing import Any, Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from llm_gateway.factory_task import (
        FactoryTask,
        TaskNode,
        WorldModel,
    )

_LOGGER = logging.getLogger(__name__)


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
        is_stopped_fn: Callable[[], bool] | None = None,
        event_callback: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        if world_model is None:
            from llm_gateway.factory_task import WorldModel
            self._world_model = WorldModel()
        else:
            self._world_model = world_model
        self._replan_handler = replan_handler
        self._max_replans = max(0, int(max_replans))
        self._is_stopped_fn = is_stopped_fn
        self._event_callback = event_callback

    def _publish_event(
        self,
        category: str,
        event: str,
        detail: str,
        data: dict[str, Any] | None = None,
        level: str = "INFO",
    ) -> None:
        if self._event_callback is not None:
            ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self._event_callback({
                "ts": ts,
                "level": level,
                "source": "runtime",
                "category": category,
                "event": event,
                "detail": detail,
                "data": data or {},
            })

    def run(
        self,
        task: FactoryTask,
        skill_executor: Callable[[str, dict[str, Any]], RuntimeStepResult | bool],
    ) -> TaskRuntimeReport:
        self._publish_event("TASK", "task_start", f"Starting task {task.task_id}", {"task_id": task.task_id})
        
        state = _RuntimeState()
        result = self._run_node(task.root, skill_executor, state, bindings={})
        
        if result.requests_replan and self._replan_handler is not None:
            max_replans = self._max_replans_for_task(task)
            while state.replan_count < max_replans:
                state.replan_count += 1
                state.policy_decisions.append(
                    self._runtime_decision("root", "replan", result.reason or "skill requested replan")
                )
                self._publish_event("TASK", "task_replan", f"Replanning triggered for task {task.task_id} (attempt {state.replan_count})", {"task_id": task.task_id})
                
                next_task = self._replan_handler(
                    self._report(False, result.reason or "replan requested", state)
                )
                if next_task is None:
                    break
                result = self._run_node(next_task.root, skill_executor, state, bindings={})
                if not result.requests_replan:
                    break
                    
        report = self._report(result.success, result.reason, state)
        
        if report.success:
            self._publish_event("TASK", "task_done", f"Task {task.task_id} completed successfully", {"task_id": task.task_id})
        else:
            self._publish_event("TASK", "task_failed", f"Task {task.task_id} failed: {report.reason}", {"task_id": task.task_id, "reason": report.reason}, level="ERR")
            
        return report

    def _max_replans_for_task(self, task: FactoryTask) -> int:
        raw_value = task.replan_policy.get("max_replans")
        if raw_value is None:
            return self._max_replans
        try:
            return max(0, int(raw_value))
        except (TypeError, ValueError):
            from llm_gateway.factory_task import FactoryTaskError
            raise FactoryTaskError("replan_policy.max_replans must be an integer") from None

    def _run_node(
        self,
        node: TaskNode,
        skill_executor: Callable[[str, dict[str, Any]], RuntimeStepResult | bool],
        state: _RuntimeState,
        *,
        bindings: dict[str, Any],
    ) -> RuntimeStepResult:
        if self._is_stopped_fn is not None and self._is_stopped_fn():
            return RuntimeStepResult(success=False, reason="operator_stopped")

        if node.type in {"skill", "observe"}:
            return self._run_skill(node, skill_executor, state, bindings=bindings)
        if node.type == "sequence":
            return self._run_children(node.children, skill_executor, state, bindings=bindings)
        if node.type == "repeat":
            for i in range(int(node.count or 0)):
                self._publish_event("TASK", "loop_start", f"Repeat iteration {i+1}/{node.count}", {"iteration": i+1, "count": node.count})
                result = self._run_children(node.children, skill_executor, state, bindings=bindings)
                if not result.success:
                    return result
            return RuntimeStepResult(success=True)
        if node.type == "retry":
            attempts = int(node.count or 0)
            last_result = RuntimeStepResult(success=False, reason="retry had no attempts")
            for i in range(attempts):
                self._publish_event("TASK", "retry_attempt", f"Retry attempt {i+1}/{attempts}", {"attempt": i+1, "count": attempts})
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
            items = self._world_model.collection(node.collection)
            for i, item in enumerate(items):
                self._publish_event("TASK", "loop_start", f"For_each item {item} ({i+1}/{len(items)})", {"item": item, "index": i, "collection": node.collection})
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
        resolved_args = self._resolve_bindings(node.args, bindings)
        state.attempts_by_skill[name] = state.attempts_by_skill.get(name, 0) + 1
        
        self._publish_event("TASK", "step_start", f"Starting skill {name}", {"name": name, "args": resolved_args})
        
        result = skill_executor(name, resolved_args)
        
        if isinstance(result, RuntimeStepResult):
            step_res = result
        else:
            step_res = RuntimeStepResult(success=bool(result))
            
        if step_res.success:
            self._publish_event("TASK", "step_done", f"Skill {name} completed", {"name": name})
        else:
            self._publish_event("TASK", "step_failed", f"Skill {name} failed: {step_res.reason}", {"name": name, "reason": step_res.reason}, level="ERR")
            
        return step_res

    def _resolve_bindings(self, value: Any, bindings: dict[str, Any]) -> Any:
        if isinstance(value, str) and value.startswith("$"):
            key = value[1:]
            if key not in bindings:
                from llm_gateway.factory_task import FactoryTaskError
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
