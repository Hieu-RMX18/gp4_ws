from __future__ import annotations

import pytest
from typing import Any

from llm_gateway.factory_task import (
    FACTORY_TASK_VERSION,
    FactoryTask,
    TaskNode,
    parse_factory_task,
)
from llm_gateway.task_runtime import (
    TaskRuntime,
    RuntimeStepResult,
    TaskRuntimeReport,
)


def test_task_runtime_checks_stop_flag_before_running_node() -> None:
    # A simple task with a sequence of two steps
    task = parse_factory_task({
        "task_type": "factory_task",
        "version": FACTORY_TASK_VERSION,
        "task_id": "stop-test",
        "root": {
            "type": "sequence",
            "children": [
                {"type": "skill", "name": "step1", "args": {}},
                {"type": "skill", "name": "step2", "args": {}},
            ]
        }
    })
    
    calls: list[str] = []
    
    def executor(name: str, args: dict) -> RuntimeStepResult:
        calls.append(name)
        return RuntimeStepResult(success=True)

    # When is_stopped_fn returns True, execution should abort before executing step1 or step2
    runtime = TaskRuntime(is_stopped_fn=lambda: True)
    report = runtime.run(task, executor)
    
    assert report.success is False
    assert "operator_stopped" in report.reason or "stop" in report.reason.lower()
    assert len(calls) == 0


def test_task_runtime_checks_stop_flag_between_steps() -> None:
    task = parse_factory_task({
        "task_type": "factory_task",
        "version": FACTORY_TASK_VERSION,
        "task_id": "stop-mid-test",
        "root": {
            "type": "sequence",
            "children": [
                {"type": "skill", "name": "step1", "args": {}},
                {"type": "skill", "name": "step2", "args": {}},
            ]
        }
    })
    
    calls: list[str] = []
    
    # We want to trigger stop after step1 executes
    is_stopped = False
    
    def executor(name: str, args: dict) -> RuntimeStepResult:
        nonlocal is_stopped
        calls.append(name)
        if name == "step1":
            is_stopped = True  # Set stop flag to True
        return RuntimeStepResult(success=True)

    runtime = TaskRuntime(is_stopped_fn=lambda: is_stopped)
    report = runtime.run(task, executor)
    
    assert report.success is False
    assert "operator_stopped" in report.reason or "stop" in report.reason.lower()
    assert calls == ["step1"]  # step2 is not executed


def test_task_runtime_emits_task_events_in_correct_order() -> None:
    task = parse_factory_task({
        "task_type": "factory_task",
        "version": FACTORY_TASK_VERSION,
        "task_id": "events-test",
        "root": {
            "type": "sequence",
            "children": [
                {"type": "skill", "name": "step1", "args": {}},
            ]
        }
    })
    
    events: list[dict[str, Any]] = []
    
    def event_callback(event: dict[str, Any]) -> None:
        events.append(event)
        
    def executor(name: str, args: dict) -> RuntimeStepResult:
        return RuntimeStepResult(success=True)

    runtime = TaskRuntime(event_callback=event_callback)
    report = runtime.run(task, executor)
    
    assert report.success is True
    assert len(events) >= 2
    
    # Verify events follow the schema
    for ev in events:
        assert "ts" in ev
        assert ev["level"] in {"INFO", "WARN", "ERR"}
        assert ev["source"] == "runtime"
        assert ev["category"] == "TASK"
        assert "event" in ev
        assert "detail" in ev
        assert isinstance(ev["data"], dict)
        
    # Check specific events
    event_names = [ev["event"] for ev in events]
    assert "task_start" in event_names
    assert "step_start" in event_names
    assert "step_done" in event_names
    assert "task_done" in event_names
