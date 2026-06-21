"""Simulation tests for closed-loop execution of FactoryTask sequences.

These tests run the actual TaskRuntime over a constructed FactoryTask,
mocking only the low-level _execute_skill dispatch to verify the
runtime control flow (sequence, retry, stop) works as expected.
"""
from typing import Dict, Any

from llm_gateway.factory_task import (
    FactoryTask,
    TaskNode,
    WorldModel,
)
from llm_gateway.task_runtime import TaskRuntime, RuntimeStepResult


def _make_pick_place_task() -> FactoryTask:
    """Create a basic observe -> pick -> place sequence."""
    return FactoryTask(
        task_id="sim-pick-1",
        version="1.0",
        mode="supervised_hardware",
        operator_summary="pick the red object and place it in the bin",
        limits={"max_retries_per_skill": 2},
        replan_policy={"enabled": False},
        root=TaskNode(
            type="sequence",
            children=[
                TaskNode(type="skill", name="observe_scene", args={}),
                TaskNode(
                    type="retry",
                    count=2,
                    children=[
                        TaskNode(
                            type="skill",
                            name="pick_object",
                            args={"object": "red_cube"},
                        )
                    ],
                ),
                TaskNode(type="skill", name="place_object", args={"target": "bin"}),
            ],
        ),
    )


class MockHardwareExecutor:
    def __init__(self):
        self.log = []
        self.fail_counts = {}

    def execute(self, name: str, args: Dict[str, Any]) -> RuntimeStepResult:
        self.log.append((name, args))
        
        # If we requested this skill to fail N times, fail it
        fails_remaining = self.fail_counts.get(name, 0)
        if fails_remaining > 0:
            self.fail_counts[name] = fails_remaining - 1
            return RuntimeStepResult(success=False, reason="mock failure")
            
        return RuntimeStepResult(success=True)


def test_happy_path_executes_all_skills():
    task = _make_pick_place_task()
    hw = MockHardwareExecutor()
    runtime = TaskRuntime(
        world_model=WorldModel(),
        is_stopped_fn=lambda: False,
        event_callback=lambda evt: None,
    )

    report = runtime.run(task, hw.execute)

    assert report.success is True
    assert len(hw.log) == 3
    assert hw.log[0][0] == "observe_scene"
    assert hw.log[1][0] == "pick_object"
    assert hw.log[2][0] == "place_object"


def test_grasp_fail_then_retry_succeeds():
    task = _make_pick_place_task()
    hw = MockHardwareExecutor()
    # Force pick to fail once, it should succeed on retry
    hw.fail_counts["pick_object"] = 1
    
    runtime = TaskRuntime(
        world_model=WorldModel(),
        is_stopped_fn=lambda: False,
        event_callback=lambda evt: None,
    )

    report = runtime.run(task, hw.execute)

    assert report.success is True
    assert len(hw.log) == 4
    assert hw.log[0][0] == "observe_scene"
    assert hw.log[1][0] == "pick_object"  # Fails
    assert hw.log[2][0] == "pick_object"  # Succeeds
    assert hw.log[3][0] == "place_object"
    assert report.attempts_by_skill["pick_object"] == 2


def test_stop_midway_aborts():
    task = _make_pick_place_task()
    hw = MockHardwareExecutor()
    
    # State for stopping
    stop_flag = False
    
    def _execute_and_stop(name: str, args: Dict[str, Any]) -> RuntimeStepResult:
        nonlocal stop_flag
        result = hw.execute(name, args)
        if name == "pick_object":
            stop_flag = True  # Stop right after pick
        return result
        
    runtime = TaskRuntime(
        world_model=WorldModel(),
        is_stopped_fn=lambda: stop_flag,
        event_callback=lambda evt: None,
    )

    report = runtime.run(task, _execute_and_stop)

    assert report.success is False
    assert report.reason == "operator_stopped"
    assert len(hw.log) == 2
    assert hw.log[0][0] == "observe_scene"
    assert hw.log[1][0] == "pick_object"
    # The place_object skill is never executed
