"""Test confirm wiring: stop-reset, event callback, dispatched_to_ros flag."""
import pytest

pytest.importorskip(
    "interfaces", reason="requires built interfaces for LLMGatewayNode imports"
)

from types import SimpleNamespace
from llm_gateway.llm_gateway_node import LLMGatewayNode
from llm_gateway.task_runtime import RuntimeStepResult


def test_confirm_runtime_resets_stop_and_reports_dispatched(monkeypatch):
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()
    node._set_runtime_stop(True)  # stale stop from a previous task
    node._motion_result_timeout_sec = 30.0
    node._safety_service_timeout_sec = 2.0

    # Minimal fakes so the runtime tree (single go_home skill) succeeds.
    from llm_gateway.factory_task import WorldModel
    monkeypatch.setattr(node, "_factory_task_world_model", lambda: WorldModel())

    # Stub the full IR compile + validate chain used by _execute_skill.
    monkeypatch.setattr(
        node, "_semantic_ir_for_runtime_skill",
        lambda task_payload, name, args: {"intent": "go_home"},
    )
    monkeypatch.setattr(
        node, "_validate_runtime_semantic_ir",
        lambda semantic_ir: RuntimeStepResult(success=True),
    )

    # Stub event sink.
    events = []
    monkeypatch.setattr(node, "_runtime_event_sink", lambda evt: events.append(evt))

    class _Req:
        operator_id = "op"
        plan_fingerprint = "abc123def456"

    class _Resp:
        accepted = False
        reason = ""
        execution_summary = ""
        dispatched_to_ros = False

    sentinel = {
        "_factory_task_runtime": True,
        "metadata": {
            "factory_task": {
                "task_id": "home-wired",
                "version": "1.0",
                "mode": "supervised_hardware",
            },
            "runtime_plan": {"type": "skill", "name": "go_home"},
        },
    }

    resp = node._on_confirm_factory_task_runtime(sentinel, _Req(), _Resp())

    assert node._runtime_is_stopped() is False     # reset at task start
    assert resp.accepted is True
    assert resp.dispatched_to_ros is True          # motion path now actuates
    assert len(events) > 0                          # event callback was wired


def test_confirm_runtime_wires_is_stopped_fn():
    """TaskRuntime receives is_stopped_fn and can abort mid-task."""
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()
    node._motion_result_timeout_sec = 30.0
    node._safety_service_timeout_sec = 2.0

    from llm_gateway.factory_task import WorldModel
    node._factory_task_world_model = lambda: WorldModel()

    call_count = {"n": 0}

    def _failing_skill(task_payload, name, args):
        call_count["n"] += 1
        # Signal stop after first skill compiles.
        node._set_runtime_stop(True)
        return {"intent": "go_home"}

    node._semantic_ir_for_runtime_skill = _failing_skill
    node._validate_runtime_semantic_ir = lambda ir: RuntimeStepResult(success=True)
    node._runtime_event_sink = lambda evt: None

    class _Req:
        operator_id = "op"
        plan_fingerprint = "abc123def456"

    class _Resp:
        accepted = False
        reason = ""
        execution_summary = ""
        dispatched_to_ros = False

    sentinel = {
        "_factory_task_runtime": True,
        "metadata": {
            "factory_task": {"task_id": "stop-test", "version": "1.0"},
            "runtime_plan": {
                "type": "sequence",
                "children": [
                    {"type": "skill", "name": "go_home"},
                    {"type": "skill", "name": "go_home"},
                ],
            },
        },
    }

    resp = node._on_confirm_factory_task_runtime(sentinel, _Req(), _Resp())

    # First skill ran and set stop, so second skill should NOT execute.
    assert resp.accepted is False
    assert "operator_stopped" in resp.reason
    assert call_count["n"] == 1  # only first skill executed
