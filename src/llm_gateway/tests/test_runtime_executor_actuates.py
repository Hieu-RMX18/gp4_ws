"""Test that _validate_runtime_command dispatches motion after safety passes."""
import pytest

pytest.importorskip(
    "interfaces", reason="requires built interfaces for LLMGatewayNode imports"
)

from llm_gateway.llm_gateway_node import LLMGatewayNode
from llm_gateway.runtime_dispatch import DispatchOutcome
from llm_gateway.task_runtime import RuntimeStepResult


def test_validate_runtime_command_dispatches_after_safety_passes(monkeypatch):
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()

    # Stub the safety+normalize chain to "valid" so we isolate dispatch.
    monkeypatch.setattr(node, "_runtime_command_is_safe",
                        lambda cmd: (True, "", {"primitive_type": "PTP"}, object()))

    dispatched = {"count": 0}

    def _fake_dispatch(normalized_command):
        dispatched["count"] += 1
        return DispatchOutcome(ok=True, reason="")
    monkeypatch.setattr(node, "_dispatch_runtime_goal", _fake_dispatch)

    result = node._validate_runtime_command({"primitive_type": "PTP"})

    assert result.success is True
    assert dispatched["count"] == 1


def test_validate_runtime_command_skips_dispatch_for_query_commands(monkeypatch):
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()

    monkeypatch.setattr(node, "_runtime_command_is_safe",
                        lambda cmd: (True, "", {"primitive_type": "GET_POSE"}, None))
    monkeypatch.setattr(node, "_is_query_command", lambda pt: True)

    dispatched = {"count": 0}

    def _fake_dispatch(normalized_command):
        dispatched["count"] += 1
        return DispatchOutcome(ok=True, reason="")
    monkeypatch.setattr(node, "_dispatch_runtime_goal", _fake_dispatch)

    result = node._validate_runtime_command({"primitive_type": "GET_POSE"})

    assert result.success is True
    assert dispatched["count"] == 0  # query commands don't dispatch motion


def test_validate_runtime_command_returns_failure_when_safety_fails(monkeypatch):
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()

    monkeypatch.setattr(node, "_runtime_command_is_safe",
                        lambda cmd: (False, "unsafe target", None, None))

    result = node._validate_runtime_command({"primitive_type": "PTP"})

    assert result.success is False
    assert "unsafe target" in result.reason
