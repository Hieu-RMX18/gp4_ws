"""Unit tests for the runtime STOP flag on LLMGatewayNode."""
from llm_gateway.llm_gateway_node import LLMGatewayNode


def test_runtime_stop_flag_defaults_false_and_sets_true():
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()
    assert node._runtime_is_stopped() is False
    node._set_runtime_stop(True)
    assert node._runtime_is_stopped() is True
    node._set_runtime_stop(False)
    assert node._runtime_is_stopped() is False
