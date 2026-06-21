from llm_gateway.llm_gateway_node import LLMGatewayNode


def test_tier1_home_runs_through_runtime(monkeypatch):
    node = object.__new__(LLMGatewayNode)
    node._init_runtime_stop_state()
    ran = {"skills": []}
    monkeypatch.setattr(node, "_run_single_command_via_runtime",
                        lambda ir: ran["skills"].append(ir.get("intent")) or True, raising=False)

    ir = {"intent": "home", "_parse_source": "direct"}
    assert node._execute_tier1_command(ir) is True
    assert ran["skills"] == ["home"]
