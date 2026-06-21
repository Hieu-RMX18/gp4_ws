import json
from llm_gateway.llm_gateway_node import LLMGatewayNode


def test_runtime_event_sink_publishes_json_string():
    node = object.__new__(LLMGatewayNode)
    published = []
    class _Pub:
        def publish(self, msg): published.append(msg.data)
    node._task_events_pub = _Pub()

    node._runtime_event_sink({
        "ts": "10:00:00.000", "level": "INFO", "source": "runtime",
        "category": "TASK", "event": "task_start", "detail": "Starting", "data": {"task_id": "t"},
    })

    assert len(published) == 1
    decoded = json.loads(published[0])
    assert decoded["category"] == "TASK"
    assert decoded["event"] == "task_start"
    assert decoded["data"]["task_id"] == "t"


def test_emit_task_event_builds_valid_schema():
    node = object.__new__(LLMGatewayNode)
    published = []
    class _Pub:
        def publish(self, msg): published.append(msg.data)
    node._task_events_pub = _Pub()

    node._emit_task_event("SAFETY", "validate_rejected", "blocked by workspace bound",
                          level="WARN", source="safety", data={"rule": "x_max"})

    decoded = json.loads(published[0])
    assert set(decoded) == {"ts", "level", "source", "category", "event", "detail", "data"}
    assert decoded["level"] == "WARN" and decoded["source"] == "safety"
