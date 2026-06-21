import json
from llm_gateway.llm_gateway_node import LLMGatewayNode


class _CapturePub:
    def __init__(self): self.events = []
    def publish(self, msg): self.events.append(json.loads(msg.data))


def _node_with_pub():
    node = object.__new__(LLMGatewayNode)
    node._task_events_pub = _CapturePub()
    node._last_robot_status_fingerprint = None
    node._runtime_stop_flag = False
    return node


def test_robot_status_alarm_emits_hardware_event():
    node = _node_with_pub()
    class _Status:
        in_error = True
        error_code = 4012
        e_stopped = False
        in_motion = False
        servo_on = True
        mode = 2
    
    node._emit_robot_status_event(_Status())
    
    ev = node._task_events_pub.events[-1]
    assert ev["category"] == "HARDWARE"
    assert ev["data"]["error_code"] == 4012
    assert ev["level"] == "ERR"
