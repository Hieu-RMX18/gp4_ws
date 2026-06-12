from types import SimpleNamespace
from hmi.backend.ros.adapter import WorkspaceRosAdapter

def test_task_event_message_is_forwarded_to_stream():
    adapter = WorkspaceRosAdapter()
    captured = []
    adapter.on_task_event_callback = lambda ev: captured.append(ev)

    msg = SimpleNamespace(data='{"ts":"10:00:00.000","level":"INFO","source":"runtime","category":"TASK","event":"task_start","detail":"x","data":{}}')
    # Call the ROS topic callback directly
    adapter._on_task_events(msg)

    assert len(captured) == 1
    assert captured[0]["channel"] == "task_event"
    assert captured[0]["event"]["category"] == "TASK"
