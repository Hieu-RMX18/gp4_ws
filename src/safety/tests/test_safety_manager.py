import threading

from industrial_msgs.msg import RobotStatus, TriState
from interfaces.msg import RobotReadiness

from safety.safety_manager import SafetyManager


class _FakeLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _make_manager(sim_mode: bool):
    manager = object.__new__(SafetyManager)
    manager._lock = threading.Lock()
    manager._sim_mode = sim_mode
    manager._robot_ready = False
    manager._last_error_reason = (
        "no hw_adapter readiness received yet (sim mode)"
        if sim_mode
        else "no robot status received yet (fail-closed)"
    )
    manager._status_received = False
    manager._adapter_ready_received = False
    published = []
    manager.publish_status = published.append
    manager.get_logger = lambda: _FakeLogger()
    return manager, published


def _make_ready_status() -> RobotStatus:
    msg = RobotStatus()
    msg.in_error.val = TriState.FALSE
    msg.e_stopped.val = TriState.FALSE
    msg.drives_powered.val = TriState.TRUE
    msg.motion_possible.val = TriState.TRUE
    return msg


def test_sim_mode_uses_hw_adapter_readiness_message():
    manager, published = _make_manager(sim_mode=True)
    msg = RobotReadiness()
    msg.ready = False
    msg.status_message = "simulation mode: robot status bypassed"

    manager.adapter_ready_callback(msg)

    assert manager.is_robot_ready is False
    assert manager.last_error_reason == "simulation mode: robot status bypassed"
    assert published[-1] == "BLOCKED: simulation mode: robot status bypassed"


def test_sim_mode_accepts_ready_hw_adapter_state():
    manager, published = _make_manager(sim_mode=True)
    msg = RobotReadiness()
    msg.ready = True
    msg.status_message = "simulation mode: robot status bypassed"

    manager.adapter_ready_callback(msg)

    assert manager.is_robot_ready is True
    assert manager.last_error_reason == ""
    assert published[-1] == "OK"


def test_sim_mode_ignores_raw_robot_status():
    manager, _ = _make_manager(sim_mode=True)
    msg = _make_ready_status()
    msg.in_error.val = TriState.TRUE

    manager.status_callback(msg)

    assert manager.is_robot_ready is False
    assert (
        manager.last_error_reason == "no hw_adapter readiness received yet (sim mode)"
    )


def test_hardware_mode_preserves_raw_robot_status_fail_closed():
    manager, published = _make_manager(sim_mode=False)
    msg = _make_ready_status()

    manager.status_callback(msg)

    assert manager.is_robot_ready is True
    assert manager.last_error_reason == ""
    assert published[-1] == "OK"
