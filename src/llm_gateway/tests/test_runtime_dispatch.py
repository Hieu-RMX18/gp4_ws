"""Unit tests for the pure ExecuteMotion dispatch+await helper."""
from llm_gateway.runtime_dispatch import dispatch_and_await, DispatchOutcome


class _FakeFuture:
    def __init__(self, value):
        self._value = value

    def done(self):
        return True

    def result(self):
        return self._value


class _FakeGoalHandle:
    def __init__(self, accepted, result):
        self.accepted = accepted
        self._result = result

    def get_result_async(self):
        return _FakeFuture(self._result)

    def cancel_goal_async(self):
        return _FakeFuture(None)


class _FakeResultWrapper:
    def __init__(self, success, message=""):
        self.result = type("R", (), {"success": success, "message": message})()


class _FakeExecuteClient:
    def __init__(self, accepted=True, success=True, message=""):
        self._handle = _FakeGoalHandle(accepted, _FakeResultWrapper(success, message))

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        return _FakeFuture(self._handle)


def _wait(future, timeout):
    return True, future.result()


def test_dispatch_returns_ok_when_goal_accepted_and_result_success():
    out = dispatch_and_await(
        _FakeExecuteClient(accepted=True, success=True),
        goal=object(), wait_fn=_wait, is_stopped_fn=lambda: False, timeout_sec=1.0,
    )
    assert out == DispatchOutcome(ok=True, reason="")


def test_dispatch_fails_when_goal_rejected():
    out = dispatch_and_await(
        _FakeExecuteClient(accepted=False), goal=object(),
        wait_fn=_wait, is_stopped_fn=lambda: False, timeout_sec=1.0,
    )
    assert out.ok is False and "rejected" in out.reason


def test_dispatch_fails_when_result_reports_failure():
    out = dispatch_and_await(
        _FakeExecuteClient(accepted=True, success=False, message="planning failed"),
        goal=object(), wait_fn=_wait, is_stopped_fn=lambda: False, timeout_sec=1.0,
    )
    assert out.ok is False and "planning failed" in out.reason


def test_dispatch_cancels_and_fails_when_stop_requested_before_send():
    out = dispatch_and_await(
        _FakeExecuteClient(), goal=object(),
        wait_fn=_wait, is_stopped_fn=lambda: True, timeout_sec=1.0,
    )
    assert out.ok is False and out.reason == "operator_stopped"
