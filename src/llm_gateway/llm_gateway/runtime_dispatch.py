"""Synchronous ExecuteMotion dispatch+await for the FactoryTask runtime executor.

Pure helper: no rclpy import, no node state. The node injects its execute action
client, its non-spinning wait function, and a stop predicate.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class DispatchOutcome:
    ok: bool
    reason: str = ""


def dispatch_and_await(
    execute_client: Any,
    *,
    goal: Any,
    wait_fn: Callable[[Any, float], tuple[bool, Any]],
    is_stopped_fn: Callable[[], bool],
    timeout_sec: float,
) -> DispatchOutcome:
    """Send a validated goal to ExecuteMotion and block until the result arrives.

    Safety contract: the caller (``_validate_runtime_command``) has already
    verified the command through ``/validate_command``.  This function only
    handles the transport layer — send, accept, await result, cancel on stop.

    Args:
        execute_client: The rclpy ActionClient for ``/execute_motion``.
        goal: A fully-built ``ExecuteMotion.Goal`` message.
        wait_fn: ``(future, timeout) -> (done, value)`` — the node's
            ``_wait_for_future_without_spinning``.
        is_stopped_fn: Returns ``True`` when an operator STOP or e-stop has
            been signalled.  Checked before send and after goal acceptance.
        timeout_sec: Maximum seconds to wait for each async step.

    Returns:
        A ``DispatchOutcome`` indicating success or the reason for failure.
    """
    # ── Pre-send stop check ──────────────────────────────────────────────
    if is_stopped_fn():
        return DispatchOutcome(ok=False, reason="operator_stopped")

    # ── Server availability ──────────────────────────────────────────────
    if execute_client is None or not execute_client.server_is_ready():
        return DispatchOutcome(ok=False, reason="ExecuteMotion action server unavailable")

    # ── Send goal ────────────────────────────────────────────────────────
    send_future = execute_client.send_goal_async(goal)
    done, goal_handle = wait_fn(send_future, timeout_sec)
    if not done or goal_handle is None:
        return DispatchOutcome(ok=False, reason="ExecuteMotion goal send timed out")
    if not getattr(goal_handle, "accepted", False):
        return DispatchOutcome(ok=False, reason="ExecuteMotion action server rejected goal")

    # ── Post-accept stop check (cancel in-flight goal) ───────────────────
    if is_stopped_fn():
        cancel = getattr(goal_handle, "cancel_goal_async", None)
        if callable(cancel):
            cancel()
        return DispatchOutcome(ok=False, reason="operator_stopped")

    # ── Await result ─────────────────────────────────────────────────────
    result_future = goal_handle.get_result_async()
    done, wrapped = wait_fn(result_future, timeout_sec)
    if not done or wrapped is None:
        return DispatchOutcome(ok=False, reason="ExecuteMotion result timed out")

    # interfaces/action/ExecuteMotion.Result: success(bool), message(str), execution_time_sec
    result = getattr(wrapped, "result", None)
    if not bool(getattr(result, "success", False)):
        msg = str(getattr(result, "message", "") or "motion failed")
        return DispatchOutcome(ok=False, reason=msg)
    return DispatchOutcome(ok=True, reason="")
