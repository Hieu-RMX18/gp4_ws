from __future__ import annotations

from .models import CommandLifecycleState, SystemRuntimeState


TERMINAL_COMMAND_STATES = {
    CommandLifecycleState.SUCCEEDED,
    CommandLifecycleState.FAILED,
    CommandLifecycleState.REJECTED,
    CommandLifecycleState.CANCELLED,
    CommandLifecycleState.EXPIRED,
}

BLOCKING_RUNTIME_STATES = {
    SystemRuntimeState.FAULT,
    SystemRuntimeState.ESTOP,
    SystemRuntimeState.LOST_CONN,
    SystemRuntimeState.SAFETY_BLOCKED,
}

ALLOWED_COMMAND_TRANSITIONS = {
    CommandLifecycleState.RECEIVED: {
        CommandLifecycleState.PARSING,
        CommandLifecycleState.REJECTED,
        CommandLifecycleState.CANCELLED,
    },
    CommandLifecycleState.PARSING: {
        CommandLifecycleState.VALIDATING,
        CommandLifecycleState.REJECTED,
        CommandLifecycleState.CANCELLED,
    },
    CommandLifecycleState.VALIDATING: {
        CommandLifecycleState.NEEDS_CONFIRMATION,
        CommandLifecycleState.REJECTED,
        CommandLifecycleState.CANCELLED,
        CommandLifecycleState.EXPIRED,
    },
    CommandLifecycleState.NEEDS_CONFIRMATION: {
        CommandLifecycleState.CONFIRMED,
        CommandLifecycleState.CANCELLED,
        CommandLifecycleState.EXPIRED,
    },
    CommandLifecycleState.CONFIRMED: {
        CommandLifecycleState.EXECUTION_REQUESTED,
        CommandLifecycleState.FAILED,
        CommandLifecycleState.CANCELLED,
    },
    CommandLifecycleState.EXECUTION_REQUESTED: {
        CommandLifecycleState.EXECUTING,
        CommandLifecycleState.FAILED,
        CommandLifecycleState.CANCELLED,
    },
    CommandLifecycleState.EXECUTING: {
        CommandLifecycleState.SUCCEEDED,
        CommandLifecycleState.FAILED,
        CommandLifecycleState.CANCELLED,
    },
    CommandLifecycleState.SUCCEEDED: set(),
    CommandLifecycleState.FAILED: set(),
    CommandLifecycleState.REJECTED: set(),
    CommandLifecycleState.CANCELLED: set(),
    CommandLifecycleState.EXPIRED: set(),
}


def ensure_command_transition(
    previous: CommandLifecycleState,
    next_state: CommandLifecycleState,
) -> None:
    allowed = ALLOWED_COMMAND_TRANSITIONS.get(previous, set())
    if next_state not in allowed:
        raise ValueError(f"invalid lifecycle transition: {previous.value} -> {next_state.value}")


def is_terminal_command_state(state: CommandLifecycleState) -> bool:
    return state in TERMINAL_COMMAND_STATES


def is_blocking_runtime_state(state: SystemRuntimeState) -> bool:
    return state in BLOCKING_RUNTIME_STATES
