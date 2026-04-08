from __future__ import annotations

from .models import CommandLifecycleState, SystemRuntimeState


TERMINAL_COMMAND_STATES = {
    CommandLifecycleState.SUCCEEDED,
    CommandLifecycleState.FAILED,
    CommandLifecycleState.REJECTED,
    CommandLifecycleState.CANCELLED,
    CommandLifecycleState.ABORTED,
}

BLOCKING_RUNTIME_STATES = {
    SystemRuntimeState.FAULT,
    SystemRuntimeState.ESTOP,
    SystemRuntimeState.LOST_CONN,
    SystemRuntimeState.SAFETY_BLOCKED,
}

ALLOWED_COMMAND_TRANSITIONS = {
    CommandLifecycleState.IDLE: {CommandLifecycleState.RECEIVED},
    CommandLifecycleState.RECEIVED: {
        CommandLifecycleState.PARSED,
        CommandLifecycleState.REJECTED,
        CommandLifecycleState.ABORTED,
    },
    CommandLifecycleState.PARSED: {
        CommandLifecycleState.VALIDATED,
        CommandLifecycleState.REJECTED,
        CommandLifecycleState.ABORTED,
    },
    CommandLifecycleState.VALIDATED: {
        CommandLifecycleState.PLANNED,
        CommandLifecycleState.REJECTED,
        CommandLifecycleState.ABORTED,
    },
    CommandLifecycleState.PLANNED: {
        CommandLifecycleState.QUALITY_CHECKED,
        CommandLifecycleState.REJECTED,
        CommandLifecycleState.ABORTED,
    },
    CommandLifecycleState.QUALITY_CHECKED: {
        CommandLifecycleState.READY_FOR_CONFIRM,
        CommandLifecycleState.REJECTED,
        CommandLifecycleState.ABORTED,
    },
    CommandLifecycleState.READY_FOR_CONFIRM: {
        CommandLifecycleState.EXECUTING,
        CommandLifecycleState.CANCELLED,
        CommandLifecycleState.ABORTED,
    },
    CommandLifecycleState.EXECUTING: {
        CommandLifecycleState.SUCCEEDED,
        CommandLifecycleState.FAILED,
        CommandLifecycleState.CANCELLED,
        CommandLifecycleState.ABORTED,
    },
    CommandLifecycleState.SUCCEEDED: set(),
    CommandLifecycleState.FAILED: set(),
    CommandLifecycleState.REJECTED: set(),
    CommandLifecycleState.CANCELLED: set(),
    CommandLifecycleState.ABORTED: set(),
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

