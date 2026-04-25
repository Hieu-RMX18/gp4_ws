"""Compatibility shim for legacy JogPendantService imports.

Use hmi.backend.services.jog_service.JogService for new code.
"""

from .jog_service import (
    DEFAULT_JOG_STATUS,
    JogBridgeState,
    JogBridgeStatusView,
    JogService,
    _build_jog_status_event,
)

JogPendantService = JogService

__all__ = [
    "DEFAULT_JOG_STATUS",
    "JogBridgeState",
    "JogBridgeStatusView",
    "JogService",
    "JogPendantService",
    "_build_jog_status_event",
]
