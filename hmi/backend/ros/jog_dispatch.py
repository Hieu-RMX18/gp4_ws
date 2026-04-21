from __future__ import annotations


class JogDispatchMixin:
    """Jog dispatch seam for WorkspaceRosAdapter.

    Jog pendant command transport is currently handled by
    hmi.backend.services.jog_pendant_service via /web_jog_command and
    /servo_bridge/* interfaces, so this mixin is intentionally empty.
    """

    pass
