from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timedelta, timezone
from threading import Lock
from uuid import uuid4

from ..domain.models import LeaseRecord, LeaseRole


def utcnow() -> datetime:
    return datetime.now(timezone.utc)


class LeaseRejectedError(RuntimeError):
    """Raised when a controller lease cannot be granted safely."""


class LeaseNotOwnedError(PermissionError):
    """Raised when a session attempts a controller action without ownership."""


class SessionLockService:
    def __init__(self, ttl_seconds: int = 15) -> None:
        self._ttl = timedelta(seconds=ttl_seconds)
        self._active_controller: LeaseRecord | None = None
        self._lock = Lock()

    def _purge_if_expired(self, now: datetime) -> None:
        if self._active_controller and self._active_controller.is_expired(now):
            self._active_controller = None

    def current_controller(self) -> LeaseRecord | None:
        with self._lock:
            now = utcnow()
            self._purge_if_expired(now)
            return self._active_controller

    def acquire_controller(
        self,
        session_id: str,
        operator_id: str,
        *,
        force_takeover: bool = False,
        takeover_reason: str | None = None,
    ) -> LeaseRecord:
        with self._lock:
            now = utcnow()
            self._purge_if_expired(now)

            if (
                self._active_controller
                and self._active_controller.session_id != session_id
            ):
                if not force_takeover:
                    raise LeaseRejectedError(
                        "controller lease is already held by another session"
                    )
                if not takeover_reason:
                    raise LeaseRejectedError(
                        "force takeover requires a non-empty takeover reason"
                    )

            if (
                self._active_controller
                and self._active_controller.session_id == session_id
            ):
                self._active_controller = replace(
                    self._active_controller,
                    operator_id=operator_id,
                    expires_at=now + self._ttl,
                )
                return self._active_controller

            self._active_controller = LeaseRecord(
                lease_id=str(uuid4()),
                lease_token=str(uuid4()),
                role=LeaseRole.CONTROLLER,
                session_id=session_id,
                operator_id=operator_id,
                acquired_at=now,
                expires_at=now + self._ttl,
                force_takeover=force_takeover,
                takeover_reason=takeover_reason,
            )
            return self._active_controller

    def renew(
        self,
        session_id: str,
        operator_id: str,
        lease_token: str,
    ) -> LeaseRecord:
        with self._lock:
            now = utcnow()
            self._purge_if_expired(now)
            if not self._active_controller:
                raise LeaseNotOwnedError("no active controller lease to renew")
            if self._active_controller.session_id != session_id:
                raise LeaseNotOwnedError("controller lease belongs to another session")
            if self._active_controller.operator_id != operator_id:
                raise LeaseNotOwnedError("controller lease belongs to another operator")
            if self._active_controller.lease_token != lease_token:
                raise LeaseNotOwnedError("lease token mismatch")

            self._active_controller = replace(
                self._active_controller,
                expires_at=now + self._ttl,
            )
            return self._active_controller

    def release(
        self,
        session_id: str,
        operator_id: str,
        lease_token: str,
    ) -> None:
        with self._lock:
            if not self._active_controller:
                return
            if self._active_controller.session_id != session_id:
                raise LeaseNotOwnedError("controller lease belongs to another session")
            if self._active_controller.operator_id != operator_id:
                raise LeaseNotOwnedError("controller lease belongs to another operator")
            if self._active_controller.lease_token != lease_token:
                raise LeaseNotOwnedError("lease token mismatch")
            self._active_controller = None

    def assert_controller(
        self,
        session_id: str,
        operator_id: str,
        lease_token: str | None,
    ) -> LeaseRecord:
        if not lease_token:
            raise LeaseNotOwnedError("controller lease token is required")

        with self._lock:
            now = utcnow()
            self._purge_if_expired(now)
            if not self._active_controller:
                raise LeaseNotOwnedError("no active controller lease")
            if self._active_controller.session_id != session_id:
                raise LeaseNotOwnedError("controller lease belongs to another session")
            if self._active_controller.operator_id != operator_id:
                raise LeaseNotOwnedError("controller lease belongs to another operator")
            if self._active_controller.lease_token != lease_token:
                raise LeaseNotOwnedError("lease token mismatch")
            return self._active_controller
