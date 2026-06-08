"""Small helpers for bounded latest-frame processing."""

from __future__ import annotations

from threading import Lock
from typing import Generic, TypeVar

T = TypeVar("T")


class LatestValueSlot(Generic[T]):
    """Single-slot mailbox: newer input replaces stale pending input."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._value: T | None = None

    def put(self, value: T) -> None:
        with self._lock:
            self._value = value

    def take(self) -> T | None:
        with self._lock:
            value = self._value
            self._value = None
            return value


class MonotonicRateGate:
    """Permit processing no faster than the configured rate."""

    def __init__(self, rate_hz: float) -> None:
        self._period_s = 0.0 if rate_hz <= 0.0 else 1.0 / float(rate_hz)
        self._last_allowed_s: float | None = None

    def allow(self, now: float) -> bool:
        if self._period_s <= 0.0:
            return True
        if self._last_allowed_s is None or now - self._last_allowed_s >= self._period_s:
            self._last_allowed_s = now
            return True
        return False
