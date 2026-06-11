"""Gripper IO adapter — ROS-coupled hardware interface.

This module owns the GripperIoAdapter (which calls WriteSingleIO/ReadSingleIO
via ROS services) and its associated config/result dataclasses.

Spec: factory_task.py is NOT allowed to call ROS hardware services.
GripperIoAdapter lives here, adjacent to the node, because it requires
a live ROS node reference for IO service calls.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class GripperConfig:
    write_single_io_service: str
    read_single_io_service: str
    open_output_address: int | str
    open_output_value: int | str
    close_output_address: int | str
    close_output_value: int | str
    closed_input_address: int | str
    closed_input_active_value: int | str
    feedback_timeout_sec: float

    @classmethod
    def from_rules(cls, rules: dict[str, Any]) -> "GripperConfig":
        raw = rules.get("gripper", {}) if isinstance(rules, dict) else {}
        return cls(
            write_single_io_service=str(raw.get("write_single_io_service", "/io_set")),
            read_single_io_service=str(raw.get("read_single_io_service", "/read_single_io")),
            open_output_address=raw.get("open_output_address", "VERIFY_CONFIG"),
            open_output_value=raw.get("open_output_value", "VERIFY_CONFIG"),
            close_output_address=raw.get("close_output_address", "VERIFY_CONFIG"),
            close_output_value=raw.get("close_output_value", "VERIFY_CONFIG"),
            closed_input_address=raw.get("closed_input_address", "VERIFY_CONFIG"),
            closed_input_active_value=raw.get("closed_input_active_value", "VERIFY_CONFIG"),
            feedback_timeout_sec=float(raw.get("feedback_timeout_sec", 1.0)),
        )

    def verified(self) -> bool:
        values = (
            self.open_output_address,
            self.open_output_value,
            self.close_output_address,
            self.close_output_value,
            self.closed_input_address,
            self.closed_input_active_value,
        )
        return all(value != "VERIFY_CONFIG" for value in values)


@dataclass(frozen=True)
class GripperResult:
    ok: bool
    error: str = ""


class GripperIoAdapter:
    def __init__(self, *, config: GripperConfig, node: Any, robot_mode_fn):
        self._config = config
        self._node = node
        self._robot_mode_fn = robot_mode_fn

    def open(self) -> GripperResult:
        return self._write_guarded(
            self._config.open_output_address, self._config.open_output_value
        )

    def close(self) -> GripperResult:
        return self._write_guarded(
            self._config.close_output_address, self._config.close_output_value
        )

    def _write_guarded(self, address: int | str, value: int | str) -> GripperResult:
        if not self._config.verified():
            return GripperResult(ok=False, error="verify_config_required")
        if self._robot_mode_fn() != "IDLE":
            return GripperResult(ok=False, error="robot_not_idle")
        
        # Call WriteSingleIO service when config is verified and robot is idle
        client = getattr(self._node, "_write_single_io_client", None)
        if client is None or not client.service_is_ready():
            return GripperResult(ok=False, error="runtime_unavailable")
        
        try:
            from motoros2_interfaces.srv import WriteSingleIO
            request = WriteSingleIO.Request()
            request.address = int(address)
            request.value = int(value)
            future = client.call_async(request)
            # Synchronous wait (node has _wait_for_future_without_spinning)
            wait_fn = getattr(self._node, "_wait_for_future_without_spinning", None)
            if not callable(wait_fn):
                return GripperResult(ok=False, error="runtime_unavailable")
            done, response = wait_fn(future, self._config.feedback_timeout_sec)
            if not done or response is None:
                return GripperResult(ok=False, error="io_timeout")
            if not response.success:
                return GripperResult(ok=False, error=f"io_failed: {response.message}")
            return GripperResult(ok=True)
        except Exception as exc:
            return GripperResult(ok=False, error=f"io_exception: {exc}")
