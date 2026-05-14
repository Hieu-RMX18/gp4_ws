"""Hardware gate evaluator — no-op for local development. Always unlocked."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class HardwareGateStatusSnapshot:
    unlocked: bool = True
    reasons: list[str] = field(default_factory=list)
    flag_enabled: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "unlocked": self.unlocked,
            "reasons": list(self.reasons),
            "flagEnabled": self.flag_enabled,
        }


class HardwareGateEvaluator:
    """No-op evaluator for local development. Always unlocked."""

    def evaluate(self) -> HardwareGateStatusSnapshot:
        return HardwareGateStatusSnapshot()
