"""Iteration budget for ReAct agent — tiered limits."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .tool_registry import Tool


@dataclass
class IterationBudget:
    max_total: int = 5
    max_motion: int = 3
    max_readonly: int = 10
    max_repair: int = 1
    wall_clock_timeout_s: float = 30.0


@dataclass
class IterationCounters:
    total: int = 0
    motion: int = 0
    readonly: int = 0
    repair: int = 0

    def can_invoke(self, tool: "Tool", budget: IterationBudget) -> tuple[bool, str]:
        if self.total >= budget.max_total:
            return False, f"max_total exceeded ({self.total}/{budget.max_total})"
        if tool.is_motion and self.motion >= budget.max_motion:
            return False, f"max_motion exceeded ({self.motion}/{budget.max_motion})"
        if tool.is_readonly and self.readonly >= budget.max_readonly:
            return (
                False,
                f"max_readonly exceeded ({self.readonly}/{budget.max_readonly})",
            )
        return True, ""

    def can_invoke_any(self, budget: IterationBudget) -> tuple[bool, str]:
        if self.total >= budget.max_total:
            return False, f"max_total exceeded ({self.total}/{budget.max_total})"
        return True, ""

    def record(self, tool: "Tool") -> None:
        self.total += 1
        if tool.is_motion:
            self.motion += 1
        if tool.is_readonly:
            self.readonly += 1
