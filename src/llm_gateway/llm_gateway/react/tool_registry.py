"""Tool registry and base classes for ReAct agent tools."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, ClassVar, Dict, List, Type

import jsonschema

if TYPE_CHECKING:
    from .agent import AgentContext


@dataclass
class ToolResult:
    ok: bool
    payload: dict | None = None
    error: str | None = None

    def to_observation(self) -> str:
        if self.ok:
            return json.dumps({"ok": True, "payload": self.payload})
        return json.dumps({"ok": False, "error": self.error})


class Tool:
    name: ClassVar[str] = ""
    description: ClassVar[str] = ""
    input_schema: ClassVar[dict] = {}
    is_motion: ClassVar[bool] = False
    is_readonly: ClassVar[bool] = False

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        raise NotImplementedError

    def validate_input(self, args: dict) -> None:
        if self.input_schema:
            jsonschema.validate(instance=args, schema=self.input_schema)


class ToolRegistry:
    """Registry of ReAct tools by name."""

    def __init__(self) -> None:
        self._tools: Dict[str, Tool] = {}

    def register(self, tool: Tool) -> "ToolRegistry":
        if not tool.name:
            raise ValueError("Tool must define a name.")
        self._tools[tool.name] = tool
        return self

    def get(self, name: str) -> Tool | None:
        return self._tools.get(name)

    def list_tools(self) -> List[dict]:
        return [
            {
                "name": t.name,
                "description": t.description,
                "input_schema": t.input_schema,
                "is_motion": t.is_motion,
                "is_readonly": t.is_readonly,
            }
            for t in self._tools.values()
        ]

    def available_tools_description(self) -> str:
        lines = ["Available tools (one tool call per response, return JSON):"]
        for t in self._tools.values():
            lines.append(f"- {t.name}: {t.description}")
        return "\n".join(lines)
