"""ReAct loop driver — reasoning + tool use for LLM intent resolution."""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import jsonschema

from .iteration_budget import IterationBudget, IterationCounters
from .state_injector import StateInjector
from .tool_registry import ToolRegistry, ToolResult

_LOGGER = logging.getLogger(__name__)


@dataclass
class ToolCall:
    name: str
    args: dict


@dataclass
class AgentContext:
    """Shared context passed to tool invocations."""

    state_injector: StateInjector
    ros_node: Any = None


class ReActAgent:
    """ReAct agent: iteratively calls tools until a valid semantic IR is produced."""

    def __init__(
        self,
        llm_client,
        tool_registry: ToolRegistry,
        state_injector: StateInjector,
        budget: IterationBudget,
        schema_validator,
    ):
        self._llm_client = llm_client
        self._tool_registry = tool_registry
        self._state_injector = state_injector
        self._budget = budget
        self._schema_validator = schema_validator

    def run(self, user_text: str, request_id: str) -> dict:
        """Run the ReAct loop and return final structured command (semantic IR)."""
        state = self._state_injector.snapshot()
        history: List[Tuple[str, Any]] = []
        counters = IterationCounters()
        start_time = time.monotonic()
        context = AgentContext(state_injector=self._state_injector)

        while True:
            allowed, reason = counters.can_invoke_any(self._budget)
            if not allowed:
                return self._handoff(reason, history)

            messages = self._build_prompt(user_text, state, history)
            try:
                llm_response = self._llm_client.generate_response_from_messages(
                    messages
                )
            except Exception as exc:
                return self._handoff(f"llm_request_failed: {exc}", history)

            tool_call = self._parse_tool_call(llm_response)
            if tool_call is None:
                semantic_ir = self._extract_semantic_ir(llm_response)
                ok, err = self._validate_semantic_ir(semantic_ir)
                if not ok:
                    if counters.repair < self._budget.max_repair:
                        counters.repair += 1
                        history.append(("observation", f"validation_error: {err}"))
                        continue
                    return self._handoff(
                        f"semantic_ir invalid after repair: {err}", history
                    )
                return semantic_ir

            tool = self._tool_registry.get(tool_call.name)
            if tool is None:
                history.append(("observation", f"unknown_tool: {tool_call.name}"))
                continue

            allowed, reason = counters.can_invoke(tool, self._budget)
            if not allowed:
                return self._handoff(
                    f"budget exceeded for {tool.name}: {reason}", history
                )

            try:
                tool.validate_input(tool_call.args)
            except jsonschema.ValidationError as exc:
                history.append(("observation", f"tool_input_invalid: {exc.message}"))
                counters.repair += 1
                continue
            except Exception as exc:
                history.append(("observation", f"tool_input_invalid: {exc}"))
                counters.repair += 1
                continue

            try:
                result = tool.invoke(tool_call.args, context)
            except Exception as exc:
                result = ToolResult(ok=False, error=str(exc))

            counters.record(tool)
            state = self._state_injector.snapshot()
            history.append(("tool_call", tool_call))
            history.append(("observation", result.to_observation()))

            elapsed = time.monotonic() - start_time
            if elapsed > self._budget.wall_clock_timeout_s:
                return self._handoff("wall_clock_timeout", history)

    def _build_prompt(
        self,
        user_text: str,
        state: Dict[str, Any],
        history: List[Tuple[str, Any]],
    ) -> List[Dict[str, str]]:
        system_lines = [
            "You are a robot task planner. You DO NOT control the robot directly.",
            "You produce a structured command that the safety system reviews and the motion system executes.",
            "",
            self._tool_registry.available_tools_description(),
            "",
            "Current robot state:",
            json.dumps(state, indent=2),
            "",
            "When you have enough information, respond WITHOUT a tool call, with the final command JSON.",
            "The command must validate against the schema.",
            'Macros ("go home, then draw a circle") are expressed as a single command:',
            '{"primitive_type": "MACRO", "steps": [<command1>, <command2>, ...]}',
            "Each step is itself a valid primitive command.",
        ]
        messages: List[Dict[str, str]] = [
            {"role": "system", "content": "\n".join(system_lines)},
            {"role": "user", "content": user_text.strip()},
        ]
        for role, content in history:
            if isinstance(content, ToolCall):
                messages.append(
                    {
                        "role": "assistant",
                        "content": json.dumps(
                            {"tool_call": content.name, "args": content.args}
                        ),
                    }
                )
            else:
                messages.append({"role": "user", "content": str(content)})
        return messages

    def _parse_tool_call(self, llm_response: str) -> Optional[ToolCall]:
        """Parse an LLM response for a tool call.

        Returns None if the response is a final command (no tool call).
        """
        try:
            data = json.loads(llm_response)
        except json.JSONDecodeError:
            return None

        if isinstance(data, dict) and "tool_call" in data:
            name = data.get("tool_call", "")
            args = data.get("args", {})
            if name and isinstance(name, str):
                return ToolCall(name=name, args=args)
        return None

    def _extract_semantic_ir(self, llm_response: str) -> dict:
        """Extract the final semantic IR from LLM response."""
        try:
            return json.loads(llm_response)
        except json.JSONDecodeError:
            return {"intent": "raw_text", "text": llm_response.strip()}

    def _validate_semantic_ir(self, semantic_ir: dict) -> Tuple[bool, str]:
        ok, err = self._schema_validator.validate_against_schema(semantic_ir)
        if ok:
            return True, ""
        return False, str(err)

    def _handoff(self, reason: str, history: List[Tuple[str, Any]]) -> dict:
        def _serialize(content: Any) -> str:
            if isinstance(content, str):
                return content
            if isinstance(content, ToolCall):
                return json.dumps({"tool_call": content.name, "args": content.args})
            return json.dumps(content)

        return {
            "_handoff": True,
            "reason": reason,
            "history": [{"role": r, "content": _serialize(c)} for r, c in history],
        }
