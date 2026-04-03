"""LLM payload parser for OpenAI/Anthropic function-call formats."""

from __future__ import annotations

import json
from typing import Any, Dict


def _decode_arguments(arguments: Any) -> Dict[str, Any]:
    if isinstance(arguments, dict):
        return arguments
    if isinstance(arguments, str):
        try:
            loaded = json.loads(arguments)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid function arguments JSON: {exc.msg}") from exc
        if not isinstance(loaded, dict):
            raise ValueError("Function arguments JSON must decode to an object.")
        return loaded
    raise ValueError("Function arguments must be an object or JSON string.")


def _canonical_call(name: Any, arguments: Any) -> Dict[str, Any]:
    if not isinstance(name, str) or not name:
        raise ValueError("Function call name must be a non-empty string.")
    return {"name": name, "arguments": _decode_arguments(arguments)}


def parse_llm_output(text: str) -> Dict[str, Any]:
    """
    Parse provider payload and return canonical dict:
      {"name": "<function_name>", "arguments": { ... }}

    Supported wrappers:
      - {"function_call": {"name": ..., "arguments": ...}}
      - {"tool_calls": [{"function": {"name": ..., "arguments": ...}}]}
      - {"choices":[{"message":{"function_call":...}}]}
      - {"choices":[{"message":{"tool_calls":[...]}}]}
      - {"content":[{"type":"tool_use","name":...,"input":{...}}]}
    """
    if not isinstance(text, str) or not text.strip():
        raise ValueError("Empty LLM payload.")

    stripped_text = text.strip()
    if stripped_text.startswith("```"):
        lines = stripped_text.splitlines()
        if len(lines) >= 2 and lines[-1].strip() == "```":
            stripped_text = "\n".join(lines[1:-1]).strip()

    try:
        payload = json.loads(stripped_text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON format: {exc.msg}") from exc

    if not isinstance(payload, dict):
        raise ValueError("Top-level payload must be a JSON object.")

    # Already canonical.
    if "name" in payload and "arguments" in payload:
        return _canonical_call(payload["name"], payload["arguments"])

    # Top-level OpenAI function_call.
    function_call = payload.get("function_call")
    if isinstance(function_call, dict):
        if "name" in function_call and "arguments" in function_call:
            return _canonical_call(function_call["name"], function_call["arguments"])

    # Top-level OpenAI tool_calls.
    tool_calls = payload.get("tool_calls")
    if isinstance(tool_calls, list) and tool_calls:
        first_call = tool_calls[0]
        if isinstance(first_call, dict):
            function_data = first_call.get("function")
            if isinstance(function_data, dict) and "name" in function_data and "arguments" in function_data:
                return _canonical_call(function_data["name"], function_data["arguments"])
            if "name" in first_call and "arguments" in first_call:
                return _canonical_call(first_call["name"], first_call["arguments"])

    # OpenAI chat completions wrapper.
    choices = payload.get("choices")
    if isinstance(choices, list) and choices:
        first_choice = choices[0]
        if isinstance(first_choice, dict):
            message = first_choice.get("message", {})
            if isinstance(message, dict):
                message_function_call = message.get("function_call")
                if isinstance(message_function_call, dict):
                    if "name" in message_function_call and "arguments" in message_function_call:
                        return _canonical_call(
                            message_function_call["name"], message_function_call["arguments"]
                        )

                message_tool_calls = message.get("tool_calls")
                if isinstance(message_tool_calls, list) and message_tool_calls:
                    first_call = message_tool_calls[0]
                    if isinstance(first_call, dict):
                        function_data = first_call.get("function")
                        if (
                            isinstance(function_data, dict)
                            and "name" in function_data
                            and "arguments" in function_data
                        ):
                            return _canonical_call(function_data["name"], function_data["arguments"])
                        if "name" in first_call and "arguments" in first_call:
                            return _canonical_call(first_call["name"], first_call["arguments"])

    # OpenAI responses wrapper.
    output = payload.get("output")
    if isinstance(output, list):
        for block in output:
            if isinstance(block, dict) and block.get("type") == "function_call":
                if "name" in block and "arguments" in block:
                    return _canonical_call(block["name"], block["arguments"])

    # Anthropic tool_use wrapper.
    content = payload.get("content")
    if isinstance(content, list):
        for block in content:
            if isinstance(block, dict) and block.get("type") == "tool_use":
                if "name" in block and "input" in block:
                    return _canonical_call(block["name"], block["input"])

    raise ValueError("No function_call/tool_calls payload found.")


class LLMParser:
    """Class wrapper for parser usage in node code."""

    def parse(self, text: str) -> Dict[str, Any]:
        return parse_llm_output(text)
