"""LLM payload parser for direct JSON and legacy tool-call formats."""

from __future__ import annotations

import json
from typing import Any, Dict


def _strip_code_fences(text: str) -> str:
    stripped_text = text.strip()
    if stripped_text.startswith("```"):
        lines = stripped_text.splitlines()
        if len(lines) >= 2 and lines[-1].strip() == "```":
            stripped_text = "\n".join(lines[1:-1]).strip()
    return stripped_text


def _load_json_object(text: str, error_message: str) -> Dict[str, Any]:
    try:
        loaded = json.loads(_strip_code_fences(text))
    except json.JSONDecodeError as exc:
        raise ValueError(error_message) from exc
    if not isinstance(loaded, dict):
        raise ValueError(error_message)
    return loaded


def _decode_arguments(arguments: Any) -> Dict[str, Any]:
    if isinstance(arguments, dict):
        return arguments
    if isinstance(arguments, str):
        return _load_json_object(arguments, "Function arguments JSON must decode to an object.")
    raise ValueError("Function arguments must be an object or JSON string.")


def _canonical_command(name: Any, arguments: Any) -> Dict[str, Any]:
    if not isinstance(name, str) or not name:
        raise ValueError("Function call name must be a non-empty string.")
    if name != "execute_motion":
        raise ValueError(f"Unsupported function call name: {name}")
    return _decode_arguments(arguments)


def _parse_message_content(content: Any) -> Dict[str, Any]:
    if isinstance(content, str):
        return _load_json_object(content, "Model content must be a JSON object.")

    if isinstance(content, list):
        text_parts = []
        for block in content:
            if isinstance(block, str):
                text_parts.append(block)
                continue
            if not isinstance(block, dict):
                continue
            if block.get("type") in {"text", "output_text"}:
                text_value = block.get("text")
                if isinstance(text_value, str):
                    text_parts.append(text_value)
                elif isinstance(text_value, dict) and isinstance(text_value.get("value"), str):
                    text_parts.append(text_value["value"])
        if text_parts:
            return _load_json_object("".join(text_parts), "Model content must be a JSON object.")

    raise ValueError("Model content must be a JSON object.")


def parse_llm_output(text: str) -> Dict[str, Any]:
    """
    Parse provider payload and return the direct command object.

    Supported wrappers:
      - Direct JSON object output from the model
      - {"choices":[{"message":{"content":"{...json...}"}}]}
      - {"function_call": {"name": ..., "arguments": ...}}           # legacy
      - {"tool_calls": [{"function": {"name": ..., "arguments": ...}}]}  # legacy
      - {"choices":[{"message":{"function_call":...}}]}              # legacy
      - {"choices":[{"message":{"tool_calls":[...]}}]}               # legacy
      - {"content":[{"type":"tool_use","name":...,"input":{...}}]}
    """
    if not isinstance(text, str) or not text.strip():
        raise ValueError("Empty LLM payload.")

    try:
        payload = json.loads(_strip_code_fences(text))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON format: {exc.msg}") from exc

    if not isinstance(payload, dict):
        raise ValueError("Top-level payload must be a JSON object.")

    # Direct command payload from the model.
    if "primitive_type" in payload or "error" in payload:
        return payload

    # Already canonical legacy wrapper.
    if "name" in payload and "arguments" in payload:
        return _canonical_command(payload["name"], payload["arguments"])

    # Top-level OpenAI function_call (legacy).
    function_call = payload.get("function_call")
    if isinstance(function_call, dict):
        if "name" in function_call and "arguments" in function_call:
            return _canonical_command(function_call["name"], function_call["arguments"])

    # Top-level OpenAI tool_calls (legacy).
    tool_calls = payload.get("tool_calls")
    if isinstance(tool_calls, list) and tool_calls:
        first_call = tool_calls[0]
        if isinstance(first_call, dict):
            function_data = first_call.get("function")
            if isinstance(function_data, dict) and "name" in function_data and "arguments" in function_data:
                return _canonical_command(function_data["name"], function_data["arguments"])
            if "name" in first_call and "arguments" in first_call:
                return _canonical_command(first_call["name"], first_call["arguments"])

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
                        return _canonical_command(
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
                            return _canonical_command(function_data["name"], function_data["arguments"])
                        if "name" in first_call and "arguments" in first_call:
                            return _canonical_command(first_call["name"], first_call["arguments"])

                if "content" in message:
                    return _parse_message_content(message["content"])

            if isinstance(first_choice.get("text"), str):
                return _load_json_object(
                    first_choice["text"], "Model content must be a JSON object."
                )

    # OpenAI responses wrapper.
    if isinstance(payload.get("output_text"), str):
        return _load_json_object(payload["output_text"], "Model content must be a JSON object.")

    output = payload.get("output")
    if isinstance(output, list):
        for block in output:
            if isinstance(block, dict) and block.get("type") == "function_call":
                if "name" in block and "arguments" in block:
                    return _canonical_command(block["name"], block["arguments"])
            if isinstance(block, dict) and block.get("type") == "message":
                if "content" in block:
                    return _parse_message_content(block["content"])

    # Anthropic tool_use wrapper.
    content = payload.get("content")
    if isinstance(content, list):
        for block in content:
            if isinstance(block, dict) and block.get("type") == "tool_use":
                if "name" in block and "input" in block:
                    return _canonical_command(block["name"], block["input"])
        return _parse_message_content(content)

    raise ValueError("No JSON command payload found.")


class LLMParser:
    """Class wrapper for parser usage in node code."""

    def parse(self, text: str) -> Dict[str, Any]:
        return parse_llm_output(text)
