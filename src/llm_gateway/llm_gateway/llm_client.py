"""OpenAI-compatible client for a local 9router backend."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Dict, List

from .llm_config import LLMBackendConfig
from .prompt_builder import build_system_prompt


class OpenAICompatibleLLMClient:
    """Minimal chat/completions client for a local OpenAI-compatible backend."""

    def __init__(self, config: LLMBackendConfig, schema_json: str):
        self._config = config
        self._system_prompt = build_system_prompt(schema_json)

    def generate_response(self, user_input: str) -> str:
        if not isinstance(user_input, str) or not user_input.strip():
            raise ValueError("user_input must be a non-empty string.")
        if not self._config.model_is_configured:
            raise ValueError(
                "llm_backend.model is not configured. Set the 9router model alias in llm.yaml or LLM_MODEL."
            )

        payload = {
            "model": self._config.model,
            "temperature": self._config.temperature,
            "max_tokens": self._config.max_tokens,
            "messages": self._build_messages(user_input),
            "stream": False,
        }
        if self._config.require_json_only:
            payload["response_format"] = {"type": "json_object"}

        request_body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
        request = urllib.request.Request(
            url=f"{self._config.base_url}/chat/completions",
            data=request_body,
            headers=self._build_headers(),
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=self._config.request_timeout_sec) as response:
                return response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            error_body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"LLM request failed with HTTP {exc.code}: {error_body}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"LLM request failed: {exc.reason}") from exc

    def _build_messages(self, user_input: str) -> List[Dict[str, str]]:
        return [
            {
                "role": "system",
                "content": self._system_prompt,
            },
            {
                "role": "user",
                "content": user_input.strip(),
            },
        ]

    def _build_headers(self) -> Dict[str, str]:
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        api_key = self._config.api_key or os.getenv("LLM_API_KEY") or os.getenv("OPENAI_API_KEY")
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"
        return headers
