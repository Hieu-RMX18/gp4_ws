"""OpenAI-compatible client for a local 9router backend."""

from __future__ import annotations

import json
import logging
import os
import random
import time
import urllib.error
import urllib.request
from typing import Callable, Dict, List

from .llm_config import LLMBackendConfig
from .prompt_builder import build_system_prompt

_LOGGER = logging.getLogger(__name__)

# HTTP status codes considered transient and worth retrying.
# 408 Request Timeout, 429 Too Many Requests, 500/502/503/504 server/upstream errors.
_TRANSIENT_HTTP_STATUS = {408, 429, 500, 502, 503, 504}


def _is_transient_http(status: int) -> bool:
    return status in _TRANSIENT_HTTP_STATUS


class OpenAICompatibleLLMClient:
    """Minimal chat/completions client for a local OpenAI-compatible backend.

    Retries only transient failures (network errors, HTTP 408/429/5xx) with
    exponential backoff and jitter. Non-transient failures (auth, 4xx except
    408/429) are raised immediately so upstream validation does not retry
    a request the server will reject deterministically.
    """

    def __init__(
        self,
        config: LLMBackendConfig,
        schema_json: str,
        sleep_fn: Callable[[float], None] = time.sleep,
        rng: random.Random | None = None,
    ):
        self._config = config
        self._system_prompt = build_system_prompt(schema_json)
        # Injected for deterministic testing.
        self._sleep = sleep_fn
        self._rng = rng or random.Random()

    def generate_response(self, user_input: str) -> str:
        if not isinstance(user_input, str) or not user_input.strip():
            raise ValueError("user_input must be a non-empty string.")
        if not self._config.model_is_configured:
            raise ValueError(
                "llm_backend.model is not configured. Set the 9router model alias in llm.yaml or LLM_MODEL."
            )

        request = self._build_request(user_input)
        max_attempts = max(1, 1 + int(self._config.max_retries))
        last_exc: Exception | None = None

        for attempt in range(1, max_attempts + 1):
            try:
                with urllib.request.urlopen(
                    request, timeout=self._config.request_timeout_sec
                ) as response:
                    return response.read().decode("utf-8")
            except urllib.error.HTTPError as exc:
                error_body = exc.read().decode("utf-8", errors="replace")
                wrapped = RuntimeError(
                    f"LLM request failed with HTTP {exc.code}: {error_body}"
                )
                wrapped.__cause__ = exc
                if not _is_transient_http(exc.code) or attempt >= max_attempts:
                    raise wrapped from exc
                last_exc = wrapped
                _LOGGER.warning(
                    "LLM transient HTTP %d on attempt %d/%d; will retry.",
                    exc.code,
                    attempt,
                    max_attempts,
                )
            except urllib.error.URLError as exc:
                wrapped = RuntimeError(f"LLM request failed: {exc.reason}")
                wrapped.__cause__ = exc
                if attempt >= max_attempts:
                    raise wrapped from exc
                last_exc = wrapped
                _LOGGER.warning(
                    "LLM network error on attempt %d/%d: %s; will retry.",
                    attempt,
                    max_attempts,
                    exc.reason,
                )

            self._sleep(self._backoff_delay(attempt))

    def generate_response_from_messages(self, messages: list[dict[str, str]]) -> str:
        """Send a request with pre-constructed messages (ReAct multi-turn)."""
        if not self._config.model_is_configured:
            raise ValueError(
                "llm_backend.model is not configured. Set the 9router model alias."
            )
        request = self._build_request_from_messages(messages)
        max_attempts = max(1, 1 + int(self._config.max_retries))
        last_exc: Exception | None = None
        for attempt in range(1, max_attempts + 1):
            try:
                with urllib.request.urlopen(
                    request, timeout=self._config.request_timeout_sec
                ) as response:
                    return response.read().decode("utf-8")
            except urllib.error.HTTPError as exc:
                error_body = exc.read().decode("utf-8", errors="replace")
                wrapped = RuntimeError(
                    f"LLM request failed with HTTP {exc.code}: {error_body}"
                )
                wrapped.__cause__ = exc
                if not _is_transient_http(exc.code) or attempt >= max_attempts:
                    raise wrapped from exc
                last_exc = wrapped
            except urllib.error.URLError as exc:
                wrapped = RuntimeError(f"LLM request failed: {exc.reason}")
                wrapped.__cause__ = exc
                if attempt >= max_attempts:
                    raise wrapped from exc
                last_exc = wrapped
            self._sleep(self._backoff_delay(attempt))
        assert last_exc is not None
        raise last_exc

    def _build_request_from_messages(
        self, messages: list[dict[str, str]]
    ) -> urllib.request.Request:
        payload = {
            "model": self._config.model,
            "temperature": self._config.temperature,
            "max_tokens": self._config.max_tokens,
            "messages": messages,
            "stream": False,
        }
        if self._config.require_json_only:
            payload["response_format"] = {"type": "json_object"}
        request_body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
        return urllib.request.Request(
            url=f"{self._config.base_url}/chat/completions",
            data=request_body,
            headers=self._build_headers(),
            method="POST",
        )

    def _build_request(self, user_input: str) -> urllib.request.Request:
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
        return urllib.request.Request(
            url=f"{self._config.base_url}/chat/completions",
            data=request_body,
            headers=self._build_headers(),
            method="POST",
        )

    def _backoff_delay(self, attempt: int) -> float:
        """Exponential backoff with ±25% jitter, capped at retry_max_delay_sec."""
        base = max(0.0, float(self._config.retry_base_delay_sec))
        cap = max(base, float(self._config.retry_max_delay_sec))
        # attempt is 1-based: first retry after attempt=1 uses base, then 2*base, ...
        exp = base * (2 ** (attempt - 1))
        capped = min(cap, exp)
        jitter = capped * 0.25
        return max(0.0, capped + self._rng.uniform(-jitter, jitter))

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
        api_key = (
            self._config.api_key
            or os.getenv("LLM_API_KEY")
            or os.getenv("OPENAI_API_KEY")
        )
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"
        return headers
