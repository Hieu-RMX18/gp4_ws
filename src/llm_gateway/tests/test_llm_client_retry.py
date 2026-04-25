"""Unit tests for OpenAICompatibleLLMClient retry policy.

Covers:
- Transient HTTP statuses (408/429/5xx) retry up to max_retries.
- Non-transient HTTP statuses (400/401/403/404) raise immediately.
- URLError (network) retries up to max_retries.
- max_retries=0 disables retry.
- Backoff delay is bounded by retry_max_delay_sec.
- Successful response after transient failures returns the body.
"""

from __future__ import annotations

import io
import random
import urllib.error
from dataclasses import replace
from types import SimpleNamespace
from typing import List
from unittest.mock import patch

import pytest

from llm_gateway.llm_client import (
    OpenAICompatibleLLMClient,
    _TRANSIENT_HTTP_STATUS,
)
from llm_gateway.llm_config import LLMBackendConfig


def _make_config(**overrides) -> LLMBackendConfig:
    base = LLMBackendConfig(
        provider="test",
        base_url="http://127.0.0.1:1/v1",
        api_key="",
        api_mode="openai_compatible",
        model="test-model",
        temperature=0.0,
        max_tokens=64,
        require_json_only=True,
        fail_on_non_json=True,
        fail_on_schema_mismatch=True,
        request_timeout_sec=1.0,
        max_retries=2,
        retry_base_delay_sec=0.01,
        retry_max_delay_sec=0.04,
    )
    return replace(base, **overrides)


class _FakeResponse:
    """Context manager mimicking urllib response with a canned body."""

    def __init__(self, body: str):
        self._body = body.encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def read(self):
        return self._body


def _http_error(status: int, body: str = "err") -> urllib.error.HTTPError:
    return urllib.error.HTTPError(
        url="http://test", code=status, msg="err",
        hdrs=None, fp=io.BytesIO(body.encode("utf-8")),
    )


def _make_client(config: LLMBackendConfig) -> OpenAICompatibleLLMClient:
    # Injected no-op sleep + deterministic RNG (jitter = 0).
    sleeps: List[float] = []
    client = OpenAICompatibleLLMClient(
        config=config,
        schema_json='{"type":"object"}',
        sleep_fn=sleeps.append,
        rng=random.Random(0),
    )
    client._recorded_sleeps = sleeps  # type: ignore[attr-defined]
    return client


def test_success_on_first_attempt_no_sleep():
    client = _make_client(_make_config())
    with patch("urllib.request.urlopen", return_value=_FakeResponse("OK")) as mock_open:
        result = client.generate_response("move to home")
    assert result == "OK"
    assert mock_open.call_count == 1
    assert client._recorded_sleeps == []


@pytest.mark.parametrize("status", sorted(_TRANSIENT_HTTP_STATUS))
def test_transient_http_retries_then_succeeds(status: int):
    client = _make_client(_make_config(max_retries=2))
    side_effects = [_http_error(status), _http_error(status), _FakeResponse("OK")]
    with patch("urllib.request.urlopen", side_effect=side_effects) as mock_open:
        result = client.generate_response("move")
    assert result == "OK"
    # 1 initial + 2 retries = 3 attempts
    assert mock_open.call_count == 3
    # 2 sleeps between the 3 attempts
    assert len(client._recorded_sleeps) == 2


@pytest.mark.parametrize("status", [400, 401, 403, 404, 422])
def test_non_transient_http_no_retry(status: int):
    client = _make_client(_make_config(max_retries=5))
    with patch("urllib.request.urlopen", side_effect=_http_error(status)) as mock_open:
        with pytest.raises(RuntimeError, match=f"HTTP {status}"):
            client.generate_response("move")
    assert mock_open.call_count == 1
    assert client._recorded_sleeps == []


def test_urlerror_retries_then_raises_after_exhaustion():
    client = _make_client(_make_config(max_retries=2))
    err = urllib.error.URLError("connection refused")
    with patch("urllib.request.urlopen", side_effect=err) as mock_open:
        with pytest.raises(RuntimeError, match="connection refused"):
            client.generate_response("move")
    # 1 initial + 2 retries = 3 attempts
    assert mock_open.call_count == 3


def test_max_retries_zero_disables_retry():
    client = _make_client(_make_config(max_retries=0))
    with patch(
        "urllib.request.urlopen",
        side_effect=urllib.error.URLError("down"),
    ) as mock_open:
        with pytest.raises(RuntimeError, match="down"):
            client.generate_response("move")
    assert mock_open.call_count == 1
    assert client._recorded_sleeps == []


def test_backoff_delay_is_capped():
    """After exponential growth, delay must not exceed retry_max_delay_sec."""
    config = _make_config(
        max_retries=5,
        retry_base_delay_sec=0.1,
        retry_max_delay_sec=0.3,
    )
    client = _make_client(config)
    err = urllib.error.URLError("oops")
    with patch("urllib.request.urlopen", side_effect=err):
        with pytest.raises(RuntimeError):
            client.generate_response("move")
    # All recorded sleeps must be <= cap * 1.25 (cap + max jitter)
    cap_plus_jitter = 0.3 * 1.25
    assert all(s <= cap_plus_jitter + 1e-9 for s in client._recorded_sleeps)


def test_transient_then_non_transient_raises_without_further_retry():
    """A non-transient 4xx after one transient retry must stop the loop."""
    client = _make_client(_make_config(max_retries=3))
    side_effects = [_http_error(503), _http_error(400)]
    with patch("urllib.request.urlopen", side_effect=side_effects) as mock_open:
        with pytest.raises(RuntimeError, match="HTTP 400"):
            client.generate_response("move")
    assert mock_open.call_count == 2


def test_empty_user_input_raises_valueerror():
    client = _make_client(_make_config())
    with pytest.raises(ValueError):
        client.generate_response("")


def test_unconfigured_model_raises_valueerror():
    client = _make_client(_make_config(model="TEEN MODEL 9ROUTER"))
    with pytest.raises(ValueError, match="not configured"):
        client.generate_response("move")
