"""Configuration loader for the phase-9 local 9router backend."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


_MODEL_PLACEHOLDER = "SET_THIS_FROM_YOUR_9ROUTER_ALIAS"


def _default_config_path() -> str:
    try:
        pkg_share = get_package_share_directory("llm_gateway")
        return os.path.join(pkg_share, "config", "llm.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "llm.yaml")


def _as_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


@dataclass(frozen=True)
class LLMBackendConfig:
    provider: str
    base_url: str
    api_key: str
    api_mode: str
    model: str
    temperature: float
    max_tokens: int
    require_json_only: bool
    fail_on_non_json: bool
    fail_on_schema_mismatch: bool
    request_timeout_sec: float

    @property
    def model_is_configured(self) -> bool:
        return bool(self.model) and self.model != _MODEL_PLACEHOLDER


def load_llm_backend_config(config_path: str | None = None) -> LLMBackendConfig:
    resolved_path = config_path or _default_config_path()
    with open(resolved_path, "r", encoding="utf-8") as config_file:
        raw_config = yaml.safe_load(config_file) or {}
    if not isinstance(raw_config, dict):
        raise ValueError("llm.yaml root must be a mapping.")

    backend = raw_config.get("llm_backend") or {}
    if not isinstance(backend, dict):
        raise ValueError("llm_backend must be a mapping.")

    def pick(key: str, default: Any) -> Any:
        env_key = f"LLM_{key.upper()}"
        return os.getenv(env_key, backend.get(key, default))

    return LLMBackendConfig(
        provider=str(backend.get("provider", "9router_local")),
        base_url=str(pick("base_url", "http://localhost:20128/v1")).rstrip("/"),
        api_key=str(pick("api_key", "")),
        api_mode=str(backend.get("api_mode", "openai_compatible")),
        model=str(pick("model", _MODEL_PLACEHOLDER)),
        temperature=float(pick("temperature", 0.0)),
        max_tokens=int(pick("max_tokens", 200)),
        require_json_only=_as_bool(pick("require_json_only", True), True),
        fail_on_non_json=_as_bool(pick("fail_on_non_json", True), True),
        fail_on_schema_mismatch=_as_bool(pick("fail_on_schema_mismatch", True), True),
        request_timeout_sec=float(pick("request_timeout_sec", 10.0)),
    )
