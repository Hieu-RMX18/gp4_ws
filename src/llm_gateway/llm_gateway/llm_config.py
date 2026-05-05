"""Configuration loader for the phase-9 local 9router backend."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


_MODEL_PLACEHOLDER = "TEEN MODEL 9ROUTER"
_DOTENV_FILENAME = ".env"


def _default_safety_rules_path() -> str:
    """Resolve safety_rules.yaml from installed package or local source tree."""
    try:
        pkg_share = get_package_share_directory("safety")
        return os.path.join(pkg_share, "config", "safety_rules.yaml")
    except Exception:
        return str(
            Path(__file__).resolve().parents[2]
            / "safety"
            / "config"
            / "safety_rules.yaml"
        )


def _load_safety_temperature() -> float:
    """Read llm.react.temperature from safety_rules.yaml SSOT."""
    try:
        path = _default_safety_rules_path()
        with open(path, "r", encoding="utf-8") as f:
            rules = yaml.safe_load(f) or {}
        llm = rules.get("llm", {})
        react = llm.get("react", {})
        return float(react.get("temperature", 0.0))
    except Exception:
        return 0.0


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


def _parse_dotenv_file(dotenv_path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    with open(dotenv_path, "r", encoding="utf-8") as dotenv_file:
        for raw_line in dotenv_file:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("export "):
                line = line[len("export ") :].strip()
            if "=" not in line:
                continue

            key, raw_value = line.split("=", 1)
            key = key.strip()
            if not key:
                continue

            value = raw_value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
                value = value[1:-1]
            elif " #" in value:
                value = value.split(" #", 1)[0].rstrip()

            values[key] = value

    return values


def _iter_candidate_dotenv_paths(config_path: str) -> list[Path]:
    explicit_env_file = os.getenv("GP4_LLM_ENV_FILE", "").strip()
    candidates: list[Path] = []
    seen: set[Path] = set()

    if explicit_env_file:
        explicit_path = Path(explicit_env_file).expanduser().resolve()
        candidates.append(explicit_path)
        seen.add(explicit_path)

    search_roots = [
        Path(config_path).resolve().parent,
        Path.cwd().resolve(),
        Path(__file__).resolve().parent,
    ]
    for root in search_roots:
        for parent in (root, *root.parents):
            candidate = parent / _DOTENV_FILENAME
            if candidate not in seen:
                candidates.append(candidate)
                seen.add(candidate)

    return candidates


def _load_dotenv_values(config_path: str) -> dict[str, str]:
    for candidate in _iter_candidate_dotenv_paths(config_path):
        if candidate.is_file():
            return _parse_dotenv_file(candidate)
    return {}


def _pick_first_non_empty(*values: Any) -> str:
    for value in values:
        if value is None:
            continue
        text = str(value)
        if text:
            return text
    return ""


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
    # Retry policy for transient failures (network errors, HTTP 5xx, HTTP 429).
    # max_retries=0 disables retry. Non-transient failures (auth, 4xx) never retry.
    max_retries: int = 2
    retry_base_delay_sec: float = 0.5
    retry_max_delay_sec: float = 4.0

    @property
    def model_is_configured(self) -> bool:
        return bool(self.model) and self.model != _MODEL_PLACEHOLDER


def load_llm_backend_config(config_path: str | None = None) -> LLMBackendConfig:
    resolved_path = config_path or _default_config_path()
    dotenv_values = _load_dotenv_values(resolved_path)
    with open(resolved_path, "r", encoding="utf-8") as config_file:
        raw_config = yaml.safe_load(config_file) or {}
    if not isinstance(raw_config, dict):
        raise ValueError("llm.yaml root must be a mapping.")

    backend = raw_config.get("llm_backend") or {}
    if not isinstance(backend, dict):
        raise ValueError("llm_backend must be a mapping.")

    def _lookup_env(env_name: str) -> str:
        env_value = os.getenv(env_name)
        if env_value:
            return env_value
        return dotenv_values.get(env_name, "")

    def _resolve_env_ref(value: Any) -> Any:
        """Resolve ${ENV_VAR} references in string values."""
        if not isinstance(value, str):
            return value
        if value.startswith("${") and value.endswith("}"):
            env_name = value[2:-1]
            return _lookup_env(env_name)
        return value

    def pick(key: str, default: Any) -> Any:
        env_key = f"LLM_{key.upper()}"
        raw = _pick_first_non_empty(_lookup_env(env_key), backend.get(key, default))
        return _resolve_env_ref(raw)

    # api_key: dedicated gateway env vars win, then generic OpenAI-compatible env,
    # then the YAML value, which can also be written as ${ENV_VAR}.
    raw_api_key = _pick_first_non_empty(
        _lookup_env("GP4_LLM_API_KEY"),
        _lookup_env("LLM_API_KEY"),
        _lookup_env("OPENAI_API_KEY"),
        _resolve_env_ref(backend.get("api_key", "")),
    )

    return LLMBackendConfig(
        provider=str(backend.get("provider", "9router_local")),
        base_url=str(pick("base_url", "http://localhost:20128/v1")).rstrip("/"),
        api_key=str(raw_api_key),
        api_mode=str(backend.get("api_mode", "openai_compatible")),
        model=str(pick("model", _MODEL_PLACEHOLDER)),
        temperature=float(pick("temperature", _load_safety_temperature())),
        max_tokens=int(pick("max_tokens", 500)),
        require_json_only=_as_bool(pick("require_json_only", True), True),
        fail_on_non_json=_as_bool(pick("fail_on_non_json", True), True),
        fail_on_schema_mismatch=_as_bool(pick("fail_on_schema_mismatch", True), True),
        request_timeout_sec=float(pick("request_timeout_sec", 10.0)),
        max_retries=int(pick("max_retries", 2)),
        retry_base_delay_sec=float(pick("retry_base_delay_sec", 0.5)),
        retry_max_delay_sec=float(pick("retry_max_delay_sec", 4.0)),
    )
