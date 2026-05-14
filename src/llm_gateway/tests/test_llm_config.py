"""Tests for llm_gateway.react_planner secret loading."""

from __future__ import annotations

from pathlib import Path

from llm_gateway.react_planner import load_llm_backend_config


def _write_config(
    tmp_path: Path, api_key_value: str, model_value: str = '"gpt-5.4"'
) -> Path:
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    config_path = config_dir / "llm.yaml"
    config_path.write_text(
        "\n".join(
            [
                "llm_backend:",
                '  provider: "provider"',
                '  base_url: "http://localhost:20128/v1"',
                f"  api_key: {api_key_value}",
                '  api_mode: "openai_compatible"',
                f"  model: {model_value}",
                "  temperature: 0.0",
                "  max_tokens: 200",
                "  require_json_only: true",
                "  fail_on_non_json: true",
                "  fail_on_schema_mismatch: true",
                "  request_timeout_sec: 10.0",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return config_path


def test_load_llm_backend_config_prefers_process_env_over_yaml(
    tmp_path: Path,
    monkeypatch,
) -> None:
    config_path = _write_config(tmp_path, '"from_yaml"')

    monkeypatch.setenv("GP4_LLM_API_KEY", "from_process_env")
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)

    config = load_llm_backend_config(str(config_path))

    assert config.api_key == "from_process_env"  # pragma: allowlist secret
