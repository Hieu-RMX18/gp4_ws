"""Tests for llm_gateway.llm_config secret loading."""

from __future__ import annotations

from pathlib import Path

from llm_gateway.llm_config import load_llm_backend_config


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


def test_load_llm_backend_config_reads_api_key_from_nearby_dotenv(
    tmp_path: Path,
    monkeypatch,
) -> None:
    config_path = _write_config(tmp_path, '"${GP4_LLM_API_KEY}"')
    (tmp_path / ".env").write_text("GP4_LLM_API_KEY=from_dotenv\n", encoding="utf-8")

    monkeypatch.delenv("GP4_LLM_API_KEY", raising=False)
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    monkeypatch.delenv("GP4_LLM_ENV_FILE", raising=False)

    config = load_llm_backend_config(str(config_path))

    assert config.api_key == "from_dotenv"


def test_load_llm_backend_config_prefers_process_env_over_dotenv_and_yaml(
    tmp_path: Path,
    monkeypatch,
) -> None:
    config_path = _write_config(tmp_path, '"from_yaml"')
    (tmp_path / ".env").write_text("GP4_LLM_API_KEY=from_dotenv\n", encoding="utf-8")

    monkeypatch.setenv("GP4_LLM_API_KEY", "from_process_env")
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    monkeypatch.delenv("GP4_LLM_ENV_FILE", raising=False)

    config = load_llm_backend_config(str(config_path))

    assert config.api_key == "from_process_env"


def test_load_llm_backend_config_supports_explicit_env_file_override(
    tmp_path: Path,
    monkeypatch,
) -> None:
    workspace_dir = tmp_path / "workspace"
    workspace_dir.mkdir()
    config_path = _write_config(
        workspace_dir, '"${GP4_LLM_API_KEY}"', model_value='"${LLM_MODEL}"'
    )

    env_file = tmp_path / "secrets.env"
    env_file.write_text(
        "\n".join(
            [
                "GP4_LLM_API_KEY=from_explicit_env_file",
                "LLM_MODEL=from_dotenv_model",
                "",
            ]
        ),
        encoding="utf-8",
    )

    monkeypatch.delenv("GP4_LLM_API_KEY", raising=False)
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    monkeypatch.setenv("GP4_LLM_ENV_FILE", str(env_file))

    config = load_llm_backend_config(str(config_path))

    assert config.api_key == "from_explicit_env_file"
    assert config.model == "from_dotenv_model"


def test_load_llm_backend_config_reads_dotenv_from_current_working_directory(
    tmp_path: Path,
    monkeypatch,
) -> None:
    launch_root = tmp_path / "workspace"
    install_share = launch_root / "install" / "llm_gateway" / "share" / "llm_gateway"
    install_share.mkdir(parents=True)
    config_path = _write_config(install_share, '"${GP4_LLM_API_KEY}"')
    (launch_root / ".env").write_text(
        "GP4_LLM_API_KEY=from_launch_root\n", encoding="utf-8"
    )

    monkeypatch.chdir(launch_root)
    monkeypatch.delenv("GP4_LLM_API_KEY", raising=False)
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    monkeypatch.delenv("GP4_LLM_ENV_FILE", raising=False)

    config = load_llm_backend_config(str(config_path))

    assert config.api_key == "from_launch_root"
