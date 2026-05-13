from __future__ import annotations

import os
from pathlib import Path


DOTENV_FILENAME = ".env"
ENV_FILE_POINTER = "GP4_LLM_ENV_FILE"


def _parse_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        raw_lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return values

    for raw_line in raw_lines:
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


def _candidate_env_paths() -> list[Path]:
    candidates: list[Path] = []
    explicit_env_file = os.getenv(ENV_FILE_POINTER, "").strip()
    if explicit_env_file:
        candidates.append(Path(explicit_env_file).expanduser().resolve())
    return candidates


def lookup_env_or_dotenv(name: str) -> str:
    env_value = os.getenv(name, "").strip()
    if env_value:
        return env_value

    for candidate in _candidate_env_paths():
        values = _parse_env_file(candidate)
        value = values.get(name, "").strip()
        if value:
            return value
    return ""
