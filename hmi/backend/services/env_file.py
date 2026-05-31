from __future__ import annotations

import os


def lookup_env_or_dotenv(name: str) -> str:
    return os.getenv(name, "")
