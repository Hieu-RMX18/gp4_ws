from __future__ import annotations

import os

def lookup_env_or_dotenv(name: str) -> str:
    try:
        import dotenv
        env_file = os.getenv("GP4_LLM_ENV_FILE")
        if env_file and os.path.exists(env_file):
            dotenv.load_dotenv(env_file)
        else:
            dotenv.load_dotenv()
    except ImportError:
        pass
    return os.getenv(name, "")
