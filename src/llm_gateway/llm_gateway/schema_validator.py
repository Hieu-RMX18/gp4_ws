"""JSON schema validator for LLM command payloads."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Tuple

import jsonschema
from ament_index_python.packages import get_package_share_directory


def _default_schema_path() -> str:
    """Resolve command_schema.json from installed package or local source tree."""
    try:
        pkg_share = get_package_share_directory("llm_gateway")
        return os.path.join(pkg_share, "config", "command_schema.json")
    except Exception:
        # Fallback for direct source-tree execution in tests/tools.
        return str(Path(__file__).resolve().parents[1] / "config" / "command_schema.json")


class SchemaValidator:
    """Load and validate command dicts against command_schema.json."""

    def __init__(self, schema_path: str | None = None):
        self._schema_path = schema_path or _default_schema_path()
        with open(self._schema_path, "r", encoding="utf-8") as schema_file:
            self._schema = json.load(schema_file)

    def validate_against_schema(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        """Return (True, '') when valid, otherwise (False, detailed_error)."""
        try:
            jsonschema.validate(instance=data, schema=self._schema)
            return True, ""
        except jsonschema.ValidationError as exc:
            path = ".".join(str(item) for item in exc.path)
            if path:
                return False, f"{path}: {exc.message}"
            return False, exc.message
        except Exception as exc:
            return False, str(exc)

    def validate(self, data: Dict[str, Any]) -> bool:
        """Compatibility helper for code paths expecting exceptions on failure."""
        valid, error = self.validate_against_schema(data)
        if not valid:
            raise ValueError(error)
        return True
