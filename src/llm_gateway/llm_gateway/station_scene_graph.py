from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

VERIFY_CONFIG = "VERIFY_CONFIG"


@dataclass(frozen=True)
class ResolveResult:
    ok: bool
    name: str = ""
    payload: dict[str, Any] | None = None
    error: str = ""
    candidates: tuple[str, ...] = ()


def load_station_semantic_map(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError("station semantic map root must be a mapping")
    return data


def map_contains_verify_config(value: Any) -> bool:
    if value == VERIFY_CONFIG:
        return True
    if isinstance(value, dict):
        return any(map_contains_verify_config(child) for child in value.values())
    if isinstance(value, list):
        return any(map_contains_verify_config(child) for child in value)
    return False


class StationSceneGraph:
    def __init__(self, data: dict[str, Any]):
        self._data = data
        self._regions = data.get("regions") if isinstance(data.get("regions"), dict) else {}
        self._objects = data.get("objects") if isinstance(data.get("objects"), dict) else {}

    @classmethod
    def from_file(cls, path: str | Path) -> "StationSceneGraph":
        return cls(load_station_semantic_map(path))

    def resolve_region(self, query: str) -> ResolveResult:
        return self._resolve_named(query, self._regions)

    def resolve_zone(self, region_name: str, query: str) -> ResolveResult:
        region = self._regions.get(region_name)
        zones = region.get("zones") if isinstance(region, dict) else None
        if not isinstance(zones, dict):
            return ResolveResult(ok=False, error="needs_clarification")
        return self._resolve_named(query, zones)

    def resolve_object(self, query: str) -> ResolveResult:
        return self._resolve_named(query, self._objects)

    def runtime_geometry_ready(self, region_name: str) -> bool:
        region = self._regions.get(region_name)
        return isinstance(region, dict) and not map_contains_verify_config(region.get("geometry"))

    def runtime_block_reason(self, region_name: str) -> str:
        return "" if self.runtime_geometry_ready(region_name) else "verify_config_required"

    def nearest_free_cell(
        self, region_name: str, object_size: dict[str, float] | None = None
    ) -> ResolveResult:
        region = self._regions.get(region_name)
        if not isinstance(region, dict):
            return ResolveResult(ok=False, error="needs_clarification")
        if not self.runtime_geometry_ready(region_name):
            return ResolveResult(ok=False, name=region_name, error="verify_config_required")
        return ResolveResult(ok=False, name=region_name, error="capability_unavailable")

    def _resolve_named(self, query: str, collection: dict[str, Any]) -> ResolveResult:
        normalized = _normalize(query)
        matches: list[str] = []
        for name, payload in collection.items():
            aliases = payload.get("aliases", []) if isinstance(payload, dict) else []
            names = [name, *[str(alias) for alias in aliases]]
            if normalized in {_normalize(candidate) for candidate in names}:
                matches.append(name)
        if len(matches) == 1:
            name = matches[0]
            return ResolveResult(ok=True, name=name, payload=collection[name])
        if len(matches) > 1:
            return ResolveResult(
                ok=False, error="needs_clarification", candidates=tuple(sorted(matches))
            )
        return ResolveResult(ok=False, error="needs_clarification")


def _normalize(value: str) -> str:
    return " ".join(str(value).strip().lower().replace("_", " ").split())
