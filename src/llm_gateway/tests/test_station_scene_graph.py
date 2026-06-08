from pathlib import Path

import pytest

from llm_gateway.station_scene_graph import (
    StationSceneGraph,
    load_station_semantic_map,
    map_contains_verify_config,
)


def test_load_station_map_preserves_verify_config(tmp_path: Path):
    path = tmp_path / "station_semantic_map.yaml"
    path.write_text(
        """
metadata:
  source: test
  geometry_verified: false
regions:
  conveyor:
    frame_id: base_link
    geometry:
      type: box
      center: {x: VERIFY_CONFIG, y: 0.0, z: 0.3}
      size: {x: 0.2, y: 0.2, z: 0.1}
    aliases: [conveyor, bang tai]
objects:
  white_workpiece:
    class_id: white_workpiece
    aliases: [phoi trang, white workpiece]
""".strip(),
        encoding="utf-8",
    )

    loaded = load_station_semantic_map(path)

    assert loaded["regions"]["conveyor"]["geometry"]["center"]["x"] == "VERIFY_CONFIG"
    assert map_contains_verify_config(loaded) is True


def test_scene_graph_resolves_aliases_and_rejects_verify_config_for_runtime(tmp_path: Path):
    path = tmp_path / "station_semantic_map.yaml"
    path.write_text(
        """
metadata:
  source: test
  geometry_verified: false
regions:
  fixture:
    frame_id: base_link
    geometry:
      type: box
      center: {x: VERIFY_CONFIG, y: 0.0, z: 0.3}
      size: {x: 0.2, y: 0.2, z: 0.1}
    aliases: [fixture, ga phoi]
objects: {}
""".strip(),
        encoding="utf-8",
    )
    graph = StationSceneGraph.from_file(path)

    resolved = graph.resolve_region("ga phoi")

    assert resolved.ok is True
    assert resolved.name == "fixture"
    assert graph.runtime_geometry_ready("fixture") is False
    assert graph.runtime_block_reason("fixture") == "verify_config_required"
