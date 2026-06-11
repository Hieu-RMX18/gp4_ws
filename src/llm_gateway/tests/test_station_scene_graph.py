from pathlib import Path

import pytest

from llm_gateway.factory_task import (
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


def test_resolve_object_dynamically_from_live_scene(tmp_path: Path):
    """When an object is NOT in config but IS in live perception, resolve it."""
    path = tmp_path / "station_semantic_map.yaml"
    path.write_text(
        """
metadata:
  source: test
regions: {}
objects:
  white_workpiece:
    class_id: white_workpiece
    aliases: [phoi trang]
""".strip(),
        encoding="utf-8",
    )
    graph = StationSceneGraph.from_file(path)

    # Without live_scene, "apple" is unknown
    result_no_scene = graph.resolve_object("apple")
    assert result_no_scene.ok is False

    # With live_scene containing an "apple" detection, it resolves
    live_scene = {
        "detections": [
            {"class_id": "apple", "score": 0.95, "position": {"x": 0.3, "y": 0.1, "z": 0.25}},
        ]
    }
    result = graph.resolve_object("apple", live_scene=live_scene)
    assert result.ok is True
    assert result.name == "apple"
    assert result.payload["dynamic"] is True
    assert result.payload["class_id"] == "apple"


def test_resolve_object_static_takes_priority_over_dynamic(tmp_path: Path):
    """Static config match must always win over a live detection."""
    path = tmp_path / "station_semantic_map.yaml"
    path.write_text(
        """
metadata:
  source: test
regions: {}
objects:
  white_workpiece:
    class_id: white_workpiece
    aliases: [phoi trang, white workpiece]
""".strip(),
        encoding="utf-8",
    )
    graph = StationSceneGraph.from_file(path)

    live_scene = {
        "detections": [
            {"class_id": "white_workpiece", "score": 0.99, "position": {"x": 0.0, "y": 0.0, "z": 0.0}},
        ]
    }
    result = graph.resolve_object("white workpiece", live_scene=live_scene)
    assert result.ok is True
    assert result.name == "white_workpiece"
    # Static result does NOT have the "dynamic" key
    assert result.payload is not None
    assert result.payload.get("dynamic") is not True


def test_resolve_object_without_live_scene_still_fails_for_unknown(tmp_path: Path):
    """Backward compatibility: no live_scene means unknown objects fail."""
    path = tmp_path / "station_semantic_map.yaml"
    path.write_text(
        """
metadata:
  source: test
regions: {}
objects: {}
""".strip(),
        encoding="utf-8",
    )
    graph = StationSceneGraph.from_file(path)

    result = graph.resolve_object("bolt")
    assert result.ok is False
    assert result.error == "needs_clarification"
