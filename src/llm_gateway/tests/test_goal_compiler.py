from pathlib import Path

from llm_gateway.intent_engine import SkillCall, compile_goal
from llm_gateway.station_scene_graph import StationSceneGraph


def _graph(tmp_path: Path) -> StationSceneGraph:
    path = tmp_path / "station_semantic_map.yaml"
    path.write_text(
        """
metadata: {source: test, geometry_verified: true}
regions:
  conveyor:
    frame_id: base_link
    geometry:
      type: box
      center: {x: 0.3, y: 0.1, z: 0.25}
      size: {x: 0.2, y: 0.2, z: 0.1}
    aliases: [conveyor]
objects:
  white_workpiece:
    class_id: white_workpiece
    aliases: [phoi trang]
""".strip(),
        encoding="utf-8",
    )
    return StationSceneGraph.from_file(path)


def test_compile_pick_and_place_goal_emits_ordered_skill_calls(tmp_path: Path):
    calls = compile_goal(
        {"action": "pick_and_place", "object": "phoi trang", "destination": "conveyor"},
        scene_graph=_graph(tmp_path),
    )

    assert [call.name for call in calls] == [
        "refresh_scene",
        "approach_object",
        "pick_object",
        "place_object",
        "verify_postcondition",
    ]
    assert calls[2].args["object_id"] == "white_workpiece"
    assert calls[3].args["destination"] == "conveyor"


def test_compile_goal_returns_clarification_for_unknown_destination(tmp_path: Path):
    calls = compile_goal(
        {"action": "pick_and_place", "object": "phoi trang", "destination": "shelf"},
        scene_graph=_graph(tmp_path),
    )

    assert calls == [
        SkillCall(name="needs_clarification", args={"field": "destination", "query": "shelf"})
    ]
