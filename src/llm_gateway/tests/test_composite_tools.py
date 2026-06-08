from types import SimpleNamespace

from llm_gateway.composite_tools import EmitSequenceTool, PickObjectTool, RefreshSceneTool


class _Node:
    def __init__(self):
        self.scene_refreshed = False

    def _invalidate_scene_cache(self):
        self.scene_refreshed = True


def test_emit_sequence_validates_child_semantic_ir():
    result = EmitSequenceTool().invoke(
        {"steps": [{"intent": "move_relative", "delta": {"z": 0.02}, "reference_frame": "base_link"}]},
        SimpleNamespace(ros_node=_Node()),
    )

    assert result.ok is True
    assert result.payload["semantic_ir"]["intent"] == "sequence"


def test_emit_sequence_rejects_raw_primitive_leakage():
    result = EmitSequenceTool().invoke(
        {"steps": [{"primitive_type": "LIN"}]},
        SimpleNamespace(ros_node=_Node()),
    )

    assert result.ok is False
    assert "primitive_type" in result.error


def test_refresh_scene_invalidates_gateway_cache():
    node = _Node()

    result = RefreshSceneTool().invoke(
        {}, SimpleNamespace(ros_node=node)
    )

    assert result.ok is True
    assert node.scene_refreshed is True


def test_pick_object_is_one_motion_tool_and_marks_world_change():
    tool = PickObjectTool()

    assert tool.is_motion is True
    assert tool.name == "pick_object"
