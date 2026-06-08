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

from types import SimpleNamespace

from llm_gateway.composite_tools import ApproachObjectTool, PlaceObjectTool, VerifyPostconditionTool

def test_approach_object_emits_motion_sequence():
    tool = ApproachObjectTool()

    assert tool.is_motion is True
    result = tool.invoke({"object_id": "white_workpiece"}, SimpleNamespace(ros_node=None))

    assert result.ok is True
    assert result.payload["semantic_ir"]["intent"] == "sequence"
    assert result.payload["object_id"] == "white_workpiece"

def test_place_object_emits_descend_release_lift_sequence():
    tool = PlaceObjectTool()

    result = tool.invoke(
        {"object_id": "white_workpiece", "destination": "conveyor"},
        SimpleNamespace(ros_node=None),
    )

    assert result.ok is True
    steps = result.payload["semantic_ir"]["steps"]
    intents = [s["intent"] for s in steps]
    assert intents == ["move_relative", "io_set", "move_relative"]
    assert result.payload["destination"] == "conveyor"
    assert tool.is_motion is True

def test_verify_postcondition_fails_closed_without_scene_fn():
    tool = VerifyPostconditionTool()

    result = tool.invoke(
        {"object_id": "white_workpiece", "destination": "conveyor"},
        SimpleNamespace(ros_node=None),
    )

    assert result.ok is False
    assert result.error == "capability_unavailable"
    assert tool.is_readonly is True

def test_verify_postcondition_passes_when_object_in_destination():
    def _scene_fn():
        return {"detections": [{"class_id": "white_workpiece", "region": "conveyor"}]}

    class _FakeNode:
        _query_scene_for_verify = staticmethod(_scene_fn)

    tool = VerifyPostconditionTool()
    result = tool.invoke(
        {"object_id": "white_workpiece", "destination": "conveyor"},
        SimpleNamespace(ros_node=_FakeNode()),
    )

    assert result.ok is True

def test_verify_postcondition_fails_when_object_not_in_destination():
    def _scene_fn():
        return {"detections": [{"class_id": "white_workpiece", "region": "fixture"}]}

    class _FakeNode:
        _query_scene_for_verify = staticmethod(_scene_fn)

    tool = VerifyPostconditionTool()
    result = tool.invoke(
        {"object_id": "white_workpiece", "destination": "conveyor"},
        SimpleNamespace(ros_node=_FakeNode()),
    )

    assert result.ok is False
    assert result.error == "postcondition_failed"
