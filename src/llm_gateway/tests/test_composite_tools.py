from types import SimpleNamespace

from llm_gateway.composite_tools import (
    ApproachObjectTool,
    CandidatePoseRequest,
    EmitSequenceTool,
    PickObjectTool,
    PlaceObjectTool,
    PostconditionVerifier,
    RefreshSceneTool,
    VerifyGraspTool,
    VerifyPostconditionTool,
    generate_candidate_poses,
    mtc_select,
)


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

def test_postcondition_verifier_requires_object_in_destination():
    verifier = PostconditionVerifier()
    result = verifier.verify_place(
        object_id="white_workpiece",
        destination="conveyor",
        scene={"detections": [{"class_id": "white_workpiece", "region": "fixture"}]},
    )

    assert result.ok is False
    assert result.error == "postcondition_failed"

def test_postcondition_verifier_accepts_object_in_destination():
    verifier = PostconditionVerifier()
    result = verifier.verify_place(
        object_id="white_workpiece",
        destination="conveyor",
        scene={"detections": [{"class_id": "white_workpiece", "region": "conveyor"}]},
    )

    assert result.ok is True

def test_candidate_pose_rejects_verify_config_geometry():
    request = CandidatePoseRequest(
        purpose="drop",
        region={"geometry": {"center": {"x": "VERIFY_CONFIG", "y": 0.0, "z": 0.3}}},
        safety_rules={
            "workspace_bounds": {
                "x_min": -0.45,
                "x_max": 0.45,
                "y_min": -0.16,
                "y_max": 0.52,
                "z_min": 0.15,
                "z_max": 0.65,
            }
        },
    )

    result = generate_candidate_poses(request)

    assert result.ok is False
    assert result.error == "verify_config_required"

def test_candidate_pose_applies_tool_offset_once_and_keeps_workspace_bounds():
    request = CandidatePoseRequest(
        purpose="drop",
        region={"geometry": {"center": {"x": 0.30, "y": 0.10, "z": 0.30}}},
        safety_rules={
            "workspace_bounds": {
                "x_min": -0.45,
                "x_max": 0.45,
                "y_min": -0.16,
                "y_max": 0.52,
                "z_min": 0.15,
                "z_max": 0.65,
            }
        },
        tcp_offset_m=0.12,
        approach_axis="+z_base",
    )

    result = generate_candidate_poses(request)

    assert result.ok is True
    assert result.poses[0]["position"]["z"] == 0.42

def test_mtc_select_returns_mtc_when_all_prereqs_met():
    result = mtc_select(
        mtc_service_ready_fn=lambda: True,
        object_pose_known=True,
        destination_known=True,
        gripper_config_verified=True,
    )
    assert result == "mtc"

def test_mtc_select_returns_primitive_when_service_unavailable():
    result = mtc_select(
        mtc_service_ready_fn=lambda: False,
        object_pose_known=True,
        destination_known=True,
        gripper_config_verified=True,
    )
    assert result == "primitive"

def test_mtc_select_returns_capability_unavailable_when_prereqs_missing():
    result = mtc_select(
        mtc_service_ready_fn=lambda: True,
        object_pose_known=False,
        destination_known=True,
        gripper_config_verified=True,
    )
    assert result == "capability_unavailable"

def test_verify_grasp_fails_closed_without_adapter():
    tool = VerifyGraspTool()

    result = tool.invoke({"object_id": "white_workpiece"}, SimpleNamespace(ros_node=None))

    assert result.ok is False
    assert result.error == "capability_unavailable"
    assert tool.is_readonly is True

def test_verify_grasp_fails_with_verify_config_gripper():
    from llm_gateway.composite_tools import GripperConfig, GripperIoAdapter

    config = GripperConfig.from_rules({"gripper": {"open_output_address": "VERIFY_CONFIG"}})
    adapter = GripperIoAdapter(config=config, node=None, robot_mode_fn=lambda: "IDLE")

    class _FakeNode:
        _gripper_adapter = adapter

    tool = VerifyGraspTool()
    result = tool.invoke(
        {"object_id": "white_workpiece"},
        SimpleNamespace(ros_node=_FakeNode()),
    )

    assert result.ok is False
    assert result.error == "verify_config_required"
