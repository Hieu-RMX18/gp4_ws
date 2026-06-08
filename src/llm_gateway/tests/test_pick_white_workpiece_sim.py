from llm_gateway.composite_tools import PostconditionVerifier


def test_pick_place_white_workpiece_completes_with_cached_scene_model():
    scene_before = {"detections": [{"class_id": "white_workpiece", "region": "fixture"}]}
    scene_after = {"detections": [{"class_id": "white_workpiece", "region": "conveyor"}]}
    verifier = PostconditionVerifier()

    assert verifier.verify_place(
        object_id="white_workpiece", destination="conveyor", scene=scene_before
    ).ok is False
    assert verifier.verify_place(
        object_id="white_workpiece", destination="conveyor", scene=scene_after
    ).ok is True
