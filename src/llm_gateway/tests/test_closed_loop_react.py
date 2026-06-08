from llm_gateway.composite_tools import PostconditionVerifier


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
