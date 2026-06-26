"""Tests for the grasp pipeline: GripperCompileConfig, pick_object, place_object."""
import pytest
from llm_gateway.factory_task import (
    TaskCompiler,
    GripperCompileConfig,
    WorldModel,
    FactoryTaskError,
    parse_factory_task,
)


def _gripper():
    return GripperCompileConfig(close=(10017, 1), open=(10017, 0))


def _wm():
    return WorldModel(
        objects={"yellow_box": {"pose": {"position": {"x": 0.30, "y": 0.10, "z": 0.05},
                                          "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}},
        regions={"fixture": {"geometry": {"center": {"x": 0.281, "y": 0.182, "z": 0.15}}}},
    )


def _compile_skill_dict(skill_name, args):
    payload = {"task_type": "factory_task", "version": "1.0", "task_id": "t",
               "root": {"type": "skill", "name": skill_name, "args": args}}
    tc = TaskCompiler(world_model=_wm(), gripper=_gripper())
    return tc.compile(parse_factory_task(payload)).semantic_ir


# ── Task 1: GripperCompileConfig injected into TaskCompiler ──────────────

def test_taskcompiler_accepts_gripper_config():
    gc = GripperCompileConfig(close=(10017, 1), open=(10017, 0))
    tc = TaskCompiler(world_model=WorldModel(), gripper=gc)
    assert tc._gripper.close == (10017, 1)
    assert tc._gripper.open == (10017, 0)


# ── Task 2: pick_object emits approach→descend→close→lift ────────────────

def test_pick_object_emits_grasp_sequence_with_real_io():
    ir = _compile_skill_dict("pick_object", {"object_ref": "yellow_box"})
    steps = ir["steps"]
    purposes = [s["metadata"]["purpose"] for s in steps]
    assert purposes == ["pick_approach", "pick_descend", "pick_gripper", "pick_lift"]
    approach = steps[0]
    assert approach["intent"] == "absolute_move_ptp"
    assert approach["target_pose"]["position"]["z"] == 0.05 + 0.08
    assert approach["target_pose"]["orientation"] == {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
    assert steps[1]["intent"] == "move_relative" and steps[1]["delta"]["z"] == -0.08
    io = steps[2]
    assert io["intent"] == "io_set" and io["io_address"] == 10017 and io["io_value"] == 1
    assert steps[3]["intent"] == "move_relative" and steps[3]["delta"]["z"] == 0.08


# ── Task 3: place_object moves to destination region and opens ───────────

def test_place_object_moves_to_destination_region_and_opens():
    ir = _compile_skill_dict("place_object", {"object": "yellow_box", "destination": "fixture"})
    steps = ir["steps"]
    approach = steps[0]
    # Must approach the FIXTURE center (0.281,0.182,0.15)+clearance, not the object
    assert approach["target_pose"]["position"]["x"] == 0.281
    assert approach["target_pose"]["position"]["y"] == 0.182
    assert approach["target_pose"]["position"]["z"] == 0.15 + 0.08
    io = steps[2]
    assert io["io_address"] == 10017 and io["io_value"] == 0  # open/release
    assert [s["metadata"]["purpose"] for s in steps] == \
        ["place_approach", "place_descend", "place_gripper", "place_lift"]


def test_place_object_requires_destination():
    with pytest.raises(FactoryTaskError):
        _compile_skill_dict("place_object", {"object": "yellow_box"})


def test_pick_object_requires_gripper_config():
    """pick_object without gripper config must raise."""
    payload = {"task_type": "factory_task", "version": "1.0", "task_id": "t",
               "root": {"type": "skill", "name": "pick_object", "args": {"object": "yellow_box"}}}
    tc = TaskCompiler(world_model=_wm())  # no gripper
    with pytest.raises(FactoryTaskError, match="requires gripper config"):
        tc.compile(parse_factory_task(payload))
