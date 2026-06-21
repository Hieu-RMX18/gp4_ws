from __future__ import annotations

import pytest

from llm_gateway.factory_task import (
    FACTORY_TASK_VERSION,
    FactoryTaskError,
    RuntimeStepResult,
    TaskCompiler,
    TaskRuntime,
    WorldModel,
    count_task_nodes,
    parse_factory_task,
)


def test_parse_factory_task_accepts_for_each_visible_object_tree() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "inspect-and-visit",
            "mode": "supervised_hardware",
            "operator_summary": "Confirm objects in station, then visit each one.",
            "root": {
                "type": "sequence",
                "children": [
                    {"type": "observe", "name": "observe_station", "args": {"region": "station"}},
                    {
                        "type": "for_each",
                        "collection": "visible_objects",
                        "item_name": "object",
                        "children": [
                            {
                                "type": "skill",
                                "name": "move_to_object",
                                "args": {"object_ref": "$object", "pose": "approach"},
                            }
                        ],
                    },
                ],
            },
        }
    )

    assert task.task_id == "inspect-and-visit"
    assert task.root.type == "sequence"
    assert task.root.children[1].type == "for_each"
    assert task.root.children[1].collection == "visible_objects"
    assert task.root.children[1].children[0].name == "move_to_object"


def test_parse_factory_task_keeps_repeat_as_runtime_loop_not_static_expansion() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "pose-a-home-100",
            "mode": "supervised_hardware",
            "root": {
                "type": "repeat",
                "count": 100,
                "children": [
                    {"type": "skill", "name": "move_to_region", "args": {"region": "pose_a"}},
                    {"type": "skill", "name": "go_home", "args": {}},
                ],
            },
        }
    )

    assert task.root.type == "repeat"
    assert task.root.count == 100
    assert count_task_nodes(task.root) == 3


def test_parse_factory_task_accepts_place_relative_then_repick() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "apple-drop-and-repick",
            "mode": "supervised_hardware",
            "root": {
                "type": "sequence",
                "children": [
                    {"type": "skill", "name": "pick_object", "args": {"object": "apple"}},
                    {
                        "type": "skill",
                        "name": "place_relative",
                        "args": {"object": "apple", "reference": "current_pose", "delta": {"z": 0.10}},
                    },
                    {"type": "skill", "name": "verify_scene", "args": {"object": "apple"}},
                    {"type": "skill", "name": "pick_object", "args": {"object": "apple"}},
                ],
            },
        }
    )

    assert task.root.children[1].name == "place_relative"
    assert task.root.children[1].args["delta"]["z"] == pytest.approx(0.10)


def test_parse_factory_task_rejects_unknown_node_type() -> None:
    with pytest.raises(FactoryTaskError, match="unsupported node type"):
        parse_factory_task(
            {
                "task_type": "factory_task",
                "version": FACTORY_TASK_VERSION,
                "task_id": "bad-node",
                "root": {"type": "dance", "children": []},
            }
        )


def test_parse_factory_task_rejects_repeat_without_count() -> None:
    with pytest.raises(FactoryTaskError, match="repeat requires positive integer count"):
        parse_factory_task(
            {
                "task_type": "factory_task",
                "version": FACTORY_TASK_VERSION,
                "task_id": "bad-repeat",
                "root": {"type": "repeat", "children": []},
            }
        )

def test_compiler_emits_guarded_semantic_ir_with_visible_policy_metadata() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "home-wait",
            "root": {
                "type": "sequence",
                "children": [
                    {"type": "skill", "name": "go_home", "args": {}},
                    {"type": "skill", "name": "wait", "args": {"wait_duration_sec": 1.5}},
                ],
            },
        }
    )

    compiled = TaskCompiler(world_model=WorldModel()).compile(task)

    assert compiled.semantic_ir["intent"] == "sequence"
    assert compiled.semantic_ir["steps"] == [
        {"intent": "go_home"},
        {"intent": "wait", "wait_duration_sec": 1.5},
    ]
    metadata = compiled.semantic_ir["metadata"]
    assert metadata["factory_task"]["task_id"] == "home-wait"
    assert metadata["runtime_plan"]["type"] == "sequence"
    assert metadata["policy_decisions"][0]["decision"] == "allow"
    assert "supervisor validation" in metadata["policy_decisions"][0]["reason"]

def test_compiler_rejects_object_motion_when_world_model_has_no_pose() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "move-to-red-block",
            "root": {
                "type": "skill",
                "name": "move_to_object",
                "args": {"object_ref": "red_block"},
            },
        }
    )

    with pytest.raises(FactoryTaskError, match="world model has no grounded pose"):
        TaskCompiler(world_model=WorldModel()).compile(task)

def test_compiler_rejects_runtime_control_nodes_in_static_review_path() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "fallback-home",
            "root": {
                "type": "fallback",
                "children": [
                    {"type": "skill", "name": "move_named_pose", "args": {"pose_name": "poseA"}},
                    {"type": "skill", "name": "go_home", "args": {}},
                ],
            },
        }
    )

    with pytest.raises(FactoryTaskError, match="fallback.*TaskRuntime"):
        TaskCompiler(world_model=WorldModel()).compile(task)

@pytest.mark.parametrize(
    ("skill_name", "args", "expected_semantic_ir"),
    [
        ("stop", {}, {"intent": "stop"}),
        ("alarm_reset", {}, {"intent": "alarm_reset"}),
        ("get_pose", {"reference_frame": "tool0"}, {"intent": "get_pose", "reference_frame": "tool0"}),
        ("set_speed", {"velocity_scale": 0.03}, {"intent": "set_speed", "velocity_scale": 0.03}),
        (
            "move_relative",
            {"delta": {"x": 0.0, "y": 0.0, "z": -0.05}, "reference_frame": "base_link"},
            {"intent": "move_relative", "delta": {"x": 0.0, "y": 0.0, "z": -0.05}, "reference_frame": "base_link"},
        ),
        (
            "move_cartesian",
            {
                "target_pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.4}},
                "reference_frame": "base_link",
                "keep_current_orientation": True,
            },
            {
                "intent": "absolute_move_ptp",
                "target_pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.4}},
                "reference_frame": "base_link",
                "keep_current_orientation": True,
            },
        ),
        (
            "move_joint",
            {"joint_index": 2, "joint_angle": 15.0, "angular_unit": "deg"},
            {"intent": "move_joint", "joint_index": 2, "joint_angle": 15.0, "angular_unit": "deg"},
        ),
        (
            "move_joint_delta",
            {"joint_index": 3, "delta_angle": -5.0, "angular_unit": "deg"},
            {"intent": "move_joint_delta", "joint_index": 3, "delta_angle": -5.0, "angular_unit": "deg"},
        ),
        (
            "move_joints",
            {"joint_target": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5], "angular_unit": "rad"},
            {"intent": "move_joints", "joint_target": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5], "angular_unit": "rad"},
        ),
        (
            "draw_shape",
            {
                "shape_type": "circle",
                "units": "cm",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "params": {"radius": 5.0},
            },
            {
                "intent": "draw_shape",
                "shape_type": "circle",
                "units": "cm",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "params": {"radius": 5.0},
            },
        ),
        (
            "draw_text",
            {
                "text": "HELLO",
                "units": "cm",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "font": {"type": "single_stroke_builtin", "height": 2.0},
            },
            {
                "intent": "draw_text",
                "text": "HELLO",
                "units": "cm",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "font": {"type": "single_stroke_builtin", "height": 2.0},
            },
        ),
    ],
)
def test_compiler_maps_advertised_deterministic_skills_to_semantic_ir(
    skill_name: str,
    args: dict,
    expected_semantic_ir: dict,
) -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": f"compile-{skill_name}",
            "root": {"type": "skill", "name": skill_name, "args": args},
        }
    )

    compiled = TaskCompiler(world_model=WorldModel()).compile(task)

    actual = dict(compiled.semantic_ir)
    actual.pop("metadata", None)
    assert actual == expected_semantic_ir

def test_runtime_retry_exhaustion_selects_fallback_at_runtime() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "fallback-after-place-failure",
            "root": {
                "type": "fallback",
                "children": [
                    {
                        "type": "retry",
                        "count": 2,
                        "children": [
                            {"type": "skill", "name": "place_object", "args": {"object": "apple"}}
                        ],
                    },
                    {"type": "skill", "name": "go_home", "args": {}},
                ],
            },
        }
    )
    calls: list[str] = []

    def executor(name: str, args: dict) -> RuntimeStepResult:
        calls.append(name)
        return RuntimeStepResult(success=name == "go_home")

    report = TaskRuntime().run(task, executor)

    assert report.success is True
    assert calls == ["place_object", "place_object", "go_home"]
    assert report.fallback_count == 1
    assert report.attempts_by_skill["place_object"] == 2
    assert any(decision["decision"] == "retry_exhausted" for decision in report.policy_decisions)
    assert any(decision["decision"] == "fallback_selected" for decision in report.policy_decisions)

def test_runtime_replan_handler_replaces_failed_plan_once() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "needs-replan",
            "root": {"type": "skill", "name": "move_to_region", "args": {"region": "bin_a"}},
        }
    )
    repaired_task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "replanned-home",
            "root": {"type": "skill", "name": "go_home", "args": {}},
        }
    )
    calls: list[str] = []

    def executor(name: str, args: dict) -> RuntimeStepResult:
        calls.append(name)
        if name == "move_to_region":
            return RuntimeStepResult(success=False, requests_replan=True, reason="object moved")
        return RuntimeStepResult(success=True)

    report = TaskRuntime(replan_handler=lambda _report: repaired_task).run(task, executor)

    assert report.success is True
    assert report.replan_count == 1
    assert calls == ["move_to_region", "go_home"]
    assert any(decision["decision"] == "replan" for decision in report.policy_decisions)

def test_runtime_uses_factory_task_replan_policy_max_replans() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "needs-two-replans",
            "replan_policy": {"max_replans": 2},
            "root": {"type": "skill", "name": "move_to_region", "args": {"region": "bin_a"}},
        }
    )
    first_repair = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "first-repair",
            "root": {"type": "skill", "name": "move_to_region", "args": {"region": "bin_b"}},
        }
    )
    second_repair = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "second-repair",
            "root": {"type": "skill", "name": "go_home", "args": {}},
        }
    )
    repairs = [first_repair, second_repair]
    calls: list[str] = []

    def executor(name: str, args: dict) -> RuntimeStepResult:
        calls.append(name)
        if name == "move_to_region":
            return RuntimeStepResult(success=False, requests_replan=True, reason="world changed")
        return RuntimeStepResult(success=True)

    def replan_handler(_report):
        return repairs.pop(0)

    report = TaskRuntime(replan_handler=replan_handler).run(task, executor)

    assert report.success is True
    assert report.replan_count == 2
    assert calls == ["move_to_region", "move_to_region", "go_home"]
    assert [decision["decision"] for decision in report.policy_decisions].count("replan") == 2


def test_compiler_maps_pick_object_to_approach_and_gripper_close_sequence() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "pick-white",
            "root": {
                "type": "skill",
                "name": "pick_object",
                "args": {"object": "white_workpiece"},
            },
        }
    )
    wm = WorldModel(
        objects={
            "white_workpiece": {
                "pose": {
                    "position": {"x": 0.3, "y": 0.1, "z": 0.4},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                }
            }
        }
    )

    compiled = TaskCompiler(world_model=wm).compile(task)

    steps = compiled.semantic_ir.get("steps", [])
    intents = [s["intent"] for s in steps]
    assert "absolute_move_ptp" in intents, "pick_object must include approach motion"
    assert "io_set" in intents, "pick_object must close gripper"
    gripper_step = next(s for s in steps if s["intent"] == "io_set")
    assert gripper_step["io_value"] == 1, "pick_object must close gripper (value=1)"


def test_compiler_maps_place_object_to_gripper_open() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "place-conveyor",
            "root": {
                "type": "skill",
                "name": "place_object",
                "args": {"object": "white_workpiece", "destination": "conveyor"},
            },
        }
    )

    compiled = TaskCompiler(world_model=WorldModel()).compile(task)

    assert compiled.semantic_ir["intent"] == "io_set"
    assert compiled.semantic_ir["io_value"] == 0, "place_object must open gripper (value=0)"


def test_compiler_maps_verify_scene_to_get_pose() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "verify-held",
            "root": {
                "type": "skill",
                "name": "verify_scene",
                "args": {"object": "white_workpiece", "expected": "held"},
            },
        }
    )

    compiled = TaskCompiler(world_model=WorldModel()).compile(task)

    assert compiled.semantic_ir["intent"] == "get_pose"
    # The runtime_plan captures the original skill, confirming it was a verify_scene node.
    assert compiled.semantic_ir["metadata"]["runtime_plan"]["name"] == "verify_scene"


def test_compiler_maps_place_relative_to_move_relative() -> None:
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "place-rel",
            "root": {
                "type": "skill",
                "name": "place_relative",
                "args": {
                    "delta": {"x": 0.0, "y": 0.0, "z": 0.10},
                    "reference_frame": "base_link",
                },
            },
        }
    )

    compiled = TaskCompiler(world_model=WorldModel()).compile(task)

    assert compiled.semantic_ir["intent"] == "move_relative"
    assert compiled.semantic_ir["delta"]["z"] == pytest.approx(0.10)


def test_compiler_pick_object_fails_closed_when_world_model_has_no_pose() -> None:
    """A pick must not compile to gripper-only IR when object motion cannot be grounded."""
    task = parse_factory_task(
        {
            "task_type": "factory_task",
            "version": FACTORY_TASK_VERSION,
            "task_id": "pick-unknown",
            "root": {
                "type": "skill",
                "name": "pick_object",
                "args": {"object": "mystery_object"},
            },
        }
    )

    with pytest.raises(FactoryTaskError, match="world model has no grounded pose"):
        TaskCompiler(world_model=WorldModel()).compile(task)
