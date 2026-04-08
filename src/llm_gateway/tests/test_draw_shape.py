"""Tests for draw_shape macro routing and geometry."""

import math
from pathlib import Path

import pytest


def _macro_policy_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


def _router(runtime_mode: str):
    from llm_gateway.intent_router import IntentRouter

    return IntentRouter(macro_policy_path=_macro_policy_path(), runtime_mode=runtime_mode)


def _draw_payload(shape: str = "square", **overrides) -> dict:
    base = {
        "intent": "draw_shape",
        "shape": shape,
        "plane": "xy",
        "reference_frame": "base_link",
        "size_m": 0.05,
        "start_pose": {
            "position": {"x": 0.30, "y": 0.00, "z": 0.30},
        },
    }
    base.update(overrides)
    return base


def _distance(p1: dict, p2: dict) -> float:
    return math.sqrt(
        (p2["x"] - p1["x"]) ** 2
        + (p2["y"] - p1["y"]) ** 2
        + (p2["z"] - p1["z"]) ** 2
    )


def _all_positions(result) -> list[dict]:
    positions = []
    for command in result.commands:
        if command["primitive_type"] == "PTP":
            positions.append(command["target_pose"]["position"])
        elif command["primitive_type"] == "CARTESIAN_PATH":
            positions.extend(waypoint["position"] for waypoint in command["waypoints"])
    return positions


def _all_orientations(result) -> list[dict]:
    orientations = []
    for command in result.commands:
        if command["primitive_type"] == "PTP":
            orientations.append(command["target_pose"]["orientation"])
        elif command["primitive_type"] == "CARTESIAN_PATH":
            orientations.extend(waypoint["orientation"] for waypoint in command["waypoints"])
    return orientations


def _cartesian_command(result) -> dict:
    assert [command["primitive_type"] for command in result.commands] == ["PTP", "CARTESIAN_PATH"]
    return result.commands[1]


def test_macro_policy_declares_draw_shape_sim_only():
    from llm_gateway.intent_router import load_macro_policy

    policy = load_macro_policy(_macro_policy_path())
    draw_shape = policy["macros"]["draw_shape"]

    assert draw_shape["availability"] == "sim_only"
    assert draw_shape["requires_current_pose"] is False
    assert draw_shape["supported_frames"] == ["base_link"]
    assert draw_shape["supported_planes"] == ["xy"]
    assert set(draw_shape["supported_shapes"]) == {
        "square",
        "triangle",
        "circle",
        "polygon",
        "rectangle",
        "arc",
        "polyline",
    }


def test_rejects_draw_shape_in_hardware_mode():
    router = _router(runtime_mode="hardware")

    with pytest.raises(ValueError, match="sim-only"):
        router.route(_draw_payload())


def test_accepts_draw_shape_in_sim_mode():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload())

    assert result.route_type == "sequence"
    assert result.metadata["macro_name"] == "draw_shape"
    assert result.metadata["requires_current_pose"] is False


def test_rejects_unsupported_shape():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="unsupported shape"):
        router.route(_draw_payload(shape="star"))


def test_rejects_unsupported_plane():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="unsupported plane"):
        router.route(_draw_payload(plane="xz"))


def test_rejects_unsupported_frame():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="unsupported reference_frame"):
        router.route(_draw_payload(reference_frame="tool0"))


def test_rejects_missing_start_pose_for_square():
    router = _router(runtime_mode="sim")
    payload = _draw_payload("square")
    del payload["start_pose"]

    with pytest.raises(ValueError, match="start_pose"):
        router.route(payload)


def test_rejects_zero_size_m_for_square():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="size_m"):
        router.route(_draw_payload("square", size_m=0.0))


def test_square_sequence_structure():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("square"))

    assert [command["primitive_type"] for command in result.commands] == ["PTP", "CARTESIAN_PATH"]


def test_square_closes_back_to_start():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("square"))

    positions = _all_positions(result)
    assert positions[0] == positions[-1]


def test_square_edge_lengths_match_size_m():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("square", size_m=0.05))
    positions = _all_positions(result)
    edges = [_distance(positions[index], positions[index + 1]) for index in range(len(positions) - 1)]

    for edge in edges:
        assert math.isclose(edge, 0.05, abs_tol=1e-9)


def test_triangle_sequence_structure():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("triangle"))

    assert [command["primitive_type"] for command in result.commands] == ["PTP", "CARTESIAN_PATH"]


def test_triangle_is_equilateral():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("triangle", size_m=0.06))
    positions = _all_positions(result)
    edges = [_distance(positions[index], positions[index + 1]) for index in range(len(positions) - 1)]

    assert len(edges) == 3
    assert math.isclose(edges[0], edges[1], abs_tol=1e-9)
    assert math.isclose(edges[1], edges[2], abs_tol=1e-9)


def test_circle_default_segments_use_single_cartesian_path():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("circle"))

    assert [command["primitive_type"] for command in result.commands] == ["PTP", "CARTESIAN_PATH"]
    assert len(_cartesian_command(result)["waypoints"]) == 32


def test_circle_custom_segments():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("circle", segments=16))

    assert len(_cartesian_command(result)["waypoints"]) == 16


def test_circle_rejects_too_few_segments():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="segments must be >= 8"):
        router.route(_draw_payload("circle", segments=4))


def test_circle_waypoints_equidistant_from_center():
    router = _router(runtime_mode="sim")
    size_m = 0.06
    radius = size_m / 2.0

    result = router.route(_draw_payload("circle", size_m=size_m))
    positions = _all_positions(result)
    center_x = 0.30 + radius
    center_y = 0.0

    for position in positions:
        distance = math.sqrt((position["x"] - center_x) ** 2 + (position["y"] - center_y) ** 2)
        assert math.isclose(distance, radius, abs_tol=1e-9)


def test_polygon_default_sides_is_6():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("polygon"))

    assert len(_cartesian_command(result)["waypoints"]) == 6


def test_polygon_rejects_out_of_range_sides():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="polygon sides must be in"):
        router.route(_draw_payload("polygon", sides=2))

    with pytest.raises(ValueError, match="polygon sides must be in"):
        router.route(_draw_payload("polygon", sides=13))


def test_polygon_vertices_are_equidistant_from_center():
    router = _router(runtime_mode="sim")
    size_m = 0.08
    radius = size_m / 2.0

    result = router.route(_draw_payload("polygon", sides=6, size_m=size_m))
    positions = _all_positions(result)
    center_x = 0.30 + radius
    center_y = 0.0

    for position in positions:
        distance = math.sqrt((position["x"] - center_x) ** 2 + (position["y"] - center_y) ** 2)
        assert math.isclose(distance, radius, abs_tol=1e-9)


def test_rectangle_requires_width_and_height():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="width_m"):
        router.route(_draw_payload("rectangle", size_m=None, width_m=0.0, height_m=0.04))

    with pytest.raises(ValueError, match="height_m"):
        router.route(_draw_payload("rectangle", size_m=None, width_m=0.05, height_m=0.0))


def test_rectangle_dimensions_and_closure():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("rectangle", width_m=0.05, height_m=0.08))
    positions = _all_positions(result)

    assert [command["primitive_type"] for command in result.commands] == ["PTP", "CARTESIAN_PATH"]
    assert positions[0] == positions[-1]
    assert math.isclose(_distance(positions[0], positions[1]), 0.05, abs_tol=1e-9)
    assert math.isclose(_distance(positions[1], positions[2]), 0.08, abs_tol=1e-9)


def test_arc_default_sweep_is_open_path():
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload("arc", radius_m=0.03, sweep_deg=180))
    positions = _all_positions(result)

    assert [command["primitive_type"] for command in result.commands] == ["PTP", "CARTESIAN_PATH"]
    assert positions[0] != positions[-1]
    assert math.isclose(positions[0]["x"], 0.30, abs_tol=1e-9)
    assert math.isclose(positions[-1]["x"], 0.36, abs_tol=1e-9)


def test_arc_points_stay_on_radius():
    router = _router(runtime_mode="sim")
    radius_m = 0.04

    result = router.route(_draw_payload("arc", radius_m=radius_m, sweep_deg=90))
    positions = _all_positions(result)
    center_x = 0.30 + radius_m
    center_y = 0.0

    for position in positions:
        distance = math.sqrt((position["x"] - center_x) ** 2 + (position["y"] - center_y) ** 2)
        assert math.isclose(distance, radius_m, abs_tol=1e-9)


def test_polyline_requires_at_least_two_points():
    router = _router(runtime_mode="sim")

    with pytest.raises(ValueError, match="at least 2 points"):
        router.route(_draw_payload("polyline", size_m=None, start_pose=None, points=[{"x": 0.3, "y": 0.0, "z": 0.3}]))


def test_polyline_preserves_ordering():
    router = _router(runtime_mode="sim")
    points = [
        {"x": 0.30, "y": 0.00, "z": 0.30},
        {"x": 0.32, "y": 0.01, "z": 0.30},
        {"x": 0.34, "y": 0.03, "z": 0.30},
    ]

    result = router.route(_draw_payload("polyline", size_m=None, start_pose=None, points=points))

    assert [command["primitive_type"] for command in result.commands] == ["PTP", "CARTESIAN_PATH"]
    assert _all_positions(result) == points


@pytest.mark.parametrize("shape,overrides", [
    ("square", {}),
    ("triangle", {}),
    ("circle", {}),
    ("polygon", {"sides": 5}),
    ("rectangle", {"width_m": 0.05, "height_m": 0.08}),
    ("arc", {"radius_m": 0.03, "sweep_deg": 180}),
    ("polyline", {"size_m": None, "start_pose": None, "points": [
        {"x": 0.30, "y": 0.00, "z": 0.30},
        {"x": 0.33, "y": 0.02, "z": 0.30},
    ]}),
])
def test_all_shapes_use_tool_down_orientation(shape: str, overrides: dict):
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload(shape, **overrides))

    for orientation in _all_orientations(result):
        assert orientation == {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}


@pytest.mark.parametrize("shape,overrides", [
    ("square", {}),
    ("triangle", {}),
    ("circle", {}),
    ("polygon", {"sides": 5}),
    ("rectangle", {"width_m": 0.05, "height_m": 0.08}),
    ("arc", {"radius_m": 0.03, "sweep_deg": 180}),
    ("polyline", {"size_m": None, "start_pose": None, "points": [
        {"x": 0.30, "y": 0.00, "z": 0.30},
        {"x": 0.33, "y": 0.02, "z": 0.30},
    ]}),
])
def test_all_shapes_carry_reference_frame(shape: str, overrides: dict):
    router = _router(runtime_mode="sim")

    result = router.route(_draw_payload(shape, **overrides))

    for command in result.commands:
        assert command["reference_frame"] == "base_link"
