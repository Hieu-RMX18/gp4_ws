"""Tests for DRAW_SHAPE routing and deterministic geometry compilation."""

import math
from pathlib import Path

import pytest


def _macro_policy_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "macro_policy.yaml")


def _router(runtime_mode: str = "hardware"):
    from llm_gateway.intent_router import IntentRouter

    return IntentRouter(macro_policy_path=_macro_policy_path(), runtime_mode=runtime_mode)


def _draw_payload(shape_type: str = "square", **overrides) -> dict:
    base = {
        "intent": "draw_shape",
        "shape_type": shape_type,
        "units": "m",
        "frame_id": "base_link",
        "workplane": {
            "mode": "base",
            "origin": {
                "position": {"x": 0.30, "y": 0.00, "z": 0.30},
            },
        },
        "params": {
            "side_m": 0.05,
        },
    }
    base.update(overrides)
    return base


def _all_draw_positions(result) -> list[dict]:
    positions = []
    for command in result.commands:
        primitive = command["primitive_type"]
        if primitive in {"PTP", "LIN"}:
            positions.append(command["target_pose"]["position"])
        elif primitive == "CARTESIAN_PATH":
            positions.extend(waypoint["position"] for waypoint in command["waypoints"])
    return positions


def _collect_draw_line_points(result) -> list[dict]:
    points = []
    for command in result.commands:
        if command["primitive_type"] == "LIN":
            points.append(command["target_pose"]["position"])
        elif command["primitive_type"] == "CARTESIAN_PATH":
            points.extend(waypoint["position"] for waypoint in command["waypoints"])
    return points


def _distance(p1: dict, p2: dict) -> float:
    return math.sqrt(
        (p2["x"] - p1["x"]) ** 2
        + (p2["y"] - p1["y"]) ** 2
        + (p2["z"] - p1["z"]) ** 2
    )


def test_macro_policy_declares_draw_shape_contract():
    from llm_gateway.intent_router import load_macro_policy

    policy = load_macro_policy(_macro_policy_path())
    draw_shape = policy["macros"]["draw_shape"]

    assert draw_shape["availability"] == "all"
    assert "base" in draw_shape["supported_workplane_modes"]
    assert "tool" in draw_shape["supported_workplane_modes"]
    assert "explicit_pose" in draw_shape["supported_workplane_modes"]
    assert draw_shape["default_units"] == "m"


def test_accepts_draw_shape_in_hardware_mode():
    router = _router(runtime_mode="hardware")

    result = router.route(_draw_payload("square"))

    assert result.route_type == "sequence"
    assert result.metadata["macro_name"] == "draw_shape"
    assert result.metadata["shape_type"] == "square"
    assert result.metadata["summary"]["draw_stroke_count"] >= 1


def test_rejects_unsupported_shape_type():
    router = _router()

    with pytest.raises(ValueError, match="unsupported_shape_type"):
        router.route(_draw_payload(shape_type="star"))


def test_rejects_invalid_polygon_sides():
    router = _router()

    with pytest.raises(ValueError, match="invalid_polygon_sides"):
        router.route(
            _draw_payload(
                shape_type="polygon",
                params={"n_sides": 2, "radius_m": 0.03},
            )
        )


def test_square_draws_closed_path_with_expected_edge_lengths():
    router = _router()

    result = router.route(_draw_payload("square", params={"side_m": 0.05}))
    points = _collect_draw_line_points(result)

    # Last retract LIN is above draw plane, so use first five in-plane points.
    in_plane = [point for point in points if math.isclose(point["z"], 0.30, abs_tol=1e-9)]
    assert len(in_plane) >= 5
    assert in_plane[0] == in_plane[-1]

    edges = [_distance(in_plane[index], in_plane[index + 1]) for index in range(len(in_plane) - 1)]
    # Keep first 4 polygon edges.
    for edge in edges[:4]:
        assert math.isclose(edge, 0.05, abs_tol=1e-6)


def test_circle_points_stay_on_declared_radius():
    router = _router()
    radius_m = 0.03

    result = router.route(
        _draw_payload(
            "circle",
            params={"radius_m": radius_m},
        )
    )

    draw_points = [
        point for point in _collect_draw_line_points(result) if math.isclose(point["z"], 0.30, abs_tol=1e-9)
    ]
    center_x = 0.30 + radius_m
    center_y = 0.00

    for point in draw_points:
        distance = math.sqrt((point["x"] - center_x) ** 2 + (point["y"] - center_y) ** 2)
        assert math.isclose(distance, radius_m, abs_tol=2e-3)


def test_arc_uses_non_closed_path_with_requested_sweep():
    router = _router()

    result = router.route(
        _draw_payload(
            "arc",
            params={"radius_m": 0.03, "sweep_deg": 180.0},
        )
    )
    draw_points = [
        point for point in _collect_draw_line_points(result) if math.isclose(point["z"], 0.30, abs_tol=1e-9)
    ]

    assert draw_points[0] != draw_points[-1]
    assert math.isclose(draw_points[0]["x"], 0.30, abs_tol=1e-6)
    assert math.isclose(draw_points[-1]["x"], 0.36, abs_tol=1e-3)


def test_polyline_preserves_point_ordering():
    router = _router()
    result = router.route(
        _draw_payload(
            "polyline",
            params={
                "points": [
                    {"x": 0.0, "y": 0.0},
                    {"x": 0.02, "y": 0.01},
                    {"x": 0.05, "y": 0.03},
                ],
            },
        )
    )

    draw_points = [
        point for point in _collect_draw_line_points(result) if math.isclose(point["z"], 0.30, abs_tol=1e-9)
    ]
    assert draw_points[0]["x"] == pytest.approx(0.30)
    assert draw_points[1]["x"] == pytest.approx(0.32)
    assert draw_points[2]["x"] == pytest.approx(0.35)


def test_tool_workplane_requires_origin_or_start_pose():
    router = _router()

    with pytest.raises(ValueError, match="missing_workplane"):
        router.route(
            {
                "intent": "draw_shape",
                "shape_type": "square",
                "units": "m",
                "frame_id": "base_link",
                "workplane": {"mode": "tool"},
                "params": {"side_m": 0.05},
            }
        )


def test_plan_only_sets_require_approval_on_all_segments():
    router = _router(runtime_mode="sim")

    result = router.route(
        _draw_payload(
            "rectangle",
            execution_mode="plan_only",
            params={"width_m": 0.05, "height_m": 0.08},
        )
    )

    for command in result.commands:
        assert command["require_approval"] is True
        assert command["plan_only"] is True


def test_dense_circle_is_chunked_into_multiple_segments():
    router = _router()

    result = router.route(
        _draw_payload(
            "circle",
            params={"radius_m": 0.10},
            stroke={"drawing_speed_scale": 0.10, "travel_speed_scale": 0.15},
        )
    )

    cartesian_commands = [command for command in result.commands if command["primitive_type"] == "CARTESIAN_PATH"]
    assert len(cartesian_commands) >= 1
    # The compiler always annotates chunk metadata for cartesian chunks.
    for command in cartesian_commands:
        assert command["chunk_index"] >= 1
        assert command["stroke_index"] >= 1


def test_all_generated_commands_carry_reference_frame():
    router = _router()

    result = router.route(_draw_payload("triangle", params={"side_m": 0.05}))

    for command in result.commands:
        assert command["reference_frame"] == "base_link"
