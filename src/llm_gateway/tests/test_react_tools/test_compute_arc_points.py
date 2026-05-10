"""Tests for compute_arc_points ReAct tool."""

import math

import pytest

from llm_gateway.react_planner import ComputeArcPointsTool


@pytest.fixture
def tool():
    return ComputeArcPointsTool()


def test_90_deg_arc_xy_plane(tool):
    args = {
        "center": {"x": 0.0, "y": 0.0, "z": 0.2},
        "radius_m": 0.05,
        "start_angle_rad": 0.0,
        "sweep_angle_rad": math.radians(90),
        "plane_normal": {"x": 0.0, "y": 0.0, "z": 1.0},
    }
    result = tool.invoke(args, None)
    assert result.ok is True
    payload = result.payload
    start = payload["start_pose"]["pose"]["position"]
    aux = payload["auxiliary_pose"]["pose"]["position"]
    end = payload["target_pose"]["pose"]["position"]
    # Start at 0 deg -> (0.05, 0, 0.2)
    assert start["x"] == pytest.approx(0.05, abs=1e-6)
    assert start["y"] == pytest.approx(0.0, abs=1e-6)
    assert start["z"] == pytest.approx(0.2, abs=1e-6)
    # Auxiliary at 45 deg -> (0.05*cos45, 0.05*sin45)
    assert aux["x"] == pytest.approx(0.05 * math.cos(math.radians(45)), abs=1e-6)
    assert aux["y"] == pytest.approx(0.05 * math.sin(math.radians(45)), abs=1e-6)
    # End at 90 deg -> (0, 0.05, 0.2)
    assert end["x"] == pytest.approx(0.0, abs=1e-6)
    assert end["y"] == pytest.approx(0.05, abs=1e-6)
    assert end["z"] == pytest.approx(0.2, abs=1e-6)


def test_zero_sweep_rejected(tool):
    args = {
        "center": {"x": 0.0, "y": 0.0, "z": 0.2},
        "radius_m": 0.05,
        "start_angle_rad": 0.0,
        "sweep_angle_rad": 0.0,
        "plane_normal": {"x": 0.0, "y": 0.0, "z": 1.0},
    }
    result = tool.invoke(args, None)
    assert result.ok is False
    assert "sweep_angle_rad must be non-zero" in result.error


def test_negative_radius_rejected(tool):
    args = {
        "center": {"x": 0.0, "y": 0.0, "z": 0.2},
        "radius_m": -0.01,
        "start_angle_rad": 0.0,
        "sweep_angle_rad": math.radians(90),
        "plane_normal": {"x": 0.0, "y": 0.0, "z": 1.0},
    }
    result = tool.invoke(args, None)
    assert result.ok is False
    assert "radius_m must be > 0" in result.error


def test_yz_plane_arc(tool):
    args = {
        "center": {"x": 0.0, "y": 0.0, "z": 0.0},
        "radius_m": 0.05,
        "start_angle_rad": 0.0,
        "sweep_angle_rad": math.radians(90),
        "plane_normal": {"x": 0.0, "y": 1.0, "z": 0.0},
    }
    result = tool.invoke(args, None)
    assert result.ok is True
    start = result.payload["start_pose"]["pose"]["position"]
    end = result.payload["target_pose"]["pose"]["position"]
    # Start at 0 deg in XZ plane -> (0.05, 0, 0)
    assert start["x"] == pytest.approx(0.05, abs=1e-6)
    assert start["y"] == pytest.approx(0.0, abs=1e-6)
    assert start["z"] == pytest.approx(0.0, abs=1e-6)
    # End at 90 deg -> (0, 0, -0.05) since normal is +Y (right-handed)
    assert end["x"] == pytest.approx(0.0, abs=1e-6)
    assert end["z"] == pytest.approx(-0.05, abs=1e-6)
