from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest
import yaml


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text()) or {}


def _parse_vec3(raw: str | None) -> tuple[float, float, float]:
    if not raw:
        return (0.0, 0.0, 0.0)
    values = [_parse_scalar(v) for v in raw.split()]
    if len(values) != 3:
        raise ValueError(f"Expected 3 values, got {raw!r}")
    return (values[0], values[1], values[2])


def _parse_scalar(token: str) -> float:
    token = token.strip()
    if token.startswith("${") and token.endswith("}"):
        expr = token[2:-1].strip()
        if expr == "pi":
            return math.pi
        if expr == "-pi":
            return -math.pi
        if expr == "+pi":
            return math.pi
        raise ValueError(f"Unsupported scalar expression: {token!r}")
    return float(token)


def _mat_mul(a, b):
    return (
        (
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ),
        (
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ),
        (
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ),
    )


def _mat_transpose(m):
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )


def _mat_from_rpy(roll: float, pitch: float, yaw: float):
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return _mat_mul(_mat_mul(rz, ry), rx)


def _quat_from_matrix(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2][1] - m[1][2]) / s
        qy = (m[0][2] - m[2][0]) / s
        qz = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        qw = (m[2][1] - m[1][2]) / s
        qx = 0.25 * s
        qy = (m[0][1] + m[1][0]) / s
        qz = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        qw = (m[0][2] - m[2][0]) / s
        qx = (m[0][1] + m[1][0]) / s
        qy = 0.25 * s
        qz = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        qw = (m[1][0] - m[0][1]) / s
        qx = (m[0][2] + m[2][0]) / s
        qy = (m[1][2] + m[2][1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def _quat_equivalent(actual, expected, tol=1e-6):
    direct = all(math.isclose(a, e, abs_tol=tol) for a, e in zip(actual, expected))
    negated = all(math.isclose(a, -e, abs_tol=tol) for a, e in zip(actual, expected))
    return direct or negated


def test_scene_objects_use_station_mesh_only():
    root = Path(__file__).resolve().parents[2]
    scene_objects_path = root / "gp4_bringup" / "config" / "scene_objects.yaml"
    xacro_path = root / "gp4_station" / "urdf" / "gp4_on_station.urdf.xacro"
    scene = _load_yaml(scene_objects_path)
    objects = scene.get("collision_objects", {})

    assert set(objects.keys()) == {"station_mesh"}
    station_mesh = objects["station_mesh"]
    assert station_mesh.get("type") == "mesh"
    assert station_mesh.get("resource") == "package://gp4_station/meshes/station3.stl"
    assert station_mesh.get("frame_id") == "base_link"

    xacro_root = ET.fromstring(xacro_path.read_text())
    station_link = xacro_root.find(".//link[@name='station_link']")
    assert station_link is not None
    visual_origin = station_link.find("./visual/origin")
    station_to_robot_origin = xacro_root.find(".//joint[@name='station_to_robot']/origin")
    assert visual_origin is not None and station_to_robot_origin is not None

    r_sm = _mat_from_rpy(*_parse_vec3(visual_origin.attrib.get("rpy")))
    r_sb = _mat_from_rpy(*_parse_vec3(station_to_robot_origin.attrib.get("rpy")))
    r_bs = _mat_transpose(r_sb)
    expected_orientation = _quat_from_matrix(_mat_mul(r_bs, r_sm))

    actual_orientation = tuple(float(v) for v in station_mesh.get("pose", {}).get("orientation", []))
    assert len(actual_orientation) == 4
    assert _quat_equivalent(actual_orientation, expected_orientation, tol=1e-4)

    # Cross-check MoveIt embedded station visual transform against scene_objects orientation.
    moveit_xacro_path = root / "gp4_moveit_config" / "config" / "motoman_gp4.urdf.xacro"
    moveit_root = ET.fromstring(moveit_xacro_path.read_text())
    base_to_station_visual = moveit_root.find(".//joint[@name='base_to_station_visual']/origin")
    assert base_to_station_visual is not None
    r_bsvis = _mat_from_rpy(*_parse_vec3(base_to_station_visual.attrib.get("rpy")))
    moveit_orientation = _quat_from_matrix(r_bsvis)
    assert _quat_equivalent(actual_orientation, moveit_orientation, tol=1e-4)


def test_safety_forbidden_zones_defined_for_station_policy():
    safety_rules_path = Path(__file__).resolve().parents[1] / "config" / "safety_rules.yaml"
    safety_rules = _load_yaml(safety_rules_path)
    zones = safety_rules.get("forbidden_zones", [])
    assert isinstance(zones, list)
    zone_names = {zone.get("name") for zone in zones}
    assert {
        "front_wall_guard",
        "right_wall_guard",
        "floor_clearance_guard",
    }.issubset(zone_names)
