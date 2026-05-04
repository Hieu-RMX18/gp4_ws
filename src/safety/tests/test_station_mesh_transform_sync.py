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
    values = [float(v) for v in raw.split()]
    if len(values) != 3:
        raise ValueError(f"Expected 3 values, got {raw!r}")
    return (values[0], values[1], values[2])


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


def _mat_vec_mul(m, v):
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _mat_from_rpy(roll: float, pitch: float, yaw: float):
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    # URDF RPY convention: R = Rz(yaw) * Ry(pitch) * Rx(roll) for column vectors.
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


def test_station_scene_pose_stays_in_sync_with_xacro():
    root_path = Path(__file__).resolve().parents[2]
    xacro_path = root_path / "gp4_station" / "urdf" / "gp4_on_station.urdf.xacro"
    scene_path = root_path / "gp4_bringup" / "config" / "scene_objects.yaml"

    xacro_root = ET.fromstring(xacro_path.read_text())
    scene = _load_yaml(scene_path)
    station_mesh = scene["collision_objects"]["station_mesh"]

    station_link = xacro_root.find(".//link[@name='station_link']")
    assert station_link is not None, "station_link not found in xacro"

    visual_origin = station_link.find("./visual/origin")
    visual_mesh = station_link.find("./visual/geometry/mesh")
    collision_origin = station_link.find("./collision/origin")
    collision_mesh = station_link.find("./collision/geometry/mesh")

    assert visual_origin is not None and visual_mesh is not None
    assert collision_origin is not None and collision_mesh is not None

    assert visual_mesh.attrib.get("filename") == collision_mesh.attrib.get("filename")
    assert visual_mesh.attrib.get("scale") == collision_mesh.attrib.get("scale")
    assert visual_origin.attrib.get("rpy") == collision_origin.attrib.get("rpy")
    assert visual_origin.attrib.get("xyz") == collision_origin.attrib.get("xyz")

    assert visual_mesh.attrib.get("filename") == station_mesh["resource"]
    assert [
        float(v) for v in visual_mesh.attrib.get("scale", "").split()
    ] == pytest.approx(station_mesh["scale"])

    mesh_xyz = _parse_vec3(visual_origin.attrib.get("xyz"))
    mesh_rpy = _parse_vec3(visual_origin.attrib.get("rpy"))
    r_sm = _mat_from_rpy(*mesh_rpy)

    station_to_robot = xacro_root.find(".//joint[@name='station_to_robot']")
    assert station_to_robot is not None, "station_to_robot joint not found in xacro"
    station_to_robot_origin = station_to_robot.find("./origin")
    assert station_to_robot_origin is not None
    p_sb = _parse_vec3(station_to_robot_origin.attrib.get("xyz"))
    r_sb = _mat_from_rpy(*_parse_vec3(station_to_robot_origin.attrib.get("rpy")))

    # scene_objects uses base_link frame: T_bm = inv(T_sb) * T_sm
    r_bs = _mat_transpose(r_sb)
    expected_orientation = _quat_from_matrix(_mat_mul(r_bs, r_sm))
    expected_position = _mat_vec_mul(
        r_bs,
        (mesh_xyz[0] - p_sb[0], mesh_xyz[1] - p_sb[1], mesh_xyz[2] - p_sb[2]),
    )

    scene_pose = station_mesh["pose"]
    actual_position = tuple(float(v) for v in scene_pose["position"])
    actual_orientation = tuple(float(v) for v in scene_pose["orientation"])

    assert actual_position == pytest.approx(expected_position, abs=1e-6)
    assert _quat_equivalent(actual_orientation, expected_orientation, tol=1e-4)
