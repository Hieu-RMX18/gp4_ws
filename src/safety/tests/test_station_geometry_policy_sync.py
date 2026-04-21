from __future__ import annotations

from pathlib import Path
import math
import struct
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
    return _mat_mul(_mat_mul(rz, ry), rx)


def _load_stl_vertices(path: Path) -> list[tuple[float, float, float]]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError("STL file too small")

    triangle_count = struct.unpack("<I", data[80:84])[0]
    expected_size = 84 + triangle_count * 50
    vertices: list[tuple[float, float, float]] = []

    if expected_size == len(data):
        offset = 84
        for _ in range(triangle_count):
            offset += 12  # normal
            for _ in range(3):
                vertices.append(struct.unpack("<fff", data[offset:offset + 12]))
                offset += 12
            offset += 2  # attribute byte count
        return vertices

    for line in data.decode("utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))

    if not vertices:
        raise ValueError("Failed to parse STL vertices")
    return vertices


def _station_mesh_bounds_in_base(xacro_root: ET.Element, vertices: list[tuple[float, float, float]]):
    station_link = xacro_root.find(".//link[@name='station_link']")
    assert station_link is not None
    visual_origin = station_link.find("./visual/origin")
    visual_mesh = station_link.find("./visual/geometry/mesh")
    station_to_robot_origin = xacro_root.find(".//joint[@name='station_to_robot']/origin")
    assert visual_origin is not None
    assert visual_mesh is not None
    assert station_to_robot_origin is not None

    mesh_scale = _parse_vec3(visual_mesh.attrib.get("scale"))
    p_sm = _parse_vec3(visual_origin.attrib.get("xyz"))
    r_sm = _mat_from_rpy(*_parse_vec3(visual_origin.attrib.get("rpy")))
    p_sb = _parse_vec3(station_to_robot_origin.attrib.get("xyz"))
    r_sb = _mat_from_rpy(*_parse_vec3(station_to_robot_origin.attrib.get("rpy")))
    r_bs = _mat_transpose(r_sb)

    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")

    for vx, vy, vz in vertices:
        scaled = (vx * mesh_scale[0], vy * mesh_scale[1], vz * mesh_scale[2])
        in_station = _mat_vec_mul(r_sm, scaled)
        translated = (
            in_station[0] + p_sm[0] - p_sb[0],
            in_station[1] + p_sm[1] - p_sb[1],
            in_station[2] + p_sm[2] - p_sb[2],
        )
        in_base = _mat_vec_mul(r_bs, translated)

        min_x = min(min_x, in_base[0])
        min_y = min(min_y, in_base[1])
        min_z = min(min_z, in_base[2])
        max_x = max(max_x, in_base[0])
        max_y = max(max_y, in_base[1])
        max_z = max(max_z, in_base[2])

    return {
        "x_min": min_x,
        "x_max": max_x,
        "y_min": min_y,
        "y_max": max_y,
        "z_min": min_z,
        "z_max": max_z,
    }


def test_workspace_and_keepout_policy_stays_conservative_against_current_mesh():
    root = Path(__file__).resolve().parents[2]
    xacro_path = root / "gp4_station" / "urdf" / "gp4_on_station.urdf.xacro"
    mesh_path = root / "gp4_station" / "meshes" / "station3.stl"
    safety_rules_path = root / "safety" / "config" / "safety_rules.yaml"

    xacro_root = ET.fromstring(xacro_path.read_text())
    vertices = _load_stl_vertices(mesh_path)
    bounds = _station_mesh_bounds_in_base(xacro_root, vertices)
    safety_rules = _load_yaml(safety_rules_path)

    ws = safety_rules["workspace_bounds"]
    margin = 0.03

    # Conservative envelope: keep inside near wall + margin and do not exceed fixed caps.
    assert ws["x_min"] >= bounds["x_min"] + margin - 0.005
    assert ws["x_max"] <= 0.45
    assert ws["y_min"] >= bounds["y_min"] + margin - 0.005
    assert ws["y_max"] <= 0.52
    assert ws["z_min"] >= 0.23
    assert ws["z_max"] <= 0.52

    zones = {zone["name"]: zone for zone in safety_rules["forbidden_zones"]}
    assert "front_wall_guard" in zones
    assert "right_wall_guard" in zones

    # Keepout wall alignment must track calibrated station orientation.
    assert zones["front_wall_guard"]["y"] == pytest.approx(bounds["y_min"], abs=0.002)
    assert zones["right_wall_guard"]["x"] == pytest.approx(bounds["x_min"], abs=0.002)
