from pathlib import Path
import re

import pytest
import yaml


_HEADER_CONSTANT_PATTERN = re.compile(
    r"static constexpr double k([A-Za-z0-9]+)\s*=\s*([-+]?\d*\.?\d+);"
)


def _load_safety_rules():
    yaml_path = Path(__file__).resolve().parents[1] / "config" / "safety_rules.yaml"
    return yaml.safe_load(yaml_path.read_text()) or {}


def _load_move_rel_limits():
    header_path = (
        Path(__file__).resolve().parents[2]
        / "motion_core"
        / "include"
        / "motion_core"
        / "move_rel_validator.hpp"
    )
    return {
        match.group(1): float(match.group(2))
        for match in _HEADER_CONSTANT_PATTERN.finditer(header_path.read_text())
    }


def test_move_rel_limits_match_safety_rules_workspace_and_delta_cap():
    safety_rules = _load_safety_rules()
    move_rel_limits = _load_move_rel_limits()

    workspace_bounds = safety_rules["workspace_bounds"]
    motion_limits = safety_rules["motion_limits"]

    expected_limits = {
        "XMin": workspace_bounds["x_min"],
        "XMax": workspace_bounds["x_max"],
        "YMin": workspace_bounds["y_min"],
        "YMax": workspace_bounds["y_max"],
        "ZMin": workspace_bounds["z_min"],
        "ZMax": workspace_bounds["z_max"],
        "MaxDeltaNorm": motion_limits["max_move_rel_translation"],
    }

    for name, expected_value in expected_limits.items():
        assert name in move_rel_limits, f"Missing MoveRelLimits::{name}."
        assert move_rel_limits[name] == pytest.approx(expected_value)


def test_move_rel_forbidden_zone_constants_match_safety_rules():
    safety_rules = _load_safety_rules()
    move_rel_limits = _load_move_rel_limits()

    constant_prefixes = {
        "table_clearance_guard": "TableClearance",
        "avoid_left_region": "AvoidLeft",
        "wall_region": "Wall",
        "corner_clearance_guard": "CornerGuard",
    }
    field_suffixes = {
        "x": "X",
        "y": "Y",
        "z": "Z",
        "size_x": "SizeX",
        "size_y": "SizeY",
        "size_z": "SizeZ",
    }

    for zone in safety_rules["forbidden_zones"]:
        prefix = constant_prefixes.get(zone["name"])
        if prefix is None:
            continue
        for field_name, suffix in field_suffixes.items():
            constant_name = f"{prefix}{suffix}"
            assert constant_name in move_rel_limits, f"Missing MoveRelLimits::{constant_name}."
            assert move_rel_limits[constant_name] == pytest.approx(zone[field_name])
