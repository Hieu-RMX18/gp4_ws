import pytest
from safety.workspace_guard import WorkspaceGuard

def test_pose_inside_bounds(safety_rules, valid_pose):
    guard = WorkspaceGuard(safety_rules)
    valid, reason = guard.check_pose(valid_pose)
    assert valid is True
    assert reason == ""

def test_pose_hits_table_forbidden_zone(safety_rules, table_collision_pose):
    guard = WorkspaceGuard(safety_rules)
    valid, reason = guard.check_pose(table_collision_pose)
    assert valid is False
    assert "collision with test_box" in reason

def test_pose_below_z_min_bounds(safety_rules, below_z_min_pose):
    guard = WorkspaceGuard(safety_rules)
    valid, reason = guard.check_pose(below_z_min_pose)
    assert valid is False
    assert "Z coordinate" in reason and "out of bounds" in reason
