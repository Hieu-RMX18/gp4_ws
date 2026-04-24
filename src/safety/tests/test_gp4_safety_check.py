from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'gp4_safety_check.py'
SPEC = importlib.util.spec_from_file_location('gp4_safety_check', SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

JOINT_NAMES = [
    'joint_1_s',
    'joint_2_l',
    'joint_3_u',
    'joint_4_r',
    'joint_5_b',
    'joint_6_t',
]


def _point(*, positions, time_from_start, velocities=None, accelerations=None):
    point = {
        'positions': positions,
        'time_from_start': time_from_start,
    }
    if velocities is not None:
        point['velocities'] = velocities
    if accelerations is not None:
        point['accelerations'] = accelerations
    return point


def _trajectory(points):
    return {
        'joint_names': JOINT_NAMES,
        'points': points,
    }


def test_valid_minimal_trajectory_passes() -> None:
    trajectory = _trajectory(
        [
            _point(positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=0.0),
            _point(positions=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1], time_from_start=1.0),
        ]
    )

    report = MODULE.validate_trajectory(trajectory)

    assert report['ok'] is True
    assert report['failureReasons'] == []
    assert report['summary']['pointCount'] == 2


def test_point_count_overflow_fails() -> None:
    points = []
    for index in range(201):
        value = min(index * 0.001, 0.2)
        positions = [value, 0.0, 0.0, 0.0, 0.0, 0.0]
        points.append(_point(positions=positions, time_from_start=float(index)))
    trajectory = _trajectory(points)

    report = MODULE.validate_trajectory(trajectory)

    assert report['ok'] is False
    assert any('point count' in reason for reason in report['failureReasons'])
    assert report['checks']['point_count']['ok'] is False


def test_non_monotonic_time_fails() -> None:
    trajectory = _trajectory(
        [
            _point(positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=0.0),
            _point(positions=[0.1, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=1.0),
            _point(positions=[0.2, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=1.0),
        ]
    )

    report = MODULE.validate_trajectory(trajectory)

    assert report['ok'] is False
    assert report['checks']['monotonic_time']['ok'] is False
    assert any('strictly increasing' in reason for reason in report['failureReasons'])


def test_large_waypoint_l2_delta_fails() -> None:
    trajectory = _trajectory(
        [
            _point(positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=0.0),
            _point(positions=[1.1, 1.1, 0.0, 0.0, 0.0, 0.0], time_from_start=1.0),
        ]
    )

    report = MODULE.validate_trajectory(trajectory)

    assert report['ok'] is False
    assert report['checks']['waypoint_l2_delta']['ok'] is False
    assert any('waypoint L2 delta' in reason for reason in report['failureReasons'])


def test_repeated_wrist_sign_flips_fail() -> None:
    points = [_point(positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=0.0)]
    values = [0.3, -0.3, 0.3, -0.3, 0.3, -0.3, 0.3]
    for index, joint_4 in enumerate(values, start=1):
        points.append(
            _point(
                positions=[0.0, 0.0, 0.0, joint_4, 0.0, 0.0],
                time_from_start=float(index),
            )
        )
    trajectory = _trajectory(points)

    report = MODULE.validate_trajectory(trajectory)

    assert report['ok'] is False
    assert report['checks']['wrist_sign_flips']['ok'] is False
    assert any('wrist sign flips' in reason for reason in report['failureReasons'])
