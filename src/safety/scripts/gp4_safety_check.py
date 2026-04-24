#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import yaml

MAX_POINT_COUNT = 200
MAX_WAYPOINT_L2_DELTA = 1.5
MAX_CONSECUTIVE_WRIST_SIGN_FLIPS = 5
NEAR_ZERO_DELTA = 1e-6
JOINT_DELTA_THRESHOLDS = {
    0: 0.4363323129985824,
    1: 0.4363323129985824,
    2: 0.4363323129985824,
    3: 0.7853981633974483,
    4: 0.7853981633974483,
    5: 0.5235987755982988,
}
WRIST_JOINT_INDICES = (3, 4, 5)
DEFAULT_JOINT_LIMITS_PATH = Path(__file__).resolve().parents[2] / 'gp4_moveit_config' / 'config' / 'joint_limits.yaml'


def _load_joint_limits(joint_limits_path: Path) -> dict[str, dict[str, float]]:
    payload = yaml.safe_load(joint_limits_path.read_text(encoding='utf-8')) or {}
    joint_limits = payload.get('joint_limits')
    if not isinstance(joint_limits, dict) or not joint_limits:
        raise ValueError(f'joint_limits missing from {joint_limits_path}')
    normalized: dict[str, dict[str, float]] = {}
    for joint_name, values in joint_limits.items():
        if not isinstance(values, dict):
            raise ValueError(f'joint limit entry for {joint_name} must be an object')
        normalized[str(joint_name)] = {
            'min_position': float(values['min_position']),
            'max_position': float(values['max_position']),
        }
    return normalized


def _load_trajectory(path: Path) -> dict[str, Any]:
    raw_text = path.read_text(encoding='utf-8')
    suffix = path.suffix.lower()
    if suffix == '.json':
        payload = json.loads(raw_text)
    elif suffix in {'.yaml', '.yml'}:
        payload = yaml.safe_load(raw_text)
    else:
        try:
            payload = json.loads(raw_text)
        except json.JSONDecodeError:
            payload = yaml.safe_load(raw_text)
    if not isinstance(payload, dict):
        raise ValueError('trajectory file must decode to a JSON/YAML object')
    for key in ('trajectory', 'joint_trajectory'):
        nested = payload.get(key)
        if isinstance(nested, dict):
            return nested
    return payload


def _as_float(value: Any, *, field: str) -> float:
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{field} must be numeric') from exc
    if not math.isfinite(converted):
        raise ValueError(f'{field} must be finite')
    return converted


def _time_from_start_seconds(raw_value: Any) -> float:
    if isinstance(raw_value, dict):
        sec = raw_value.get('sec', raw_value.get('secs', 0.0))
        nanosec = raw_value.get('nanosec', raw_value.get('nsec', raw_value.get('nanoseconds', 0.0)))
        return _as_float(sec, field='time_from_start.sec') + (_as_float(nanosec, field='time_from_start.nanosec') / 1_000_000_000.0)
    return _as_float(raw_value, field='time_from_start')


def _shortest_angular_delta(raw_delta: float) -> float:
    while raw_delta > math.pi:
        raw_delta -= 2.0 * math.pi
    while raw_delta < -math.pi:
        raw_delta += 2.0 * math.pi
    return raw_delta


def _check_result(*, ok: bool, detail: str, **metadata: Any) -> dict[str, Any]:
    result: dict[str, Any] = {'ok': ok, 'detail': detail}
    result.update(metadata)
    return result


def validate_trajectory(
    trajectory: dict[str, Any],
    *,
    joint_limits_path: str | Path = DEFAULT_JOINT_LIMITS_PATH,
) -> dict[str, Any]:
    resolved_joint_limits_path = Path(joint_limits_path)
    joint_limits = _load_joint_limits(resolved_joint_limits_path)

    joint_names_raw = trajectory.get('joint_names', trajectory.get('jointNames'))
    points_raw = trajectory.get('points')
    if not isinstance(joint_names_raw, list) or not joint_names_raw:
        raise ValueError('trajectory must provide a non-empty joint_names list')
    if not isinstance(points_raw, list):
        raise ValueError('trajectory must provide a points list')

    joint_names = [str(name) for name in joint_names_raw]
    point_count = len(points_raw)
    joint_count = len(joint_names)
    checks: dict[str, dict[str, Any]] = {}
    failure_reasons: list[str] = []

    checks['point_count'] = _check_result(
        ok=point_count <= MAX_POINT_COUNT,
        detail=(
            f'point count {point_count} exceeds limit {MAX_POINT_COUNT}'
            if point_count > MAX_POINT_COUNT
            else f'point count {point_count} within limit {MAX_POINT_COUNT}'
        ),
        actual=point_count,
        limit=MAX_POINT_COUNT,
    )

    time_values: list[float] = []
    positions_per_point: list[list[float]] = []
    non_finite_velocity_detail: str | None = None
    non_finite_acceleration_detail: str | None = None
    positions_detail: str | None = None
    joint_bounds_detail: str | None = None

    for point_index, point in enumerate(points_raw):
        if not isinstance(point, dict):
            raise ValueError(f'point {point_index} must be an object')
        positions_raw = point.get('positions')
        if not isinstance(positions_raw, list) or len(positions_raw) != joint_count:
            raise ValueError(
                f'point {point_index} must provide positions for exactly {joint_count} joints'
            )

        time_values.append(_time_from_start_seconds(point.get('time_from_start', point.get('timeFromStart', 0.0))))

        positions: list[float] = []
        for joint_index, (joint_name, raw_position) in enumerate(zip(joint_names, positions_raw)):
            position = _as_float(raw_position, field=f'point[{point_index}].positions[{joint_index}]')
            positions.append(position)
            bounds = joint_limits.get(joint_name)
            if bounds is None:
                raise ValueError(f'joint {joint_name} missing from joint limits file {resolved_joint_limits_path}')
            if position < bounds['min_position'] or position > bounds['max_position']:
                if joint_bounds_detail is None:
                    joint_bounds_detail = (
                        f'point {point_index} joint {joint_name} position {position:.6f} outside '
                        f"[{bounds['min_position']:.6f}, {bounds['max_position']:.6f}]"
                    )
        positions_per_point.append(positions)

        velocities_raw = point.get('velocities')
        if velocities_raw is not None:
            if not isinstance(velocities_raw, list) or len(velocities_raw) != joint_count:
                raise ValueError(
                    f'point {point_index} velocities must match joint count {joint_count}'
                )
            for joint_index, raw_velocity in enumerate(velocities_raw):
                try:
                    _as_float(raw_velocity, field=f'point[{point_index}].velocities[{joint_index}]')
                except ValueError as exc:
                    if non_finite_velocity_detail is None:
                        non_finite_velocity_detail = str(exc)

        accelerations_raw = point.get('accelerations')
        if accelerations_raw is not None:
            if not isinstance(accelerations_raw, list) or len(accelerations_raw) != joint_count:
                raise ValueError(
                    f'point {point_index} accelerations must match joint count {joint_count}'
                )
            for joint_index, raw_acceleration in enumerate(accelerations_raw):
                try:
                    _as_float(raw_acceleration, field=f'point[{point_index}].accelerations[{joint_index}]')
                except ValueError as exc:
                    if non_finite_acceleration_detail is None:
                        non_finite_acceleration_detail = str(exc)

    checks['positions_finite'] = _check_result(
        ok=positions_detail is None,
        detail=positions_detail or 'all positions are finite',
    )
    checks['velocities_finite'] = _check_result(
        ok=non_finite_velocity_detail is None,
        detail=non_finite_velocity_detail or 'all provided velocities are finite',
    )
    checks['accelerations_finite'] = _check_result(
        ok=non_finite_acceleration_detail is None,
        detail=non_finite_acceleration_detail or 'all provided accelerations are finite',
    )
    checks['joint_bounds'] = _check_result(
        ok=joint_bounds_detail is None,
        detail=joint_bounds_detail or 'all points satisfy joint position bounds',
        limitsPath=str(resolved_joint_limits_path),
    )

    monotonic_detail = 'time_from_start values are strictly increasing'
    for index in range(1, len(time_values)):
        if not time_values[index] > time_values[index - 1]:
            monotonic_detail = (
                f'time_from_start must be strictly increasing: point {index - 1}={time_values[index - 1]:.9f}, '
                f'point {index}={time_values[index]:.9f}'
            )
            break
    checks['monotonic_time'] = _check_result(
        ok=monotonic_detail == 'time_from_start values are strictly increasing',
        detail=monotonic_detail,
    )

    continuity_detail = 'all consecutive waypoint deltas stay within WristFlipGuard thresholds'
    l2_delta_detail = f'all consecutive waypoint L2 deltas stay <= {MAX_WAYPOINT_L2_DELTA:.3f}'
    for point_index in range(1, len(positions_per_point)):
        deltas: list[float] = []
        for joint_index, joint_name in enumerate(joint_names):
            raw_delta = positions_per_point[point_index][joint_index] - positions_per_point[point_index - 1][joint_index]
            if joint_index == 5:
                raw_delta = _shortest_angular_delta(raw_delta)
            delta = abs(raw_delta)
            deltas.append(raw_delta)
            threshold = JOINT_DELTA_THRESHOLDS.get(joint_index, JOINT_DELTA_THRESHOLDS[0])
            if delta > threshold and continuity_detail.startswith('all consecutive'):
                continuity_detail = (
                    f'point {point_index - 1}->{point_index} joint {joint_name} delta {delta:.6f} exceeds '
                    f'WristFlipGuard threshold {threshold:.6f}'
                )
        l2_delta = math.sqrt(sum(delta * delta for delta in deltas))
        if l2_delta > MAX_WAYPOINT_L2_DELTA and l2_delta_detail.startswith('all consecutive'):
            l2_delta_detail = (
                f'point {point_index - 1}->{point_index} waypoint L2 delta {l2_delta:.6f} exceeds '
                f'limit {MAX_WAYPOINT_L2_DELTA:.6f}'
            )
    checks['continuity_thresholds'] = _check_result(
        ok=continuity_detail.startswith('all consecutive'),
        detail=continuity_detail,
        thresholdsRad={str(index): threshold for index, threshold in JOINT_DELTA_THRESHOLDS.items()},
    )
    checks['waypoint_l2_delta'] = _check_result(
        ok=l2_delta_detail.startswith('all consecutive'),
        detail=l2_delta_detail,
        limit=MAX_WAYPOINT_L2_DELTA,
    )

    wrist_flip_detail = 'wrist sign flips stay within limit'
    for joint_index in WRIST_JOINT_INDICES:
        if joint_index >= joint_count:
            continue
        consecutive_flips = 0
        previous_delta = 0.0
        for point_index in range(1, len(positions_per_point)):
            delta = positions_per_point[point_index][joint_index] - positions_per_point[point_index - 1][joint_index]
            if joint_index == 5:
                delta = _shortest_angular_delta(delta)
            if abs(delta) < NEAR_ZERO_DELTA:
                continue
            if previous_delta != 0.0:
                if (previous_delta > 0.0 and delta < 0.0) or (previous_delta < 0.0 and delta > 0.0):
                    consecutive_flips += 1
                    if consecutive_flips > MAX_CONSECUTIVE_WRIST_SIGN_FLIPS:
                        wrist_flip_detail = (
                            f'wrist sign flips exceed limit at joint {joint_names[joint_index]}: '
                            f'{consecutive_flips} consecutive flips > {MAX_CONSECUTIVE_WRIST_SIGN_FLIPS}'
                        )
                        break
                else:
                    consecutive_flips = 0
            previous_delta = delta
        if not wrist_flip_detail.startswith('wrist sign flips stay'):
            break
    checks['wrist_sign_flips'] = _check_result(
        ok=wrist_flip_detail.startswith('wrist sign flips stay'),
        detail=wrist_flip_detail,
        limit=MAX_CONSECUTIVE_WRIST_SIGN_FLIPS,
    )

    for check_name, result in checks.items():
        if not result['ok']:
            failure_reasons.append(f'{check_name}: {result["detail"]}')

    return {
        'ok': not failure_reasons,
        'checks': checks,
        'failureReasons': failure_reasons,
        'summary': {
            'pointCount': point_count,
            'jointCount': joint_count,
            'jointNames': joint_names,
            'jointLimitsPath': str(resolved_joint_limits_path),
        },
    }


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Offline GP4 trajectory safety checker with WristFlipGuard-aligned continuity checks.'
    )
    parser.add_argument('trajectory_path', help='Path to a JSON or YAML joint trajectory file.')
    parser.add_argument(
        '--joint-limits',
        default=str(DEFAULT_JOINT_LIMITS_PATH),
        help='Path to gp4 joint_limits.yaml (default: repo source tree copy).',
    )
    parser.add_argument(
        '--pretty',
        action='store_true',
        help='Pretty-print JSON output.',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv or sys.argv[1:])
    trajectory_path = Path(args.trajectory_path)
    try:
        report = validate_trajectory(
            _load_trajectory(trajectory_path),
            joint_limits_path=args.joint_limits,
        )
    except Exception as exc:
        report = {
            'ok': False,
            'checks': {},
            'failureReasons': [str(exc)],
            'summary': {
                'inputPath': str(trajectory_path),
                'jointLimitsPath': str(args.joint_limits),
            },
        }

    report['summary']['inputPath'] = str(trajectory_path)
    json_kwargs = {'ensure_ascii': True}
    if args.pretty:
        json_kwargs['indent'] = 2
    print(json.dumps(report, **json_kwargs))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
