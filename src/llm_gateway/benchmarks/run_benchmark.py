#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any

import yaml

REPO_LLM_GATEWAY_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_LLM_GATEWAY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_LLM_GATEWAY_ROOT))
REPO_SAFETY_ROOT = REPO_LLM_GATEWAY_ROOT.parent / 'safety'
if REPO_SAFETY_ROOT.exists() and str(REPO_SAFETY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_SAFETY_ROOT))

from llm_gateway.intent_router import IntentRouter
from llm_gateway.normalizer import Normalizer
from llm_gateway.parser import LLMParser
from llm_gateway.schema_validator import SchemaValidator
from llm_gateway.semantic_validator import SemanticValidator

SUPPORTED_LEVELS = ('L1', 'L2')
RESERVED_LEVELS = ('L3', 'L4')
DEFAULT_OUTPUT_BASENAME = 'benchmark_summary'
DEFAULT_L1_ITERATIONS = 25
DEFAULT_SAFETY_RULES_PATH = REPO_LLM_GATEWAY_ROOT.parent / 'safety' / 'config' / 'safety_rules.yaml'


def _macro_policy_path() -> str:
    return str(REPO_LLM_GATEWAY_ROOT / 'config' / 'macro_policy.yaml')


def _load_safety_rules() -> dict[str, Any]:
    payload = yaml.safe_load(DEFAULT_SAFETY_RULES_PATH.read_text(encoding='utf-8')) or {}
    if not isinstance(payload, dict):
        raise ValueError(f'safety rules root must be an object: {DEFAULT_SAFETY_RULES_PATH}')
    return payload


def _percentile(samples: list[float], percentile: float) -> float:
    if not samples:
        return 0.0
    if len(samples) == 1:
        return samples[0]
    ordered = sorted(samples)
    rank = (len(ordered) - 1) * percentile
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] + ((ordered[upper] - ordered[lower]) * fraction)


def _l1_corpus() -> list[dict[str, Any]]:
    return [
        {'name': 'home', 'payload': json.dumps({'primitive_type': 'HOME'})},
        {'name': 'wait', 'payload': json.dumps({'primitive_type': 'WAIT', 'wait_duration_sec': 2.0})},
        {'name': 'stop', 'payload': json.dumps({'primitive_type': 'STOP'})},
        {'name': 'get_pose', 'payload': json.dumps({'primitive_type': 'GET_POSE', 'reference_frame': 'base_link'})},
        {'name': 'set_speed', 'payload': json.dumps({'primitive_type': 'SET_SPEED', 'velocity_scale': 0.05})},
        {
            'name': 'move_rel',
            'payload': json.dumps(
                {
                    'primitive_type': 'MOVE_REL',
                    'delta_x': 0.01,
                    'delta_y': 0.0,
                    'delta_z': 0.0,
                    'reference_frame': 'base_link',
                }
            ),
        },
        {
            'name': 'ptp_pose',
            'payload': json.dumps(
                {
                    'primitive_type': 'PTP',
                    'reference_frame': 'base_link',
                    'target_pose': {
                        'position': {'x': 0.3, 'y': 0.0, 'z': 0.4},
                    },
                }
            ),
        },
        {
            'name': 'move_joint',
            'payload': json.dumps(
                {
                    'primitive_type': 'MOVE_JOINT',
                    'joint_index': 2,
                    'joint_angle': 0.2,
                }
            ),
        },
        {
            'name': 'move_joints',
            'payload': json.dumps(
                {
                    'primitive_type': 'MOVE_JOINTS',
                    'joint_target': [0.0, 0.1, -0.1, 0.2, 0.0, -0.2],
                }
            ),
        },
        {
            'name': 'io_set',
            'payload': json.dumps(
                {
                    'primitive_type': 'IO_SET',
                    'io_address': 10010,
                    'io_value': 1,
                }
            ),
        },
    ]


def _l2_corpus() -> list[dict[str, Any]]:
    return [
        {
            'name': 'vi_go_home',
            'utterance': 'về nhà',
            'payload': {'intent': 'go_home'},
            'expected': {'routeType': 'primitive', 'primitiveTypes': ['HOME']},
        },
        {
            'name': 'en_go_home',
            'utterance': 'bring the robot back to start',
            'payload': {'intent': 'go_home'},
            'expected': {'routeType': 'primitive', 'primitiveTypes': ['HOME']},
        },
        {
            'name': 'vi_move_relative',
            'utterance': 'nâng lên 5cm',
            'payload': {'intent': 'move_relative', 'delta': {'x': 0.0, 'y': 0.0, 'z': 0.05}},
            'expected': {
                'routeType': 'primitive',
                'primitiveTypes': ['MOVE_REL'],
                'commandFields': {'delta_z': 0.05, 'reference_frame': 'base_link'},
            },
        },
        {
            'name': 'en_absolute_move_ptp',
            'utterance': 'move to x=0.3 y=0 z=0.4',
            'payload': {'intent': 'absolute_move_ptp', 'target_pose': {'position': {'x': 0.3, 'y': 0.0, 'z': 0.4}}},
            'expected': {
                'routeType': 'primitive',
                'primitiveTypes': ['PTP'],
                'commandFields': {'reference_frame': 'base_link'},
                'nestedFields': {'target_pose.position.x': 0.3, 'target_pose.position.z': 0.4},
            },
        },
        {
            'name': 'vi_wait',
            'utterance': 'chờ 3 giây',
            'payload': {'intent': 'wait', 'wait_duration_sec': 3.0},
            'expected': {
                'routeType': 'primitive',
                'primitiveTypes': ['WAIT'],
                'commandFields': {'wait_duration_sec': 3.0},
            },
        },
        {
            'name': 'en_stop',
            'utterance': 'emergency stop',
            'payload': {'intent': 'stop'},
            'expected': {'routeType': 'primitive', 'primitiveTypes': ['STOP']},
        },
        {
            'name': 'vi_io_set',
            'utterance': 'bật đầu ra 10010',
            'payload': {'intent': 'io_set', 'io_address': 10010, 'io_value': 1},
            'expected': {
                'routeType': 'primitive',
                'primitiveTypes': ['IO_SET'],
                'commandFields': {'io_address': 10010, 'io_value': 1},
            },
        },
        {
            'name': 'en_sequence',
            'utterance': 'go home, wait one second, then move linearly to x 0.3 y 0 z 0.3',
            'payload': {
                'intent': 'sequence',
                'steps': [
                    {'intent': 'go_home'},
                    {'intent': 'wait', 'wait_duration_sec': 1.0},
                    {
                        'intent': 'absolute_move_lin',
                        'reference_frame': 'base_link',
                        'target_pose': {'position': {'x': 0.3, 'y': 0.0, 'z': 0.3}},
                    },
                ],
            },
            'expected': {
                'routeType': 'sequence',
                'primitiveTypes': ['HOME', 'WAIT', 'LIN'],
                'commandFields': {'reference_frame': 'base_link'},
            },
        },
    ]


def _resolve_nested_field(command: dict[str, Any], dotted_path: str) -> Any:
    current: Any = command
    for part in dotted_path.split('.'):
        if not isinstance(current, dict):
            raise KeyError(dotted_path)
        current = current[part]
    return current


def _run_l1() -> dict[str, Any]:
    parser = LLMParser()
    schema_validator = SchemaValidator()
    normalizer = Normalizer()
    semantic_validator = SemanticValidator(safety_rules=_load_safety_rules())
    samples_ms: list[float] = []
    corpus = _l1_corpus()

    for _ in range(DEFAULT_L1_ITERATIONS):
        for case in corpus:
            started = time.perf_counter_ns()
            parsed = parser.parse(case['payload'])
            valid, error = schema_validator.validate_against_schema(parsed)
            if not valid:
                raise ValueError(f"L1 schema validation failed for {case['name']}: {error}")
            normalized = normalizer.normalize(parsed)
            semantic_validator.validate(normalized)
            elapsed_ms = (time.perf_counter_ns() - started) / 1_000_000.0
            samples_ms.append(elapsed_ms)

    metrics = {
        'p50_ms': round(_percentile(samples_ms, 0.50), 6),
        'p95_ms': round(_percentile(samples_ms, 0.95), 6),
        'p99_ms': round(_percentile(samples_ms, 0.99), 6),
        'mean_ms': round(statistics.fmean(samples_ms), 6),
    }
    return {
        'level': 'L1',
        'caseCount': len(corpus),
        'samples': len(samples_ms),
        'metrics': metrics,
        'cases': [{'name': case['name']} for case in corpus],
        'notes': [
            'L1 measures parser plus schema/normalization/semantic validation latency on fixed direct-command payloads.',
        ],
    }


def _run_l2() -> dict[str, Any]:
    router = IntentRouter(macro_policy_path=_macro_policy_path(), runtime_mode='hardware')
    corpus = _l2_corpus()
    passed_cases = 0
    case_results: list[dict[str, Any]] = []

    for case in corpus:
        result = router.route(case['payload'])
        primitive_types = [command.get('primitive_type') for command in result.commands]
        expected = case['expected']
        issues: list[str] = []

        if result.route_type != expected['routeType']:
            issues.append(
                f"route_type expected {expected['routeType']} got {result.route_type}"
            )
        if primitive_types != expected['primitiveTypes']:
            issues.append(
                f"primitive types expected {expected['primitiveTypes']} got {primitive_types}"
            )

        command_fields = expected.get('commandFields', {})
        if command_fields:
            first_command = result.commands[-1] if expected['routeType'] == 'sequence' else result.commands[0]
            for key, expected_value in command_fields.items():
                actual_value = first_command.get(key)
                if actual_value != expected_value:
                    issues.append(f'field {key} expected {expected_value!r} got {actual_value!r}')

        nested_fields = expected.get('nestedFields', {})
        if nested_fields:
            first_command = result.commands[0]
            for key, expected_value in nested_fields.items():
                actual_value = _resolve_nested_field(first_command, key)
                if actual_value != expected_value:
                    issues.append(f'nested field {key} expected {expected_value!r} got {actual_value!r}')

        passed = not issues
        if passed:
            passed_cases += 1
        case_results.append(
            {
                'name': case['name'],
                'utterance': case['utterance'],
                'passed': passed,
                'routeType': result.route_type,
                'primitiveTypes': primitive_types,
                'issues': issues,
            }
        )

    case_count = len(corpus)
    accuracy = 0.0 if case_count == 0 else round(passed_cases / case_count, 6)
    return {
        'level': 'L2',
        'caseCount': case_count,
        'passedCases': passed_cases,
        'accuracy': accuracy,
        'cases': case_results,
        'notes': [
            'L2 measures IntentRouter correctness against a fixed EN+VI semantic corpus derived from current prompt-builder examples.',
            'This harness intentionally avoids live LLM calls and auto-confirm/node execution surfaces.',
        ],
    }


def _write_outputs(report: dict[str, Any], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / f'{DEFAULT_OUTPUT_BASENAME}.json'
    csv_path = output_dir / f'{DEFAULT_OUTPUT_BASENAME}.csv'

    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=True), encoding='utf-8')

    rows: list[dict[str, Any]] = []
    for level_name, result in report['results'].items():
        row: dict[str, Any] = {
            'level': level_name,
            'case_count': result.get('caseCount', 0),
        }
        if level_name == 'L1':
            metrics = result['metrics']
            row.update(metrics)
            row['samples'] = result['samples']
        if level_name == 'L2':
            row['passed_cases'] = result['passedCases']
            row['accuracy'] = result['accuracy']
        rows.append(row)

    fieldnames = sorted({key for row in rows for key in row.keys()})
    with csv_path.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run_levels(levels: list[str], *, output_dir: str | Path) -> dict[str, Any]:
    selected_levels = list(levels)
    results: dict[str, Any] = {}
    for level in selected_levels:
        if level == 'L1':
            results[level] = _run_l1()
        elif level == 'L2':
            results[level] = _run_l2()
        else:
            raise ValueError(f'Unsupported benchmark level: {level}')

    report = {
        'levels': selected_levels,
        'results': results,
        'outputDir': str(Path(output_dir)),
    }
    _write_outputs(report, Path(output_dir))
    return report


def _parse_levels(levels_text: str) -> list[str]:
    requested = [level.strip().upper() for level in levels_text.split(',') if level.strip()]
    if not requested:
        raise ValueError('At least one benchmark level is required.')
    unsupported = [level for level in requested if level not in SUPPORTED_LEVELS]
    if unsupported:
        raise ValueError(
            'Unsupported benchmark levels: '
            + ', '.join(unsupported)
            + '. L3/L4 are intentionally not implemented in this source-only harness because their live/backend surfaces are unstable on the current branch.'
        )
    return requested


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run fixed-corpus llm_gateway benchmarks for the current branch state.',
        epilog=(
            'Supported levels: L1,L2. '
            'Reserved but intentionally not implemented here: L3,L4 (live HMI/sim timing) to avoid fabricating unstable runtime APIs.'
        ),
    )
    parser.add_argument(
        '--levels',
        required=True,
        help='Comma-separated benchmark levels to run, for example: L1,L2',
    )
    parser.add_argument(
        '--output',
        required=True,
        help='Directory where benchmark_summary.json and benchmark_summary.csv will be written.',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv or sys.argv[1:])
    levels = _parse_levels(args.levels)
    report = run_levels(levels, output_dir=args.output)
    print(json.dumps(report, indent=2, ensure_ascii=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
