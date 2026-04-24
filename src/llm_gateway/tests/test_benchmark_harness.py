from __future__ import annotations

import importlib.util
import json
from pathlib import Path


BENCHMARK_PATH = Path(__file__).resolve().parents[1] / 'benchmarks' / 'run_benchmark.py'
SPEC = importlib.util.spec_from_file_location('run_benchmark', BENCHMARK_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_run_levels_writes_json_and_csv(tmp_path: Path) -> None:
    report = MODULE.run_levels(['L1', 'L2'], output_dir=tmp_path)

    assert report['levels'] == ['L1', 'L2']
    assert (tmp_path / 'benchmark_summary.json').exists()
    assert (tmp_path / 'benchmark_summary.csv').exists()

    written_json = json.loads((tmp_path / 'benchmark_summary.json').read_text(encoding='utf-8'))
    assert written_json['levels'] == ['L1', 'L2']


def test_l1_reports_percentiles(tmp_path: Path) -> None:
    report = MODULE.run_levels(['L1'], output_dir=tmp_path)

    metrics = report['results']['L1']['metrics']
    assert metrics['p50_ms'] >= 0.0
    assert metrics['p95_ms'] >= metrics['p50_ms']
    assert metrics['p99_ms'] >= metrics['p95_ms']
    assert report['results']['L1']['caseCount'] > 0


def test_l2_reports_full_correctness_on_fixed_corpus(tmp_path: Path) -> None:
    report = MODULE.run_levels(['L2'], output_dir=tmp_path)

    result = report['results']['L2']
    assert result['passedCases'] == result['caseCount']
    assert result['accuracy'] == 1.0
    assert result['caseCount'] >= 6
