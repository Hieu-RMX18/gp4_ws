from __future__ import annotations

import hashlib
import json
import os
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import mock

from hmi.backend.services.hardware_gate import HardwareGateEvaluator


class HardwareGateEvaluatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmpdir = TemporaryDirectory()
        self.addCleanup(self._tmpdir.cleanup)
        self._tmp_path = Path(self._tmpdir.name)
        self._report_path = self._tmp_path / 'report.txt'
        self._report_path.write_text('hardware-check-report\n', encoding='utf-8')
        self._report_sha256 = hashlib.sha256(self._report_path.read_bytes()).hexdigest()

    def _valid_payload(self) -> dict[str, object]:
        return {
            'approved': True,
            'approvedBy': 'qa.engineer',
            'approvedAt': '2026-04-22T10:00:00Z',
            'reportPath': str(self._report_path),
            'reportSha256': self._report_sha256,
            'checklist': {
                'timingJitter': True,
                'disconnectReconnect': True,
                'robotStatusSemantics': True,
                'jointSourcePrecedence': True,
                'auditVisibility': True,
            },
        }

    def _write_evidence(self, payload: dict[str, object]) -> Path:
        evidence_path = self._tmp_path / 'hardware_gate.json'
        evidence_path.write_text(json.dumps(payload), encoding='utf-8')
        return evidence_path

    def _evaluate(
        self,
        *,
        env_value: str | None = '1',
        payload: dict[str, object] | None = None,
        raw_evidence: str | None = None,
        evidence_path: Path | None = None,
    ):
        if evidence_path is None:
            evidence_path = self._tmp_path / 'hardware_gate.json'
        if raw_evidence is not None:
            evidence_path.write_text(raw_evidence, encoding='utf-8')
        elif payload is not None:
            evidence_path = self._write_evidence(payload)

        env_patch: dict[str, str] = {}
        if env_value is not None:
            env_patch['HMI_ENABLE_HARDWARE_COMMANDS'] = env_value

        with mock.patch.dict(os.environ, env_patch, clear=False):
            if env_value is None:
                os.environ.pop('HMI_ENABLE_HARDWARE_COMMANDS', None)
            evaluator = HardwareGateEvaluator(evidence_path=evidence_path)
            return evaluator.evaluate()

    def test_blocks_when_env_flag_is_missing(self) -> None:
        snapshot = self._evaluate(env_value=None, payload=self._valid_payload())

        self.assertFalse(snapshot.unlocked)
        self.assertIn('HMI_ENABLE_HARDWARE_COMMANDS is not enabled.', snapshot.reasons)

    def test_blocks_when_evidence_file_is_missing(self) -> None:
        snapshot = self._evaluate(evidence_path=self._tmp_path / 'missing.json')

        self.assertFalse(snapshot.unlocked)
        self.assertIn('hardware gate evidence missing', snapshot.reasons[0])

    def test_blocks_when_json_is_malformed(self) -> None:
        snapshot = self._evaluate(raw_evidence='{"approved": true', env_value='1')

        self.assertFalse(snapshot.unlocked)
        self.assertIn('is not valid JSON', snapshot.reasons[0])

    def test_blocks_when_approved_is_false(self) -> None:
        payload = self._valid_payload()
        payload['approved'] = False

        snapshot = self._evaluate(payload=payload)

        self.assertFalse(snapshot.unlocked)
        self.assertIn('approved: true', '\n'.join(snapshot.reasons))

    def test_blocks_when_approved_by_is_missing(self) -> None:
        payload = self._valid_payload()
        payload['approvedBy'] = None

        snapshot = self._evaluate(payload=payload)

        self.assertFalse(snapshot.unlocked)
        self.assertIn('missing approvedBy', '\n'.join(snapshot.reasons))

    def test_blocks_when_approved_at_is_missing(self) -> None:
        payload = self._valid_payload()
        payload['approvedAt'] = None

        snapshot = self._evaluate(payload=payload)

        self.assertFalse(snapshot.unlocked)
        self.assertIn('missing approvedAt', '\n'.join(snapshot.reasons))

    def test_blocks_when_checklist_is_missing(self) -> None:
        payload = self._valid_payload()
        payload.pop('checklist')

        snapshot = self._evaluate(payload=payload)

        self.assertFalse(snapshot.unlocked)
        self.assertIn('missing checklist', '\n'.join(snapshot.reasons))

    def test_blocks_when_checklist_is_incomplete(self) -> None:
        payload = self._valid_payload()
        payload['checklist'] = {
            'timingJitter': True,
            'disconnectReconnect': False,
            'robotStatusSemantics': True,
            'jointSourcePrecedence': True,
            'auditVisibility': True,
        }

        snapshot = self._evaluate(payload=payload)

        self.assertFalse(snapshot.unlocked)
        self.assertIn('checklist is incomplete', '\n'.join(snapshot.reasons))

    def test_blocks_when_report_path_is_missing(self) -> None:
        payload = self._valid_payload()
        payload['reportPath'] = None

        snapshot = self._evaluate(payload=payload)

        self.assertFalse(snapshot.unlocked)
        self.assertIn('missing reportPath', '\n'.join(snapshot.reasons))

    def test_blocks_when_report_sha256_is_missing(self) -> None:
        payload = self._valid_payload()
        payload['reportSha256'] = None

        snapshot = self._evaluate(payload=payload)

        self.assertFalse(snapshot.unlocked)
        self.assertIn('missing reportSha256', '\n'.join(snapshot.reasons))

    def test_blocks_when_report_sha256_mismatches(self) -> None:
        payload = self._valid_payload()
        payload['reportSha256'] = '0' * 64

        snapshot = self._evaluate(payload=payload)

        self.assertFalse(snapshot.unlocked)
        self.assertIn('report SHA256 does not match', '\n'.join(snapshot.reasons))

    def test_unlocks_when_all_requirements_are_met(self) -> None:
        snapshot = self._evaluate(payload=self._valid_payload())

        self.assertTrue(snapshot.unlocked)
        self.assertEqual(snapshot.reasons, [])
        self.assertTrue(snapshot.flag_enabled)
        self.assertEqual(snapshot.approved_by, 'qa.engineer')
        self.assertEqual(snapshot.approved_at, '2026-04-22T10:00:00Z')
        self.assertEqual(snapshot.report_path, str(self._report_path))
        self.assertEqual(snapshot.report_sha256, self._report_sha256)
        self.assertTrue(snapshot.report_sha256_match)
        self.assertIsNotNone(snapshot.checklist)


if __name__ == '__main__':
    unittest.main()
