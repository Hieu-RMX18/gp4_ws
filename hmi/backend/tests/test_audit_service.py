from __future__ import annotations

import sqlite3
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from hmi.backend.domain.models import SystemRuntimeState
from hmi.backend.services.audit_service import AuditService


class AuditServiceTests(unittest.TestCase):
    def test_telemetry_snapshot_retention_by_count(self) -> None:
        with TemporaryDirectory() as temp_dir:
            db_path = Path(temp_dir) / 'audit.sqlite3'
            audit = AuditService(
                db_path,
                max_telemetry_snapshots=2,
                telemetry_retention_days=365,
            )

            for index in range(3):
                audit.record_telemetry_snapshot(
                    transport_state='connected',
                    runtime_state=SystemRuntimeState.NORMAL,
                    payload={'index': index},
                )

            with sqlite3.connect(db_path) as connection:
                count = connection.execute(
                    'SELECT COUNT(*) FROM telemetry_snapshots'
                ).fetchone()[0]

            self.assertEqual(count, 2)
            self.assertEqual(
                audit.telemetry_retention_policy(),
                {
                    'maxTelemetrySnapshots': 2,
                    'telemetryRetentionDays': 365,
                },
            )


if __name__ == '__main__':
    unittest.main()
