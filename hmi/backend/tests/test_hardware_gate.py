from __future__ import annotations

import unittest

from hmi.backend.services.hardware_gate import HardwareGateEvaluator


class HardwareGateEvaluatorTests(unittest.TestCase):
    def test_always_unlocked(self) -> None:
        snapshot = HardwareGateEvaluator().evaluate()
        self.assertTrue(snapshot.unlocked)
        self.assertEqual(snapshot.reasons, [])
        self.assertTrue(snapshot.flag_enabled)


if __name__ == "__main__":
    unittest.main()
