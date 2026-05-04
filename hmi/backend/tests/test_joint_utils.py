"""Tests for extracted joint target resolution utilities."""

from __future__ import annotations

from types import SimpleNamespace
import unittest

from hmi.backend.domain.constants import GP4_JOINT_NAMES


class TestResolveJointTarget(unittest.TestCase):
    def _call(self, parameters: dict) -> tuple[int | None, str | None]:
        from hmi.backend.domain.joint_utils import resolve_joint_target

        return resolve_joint_target(parameters, GP4_JOINT_NAMES)

    def test_zero_based_index(self) -> None:
        idx, name = self._call({"jointIndexZeroBased": 0})
        self.assertEqual(idx, 0)
        self.assertEqual(name, "joint_1_s")

    def test_zero_based_index_last(self) -> None:
        idx, name = self._call({"jointIndexZeroBased": 5})
        self.assertEqual(idx, 5)
        self.assertEqual(name, "joint_6_t")

    def test_joint_name_resolved(self) -> None:
        idx, name = self._call({"jointNameResolved": "joint_4_r"})
        self.assertEqual(idx, 3)
        self.assertEqual(name, "joint_4_r")

    def test_one_based_index(self) -> None:
        idx, name = self._call({"jointIndex": 6})
        self.assertEqual(idx, 5)
        self.assertEqual(name, "joint_6_t")

    def test_shorthand_joint_name(self) -> None:
        idx, name = self._call({"joint": "joint_3_u"})
        self.assertEqual(idx, 2)
        self.assertEqual(name, "joint_3_u")

    def test_no_match_returns_none(self) -> None:
        idx, name = self._call({})
        self.assertIsNone(idx)
        self.assertIsNone(name)

    def test_out_of_range_index_ignored(self) -> None:
        idx, name = self._call({"jointIndexZeroBased": 99})
        self.assertIsNone(idx)
        self.assertIsNone(name)


class TestReadJointPositionDeg(unittest.TestCase):
    def test_found(self) -> None:
        from hmi.backend.domain.joint_utils import read_joint_position_deg

        joints = [SimpleNamespace(name="joint_1_s", position_deg=45.0)]
        self.assertEqual(read_joint_position_deg("joint_1_s", joints), 45.0)

    def test_not_found(self) -> None:
        from hmi.backend.domain.joint_utils import read_joint_position_deg

        self.assertIsNone(read_joint_position_deg("joint_1_s", []))


if __name__ == "__main__":
    unittest.main()
