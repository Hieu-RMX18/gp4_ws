"""Tests for tools/validate_safety_chain.py Check 3.

Check 3 now only validates joint_5_b presence and parseability. The numeric
envelope is enforced by Check 1 against URDF joint_limits.yaml.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPT = _REPO_ROOT / "tools" / "validate_safety_chain.py"


def _load():
    spec = importlib.util.spec_from_file_location("validate_safety_chain", _SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _hw_with_j5(j5_hw: float) -> dict:
    return {
        "joint_limits": {
            "joint_1_s": {"min_position": -2.967, "max_position": 2.967},
            "joint_2_l": {"min_position": -1.9198, "max_position": 2.2689},
            "joint_3_u": {"min_position": -1.1344, "max_position": 3.4906},
            "joint_4_r": {"min_position": -2.443, "max_position": 2.443},
            "joint_5_b": {"min_position": -j5_hw, "max_position": j5_hw},
            "joint_6_t": {"min_position": -3.142, "max_position": 3.142},
        },
    }


_MOTOROS2 = {
    "joint_names": [
        "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t",
    ],
}


def _safety(j5: str = "  joint_5_b: {min: -2.00, max:  2.00}\n") -> dict:
    text = f"""\
joint_position_guard:
  enabled: true
manipulability_guard:
  enabled: true
cumulative_rotation_guard:
  enabled: true
calibration:
  max_reprojection_error_mm: 3.0
workspace_bounds:
  x_min: -0.45
  x_max:  0.45
  y_min: -0.16
  y_max:  0.52
  z_min:  0.15
  z_max:  0.65
motion_limits:
  max_move_rel_translation: 0.21
operational_joint_limits:
  joint_1_s: {{min: -2.967, max:  2.967}}
  joint_2_l: {{min: -1.9198, max:  2.2689}}
  joint_3_u: {{min: -1.1344, max:  3.4906}}
  joint_4_r: {{min: -2.443, max:  2.443}}
{j5}  joint_6_t: {{min: -3.142, max:  3.142}}
"""
    return yaml.safe_load(text) or {}


def _run_main(j5_line: str, j5_hw: float = 2.1467):
    mod = _load()
    safety = _safety(j5_line)
    hw = _hw_with_j5(j5_hw)
    # main() calls load_yaml 4x: safety, joint_limits, motoros2, extrinsics.
    # We patch Path.exists to return True for everything except the
    # extrinsics path (which would otherwise trigger Check 5). All load_yaml
    # calls are intercepted with pre-built dicts. Check 6 needs the move_rel
    # constants to match the safety values.
    repo_root = Path(_SCRIPT).resolve().parent.parent

    def _exists(self):
        path_str = str(self)
        if "extrinsics.yaml" in path_str:
            return False
        return True

    move_rel = {
        "XMin": -0.45, "XMax": 0.45, "YMin": -0.16, "YMax": 0.52,
        "ZMin": 0.15, "ZMax": 0.65, "MaxDeltaNorm": 0.21,
    }
    with (
        patch.object(mod, "load_yaml", side_effect=[safety, hw, _MOTOROS2, {}]),
        patch.object(mod, "load_move_rel_limits", return_value=move_rel),
        patch("builtins.open"),
        patch.object(Path, "exists", _exists),
        contextlib.redirect_stderr(io.StringIO()) as buf,
    ):
        status = mod.main()
    return status, buf.getvalue()


class TestCheck3(unittest.TestCase):
    def test_j5_present_passes_check3(self) -> None:
        """±2.00 inside URDF envelope: main() returns 0."""
        status, stderr = _run_main("  joint_5_b: {min: -2.00, max:  2.00}\n")
        self.assertEqual(status, 0, msg=stderr)

    def test_j5_missing_reports_error(self) -> None:
        status, stderr = _run_main("")
        self.assertNotEqual(status, 0)
        self.assertIn("joint_5_b missing from operational_joint_limits", stderr)

    def test_j5_unparseable_reports_error(self) -> None:
        status, stderr = _run_main("  joint_5_b: not_a_dict\n")
        self.assertNotEqual(status, 0)
        self.assertIn(
            "joint_5_b has no parseable limits in operational_joint_limits", stderr,
        )

    def test_j5_outside_hw_envelope_caught_by_check1(self) -> None:
        status, stderr = _run_main(
            "  joint_5_b: {min: -2.50, max:  2.50}\n", j5_hw=2.1467,
        )
        self.assertIn("operational max 2.5000 > hardware max 2.1467", stderr)
        self.assertIn("operational min -2.5000 < hardware min -2.1467", stderr)
        self.assertNotIn("joint_5_b has no parseable limits", stderr)
        self.assertNotIn("joint_5_b missing from operational_joint_limits", stderr)
        self.assertNotEqual(status, 0)


if __name__ == "__main__":
    unittest.main()
