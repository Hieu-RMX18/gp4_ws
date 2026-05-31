#!/usr/bin/env python3
"""Smoke tests for hardware bringup launch files.

Verifies launch-description parseability and pre-flight safety gates
without requiring a real robot or micro-ROS Agent.
"""

import importlib.util
import os
import sys
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_DIR = REPO_ROOT / "src" / "gp4_bringup" / "launch"

ROS_AVAILABLE = bool(os.environ.get("AMENT_PREFIX_PATH"))
REASON_SKIP = "ROS environment not sourced"


def _load_launch_module(name: str):
    path = LAUNCH_DIR / f"{name}.launch.py"
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _loginfo_text(entity):
    """Extract plain text from a launch LogInfo entity (msg is a list of substitutions)."""
    msg = getattr(entity, "msg", "")
    if isinstance(msg, list):
        parts = []
        for sub in msg:
            text = getattr(sub, "text", None)
            if text is not None:
                parts.append(text)
            else:
                parts.append(str(sub))
        return "".join(parts)
    return str(msg)


@unittest.skipUnless(ROS_AVAILABLE, REASON_SKIP)
class TestHwLaunchSmoke(unittest.TestCase):
    def test_hw_launch_check_rmw_rejects_wrong_impl(self):
        """V4 A8: wrong RMW_IMPLEMENTATION must trigger shutdown."""
        hw_launch = _load_launch_module("hw")

        with mock.patch.dict(
            os.environ, {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
        ):
            result = hw_launch._check_rmw_and_agent(None)

        shut_down_found = False
        for entity in result:
            if type(entity).__name__ == "EmitEvent":
                event = getattr(entity, "event", None)
                if event and type(event).__name__ == "Shutdown":
                    shut_down_found = True
                    break
        self.assertTrue(
            shut_down_found,
            msg=f"Expected Shutdown event for wrong RMW, got: {result}",
        )

    def test_hw_launch_check_rmw_accepts_fastrtps(self):
        """V4 A8: correct RMW must pass through."""
        hw_launch = _load_launch_module("hw")

        with mock.patch.dict(
            os.environ, {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"}
        ), mock.patch(
            "subprocess.run", return_value=mock.Mock()
        ):
            result = hw_launch._check_rmw_and_agent(None)

        pass_found = any(
            type(entity).__name__ == "LogInfo"
            and "passed" in _loginfo_text(entity)
            for entity in result
        )
        self.assertTrue(
            pass_found,
            msg=f"Expected pass message for correct RMW, got: {result}",
        )

    def test_hw_launch_parses(self):
        """hw.launch.py must generate a LaunchDescription."""
        hw_launch = _load_launch_module("hw")

        ld = hw_launch.generate_launch_description()
        self.assertIsNotNone(ld)
        from launch.actions import DeclareLaunchArgument

        arg_names = {
            e.name
            for e in ld.entities
            if isinstance(e, DeclareLaunchArgument)
        }
        self.assertIn("robot_ip", arg_names)
        self.assertIn("agent_ip", arg_names)
        self.assertIn("use_rviz", arg_names)

    def test_system_launch_parses(self):
        """system.launch.py must generate a LaunchDescription."""
        system_launch = _load_launch_module("system")

        ld = system_launch.generate_launch_description()
        self.assertIsNotNone(ld)
        from launch.actions import IncludeLaunchDescription

        includes = [
            e for e in ld.entities if isinstance(e, IncludeLaunchDescription)
        ]
        self.assertEqual(
            len(includes), 3, "Expected sim, hw, llm_stack includes"
        )


if __name__ == "__main__":
    unittest.main()
