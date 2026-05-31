"""Test-only ROS environment for sandboxed QoS/node tests."""

import os
import tempfile
from pathlib import Path


_ros_log_dir = Path(tempfile.gettempdir()) / "gp4_perception_ros_test_logs"
_ros_log_dir.mkdir(parents=True, exist_ok=True)

os.environ["ROS_LOG_DIR"] = str(_ros_log_dir)
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
