"""Contract tests for the unified visualizer rollout.

The unified OpenCV window is the default path, but the legacy visualizer entry
points stay available so existing launch scripts and operator habits do not
break during the transition.
"""

from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src" / "gp4_perception"


def test_legacy_visualizer_files_remain_for_backward_compatibility():
    assert (PKG / "gp4_perception" / "detection_visualizer.py").exists()
    assert (PKG / "gp4_perception" / "preprocessing_visualizer.py").exists()


def test_console_scripts_keep_legacy_and_add_unified_entrypoint():
    setup_text = (PKG / "setup.py").read_text()

    assert "unified_visualizer = gp4_perception.unified_visualizer:main" in setup_text
    assert "detection_visualizer = gp4_perception.detection_visualizer:main" in setup_text
    assert (
        "preprocessing_visualizer = gp4_perception.preprocessing_visualizer:main"
        in setup_text
    )


def test_full_perception_launch_defaults_to_unified_with_legacy_fallback():
    launch_text = (PKG / "launch" / "perception_full.launch.py").read_text()

    assert "DeclareLaunchArgument" in launch_text
    assert '"use_unified_gui"' in launch_text
    assert 'default_value="true"' in launch_text
    assert 'executable="unified_visualizer"' in launch_text
    assert 'executable="detection_visualizer"' in launch_text
    assert 'executable="preprocessing_visualizer"' in launch_text
    assert "IfCondition(LaunchConfiguration(\"use_unified_gui\"))" in launch_text
    assert "UnlessCondition(LaunchConfiguration(\"use_unified_gui\"))" in launch_text


def test_visualization_config_sets_frame_rate_and_bandwidth_limits():
    cfg = yaml.safe_load((PKG / "config" / "perception.yaml").read_text())
    viz = cfg["perception"]["visualization"]

    assert viz["max_process_fps"] == 30.0
    assert viz["max_annotated_width_px"] == 960
    assert viz["qos"]["debug_images"]["reliability"] == "best_effort"


def test_unified_visualizer_uses_bounded_rate_and_qos_contract():
    source = (PKG / "gp4_perception" / "unified_visualizer.py").read_text()

    assert "from gp4_perception.latest_frame import MonotonicRateGate" in source
    assert "self._process_gate = MonotonicRateGate(self._max_process_fps)" in source
    assert "if self._processing:" in source
    assert "if not self._process_gate.allow(time.monotonic()):" in source
    assert "queue_size=2" in source
    assert "ann_qos = QoSProfile(" in source
    assert 'self.create_publisher(Image, "/perception/annotated_image", ann_qos)' in source
    assert "_downscale_to_max_width(top_row, self._max_annotated_width)" in source
