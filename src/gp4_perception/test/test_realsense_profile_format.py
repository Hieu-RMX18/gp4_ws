"""Regression tests for RealSense launch profile string compatibility."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
PROFILE_VALUES = (
    "848,480,30",
    "1280,720,30",
)


def test_realsense_profiles_use_comma_format_in_gp4_perception_configs():
    """RealSense ROS Humble launch defaults expose stream profiles as W,H,FPS."""
    config_path = ROOT / "src" / "gp4_perception" / "config" / "d435i.yaml"
    launch_path = ROOT / "src" / "gp4_perception" / "launch" / "calibration_collect.launch.py"

    config_text = config_path.read_text()
    launch_text = launch_path.read_text()

    for profile_value in PROFILE_VALUES:
        assert profile_value in config_text or profile_value in launch_text

    assert "848x480x30" not in config_text
    assert "1280x720x30" not in config_text
    assert "848x480x30" not in launch_text
    assert "1280x720x30" not in launch_text


def test_d435i_yaml_keeps_expected_stream_profiles():
    """The documented D435i profile values should match the launchable format."""
    config_path = ROOT / "src" / "gp4_perception" / "config" / "d435i.yaml"
    data = yaml.safe_load(config_path.read_text())
    realsense = data["realsense"]

    assert realsense["depth_module"]["depth_profile"] == "848,480,30"
    assert realsense["depth_module"]["infra_profile"] == "848,480,30"
    assert realsense["rgb_camera"]["color_profile"] == "1280,720,30"
