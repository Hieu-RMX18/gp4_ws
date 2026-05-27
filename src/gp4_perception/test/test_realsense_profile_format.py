"""Regression tests for RealSense launch profile and parameter compatibility."""

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
PROFILE_VALUES = (
    "640x480x15",
    "848x480x30",
    "1280x720x30",
)

def test_realsense_profiles_use_x_format_in_gp4_perception_configs():
    """RealSense ROS Humble launch profiles use WxHxFPS strings."""
    config_path = ROOT / "src" / "gp4_perception" / "config" / "d435i.yaml"
    launch_path = ROOT / "src" / "gp4_perception" / "launch" / "calibration_collect.launch.py"

    config_text = config_path.read_text()
    launch_text = launch_path.read_text()

    for profile_value in PROFILE_VALUES:
        assert profile_value in config_text or profile_value in launch_text

    assert "848,480,30" not in config_text
    assert "1280,720,30" not in config_text
    assert "848,480,30" not in launch_text
    assert "1280,720,30" not in launch_text

def test_d435i_yaml_keeps_expected_stream_profiles():
    """The documented D435i profile values should match the launchable format."""
    config_path = ROOT / "src" / "gp4_perception" / "config" / "d435i.yaml"
    data = yaml.safe_load(config_path.read_text())
    realsense = data["realsense"]

    assert realsense["depth_module"]["depth_profile"] == "848x480x30"
    assert realsense["depth_module"]["infra_profile"] == "848x480x30"
    assert realsense["rgb_camera"]["color_profile"] == "1280x720x30"

def test_realsense_launches_do_not_forward_wrapper_arguments_to_rs_launch():
    """Wrapper-only launch args should not be visible to RealSense rs_launch.py."""
    launch_paths = (
        ROOT / "src" / "gp4_perception" / "launch" / "camera.launch.py",
        ROOT / "src" / "gp4_perception" / "launch" / "calibration_collect.launch.py",
    )

    for launch_path in launch_paths:
        launch_text = launch_path.read_text()
        assert "IncludeLaunchDescription" not in launch_text
        assert "rs_launch.py" not in launch_text
        assert '"depth_qos"' not in launch_text
        assert '"color_qos"' not in launch_text
        assert '"depth_module.emitter_enabled"' not in launch_text

def test_realsense_launches_use_typed_boolean_parameters():
    """Launch files must not pass bool/double RealSense params as strings."""
    launch_paths = (
        ROOT / "src" / "gp4_perception" / "launch" / "camera.launch.py",
        ROOT / "src" / "gp4_perception" / "launch" / "calibration_collect.launch.py",
    )

    for launch_path in launch_paths:
        launch_text = launch_path.read_text()
        assert "ParameterValue" in launch_text
        assert '"enable_color": "true"' not in launch_text
        assert '"enable_depth": "true"' not in launch_text
        assert '"enable_infra1": "false"' not in launch_text
        assert '"enable_gyro": "false"' not in launch_text
        assert '"publish_tf": "true"' not in launch_text
        assert '"tf_publish_rate": "0.0"' not in launch_text

def test_full_perception_keeps_realsense_internal_tf_enabled():
    """Full perception should not disable RealSense internal camera TF frames."""
    camera_launch = (
        ROOT / "src" / "gp4_perception" / "launch" / "camera.launch.py"
    ).read_text()
    full_launch = (
        ROOT / "src" / "gp4_perception" / "launch" / "perception_full.launch.py"
    ).read_text()

    assert '"publish_tf": True' in camera_launch
    assert '"publish_tf": "false"' not in full_launch

def test_calibration_tools_use_10x11_charuco_board():
    """Preview and validation tools must match fiducials.yaml board geometry."""
    tool_paths = (
        ROOT / "src" / "gp4_perception" / "tools" / "camera_preview.py",
        ROOT / "src" / "gp4_perception" / "tools" / "validate_calibration.py",
    )

    for path in tool_paths:
        text = path.read_text()
        assert "_BOARD_ROWS = 8" not in text
        assert "charuco_8x11" not in text
        assert "board_rows" in text
        assert "board_columns" in text
