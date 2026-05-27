"""Unit tests for the standalone calibration validation tool."""

import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
VALIDATOR_PATH = ROOT / "src" / "gp4_perception" / "tools" / "validate_calibration.py"


def _load_validate_calibration_module():
    spec = importlib.util.spec_from_file_location("validate_calibration", VALIDATOR_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_board_position_in_base_composes_camera_root_and_optical_frames():
    """OpenCV points are optical-frame points and must pass through camera_link."""
    validate_calibration = _load_validate_calibration_module()

    base_from_camera_link = np.eye(4)
    base_from_camera_link[:3, 3] = [1.0, 2.0, 3.0]
    camera_link_from_color_optical = np.eye(4)
    camera_link_from_color_optical[:3, 3] = [0.1, 0.2, 0.3]
    board_pos_color_optical = np.array([0.4, 0.5, 0.6])

    board_pos_base = validate_calibration._board_position_in_base(
        board_pos_color_optical,
        base_from_camera_link,
        camera_link_from_color_optical,
    )

    np.testing.assert_allclose(board_pos_base, [1.5, 2.7, 3.9])
