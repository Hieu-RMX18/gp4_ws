"""Pure-function tests for the RGB + aligned-depth detection path.

These avoid rclpy: they exercise the deprojection math, median-depth over a
mask, metric bbox sizing, HSV detection, and border-hue checks directly.
"""

from __future__ import annotations

import numpy as np
import pytest

from gp4_perception.detection_visualizer import (
    _bbox_size_m,
    _deproject_pixel,
    _median_depth_m,
)


class TestDeprojectPixel:
    def test_center_pixel_maps_to_optical_axis(self):
        # u==cx, v==cy => x=y=0, z=depth
        x, y, z = _deproject_pixel(320.0, 240.0, 0.5, fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)
        assert z == pytest.approx(0.5)

    def test_offset_pixel_uses_pinhole_model(self):
        # X = (u-cx) Z / fx ; Y = (v-cy) Z / fy
        x, y, z = _deproject_pixel(420.0, 140.0, 0.8, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
        assert x == pytest.approx((420.0 - 320.0) * 0.8 / 500.0)
        assert y == pytest.approx((140.0 - 240.0) * 0.8 / 500.0)
        assert z == pytest.approx(0.8)


class TestMedianDepthM:
    def test_median_over_valid_masked_pixels_scaled_to_metres(self):
        depth = np.array([[1000, 1000], [2000, 0]], dtype=np.uint16)  # raw mm
        mask = np.array([[255, 255], [255, 255]], dtype=np.uint8)
        # valid nonzero raw = [1000, 1000, 2000]; median 1000 -> 1.0 m
        assert _median_depth_m(depth, mask, depth_scale=0.001) == pytest.approx(1.0)

    def test_ignores_zero_and_unmasked_pixels(self):
        depth = np.array([[0, 500], [3000, 700]], dtype=np.uint16)
        mask = np.array([[255, 255], [0, 255]], dtype=np.uint8)
        # masked & nonzero raw = [500, 700]; median 600 -> 0.6 m
        assert _median_depth_m(depth, mask, depth_scale=0.001) == pytest.approx(0.6)

    def test_returns_none_when_all_invalid(self):
        depth = np.zeros((2, 2), dtype=np.uint16)
        mask = np.full((2, 2), 255, dtype=np.uint8)
        assert _median_depth_m(depth, mask, depth_scale=0.001) is None

    def test_returns_none_when_mask_empty(self):
        depth = np.full((2, 2), 1000, dtype=np.uint16)
        mask = np.zeros((2, 2), dtype=np.uint8)
        assert _median_depth_m(depth, mask, depth_scale=0.001) is None


class TestBboxSizeM:
    def test_pixel_extent_scales_by_depth_over_focal(self):
        sx, sy = _bbox_size_m(100, 50, 0.5, fx=500.0, fy=500.0)
        assert sx == pytest.approx(100 * 0.5 / 500.0)
        assert sy == pytest.approx(50 * 0.5 / 500.0)


# ---------------------------------------------------------------------------
# Task 1: Verify color_classes + rgb_detector config
# ---------------------------------------------------------------------------
class TestColorClassesConfig:
    def test_config_loads_color_classes(self):
        from pathlib import Path
        import yaml

        cfg_path = Path(__file__).resolve().parents[1] / "config" / "perception.yaml"
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        classes = cfg["perception"]["color_classes"]
        assert len(classes) >= 4
        red = next(c for c in classes if c["class_id"] == "red_box")
        assert red["enabled"] is True
        assert len(red["hsv_ranges"]) == 2  # red wraparound
        assert red["min_area_px"] > 0

    def test_config_loads_rgb_detector(self):
        from pathlib import Path
        import yaml

        cfg_path = Path(__file__).resolve().parents[1] / "config" / "perception.yaml"
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        det = cfg["perception"]["rgb_detector"]
        assert det["depth_scale_m"] == 0.001
        assert "base_frame" in det
        assert "camera_optical_frame" in det


# ---------------------------------------------------------------------------
# Task 2: Edge-case tests for pure helpers
# ---------------------------------------------------------------------------
class TestDeprojectEdgeCases:
    def test_zero_depth_returns_zero_xyz(self):
        x, y, z = _deproject_pixel(400.0, 300.0, 0.0, fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        assert z == 0.0
        assert x == 0.0
        assert y == 0.0


class TestMedianDepthEdgeCases:
    def test_returns_none_when_depth_is_none(self):
        assert _median_depth_m(None, np.ones((2, 2), dtype=np.uint8)) is None

    def test_returns_none_when_mask_is_none(self):
        assert _median_depth_m(np.ones((2, 2), dtype=np.uint16), None) is None

    def test_single_valid_pixel(self):
        depth = np.array([[500, 0], [0, 0]], dtype=np.uint16)
        mask = np.array([[255, 0], [0, 0]], dtype=np.uint8)
        assert _median_depth_m(depth, mask) == pytest.approx(0.5)
