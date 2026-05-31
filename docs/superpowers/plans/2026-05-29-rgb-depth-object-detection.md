# RGB + Aligned-Depth Object Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `detection_visualizer.py` the owner of an RGB-detect → aligned-depth-median → deproject → TF → temporal-track → publish path, and demote the pointcloud `scene_processor.py` to debug/fallback by stopping its `/perception/detections` publish.

**Architecture:** Sync `/camera/color/image_raw` + `/camera/aligned_depth_to_color/image_raw` with cached `/camera/color/camera_info`. Per configured class: HSV mask → morphology → contour bbox → median depth over mask → deproject bbox center to `camera_color_optical_frame` → TF `PointStamped` to `base_link` (fallback to camera frame) → reuse `TemporalTracker` → publish `vision_msgs/Detection3DArray` + a side-by-side RGB|depth-colormap `/perception/annotated_image`.

**Tech Stack:** ROS 2 Humble (rclpy), `message_filters.ApproximateTimeSynchronizer`, OpenCV (`cv2`), NumPy, `cv_bridge`, `tf2_ros` + `tf2_geometry_msgs`, `vision_msgs`. Tests: pytest (pure functions, no ROS spin).

---

## Environment

All commands run from `~/gp4_ws` with the project venv active:

```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
```

Pure-function tests run without a ROS build (cv2 + numpy only). The colcon build/test at the end validates the full node.

## Decisions locked during planning

- **GUI removed:** the old node opened an OpenCV window (`cv2.namedWindow`/`imshow`/`waitKey`) and a "BBox Filter" trackbar that toggled `scene_processor` via `/scene_processor/set_parameters`. The approved design's keep-list (`CvBridge`/TF/camera_info/annotated-pub/`_color_for_class`) omits these, so the rewrite drops the window, trackbar, and `SetParameters` client. The node becomes publish-only (headless-safe). **If the bbox-filter remote toggle must survive, stop and re-scope before Task 5.**
- **Border reuse:** `scene_geometry._has_chromatic_border` consumes Nx3 pointcloud RGB pixels and cannot run on a 2D mask. A small 2D analog (`_border_dominant_hue`) is added instead. `white_workpiece` ships **disabled** in config and is the only class needing it.
- **Detection3D shape:** built to match what `TemporalTracker` and downstream consumers already read: `det.results[0].hypothesis.class_id`, `det.results[0].hypothesis.score`, `det.results[0].pose.pose.position`, `det.bbox.size.{x,y,z}`.

## File structure

| File | Responsibility after this plan |
|------|-------------------------------|
| `src/gp4_perception/config/perception.yaml` | + `color_classes` table (HSV/morph/min_area/require_border per class), + `rgb_detector` topic names, + `visualization.show_depth_panel`. |
| `src/gp4_perception/gp4_perception/detection_visualizer.py` | Rewritten: module-level pure helpers (`_deproject_pixel`, `_median_depth_m`, `_bbox_size_m`, `_detect_in_ranges`, `_border_dominant_hue`) + `DetectionVisualizer` node owning the RGB+depth path. Keeps `_color_for_class`. Deletes `_build_3d_bbox_corners`, `_project_points_to_image`, the `Detection3DArray` subscription, and the OpenCV GUI. |
| `src/gp4_perception/gp4_perception/scene_processor.py` | Remove `_det_pub` publisher + the detection-publish lines in `_publish_detections`; keep the collision-removal loop, `GetObjectPositions`, `/perception/debug_clusters`. |
| `src/gp4_perception/test/test_detection_visualizer.py` | **New.** Pure-function tests: deproject math, median-depth (incl. invalid/zero), bbox metric size, HSV detection, border hue. |
| `src/gp4_perception/test/test_scene_processor.py` | Update the 2 tests that set `processor._det_pub` so they no longer reference a removed attribute. |

---

## Task 1: Add RGB detector config to perception.yaml

**Files:**
- Modify: `src/gp4_perception/config/perception.yaml`

- [ ] **Step 1: Append the new config blocks under the existing `perception:` mapping**

Add these keys at the end of the `perception:` block (after `depth_noise:` … line 37), keeping 2-space indentation so they are siblings of `voxel_size_m`:

```yaml
  # ---- RGB-first detector (detection_visualizer.py owns /perception/detections) ----
  rgb_detector:
    color_topic: /camera/color/image_raw
    depth_topic: /camera/aligned_depth_to_color/image_raw
    camera_info_topic: /camera/color/camera_info
    depth_scale_m: 0.001          # 16UC1 raw units -> metres
    sync_slop_s: 0.05
    sync_queue: 10
    base_frame: base_link
    camera_optical_frame: camera_color_optical_frame
    bbox_thickness_z_m: 0.03      # nominal Z extent (single deprojected point has none)
  visualization:
    show_depth_panel: true
    output_width_px: 960          # combined image downscaled to this width before publish
  # HSV mask source of truth for the 2D path. Ranges are [h_min,s_min,v_min,h_max,s_max,v_max]
  # in OpenCV HSV (H 0-179). Red wraps 0/179 so it carries two ranges.
  color_classes:
    - class_id: red_box
      enabled: true
      hsv_ranges:
        - [0, 90, 60, 10, 255, 255]
        - [170, 90, 60, 179, 255, 255]
      min_area_px: 400
      morph_kernel: 3
      require_border: null
    - class_id: yellow_ball
      enabled: true
      hsv_ranges:
        - [22, 90, 80, 35, 255, 255]
      min_area_px: 300
      morph_kernel: 3
      require_border: null
    - class_id: apple
      enabled: true
      hsv_ranges:
        - [0, 90, 60, 10, 255, 255]
        - [170, 90, 60, 179, 255, 255]
      min_area_px: 300
      morph_kernel: 3
      require_border: null
    - class_id: orange
      enabled: true
      hsv_ranges:
        - [10, 110, 90, 22, 255, 255]
      min_area_px: 300
      morph_kernel: 3
      require_border: null
    - class_id: white_workpiece
      enabled: false              # needs blue-border validation; enable after tuning
      hsv_ranges:
        - [0, 0, 180, 179, 40, 255]
      min_area_px: 500
      morph_kernel: 5
      require_border: blue
```

- [ ] **Step 2: Verify the YAML parses**

Run:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
python -c "import yaml,sys; d=yaml.safe_load(open('src/gp4_perception/config/perception.yaml')); c=d['perception']['color_classes']; print('classes:', [x['class_id'] for x in c]); print('enabled:', [x['class_id'] for x in c if x['enabled']]); print('show_depth_panel:', d['perception']['visualization']['show_depth_panel'])"
```
Expected:
```
classes: ['red_box', 'yellow_ball', 'apple', 'orange', 'white_workpiece']
enabled: ['red_box', 'yellow_ball', 'apple', 'orange']
show_depth_panel: True
```

- [ ] **Step 3: Commit**

```bash
git add src/gp4_perception/config/perception.yaml
git commit -m "feat(perception): add color_classes + rgb_detector config for 2D detection path"
```

---

## Task 2: Pure deprojection + median-depth + bbox-size helpers (TDD)

**Files:**
- Create: `src/gp4_perception/test/test_detection_visualizer.py`
- Modify: `src/gp4_perception/gp4_perception/detection_visualizer.py` (add module-level functions)

- [ ] **Step 1: Write the failing tests**

Create `src/gp4_perception/test/test_detection_visualizer.py`:

```python
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
```

- [ ] **Step 2: Run the tests to verify they fail (import error)**

Run:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
cd src/gp4_perception && python -m pytest test/test_detection_visualizer.py -v
```
Expected: collection/import error — `cannot import name '_deproject_pixel'` (functions not defined yet).

- [ ] **Step 3: Add the three pure helpers near the top of `detection_visualizer.py`**

Insert after the `_color_for_class` function (replacing the soon-to-be-deleted `_build_3d_bbox_corners`/`_project_points_to_image` is done in Task 5; for now just add these new functions above the class):

```python
def _deproject_pixel(
    u: float, v: float, z_m: float, fx: float, fy: float, cx: float, cy: float
) -> tuple[float, float, float]:
    """Pinhole deprojection of one pixel to camera_color_optical_frame metres."""
    x = (u - cx) * z_m / fx
    y = (v - cy) * z_m / fy
    return (x, y, z_m)


def _median_depth_m(
    depth_raw: np.ndarray | None,
    mask: np.ndarray | None,
    depth_scale: float = 0.001,
) -> float | None:
    """Median depth in metres over masked, nonzero pixels.

    Returns None when no valid (masked & nonzero) pixel exists.
    """
    if depth_raw is None or mask is None:
        return None
    valid = (mask > 0) & (depth_raw > 0)
    sel = depth_raw[valid]
    if sel.size == 0:
        return None
    return float(np.median(sel)) * depth_scale


def _bbox_size_m(
    w_px: float, h_px: float, z_m: float, fx: float, fy: float
) -> tuple[float, float]:
    """Estimate metric bbox width/height from pixel extent at depth z."""
    return (w_px * z_m / fx, h_px * z_m / fy)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
cd src/gp4_perception && python -m pytest test/test_detection_visualizer.py -v
```
Expected: all tests in `TestDeprojectPixel`, `TestMedianDepthM`, `TestBboxSizeM` PASS.

- [ ] **Step 5: Commit**

```bash
git add src/gp4_perception/test/test_detection_visualizer.py src/gp4_perception/gp4_perception/detection_visualizer.py
git commit -m "feat(perception): pure deproject/median-depth/bbox-size helpers + tests"
```

---

## Task 3: HSV multi-range detection helper (TDD)

**Files:**
- Modify: `src/gp4_perception/test/test_detection_visualizer.py`
- Modify: `src/gp4_perception/gp4_perception/detection_visualizer.py`

- [ ] **Step 1: Add the failing test**

Append to `test/test_detection_visualizer.py`:

```python
import cv2  # noqa: E402

from gp4_perception.detection_visualizer import _detect_in_ranges  # noqa: E402


class TestDetectInRanges:
    def _red_square_bgr(self) -> np.ndarray:
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        img[30:70, 40:90] = (0, 0, 255)  # BGR red, 40x50 block
        return img

    def test_finds_red_block_bbox_above_min_area(self):
        bgr = self._red_square_bgr()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        ranges = [(0, 90, 60, 10, 255, 255), (170, 90, 60, 179, 255, 255)]
        dets = _detect_in_ranges(hsv, ranges, min_area_px=200, morph_kernel=3)
        assert len(dets) >= 1
        x, y, w, h = dets[0]["bbox"]
        # bbox should roughly bound the 40x90 / 30x70 block
        assert 35 <= x <= 45 and 25 <= y <= 35
        assert 45 <= w <= 55 and 35 <= h <= 45
        assert dets[0]["area"] >= 200

    def test_rejects_below_min_area(self):
        bgr = self._red_square_bgr()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        ranges = [(0, 90, 60, 10, 255, 255), (170, 90, 60, 179, 255, 255)]
        dets = _detect_in_ranges(hsv, ranges, min_area_px=100000, morph_kernel=3)
        assert dets == []

    def test_returns_largest_first(self):
        bgr = np.zeros((120, 200, 3), dtype=np.uint8)
        bgr[10:30, 10:30] = (0, 0, 255)    # small 20x20
        bgr[50:110, 80:180] = (0, 0, 255)  # large 60x100
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        ranges = [(0, 90, 60, 10, 255, 255), (170, 90, 60, 179, 255, 255)]
        dets = _detect_in_ranges(hsv, ranges, min_area_px=50, morph_kernel=3)
        assert len(dets) == 2
        assert dets[0]["area"] > dets[1]["area"]
```

- [ ] **Step 2: Run to verify failure**

Run:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
cd src/gp4_perception && python -m pytest test/test_detection_visualizer.py::TestDetectInRanges -v
```
Expected: import error `cannot import name '_detect_in_ranges'`.

- [ ] **Step 3: Implement `_detect_in_ranges`**

Add below `_bbox_size_m` in `detection_visualizer.py`:

```python
def _detect_in_ranges(
    hsv: np.ndarray,
    ranges: list[tuple[int, int, int, int, int, int]],
    min_area_px: float,
    morph_kernel: int = 3,
) -> list[dict]:
    """OR a set of HSV ranges into one mask, clean it, and return bbox detections.

    Each range is (h_min, s_min, v_min, h_max, s_max, v_max) in OpenCV HSV.
    Returns dicts {"bbox": (x, y, w, h), "area": float, "mask": ndarray}
    for contours with area >= min_area_px, largest area first. "mask" is the
    full-frame cleaned mask (used later for median depth inside the bbox).
    """
    combined = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for h0, s0, v0, h1, s1, v1 in ranges:
        lower = np.array([h0, s0, v0], dtype=np.uint8)
        upper = np.array([h1, s1, v1], dtype=np.uint8)
        combined |= cv2.inRange(hsv, lower, upper)

    if morph_kernel and morph_kernel > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel)
        )
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    dets: list[dict] = []
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < min_area_px:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        dets.append({"bbox": (x, y, w, h), "area": area, "mask": combined})
    dets.sort(key=lambda d: d["area"], reverse=True)
    return dets
```

- [ ] **Step 4: Run to verify pass**

Run:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
cd src/gp4_perception && python -m pytest test/test_detection_visualizer.py::TestDetectInRanges -v
```
Expected: all 3 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/gp4_perception/test/test_detection_visualizer.py src/gp4_perception/gp4_perception/detection_visualizer.py
git commit -m "feat(perception): HSV multi-range contour detection helper + tests"
```

---

## Task 4: 2D border-hue helper for require_border classes (TDD)

**Files:**
- Modify: `src/gp4_perception/test/test_detection_visualizer.py`
- Modify: `src/gp4_perception/gp4_perception/detection_visualizer.py`

- [ ] **Step 1: Add the failing test**

Append to `test/test_detection_visualizer.py`:

```python
from gp4_perception.detection_visualizer import _border_dominant_hue  # noqa: E402

# OpenCV HSV range for "blue" used by white_workpiece require_border.
_BLUE_RANGE = [(100, 90, 60, 130, 255, 255)]


class TestBorderDominantHue:
    def test_blue_ring_around_white_center_is_detected(self):
        bgr = np.full((100, 100, 3), 255, dtype=np.uint8)  # white fill
        bgr[0:100, 0:10] = (255, 0, 0)    # BGR blue left ring
        bgr[0:100, 90:100] = (255, 0, 0)  # blue right ring
        bgr[0:10, 0:100] = (255, 0, 0)    # top
        bgr[90:100, 0:100] = (255, 0, 0)  # bottom
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        assert _border_dominant_hue(hsv, (0, 0, 100, 100), _BLUE_RANGE, ring_frac=0.15) is True

    def test_all_white_has_no_blue_border(self):
        bgr = np.full((100, 100, 3), 255, dtype=np.uint8)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        assert _border_dominant_hue(hsv, (0, 0, 100, 100), _BLUE_RANGE, ring_frac=0.15) is False
```

- [ ] **Step 2: Run to verify failure**

Run:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
cd src/gp4_perception && python -m pytest test/test_detection_visualizer.py::TestBorderDominantHue -v
```
Expected: import error `cannot import name '_border_dominant_hue'`.

- [ ] **Step 3: Implement `_border_dominant_hue`**

Add below `_detect_in_ranges`:

```python
def _border_dominant_hue(
    hsv: np.ndarray,
    bbox: tuple[int, int, int, int],
    ranges: list[tuple[int, int, int, int, int, int]],
    ring_frac: float = 0.15,
    min_ratio: float = 0.15,
) -> bool:
    """True when the outer ring of `bbox` is dominated by the given HSV ranges.

    2D analog of scene_geometry._has_chromatic_border (which is pointcloud-only).
    Used to validate require_border classes (e.g. white_workpiece -> blue).
    """
    x, y, w, h = bbox
    roi = hsv[y : y + h, x : x + w]
    if roi.size == 0:
        return False
    border_w = max(1, int(round(min(w, h) * ring_frac)))
    ring = np.zeros(roi.shape[:2], dtype=bool)
    ring[:border_w, :] = True
    ring[-border_w:, :] = True
    ring[:, :border_w] = True
    ring[:, -border_w:] = True
    ring_px = int(ring.sum())
    if ring_px == 0:
        return False

    match = np.zeros(roi.shape[:2], dtype=np.uint8)
    for h0, s0, v0, h1, s1, v1 in ranges:
        lower = np.array([h0, s0, v0], dtype=np.uint8)
        upper = np.array([h1, s1, v1], dtype=np.uint8)
        match |= cv2.inRange(roi, lower, upper)
    match_in_ring = int(((match > 0) & ring).sum())
    return (match_in_ring / ring_px) >= min_ratio
```

- [ ] **Step 4: Run to verify pass**

Run:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
cd src/gp4_perception && python -m pytest test/test_detection_visualizer.py -v
```
Expected: every test in the file PASSES (deproject, median-depth, bbox-size, detect, border).

- [ ] **Step 5: Commit**

```bash
git add src/gp4_perception/test/test_detection_visualizer.py src/gp4_perception/gp4_perception/detection_visualizer.py
git commit -m "feat(perception): 2D border-hue validation helper for require_border classes + tests"
```

---

## Task 5: Rewrite the DetectionVisualizer node around the RGB+depth path

**Files:**
- Modify: `src/gp4_perception/gp4_perception/detection_visualizer.py`

This replaces the node class, the module docstring/imports, deletes `_build_3d_bbox_corners`, `_project_points_to_image`, the `Detection3DArray` subscription, and the OpenCV GUI/trackbar/SetParameters client. The pure helpers from Tasks 2–4 stay. There is no unit test that spins ROS; correctness of the node is verified by the colcon build + import smoke check here and the live acceptance checks in Task 7.

- [ ] **Step 1: Replace the module docstring and import block (lines 1–37)**

Replace the file header (docstring through `_LOGGER = ...`) with:

```python
"""RGB-first object detector with aligned-depth XYZ.

Owns /perception/detections. Syncs color + aligned-depth images, masks each
configured class in HSV, takes median depth over the mask, deprojects the bbox
center to camera_color_optical_frame, transforms to base_link via TF (falling
back to the camera frame), stabilises with TemporalTracker, and publishes both
a vision_msgs/Detection3DArray and a side-by-side RGB|depth annotated image.

Subscribes:
    /camera/color/image_raw                     (Image, bgr8)
    /camera/aligned_depth_to_color/image_raw    (Image, 16UC1)
    /camera/color/camera_info                   (CameraInfo)

Publishes:
    /perception/detections          (vision_msgs/Detection3DArray)
    /perception/annotated_image     (Image, bgr8)

Entry point: ros2 run gp4_perception detection_visualizer
"""

from __future__ import annotations

import logging
import os
import sys

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, Pose
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

import tf2_geometry_msgs  # noqa: F401  (registers PointStamped do_transform)

from gp4_perception.temporal_tracker import TemporalTracker

_LOGGER = logging.getLogger(__name__)
```

- [ ] **Step 2: Keep `_color_for_class` and the `_COLORS`/`_DEFAULT_COLOR` block; delete `_build_3d_bbox_corners` and `_project_points_to_image`**

Remove the two functions (`_build_3d_bbox_corners`, `_project_points_to_image`) entirely. Remove the now-unused `_WINDOW_NAME` constant and the `from scipy.spatial.transform import Rotation` import (deleted in Step 1's import block). Keep `_COLORS`, `_DEFAULT_COLOR`, `_color_for_class`, and the Task 2–4 helpers.

- [ ] **Step 3: Add a config loader helper above the class**

```python
def _load_perception_config() -> dict:
    """Load perception.yaml from the installed package share dir."""
    share = get_package_share_directory("gp4_perception")
    path = os.path.join(share, "config", "perception.yaml")
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return data.get("perception", {})
```

- [ ] **Step 4: Replace the entire `DetectionVisualizer` class with the RGB+depth implementation**

```python
class DetectionVisualizer(Node):
    """RGB-first detector: HSV mask -> median depth -> deproject -> TF -> publish."""

    def __init__(self) -> None:
        super().__init__("detection_visualizer")

        cfg = _load_perception_config()
        rgb = cfg.get("rgb_detector", {})
        vis = cfg.get("visualization", {})
        self._classes = [c for c in cfg.get("color_classes", []) if c.get("enabled")]

        self._depth_scale = float(rgb.get("depth_scale_m", 0.001))
        self._base_frame = rgb.get("base_frame", "base_link")
        self._optical_frame = rgb.get("camera_optical_frame", "camera_color_optical_frame")
        self._bbox_z = float(rgb.get("bbox_thickness_z_m", 0.03))
        self._show_depth = bool(vis.get("show_depth_panel", True))
        self._out_width = int(vis.get("output_width_px", 960))

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._camera_matrix: np.ndarray | None = None

        self._tracker = TemporalTracker(
            window_frames=int(cfg.get("temporal_window_frames", 5)),
            min_hits=int(cfg.get("temporal_min_hits", 3)),
            jitter_max_mm=float(cfg.get("centroid_jitter_max_mm", 15.0)),
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            CameraInfo,
            rgb.get("camera_info_topic", "/camera/color/camera_info"),
            self._on_camera_info,
            qos,
        )
        self._color_sub = Subscriber(
            self, Image, rgb.get("color_topic", "/camera/color/image_raw"), qos_profile=qos
        )
        self._depth_sub = Subscriber(
            self,
            Image,
            rgb.get("depth_topic", "/camera/aligned_depth_to_color/image_raw"),
            qos_profile=qos,
        )
        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub],
            queue_size=int(rgb.get("sync_queue", 10)),
            slop=float(rgb.get("sync_slop_s", 0.05)),
        )
        self._sync.registerCallback(self._on_synced)

        self._det_pub = self.create_publisher(Detection3DArray, "/perception/detections", 10)
        self._annotated_pub = self.create_publisher(Image, "/perception/annotated_image", 10)

        self.get_logger().info(
            "DetectionVisualizer (RGB+depth) up: %d active classes",
            len(self._classes),
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_synced(self, color_msg: Image, depth_msg: Image) -> None:
        if self._camera_matrix is None:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_raw = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"CvBridge conversion failed: {exc}")
            return

        timer = self.get_clock().now()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        fx, fy = self._camera_matrix[0, 0], self._camera_matrix[1, 1]
        cx, cy = self._camera_matrix[0, 2], self._camera_matrix[1, 2]

        tf_ok, transform = self._lookup_tf(color_msg.header.stamp)

        raw_detections: list[Detection3D] = []
        overlay_items: list[dict] = []  # for drawing even when not published

        for cls in self._classes:
            ranges = [tuple(r) for r in cls.get("hsv_ranges", [])]
            found = _detect_in_ranges(
                hsv,
                ranges,
                float(cls.get("min_area_px", 300)),
                int(cls.get("morph_kernel", 3)),
            )
            if not found:
                continue
            det = found[0]  # largest contour for this class
            x, y, w, h = det["bbox"]

            require = cls.get("require_border")
            if require:
                border_ranges = self._border_ranges_for(require)
                if border_ranges and not _border_dominant_hue(hsv, det["bbox"], border_ranges):
                    continue

            # Median depth over the class mask, restricted to this bbox.
            mask_roi = det["mask"][y : y + h, x : x + w]
            depth_roi = depth_raw[y : y + h, x : x + w]
            z_m = _median_depth_m(depth_roi, mask_roi, self._depth_scale)

            u_c, v_c = x + w / 2.0, y + h / 2.0
            item = {
                "class_id": cls["class_id"],
                "bbox": det["bbox"],
                "z_m": z_m,
                "cam_xyz": None,
                "base_xyz": None,
                "tf_ok": tf_ok,
            }

            if z_m is None:
                overlay_items.append(item)
                continue  # XYZ_INVALID -> draw only, do not publish

            cam_xyz = _deproject_pixel(u_c, v_c, z_m, fx, fy, cx, cy)
            item["cam_xyz"] = cam_xyz

            pub_xyz, frame = cam_xyz, self._optical_frame
            if tf_ok:
                base_xyz = self._transform_point(cam_xyz, transform)
                if base_xyz is not None:
                    item["base_xyz"] = base_xyz
                    pub_xyz, frame = base_xyz, self._base_frame

            sx, sy = _bbox_size_m(w, h, z_m, fx, fy)
            raw_detections.append(
                self._make_detection(cls["class_id"], pub_xyz, (sx, sy, self._bbox_z))
            )
            overlay_items.append(item)

        stable = self._tracker.update(raw_detections)
        self._publish_detections(stable, tf_ok)

        elapsed_ms = (self.get_clock().now() - timer).nanoseconds / 1e6
        annotated = self._render_overlay(bgr, depth_raw, overlay_items, len(stable), elapsed_ms)
        self._publish_annotated(annotated, color_msg.header)

    # -- helpers ---------------------------------------------------------

    def _border_ranges_for(self, color_name: str) -> list[tuple]:
        # Built-in HSV ranges for border-color validation (require_border).
        table = {
            "blue": [(100, 90, 60, 130, 255, 255)],
            "red": [(0, 90, 60, 10, 255, 255), (170, 90, 60, 179, 255, 255)],
        }
        return table.get(color_name, [])

    def _lookup_tf(self, stamp):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._optical_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            return True, tf
        except Exception:  # noqa: BLE001
            return False, None

    def _transform_point(self, cam_xyz, transform):
        try:
            ps = PointStamped()
            ps.header.frame_id = self._optical_frame
            ps.point = Point(x=float(cam_xyz[0]), y=float(cam_xyz[1]), z=float(cam_xyz[2]))
            out = tf2_geometry_msgs.do_transform_point(ps, transform)
            return (out.point.x, out.point.y, out.point.z)
        except Exception:  # noqa: BLE001
            return None

    def _make_detection(self, class_id, xyz, size_xyz) -> Detection3D:
        det = Detection3D()
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_id
        hyp.hypothesis.score = 1.0
        hyp.pose.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        hyp.pose.pose.orientation.w = 1.0
        det.results = [hyp]
        bbox = BoundingBox3D()
        bbox.center.position = hyp.pose.pose.position
        bbox.center.orientation.w = 1.0
        bbox.size.x, bbox.size.y, bbox.size.z = (
            float(size_xyz[0]),
            float(size_xyz[1]),
            float(size_xyz[2]),
        )
        det.bbox = bbox
        return det

    def _publish_detections(self, stable, tf_ok) -> None:
        arr = Detection3DArray()
        frame = self._base_frame if tf_ok else self._optical_frame
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = frame
        for det, score in stable:
            det.results[0].hypothesis.score = float(score)
            arr.detections.append(det)
        self._det_pub.publish(arr)

    def _render_overlay(self, bgr, depth_raw, items, n_stable, elapsed_ms):
        rgb_panel = bgr.copy()
        for it in items:
            x, y, w, h = it["bbox"]
            color = _color_for_class(it["class_id"])
            cv2.rectangle(rgb_panel, (x, y), (x + w, y + h), color, 2)
            lines = [it["class_id"]]
            if it["z_m"] is None:
                lines.append("XYZ_INVALID")
            else:
                lines.append(f"dist {it['z_m']:.3f} m")
                cxw, cyw, czw = it["cam_xyz"]
                lines.append(f"cam {cxw:.3f},{cyw:.3f},{czw:.3f}")
                if it["base_xyz"] is not None:
                    bx, by, bz = it["base_xyz"]
                    lines.append(f"base {bx:.3f},{by:.3f},{bz:.3f}")
                else:
                    lines.append("base TF_UNAVAILABLE")
            ly = max(y - 8, 12)
            for i, txt in enumerate(lines):
                cv2.putText(
                    rgb_panel, txt, (x, ly + i * 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1,
                )

        cv2.putText(
            rgb_panel, f"T.S. {elapsed_ms:.1f} ms | stable {n_stable}",
            (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
        )

        out = rgb_panel
        if self._show_depth:
            depth_vis = self._depth_colormap(depth_raw)
            for it in items:
                x, y, w, h = it["bbox"]
                cv2.rectangle(depth_vis, (x, y), (x + w, y + h), (255, 255, 255), 1)
                cv2.circle(
                    depth_vis, (int(x + w / 2), int(y + h / 2)), 3, (255, 255, 255), -1
                )
            if depth_vis.shape[0] == rgb_panel.shape[0]:
                out = np.hstack([rgb_panel, depth_vis])

        if self._out_width and out.shape[1] > self._out_width:
            scale = self._out_width / out.shape[1]
            out = cv2.resize(out, (self._out_width, int(out.shape[0] * scale)))
        return out

    @staticmethod
    def _depth_colormap(depth_raw):
        d = depth_raw.astype(np.float32)
        valid = d > 0
        if valid.any():
            lo, hi = np.percentile(d[valid], [5, 95])
            d = np.clip((d - lo) / max(hi - lo, 1.0), 0, 1)
        d8 = (d * 255).astype(np.uint8)
        vis = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
        vis[~valid] = (0, 0, 0)
        return vis

    def _publish_annotated(self, cv_image, header) -> None:
        try:
            msg = self._bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            msg.header = header
            self._annotated_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to publish annotated image: {exc}")
```

- [ ] **Step 5: Keep `main()` but drop the OpenCV window teardown**

Replace the `main()` finally block so it no longer calls `cv2.destroyAllWindows()`:

```python
def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = DetectionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 6: Re-run pure-function tests**

```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
cd src/gp4_perception && python -m pytest test/test_detection_visualizer.py -v
```
Expected: all pure-function tests still PASS (the node rewrite did not touch the helpers).

- [ ] **Step 7: Confirm `yaml` is declared as a runtime need (ament)**

`PyYAML` ships with the ROS 2 Python environment; no `package.xml` change is required. Verify the import resolves:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate && python -c "import yaml; print('yaml ok')"
```
Expected: `yaml ok`.

- [ ] **Step 8: Commit**

```bash
git add src/gp4_perception/gp4_perception/detection_visualizer.py
git commit -m "feat(perception): rewrite detection_visualizer as RGB-first depth detector"
```

---

## Task 6: Stop scene_processor publishing /perception/detections

**Files:**
- Modify: `src/gp4_perception/gp4_perception/scene_processor.py`
- Modify: `src/gp4_perception/test/test_scene_processor.py`

- [ ] **Step 1: Remove the `_det_pub` publisher (scene_processor.py ~line 176)**

Delete these three lines:

```python
        self._det_pub = self.create_publisher(
            Detection3DArray, "/perception/detections", 10
        )
```

- [ ] **Step 2: Remove only the detection-publish lines inside `_publish_detections` (~line 586-589)**

In `_publish_detections`, delete the array build + publish (keep the TTL eviction above and the collision-removal loop below):

```python
        arr = Detection3DArray()
        arr.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
        arr.detections = [d for _, d in self._last_detections]
        self._det_pub.publish(arr)
```

The collision-removal loop references `arr.header`. Replace that single reference: insert a local header where the loop needs it. Change the loop's `co.header = arr.header` to:

```python
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
```
placed right before the `for i in range(20):` loop, and change `co.header = arr.header` to `co.header = header`.

- [ ] **Step 3: Drop the now-unused `Detection3DArray` import if nothing else uses it**

Run:
```bash
cd /home/hieu2/gp4_ws/src/gp4_perception
grep -n "Detection3DArray" gp4_perception/scene_processor.py
```
If the only remaining hit is the import line (line ~32), remove `Detection3DArray,` from the `vision_msgs.msg` import group. If `Detection3D` (singular) is still used elsewhere, keep it.

- [ ] **Step 4: Update `test_uncalibrated_publish_clears_cached_scene`**

In `test/test_scene_processor.py`, this test sets `processor._det_pub = FailingPublisher()` and asserts the cache is cleared when uncalibrated. Since `_publish_detections` returns early on `not self._calibration_allows_scene_output()` before any publish, remove the `_det_pub` line (the attribute no longer exists). Edit the test body to:

```python
    def test_uncalibrated_publish_clears_cached_scene(self):
        class FailingPublisher:
            def publish(self, _msg):
                raise AssertionError("uncalibrated detections must not be published")

        processor = object.__new__(SceneProcessor)
        processor._last_detections = [(time.time(), object())]
        processor._published_collision_ids = set()
        processor._ttl = 2.0
        processor._collision_pub = FailingPublisher()
        processor._calibration_status = lambda: (False, "calibration_invalid", "", 0.0)

        processor._publish_detections()

        assert processor._last_detections == []
```

- [ ] **Step 5: Update `test_uncalibrated_publish_removes_advertised_collision_objects`**

Remove the `processor._det_pub = FailingDetectionPublisher()` line and the now-unused `FailingDetectionPublisher` class. The test keeps verifying collision removal:

```python
    def test_uncalibrated_publish_removes_advertised_collision_objects(self):
        published = []

        class CollisionPublisher:
            def publish(self, msg):
                published.append(msg)

        processor = object.__new__(SceneProcessor)
        processor._last_detections = [(time.time(), object())]
        processor._published_collision_ids = {"perception_obj_0", "perception_obj_2"}
        processor._ttl = 2.0
        processor._collision_pub = CollisionPublisher()
        processor._calibration_status = lambda: (False, "calibration_invalid", "", 0.0)

        processor._publish_detections()

        assert {msg.id for msg in published} == {"perception_obj_0", "perception_obj_2"}
        assert all(msg.operation == CollisionObject.REMOVE for msg in published)
        assert processor._published_collision_ids == set()
```

- [ ] **Step 6: Run the scene_processor tests**

Run:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
cd src/gp4_perception && python -m pytest test/test_scene_processor.py -v
```
Expected: all tests PASS (no reference to `_det_pub`).

- [ ] **Step 7: Run impact + change check before commit**

Run:
```bash
cd /home/hieu2/gp4_ws
git diff --stat
```
Expected: only `scene_processor.py` and `test_scene_processor.py` changed in this task.

- [ ] **Step 8: Commit**

```bash
git add src/gp4_perception/gp4_perception/scene_processor.py src/gp4_perception/test/test_scene_processor.py
git commit -m "refactor(perception): scene_processor stops publishing /perception/detections (RGB path owns it)"
```

---

## Task 7: Build, full test, and live acceptance

**Files:** none (verification only)

- [ ] **Step 1: colcon build the package**

Run:
```bash
cd /home/hieu2/gp4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gp4_perception --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
Expected: `Finished <<< gp4_perception` with no errors.

- [ ] **Step 2: Import smoke test of the node module (catches typos/missing symbols)**

Run:
```bash
source /home/hieu2/gp4_ws/install/setup.bash
python -c "from gp4_perception.detection_visualizer import DetectionVisualizer, _deproject_pixel, _median_depth_m, _bbox_size_m, _detect_in_ranges, _border_dominant_hue; print('import ok')"
```
Expected: `import ok`.

- [ ] **Step 3: Run the full gp4_perception pytest suite**

Run:
```bash
source /home/hieu2/gp4_ws/.venv/bin/activate
cd src/gp4_perception && python -m pytest test/ -q
```
Expected: all tests pass (new `test_detection_visualizer.py` + updated `test_scene_processor.py` included).

- [ ] **Step 4: GitNexus change scope check**

Use `gitnexus_detect_changes()` and confirm only `detection_visualizer.py`, `scene_processor.py`, the two test files, and `perception.yaml` appear. Report any unexpected affected execution flow.

- [ ] **Step 5: Live acceptance (requires camera bringup)**

With the RealSense + perception stack running, confirm against the spec acceptance checks:
```bash
ros2 topic echo /perception/detections --once
ros2 run rqt_image_view rqt_image_view /perception/annotated_image
```
Verify: `red_box` bbox visible; `dist`/`cam xyz` populated when depth valid; `base xyz` populated when TF up; `base TF_UNAVAILABLE` shown when TF missing; `XYZ_INVALID` shown when depth invalid; `/perception/detections` publishes stable detections; scene_processor still serves `GetObjectPositions` + `/perception/debug_clusters`.

- [ ] **Step 6: Final commit (if any verification fixups were needed)**

```bash
cd /home/hieu2/gp4_ws
git add -A
git commit -m "test(perception): verify RGB-first detection path build + suite"
```

---

## Self-review

**1. Spec coverage**
- RGB+aligned-depth sync, cached intrinsics → Task 5 (`ApproximateTimeSynchronizer`, `_on_camera_info`). ✓
- Per-class HSV → morphology → contours → bbox → Task 3 (`_detect_in_ranges`). ✓
- Median depth over mask, skip zeros/invalid → Task 2 (`_median_depth_m`). ✓
- Deproject bbox center → Task 2 (`_deproject_pixel`). ✓
- TF PointStamped → base_link with fallback → Task 5 (`_lookup_tf`, `_transform_point`, frame switch). ✓
- TemporalTracker reuse → Task 5 (`self._tracker.update`). ✓
- Publish Detection3DArray (frame = base_link or camera) → Task 5 (`_publish_detections`). ✓
- Annotated RGB | depth-colormap overlay, T.S. ms + count, downscale to config width → Task 5 (`_render_overlay`, `_depth_colormap`). ✓
- Config: color_classes, show_depth_panel, depth/topic names → Task 1. ✓
- scene_processor stops publishing detections, keeps service/collision/debug → Task 6. ✓
- Edge cases: TF missing (cam xyz + TF_UNAVAILABLE, publish in camera frame), depth invalid (draw + XYZ_INVALID, not published) → Task 5 overlay + publish guards. ✓
- bbox.size metric from pixel extent × Z / focal; nominal Z → Task 2 + `_make_detection`. ✓
- New test_detection_visualizer.py (deproject + median + invalid) → Tasks 2–4. ✓
- Update 2 scene_processor tests → Task 6. ✓

**2. Placeholder scan:** No TBD/TODO. Border reuse honestly implemented as a 2D analog rather than a false claim of reusing the pointcloud function.

**3. Type consistency:** `_detect_in_ranges` returns dicts with keys `bbox`/`area`/`mask`, consumed consistently in Task 5. `_make_detection` builds the exact `results[0].hypothesis.class_id` / `.score` / `pose.pose.position` shape `TemporalTracker._centroid`/`_class_id` read. `_median_depth_m`/`_deproject_pixel`/`_bbox_size_m` signatures match call sites.

**Open item for the user:** the rewrite removes the OpenCV live window + "BBox Filter" trackbar (and its `/scene_processor/set_parameters` client). This matches the approved design's keep-list but is a feature removal — confirm acceptable, or it can be re-added as a separate ROS param.
