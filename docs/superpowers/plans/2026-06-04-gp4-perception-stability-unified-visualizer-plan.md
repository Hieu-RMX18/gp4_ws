# GP4 Perception Stability Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stabilize the current GP4 perception stack without removing PointCloud2, without reducing camera profiles in the first pass, and without modifying robot motion, MoveIt, MotoROS2, TF calibration values, or safety logic.

**Architecture:** Keep the current two-branch design. `scene_processor.py` continues to consume PointCloud2 for 3D clustering, PCA, semantic support, and MoveIt collision objects. `unified_visualizer.py` continues to consume RGB + aligned depth + CameraInfo for 2D HSV detection, depth deprojection, dashboard rendering, preprocessing tabs, and debug images. Add bounded latest-frame processing, independent rate control, visual bbox hysteresis, and safer debug defaults.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, `rclpy`, `message_filters`, Intel RealSense D435i, OpenCV, NumPy, `vision_msgs`, `moveit_msgs`, pytest.

---

## Verified Current Source Map

The active source tree is:

```text
gp4_perception/
├── gp4_perception/
│   ├── scene_processor.py
│   ├── scene_geometry.py
│   ├── temporal_tracker.py
│   ├── unified_visualizer.py
│   ├── calibration.py
│   ├── query_perception_tool.py
│   └── safety_guards.py
├── launch/
│   ├── perception_full.launch.py
│   └── camera.launch.py
├── config/
│   └── perception.yaml
├── test/
│   ├── test_scene_processor.py
│   ├── test_temporal_tracker.py
│   ├── test_detection_visualizer.py
│   ├── test_qos_match.py
│   └── ...
└── setup.py
```

The active entry points are:

```text
scene_processor = gp4_perception.scene_processor:main
unified_visualizer = gp4_perception.unified_visualizer:main
tf_publisher = gp4_perception.calibration:main_tf_publisher
```

`unified_visualizer.py` explicitly merges the former `detection_visualizer.py` and `preprocessing_visualizer.py`. Do not recreate or modify removed source modules.

---

## Safety Boundaries

- Do not modify MoveIt configuration.
- Do not modify collision-checking semantics outside `gp4_perception`.
- Do not modify MotoROS2, `hw_adapter`, robot actions, or motion primitives.
- Do not change TF calibration values.
- Do not reduce `depth_profile=848x480x30` or `color_profile=1280x720x30` in the first pass.
- Keep PointCloud2 enabled.
- Run all validation with robot motion disabled.
- Any stale perception data must remain fail-closed for object-query consumers.

---

## Task 0: Create Isolated Worktree and Capture Baseline

**Files:**
- Create: `docs/superpowers/evidence/perception-stability-before.md`

- [ ] **Step 1: Create an isolated worktree**

```bash
cd ~/gp4_ws
git status --short
git worktree add ../gp4_ws-perception-stability -b perception-stability-hotfix
cd ../gp4_ws-perception-stability
```

Expected: clean working tree on branch `perception-stability-hotfix`.

- [ ] **Step 2: Source ROS 2 and workspace**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

- [ ] **Step 3: Record runtime graph and topic rates with robot motion disabled**

```bash
mkdir -p docs/superpowers/evidence
{
  date -Is
  echo '=== nodes ==='
  ros2 node list
  echo '=== perception topics ==='
  ros2 topic list | grep -E '^/camera|^/perception|collision_object'
  echo '=== usb ==='
  lsusb -t
  echo '=== color hz ==='
  timeout 20 ros2 topic hz /camera/color/image_raw
  echo '=== aligned depth hz ==='
  timeout 20 ros2 topic hz /camera/aligned_depth_to_color/image_raw
  echo '=== pointcloud hz ==='
  timeout 20 ros2 topic hz /camera/depth/color/points
  echo '=== detections hz ==='
  timeout 20 ros2 topic hz /perception/detections
  echo '=== dashboard hz ==='
  timeout 20 ros2 topic hz /perception/debug_dashboard_image
  echo '=== cpu ==='
  ps -eo pid,comm,%cpu,%mem --sort=-%cpu | head -n 20
  echo '=== ram ==='
  free -h
} | tee docs/superpowers/evidence/perception-stability-before.md
```

Expected: evidence file contains actual rates and resource usage. If a topic name differs, record `VERIFY_RUNTIME` and update the command to the runtime topic.

- [ ] **Step 4: Commit baseline evidence**

```bash
git add docs/superpowers/evidence/perception-stability-before.md
git commit -m "docs: capture perception stability baseline"
```

---

## Task 1: Add Reusable Latest-Frame Processing Primitives

**Files:**
- Create: `gp4_perception/gp4_perception/latest_frame.py`
- Create: `gp4_perception/test/test_latest_frame.py`

- [ ] **Step 1: Write failing tests**

Create `gp4_perception/test/test_latest_frame.py`:

```python
from gp4_perception.latest_frame import LatestValueSlot, MonotonicRateGate


def test_latest_value_slot_overwrites_stale_value():
    slot = LatestValueSlot()
    slot.put("old")
    slot.put("new")
    assert slot.take() == "new"
    assert slot.take() is None


def test_rate_gate_allows_first_call_and_rejects_early_call():
    gate = MonotonicRateGate(rate_hz=10.0)
    assert gate.allow(now=1.0)
    assert not gate.allow(now=1.05)
    assert gate.allow(now=1.10)


def test_zero_rate_disables_gate_blocking():
    gate = MonotonicRateGate(rate_hz=0.0)
    assert gate.allow(now=1.0)
    assert gate.allow(now=1.0)
```

- [ ] **Step 2: Run tests and verify failure**

```bash
cd ~/gp4_ws-perception-stability/src/gp4_perception
pytest -q test/test_latest_frame.py
```

Expected: FAIL because `gp4_perception.latest_frame` does not exist.

- [ ] **Step 3: Add minimal implementation**

Create `gp4_perception/gp4_perception/latest_frame.py`:

```python
"""Small helpers for bounded latest-frame processing."""

from __future__ import annotations

from threading import Lock
from typing import Generic, TypeVar

T = TypeVar("T")


class LatestValueSlot(Generic[T]):
    """Single-slot mailbox: newer input replaces stale pending input."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._value: T | None = None

    def put(self, value: T) -> None:
        with self._lock:
            self._value = value

    def take(self) -> T | None:
        with self._lock:
            value = self._value
            self._value = None
            return value


class MonotonicRateGate:
    """Permit processing no faster than the configured rate."""

    def __init__(self, rate_hz: float) -> None:
        self._period_s = 0.0 if rate_hz <= 0.0 else 1.0 / float(rate_hz)
        self._last_allowed_s: float | None = None

    def allow(self, now: float) -> bool:
        if self._period_s <= 0.0:
            return True
        if self._last_allowed_s is None or now - self._last_allowed_s >= self._period_s:
            self._last_allowed_s = now
            return True
        return False
```

- [ ] **Step 4: Run tests**

```bash
pytest -q test/test_latest_frame.py
```

Expected: `3 passed`.

- [ ] **Step 5: Commit**

```bash
git add gp4_perception/latest_frame.py test/test_latest_frame.py
git commit -m "feat(perception): add bounded latest-frame helpers"
```

---

## Task 2: Add Real Hysteresis to the Existing 3D Temporal Tracker

**Files:**
- Modify: `gp4_perception/gp4_perception/temporal_tracker.py`
- Modify: `gp4_perception/test/test_temporal_tracker.py`
- Modify: `gp4_perception/config/perception.yaml`

- [ ] **Step 1: Add failing tests for 1-frame and 2-frame misses**

Append tests that construct existing fake `Detection3D` objects using the helpers already present in `test/test_temporal_tracker.py`:

```python
def test_confirmed_track_survives_two_missing_frames():
    tracker = TemporalTracker(window_frames=5, min_hits=3, jitter_max_mm=15.0,
                              miss_tolerance_frames=2)
    det = make_detection("red_box", 0.30, 0.00, 0.10)
    assert tracker.update([det]) == []
    assert tracker.update([det]) == []
    assert len(tracker.update([det])) == 1
    assert len(tracker.update([])) == 1
    assert len(tracker.update([])) == 1
    assert tracker.update([]) == []


def test_unconfirmed_track_does_not_survive_missing_frame():
    tracker = TemporalTracker(window_frames=5, min_hits=3, jitter_max_mm=15.0,
                              miss_tolerance_frames=2)
    det = make_detection("red_box", 0.30, 0.00, 0.10)
    assert tracker.update([det]) == []
    assert tracker.update([]) == []
```

- [ ] **Step 2: Run failing tests**

```bash
pytest -q test/test_temporal_tracker.py
```

Expected: FAIL because `miss_tolerance_frames` is not accepted and misses are not emitted.

- [ ] **Step 3: Implement minimal hysteresis**

Update `_Track`:

```python
@dataclass
class _Track:
    class_id: str
    centroid: np.ndarray
    hits: deque = field(default_factory=deque)
    last_detection: Any | None = None
    missed_frames: int = 0
    confirmed: bool = False

    def score(self, window: int) -> float:
        return sum(self.hits) / window
```

Update constructor:

```python
def __init__(
    self,
    window_frames: int = 5,
    min_hits: int = 3,
    jitter_max_mm: float = 15.0,
    miss_tolerance_frames: int = 2,
) -> None:
    self._window = int(window_frames)
    self._min_hits = int(min_hits)
    self._jitter_max_m = float(jitter_max_mm) / 1000.0
    self._miss_tolerance = int(miss_tolerance_frames)
    self._tracks: list[_Track] = []
```

In `update()`, when matched:

```python
track.centroid = cen
track.last_detection = det
track.missed_frames = 0
track.hits.append(1)
```

When unmatched:

```python
track.hits.append(0)
track.missed_frames += 1
```

After trimming windows:

```python
for track in self._tracks:
    if sum(track.hits) >= self._min_hits:
        track.confirmed = True

self._tracks = [
    track for track in self._tracks
    if sum(track.hits) > 0 and track.missed_frames <= self._miss_tolerance
]
```

Build results from all confirmed tracks still within tolerance:

```python
results = []
for track in self._tracks:
    if track.confirmed and track.last_detection is not None:
        results.append((track.last_detection, track.score(self._window)))
return results
```

- [ ] **Step 4: Add YAML config**

Under `perception:` add:

```yaml
temporal_miss_tolerance_frames: 2
```

- [ ] **Step 5: Wire config into `scene_processor.py`**

Where `TemporalTracker(...)` is constructed, pass:

```python
miss_tolerance_frames=int(self._cfg.get("temporal_miss_tolerance_frames", 2)),
```

- [ ] **Step 6: Run tests**

```bash
pytest -q test/test_temporal_tracker.py
```

Expected: all tracker tests PASS.

- [ ] **Step 7: Commit**

```bash
git add gp4_perception/temporal_tracker.py gp4_perception/scene_processor.py config/perception.yaml test/test_temporal_tracker.py
git commit -m "feat(perception): add 3d tracker hysteresis"
```

---

## Task 3: Remove Unused CameraInfo Synchronization from Scene Processor

**Files:**
- Modify: `gp4_perception/gp4_perception/scene_processor.py`
- Modify: `gp4_perception/test/test_scene_processor.py`

- [ ] **Step 1: Add a regression test**

Add a source-level regression test:

```python
from pathlib import Path


def test_scene_processor_does_not_wait_for_camera_info_sync():
    source = Path(__file__).parents[1] / "gp4_perception" / "scene_processor.py"
    text = source.read_text()
    assert "ApproximateTimeSynchronizer" not in text
    assert "CameraInfo" not in text
```

- [ ] **Step 2: Run test and verify failure**

```bash
pytest -q test/test_scene_processor.py::test_scene_processor_does_not_wait_for_camera_info_sync
```

Expected: FAIL.

- [ ] **Step 3: Replace synchronized subscriptions with direct PointCloud2 subscription**

Remove imports for `CameraInfo`, `Subscriber`, and `ApproximateTimeSynchronizer` if unused elsewhere.

Replace the current subscriber block with:

```python
self._cloud_sub = self.create_subscription(
    PointCloud2,
    "/camera/depth/color/points",
    self._on_cloud,
    qos_profile,
)
```

Rename callback:

```python
def _on_cloud(self, cloud: PointCloud2) -> None:
```

Keep the processing body unchanged for this task.

- [ ] **Step 4: Run tests**

```bash
pytest -q test/test_scene_processor.py
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gp4_perception/scene_processor.py test/test_scene_processor.py
git commit -m "fix(perception): stop gating pointcloud on unused camera info"
```

---

## Task 4: Convert Scene Processor to Latest-Cloud-Wins Worker

**Files:**
- Modify: `gp4_perception/gp4_perception/scene_processor.py`
- Modify: `gp4_perception/config/perception.yaml`
- Modify: `gp4_perception/test/test_scene_processor.py`

- [ ] **Step 1: Add configuration**

Under `perception:` add:

```yaml
pointcloud_processor:
  processing_rate_hz: 8.0
```

- [ ] **Step 2: Add source-level regression test**

```python
def test_scene_processor_uses_latest_cloud_worker():
    source = Path(__file__).parents[1] / "gp4_perception" / "scene_processor.py"
    text = source.read_text()
    assert "LatestValueSlot" in text
    assert "_process_latest_cloud" in text
    assert "_latest_cloud.put(cloud)" in text
```

- [ ] **Step 3: Wire latest-cloud slot and worker timer**

Import:

```python
from gp4_perception.latest_frame import LatestValueSlot
```

In `__init__`:

```python
pc_proc_cfg = self._cfg.get("pointcloud_processor", {})
self._cloud_processing_rate_hz = float(pc_proc_cfg.get("processing_rate_hz", 8.0))
self._latest_cloud = LatestValueSlot[PointCloud2]()
self._last_cloud_callback_time = time.monotonic()
self._last_cloud_processed_time = time.monotonic()
self._cloud_worker_timer = self.create_timer(
    1.0 / max(self._cloud_processing_rate_hz, 0.1),
    self._process_latest_cloud,
)
```

Replace callback body with:

```python
def _on_cloud(self, cloud: PointCloud2) -> None:
    self._last_cloud_callback_time = time.monotonic()
    self._latest_cloud.put(cloud)
```

Move the former heavy body into:

```python
def _process_latest_cloud(self) -> None:
    cloud = self._latest_cloud.take()
    if cloud is None:
        return
    self._last_cloud_processed_time = time.monotonic()
    self._process_cloud(cloud)


def _process_cloud(self, cloud: PointCloud2) -> None:
    # Existing heavy processing body from the former callback starts here.
```

- [ ] **Step 4: Improve watchdog diagnostics**

Replace `_check_camera_health()` with:

```python
def _check_camera_health(self) -> None:
    now = time.monotonic()
    input_age = now - self._last_cloud_callback_time
    processed_age = now - self._last_cloud_processed_time
    if input_age > 5.0:
        _LOGGER.warning("CLOUD_INPUT_STALE: no PointCloud2 callback for %.1f s", input_age)
    elif processed_age > 5.0:
        _LOGGER.warning("CLOUD_PROCESSING_OVERRUN: latest cloud not processed for %.1f s", processed_age)
```

- [ ] **Step 5: Run tests**

```bash
pytest -q test/test_scene_processor.py test/test_latest_frame.py
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add gp4_perception/scene_processor.py config/perception.yaml test/test_scene_processor.py
git commit -m "perf(perception): process latest pointcloud at bounded rate"
```

---

## Task 5: Add Visual Detection Hysteresis for Dashboard Bboxes

**Files:**
- Create: `gp4_perception/gp4_perception/visual_tracking.py`
- Create: `gp4_perception/test/test_visual_tracking.py`
- Modify: `gp4_perception/config/perception.yaml`

- [ ] **Step 1: Write failing tests**

Create `gp4_perception/test/test_visual_tracking.py`:

```python
from gp4_perception.visual_tracking import VisualDetectionTracker


def obj(class_id="red_box", bbox=(100, 100, 80, 60), confidence=0.8, z_m=0.4):
    x, y, w, h = bbox
    return {
        "class_id": class_id,
        "bbox": bbox,
        "center_uv": (x + w // 2, y + h // 2),
        "confidence": confidence,
        "distance_m": z_m,
    }


def test_visual_track_is_confirmed_after_two_hits():
    tracker = VisualDetectionTracker(min_hits=2, miss_tolerance_frames=2,
                                     max_centroid_distance_px=45.0, min_iou=0.15,
                                     ema_alpha=0.35)
    assert tracker.update([obj()]) == []
    assert len(tracker.update([obj(bbox=(102, 101, 80, 60))])) == 1


def test_confirmed_visual_track_survives_two_misses_for_display_only():
    tracker = VisualDetectionTracker(min_hits=2, miss_tolerance_frames=2,
                                     max_centroid_distance_px=45.0, min_iou=0.15,
                                     ema_alpha=0.35)
    tracker.update([obj()])
    tracker.update([obj()])
    first = tracker.update([])
    second = tracker.update([])
    expired = tracker.update([])
    assert first[0]["held_for_display"] is True
    assert second[0]["held_for_display"] is True
    assert expired == []
```

- [ ] **Step 2: Run tests and verify failure**

```bash
pytest -q test/test_visual_tracking.py
```

Expected: FAIL because module does not exist.

- [ ] **Step 3: Implement tracker**

Create `gp4_perception/gp4_perception/visual_tracking.py` with a small deterministic tracker. Requirements:

```text
- Match only same class_id.
- Match by bbox IoU >= min_iou OR centroid distance <= max_centroid_distance_px.
- Confirm after min_hits.
- Smooth bbox and center with EMA.
- Preserve last object for at most miss_tolerance_frames.
- Mark preserved objects with held_for_display=True.
- Fresh matched objects use held_for_display=False.
```

- [ ] **Step 4: Add YAML config**

Under `perception:` add:

```yaml
visual_tracking:
  enabled: true
  min_hits: 2
  miss_tolerance_frames: 2
  max_centroid_distance_px: 45.0
  min_iou: 0.15
  ema_alpha: 0.35
```

- [ ] **Step 5: Run tests**

```bash
pytest -q test/test_visual_tracking.py
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add gp4_perception/visual_tracking.py test/test_visual_tracking.py config/perception.yaml
git commit -m "feat(perception): add visual bbox hysteresis"
```

---

## Task 6: Rate-Limit Unified Visualizer Before Heavy RGB-D Processing

**Files:**
- Modify: `gp4_perception/gp4_perception/unified_visualizer.py`
- Modify: `gp4_perception/config/perception.yaml`
- Modify: `gp4_perception/test/test_detection_visualizer.py`

- [ ] **Step 1: Add runtime config**

Under `rgb_detector:` add:

```yaml
processing_rate_hz: 10.0
sync_queue: 2
```

- [ ] **Step 2: Add visualization defaults**

Under `visualization:` add or update:

```yaml
max_width_px: 960
zoom_roi:
  enabled: false
qos:
  debug_images:
    reliability: "best_effort"
    depth: 1
debug_masks:
  enabled: false
  publish_blue_border: false
  publish_white_workpiece: false
```

- [ ] **Step 3: Add source-level regression test**

```python
def test_unified_visualizer_has_processing_rate_gate():
    source = Path(__file__).parents[1] / "gp4_perception" / "unified_visualizer.py"
    text = source.read_text()
    assert "processing_rate_hz" in text
    assert "MonotonicRateGate" in text
    assert "if not self._processing_gate.allow" in text
```

- [ ] **Step 4: Wire gate at the top of `_on_synced_rgbd()`**

Import:

```python
from gp4_perception.latest_frame import MonotonicRateGate
from gp4_perception.visual_tracking import VisualDetectionTracker
```

In `__init__` after reading `rgb_cfg`:

```python
self._processing_rate_hz = float(rgb_cfg.get("processing_rate_hz", 10.0))
self._processing_gate = MonotonicRateGate(self._processing_rate_hz)
```

At the first line of `_on_synced_rgbd()`:

```python
if not self._processing_gate.allow(time.monotonic()):
    return
```

This must execute before CvBridge conversion and before HSV detection.

- [ ] **Step 5: Use BEST_EFFORT QoS for visualization image publishers**

Create one image QoS profile from YAML and use it for:

```text
/perception/annotated_image
/perception/debug_dashboard_image
/perception/preprocessing_debug
/perception/zoom_roi_image
/perception/debug_mask/blue_border
/perception/debug_mask/white_workpiece
```

Use:

```python
QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
```

Do not change `/perception/detections` QoS unless runtime evidence requires it.

- [ ] **Step 6: Run tests**

```bash
pytest -q test/test_detection_visualizer.py test/test_qos_match.py
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add gp4_perception/unified_visualizer.py config/perception.yaml test/test_detection_visualizer.py test/test_qos_match.py
git commit -m "perf(perception): bound unified visualizer processing load"
```

---

## Task 7: Integrate Visual Tracker into Unified Visualizer Safely

**Files:**
- Modify: `gp4_perception/gp4_perception/unified_visualizer.py`
- Modify: `gp4_perception/test/test_detection_visualizer.py`

- [ ] **Step 1: Instantiate tracker from YAML**

In `__init__`:

```python
visual_tracking_cfg = pcfg.get("visual_tracking", {})
self._visual_tracking_enabled = bool(visual_tracking_cfg.get("enabled", True))
self._visual_tracker = VisualDetectionTracker(
    min_hits=int(visual_tracking_cfg.get("min_hits", 2)),
    miss_tolerance_frames=int(visual_tracking_cfg.get("miss_tolerance_frames", 2)),
    max_centroid_distance_px=float(visual_tracking_cfg.get("max_centroid_distance_px", 45.0)),
    min_iou=float(visual_tracking_cfg.get("min_iou", 0.15)),
    ema_alpha=float(visual_tracking_cfg.get("ema_alpha", 0.35)),
)
```

- [ ] **Step 2: Stabilize visual objects after raw detection**

Immediately after:

```python
objects = detect_color_objects(rgb, self._color_classes, self._postprocess_cfg)
```

add:

```python
raw_objects = objects
if self._visual_tracking_enabled:
    objects = self._visual_tracker.update(raw_objects)
```

- [ ] **Step 3: Keep held bboxes display-only**

Before publishing a `Detection3D`, add:

```python
held_for_display = bool(obj.get("held_for_display", False))
```

Always draw the held bbox. Do not append held objects to `/perception/detections`:

```python
if held_for_display:
    lines = [f"{obj_id} {class_id}", "HELD_DISPLAY_ONLY"]
    _draw_detection_callout(...)
    continue
```

Rationale: hysteresis improves GUI stability but must not make stale target coordinates executable.

- [ ] **Step 4: Add regression test**

Add a test asserting that the source contains `HELD_DISPLAY_ONLY` and that held objects skip Detection3D publishing.

- [ ] **Step 5: Run tests**

```bash
pytest -q test/test_visual_tracking.py test/test_detection_visualizer.py
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add gp4_perception/unified_visualizer.py test/test_detection_visualizer.py
git commit -m "feat(perception): stabilize dashboard bboxes without publishing stale targets"
```

---

## Task 8: Fix White Workpiece Near-Range Rejection and Add Reject Diagnostics

**Files:**
- Modify: `gp4_perception/config/perception.yaml`
- Modify: `gp4_perception/gp4_perception/unified_visualizer.py`
- Modify: `gp4_perception/test/test_detection_visualizer.py`

- [ ] **Step 1: Increase only the area ceiling**

Change:

```yaml
max_area_px: 30000
```

to:

```yaml
max_area_px: 120000
```

Do not relax all ratios at once.

- [ ] **Step 2: Add reject-reason instrumentation**

In the white-workpiece detector path, log structured diagnostics when a candidate is rejected:

```text
[white_workpiece] reject=max_area bbox_area_px=... aspect_ratio=...
[white_workpiece] reject=min_area bbox_area_px=... aspect_ratio=...
[white_workpiece] reject=inner_white_ratio inner_white_ratio=...
[white_workpiece] reject=blue_border_ratio blue_border_ratio=...
[white_workpiece] reject=depth_invalid
```

Use debug-level logs by default so runtime output remains bounded.

- [ ] **Step 3: Add tests**

Add tests that verify:

```text
- a synthetic blue-border white-fill rectangle larger than 30000 px and smaller than 120000 px is accepted;
- a candidate above 120000 px is rejected;
- reject reason is returned or logged for max_area.
```

- [ ] **Step 4: Run tests**

```bash
pytest -q test/test_detection_visualizer.py
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add config/perception.yaml gp4_perception/unified_visualizer.py test/test_detection_visualizer.py
git commit -m "fix(perception): support near-range white workpiece detection"
```

---

## Task 9: Keep Camera Logging Useful During Validation

**Files:**
- Modify: `gp4_perception/launch/camera.launch.py`

- [ ] **Step 1: Change validation log level from `info` to `warn`**

Change:

```python
arguments=["--ros-args", "--log-level", "info"],
```

to:

```python
arguments=["--ros-args", "--log-level", "warn"],
```

Do not change to `error` during validation. Warnings are needed to investigate PointCloud2 texture issues.

- [ ] **Step 2: Keep PointCloud2 texture parameters unchanged**

Do not change:

```text
pointcloud.stream_filter = 2
pointcloud.stream_index_filter = 0
```

unless runtime evidence proves the installed RealSense wrapper requires different values.

- [ ] **Step 3: Commit**

```bash
git add launch/camera.launch.py
git commit -m "chore(perception): reduce camera log noise while retaining warnings"
```

---

## Task 10: Build, Unit Test, and Runtime Validate

**Files:**
- Create: `docs/superpowers/evidence/perception-stability-after.md`

- [ ] **Step 1: Build package**

```bash
cd ~/gp4_ws-perception-stability
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gp4_perception
source install/setup.bash
```

Expected: package builds successfully.

- [ ] **Step 2: Run unit tests**

```bash
cd src/gp4_perception
pytest -q
```

Expected: all tests PASS.

- [ ] **Step 3: Launch camera and perception with robot motion disabled**

```bash
ros2 launch gp4_perception perception_full.launch.py
```

- [ ] **Step 4: Open one raw RQT view only**

```bash
ros2 run rqt_image_view rqt_image_view
```

Select only:

```text
/perception/debug_dashboard_image
```

Use `raw` transport. Do not use Theora or compressed depth transports.

- [ ] **Step 5: Capture after evidence**

```bash
cd ~/gp4_ws-perception-stability
{
  date -Is
  echo '=== color hz ==='
  timeout 20 ros2 topic hz /camera/color/image_raw
  echo '=== aligned depth hz ==='
  timeout 20 ros2 topic hz /camera/aligned_depth_to_color/image_raw
  echo '=== pointcloud hz ==='
  timeout 20 ros2 topic hz /camera/depth/color/points
  echo '=== detections hz ==='
  timeout 20 ros2 topic hz /perception/detections
  echo '=== dashboard hz ==='
  timeout 20 ros2 topic hz /perception/debug_dashboard_image
  echo '=== collision object topic ==='
  timeout 10 ros2 topic hz /collision_object
  echo '=== cpu ==='
  ps -eo pid,comm,%cpu,%mem --sort=-%cpu | head -n 20
  echo '=== ram ==='
  free -h
} | tee docs/superpowers/evidence/perception-stability-after.md
```

- [ ] **Step 6: Validate behavior manually**

Record results in the evidence file:

```text
- stationary bbox does not blink;
- moving object updates without multi-second lag;
- held bbox survives at most 2 missing visual frames and is labeled HELD_DISPLAY_ONLY;
- white_workpiece is detected both near and far;
- PointCloud2 path remains enabled;
- collision objects still publish;
- no robot motion command was triggered;
- exact remaining RealSense warnings are copied verbatim and marked VERIFY_RUNTIME.
```

- [ ] **Step 7: Commit evidence**

```bash
git add docs/superpowers/evidence/perception-stability-after.md
git commit -m "docs: capture perception stability validation evidence"
```

---

## Task 11: Final Diff Audit

- [ ] **Step 1: Verify changed scope**

```bash
cd ~/gp4_ws-perception-stability
git diff --stat main...HEAD
git diff --name-only main...HEAD
```

Expected: changes remain inside `src/gp4_perception` and documentation evidence.

- [ ] **Step 2: Confirm forbidden paths are untouched**

```bash
git diff --name-only main...HEAD | grep -E 'moveit|motion_core|hw_adapter|MotoROS2|safety|extrinsics.yaml' && exit 1 || true
```

Expected: no forbidden path is printed.

- [ ] **Step 3: Produce final report**

Report:

```text
1. Modified files.
2. Commit hashes.
3. Unit-test summary.
4. Before/after FPS.
5. Before/after CPU and RAM.
6. White-workpiece near/far results.
7. Whether /collision_object still publishes.
8. Remaining warnings marked VERIFY_RUNTIME.
9. Explicit confirmation that robot motion paths were not changed or triggered.
```

---

## Acceptance Criteria

The patch is accepted only when all of the following are true:

```text
- unified_visualizer remains the only RGB-D GUI/debug visualizer source;
- PointCloud2 remains enabled;
- camera profiles remain 848x480x30 depth and 1280x720x30 color;
- scene_processor no longer waits for unused CameraInfo synchronization;
- PointCloud2 heavy processing is latest-cloud-wins and bounded to 8 Hz initially;
- unified_visualizer heavy processing is bounded to 10 Hz before CvBridge conversion;
- internal debug image publishers use BEST_EFFORT KEEP_LAST depth=1;
- zoom ROI and debug masks are disabled by default;
- visual bbox hysteresis is separate from 3D collision-object hysteresis;
- held visual bboxes are display-only and never published as fresh robot targets;
- white_workpiece supports near placement without globally loosening every threshold;
- RQT is tested with raw transport only;
- no robot motion path is modified or triggered.
```

---

## Known Runtime Follow-Up

Mark this as `VERIFY_RUNTIME` until measured on the deployed machine:

```text
No stream match for pointcloud chosen texture Process - Color
```

Do not suppress the warning permanently and do not change RealSense texture parameters blindly. First collect:

```bash
ros2 param get /camera pointcloud.stream_filter
ros2 param get /camera pointcloud.stream_index_filter
ros2 topic hz /camera/depth/color/points
ros2 topic echo --once /camera/depth/color/points | head -n 40
```

