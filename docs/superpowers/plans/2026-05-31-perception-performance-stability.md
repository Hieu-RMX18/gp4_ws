# Perception Pipeline Performance & Stability Optimization

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix lag/low FPS, bbox flickering, rqt black screen errors, and white_workpiece close-range detection failure.

**Architecture:** 4 ROS 2 nodes run simultaneously. Root causes: (1) no frame rate control — all nodes process every 30fps frame, (2) large uncompressed images flooding ROS topics, (3) camera driver WARN spam consuming I/O, (4) temporal tracker too strict for degraded FPS, (5) white_workpiece params not tuned for close range.

**Tech Stack:** ROS 2 Humble, Python, OpenCV, NumPy, realsense-ros

**Branch:** `ws-deep-rebuild-3526`

---

### Task 1: Camera Launch — Suppress Log Spam

**Files:**
- Modify: `src/gp4_perception/launch/camera.launch.py`

The terminal shows hundreds of WARN-level messages per second:
- `No stream match for pointcloud chosen texture Process - Color`
- `[16UC1] is not a color format, but [mono8] is`
- `Packet was not a Theora header` (rqt subscriber side)

This log I/O overhead degrades all nodes sharing the process.

- [ ] **Step 1: Raise camera node log level to suppress WARN spam**

In `camera.launch.py`, change the arguments line:

```python
# Line 144, change:
arguments=["--ros-args", "--log-level", "info"],
# To:
arguments=["--ros-args", "--log-level", "error"],
```

This suppresses the repetitive WARN messages while still showing actual errors.

- [ ] **Step 2: Build and verify**

```bash
cd /home/hieu2/gp4_ws && colcon build --symlink-install --packages-select gp4_perception
```

- [ ] **Step 3: Commit**

```bash
git add src/gp4_perception/launch/camera.launch.py
git commit -m "perf: suppress camera WARN log spam to reduce I/O overhead"
```

---

### Task 2: Rate-Control Detection Visualizer & Reduce Bandwidth

**Files:**
- Modify: `src/gp4_perception/gp4_perception/detection_visualizer.py`
- Modify: `src/gp4_perception/config/perception.yaml`

Currently `_on_synced_rgbd` processes EVERY synced frame (up to 30fps) and publishes full-resolution uncompressed images. This is the #1 bandwidth bottleneck.

- [ ] **Step 1: Add rate limiting config to perception.yaml**

Add under `perception.visualization`:

```yaml
  visualization:
    max_process_fps: 10.0       # max detection processing rate (Hz)
    max_annotated_width_px: 960 # downscale annotated image before publish
```

- [ ] **Step 2: Add rate control + busy guard to detection_visualizer**

In `DetectionVisualizer.__init__`, after `self._last_zoom_time = time.time()` (line 789), add:

```python
        self._max_process_fps = float(viz_cfg.get("max_process_fps", 10.0))
        self._min_process_interval = 1.0 / self._max_process_fps if self._max_process_fps > 0 else 0.0
        self._last_process_time = 0.0
        self._processing = False
        self._max_annotated_width = viz_cfg.get("max_annotated_width_px", 960)
```

- [ ] **Step 3: Add frame skip guard at top of _on_synced_rgbd**

At the very beginning of `_on_synced_rgbd` (line 808), before the `if self._fx is None` check, add:

```python
        import time as _time
        now = _time.time()
        if self._processing:
            return  # still processing previous frame, drop this one
        if (now - self._last_process_time) < self._min_process_interval:
            return  # rate limit
        self._processing = True
        self._last_process_time = now
```

Then wrap the entire rest of the method body in `try/finally`:

```python
        try:
            # ... existing code from line 809 onwards ...
        finally:
            self._processing = False
```

- [ ] **Step 4: Reduce sync_queue from 10 to 2**

In `__init__`, change the ApproximateTimeSynchronizer (around line 746):

```python
        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub],
            queue_size=2,   # was self._sync_queue (10) — reduce to prevent queue buildup
            slop=self._sync_slop,
        )
```

- [ ] **Step 5: Downscale annotated image before publish**

Before publishing the annotated image (around line 1014), add downscale logic:

```python
        # Downscale for bandwidth before publishing.
        ch, cw = combined.shape[:2]
        max_w = self._max_annotated_width
        if max_w and cw > max_w:
            s = max_w / cw
            combined = cv2.resize(combined, (max_w, int(ch * s)),
                                  interpolation=cv2.INTER_AREA)
```

- [ ] **Step 6: Change annotated publisher QoS to BEST_EFFORT depth=1**

In `__init__`, change `_annotated_pub` (around line 757):

```python
        ann_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._annotated_pub = self.create_publisher(
            Image, "/perception/annotated_image", ann_qos
        )
```

- [ ] **Step 7: Build and verify**

```bash
cd /home/hieu2/gp4_ws && colcon build --symlink-install --packages-select gp4_perception
```

- [ ] **Step 8: Commit**

```bash
git add src/gp4_perception/gp4_perception/detection_visualizer.py src/gp4_perception/config/perception.yaml
git commit -m "perf: rate-limit detection_visualizer to 10Hz, downscale output, reduce queue"
```

---

### Task 3: Rate-Control Preprocessing Visualizer

**Files:**
- Modify: `src/gp4_perception/gp4_perception/preprocessing_visualizer.py`

Currently processes ALL 9 OpenCV stages on every frame at full camera FPS. Must rate-limit.

- [ ] **Step 1: Add rate limiting to preprocessing_visualizer**

In `PreprocessingVisualizer.__init__`, after `self._v_min = 60` (line 207), add:

```python
        import time as _time
        self._max_fps = 5.0
        self._min_interval = 1.0 / self._max_fps
        self._last_process_time = 0.0
```

- [ ] **Step 2: Add frame skip at top of _on_image**

At the beginning of `_on_image` (line 381), before the CvBridge call, add:

```python
        import time as _time
        now = _time.time()
        if (now - self._last_process_time) < self._min_interval:
            return  # rate limit — skip this frame
        self._last_process_time = now
```

- [ ] **Step 3: Build and verify**

```bash
cd /home/hieu2/gp4_ws && colcon build --symlink-install --packages-select gp4_perception
```

- [ ] **Step 4: Commit**

```bash
git add src/gp4_perception/gp4_perception/preprocessing_visualizer.py
git commit -m "perf: rate-limit preprocessing_visualizer to 5Hz"
```

---

### Task 4: Rate-Control Scene Processor

**Files:**
- Modify: `src/gp4_perception/gp4_perception/scene_processor.py`

PointCloud2 processing is CPU-heavy. Add frame skip and reduce sync queue.

- [ ] **Step 1: Add rate control to scene_processor**

In `SceneProcessor.__init__`, after `self._logged_cloud_fields = False` (line 192), add:

```python
        self._max_process_fps = 10.0
        self._min_process_interval = 1.0 / self._max_process_fps
        self._last_process_time_mono = 0.0
        self._processing = False
```

- [ ] **Step 2: Add frame skip guard at top of _on_synced**

At the beginning of `_on_synced` (line 228), add:

```python
        now_mono = time.time()
        if self._processing:
            return
        if (now_mono - self._last_process_time_mono) < self._min_process_interval:
            return
        self._processing = True
        self._last_process_time_mono = now_mono
```

Wrap the rest in `try/finally`:

```python
        try:
            self._last_cloud_time = time.time()
            # ... rest of existing code ...
        finally:
            self._processing = False
```

- [ ] **Step 3: Reduce sync_queue from 10 to 2**

Change line 170:

```python
        self._sync = ApproximateTimeSynchronizer(
            [self._cloud_sub, self._info_sub], queue_size=2, slop=0.05
        )
```

- [ ] **Step 4: Build and verify**

```bash
cd /home/hieu2/gp4_ws && colcon build --symlink-install --packages-select gp4_perception
```

- [ ] **Step 5: Commit**

```bash
git add src/gp4_perception/gp4_perception/scene_processor.py
git commit -m "perf: rate-limit scene_processor to 10Hz, reduce sync queue"
```

---

### Task 5: Improve Temporal Tracker Stability (Hysteresis)

**Files:**
- Modify: `src/gp4_perception/gp4_perception/temporal_tracker.py`
- Modify: `src/gp4_perception/test/test_temporal_tracker.py`

Current tracker drops a detection the moment it misses min_hits in the window. With low FPS, this causes flickering. Add hysteresis: once confirmed, a track persists through brief dropouts.

- [ ] **Step 1: Write failing tests for hysteresis behavior**

Add to `test/test_temporal_tracker.py`:

```python
    def test_confirmed_track_survives_brief_dropout(self):
        """Once confirmed, a 1-frame dropout should NOT kill the track."""
        tr = TemporalTracker(window_frames=5, min_hits=3, jitter_max_mm=15.0,
                             miss_tolerance=2)
        det = lambda: _det("red_box", 0.3, 0.0, 0.1)
        # Build up to confirmed (3 hits)
        tr.update([det()])
        tr.update([det()])
        out = tr.update([det()])
        assert len(out) == 1  # confirmed

        # 1-frame dropout
        out_miss = tr.update([])
        # Track should persist via hysteresis (returns last known detection info)
        # But with no detection to return, it returns empty
        # The key test: next frame with detection should still be stable
        out_back = tr.update([det()])
        assert len(out_back) == 1  # should still be confirmed, not reset

    def test_confirmed_track_dies_after_miss_tolerance(self):
        """Confirmed track should die after miss_tolerance consecutive misses."""
        tr = TemporalTracker(window_frames=5, min_hits=3, jitter_max_mm=15.0,
                             miss_tolerance=2)
        det = lambda: _det("red_box", 0.3, 0.0, 0.1)
        tr.update([det()])
        tr.update([det()])
        tr.update([det()])  # confirmed

        # 3 consecutive misses (> miss_tolerance=2)
        tr.update([])
        tr.update([])
        tr.update([])

        # Now detection should need to re-confirm
        out = tr.update([det()])
        assert len(out) == 0  # not yet re-confirmed
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/hieu2/gp4_ws && python -m pytest src/gp4_perception/test/test_temporal_tracker.py -v
```

Expected: FAIL — `TypeError: __init__() got unexpected keyword argument 'miss_tolerance'`

- [ ] **Step 3: Implement hysteresis in temporal_tracker.py**

Replace the full `temporal_tracker.py` content:

```python
"""Temporal voting tracker for 3D detections.

Suppresses flicker by requiring a detection to appear in a stable position
across several consecutive frames before it is published. Once confirmed,
tracks persist through brief dropouts (hysteresis) controlled by
``miss_tolerance``.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np


def _centroid(detection: Any) -> np.ndarray:
    pos = detection.results[0].pose.pose.position
    return np.array([pos.x, pos.y, pos.z], dtype=np.float64)


def _class_id(detection: Any) -> str:
    return str(detection.results[0].hypothesis.class_id)


@dataclass
class _Track:
    class_id: str
    centroid: np.ndarray
    hits: deque = field(default_factory=deque)
    confirmed: bool = False
    consecutive_misses: int = 0


class TemporalTracker:
    """Sliding-window detection stabiliser with hysteresis.

    A detection is returned only once its track has accumulated ``min_hits``
    hits within the last ``window_frames`` frames. Once confirmed, the track
    survives up to ``miss_tolerance`` consecutive misses before being reset.
    """

    def __init__(
        self,
        window_frames: int = 5,
        min_hits: int = 3,
        jitter_max_mm: float = 15.0,
        miss_tolerance: int = 2,
    ) -> None:
        self._window = int(window_frames)
        self._min_hits = int(min_hits)
        self._jitter_max_m = float(jitter_max_mm) / 1000.0
        self._miss_tolerance = int(miss_tolerance)
        self._tracks: list[_Track] = []

    def update(self, detections: list[Any]) -> list[tuple[Any, float]]:
        """Advance one frame; return ``(detection, temporal_score)`` for tracks
        that meet the stability threshold this frame."""
        matched_tracks: set[int] = set()
        results: list[tuple[Any, float]] = []
        used_detections: list[tuple[Any, _Track]] = []

        for det in detections:
            cls = _class_id(det)
            cen = _centroid(det)
            track = self._match(cls, cen, matched_tracks)
            if track is None:
                track = _Track(class_id=cls, centroid=cen)
                self._tracks.append(track)
            track.centroid = cen
            track.hits.append(1)
            track.consecutive_misses = 0  # reset on hit
            matched_tracks.add(id(track))
            used_detections.append((det, track))

        # Tracks not matched this frame record a miss.
        for track in self._tracks:
            if id(track) not in matched_tracks:
                track.hits.append(0)
                track.consecutive_misses += 1

        # Trim each track's window.
        for track in self._tracks:
            while len(track.hits) > self._window:
                track.hits.popleft()

        # Evict dead tracks: no hits left, OR confirmed but exceeded miss_tolerance.
        surviving: list[_Track] = []
        for t in self._tracks:
            total_hits = sum(t.hits)
            if total_hits == 0:
                continue  # completely dead
            if t.confirmed and t.consecutive_misses > self._miss_tolerance:
                t.confirmed = False  # reset confirmation
                continue  # evict
            surviving.append(t)
        self._tracks = surviving

        for det, track in used_detections:
            total_hits = sum(track.hits)
            if total_hits >= self._min_hits:
                track.confirmed = True
                results.append((det, track.score(self._window)))
            elif track.confirmed:
                # Still confirmed from hysteresis — keep publishing
                results.append((det, track.score(self._window)))
        return results

    def _match(
        self, cls: str, cen: np.ndarray, already_matched: set[int]
    ) -> _Track | None:
        best: _Track | None = None
        best_dist = self._jitter_max_m
        for track in self._tracks:
            if track.class_id != cls or id(track) in already_matched:
                continue
            dist = float(np.linalg.norm(track.centroid - cen))
            if dist <= best_dist:
                best_dist = dist
                best = track
        return best

    def score(self, _track: _Track) -> float:
        return sum(_track.hits) / self._window
```

Wait — `_Track` already has a `score` method. Let me keep it consistent:

Actually, looking at the original code, `_Track.score()` is called as `track.score(self._window)`. The `TemporalTracker` doesn't have a `score` method. Let me keep it clean.

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/hieu2/gp4_ws && python -m pytest src/gp4_perception/test/test_temporal_tracker.py -v
```

Expected: ALL PASS

- [ ] **Step 5: Update perception.yaml with miss_tolerance config**

Add under `perception:` section after `centroid_jitter_max_mm`:

```yaml
  temporal_miss_tolerance: 2
```

- [ ] **Step 6: Update scene_processor.py to pass miss_tolerance**

In `SceneProcessor.__init__`, update the `_tracker` creation (line 120):

```python
        self._tracker = TemporalTracker(
            window_frames=int(self._cfg.get("temporal_window_frames", 5)),
            min_hits=int(self._cfg.get("temporal_min_hits", 3)),
            jitter_max_mm=float(self._cfg.get("centroid_jitter_max_mm", 15.0)),
            miss_tolerance=int(self._cfg.get("temporal_miss_tolerance", 2)),
        )
```

- [ ] **Step 7: Build and run all tracker tests**

```bash
cd /home/hieu2/gp4_ws && colcon build --symlink-install --packages-select gp4_perception
python -m pytest src/gp4_perception/test/test_temporal_tracker.py -v
```

- [ ] **Step 8: Commit**

```bash
git add src/gp4_perception/gp4_perception/temporal_tracker.py \
        src/gp4_perception/test/test_temporal_tracker.py \
        src/gp4_perception/gp4_perception/scene_processor.py \
        src/gp4_perception/config/perception.yaml
git commit -m "feat: add hysteresis to temporal tracker — bbox persists through brief dropouts"
```

---

### Task 6: Fix White Workpiece Close-Range Detection

**Files:**
- Modify: `src/gp4_perception/config/perception.yaml`
- Modify: `src/gp4_perception/test/test_detection_visualizer.py`

At close range: pixel area increases beyond `max_area_px: 30000`, and blue_border/white_fill ratios shift. Current confidence at 775mm is only 0.39.

- [ ] **Step 1: Increase max_area_px and relax ratio thresholds**

In `perception.yaml`, update `white_workpiece` config:

```yaml
    - class_id: white_workpiece
      enabled: true
      detector_type: "blue_border_white_fill"
      min_area_px: 400             # was 500 — detect smaller at distance
      max_area_px: 120000          # was 30000 — allow close-range detection
      morph_kernel: 5
      require_border: null
      blue_border_hsv: [95, 50, 40, 135, 255, 255]
      white_fill_hsv: [0, 0, 120, 179, 100, 255]   # was V_min=135,S_max=90 — relax
      inner_white_min_ratio: 0.30  # was 0.45 — more tolerant at close range
      min_blue_border_ratio: 0.02  # was 0.04 — border thinner at close range
      shape_filter:
        min_aspect_ratio: 0.3      # was 0.4
        max_aspect_ratio: 5.0      # was 4.0
      reject_if_inside_class: red_box
```

Key changes:
- `max_area_px`: 30000 → 120000 (at ~300mm, a 100mm object fills ~40k+ pixels)
- `inner_white_min_ratio`: 0.45 → 0.30 (close-range perspective distortion)
- `min_blue_border_ratio`: 0.04 → 0.02 (border is proportionally thinner at close range)
- `white_fill_hsv`: Relaxed S_max 90→100 and V_min 135→120 for lighting variation

- [ ] **Step 2: Build and verify**

```bash
cd /home/hieu2/gp4_ws && colcon build --symlink-install --packages-select gp4_perception
```

- [ ] **Step 3: Commit**

```bash
git add src/gp4_perception/config/perception.yaml
git commit -m "fix: white_workpiece detection — increase area limits and relax ratios for close range"
```

---

### Task 7: Full Build & Integration Test

- [ ] **Step 1: Full build**

```bash
cd /home/hieu2/gp4_ws && colcon build --symlink-install
```

- [ ] **Step 2: Run all perception tests**

```bash
cd /home/hieu2/gp4_ws && python -m pytest src/gp4_perception/test/ -v
```

- [ ] **Step 3: Show test results**

```bash
cd /home/hieu2/gp4_ws && colcon test --packages-select gp4_perception
colcon test-result --verbose
```

- [ ] **Step 4: Show changed files**

```bash
git status --short
```

- [ ] **Step 5: Commit any remaining changes**

```bash
git add -A && git commit -m "chore: perception pipeline performance & stability optimization"
```

---

## Verification Plan

### Automated Tests
- `python -m pytest src/gp4_perception/test/test_temporal_tracker.py -v` — hysteresis tests
- `colcon test --packages-select gp4_perception && colcon test-result --verbose`

### Manual Verification (User)
1. Launch: `ros2 launch gp4_perception perception_full.launch.py`
2. Open rqt_image_view → subscribe to `/perception/debug_dashboard_image` with **raw** transport
3. Verify: terminal no longer floods with WARN messages
4. Verify: dashboard updates smoothly without lag
5. Place white_workpiece at ~400mm — verify detection
6. Move red_box slowly — verify bbox stays stable (no flickering)

### Expected Improvements
| Metric | Before | After |
|--------|--------|-------|
| Detection processing FPS | ~30 (unbounded) | ~10 (controlled) |
| Preprocessing FPS | ~30 (unbounded) | ~5 (controlled) |
| Annotated image width | ~2128px | ~960px |
| Sync queue depth | 10 | 2 |
| Terminal log spam | Hundreds/sec | Errors only |
| Bbox dropout tolerance | 0 miss frames | 2 miss frames |
| white_workpiece max_area | 30k px | 120k px |
