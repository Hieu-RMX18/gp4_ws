# Perception RGB-D Detection Fix + Temporal + Calibration Cleanup

Date: 2026-05-29
Status: Approved (design)
Scope: `gp4_perception` package — 3 waves, Wave 3 ships as a separate PR.

## Problem

Detection visualizer shows a `flat` bounding box aimed at the power-strip/sockets on
the rail, while the red box and the white/blue-border rectangle beside it are never
detected.

### Root cause (verified against code)

The label being bare `flat` (not `red_box`) is the decisive evidence: per
`scene_geometry._semantic_class_id`, a bare shape name is only returned when
`color == "unknown"`. **Color is `unknown` for every cluster.**

Why color is always unknown — `scene_processor._on_synced` color lookup:

```python
tree = _cKDTree(xyz_all)                       # xyz_all is in CAMERA frame
_, idx = tree.query(cluster[: min(200, len)])  # cluster is in BASE_LINK frame
```

Cluster points were transformed to `base_link`, but the KDTree is built on the
camera-frame cloud. Querying base_link coordinates against a camera-frame tree returns
nonsense nearest neighbors → random RGB → desaturated mean → `unknown`. The `indices`
array that was meant to track RGB is built, partially updated at ROI crop, then never
used (`_voxel_downsample` / `_ransac_plane` slice points without touching it).

Secondary factors:
- `_dominant_color_name` uses **mean** RGB. Text/logo on the red box pollutes the mean
  even if the frame bug were fixed.
- `_ransac_plane` removes the single largest plane with no normal check, so the red
  box's large top face is removed together with the table.
- `score = 1.0` hard-coded → no confidence signal → junk clusters (sockets) publish.
- No temporal voting → detections flicker, centroid jumps.

## Design

### Wave 1 — Detection fix (minimal)

Principle: the point cloud already carries per-point RGB aligned with XYZ. Carry that
RGB array through every filtering step by index — exact, no KDTree, no frame mismatch,
no camera-intrinsics projection.

`scene_geometry.py`:
- `_voxel_downsample_indices(pts, voxel) -> np.ndarray` — the kept indices.
  `_voxel_downsample` is refactored to call it (back-compat preserved).
- `_ransac_plane_fit(pts, threshold, max_iter, normal_z_min) -> (inlier_idx, outlier_idx, normal)`.
  Plane is only removed when `abs(normal_z) >= normal_z_min` (horizontal table/floor).
  Vertical object faces (normal_z below threshold) are kept. `_ransac_plane` is
  refactored to call it with `normal_z_min=0.0` (back-compat).
- `_euclidean_cluster_indices(pts, tol, min, max) -> list[np.ndarray]`.
  `_euclidean_clusters` refactored to call it (back-compat).
- `_dominant_color_voting(cluster_rgb) -> (name, confidence)` — per-pixel HSV vote over
  `_COLOR_RANGES`. Chromatic colors are preferred when their vote ratio exceeds
  `_CHROMATIC_MIN_RATIO` (handles a white object with a thin blue border). Confidence is
  the winning ratio.
- `_confidence_score(color_conf, geometry_conf, depth_conf, temporal_conf=None)` —
  weighted multi-factor score, weights renormalized when `temporal_conf is None`.

`scene_processor.py` `_on_synced`:
- Carry `rgb` alongside `pts` through ROI crop, voxel, RANSAC (slice by the same
  indices). Order is preserved by `_transform_points`, so `rgb_all` stays aligned.
- Replace the KDTree color block with exact `cluster_rgb = rgb[cluster_idx]` → voting.
- Compute `hyp.score` from `_confidence_score`; publish only when
  `score >= min_publish_confidence`.
- Pass `ransac_plane_normal_z_min` from config.

`config/perception.yaml`:
- `ransac_plane_normal_z_min: 0.85`
- `min_publish_confidence: 0.55`

Out of scope for Wave 1: 3D physical-size filter (no measured object dimensions yet);
HSV-2D color-guided clustering (deferred to a possible Wave 1b if exact voting proves
insufficient on the white/blue-border rectangle).

### Wave 2 — Temporal voting

`temporal_tracker.py` (new): sliding window of `temporal_window_frames` (5), matches
current detections to tracked objects by centroid proximity, returns detections stable
for `temporal_min_hits` (3) with centroid jitter `< centroid_jitter_max_mm` (15 mm) plus
a per-detection `temporal_score`.

`scene_processor._on_synced`: feed detections through the tracker before caching; fold
`temporal_score` into `_confidence_score` (weights renormalized to include it).

Config: `temporal_window_frames`, `temporal_min_hits`, `centroid_jitter_max_mm`.

### Wave 3 — Calibration solver cleanup (separate PR)

`calibration.py`: `_HAND_EYE_METHODS` reduced from 5 to 2 — `PARK` (primary) and
`DANIILIDIS` (cross-check). Replace `_solve_best_method` with
`_solve_park_with_crosscheck`: solve with PARK, cross-check with DANIILIDIS, warn when
residuals differ by `> 2 mm` ("solvers disagree, data may be noisy"); always return PARK
unless PARK fails.

## Testing

New unit tests (pytest):
- `test_color_voting.py` — voting vs mean on multi-color patches; chromatic preference
  over a dominant achromatic background; confidence is the winning ratio.
- RANSAC normal-z gate: a vertical plane is kept; a horizontal plane is removed.
- `_voxel_downsample_indices` / `_euclidean_cluster_indices` index correctness (RGB stays
  aligned through the pipeline).
- `_confidence_score` formula + renormalization with/without temporal.
- `test_temporal_tracker.py` — sliding-window stability, jitter rejection, class
  consistency.
- Calibration: only PARK + DANIILIDIS run; disagreement warning fires.

Existing tests must not regress (back-compat wrappers preserve current signatures).

## Verification

```bash
colcon build --symlink-install --packages-select gp4_perception
colcon test --packages-select gp4_perception --output-on-failure
colcon test-result --packages-select gp4_perception --verbose
```

Manual: launch `perception_full.launch.py`, place red_box + blue_rectangle; verify
red_box detected with `class_id=red_box` and score above threshold, sockets filtered out.
