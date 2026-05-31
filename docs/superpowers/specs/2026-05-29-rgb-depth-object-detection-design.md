# RGB + Aligned-Depth Object Detection Design

Date: 2026-05-29

## Goal

Implement a fast RGB-first perception path for tabletop objects using:

- `/camera/color/image_raw` for 2D object masks and bounding boxes.
- `/camera/aligned_depth_to_color/image_raw` for aligned depth.
- `/camera/color/camera_info` for intrinsics.
- TF2 for `camera_color_optical_frame` → `base_link` point transforms.

The path must not use `PointCloud2` RGB for color classification and must not call any robot execution path.

## Architecture

Repurpose `gp4_perception/detection_visualizer.py` from a 3D-detection overlay subscriber into the owner of the RGB + aligned-depth detection pipeline.

`scene_processor.py` remains available for pointcloud debug/fallback, collision object publication, `/perception/debug_clusters`, and the existing `GetObjectPositions` service. It stops publishing `/perception/detections`, so the new RGB path is the single owner of that topic.

No new runtime source file is required.

## Runtime data flow

```text
/camera/color/image_raw ┐
/camera/aligned_depth_to_color/image_raw ├─ ApproxTimeSynchronizer ─► RGB detector
/camera/color/camera_info ─ cached intrinsics ┘

RGB detector:
  HSV mask per configured class
  → morphology cleanup
  → contour extraction and bbox filtering
  → median depth over object mask/bbox
  → deproject bbox center to camera XYZ
  → TF2 PointStamped camera_color_optical_frame → base_link
  → temporal stability filter
  → publish /perception/detections
  → publish /perception/annotated_image
```

## Detection classes

Configuration lives in `src/gp4_perception/config/perception.yaml`, not as hardcoded HSV tables in the node.

Initial class targets:

1. `red_box` — HSV red wraparound ranges.
2. `yellow_ball` — HSV yellow range.
3. `apple` / `orange` — HSV orange/red ranges with roundness filtering.
4. `white_workpiece` — bright/low-saturation white mask plus blue-border validation.

`red_box` is the first priority. Other class entries are present and tunable but can be tightened after camera captures.

## Depth and XYZ

For `16UC1` depth:

```text
depth_m = raw_depth * 0.001
```

Depth is computed from the median of valid nonzero pixels inside the object mask within the bbox. A single center pixel is not used.

Camera coordinates use the color-camera intrinsics:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = median_depth_m
```

Camera XYZ frame is `camera_color_optical_frame`.

## TF behavior

The detector attempts a non-blocking TF lookup from `base_link` to `camera_color_optical_frame` and transforms a `PointStamped` into `base_link`.

If TF is unavailable:

- The overlay still shows `camera_xyz`.
- The overlay shows `base_xyz=TF_UNAVAILABLE`.
- Published detections use `camera_color_optical_frame` so the 3D point is still meaningful.

If depth is invalid:

- The bbox is still drawn.
- The overlay shows `XYZ_INVALID`.
- The detection is not published because it has no valid 3D point.

## `/perception/detections`

The new RGB path publishes stable `vision_msgs/Detection3DArray` detections.

- Header frame is `base_link` when TF succeeds.
- Header frame is `camera_color_optical_frame` when TF is unavailable.
- Pose position stores `base_xyz` or `camera_xyz` according to the header frame.
- `bbox.size.x` and `bbox.size.y` are estimated from pixel bbox dimensions and median depth using intrinsics.
- `bbox.size.z` uses a small nominal thickness because the image path estimates a representative point, not a full 3D pointcloud extent.
- Confidence combines class mask quality, contour/shape quality, depth validity, and temporal stability.

The execution service remains in `scene_processor.py`, so this topic ownership change does not add a robot execution path.

## Overlay

`/perception/annotated_image` shows:

- RGB image with bbox.
- `class_id`.
- confidence.
- `distance_m`.
- `camera_xyz`.
- `base_xyz`, or `TF_UNAVAILABLE`.
- `XYZ_INVALID` when depth is invalid.

By default, the annotated image also includes a side-by-side aligned-depth colormap panel to match the reference screenshots:

```text
RGB overlay | aligned-depth colormap
```

This is controlled by `visualization.show_depth_panel: true` in `perception.yaml`.

## Files changed

- `src/gp4_perception/gp4_perception/detection_visualizer.py`
  - Delete old 3D bbox projection subscriber logic.
  - Add RGB+aligned-depth subscriptions, detection, median-depth, deprojection, TF, overlay, and `Detection3DArray` publisher.

- `src/gp4_perception/gp4_perception/scene_processor.py`
  - Stop publishing `/perception/detections`.
  - Keep `GetObjectPositions`, collision objects, and debug markers.

- `src/gp4_perception/config/perception.yaml`
  - Add RGB detector topics/classes/morphology thresholds.
  - Add `visualization.show_depth_panel`.

- Tests
  - Update scene processor tests that assumed `_det_pub` exists.
  - Add focused pure-function tests for median depth, deprojection, and invalid-depth behavior.

## Non-goals

- No `/execute_motion` calls.
- No robot motion or planning changes.
- No new ROS interface definitions.
- No new source module unless implementation proves the existing files cannot remain maintainable.
- No use of `PointCloud2` RGB for primary color classification.

## Acceptance checks

- `red_box` has a visible bbox on `/perception/annotated_image`.
- `distance_m` is reasonable.
- `camera_xyz` is populated when depth is valid.
- `base_xyz` is populated when TF is available.
- `base_xyz=TF_UNAVAILABLE` is displayed when TF is missing.
- `XYZ_INVALID` is displayed when depth is invalid.
- `/perception/detections` publishes stable detections from the RGB+depth path.
- Existing pointcloud cluster detector remains available as debug/fallback only.
