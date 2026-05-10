# Manual: RealSense D435i Bring-Up for GP4 Workcell

**Date:** 2026-05-04
**Wave:** W4
**Audience:** Operator / commissioning engineer

## 0. D435i catalog facts checked for this workcell

Official RealSense catalog/datasheet facts checked on 2026-05-09:

| Item | D435i value | GP4 workcell implication |
|---|---:|---|
| Ideal operating range | 0.3 m to 3 m | Mount the camera so the table/object ROI stays inside this range. |
| Minimum depth distance at max resolution | about 28 cm | Reject or reframe object targets closer than this unless separately validated. |
| Depth accuracy | less than 2% at 2 m | Keep local depth-noise and calibration gates; do not treat catalog accuracy as a safety guarantee. |
| Depth FOV | 87 deg x 58 deg | Good table coverage is plausible, but final coverage must be verified after mounting. |
| Depth stream | up to 1280 x 720, up to 90 fps | Current D435i class is adequate for perception support. |
| RGB stream | 1920 x 1080 at 30 fps, 69 deg x 42 deg FOV | RGB FOV is narrower than depth, so aligned images may crop edge depth data. |
| Mounting | one 1/4-20 UNC point and two M3 points | Use a rigid eye-to-hand bracket; calibration is invalid if the mount moves. |

Sources:

- RealSense D435i product page: https://www.intelrealsense.com/depth-camera-d435i/
- Intel D435i specifications page: https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html

---

## 1. Physical setup

- Camera is **fixed in the cell** (eye-to-hand), NOT on the flange.
- Mount the D435i so the GP4 workspace (0.3–0.8 m from camera) fills most of the FOV.
- Ensure the camera has a clear line of sight to the table surface.
- Connect via USB 3.0 (USB-C cable). USB 2.0 works but limits frame rate and depth resolution.

## 2. Software bring-up

```bash
# Source workspace
source install/setup.bash

# Launch camera only
ros2 launch gp4_perception camera.launch.py

# Verify
ros2 topic list | grep camera
# Expected: /camera/color/image_raw, /camera/depth/color/points, /camera/color/camera_info
```

## 3. Calibration procedure

### 3.1 Prepare fiducial board

Use the ArUco board defined in `config/fiducials.yaml` (DICT_5X5_100, 5×7 grid, 35 mm markers).
Print the board at 1:1 scale. Attach it rigidly to the gripper flange.

```bash
python3 tools/generate_aruco_board.py --output aruco_board_5x7.png
```

Print the generated PNG at the reported DPI. Do not rescale it in the print
dialog; wrong scale invalidates the hand-eye solve.

### 3.2 Collect calibration data

```bash
ros2 launch gp4_perception calibration_collect.launch.py
```

In a second terminal, jog the robot to 12–24 distinct poses:
- Vary wrist orientation (roll, pitch, yaw) to condition the SVD.
- Keep the board visible in the camera at each pose.
- Avoid collinear rotations.

### 3.3 Solve calibration

```bash
ros2 service call /perception/calibrate_hand_eye interfaces/srv/CalibrateHandEye \
  "{fiducial_id: 'board_5x7', min_samples: 12}"
```

Check the response:
- `success: true`
- `reprojection_error_mm` ≤ 3.0
- `calibration_date_iso` is a valid ISO 8601 string

### 3.4 Verify extrinsics

```bash
cat src/gp4_perception/config/extrinsics.yaml
```

The `calibration_date` field must be a real timestamp, not `<NOT_CALIBRATED>`.

### 3.5 Verify TF

```bash
ros2 run tf2_tools view_frames
# base_link -> camera_color_optical_frame must exist
```

## 4. Full perception stack

```bash
ros2 launch gp4_perception perception_full.launch.py
```

This starts: camera, scene_processor, the scene-owned perception status/query
service, and tf_publisher.

## 5. Verify QoS

```bash
ros2 topic info /camera/depth/color/points -v
```

Publisher and subscriber must both show:
- Reliability: BEST_EFFORT
- Durability: VOLATILE

If subscriber shows RELIABLE, the callback will never fire.

## 6. Verify detections

Place a known-size box on the table within the workspace ROI.

```bash
ros2 topic echo /perception/detections --once
```

Expected: a Detection3DArray with at least one entry, pose in base_link frame.

```bash
ros2 service call /perception/get_object_positions interfaces/srv/GetObjectPositions \
  "{class_filter: ''}"
```

Expected: `ok: true` with the same detections after calibration. Before
calibration, this service must reject fail-closed with `calibration_invalid`.
For an object target to be usable by the LLM/ReAct motion planner, the response
must also report:

- `calibration_date_iso`: a real ISO 8601 timestamp from the hand-eye solve.
- `calibration_age_days`: within the configured freshness window.
- `depth_in_range: true`.
- `depth_noise_mm_p95`: populated from recent depth samples, not the startup
  default.

If `depth_in_range` is false or calibration metadata is missing, do not use the
object pose for motion. Re-check lighting, camera range, calibration, and the
depth profile before re-querying.

## 7. Verify safety chain

```bash
python3 tools/validate_safety_chain.py
```

Must exit 0 after calibration. Before calibration, it will report the `<NOT_CALIBRATED>` error (expected).

## 8. IR cross-talk

If depth quality is poor (random holes, noisy edges) under fluorescent lighting:
```bash
ros2 launch gp4_perception camera.launch.py emitter_enabled:=false
```

This disables the IR projector and relies on ambient IR. Depth range may decrease.

## 9. Troubleshooting checklist

- [ ] Camera USB connected (USB 3.0 preferred)
- [ ] `ros2 topic list` shows camera topics
- [ ] QoS matches (BEST_EFFORT on both sides)
- [ ] extrinsics.yaml has valid calibration_date
- [ ] TF tree includes base_link → camera_color_optical_frame
- [ ] `validate_safety_chain.py` passes
- [ ] Box on table appears in /perception/detections
