# Manual: RealSense D435i Bring-Up for GP4 Workcell

**Date:** 2026-05-04
**Wave:** W4
**Audience:** Operator / commissioning engineer

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

This starts: camera, scene_processor, status_publisher, object_query_service, tf_publisher.

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
