# gp4_perception

RealSense D435i eye-to-hand perception stack for the GP4 workcell.

For the complete operator procedure, see
[`docs/perception/d435i_hand_eye_calibration_runbook.md`](../../docs/perception/d435i_hand_eye_calibration_runbook.md).

## Dependencies

### apt
```bash
sudo apt install ros-humble-realsense2-camera ros-humble-vision-msgs \
  ros-humble-cv-bridge ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-message-filters ros-humble-moveit-ros-planning-interface \
  python3-opencv python3-numpy python3-scipy python3-yaml
```

### pip (user install)
```bash
pip install --user pyyaml numpy scipy
```

### OpenCV version check
`cv2.calibrateHandEye` requires OpenCV 4.1+. Verify:
```bash
python3 -c "import cv2; print(cv2.__version__); assert hasattr(cv2, 'calibrateHandEye'), 'calibrateHandEye missing'"
```

## Camera bring-up

```bash
ros2 launch gp4_perception camera.launch.py
```

Verify topics:
```bash
ros2 topic list | rg camera
ros2 topic info /camera/depth/color/points -v
# Confirm Reliability=BEST_EFFORT, Durability=VOLATILE (SensorDataQoS)
```

### Check camera topics in detail

```bash
# List all camera-related topics
ros2 topic list | rg camera

# Check topic publish rate
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/color/points

# Echo camera info (intrinsics) — single message
ros2 topic echo /camera/color/camera_info --once

# Echo depth image header — single message
ros2 topic echo /camera/depth/image_rect_raw --once
```

### TF2 echo — tool0 / base_link / camera

Verify the transform chain between robot frames and camera frames:

```bash
# base_link → tool0 (robot TCP)
ros2 run tf2_ros tf2_echo base_link tool0

# base_link → camera_link (camera mount position)
ros2 run tf2_ros tf2_echo base_link camera_link

# base_link → camera_color_optical_frame (used by detections)
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# tool0 → camera_link (camera relative to TCP)
ros2 run tf2_ros tf2_echo tool0 camera_link

# camera_link → camera_color_optical_frame (internal camera TF)
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

### TF tree inspection

```bash
# Generate full TF tree PDF (saves frames.pdf in current directory)
ros2 run tf2_tools view_frames

# Echo live TF and static TF topics
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

> **Tip:** If any `tf2_echo` prints `Could not transform ...` or `Lookup would
> require extrapolation into the future`, the source node is either not running
> or publishing on a different `ROS_DOMAIN_ID`.


## Calibration

The short path is below. Use the full runbook for preflight, TF checks, safety
boundaries, and troubleshooting.

1. Attach the Charuco 10x11 board to the gripper (eye-to-hand setup).
2. Launch calibration collection:
   ```bash
   ros2 launch gp4_perception calibration_collect.launch.py
   ```
3. Jog the robot through varied poses (12–24 recommended). Spread orientations.
4. Call the service:
  ```bash
  ros2 service call /perception/calibrate_hand_eye interfaces/srv/CalibrateHandEye \
     "{fiducial_id: 'charuco_10x11_20mm_15mm', min_samples: 12}"
  ```
5. Verify extrinsics:
   ```bash
   cat src/gp4_perception/config/extrinsics.yaml
   # calibration_date must be a real ISO 8601 timestamp, not <NOT_CALIBRATED>
   ```

## Full perception stack

```bash
ros2 launch gp4_perception perception_full.launch.py
```

## QoS verification

RealSense publishes with `SensorDataQoS` (BEST_EFFORT, VOLATILE). The scene processor
subscribes with matching QoS. Mismatched QoS (RELIABLE subscriber) causes silent
message drops — the callback never fires.

Verify after launch:
```bash
ros2 topic info /camera/depth/color/points -v
```

## Common failure modes

| Symptom | Cause | Fix |
|---------|-------|-----|
| Stale calibration error on launch | extrinsics.yaml has `<NOT_CALIBRATED>` or date >30 days | Re-run calibration service |
| Frame mismatch in TF tree | Missing or conflicting camera root/optical transforms | Check `base_link -> camera_link` in extrinsics and RealSense `camera_link -> camera_color_optical_frame` TF |
| Random missing depth pixels | IR projector cross-talk with fluorescent lighting | Set `emitter_enabled:=false` in launch |
| Scene processor callback never fires | QoS mismatch (RELIABLE subscriber vs BEST_EFFORT publisher) | Ensure `qos_profile_sensor_data` is used |
| MoveIt becomes unresponsive | Collision objects published too frequently | Lower detection rate or increase TTL in perception.yaml |

## Safety guards

Three guards run before any perception result is used:
1. **Calibration freshness** — rejects if >30 days old
2. **Reprojection error** — rejects if >3 mm
3. **Depth noise** — range-aware interpolated threshold from perception.yaml breakpoints

These are also checked by `tools/validate_safety_chain.py` at CI time.

## Testing

```bash
colcon test --packages-select gp4_perception
```
