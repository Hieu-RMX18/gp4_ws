# RealSense D435i Hand-Eye Calibration Runbook

This runbook brings up the Intel RealSense D435i for the GP4 workcell and
solves the eye-to-hand transform used by perception:

```text
base_link -> camera_color_optical_frame
```

The camera is fixed to the station. The ArUco board is rigidly attached to the
robot tool or end effector and moves with the robot during sample collection.

## Safety Boundary

- Do not enable hardware execution from this procedure.
- Jog the GP4 manually and slowly while collecting calibration samples.
- Keep the emergency stop reachable during all robot motion.
- Treat the RealSense as non-safety-rated perception only.
- Do not use perception detections for robot motion unless calibration,
  depth-quality checks, SafeGate validation, and human approval all pass.
- Keep `hardware_execute` disabled unless a separate hardware-execution wave
  explicitly allows it.

## Preconditions

Use a terminal in the workspace root:

```bash
cd /home/hieu2/gp4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Verify the local software stack:

```bash
ros2 pkg prefix realsense2_camera
ros2 pkg prefix gp4_perception
ros2 interface show interfaces/srv/CalibrateHandEye
python3 -c "import cv2; print(cv2.__version__); print(hasattr(cv2, 'calibrateHandEye'), hasattr(cv2, 'aruco'))"
```

Expected:

- `realsense2_camera` resolves under the ROS 2 Humble install.
- `gp4_perception` resolves under this workspace install.
- OpenCV prints `True True` for `calibrateHandEye` and `aruco`.

Check the USB device before launching ROS:

```bash
lsusb | rg "8086|Intel|RealSense"
rs-enumerate-devices
```

If no D435i is listed, stop here and fix cabling, USB3, power, udev rules, or
camera availability before continuing.

## Camera-Only Bringup

Launch the camera without the rest of the robot stack:

```bash
ros2 launch gp4_perception camera.launch.py serial:=<D435I_SERIAL>
```

If there is only one RealSense connected, `serial:=` can be omitted. If multiple
cameras are connected, always pass the D435i serial.

In another sourced terminal, verify topics:

```bash
ros2 topic list | rg camera
ros2 topic info /camera/depth/color/points -v
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/color/points
```

Expected topics include:

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/color/points`

Expected QoS for RealSense sensor streams:

- Reliability: `BEST_EFFORT`
- Durability: `VOLATILE`

If `/camera/depth/color/points` is missing, confirm `pointcloud.enable` is true
in `src/gp4_perception/launch/camera.launch.py` and that depth/color streams
are healthy.

## Calibration Target

Generate the board from the same YAML that calibration uses:

```bash
python3 tools/generate_aruco_board.py --output /tmp/aruco_board_5x7.png --dpi 300
```

Print at 100 percent scale. Do not fit-to-page. After printing, measure one
marker edge with calipers or a ruler and confirm it is 35 mm, matching:

```bash
rg -n "marker_length_m" src/gp4_perception/config/fiducials.yaml
```

Mount rules:

- Attach the board rigidly to the gripper, tool plate, or temporary tool frame.
- The board must not flex or slip during motion.
- Keep the board fully visible to the camera in each sample.
- Keep the D435i fixed during the entire calibration.
- Record the physical mount notes in the operator log.

## TF Preflight

Calibration samples are accepted only when both the marker pose and robot pose
are available at the image timestamp. Confirm the robot TF path before solving:

```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

Expected:

- A continuous transform from `base_link` to `tool0`.
- No repeated extrapolation or missing-frame errors.

If this fails, fix robot state publishing, joint states, namespacing, or the
MoveIt/driver bringup before running calibration.

## Hand-Eye Calibration Run

Launch the calibration collection stack:

```bash
ros2 launch gp4_perception calibration_collect.launch.py serial:=<D435I_SERIAL>
```

Collect 12 to 24 samples by jogging through varied poses:

- Spread the board across the camera field of view.
- Vary wrist orientation, not just XYZ position.
- Avoid near-identical poses.
- Avoid poses where the board is partly occluded, blurred, or near image edges.
- Prefer slow stops before each sample so image and TF timestamps are stable.

The calibration service logs each accepted sample. Once enough samples have
been collected, solve:

```bash
ros2 service call /perception/calibrate_hand_eye interfaces/srv/CalibrateHandEye \
  "{fiducial_id: 'board_5x7', min_samples: 12}"
```

Expected success response:

- `success: true`
- `failure_reason: ''`
- `n_samples_collected >= 12`
- `reprojection_error_mm <= 3.0`
- `extrinsics_yaml_path` points to the installed `gp4_perception` config path

With the current symlink-install layout, the installed config path resolves back
to `src/gp4_perception/config/extrinsics.yaml`. Treat that file as a calibrated
artifact and review it before committing.

## Verification

Inspect the generated calibration:

```bash
cat src/gp4_perception/config/extrinsics.yaml
python3 tools/validate_safety_chain.py
```

The `hand_eye_extrinsics` block must have:

- `parent_frame: base_link`
- `child_frame: camera_color_optical_frame`
- a real ISO 8601 `calibration_date`, not `<NOT_CALIBRATED>`
- `reprojection_error_mm <= 3.0`
- `n_samples >= 12`

Launch the full perception stack:

```bash
ros2 launch gp4_perception perception_full.launch.py serial:=<D435I_SERIAL>
```

Verify status and object query behavior:

```bash
ros2 topic echo /perception/status
ros2 service call /perception/get_object_positions interfaces/srv/GetObjectPositions \
  "{class_filter: ''}"
```

Expected status progression:

- `DISABLED` when calibration is missing, stale, or invalid.
- `DEGRADED` when calibration is valid but depth quality is not ready or out of
  range.
- `READY` only when calibration and depth-quality gates both pass.

## Troubleshooting

| Symptom | Likely Cause | Check |
| --- | --- | --- |
| D435i absent from `lsusb` | USB/cable/power/udev issue | `rs-enumerate-devices` |
| Camera launches but no point cloud | Point cloud disabled or depth stream unhealthy | `/camera/depth/color/points` |
| Topic exists but callbacks never fire | QoS mismatch | `ros2 topic info <topic> -v` |
| No calibration samples collected | Missing `base_link -> tool0` TF, missing `CameraInfo`, or marker not detected | `tf2_echo`, camera image, node logs |
| Reprojection error over 3 mm | Weak pose diversity, bad marker scale, board motion, or blurry samples | Reprint/measure board and recollect poses |
| Perception remains disabled | Missing/stale `extrinsics.yaml` or high reprojection error | `tools/validate_safety_chain.py` |

## Acceptance Checklist

- [ ] D435i appears in `lsusb` and `rs-enumerate-devices`.
- [ ] `camera.launch.py` publishes color, camera-info, and point-cloud topics.
- [ ] Sensor topics use `BEST_EFFORT` and `VOLATILE` QoS.
- [ ] Printed ArUco marker length is verified at 35 mm.
- [ ] `tf2_echo base_link tool0` works before collection.
- [ ] Calibration service succeeds with at least 12 samples.
- [ ] `reprojection_error_mm` is no more than 3.0.
- [ ] `tools/validate_safety_chain.py` accepts the calibration.
- [ ] `/perception/status` reaches `READY` only after calibration and depth checks.
- [ ] No command in this runbook enables hardware trajectory execution.
