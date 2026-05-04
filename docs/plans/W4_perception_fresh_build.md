# W4 — Perception Fresh Build (RealSense D435i, Eye-to-Hand)

**Wave class:** New capability + new safety dependency
**Risk:** High (new hardware integration; calibration drift is invisible failure mode)
**Estimated effort:** 5–7 working days (informed by W0's review of `36520035`)
**Depends on:** W3 (`query_perception` stub registered in ReAct registry; W4 fills the body)
**Unblocks:** W5 (HMI consolidation needs perception's ROS surface stable)

---

## Goal

Build `gp4_perception` as a fresh package on `ws-deep-rebuild-3526`. The W0 review of commit `36520035` (`docs/perception/REVIEW_OF_36520035.md`) provides design references but is **not** copy-pasted. Fresh code, informed by what the previous attempt did wrong (which the review document spelled out).

Five deliverables:

1. RealSense D435i launch with the right depth profile for GP4's 0.3–0.8 m workspace.
2. Hand-eye calibration service that solves `AX = XB` with `cv2.calibrateHandEye` (PARK method) and writes `calibration_date` at solve time, never as a template literal.
3. Scene processor: ROI crop → voxel downsample → RANSAC plane removal → Euclidean clustering → PCA bounding box, with `ApproximateTimeSynchronizer` and **publisher-matching QoS** (RealSense uses `SensorDataQoS`).
4. Two safety guards in the safety chain (W1's): calibration freshness (≤30 days), reprojection error (≤3 mm), depth noise (≤5 mm at the GP4 working distance — range-aware, not flat).
5. `query_perception` tool body that fills W3's stub, gated on `RobotState == IDLE`.

---

## Discovery (paste raw output)

```bash
# A. Confirm perception package does not already exist on this branch
ls src/ | rg -i percep
git ls-files | rg "src/gp4_perception"

# B. Read the W0 review document
cat docs/perception/REVIEW_OF_36520035.md | head -120

# C. RealSense package availability
ros2 pkg prefix realsense2_camera 2>/dev/null
ros2 pkg list | rg -i realsense

# D. PCL / OpenCV / cv_bridge / vision_msgs availability
ros2 pkg prefix pcl_ros pcl_conversions cv_bridge vision_msgs 2>/dev/null
apt list --installed 2>/dev/null | rg -e 'libopencv\|libpcl'

# E. ArUco / Charuco library for calibration
ros2 pkg prefix image_proc image_pipeline 2>/dev/null
python3 -c "import cv2; print(cv2.__version__); print(hasattr(cv2, 'calibrateHandEye'))"

# F. Confirm joint_states and TF flow (calibration depends on FK)
ros2 topic info /yaskawa/joint_states -v 2>/dev/null
ros2 run tf2_tools view_frames 2>/dev/null

# G. Camera physical setup (operator must answer if unknown)
echo "Operator must confirm: camera is fixed in the cell (eye-to-hand), not on the flange."
echo "Operator must confirm: distance from camera to GP4 base origin (approx. m)."

# H. Whether the safety chain validator from W1 expects perception inputs
rg -n "calibration|perception" tools/validate_safety_chain.py
```

---

## Tasks
### W4.T0 — Perception interface contract (precondition; per F6)

**Why:** F1 showed that primitive_blended_sequence.cpp existed but had no public path because the action interface lacked the field. Same risk applies to perception: building gp4_perception with services like `/perception/calibrate_hand_eye`, `/perception/get_object_positions`, and topic `/perception/detections` without first defining the message and service interfaces leads to:
- HMI consumes ad-hoc message shapes and breaks on every iteration.
- Other ROS nodes cannot subscribe in a typed way.
- Re-running the calibration with a different field shape silently breaks downstream.

We define the contract first, build the package against the contract second.

**Tasks:**

1. **`src/interfaces/srv/CalibrateHandEye.srv`** (new):
-Request
string fiducial_id           # ArUco / Charuco board id
uint16 min_samples           # minimum N pose pairs; reject if collected < min
-Response
bool success
string failure_reason        # empty on success
string extrinsics_yaml_path  # path to written file on success
float64 reprojection_error_mm
uint16 n_samples_collected
string calibration_date_iso  # filled at solve time, never templated (per existing W4 rule)

2. **`src/interfaces/srv/GetObjectPositions.srv`** (new):
-Request
string class_filter          # empty = all classes
Response
bool ok
string failure_reason        # empty on success; e.g. "calibration_invalid", "perception_blocked_during_motion"
vision_msgs/Detection3D[] detections   # poses already transformed to base_link

3. **`src/interfaces/srv/CheckCamera.srv`** (new):

-Response
bool connected
string firmware_version
string serial_number
float64 frame_rate_color_hz
float64 frame_rate_depth_hz
string failure_reason        # empty when connected

4. **`src/interfaces/msg/PerceptionStatus.msg`** (new) — published periodically on `/perception/status`:
builtin_interfaces/Time stamp
bool calibration_valid
string calibration_date_iso  # propagated from extrinsics.yaml
float64 calibration_age_days
bool depth_in_range          # current depth-noise sample within budget
float64 depth_noise_mm_p95   # rolling P95 over last 50 samples
string capability            # "READY" | "DEGRADED" | "DISABLED"
string detail

5. **Register** in `src/interfaces/CMakeLists.txt` (`rosidl_generate_interfaces(... ADD_LINTER_TESTS)` block) and ensure `vision_msgs`, `geometry_msgs`, `builtin_interfaces` are in `package.xml` `<depend>`.

6. **Build and verify generated bindings:**
```bash
   colcon build --packages-select interfaces
   ros2 interface show interfaces/srv/CalibrateHandEye
   ros2 interface show interfaces/srv/GetObjectPositions
   ros2 interface show interfaces/srv/CheckCamera
   ros2 interface show interfaces/msg/PerceptionStatus
```
   Pass criterion: each `ros2 interface show` returns the spec without error.

7. **HMI compatibility (per W0.T9 + F5).**
   - Update `docs/hmi/HMI_ROS_INTERFACES.md` to add the four new interfaces with `Change sensitivity = LOW` (HMI does not yet consume them; future HMI features may).
   - If HMI plans to surface calibration status, document the channel: `/perception/status` (PerceptionStatus.msg).

**No-conflict guarantee:** four NEW interface definitions, zero modification to existing interfaces. Compilation of any other package that does not yet `<depend>` on these stays untouched.

**Stop signal:** Tasks 1–6 verified. Then W4.T1 may proceed (build gp4_perception consuming these typed interfaces, NOT inventing message shapes inline).

### W4.T1 — Create `src/gp4_perception/` package

Standard ROS 2 layout, fresh:

```
src/gp4_perception/
├── package.xml
├── CMakeLists.txt              # only if any C++; W4 is Python-first
├── setup.py
├── setup.cfg
├── resource/gp4_perception
├── gp4_perception/
│   ├── __init__.py
│   ├── camera_launcher.py      # NEW; replaces the old realsense_health_node logic
│   ├── calibration_service.py  # implements interfaces/srv/CalibrateHandEye.srv (W4.T0)
│   ├── object_query_service.py # implements interfaces/srv/GetObjectPositions.srv (W4.T0)
│   ├── camera_check_service.py # implements interfaces/srv/CheckCamera.srv (W4.T0)
│   ├── status_publisher.py     # publishes interfaces/msg/PerceptionStatus.msg (W4.T0)
│   ├── scene_processor.py      # cloud → detections
│   ├── query_perception_tool.py # body of W3's stub
│   ├── safety_guards.py        # freshness / reprojection / depth-noise checks
│   └── tf_publisher.py         # static transform broadcaster from extrinsics YAML
├── config/
│   ├── d435i.yaml              # camera params for GP4 workspace
│   ├── extrinsics.yaml         # RUNTIME-FILLED; commit empty/template ONLY
│   ├── extrinsics_schema.yaml  # validates extrinsics.yaml shape
│   ├── perception.yaml         # ROI crop, voxel size, RANSAC threshold, depth noise table
│   └── fiducials.yaml          # ArUco/Charuco board spec
├── launch/
│   ├── camera.launch.py
│   ├── calibration_collect.launch.py
│   └── perception_full.launch.py
├── test/
│   ├── test_calibration_service.py
│   ├── test_scene_processor.py
│   ├── test_safety_guards.py
│   └── test_query_perception.py
└── README.md
```

`package.xml` declares deps: `rclpy`, `sensor_msgs`, `geometry_msgs`, `vision_msgs`, `tf2_ros`, `cv_bridge`, `pcl_ros`, `pcl_conversions`, `realsense2_camera`. Document the OpenCV version requirement (`cv2.calibrateHandEye` requires 4.1+).

### W4.T2 — Camera launch with correct QoS and depth profile

`launch/camera.launch.py`:

- Wraps `realsense2_camera_node`.
- Args: `align_depth.enable: true`, `enable_sync: true`, `pointcloud.enable: true`.
- Depth profile: `848x480x30` (proven for 0.3–0.8 m). Configurable via launch arg.
- IR emitter on by default; arg to disable when external IR present.
- Frames: `camera_link`, `camera_color_optical_frame`, `camera_depth_optical_frame` (the standard RealSense conventions). The static transform from `base_link` to `camera_link` is published by `tf_publisher.py` reading `config/extrinsics.yaml`.

`config/d435i.yaml`:

```yaml
realsense:
  device_serial: "<RUNTIME>"   # operator fills or auto-discovers
  align_depth_enable: true
  enable_sync: true
  pointcloud_enable: true
  depth_profile: "848x480x30"
  emitter_enabled: true
```

### W4.T3 — Hand-eye calibration service

`gp4_perception/calibration_service.py`:

ROS 2 service `/perception/calibrate_hand_eye` (request: ArUco/Charuco board id; response: success bool + extrinsics file path).

Flow:

1. Operator places the board on the flange (not on the table) for eye-to-hand: AX=XB with the board attached to the gripper, robot moves through N poses, camera observes the board from the fixed cell mount.
2. Service collects pose pairs: `(T_base_to_gripper, T_camera_to_target)`. Minimum N=12, recommended N=24. Spread pose orientations so the SVD is well-conditioned.
3. Solver: `cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_PARK)`.
4. Compute reprojection error: project board corners using the solved extrinsics back into the image, compare with detected corners. RMS in mm.
5. Write `config/extrinsics.yaml`. **`calibration_date` is set with `datetime.utcnow().isoformat() + "Z"` at write time.** Never templated. The committed `extrinsics.yaml` in git is either empty or contains a literal placeholder string `<NOT_CALIBRATED>` that the safety guard rejects.

YAML shape:

```yaml
hand_eye_extrinsics:
  parent_frame: base_link
  child_frame:  camera_color_optical_frame
  translation:  {x: <runtime>, y: <runtime>, z: <runtime>}
  rotation_quat:{x: <runtime>, y: <runtime>, z: <runtime>, w: <runtime>}
  calibration_date: "<runtime ISO 8601 UTC>"
  reprojection_error_mm: <runtime>
  n_samples: <runtime>
  solver: PARK
  workspace_distance_m: <runtime>   # mean camera-to-target distance during calibration
```

The `workspace_distance_m` field feeds the range-aware depth-noise guard.

Frame conventions: OpenCV uses image-coordinate optical frames. ROS uses right-hand FRD. The translation/rotation must be in `base_link` ↔ `camera_color_optical_frame`. Document the conversion explicitly in the service docstring; the W0 review document called this out as a frequent debugging trap.

### W4.T4 — TF publisher

`gp4_perception/tf_publisher.py`:

On startup, reads `config/extrinsics.yaml`. If `calibration_date` is missing or `<NOT_CALIBRATED>`, refuses to publish and logs ERROR. Otherwise, broadcasts the static transform `base_link → camera_color_optical_frame`. No dynamic updates; calibration changes require service re-call and re-launch.

### W4.T5 — Scene processor with sync + matching QoS

`gp4_perception/scene_processor.py`:

```python
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import PointCloud2, CameraInfo

class SceneProcessor(Node):
    def __init__(self):
        super().__init__('scene_processor')
        # CRITICAL: subscriber QoS must match RealSense publishers.
        # RealSense publishes with SensorDataQoS (best-effort, depth=5).
        # Default subscriber QoS is RELIABLE; mismatch causes ApproximateTimeSynchronizer
        # to silently drop every message. The callback would never fire.
        # Verify after launch with: ros2 topic info /camera/depth/color/points -v
        self._cloud_sub = Subscriber(self, PointCloud2,
                                      '/camera/depth/color/points',
                                      qos_profile=qos_profile_sensor_data)
        self._info_sub  = Subscriber(self, CameraInfo,
                                      '/camera/color/camera_info',
                                      qos_profile=qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer(
            [self._cloud_sub, self._info_sub], queue_size=10, slop=0.05)
        self._sync.registerCallback(self._on_synced)
        ...
```

Pipeline:

1. ROI crop using `perception.workspace_bbox_m` (from `config/perception.yaml`). Anything outside the GP4 reachable cube is dropped.
2. Voxel downsample (default 0.005 m).
3. RANSAC plane removal: largest plane is the table; remove it.
4. Euclidean clustering on the remainder.
5. For each cluster: PCA → oriented bounding box → centroid pose. Compute depth noise as the std-dev of z-values within a 5x5 patch around the centroid.
6. Reject clusters where depth noise > the range-aware threshold (see W4.T6 below).
7. Publish `vision_msgs/Detection3DArray` on `/perception/detections`. Push each surviving detection as a MoveIt `CollisionObject` via `planning_scene_interface.applyCollisionObjects`. TTL: remove if not re-detected for 2 s.

`config/perception.yaml`:

```yaml
perception:
  workspace_bbox_m:
    x: [0.20, 0.55]
    y: [-0.30, 0.30]
    z: [0.00, 0.40]
  voxel_size_m: 0.005
  ransac_distance_threshold_m: 0.005
  cluster_tolerance_m: 0.02
  cluster_min_size: 50
  cluster_max_size: 5000
  detection_ttl_s: 2.0
  depth_noise:
    # Range-aware threshold. D435i noise grows with distance; one flat number is wrong.
    # Linear interpolation between the breakpoints below.
    breakpoints:
      - {distance_m: 0.30, noise_mm_max: 2.0}
      - {distance_m: 0.50, noise_mm_max: 3.5}
      - {distance_m: 0.80, noise_mm_max: 6.0}
    extrapolation: "reject"   # outside breakpoints → reject detection
```

### W4.T6 — Safety guards

`gp4_perception/safety_guards.py`:

```python
def check_calibration_freshness(extrinsics_yaml: dict, max_age_days: int) -> tuple[bool, str]:
    date_str = extrinsics_yaml.get("hand_eye_extrinsics", {}).get("calibration_date")
    if not date_str or date_str == "<NOT_CALIBRATED>":
        return False, "calibration_date missing — run /perception/calibrate_hand_eye"
    cal_date = datetime.fromisoformat(date_str.rstrip("Z"))
    age_days = (datetime.utcnow() - cal_date).days
    if age_days > max_age_days:
        return False, f"calibration is {age_days} days old (max {max_age_days})"
    return True, ""

def check_reprojection_error(extrinsics_yaml: dict, max_mm: float) -> tuple[bool, str]:
    err = extrinsics_yaml.get("hand_eye_extrinsics", {}).get("reprojection_error_mm")
    if err is None:
        return False, "reprojection_error_mm missing"
    if err > max_mm:
        return False, f"reprojection_error_mm = {err} > max {max_mm}"
    return True, ""

def check_depth_noise(detection_distance_m: float, observed_noise_mm: float, breakpoints: list) -> tuple[bool, str]:
    threshold = _interpolate_threshold(detection_distance_m, breakpoints)
    if threshold is None:
        return False, f"distance {detection_distance_m} m outside calibrated breakpoints"
    if observed_noise_mm > threshold:
        return False, f"depth_noise {observed_noise_mm:.2f} mm > threshold {threshold:.2f} mm at {detection_distance_m:.2f} m"
    return True, ""
```

Wire into `tools/validate_safety_chain.py` (W1's). The script now also loads `src/gp4_perception/config/extrinsics.yaml` and runs the freshness + reprojection checks. Exit non-zero on failure.

SSOT additions in `src/safety/config/safety_rules.yaml`:

```yaml
safety:
  calibration:
    max_age_days: 30
    max_reprojection_error_mm: 3.0
```

### W4.T7 — `query_perception` tool body (fills W3 stub)

`gp4_perception/query_perception_tool.py` exports a function that the W3 ReAct registry imports. The W3 stub file `src/llm_gateway/llm_gateway/react/tools/query_perception.py` is updated to call this implementation when the perception package is installed.

Behaviour:

```python
def invoke(self, args: dict, context: AgentContext) -> ToolResult:
    state = context.state_injector.snapshot()
    if state["mode"] != "IDLE":
        return ToolResult(ok=False,
            error=f"perception_blocked_during_motion (mode={state['mode']})")

    extrinsics = self._load_extrinsics()
    ok, reason = check_calibration_freshness(extrinsics, self._max_age_days)
    if not ok:
        return ToolResult(ok=False, error=f"calibration_invalid: {reason}")

    detections = self._latest_detections()  # snapshot of /perception/detections
    if args.get("class_filter"):
        detections = [d for d in detections if d["class"] == args["class_filter"]]

    # Transform poses from optical frame to base_link
    detections = [self._tf_to_base(d) for d in detections]
    return ToolResult(ok=True, payload={"detections": detections})
```

Capability flag in W3's state injector flips: `capabilities.perception: true`.

### W4.T8 — Tests

- `test_calibration_service.py`: synthetic AX=XB inputs → service produces extrinsics within 1 mm of ground truth; `calibration_date` is a valid ISO 8601 string.
- `test_scene_processor.py`: synthetic point cloud with a known box → detection matches in pose, with depth noise reported.
- `test_safety_guards.py`:
  - calibration 31 days old → reject;
  - reprojection 3.5 mm → reject;
  - depth noise 4 mm at 0.5 m distance → reject (interpolated threshold 3.5 mm);
  - depth noise 4 mm at 0.7 m distance → accept (interpolated threshold ~5 mm).
- `test_query_perception.py`: while mode == MOVING → returns blocked; while IDLE + valid calibration → returns detections.
- `test_qos_match.py`: integration test that the scene_processor callback fires when a publisher uses `SensorDataQoS`. The same test with default reliable QoS demonstrates that callbacks DO NOT fire (this proves we made the right choice).

### W4.T9 — RViz config

`config/perception.rviz`: pre-configured with PointCloud2, Detection3DArray, TF, PlanningScene displays and a sensible viewpoint. Shipped so operators have a one-launch debug view.

### W4.T10 — Bring-up documentation

`README.md` of `gp4_perception`:

- Dependencies (apt + pip).
- Camera bring-up: `ros2 launch gp4_perception camera.launch.py`.
- Calibration: physical setup, then `ros2 launch gp4_perception calibration_collect.launch.py`, then call `/perception/calibrate_hand_eye`.
- Verifying QoS: `ros2 topic info /camera/depth/color/points -v` and confirm subscriber QoS matches.
- Common failure modes: stale calibration, frame mismatch, IR cross-talk with workshop fluorescent lighting.

Plus `docs/operation/MANUAL_REALSENSE_D435I_BRINGUP.md` (the W0 review document referenced this; W4 writes a new fresh version).

---

## Verification

| # | Check | Pass criterion |
|---|---|---|
| 1 | `ros2 launch gp4_perception camera.launch.py` | RealSense node up, `/camera/depth/color/points` and `/camera/color/camera_info` published |
| 2 | `ros2 topic info /camera/depth/color/points -v` | Reliability=BEST_EFFORT, Durability=VOLATILE — confirms `SensorDataQoS` |
| 3 | Scene processor with QoS match | Callback fires; detections published |
| 4 | Scene processor with WRONG QoS (test only) | Callback does NOT fire — confirms our QoS choice was necessary |
| 5 | Calibration service with 24 sample pose pairs | Produces extrinsics; reprojection ≤3 mm; YAML written with valid `calibration_date` |
| 6 | `cat config/extrinsics.yaml` | `calibration_date` is a real ISO 8601 timestamp, not `<NOT_CALIBRATED>` after running calibration |
| 7 | `python tools/validate_safety_chain.py` | Exit 0 with calibration valid; exit non-zero with stale or missing calibration |
| 8 | `query_perception` while robot mode == MOVING | Returns `perception_blocked_during_motion` |
| 9 | `query_perception` while IDLE | Returns detections in `base_link` frame |
| 10 | Drop a known-size box on the table | Appears in `/perception/detections` and in the MoveIt PlanningScene within 1 s |
| 11 | Place box at FOV edge with depth noise > range threshold | Detection rejected with `depth_noise_exceeded` |
| 12 | `colcon test --packages-select gp4_perception` | All tests green |
| 13 | LLM prompt "what objects do you see" → ReAct → query_perception | Returns valid pose list (with gripper absent: ReAct cannot proceed to pick, but query is fine) |
|14 | ros2 interface show interfaces/srv/CalibrateHandEye  |Returns spec; matches W4.T0 Task 1
|15 |ros2 interface show interfaces/srv/GetObjectPositions  |Returns spec
|16 |ros2 interface show interfaces/srv/CheckCamera |Returns spec
|17 |ros2 interface show interfaces/msg/PerceptionStatus  |Returns spec
|18 |gp4_perception services |Use the typed interfaces from W4.T0; no ad-hoc dict / json shapes
---

## DON'T

- Do not mount the camera on the flange. This wave is eye-to-hand only. Eye-in-hand is a separate wave.
- Do not use camera intrinsics from a generic D435i calibration; intrinsics come from the device's factory data via `realsense2_camera`.
- Do not use perception detections as **hard** safety limits. They are advisory inputs to the planner. The safety gate (W1) is authoritative.
- Do not publish raw point cloud at 30 Hz on a Reliable QoS topic. `SensorDataQoS` for high-rate streams.
- Do not hardcode `calibration_date` anywhere — not in templates, not in fixtures, not in tests. Tests use `datetime.utcnow()` or a clock fixture.
- Do not skip the depth-noise check "for testing". The whole point of this wave's safety contract is range-aware noise rejection.
- Do not bundle eye-in-hand support, ML object recognition, or grasp planning in this wave. Each is a separate wave.
- Do not copy code blocks from `36520035`. Use the W0 review as a design guide, write fresh.
- Do not configure mypy --strict on this package on day one. Add to mypy gradually after the package stabilises (move to mypy-strict in a follow-up).

---

## Output artefacts

- `src/gp4_perception/` — entire new package
- `src/llm_gateway/llm_gateway/react/tools/query_perception.py` — diff: replace stub with delegation to `gp4_perception.query_perception_tool`
- `src/llm_gateway/llm_gateway/react/state_injector.py` — diff: `capabilities.perception` reflects whether the package is loaded
- `tools/validate_safety_chain.py` — diff: load extrinsics, run guards
- `src/safety/config/safety_rules.yaml` — diff: `safety.calibration.*` SSOT keys
- `docs/operation/MANUAL_REALSENSE_D435I_BRINGUP.md` — new
- `MIGRATION-W4.md`

---

## Rollback procedure

```bash
# Quickest: disable via SSOT
# Edit safety_rules.yaml: perception.enabled: false
# query_perception tool reverts to the W3 stub error.
# scene_processor and calibration_service stop being launched.

# Full revert
git revert -m 1 <W4 merge commit>
# Note: the perception package directory is preserved; only the wiring is undone.
# Allows W4 retry without re-doing the package skeleton.
```

---

## Risk notes

- **Calibration drift**: physical bumps, temperature swings, table moves all invalidate calibration silently. The 30-day freshness check is a coarse proxy. Operators should re-calibrate after any physical change.
- **OpenCV version**: `cv2.calibrateHandEye` requires OpenCV 4.1+. Older systems will fail at import. The W4 README must document the version requirement and provide a check command.
- **Frame conventions**: OpenCV image frames vs ROS robot frames are the #1 calibration debugging trap. Document the conversion in the calibration service docstring with explicit math.
- **D435i IR cross-talk**: workshop fluorescent lighting can interfere with the IR projector. The launch file exposes `emitter_enabled` for operator override. Document the symptom (random missing depth pixels) in the README.
- **PlanningScene update rate**: pushing collision objects too often (e.g. every camera frame) thrashes MoveIt. Detection TTL = 2 s combined with a re-detection rate of 5 Hz keeps load bounded. If MoveIt becomes unresponsive, lower the rate.
- **Hardware test required for full acceptance**: W4 cannot be sim-only. Operator runs the bring-up sequence with a physical D435i + fiducial board.

---

## Stop signal

End of W4. Do not proceed to W5 until:

- W4 PR merged.
- A real D435i calibration run produces valid extrinsics (PR includes the calibration log + reprojection error).
- Operator has run `query_perception` end-to-end with a real box on the table and confirms detection.
- Stale calibration test demonstrably blocks motion (PR includes the rejection log).

State explicitly: `End of W4. Awaiting review before W5.`

---

**Reliability tag:** `[NEEDS-VALIDATION]` — perception inherently depends on hardware. Sim tests cover the calibration solver, scene processor algorithms, and safety guards, but the hardware test is the real acceptance gate. The W0 review of `36520035` is `[VERIFIED]` reference material; whether each piece of that prior code was correct is what the review document examined.
