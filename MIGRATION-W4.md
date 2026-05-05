# MIGRATION-W4 — Perception Fresh Build

**Branch:** ws-deep-rebuild-3526
**Date:** 2026-05-04

## What changed

### New package: `gp4_perception`
- Full perception stack: camera launch, hand-eye calibration + TF, scene processing, query_perception tool.
- Implements typed ROS interfaces from `interfaces` package (W4.T0).

### New interfaces (in `src/interfaces/`)
- `srv/CalibrateHandEye.srv`
- `srv/GetObjectPositions.srv`
- `srv/CheckCamera.srv`
- `msg/PerceptionStatus.msg`
- `vision_msgs` added as dependency to interfaces package.

### Modified files
| File | Change |
|------|--------|
| `src/interfaces/CMakeLists.txt` | Added 4 perception interface files + vision_msgs dep |
| `src/interfaces/package.xml` | Added `<depend>vision_msgs</depend>` |
| `src/safety/config/safety_rules.yaml` | Added `calibration.max_age_days` and `calibration.max_reprojection_error_mm` |
| `tools/validate_safety_chain.py` | Added checks 4 (calibration SSOT) and 5 (extrinsics YAML validity) |
| `src/llm_gateway/llm_gateway/react/tools/query_perception.py` | Replaced W3 stub with delegation to `gp4_perception.query_perception_tool` |
| `docs/hmi/HMI_ROS_INTERFACES.md` | Added 4 perception interfaces (LOW sensitivity) |

### New files (in `src/gp4_perception/`)
- `gp4_perception/calibration.py` — merged: CalibrationService + TFPublisher
- `gp4_perception/scene_processor.py` — with camera health monitoring
- `gp4_perception/query_perception_tool.py` — fills W3 stub
- `gp4_perception/safety_guards.py` — contains EXTRINSICS_SCHEMA dict
- `config/d435i.yaml`, `config/extrinsics.yaml`, `config/perception.yaml`, `config/fiducials.yaml`
- `launch/camera.launch.py`, `launch/calibration_collect.launch.py`, `launch/perception_full.launch.py`
- `config/perception.rviz`
- `test/test_calibration.py`, `test/test_scene_processor.py`, `test/test_safety_guards.py`, `test/test_query_perception.py`, `test/test_qos_match.py`

### Removed files (consolidation)
- `gp4_perception/calibration_service.py` → merged into `calibration.py`
- `gp4_perception/tf_publisher.py` → merged into `calibration.py`
- `gp4_perception/camera_launcher.py` → removed (launch file wraps realsense directly)
- `gp4_perception/camera_check_service.py` → removed (CheckCamera service deferred)
- `gp4_perception/object_query_service.py` → removed (GetObjectPositions service deferred)
- `gp4_perception/status_publisher.py` → removed (PerceptionStatus topic deferred)
- `config/extrinsics_schema.yaml` → schema now in `safety_guards.py` EXTRINSICS_SCHEMA dict

### New docs
- `docs/operation/MANUAL_REALSENSE_D435I_BRINGUP.md`

## Breaking changes

None. All new interfaces are additive. No existing ROS surfaces were modified.
The `validate_safety_chain.py` script will now report errors for missing calibration SSOT keys and uncalibrated extrinsics — this is intentional and expected before the first calibration run.

## HMI impact

LOW. Four new ROS surfaces added with LOW change sensitivity. HMI does not yet consume them. Future HMI features may surface calibration status via `/perception/status`.

## Rollback

```bash
# Quickest: disable perception in SSOT
# Edit safety_rules.yaml: add perception.enabled: false (not yet implemented as feature flag)
# Or: git revert -m 1 <W4 merge commit>
```

## Verification commands

```bash
colcon build --packages-select interfaces gp4_perception
colcon test --packages-select gp4_perception
python3 tools/validate_safety_chain.py
ros2 interface show interfaces/srv/CalibrateHandEye
ros2 interface show interfaces/srv/GetObjectPositions
ros2 interface show interfaces/srv/CheckCamera
ros2 interface show interfaces/msg/PerceptionStatus
```
