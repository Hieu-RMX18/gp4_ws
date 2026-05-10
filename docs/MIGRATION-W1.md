# MIGRATION-W1 — Kill Silent Fallback + Joint Guard

**Wave:** W1
**Branch:** ws-deep-rebuild-3526
**Date:** 2026-05-04

## What changed

### T0 — Delete `park_safe`, reconcile named states
- Removed `park_safe` group_state from `motoman_gp4.srdf`
- Updated `seed_manager.cpp` fallback seed from park_safe values → `ready` pose values
- Removed `"park"` from HMI supervisor quick-action list
- Updated comments in `seed_manager.hpp`/`.cpp` removing park_safe references
- **J5 limit widened to ±1.603 rad** (from ±1.571) per operator decision to accommodate `home` pose (J5=-1.602)

### T1 — Kill silent LIN fallback
- `primitive_router_dispatch.cpp`: replaced `computeCartesianPath` fallback on Pilz LIN failure with hard `RCLCPP_ERROR` + failure return
- Non-Pilz LIN path now also returns hard failure instead of fallback
- Legitimate `computeCartesianPath` call in `CARTESIAN_PATH` primitive handler is untouched

### T2 — Operational joint limits YAML
- Added `operational_joint_limits`, `joint_position_guard`, `manipulability_guard`, `cumulative_rotation_guard` sections to `safety_rules.yaml`

### T3 — JointPositionGuard class
- New: `joint_position_guard.hpp` / `.cpp` in `motion_core`
- 8 unit tests in `test_joint_position_guard.cpp`
- Iterates trajectory points, rejects if any joint position is outside configured limits

### T4 — Three-stage guard wiring
- **Stage A** (pre-downsample): `PrimitiveRouterDispatch` checks raw planner output
- **Stage B** (QualityGate): `QualityGate::validate_plan` checks post-downsample trajectory
- **Stage C** (dispatch boundary): `TrajectoryExecutor` in `hw_adapter` checks before `async_send_goal`
- `hw_adapter` now depends on `motion_core` for `JointPositionGuard`
- `motion_core_node` loads limits from YAML via `safety_rules_yaml_path` ROS parameter

### T5 — validate_safety_chain.py
- New: `tools/validate_safety_chain.py`
- Checks: operational limits ⊆ hardware limits, motoros2 joints covered, J5 at ±1.603

### T6 — no-fallback-guard CI
- New: `tools/lint/no_silent_motion_fallback.sh`
- CI jobs `safety-chain` and `no-fallback-guard` now invoke real scripts (replaced stubs)

### T7 — ManipulabilityGuard
- New: `manipulability_guard.hpp` / `.cpp` in `motion_core`
- Yoshikawa index via MoveIt Jacobian, floor=0.05, sample every 5th point
- Wired into `QualityGate::validate_plan`
- Graceful no-op when robot model not available

### T8 — CumulativeRotationGuard
- Extended `WristFlipGuard` with `check_cumulative_rotation` method
- Per-joint cumulative |delta| check with configurable max_rad from YAML
- 4 unit tests added to `test_wrist_flip_guard.cpp`

## How to roll back

```bash
git revert <W1-commit-sha>
```

Or cherry-pick specific reversions:
- T0: Restore park_safe in SRDF, revert seed_manager, re-add "park" to HMI
- T1: Restore the computeCartesianPath fallback block in primitive_router_dispatch.cpp
- T2: Remove new YAML sections from safety_rules.yaml
- T3/T4: Remove JointPositionGuard files, revert QualityGate/PrimitiveRouterDispatch/TrajectoryExecutor
- T5/T6: Remove tools/, revert ci.yml to stubs
- T7: Remove ManipulabilityGuard files, revert QualityGate
- T8: Revert WristFlipGuard changes

## Build verification

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motion_core hw_adapter
colcon test --packages-select motion_core
python3 tools/validate_safety_chain.py
bash tools/lint/no_silent_motion_fallback.sh
```

## Safety notes

- J5 operational limit: ±1.603 rad (widened from ±1.571 per operator 2026-05-04)
- All operational limits are strict subsets of hardware limits in joint_limits.yaml
- JointPositionGuard is defence-in-depth at 3 pipeline stages
- ManipulabilityGuard requires robot model at runtime; no-op without it
- CumulativeRotationGuard uses J6 unwrap for continuous rotation joints
