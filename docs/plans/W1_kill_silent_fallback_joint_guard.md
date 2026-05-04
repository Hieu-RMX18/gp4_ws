# W1 — Kill Silent Fallback, Add JointPositionGuard

**Wave class:** Safety hardening
**Risk:** Medium (changes motion behaviour; sim required, hardware deferred)
**Estimated effort:** 3–4 working days
**Depends on:** W0 (governance, branch, CI stubs in place)
**Unblocks:** W2 (drawing rewire is masked by silent fallback)

---

## Goal

Two changes, both targeting the gap between "what the safety system claims to enforce" and "what it actually catches":

1. Remove the silent `computeCartesianPath` fallback from `primitive_router_dispatch.cpp`. When Pilz LIN fails, the request fails — loudly, with primitive name, goal sequence, and the underlying error. No fallback. No second attempt. No surprise wrist-flip-friendly path replacing a wrist-flip-rejecting one.

2. Introduce `JointPositionGuard` as a brand-new check inside `QualityGate`. Iterate every trajectory point. Reject if any joint position exceeds the operational limit declared in YAML. This is in addition to (not instead of) `WristFlipGuard`, which keeps its delta and sign-flip checks.

Both changes increase the rejection rate on bad plans. That is intentional. The current system silently produces unsafe trajectories; W1 makes the unsafe cases visible so they can be fixed in W2.

---

## Why this comes before W2

W2 rewires drawing to emit `BLENDED_SEQUENCE` instead of `CARTESIAN_PATH`. But while the LIN silent fallback is still active, every Pilz failure cleanly substitutes a `computeCartesianPath` plan that bypasses the wrist guard's intent. A drawing test would appear to succeed even when the new code path was bypassed. W2 verification depends on W1 closing this trap.

---

## Discovery (paste raw output)

```bash
# A. Confirm the silent fallback exists exactly where SUMMARY says
rg -n "computeCartesianPath fallback|attempting computeCartesianPath" src/motion_core/

# B. Confirm full context around lines 857–895 of primitive_router_dispatch.cpp
sed -n '840,910p' src/motion_core/src/primitive_router_dispatch.cpp

# C. Confirm WristFlipGuard scope (delta-only, no absolute position)
sed -n '1,50p' src/motion_core/include/motion_core/wrist_flip_guard.hpp
sed -n '1,60p' src/motion_core/src/wrist_flip_guard.cpp

# D. Find the QualityGate constructor and validate_trajectory call site
rg -n "class QualityGate|validate_trajectory" src/motion_core/

# E. Find every existing reference to operational/joint limits in YAML
rg -n "joint_limits|joint_limits_override|motion_limits|operational" src/safety/config/ src/gp4_moveit_config/config/

# F. Find policy_loader and confirm the _FAILSAFE_MOTION_LIMITS pattern
rg -n "_FAILSAFE_MOTION_LIMITS|FAILSAFE_MOTION" src/safety/

# G. Existing safety tests for WristFlipGuard
ls src/motion_core/test/ src/safety/tests/

# H. Check whether anyone else calls computeCartesianPath in motion_core (for the regression guard)
rg -n "computeCartesianPath" src/motion_core/ src/primitives/

# I. URDF/SRDF location for cross-validation script
find src/gp4_moveit_config/ -name "*.urdf*" -o -name "*.srdf*" -o -name "*.xacro" | head
```

If H returns hits in `primitives/src/primitive_lin.cpp:167` and `primitive_router_dispatch.cpp:716` (the legitimate CARTESIAN_PATH primitive path), that is expected. The silent fallback sits at lines 857–895 in `primitive_router_dispatch.cpp` and at the LIN primitive's failure handler. Both go away in W1.

---

## Tasks
### W1.T0 — Delete `park_safe`, reconcile named states (precondition for J5 ±90° enforcement)

**Why:** W0.T8 produced `docs/audit/NAMED_STATE_AUDIT.md`. W1 will activate JointPositionGuard with J5 ±90°. Any pose that conflicts must be removed BEFORE the guard becomes active, otherwise the very first move-to-named-pose call will fail-closed by the new guard.

**Per U1, `park_safe` is `delete`. No widening of J5 limits.** This task makes the deletion executable.

**Tasks:**

1. **For each row in `NAMED_STATE_AUDIT.md` with status=`delete`** (specifically `park_safe` and any other entries the human approved for deletion):
   - SRDF / MoveIt: remove the `<group_state name="park_safe" ... >` block from `src/gp4_moveit_config/config/<arm>.srdf` (path confirmed by W0 audit grep). Surgical removal — do not touch other group_states.
   - Python: remove any `park_safe` constants, dictionary keys, enum members, fallback references. Specific patterns to search and remove:
```bash
     rg -n -e 'park_safe|PARK_SAFE' --type py
```
     Each hit either gets the symbol removed (if dead after deletion) or replaced with a documented alternative reference if still useful.
   - C++: same procedure for `.cpp` / `.hpp`.
   - YAML / config: remove `park_safe` keys from named-pose YAMLs.
   - HMI: remove `park_safe` from HMI quick-action lists (`hmi/frontend/src/.../*.tsx|.ts|.json`) and from any HMI backend fixtures or test data that hard-code the symbol.

2. **For each row with status=`relocate`** (if any):
   - Update the SRDF group_state with new joint values that satisfy `operational_joint_limits` from W1.T2.
   - Document the old → new mapping in `MIGRATION-W1.md` with rationale.

3. **For each row with status=`escalate`**:
   - W1 stops. Human decides. Do NOT proceed without resolution. Update audit doc.

4. **Verify deletion is complete:**
```bash
   rg -n -e 'park_safe|PARK_SAFE' src/ hmi/ docs/
   # expected: 0 hits, OR only hits inside docs/audit/NAMED_STATE_AUDIT.md (audit log itself, allowed)
```

5. **Update tests.** Tests that referenced `park_safe` (search: `rg -n 'park_safe' --type py --type cpp` inside `*test*` paths) must be updated to use a still-valid named state, or removed if they only exercised `park_safe` itself.

**No-conflict guarantee:** because W0.T8 was a dry audit, and W1.T0 deletes only what the audit marked `delete`, this task does NOT delete anything the audit failed to capture. Any post-deletion `rg park_safe` hit indicates an incomplete audit — re-run W0.T8 before proceeding.

**Stop signal:** `rg -e 'park_safe|PARK_SAFE'` returns 0 productive hits. `colcon test` still green (because tests have been updated). Then proceed to W1.T1.

### W1.T1 — Replace the silent LIN fallback with a hard failure

File: `src/motion_core/src/primitive_router_dispatch.cpp`
Lines: approximately 857–895 (the block containing `"attempting computeCartesianPath fallback"`)

Surgical change. The existing structure attempts Pilz LIN, then on failure re-plans with `computeCartesianPath`. The new behaviour: on Pilz LIN failure, return immediately with `result.success = false` and a `result.reason` that names the primitive, goal sequence, planner ID, and the Pilz error string.

Pseudo-diff:

```diff
   if (lin_success) {
     // existing happy path unchanged
     ...
   } else {
-    RCLCPP_WARN(logger_,
-      "Pilz LIN planning failed for goal_seq=%lu (%s), "
-      "attempting computeCartesianPath fallback.",
-      goal_seq, pilz_error.c_str());
-    cartesian_fraction = move_group->computeCartesianPath(
-      waypoints, ..., trajectory);
-    if (cartesian_fraction < kMinCartesianFraction) {
-      result.reason = "both Pilz LIN and computeCartesianPath failed for LIN primitive";
-      ...
-    }
-    // ... rest of fallback path
+    RCLCPP_ERROR(logger_,
+      "Pilz LIN planning failed for goal_seq=%lu primitive=LIN planner=%s: %s. "
+      "No fallback. Caller must replan or retry with PTP.",
+      goal_seq, planner_id.c_str(), pilz_error.c_str());
+    result.success = false;
+    result.reason = std::string("Pilz LIN failed (no fallback): ") + pilz_error;
+    return result;
   }
```

Same treatment for the second fallback site near line 887 (the standalone `computeCartesianPath` call after a different LIN failure).

**Do not touch** the legitimate CARTESIAN_PATH primitive path (line 716). That is a primitive in its own right (`primitive_type == "CARTESIAN_PATH"`), used by callers who explicitly request multi-waypoint smooth motion. W1 does not remove it. W2 deprecates the use of CARTESIAN_PATH **for drawing** — that is W2's scope, not W1's.

### W1.T2 — Add `operational_joint_limits` to the safety YAML

File: `src/safety/config/safety_rules.yaml` (confirmed location).

Add a new top-level section. The agent must NOT remove or rename existing keys (`joint_limits_override`, `motion_limits`, etc.). Adding only.

```yaml
Add a comment block above the operational_joint_limits YAML block:
# operational_joint_limits — per U1 of PATCH-v3.1, J5 is HARD ±90°.
# Do NOT widen this limit. If a use case requires J5 outside ±90°,
# raise it as a new safety review, NOT as a unilateral edit here.
# park_safe and any other named state requiring J5 < -1.571 rad
# was deleted in W1.T0 per the W0.T8 audit.
operational_joint_limits:
  ...
  # Soft limits enforced by JointPositionGuard at the trajectory point level.
  # MUST be a strict subset of the URDF/SRDF hardware limits in
  # src/gp4_moveit_config/config/joint_limits.yaml.
  # Hardware limits stay untouched; this is an additional, tighter envelope.
  joint_1_s: {min: -2.967, max:  2.967}   # +/-170 deg (= hardware S)
  joint_2_l: {min: -1.920, max:  2.269}   # = hardware L (asymmetric)
  joint_3_u: {min: -1.134, max:  3.491}   # = hardware U (asymmetric)
  joint_4_r: {min: -2.443, max:  2.443}   # +/-140 deg (hardware R is +/-200 deg, derated)
  joint_5_b: {min: -1.571, max:  1.571}   # +/-90 deg (hardware B is +/-123 deg, derated; HARD per U1)
  joint_6_t: {min: -3.142, max:  3.142}   # +/-180 deg (hardware T is +/-455 deg, derated; W7 may opt-in extended)

joint_position_guard:
  enabled: true
  reject_message_template: "joint_position_guard reject at point[{idx}]: {joint} = {value:.4f} rad outside [{min:.4f}, {max:.4f}]"
```

Rationale for the derated J4/J5/J6 numbers: per the GP4 datasheet, hardware allows wider motion, but for cable spool protection and to avoid the wrist-flip-prone region, the operational envelope is tighter. The numbers match what reviewer 1 and reviewer 2 of the original plan independently agreed on. They can be widened later via PR with safety review. T-axis tiered mode is W7.

### W1.T3 — Add the C++ `JointPositionGuard` class

New files (only new files allowed in this wave):

- `src/motion_core/include/motion_core/joint_position_guard.hpp`
- `src/motion_core/src/joint_position_guard.cpp`
- `src/motion_core/test/test_joint_position_guard.cpp`

Header API (small, single-purpose, mirrors `WristFlipGuard` style):

```cpp
#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace motion_core
{

struct JointLimit { double min; double max; };

class JointPositionGuard
{
public:
  JointPositionGuard() = default;
  // Construct with a map: joint_name -> {min, max} loaded from YAML.
  explicit JointPositionGuard(std::unordered_map<std::string, JointLimit> limits);

  // Returns false if any point violates a limit.
  // On rejection, fills `reason` with a message naming the joint, point index,
  // value, and limit. Returns true if no limit applies (no joint match) or all pass.
  bool check_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::string & reason) const;

  // Inspection helpers used by tests.
  bool has_limit(const std::string & joint_name) const;
  JointLimit get_limit(const std::string & joint_name) const;

private:
  std::unordered_map<std::string, JointLimit> limits_;
};

}  // namespace motion_core
```

Implementation: iterate `traj.joint_names` and `traj.points`. For each `points[i]`, for each joint index `j`, look up `limits_[joint_names[j]]`; if not found, skip (do not crash on extra joints — base/station axes if present). If `points[i].positions[j]` outside `[min, max]`, fill `reason` and return false.

Tests in `test_joint_position_guard.cpp`:

- Pass: empty trajectory.
- Pass: trajectory all within limits.
- Pass: joint not in the limits map (e.g. station axis) — guard reports "no applicable limits", returns true.
- Fail: `joint_4_r = +2.444` rad at the last point.
- Fail: `joint_5_b = -1.572` rad at point [3].
- Fail: `joint_6_t = +5.0` rad (default mode rejects beyond ±π).
- Reason message format matches the YAML template.

Hook into `CMakeLists.txt` and `package.xml` per R7.

### W1.T4 — Wire `JointPositionGuard` into the safety chain at THREE stages (multi-stage)

**Why (per F2):** the existing pipeline downsamples trajectories before QualityGate. A single hook inside QualityGate may not see intermediate joint excursions that downsampling has elided. We place the guard at three stages so a violation at any planning depth is caught.

**Stage A — Pre-downsample (planner output, raw waypoints).**
- Hook: immediately after the planner returns a `MoveItErrorCode::SUCCESS`, BEFORE any time-parameterization or waypoint reduction.
- Code path: in `motion_core` (path confirmed by W1.T1 discovery), find the function that receives the planner's `RobotTrajectory` from MoveIt. Insert a guard call at the top of that function:
```cpp
  std::string reason;
  if (!joint_position_guard_.check_trajectory(trajectory.joint_trajectory, reason)) {
    RCLCPP_ERROR(get_logger(),
      "JointPositionGuard rejected raw planner output: %s", reason.c_str());
    return reject(reason);
  }
```
- Test: a hand-built `RobotTrajectory` with one waypoint at `joint_5_b = 1.6 rad` (just above ±π/2) is rejected at this stage with the offending point index in the reason.

**Stage B — Inside QualityGate::validate_trajectory (post-downsample).**
- This is the hook described in the original W1.T4. Keep it as-is per the existing wave file.

**Stage C — hw_adapter dispatch boundary (last line of defense).**
- Hook: in `src/hw_adapter/src/trajectory_executor.cpp` (path confirmed by W1.T1 discovery as a 624-line file in the verified package list), inside the function that dispatches a `JointTrajectory` to the controller's action client.
- Insert one final guard call right before `client_->async_send_goal(goal)`. Same API as Stage A.
- Rationale: even if Stages A/B accept, a downstream transformation (e.g. controller-side blending) could drift; Stage C catches drift in the hw_adapter's outgoing message.
- Code:
```cpp
  std::string reason;
  if (!joint_position_guard_.check_trajectory(goal.trajectory, reason)) {
    RCLCPP_FATAL(get_logger(),
      "JointPositionGuard rejected at hw_adapter dispatch: %s", reason.c_str());
    // Do not send goal. Return failure to caller.
    return TrajectoryDispatchResult::reject(reason);
  }
```

**Shared instance:** all three stages use the same `JointPositionGuard` instance (constructed once from YAML at node start). The instance is thread-safe (read-only after construction) — no locking needed.

**Tests** (additive to the existing W1.T4 test list):

- A trajectory built with one violating point at index k passes before Stage A but fails at Stage A — verifies pre-downsample catch.
- A trajectory whose violating point at index k is removed by downsampling fails at Stage B — verifies post-downsample catch.
- A trajectory crafted so the hw_adapter mutates joint values to violate (mocked) fails at Stage C — verifies dispatch boundary catch.

**No-conflict guarantee:** Stage B is the original wave's hook (unchanged behaviour). Stages A and C are additive — same API, same YAML config, same rejection contract. No existing test changes its expected behaviour. Acceptance is purely *broader*: more violations are caught earlier.

### W1.T5 — `tools/validate_safety_chain.py` (replaces the W0 stub)

File: `tools/validate_safety_chain.py`. Executable.

Behaviour:

1. Load `src/safety/config/safety_rules.yaml`.
2. Load `src/gp4_moveit_config/config/joint_limits.yaml`.
3. Optionally load `src/motion_core/config/motion_core_safety.yaml` (if W1.T4 chose option B).
4. Assert: every joint in `operational_joint_limits` has `min ≥ urdf_min` and `max ≤ urdf_max`.
5. Assert: if option B was used, the mirrored YAML matches `safety_rules.yaml` to numerical equality.
6. Assert: every joint listed in `motoros2_config.yaml`'s `joint_names` (verified: `joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t`) has an entry in `operational_joint_limits`.
7. Print a summary table on success. Exit non-zero with a diff on any mismatch.

CI job `safety-chain` (W0 stub) now invokes this script for real.

### W1.T6 — `no-fallback-guard` CI job (replaces the W0 stub)

File: `tools/lint/no_silent_motion_fallback.sh` (new).

```bash
#!/usr/bin/env bash
set -euo pipefail
# Forbid the phrase that previously marked the silent fallback.
# Allows the legitimate CARTESIAN_PATH primitive, which is a separate code
# path explicitly requested by the caller.
if rg -n "attempting computeCartesianPath" src/motion_core/ src/primitives/ ; then
  echo "no_silent_motion_fallback: forbidden silent fallback detected. See W1." >&2
  exit 1
fi
# Forbid raw computeCartesianPath calls inside the LIN-handling code path of
# primitive_router_dispatch.cpp. CARTESIAN_PATH primitive code is allowed.
if rg -n "computeCartesianPath" src/motion_core/src/primitive_router_dispatch.cpp \
    | rg -v "primitive == \"CARTESIAN_PATH\"" ; then
  echo "no_silent_motion_fallback: computeCartesianPath used outside CARTESIAN_PATH primitive scope" >&2
  exit 1
fi
echo "no_silent_motion_fallback: OK"
```
### W1.T7 — ManipulabilityGuard (per F4)

**Why:** the existing `WristFlipGuard` checks per-step deltas only. Servo singularity thresholds (`servo_gp4_jog.yaml: lower_singularity_threshold=17.0, hard_stop_singularity_threshold=30.0`) apply only to Servo mode jog/teleop, not to MoveIt-planned trajectories. A trajectory that smoothly transitions through J5 ≈ 0° (singular pose) passes WristFlipGuard but produces ill-conditioned Jacobian and unpredictable joint velocities at execution time.

**New file:** `src/motion_core/include/motion_core/manipulability_guard.hpp`
**New file:** `src/motion_core/src/manipulability_guard.cpp`
**New file:** `src/motion_core/test/test_manipulability_guard.cpp`

**API:**

```cpp
namespace motion_core {

class ManipulabilityGuard {
public:
  ManipulabilityGuard(
    moveit::core::RobotModelConstPtr robot_model,
    const std::string & group_name,
    double floor);

  // Sample manipulability at every K-th waypoint (default K from YAML).
  // For each sample, compute Yoshikawa index:
  //   w = sqrt(det(J * J^T))
  // where J is the geometric Jacobian at the EEF link.
  // Reject if any sample has w < floor_.
  bool check_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::string & reason) const;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;
  double floor_;
};

}  // namespace motion_core
```

**Implementation note:** use `moveit::core::RobotState::getJacobian(joint_model_group, ref_link)` then `(J * J.transpose()).determinant()` and `std::sqrt(...)`. This is the standard Yoshikawa index for serial chains.

**SSOT additions** to `src/safety/config/safety_rules.yaml`:

```yaml
manipulability_guard:
  enabled: true
  floor: 0.05                # Yoshikawa index floor; <0.05 is near-singular for GP4 6-DOF wrist
  sample_every_n_points: 5   # check every 5th waypoint to keep cost bounded
  reject_message_template: "manipulability_guard reject at point[{idx}]: w={w:.4f} < floor {floor:.4f}"
```

**Tests:**

- A trajectory that stays away from singularity (e.g. J5 ∈ [0.3, 1.2] rad always): all sampled points have `w > 0.05` → pass.
- A trajectory passing through J5 ≈ 0° (e.g. J5 sweeping -0.05, 0.0, +0.05 rad): at least one sampled point has `w < 0.05` → fail, with point index and `w` value in reason.
- A trajectory with `sample_every_n_points = 5` and a single bad point at index 7 (between samples 5 and 10): document this as a known under-sampling case; recommend `sample_every_n_points: 1` for production. (Cost: ~5x guard time per trajectory; acceptable.)

**Wire into QualityGate** (Stage B of multi-stage from Patch 2.2):

After `joint_position_guard_.check_trajectory(...)` returns true, call `manipulability_guard_.check_trajectory(...)` next. Same return convention. Order: JointPositionGuard first (cheap), ManipulabilityGuard second (Jacobian compute is heavier).

### W1.T8 — CumulativeRotationGuard (per F4 + Cascade C1)

**Why:** WristFlipGuard checks per-step deltas (max 30° per step on wrist axes). A trajectory with 20 steps × 25° each = 500° accumulated rotation on J6 still passes per-step but is physically dangerous (cable spool damage). Need cumulative bound.

**Add to existing** `src/motion_core/include/motion_core/wrist_flip_guard.hpp` and `wrist_flip_guard.cpp` (do NOT create a new class — reviewer Cascade's name suggests a separate class, but per karpathy §3 we extend the existing class to keep the safety surface compact).

**API extension:**

```cpp
class WristFlipGuard {
 public:
  // ... existing API unchanged ...

  // New method.
  bool check_cumulative_rotation(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::string & reason) const;
};
```

**Implementation:** for each wrist axis (J4, J5, J6 — names from `motoros2_config.yaml`), compute `sum |positions[i+1] - positions[i]|` over all consecutive pairs. Reject if cumulative sum exceeds `cumulative_rotation_max[joint_name]`.

**SSOT additions** to `safety_rules.yaml`:

```yaml
cumulative_rotation_guard:
  enabled: true
  max_rad:
    joint_4_r: 6.283   # 360 deg total trajectory rotation
    joint_5_b: 4.189   # 240 deg
    joint_6_t: 9.425   # 540 deg (more permissive — common in wind-on tasks)
```

**Tests** in `test_wrist_flip_guard.cpp`:

- Trajectory with J6 cumulative 6.0 rad (under 9.425): pass.
- Trajectory with J6 cumulative 10.0 rad: fail.
- A purely back-and-forth trajectory on J4 (e.g. -1.0, +1.0, -1.0, +1.0 rad over 4 steps) accumulates 6.0 rad on J4: fail (exceeds 6.283 rad limit due to absolute deltas summing). This is the intended behaviour — repeated back-and-forth IS cumulative wear.

**Wire into QualityGate**: after ManipulabilityGuard, call `wrist_flip_guard_.check_cumulative_rotation(...)`. Existing `wrist_flip_guard_.check_trajectory(...)` (per-step delta) remains the first wrist check.

**Order in QualityGate after Patch 2.2 + 2.3:**

1. JointPositionGuard (operational position envelope)
2. WristFlipGuard::check_trajectory (per-step delta + sign flip)
3. WristFlipGuard::check_cumulative_rotation (NEW, total rotation)
4. ManipulabilityGuard (Jacobian condition)

Each is independent. Each can short-circuit on first failure (already standard in QualityGate).

Wire into `.github/workflows/ci.yml` job `no-fallback-guard`.

---

## Verification

| # | Check | Pass criterion |
|---|---|---|
| 1 | `rg -n "attempting computeCartesianPath" src/` | 0 results |
| 2 | Forced Pilz LIN failure in sim (mock the planner adapter) | Action returns failure with `reason` containing `"Pilz LIN failed (no fallback)"` |
| 3 | Existing LIN happy-path test | Still passes |
| 4 | Trajectory submitted with `joint_4_r[5] = +2.5 rad` | QualityGate rejects with `joint_position_guard reject at point[5]: joint_4_r = 2.5000 rad outside [-2.4430, 2.4430]` |
| 5 | Trajectory all within limits | Passes both WristFlipGuard and JointPositionGuard |
| 6 | `python tools/validate_safety_chain.py` | Exit 0; YAML mirror agreement (if option B) verified |
| 7 | `bash tools/lint/no_silent_motion_fallback.sh` | Exit 0 |
| 8 | `colcon build --symlink-install --packages-select motion_core` | Green |
| 9 | `colcon test --packages-select motion_core safety` | Green; new tests visible |
| 10 | CI job `safety-chain` | Green (real, not stub) |
| 11 | CI job `no-fallback-guard` | Green (real, not stub) |
| 12 | `colcon test` overall | No regressions vs W0 baseline; allowed: tests that depended on the silent fallback now legitimately fail (must be listed in PR) |

For verification 2, the agent provides a small gtest fixture or a Python integration test under `src/motion_core/test/` that injects a Pilz failure (mock or planner ID that always fails). Hardware test is NOT required for W1 merge.

| 13 |  rg -e 'park_safe|PARK_SAFE' src/ hmi/ | 0 productive hits (audit doc may still list the deleted symbol) |
| 14 |  Trajectory with joint_5_b = 1.6 rad at any waypoint |Rejected by JointPositionGuard at one of Stage A/B/C, with point index in reason|
| 15 |  Trajectory passing through manipulability < 0.05 |Rejected by ManipulabilityGuard with sample index and w value |
| 16 |  Trajectory with joint_6_t cumulative 10 rad |Rejected by WristFlipGuard::check_cumulative_rotation|
| 17 | colcon test --packages-select motion_core safety |Green; new tests visible (cumulative, manipulability, multi-stage A/B/C)|
---

## DON'T

- Do not remove the legitimate `CARTESIAN_PATH` primitive code path at `primitive_router_dispatch.cpp:716`. That is W2's domain (and W2 only deprecates its **use by drawing**, not the primitive itself).
- Do not introduce a "configurable fallback" behind a flag. Hard removal. The flag-based approach reintroduces the bug at the next misconfiguration.
- Do not soften the JointPositionGuard rejection message for "operator readability". The detailed format (joint name, point index, exact value, limit) is what makes debugging fast.
- Do not auto-derate hardware joint limits in the URDF/SRDF. The URDF mirrors the datasheet (correct). Derating lives in `safety_rules.yaml`'s new `operational_joint_limits` block.
- Do not couple `JointPositionGuard` construction to a singleton or global. Constructor takes the limits map as input. Keeps testing trivial.
- Do not run hardware tests for W1. The wave is sim-validated only. Hardware confirmation is a separate gate.
- Do not change `kDefaultVelocityScaling`. Per discovery, it is not a bypass; the velocity_scale flow is correctly fail-soft via comment-documented contract. Touch this only if W6 finds it broken under audit.
- Do not widen J5 ±90° per U1. Any J5 widening is a separate safety review.
- Do not place JointPositionGuard at only one stage. F2 mandates A/B/C placement.
- Do not skip ManipulabilityGuard for "performance" reasons. Cost is bounded by `sample_every_n_points`.
- Do not delete a named pose without an entry in `NAMED_STATE_AUDIT.md` confirming it.
---

## Output artefacts

W1 PR contents:

- `src/motion_core/src/primitive_router_dispatch.cpp` — diff removing the silent fallback at lines ~857–895
- `src/motion_core/include/motion_core/joint_position_guard.hpp` — new file
- `src/motion_core/src/joint_position_guard.cpp` — new file
- `src/motion_core/test/test_joint_position_guard.cpp` — new file
- `src/motion_core/include/motion_core/quality_gate.hpp` — diff adding `JointPositionGuard` member
- `src/motion_core/src/quality_gate.cpp` — diff calling the new guard
- `src/motion_core/test/test_quality_gate.cpp` — diff with two new test cases
- `src/motion_core/CMakeLists.txt` — diff registering new sources
- `src/motion_core/package.xml` — diff if `yaml-cpp` is added (option A)
- `src/safety/config/safety_rules.yaml` — diff adding `operational_joint_limits` and `joint_position_guard` sections
- `tools/validate_safety_chain.py` — new file
- `tools/lint/no_silent_motion_fallback.sh` — new file
- `.github/workflows/ci.yml` — diff replacing the stub bodies of `safety-chain` and `no-fallback-guard`
- `MIGRATION-W1.md` — what changed, why, and how to roll back

---

## Rollback procedure

```bash
# Revert the W1 PR
git revert -m 1 <W1 merge commit hash>

# If only the JointPositionGuard piece is problematic but the silent-fallback
# removal is fine, the agent must produce a follow-up PR that disables the new
# guard via `joint_position_guard.enabled: false` in safety_rules.yaml. Do NOT
# rip the C++ class out.

# If the silent-fallback removal causes hardware-blocking problems and W2 has
# not yet landed, the temporary mitigation is to fall back to PTP for affected
# trajectories. Do NOT re-introduce the silent fallback. Operator decides
# manually.
```

---

## Risk notes

- **Hardware regression risk:** trajectories that "worked" by silently falling back to a wrist-flippy `computeCartesianPath` will now fail. This is the intended behaviour; treat each failure as a real bug. The corresponding fix is W2 (rewire drawing to BLENDED_SEQUENCE) or operator-level retry with PTP.
- **Test regression risk:** existing tests that asserted the fallback path will fail. Catalog them in the PR. They are either: (a) updated to expect failure now (W1's responsibility), or (b) updated to expect BLENDED_SEQUENCE (W2's responsibility — mark as `# TODO(W2)` and skip in W1 if needed).
- **YAML drift risk:** if option B is chosen, the dual YAML files can drift. `tools/validate_safety_chain.py` makes this a hard CI failure — use it religiously.
- **Sim ≠ hardware:** all W1 verification is sim. Hardware validation requires the operator + an explicit gate. W1 merge is sim-validated only.

---

## Stop signal

End of W1. Do not proceed to W2 until:

- W1 PR merged and CI green on `ws-deep-rebuild-3526`.
- A sim test demonstrating the new JointPositionGuard reject behaviour is filed and visible in CI logs.
- The two CI stub jobs (`safety-chain`, `no-fallback-guard`) are now real and passing.

State explicitly: `End of W1. Awaiting review before W2.`

---

**Reliability tag:** `[VERIFIED]` for the file:line targets — discovery confirmed `primitive_router_dispatch.cpp` lines 857–895, `wrist_flip_guard.cpp` is delta-only, `safety_rules.yaml` exists at `src/safety/config/`. `[NEEDS-VALIDATION]` for option A vs B in W1.T4 — depends on whether `yaml-cpp` integrates cleanly with the existing CMake (agent must check during implementation). `[NEEDS-VALIDATION]` for the exact list of existing tests that will need `# TODO(W2)` markers — depends on which tests currently exercise the silent fallback.
