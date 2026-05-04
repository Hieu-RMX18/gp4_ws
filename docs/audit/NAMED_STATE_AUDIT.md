# Named-State Audit — W0.T8

**Date:** 2026-05-04
**Purpose:** Identify all named poses that conflict with W1 operational joint limits.
**Operational limits (W1):** J4 ±2.443 rad, J5 ±1.603 rad (~±91.8°, widened from ±1.571 per operator 2026-05-04), J6 ±3.142 rad.

## Discovery evidence

```
$ rg -n -e 'group_state name=' src/gp4_moveit_config/
src/gp4_moveit_config/config/motoman_gp4.srdf:17:    <group_state name="home" group="gp4_arm">
src/gp4_moveit_config/config/motoman_gp4.srdf:26:    <group_state name="park_safe" group="gp4_arm">
src/gp4_moveit_config/config/motoman_gp4.srdf:35:    <group_state name="ready" group="gp4_arm">
src/gp4_moveit_config/config/motoman_gp4.srdf:44:    <group_state name="poseA" group="gp4_arm">
src/gp4_moveit_config/config/motoman_gp4.srdf:53:    <group_state name="poseB" group="gp4_arm">

$ rg -n -e 'park_safe|park-safe|PARK_SAFE' --type py --type cpp --type yaml
src/motion_core/src/seed_manager.cpp:65:  // V4 D2 Priority 3: commissioning named-target fallback (park_safe, ready)
src/motion_core/src/seed_manager.cpp:225:  // group_state "park_safe" so IK fallback starts from the same conservative
src/motion_core/include/motion_core/seed_manager.hpp:23:  /// 3. commissioning named-target fallback (park_safe/ready-style seed)

$ rg -n -e 'park_safe|park-safe|"park"' hmi/
hmi/backend/services/supervisor_sequence.py:58:    "park",
```

## Named-state audit table

| Pose name | Defined in | J1 (s) | J2 (l) | J3 (u) | J4 (r) | J5 (b) | J6 (t) | Conflicts with W1 limits? | Owner |
|-----------|------------|--------|--------|--------|--------|--------|--------|---------------------------|-------|
| home | SRDF:17 | 1.548 | -0.159 | -0.159 | 0.0 | -1.602 | 0.054 | No (J5 within ±1.603 after operator widening 2026-05-04) | active |
| park_safe | SRDF:26 | 0.0 | 0.55 | -0.2 | 0.0 | **-2.0** | 0.0 | **YES**: J5 = -2.0 far outside ±1.603 | deleted (W1.T0) |
| ready | SRDF:35 | 1.938 | 0.090 | -0.159 | 0.0 | -1.175 | 0.053 | No | active |
| poseA | SRDF:44 | 2.072 | 0.371 | -0.177 | ~0.0 | -1.283 | 0.054 | No | active |
| poseB | SRDF:53 | 1.112 | 0.398 | -0.263 | -0.123 | -1.069 | -0.138 | No | active |

## Additional references to park_safe

| Location | Type | Action |
|----------|------|--------|
| `src/motion_core/src/seed_manager.cpp:65` | Comment only | W1 updates comment |
| `src/motion_core/src/seed_manager.cpp:225` | Comment only | W1 updates comment |
| `src/motion_core/include/motion_core/seed_manager.hpp:23` | Comment only | W1 updates comment |
| `hmi/backend/services/supervisor_sequence.py:58` | String "park" in list | W1 removes entry |

## Conflict resolution notes

**park_safe:** Per U1, the user has chosen `delete`. W1.T0 removes the SRDF
`<group_state>` block, all Python/C++ references, and HMI quick-action entries.

**home:** J5 = -1.602 rad. Operator widened J5 operational limit to ±1.603 rad
on 2026-05-04 (Option C resolved). Home pose is now within limits with 0.001 rad margin.

## Stop signal

W0 PR includes this file. Human reviews each row before merging W0.
