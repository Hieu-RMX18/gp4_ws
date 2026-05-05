# MIGRATION-W7.md — T-Axis Tiered Mode

**Wave:** W7
**Branch:** ws-deep-rebuild-3526
**Date:** 2026-05-05
**Status:** COMPLETE
**Next wave:** W8 (scheduled 2026-05-19)

---

## Verification Results

### Build
- `colcon build --symlink-install` (full workspace, 20 packages) — PASS
- `motion_core` rebuilt clean (no warnings after unused-parameter fix)

### Tests
- Python: `pytest src/safety/tests/test_extended_mode.py` — 10/10 passed
- Python: `pytest src/safety/tests/test_command_validator.py` — 16/16 passed (no regressions)
- C++: `test_joint_position_guard` — 13/13 passed (8 existing + 5 new Mode tests)
- C++: `ctest --test-dir build/motion_core` — 11/12 passed (1 pre-existing W6 failure: `QualityGateTest.RejectsPlanWithNonFiniteValues`)

### Lint / Audit
- `tools/validate_safety_chain.py` — FAIL (expected: extrinsics not yet calibrated); tiered shape parsing PASS
- `tools/lint/no_silent_motion_fallback.sh` — PASS
- `tools/lint/aged_deprecation_check.py` — PASS (0 past-due)
- `rg "extended_mode" src/ hmi/ docs/` — hits only in W7-added code ✅

---

## W7.T1 — Tiered joint_6_t in safety_rules.yaml

**Result:** `joint_6_t` converted from flat `{min, max}` to tiered structure.

| Tier | min | max | Usage |
|---|---|---|---|
| `default` | -3.142 | 3.142 | ±180° — all commands unless opted in |
| `extended` | -7.941 | 7.941 | ±455° — hardware max; opt-in only |

`extended_preconditions` block added:
- `cable_inspection_signed_off: true`
- `max_velocity_scale: 0.10`
- `requires_operator_confirm: true`
- `max_continuous_extended_time_s: 30`
- `cool_down_s_between_runs: 60`

**File:** `src/safety/config/safety_rules.yaml`

---

## W7.T2 — JointPositionGuard Mode API

**Result:** C++ `JointPositionGuard` now supports tiered limits.

| Addition | Location |
|---|---|
| `enum class Mode { Default, Extended }` | `joint_position_guard.hpp` |
| `struct TieredLimit { JointLimit default_limit; std::optional<JointLimit> extended_limit; }` | `joint_position_guard.hpp` |
| Constructor from `std::unordered_map<std::string, TieredLimit>` | `joint_position_guard.cpp` |
| Legacy constructor auto-wraps flat limits into `TieredLimit{lim, std::nullopt}` | `joint_position_guard.cpp` |
| `check_trajectory(..., Mode mode = Mode::Default)` selects active limit tier | `joint_position_guard.cpp` |

Backward compatibility: flat-configured joints (joint_1_s through joint_5_b) work unchanged.

---

## W7.T3 — Precondition Gate in CommandValidator

**Result:** Python `CommandValidator` now enforces 5 preconditions before any extended-mode command is accepted.

| # | Precondition | Fail message prefix |
|---|---|---|
| 1 | `cable_inspection_signed_off_token` present | "requires cable_inspection_signed_off_token" |
| 2 | `velocity_scale` ≤ 0.10 | "velocity_scale=X > cap 0.1" |
| 3 | `operator_confirm_token` present | "requires operator_confirm_token" |
| 4 | Cooldown ≥ 60s since last extended run | "cooldown not elapsed" |
| 5 | `estimated_duration_s` ≤ 30s | "estimated_duration Xs > cap 30s" |

`_ExtendedRunTracker` (module-level singleton) records run end times for cooldown enforcement.
`record_extended_run_end()` exposed for `execution_gate.py` to call after command completion.

**File:** `src/safety/safety/command_validator.py`

---

## W7.T4 — Mode Plumbing (ExecuteMotion → motion_core → hw_adapter)

**Result:** `extended_mode` flag propagates through the entire pipeline with zero default-mode behavioural change.

### Action interface extensions

| Interface | Fields added |
|---|---|
| `ExecuteMotion.action` (goal) | `bool extended_mode`, `string[] mode_tokens`, `float64 estimated_duration_s` |
| `DispatchTrajectory.action` (goal) | `bool extended_mode` |

### Pipeline flow

1. **HMI/Caller** sends `ExecuteMotion` with `extended_mode=true` + tokens + `estimated_duration_s`
2. **`motion_primitive_executor`** reads `goal->extended_mode`, sets `planning_request.joint_position_guard_mode = Mode::Extended`
3. **`PrimitiveRouterDispatch` Stage A** (pre-downsample) checks `joint_position_guard_.check_trajectory(..., mode)`
4. **`DispatchTrajectoryExecutor`** sets `dispatch_metadata.extended_mode = true`, passes `mode` to `QualityGate::validate_plan`
5. **`QualityGate` Stage B** checks guard with `mode`
6. **`hw_adapter` Stage C** reads `goal->extended_mode` into `TrajectoryExecutionRequest`, checks guard with mode

All stages default to `Mode::Default`; only explicit opt-in activates extended limits.

---

## W7.T5 — validate_safety_chain.py Tiered Shape

**Result:** `validate_safety_chain.py` now parses both flat and tiered `operational_joint_limits` entries.

- `extract_limit(joint_entry)` helper: returns `(min, max)` from either `{min, max}` or `{default: {min, max}}`
- Hardware-subset check uses `default` tier (never `extended` — extended is opt-in, not baseline)
- J5 hard-limit check unchanged

---

## W7.T6 — Tests

### New Python tests (`test_extended_mode.py`)

| Test | What it verifies |
|---|---|
| `test_default_mode_passes` | Default commands unaffected |
| `test_extended_mode_missing_operator_confirm` | Token gate rejects |
| `test_extended_mode_velocity_exceeds_cap` | 0.10 cap rejects |
| `test_extended_mode_all_preconditions_satisfied` | Happy path passes |
| `test_extended_mode_missing_cable_inspection` | Cable token gate rejects |
| `test_extended_mode_duration_exceeds_cap` | 30s cap rejects |
| `test_cooldown_rejects_immediate_retry` | 60s cooldown enforced |
| `test_cooldown_allows_after_wait` | Cooldown expiry allows retry |
| `test_extended_mode_no_tiered_config` | Flat config → extended rejected |
| `test_default_mode_ignores_extended_fields` | Default mode ignores tokens |

### New C++ tests (`test_joint_position_guard.cpp`)

| Test | What it verifies |
|---|---|
| `ModeDefaultRejectsExtendedRange` | 5.0 rad rejected in Default mode |
| `ModeExtendedAcceptsExtendedRange` | 5.0 rad accepted in Extended mode |
| `ModeExtendedRejectsBeyondHardwareCap` | 8.0 rad rejected even in Extended |
| `ModeExtendedFallsBackToDefaultForNonTiered` | No extended tier → falls back to default |

---

## W7.T7 — Documentation

| File | Content |
|---|---|
| `docs/operation/EXTENDED_MODE_RUNBOOK.md` | Full operator runbook: risks, physical checks, token flow, velocity cap rationale, cooldown rationale, audit log format, rollback instructions |
| `AGENTS.md` | New hard-never rule: "Never request extended_mode without all required tokens; never ship default extended_mode = true" |
| `docs/plans/SUMMARY.md` | W7 marked ✅ COMPLETE |

---

## Files Changed

### New files
- `src/safety/tests/test_extended_mode.py`
- `docs/operation/EXTENDED_MODE_RUNBOOK.md`
- `MIGRATION-W7.md`

### Modified files
- `src/safety/config/safety_rules.yaml` — tiered joint_6_t structure
- `src/motion_core/include/motion_core/joint_position_guard.hpp` — Mode enum, TieredLimit
- `src/motion_core/src/joint_position_guard.cpp` — tiered limit logic
- `src/safety/safety/command_validator.py` — precondition gate, cooldown tracker
- `src/interfaces/action/ExecuteMotion.action` — extended_mode, mode_tokens, estimated_duration_s
- `src/interfaces/action/DispatchTrajectory.action` — extended_mode passthrough
- `src/motion_core/include/motion_core/quality_gate.hpp` — Mode parameter on validate_plan
- `src/motion_core/src/quality_gate.cpp` — passes mode to JointPositionGuard
- `src/motion_core/include/motion_core/primitive_router_dispatch.hpp` — joint_position_guard_mode in PlanningRequest
- `src/motion_core/src/primitive_router_dispatch.cpp` — Stage A guard uses mode
- `src/motion_core/src/motion_primitive_executor.cpp` — sets mode from goal, passes to dispatch
- `src/motion_core/include/motion_core/dispatch_trajectory_executor.hpp` — extended_mode in DispatchMetadata, mode in apply_budget_quality_and_dispatch
- `src/motion_core/src/dispatch_trajectory_executor.cpp` — passes mode through pipeline
- `src/hw_adapter/include/hw_adapter/trajectory_executor.hpp` — extended_mode in TrajectoryExecutionRequest
- `src/hw_adapter/src/hw_adapter_dispatch.cpp` — reads extended_mode from DispatchTrajectory goal
- `src/hw_adapter/src/trajectory_executor.cpp` — Stage C guard uses mode
- `src/motion_core/src/motion_core_node.cpp` — YAML parser handles flat + tiered limits
- `tools/validate_safety_chain.py` — extract_limit() for tiered shapes
- `src/motion_core/test/test_joint_position_guard.cpp` — 5 new Mode tests
- `docs/plans/SUMMARY.md` — W7 marked COMPLETE
- `AGENTS.md` — extended_mode hard-never rule

---

## Rollback

```bash
# Disable extended mode entirely (no code change needed)
# Edit src/safety/config/safety_rules.yaml:
#   joint_6_t: {min: -3.142, max: 3.142}
# Any extended_mode=true request will then fail with "config has no extended tier"

# Or revert all W7 commits
git revert <first-w7-commit>..<last-w7-commit>
```

---

**End of W7. T-axis tiered mode complete. Next: W8 cleanup wave (scheduled 2026-05-19).**
