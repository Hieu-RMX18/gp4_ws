# EXTENDED_MODE_RUNBOOK — GP4 T-Axis Tiered Mode (W7)

**Status:** Active (opt-in only)
**Applies to:** GP4 workcell with Yaskawa YRC1000micro controller
**Last updated:** 2026-05-05

---

## What extended mode is

The GP4's T-axis (joint_6_t, wrist roll) has a hardware envelope of ±455° (±7.941 rad). By default, the operational envelope is derated to ±180° (±3.142 rad) for safety. Extended mode allows a single command to use the full ±455° range, provided all preconditions are satisfied.

Extended mode is **per-command**, not session-wide. Once the command finishes, the next command starts in default mode again.

## What extended mode is NOT

- NOT a session-wide toggle
- NOT suitable for drawing, pick-and-place, or general manipulation
- NOT a way to bypass velocity limits (hard cap of 0.10 enforced)
- NOT available without explicit operator sign-off

## Hardware risks

- **Cable spool stress**: The internal cable harness experiences nonlinear stress beyond ±180°. Repeated extended-range motion without cooldown can cause micro-fractures.
- **Wrist torque limits**: At high J6 angles combined with J5 near 0°, wrist torque can spike. The velocity cap of 0.10 mitigates this.
- **Silent damage**: Cable damage does not necessarily trigger an immediate alarm. Weekly visual inspection and the 30-day calibration check (W4) are the corroborating mechanisms.

## Required physical checks before sign-off

Before issuing a `cable_inspection_signed_off_token`:

1. **Visual inspection**: Check the J6 cable harness for fraying, kinking, or discoloration.
2. **Range-of-motion test**: Manually jog J6 through ±455° at low speed (≤5%) and listen for unusual noise.
3. **Calibration recency**: Confirm the last hand-eye calibration is within 30 days (`tools/validate_safety_chain.py`).
4. **Document**: Record inspection date, findings, and token issuer in the operations log.

## How to obtain tokens

Tokens are opaque hashes issued by the operator interface after procedural sign-off. They are NOT string constants in the codebase.

- `cable_inspection_signed_off_token`: Issued after physical inspection (see above). Valid for 7 days.
- `operator_confirm_token`: Single-use, time-bound (60s). Issued per-command by the operator at the HMI.

## Velocity cap rationale

Extended mode enforces a hard velocity cap of 0.10 (10% of max joint velocity). This is stricter than the default cap of 0.06 for general motion but allows sufficient speed for inspection tasks while keeping cable stress within safe limits.

## Cooldown rationale

A 60-second cooldown is enforced between extended-mode commands. This allows the cable harness to retract and dissipate any heat buildup from the previous extended-range motion. Bypassing the cooldown risks cumulative cable damage.

## Preconditions enforced by the safety chain

| # | Precondition | Check location | Failure message |
|---|---|---|---|
| 1 | `cable_inspection_signed_off_token` present | `command_validator.py` | "requires cable_inspection_signed_off_token" |
| 2 | `velocity_scale` ≤ 0.10 | `command_validator.py` | "velocity_scale=X > cap 0.1" |
| 3 | `operator_confirm_token` present | `command_validator.py` | "requires operator_confirm_token" |
| 4 | Cooldown ≥ 60s since last extended run | `command_validator.py` | "cooldown not elapsed" |
| 5 | `estimated_duration_s` ≤ 30s | `command_validator.py` | "estimated_duration Xs > cap 30s" |
| 6 | J6 positions within ±7.941 rad | `JointPositionGuard` (Stages A/B/C) | "outside [min, max] (mode=extended)" |

## Reading the audit log

Every extended-mode goal is logged by the audit logger with tokens redacted to last 8 characters:

```
[audit] extended_mode goal_id=<uuid> tokens=cab_ins=****1234,op_conf=****5678
```

Search for `extended_mode` in the supervisor audit log to verify behaviour.

## Rollback

To disable extended mode entirely, remove the `extended` block from `joint_6_t` in `safety_rules.yaml`. Any `extended_mode: true` request will then be rejected with "config has no extended tier".

```bash
# Quick rollback: edit safety_rules.yaml
# Change joint_6_t from tiered back to flat:
#   joint_6_t: {min: -3.142, max: 3.142}
```

## References

- `docs/plans/W7_t_axis_tiered_mode.md` — design spec
- `src/safety/config/safety_rules.yaml` — SSOT for limits and preconditions
- `src/safety/safety/command_validator.py` — precondition gate implementation
- `src/motion_core/include/motion_core/joint_position_guard.hpp` — Mode enum and tiered API
- `tools/validate_safety_chain.py` — safety chain validator (handles tiered shape)
