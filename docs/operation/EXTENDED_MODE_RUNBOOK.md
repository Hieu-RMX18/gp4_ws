# EXTENDED_MODE_RUNBOOK — GP4 T-Axis Tiered Mode (W7)

**Status:** Disabled (legacy runbook retained for traceability)
**Applies to:** GP4 workcell with Yaskawa YRC1000micro controller
**Last updated:** 2026-05-13

---

## What extended mode is

The GP4's T-axis (joint_6_t, wrist roll) has a hardware envelope of ±455° (±7.941 rad). The current operational envelope is derated to ±180° (±3.142 rad) for safety.

Extended mode is currently **disabled** for hardware debugging and normal operation.
Any `extended_mode: true` request is rejected by `CommandValidator` with a
message directing the operator to use the default `joint_6_t` envelope.

## What extended mode is NOT

- NOT available in the current runtime policy
- NOT a session-wide toggle
- NOT suitable for drawing, pick-and-place, or general manipulation
- NOT a way to bypass velocity limits

## Hardware risks

- **Cable spool stress**: The internal cable harness experiences nonlinear stress beyond ±180°. Repeated extended-range motion without cooldown can cause micro-fractures.
- **Wrist torque limits**: At high J6 angles combined with J5 near 0°, wrist torque can spike. The velocity cap of 0.10 mitigates this.
- **Silent damage**: Cable damage does not necessarily trigger an immediate alarm. Weekly visual inspection and the 30-day calibration check (W4) are the corroborating mechanisms.

## Required physical checks before sign-off

This procedure is retained only for a future safety review if extended mode is
reintroduced. No current runtime path issues or consumes these tokens.

Before any future re-enable decision:

1. **Visual inspection**: Check the J6 cable harness for fraying, kinking, or discoloration.
2. **Range-of-motion test**: Manually jog J6 through ±455° at low speed (≤5%) and listen for unusual noise.
3. **Calibration recency**: Confirm the last hand-eye calibration is within 30 days (`tools/validate_safety_chain.py`).
4. **Document**: Record inspection date, findings, and token issuer in the operations log.

## Legacy tokens

The following token names are legacy documentation only while extended mode is
disabled. They should not be required for normal hardware debugging or standard
`joint_6_t` motion inside ±180°.

- `cable_inspection_signed_off_token`: Issued after physical inspection (see above). Valid for 7 days.
- `operator_confirm_token`: Single-use, time-bound (60s). Issued per-command by the operator at the HMI.

## Legacy velocity cap rationale

If extended mode is reintroduced, it should enforce a hard velocity cap no higher
than 0.10 (10% of max joint velocity). The current default motion cap remains
0.06.

## Legacy cooldown rationale

If extended mode is reintroduced, a cooldown between extended-mode commands
should be required to reduce cumulative cable stress.

## Disabled-mode behavior

| # | Check | Location | Expected result |
|---|---|---|---|
| 1 | `extended_mode: true` request | `command_validator.py` | Reject: "extended_mode disabled; use default joint_6_t envelope" |
| 2 | Standard motion with no extended tokens | `command_validator.py` | Validate against normal motion limits |
| 3 | J6 positions | `JointPositionGuard` (Stages A/B/C) | Enforce default ±180° envelope |

## Reading the audit log

If extended mode is reintroduced later, every extended-mode goal should be logged
by the audit logger with tokens redacted to last 8 characters:

```
[audit] extended_mode goal_id=<uuid> tokens=cab_ins=****1234,op_conf=****5678
```

Search for `extended_mode` in the supervisor audit log only if a future wave
reintroduces this mode.

## Rollback

Extended mode is already disabled by omitting the `extended` block from
`joint_6_t` in `safety_rules.yaml`.

```bash
# Quick rollback: edit safety_rules.yaml
# Change joint_6_t from tiered back to flat:
#   joint_6_t: {min: -3.142, max: 3.142}
```

## References

- `src/safety/config/safety_rules.yaml` — SSOT for default joint limits
- `src/safety/safety/command_validator.py` — disabled-mode rejection
- `src/motion_core/include/motion_core/joint_position_guard.hpp` — Mode enum and tiered API
- `tools/validate_safety_chain.py` — safety chain validator (handles tiered shape)
