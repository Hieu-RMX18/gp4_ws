# W7 — T-Axis Tiered Mode (joint_6_t Default ±180° / Extended ±455° Opt-in)

**Wave class:** Safety extension (optional)
**Risk:** Low (gated by precondition checks; default unchanged)
**Estimated effort:** 2–3 working days
**Depends on:** W1 (operational_joint_limits installed; JointPositionGuard active), W6 (cleanup complete; CI tightened)
**Unblocks:** None (final wave in this plan)

---

## Goal

The Yaskawa GP4's T-axis (joint_6_t, the wrist roll) has a hardware envelope of ±455°. W1 derated this to ±180° via `operational_joint_limits` because:

- Cable spool damage risk grows nonlinearly past ±180°.
- Most useful tasks (drawing, pick-and-place, scanning) fit inside ±180°.
- Wrist-flip prone region (J5 ≈ 0 with J4/J6 large) is mitigated when J6 is bounded.

But some tasks legitimately need extended T-axis travel: cable inspection through holes, single-station screwdriving with long bit insertion, certain scanning patterns. W7 adds an opt-in extended mode that widens J6 to ±455° provided every precondition is satisfied. Mode promotion is per-command, not session-wide. Once the command finishes, the next command starts in default mode again.

W7 does NOT change default behaviour. A command without the explicit opt-in flags executes exactly as in W6. The wave only adds a new gated path.

---

## Why this is optional

The user may decide that all production tasks fit within ±180°. In that case W7 is unnecessary and the workspace ends at W6. If extended T-axis is needed for a known use case, W7 lands. The decision is a human one, not an agent one.

The plan includes W7 because the GP4's hardware capability is real — leaving it unreachable forever is also a choice that should be made deliberately, not by default.

---

## Discovery (paste raw output)

```bash
# A. Confirm W1's operational_joint_limits structure for joint_6_t
rg -n "joint_6_t|operational_joint_limits" src/safety/config/safety_rules.yaml

# B. Confirm JointPositionGuard reads joint_6_t correctly
rg -n "joint_6_t" src/motion_core/

# C. Existing safety preconditions infrastructure (where confirms / signed_off flags live)
rg -n "signed_off|operator_confirm|precondition" src/safety/ src/llm_gateway/

# D. ExecuteMotion goal fields (what does the action carry?)
cat src/interfaces/action/ExecuteMotion.action 2>/dev/null
ls src/interfaces/action/

# E. ValidateCommand request fields
cat src/interfaces/srv/ValidateCommand.srv 2>/dev/null

# F. Audit logger (W7 must log mode promotion attempts)
rg -n "audit\|AuditLogger" src/supervisor/

# G. HMI surface for mode promotion (does HMI need a UI element? — out of scope here, but flag)
rg -n "extended\|tiered\|joint_6_t" hmi/

# H. velocity_scale enforcement (W7's hard cap of 0.10 in extended mode)
rg -n "velocity_scale" src/safety/safety/command_validator.py
```

---

## Tasks

### W7.T1 — Extend `operational_joint_limits` schema for tiered mode

File: `src/safety/config/safety_rules.yaml`

W1 wrote a flat key for joint_6_t. W7 turns it into a tiered structure:

```yaml
operational_joint_limits:
  joint_1_s: {min: -2.967, max:  2.967}
  joint_2_l: {min: -1.920, max:  2.269}
  joint_3_u: {min: -1.134, max:  3.491}
  joint_4_r: {min: -2.443, max:  2.443}
  joint_5_b: {min: -1.571, max:  1.571}
  joint_6_t:
    default:  {min: -3.142, max:  3.142}     # ±180 deg
    extended: {min: -7.941, max:  7.941}     # ±455 deg
    extended_preconditions:
      cable_inspection_signed_off: true       # operator certifies cable not at risk
      max_velocity_scale: 0.10                # hard cap; runtime-enforced
      requires_operator_confirm: true         # explicit confirm token per command
      max_continuous_extended_time_s: 30      # cannot stay in extended mode beyond this
      cool_down_s_between_runs: 60            # gap before next extended-mode command

joint_position_guard:
  enabled: true
  reject_message_template: "joint_position_guard reject at point[{idx}]: {joint} = {value:.4f} rad outside [{min:.4f}, {max:.4f}] (mode={mode})"
```

**Backward compatibility:** if the new schema is rejected by W1's loader, the loader is updated to handle both flat and tiered shapes. The flat shape is interpreted as `default`-only (no extended mode); reject any `extended_mode: true` request immediately.

### W7.T2 — Update C++ `JointPositionGuard` for tiered limits

File: `src/motion_core/include/motion_core/joint_position_guard.hpp` and `joint_position_guard.cpp`.

```cpp
struct TieredLimit {
  JointLimit default_limit;
  std::optional<JointLimit> extended_limit;  // nullopt if no extended tier
};

class JointPositionGuard {
public:
  enum class Mode { Default, Extended };

  bool check_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj,
    Mode mode,
    std::string & reason) const;
  // ... existing API preserved; default-mode wrapper calls with Mode::Default
};
```

The mode is an explicit parameter at the call site. There is no implicit default to extended; the caller MUST pass it.

`QualityGate::validate_trajectory` gains a `Mode` parameter, defaulting to `Default`. The motion_core dispatcher passes `Default` unless the request payload's `extended_mode == true` and the precondition checks below have already been satisfied upstream.

### W7.T3 — Precondition gate in `command_validator.py`

File: `src/safety/safety/command_validator.py`.

Before any command claiming `extended_mode: true` reaches motion_core, the safety chain runs the precondition gate:

```python
def _check_extended_mode_preconditions(self, command: dict) -> tuple[bool, str]:
    if not command.get("extended_mode"):
        return True, ""   # default mode; no preconditions
    cfg = self._policy["operational_joint_limits"]["joint_6_t"]
    if "extended" not in cfg:
        return False, "extended_mode requested but config has no extended tier"

    pre = cfg["extended_preconditions"]

    # 1. Cable inspection sign-off
    if pre["cable_inspection_signed_off"] and not command.get("cable_inspection_signed_off_token"):
        return False, "extended_mode requires cable_inspection_signed_off_token; operator must certify"

    # 2. Velocity cap
    cap = pre["max_velocity_scale"]
    vel = command.get("velocity_scale", 0.06)  # default per W1
    if vel > cap:
        return False, f"extended_mode velocity_scale={vel} > cap {cap}"

    # 3. Operator confirm token
    if pre["requires_operator_confirm"] and not command.get("operator_confirm_token"):
        return False, "extended_mode requires operator_confirm_token (single-command scope)"

    # 4. Cooldown enforcement (uses an in-memory state)
    last_extended_end = self._extended_runs.last_end_time()
    if last_extended_end is not None:
        elapsed = (datetime.utcnow() - last_extended_end).total_seconds()
        if elapsed < pre["cool_down_s_between_runs"]:
            return False, f"extended_mode cooldown not elapsed: {elapsed:.1f}s < {pre['cool_down_s_between_runs']}s"

    # 5. Estimated duration cap
    estimated_s = command.get("estimated_duration_s", 0)
    if estimated_s > pre["max_continuous_extended_time_s"]:
        return False, f"extended_mode estimated_duration {estimated_s}s > cap {pre['max_continuous_extended_time_s']}s"

    return True, ""
```

The `_extended_runs` tracker is a simple module-level state that records the end time of every extended-mode command. Thread-safety: `validate_command` is single-threaded per the existing safety chain executor; if that changes, lock.

### W7.T4 — Mode plumbing through ExecuteMotion

File: `src/interfaces/action/ExecuteMotion.action` (read in discovery D; modify if needed).

Add an `extended_mode` boolean and `mode_tokens` string array to the goal. If the action interface cannot be modified without major HMI rework, route the flag via the request payload's freeform metadata field (whichever `goal_mapper.py` already uses).

The audit logger (`src/supervisor/src/audit_logger.cpp:142,152`) records every extended-mode goal with the tokens redacted to last 8 chars (privacy + diagnostic).

### W7.T5 — `query_extended_mode_status` ROS service (optional)

If HMI wants to display "extended mode active / cooling down / available", add a service `/safety/extended_mode_status` that returns:

```yaml
status: <available | active | cooldown>
last_run_end: <ISO 8601 or null>
seconds_until_available: <float>
```

This is small and read-only. If HMI does not need it, skip.

### W7.T6 — Tests

`src/safety/tests/test_extended_mode.py`:

- Default-mode command: passes, no preconditions checked.
- Extended-mode without `operator_confirm_token`: rejected.
- Extended-mode with stale `operator_confirm_token` (older than 60s, if the contract specifies freshness): rejected.
- Extended-mode with `velocity_scale=0.15`: rejected (>cap 0.10).
- Extended-mode with all preconditions satisfied: accepted; J6 trajectory point at +6.0 rad passes JointPositionGuard.
- Default-mode with J6 at +6.0 rad: rejected (>π).
- Cooldown: extended run completes; immediate retry rejected; after 61s wait, accepted.
- Audit logger: extended-mode runs visible in audit log with tokens redacted.

`src/motion_core/test/test_joint_position_guard.cpp`:

- Mode::Default rejects J6 = +5.0 rad.
- Mode::Extended accepts J6 = +5.0 rad.
- Mode::Extended rejects J6 = +8.0 rad (still beyond extended cap of 7.941).

### W7.T7 — Documentation

`docs/operation/EXTENDED_MODE_RUNBOOK.md` (new):

- What extended mode is and is not.
- Hardware risks (cable spool stress, wrist torque limits at high speeds).
- Required physical checks before sign-off.
- How to obtain the `cable_inspection_signed_off_token` (procedural, not just a constant).
- Velocity cap rationale.
- Cooldown rationale.
- Reading the audit log to verify behaviour.

`AGENTS.md` adds a single line under "Hard never": `Never request extended_mode without all four tokens; never ship default extended_mode = true.`

---

## Verification

| # | Check | Pass criterion |
|---|---|---|
| 1 | Default-mode command with J6 = +5.0 rad | Rejected by JointPositionGuard with "outside [-3.1416, 3.1416]" |
| 2 | Extended-mode command with all tokens, J6 = +5.0 rad | Accepted; trajectory executes in sim |
| 3 | Extended-mode missing `operator_confirm_token` | Rejected at precondition gate, not at JointPositionGuard |
| 4 | Extended-mode with `velocity_scale=0.15` | Rejected with "velocity_scale ... > cap 0.10" |
| 5 | Two extended-mode commands within 60s | Second rejected with "cooldown not elapsed" |
| 6 | Audit log inspection | Each extended-mode run logged with tokens redacted |
| 7 | `python tools/validate_safety_chain.py` | Passes; new schema accepted |
| 8 | `colcon test --packages-select safety motion_core` | Green |
| 9 | Default-mode regression suite | All W1–W6 tests still pass; no behaviour change for default mode |
| 10 | Hardware verification (operator-led) | Single demonstration of an extended-mode cable inspection task with all tokens; recorded |

---

## DON'T

- Do not enable extended mode by default. The default safety stance is preserved.
- Do not allow extended mode to be toggled session-wide. Per-command only.
- Do not store the `cable_inspection_signed_off_token` as a string constant in the codebase. The token comes from a procedural sign-off captured by the operator interface; the value is an opaque hash with provenance, not a literal.
- Do not exceed the velocity cap "for testing". Velocity cap is part of the safety contract.
- Do not remove the cooldown to "speed up demos". The cooldown lets cable retract; bypassing risks cable damage.
- Do not extend the J6 limits beyond ±455°. ±455° is the hardware spec; further is impossible without damage.
- Do not use extended mode for drawing or pick-and-place. The default ±180° envelope is sufficient. Extended mode is for narrow inspection / insertion tasks.
- Do not allow extended mode and the W2 BLENDED_SEQUENCE blend radius to interact silently. If a sequence step has J6 beyond ±180°, the mode flag must be set explicitly for that step (and W7 must validate per-step, not just per-sequence).
- Do not bundle W7 with any non-tiered-mode change. W7's commit history must be cleanly revertable.

---

## Output artefacts

- `src/safety/config/safety_rules.yaml` — diff: tiered structure for joint_6_t
- `src/safety/safety/command_validator.py` — diff: precondition gate
- `src/safety/safety/policy_loader.py` — diff: handle tiered shape
- `src/safety/tests/test_extended_mode.py` — new
- `src/motion_core/include/motion_core/joint_position_guard.hpp` — diff: Mode enum, tiered API
- `src/motion_core/src/joint_position_guard.cpp` — diff
- `src/motion_core/include/motion_core/quality_gate.hpp` — diff: Mode parameter
- `src/motion_core/src/quality_gate.cpp` — diff
- `src/motion_core/test/test_joint_position_guard.cpp` — diff: extended-mode cases
- `src/interfaces/action/ExecuteMotion.action` — diff: extended_mode + mode_tokens (if path A) OR no diff (if path B uses metadata)
- `src/llm_gateway/llm_gateway/goal_mapper.py` — diff: route mode flag
- `src/supervisor/src/audit_logger.cpp` — diff: log redacted tokens
- `tools/validate_safety_chain.py` — diff: tiered-shape support
- `docs/operation/EXTENDED_MODE_RUNBOOK.md` — new
- `AGENTS.md` — one-line addition under "Hard never"
- `MIGRATION-W7.md`

---

## Rollback procedure

```bash
# Quickest: SSOT toggle
# Edit safety_rules.yaml: remove the extended block from joint_6_t,
# revert to W1's flat structure. Any extended_mode request now hits
# "config has no extended tier" and is rejected.

# Full revert
git revert -m 1 <W7 merge commit>
# Default behaviour is unaffected because W1's defaults stand on their own.
```

---

## Risk notes

- **Operator habit risk**: once extended mode exists, operators may push for laxer use. The cooldown + velocity cap + per-command tokens make this hard but not impossible. Audit log review is the long-term check.
- **Cable damage is silent**: a damaged cable does not necessarily cause an immediate alarm. The 30-day calibration check (W4) and weekly visual inspection (operations runbook) are the corroborating mechanisms.
- **Mode mixing in sequences**: a BLENDED_SEQUENCE (W2) where some steps are default-mode and others extended-mode is a per-step mode property, NOT a sequence-level property. The C++ dispatcher must check each step's mode independently. If the dispatcher cannot do per-step mode, W7 forbids extended_mode inside sequences.
- **Token freshness**: if `operator_confirm_token` is reusable across commands, the operator confirm becomes a no-op. The token must be single-use AND time-bound (e.g. 60s). The operator interface must enforce this; the safety chain checks the token's claims.

---

## Stop signal

End of W7. End of plan v3.

State explicitly: `End of W7. Rebuild plan v3 complete. Recurring cleanup waves continue per W6 cadence.`

---

**Reliability tag:** `[VERIFIED]` for the design — tiered limits with precondition gates is a standard safety pattern; W7 follows it without invention. `[NEEDS-VALIDATION]` for the GP4-specific cable-stress thresholds (the 30s continuous + 60s cooldown numbers are conservative defaults; operator may tune after demonstrating cable behaviour). `[KNOWN-GAP]` for the operator-side procedural sign-off — that lives outside the codebase; W7 only consumes the resulting token and audits.
