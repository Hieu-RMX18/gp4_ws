# AGENTS.md — gp4_ws

## Discover before you write
1. Run `find src/ -name "*<keyword>*"` and `rg -e "<symbol>"` BEFORE creating
   anything. Paste output in your reply. If shell unavailable, STOP and ask.
2. Reuse code in the same domain. New file when an equivalent exists = REJECTED.

## Write surgically
3. Diff-only output. Max 200 LOC changed per file per change.
4. Match existing style. Do NOT "improve" adjacent code.
5. All numeric constants for safety / motion / perception -> YAML SSOT.
   Zero magic numbers in .py / .cpp source.

## Remove safely
6. Tag `# DEPRECATED: removal_date=<today+28d>, reason=<why>`.
   Hard delete only in cleanup waves AFTER `rg <symbol>` returns 0.

## Hard never
- Never publish directly to /joint_trajectory_controller/*
- Never bypass the safety chain "for testing"
- Never fabricate ros2 / rg / find output. If no shell, STOP and ask.
- Never combine refactor + behavior change in one commit
- Never silently fallback to computeCartesianPath in motion code (banned W1)
- Never reimplement ROS-side logic in hmi/backend/ -- call the ROS service
- Never change a HIGH-sensitivity ROS surface (per docs/hmi/HMI_ROS_INTERFACES.md)
  without paired hmi/ patch in the same PR
- Never use `park_safe` as a named state (deleted per U1, W1.T0).
  Use only states listed in docs/audit/NAMED_STATE_AUDIT.md status=`active`.
- Never silently downsample a trajectory before QualityGate.
  Multi-stage placement is mandatory (W1.T4).
- Never request extended_mode without all required tokens; never ship
  default extended_mode = true. (W7)

## Deprecation lifecycle
- Tag `# DEPRECATED: removal_date=<today+28d>, reason=<why>`.
  Each DEPRECATED tag must have a removal_date.
- Hard delete only in cleanup waves AFTER `rg <symbol>` returns 0 hits.
- Do NOT delete code outside cleanup waves. Tag and wait.

## Branch hygiene
- Working branch is ws-deep-rebuild-3526. State branch at start of every reply.
- Do not checkout a new branch to escape complexity. Refactor in place.
- Surviving branches: main, super-fix, hmi-pro, ws-deep-rebuild-3526.

## Python deps for ROS 2 Humble
- requirements.txt with exact pins. pip install --user. Never sudo pip.
- After install: python3 -c 'import rclpy; print("OK")' must pass.
- Do not activate venv inside a sourced ROS workspace.
- For complex isolated stacks (LLM extras), use a container (Docker / Distrobox).

## Safety invariants
- J5 (joint_5_b) stays at +/-90 deg (1.571 rad). No widening without safety review.
- BLENDED_SEQUENCE requires the typed SequenceStep interface (W2.T0).
  Do not emit BLENDED_SEQUENCE before the interface is merged.
- calibration_date is runtime-filled, never hardcoded in templates or tests.

## Response contract
Every reply contains, in order:
  1. Current branch and wave name
  2. Discovery output pasted (commands echoed as `$ <cmd>`)
  3. Plan: files changed, files NOT changed, rationale
  4. Diffs (unified diff format)
  5. Verification commands actually run
  6. Risks / safety notes
  7. [VERIFIED] / [NEEDS-VALIDATION] / [KNOWN-GAP] tag
  8. End-of-wave stop signal
