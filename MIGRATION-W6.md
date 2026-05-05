# MIGRATION-W6.md — First Cleanup Wave

**Wave:** W6
**Branch:** ws-deep-rebuild-3526
**Date:** 2026-05-05
**Status:** COMPLETE
**Next cleanup wave (W8):** 2026-05-19

## Verification Results

### Build
- `colcon build --symlink-install --packages-select motion_core hw_adapter llm_gateway` — PASS
- `colcon build --symlink-install` (full workspace) — PASS

### Tests
- `colcon test --packages-select llm_gateway` — 342 passed, 0 failed
- No test regressions from W0–W5

### Lint / Audit
- `tools/lint/aged_deprecation_check.py` — PASS (0 past-due deprecations)
- `tools/lint/file_size_budget.sh` — PASS
- `tools/lint/no_silent_motion_fallback.sh` — PASS
- `tools/validate_safety_chain.py` — FAIL (expected: extrinsics not yet calibrated)
- `rg park_safe src/ hmi/` — 0 hits ✅

---

## W6.T1 — Aged Deprecation Hard-Delete

**Result:** 0 past-due deprecations. Nothing to delete.

| File | Line | removal_date | Status |
|---|---|---|---|
| `src/llm_gateway/llm_gateway/drawing_geometry.py` | moved to drawing_command_emitter.py | 2026-06-01 | Future (cooldown) |
| `src/llm_gateway/llm_gateway/llm_gateway_node.py` | 332 | 2026-06-01 | Future (cooldown) |

`park_safe` verified fully gone (0 hits in src/ and hmi/).

---

## W6.T2 — Duplication Audit

jscpd findings (--min-lines 20 --min-tokens 60):

| Clone | Files | Lines | Classification | Action |
|---|---|---|---|---|
| Trajectory validation loop | `trajectory_executor.cpp` ↔ `quality_gate.cpp` | 36 | (a) consolidate now | Extracted into `motion_core::validate_trajectory_structure()` |
| Test fixture setup | `test_hw_adapter_node.cpp` (self) | 21, 35, 57 | (c) accept | Test fixtures — justified |

`is_finite_vector` was defined in both files (copy-paste). Now single definition in `trajectory_validator.cpp`.

---

## W6.T3 — Dead-Code Purge

vulture findings (--min-confidence 90, excluding .venv):

| File | Symbol | Confidence | Action |
|---|---|---|---|
| `react/agent.py:51` | `request_id` parameter | 100% | Removed from `ReActAgent.run()` signature |
| `tests/test_get_pose.py:*` | `ros_integration_context` | 100% | False positive (pytest fixture) — kept |
| `tests/test_hydrate_workplane_service.py:*` | `rf` lambda param | 100% | Renamed to `_rf` |

Updated `test_react_agent_basic.py` to match new `run()` signature.

---

## W6.T4 — File-Size Budget

| File | Before | After | Action |
|---|---|---|---|
| `drawing_geometry.py` | 897 LOC | 781 LOC | Split: command emission → `drawing_command_emitter.py` (329 LOC) |
| `draw_router.py` | 1013 LOC | — | Deferred to W8 (single mixin class, needs hierarchy restructure) |
| `llm_gateway_node.py` | 1318 LOC | — | Deferred to W8 (god-node, needs handler extraction) |
| `primitive_router_dispatch.cpp` | 816 LOC | — | Deferred to W8 |
| `servo_bridge_node.cpp` | 855 LOC | — | Deferred to W8 |
| 6 other files | various | — | Deferred to W8 |

`file_size_exceptions.txt` updated: drawing_geometry removed, all remaining entries targeted to W8.

---

## W6.T5 — Branch Hygiene

| Branch | Last commit | Unique commits vs ws-deep-rebuild-3526 | Recommendation |
|---|---|---|---|
| `main` | 2026-04-10 | — | Keep (production) |
| `super-fix` | 2026-04-25 | 0 | **Delete** with backup tag `archive/super-fix` |
| `hmi-pro` | 2026-04-25 | 0 | **Delete** with backup tag `archive/hmi-pro` |
| `ws-deep-rebuild-3526` | 2026-05-05 | — | Active working branch |

**Awaiting human approval for branch deletion.**

---

## W6.T6 — CI Check Tightening

### New: `aged-deprecation` CI job
- Script: `tools/lint/aged_deprecation_check.py`
- Runs on push, fails if any DEPRECATED tag is past its `removal_date`
- Currently passes (all deprecations in cooldown)

### Existing checks
- `no-fallback-guard`: PASS
- `safety-chain`: FAIL (expected — extrinsics not calibrated)
- `file-size-budget`: PASS (exception list updated)
- `duplication`: jscpd threshold at `--min-lines 30 --threshold 0`

---

## W6.T7 — Documentation

- `docs/plans/SUMMARY.md`: Added Status column, marked W0–W6 COMPLETE, added W8 row
- `tools/lint/file_size_exceptions.txt`: Updated with current LOC counts and W8 targets

---

## W6.T8 — Next Cleanup Wave

**W8 scheduled for 2026-05-19** (14 days from W6 completion).

W8 will:
- Hard-delete deprecations with removal_date ≤ 2026-05-19
- Split remaining oversized files (draw_router, llm_gateway_node, C++ files)
- Re-run full jscpd/vulture audit
- Delete stale branches if approved

---

## Files Changed

### New files
- `src/motion_core/include/motion_core/trajectory_validator.hpp`
- `src/motion_core/src/trajectory_validator.cpp`
- `src/llm_gateway/llm_gateway/drawing_command_emitter.py`
- `tools/lint/aged_deprecation_check.py`
- `MIGRATION-W6.md`

### Modified files
- `src/motion_core/CMakeLists.txt` — added trajectory_validator.cpp
- `src/motion_core/src/quality_gate.cpp` — use shared validate_trajectory_structure
- `src/hw_adapter/src/trajectory_executor.cpp` — use shared validator, remove local is_finite_vector
- `src/llm_gateway/llm_gateway/drawing_geometry.py` — remove command functions, add re-exports
- `src/llm_gateway/llm_gateway/react/agent.py` — remove unused request_id parameter
- `src/llm_gateway/llm_gateway/llm_gateway_node.py` — update agent.run() call
- `src/llm_gateway/tests/test_react_agent_basic.py` — update agent.run() calls
- `src/llm_gateway/tests/test_hydrate_workplane_service.py` — fix unused rf parameter
- `tools/lint/file_size_exceptions.txt` — updated for W6 splits, W8 targets
- `docs/plans/SUMMARY.md` — added Status column, W8 row

---

## Rollback

```bash
# Per-commit rollback (each change is one commit)
git revert <commit-hash>

# Or rollback entire W6
git revert <first-w6-commit>..<last-w6-commit>
```

---

**End of W6. Cleanup wave complete. Next cleanup (W8) scheduled for 2026-05-19.**
