# MIGRATION-W2 — Drawing Rewire to BLENDED_SEQUENCE

**Branch**: `ws-deep-rebuild-3526`
**Date**: 2026-05-04
**Status**: COMPLETE

## Summary

Wave 2 rewires the drawing pipeline to emit `BLENDED_SEQUENCE` primitives instead of
`CARTESIAN_PATH`, adds a degenerate CIRC arc check, consolidates workplane hydration
with an SSOT fallback, and extends the `ExecuteMotion` action interface with typed
`SequenceStep` messages.

## Tasks Completed

| Task | Description | Status |
|---|---|---|
| W2.T0 | Interface contract: `SequenceStep.msg`, `ExecuteMotion.action` extension, `llm_schema.yaml`, `primitive_blended_sequence.cpp` | Done |
| W2.T1 | Add `BLENDED_SEQUENCE` to JSON/YAML schema with constraints | Done |
| W2.T2 | Rewire `drawing_geometry.py` + `draw_router.py` to emit `BLENDED_SEQUENCE` | Done |
| W2.T3 | Update `normalizer.py` for `BLENDED_SEQUENCE` per-step normalization | Done |
| W2.T4 | Update `goal_mapper.py` for `BLENDED_SEQUENCE` goal mapping | Done |
| W2.T5 | Consolidate `_hydrate_draw_workplane` — SSOT fallback + deprecate HMI copy | Done |
| W2.T6 | CIRC degenerate arc check in `semantic_validator.py` | Done |
| W2.T7 | Update tests + add new tests (schema, normalizer, semantic, draw_shape) | Done |
| W2.T8 | `plan_only` drawing fix — confirmed correct (gateway rejects execution, router marks metadata) | Done |

## Files Changed

### New files
- `src/interfaces/msg/SequenceStep.msg` — typed step for BLENDED_SEQUENCE

### Modified files
- `src/interfaces/action/ExecuteMotion.action` — added `SequenceStep[] sequence_steps` field
- `src/interfaces/CMakeLists.txt` — registered `SequenceStep.msg`
- `src/primitives/src/primitive_blended_sequence.cpp` — `execute(goal, mgi)` now consumes `goal.sequence_steps`
- `src/llm_gateway/config/llm_schema.yaml` — added `BLENDED_SEQUENCE` enum, `sequence_steps` array schema, conditional validation
- `src/safety/config/safety_rules.yaml` — added `drawing.*` SSOT keys and `circ.degenerate_tolerance`
- `src/llm_gateway/llm_gateway/drawing_geometry.py` — added `_blended_sequence_command`, feature flag, deprecated `_cartesian_path_command`
- `src/llm_gateway/llm_gateway/draw_router.py` — passes `use_blended_sequence` + `blend_radius_m` from SSOT
- `src/llm_gateway/llm_gateway/normalizer.py` — added `BLENDED_SEQUENCE` planner default + per-step normalization
- `src/llm_gateway/llm_gateway/goal_mapper.py` — added `BLENDED_SEQUENCE` branch in `to_execute_motion_goal` and `to_command_payload`
- `src/llm_gateway/llm_gateway/semantic_validator.py` — added `BLENDED_SEQUENCE` validation + `_check_circ_degenerate`
- `src/llm_gateway/llm_gateway/command_pipeline.py` — SSOT fallback workplane when `/get_current_pose` unavailable
- `src/safety/safety/execution_gate.py` — added `BLENDED_SEQUENCE` per-step workspace validation
- `hmi/backend/services/intent_resolution.py` — `DEPRECATED` tag on local `_hydrate_draw_workplane`
- `src/llm_gateway/tests/test_schema_validator.py` — 4 new BLENDED_SEQUENCE tests
- `src/llm_gateway/tests/test_normalizer.py` — 1 new BLENDED_SEQUENCE test
- `src/llm_gateway/tests/test_semantic_validator.py` — 7 new tests (BLENDED_SEQUENCE + CIRC degenerate)
- `src/llm_gateway/tests/test_draw_shape.py` — updated helpers for BLENDED_SEQUENCE
- `src/llm_gateway/tests/test_command_pipeline.py` — updated fallback test
- `src/llm_gateway/tests/test_contract_consistency.py` — added `BLENDED_SEQUENCE` to frozen set

## Feature Flag

Set `drawing.use_blended_sequence: false` in `safety_rules.yaml` to revert to
`CARTESIAN_PATH` emit. Default is `true`.

## Rollback

```bash
git revert <W2-merge-commit>
```

Or set the feature flag to `false` for a runtime-only rollback of the emit path.

## Verification

- `colcon build`: 19 packages, 0 errors
- `colcon test`: 2754 tests, 0 errors, 2 pre-existing failures (jog_pendant, unrelated)
- Python pytest: 300 passed, 13 skipped (integration tests)
- C++ gtest: all primitives tests pass including `test_primitive_blended_sequence`

## Architecture Notes

- `BLENDED_SEQUENCE` is emitted by `_blended_sequence_command()` in `drawing_geometry.py`
- First/last steps have `blend_radius_m=0.0`; intermediates use SSOT `drawing.blend_radius_m` (default 0.008)
- `CARTESIAN_PATH` support is preserved for non-drawing callers; `_cartesian_path_command` is deprecated
- Workplane fallback loads from `safety_rules.yaml` `drawing.fallback_workplane` when `/get_current_pose` is unavailable
- CIRC degenerate check uses cross-product ratio against `circ.degenerate_tolerance` (1e-3)
- HMI's local `_hydrate_draw_workplane` is deprecated; W5 will replace with ROS service call
