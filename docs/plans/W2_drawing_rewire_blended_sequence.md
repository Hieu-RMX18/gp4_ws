# W2 — Drawing Rewire to BLENDED_SEQUENCE, Workplane Consolidation, CIRC Degenerate Check

**Wave class:** Motion behaviour rewire
**Risk:** Medium (drawing tests will need updates; sim-validate before hardware)
**Estimated effort:** 3–5 working days
**Depends on:** W1 (silent fallback closed; otherwise verification is unreliable)
**Unblocks:** W3 (ReAct's `submit_motion` calls drawing pipeline; needs stable output)

---

## Goal

Three changes, one theme: drawing pipeline emits the `BLENDED_SEQUENCE` primitive (which already exists with full `MotionSequenceRequest` support), workplane logic stops being implemented in three places, and the gateway rejects degenerate CIRC arcs at parse time instead of at the C++ planner.

The C++ side is ready. `primitives/src/primitive_blended_sequence.cpp` (827 LOC) already implements `moveit_msgs::msg::MotionSequenceRequest` with `MotionSequenceItem` and `blend_radius`. The dispatcher at `primitive_dispatcher.cpp:71` registers it. Tests exist at `test_primitive_blended_sequence.cpp`. **What's missing is callers.**

W2's job is to make the drawing pipeline a caller. Then deprecate the use of `CARTESIAN_PATH` for drawing (the primitive itself stays for non-drawing callers; W6 audits whether anyone else still needs it).

---

## Discovery (paste raw output)

```bash
# A. Find every emit site of CARTESIAN_PATH primitive_type
rg -n '"primitive_type": "CARTESIAN_PATH"|primitive_type.*=.*"CARTESIAN_PATH"' --type py

# B. Find every emit site that should switch to BLENDED_SEQUENCE
rg -n "drawing_geometry|draw_router|draw_shape|draw_text" --type py -l

# C. Confirm BLENDED_SEQUENCE schema/contract
rg -n "BLENDED_SEQUENCE|sequence_steps|blend_radius" --type py
sed -n '1,60p' src/primitives/include/primitives/primitive_blended_sequence.hpp
sed -n '430,470p' src/primitives/src/primitive_blended_sequence.cpp

# D. Find the workplane logic in all three locations
rg -n "_hydrate_draw_workplane|hydrate_draw_workplane" --type py

# E. Find the CIRC primitive emit and any existing checks
rg -n '"primitive_type": "CIRC"|primitive_type.*"CIRC"' --type py
rg -n "auxiliary|aux_pose|aux_point" --type py | rg -i "circ"

# F. Existing schema for CARTESIAN_PATH and (if present) BLENDED_SEQUENCE
find src/llm_gateway -name "schema*.json" -o -name "schemas/*"
find src/interfaces -name "*.msg" -o -name "*.action" -o -name "*.srv"
ls src/interfaces/

# G. Tests that exercise drawing
ls src/llm_gateway/tests/ | rg -i "draw|geometry|sequence"

# H. HMI references (W2 changes ROS surface; HMI must stay compatible)
rg -n '"CARTESIAN_PATH"|primitive_type.*CARTESIAN_PATH' hmi/backend/
```

If discovery shows BLENDED_SEQUENCE has no JSON schema entry in `llm_gateway/`, that is W2.T1's first task. Discovery output paste is mandatory.

---

## Tasks

### W2.T0 — Interface contract: ExecuteMotion + SequenceStep + llm_schema (precondition; per F1)

**Why this is mandatory and comes first:**

The original W2 assumed BLENDED_SEQUENCE could be emitted by `drawing_geometry.py` and would route through the existing public path. The reviewer demonstrated this is false:

- `src/interfaces/action/ExecuteMotion.action` has no `sequence_steps` or `blend_radius` fields (verified by reviewer).
- `src/llm_gateway/config/llm_schema.yaml` does not allow `primitive_type: BLENDED_SEQUENCE` (verified).
- `src/primitives/src/primitive_blended_sequence.cpp:825` explicitly rejects: `"ExecuteMotion goal for BLENDED_SEQUENCE lacks sequence steps; use typed SequenceStep API"` (verified by transcript discovery).

So `primitive_blended_sequence.cpp` exists, the dispatcher registers it, but **there is no public path to invoke it**. Emitting BLENDED_SEQUENCE today fails immediately at line 825. W2 must wire the contract first.

**Tasks:**

1. **Define the SequenceStep message.** New file `src/interfaces/msg/SequenceStep.msg`:
One step of a blended LIN sequence.
string primitive_type           # "LIN" or "PTP" or "CIRC"; "LIN" for drawing
geometry_msgs/PoseStamped target_pose
float64 blend_radius_m          # 0.0 for first/last, > 0.0 for intermediates
string planner_id               # "PILZ_LIN" or "PILZ_PTP"
float64 velocity_scale          # 0.0–1.0; required (per W1 fail-closed)
float64 acceleration_scale      # 0.0–1.0; required

2. **Extend ExecuteMotion.action.** In `src/interfaces/action/ExecuteMotion.action`, add to the goal section (do NOT remove existing fields):
Existing goal fields stay.
New optional field: when primitive_type == "BLENDED_SEQUENCE",
sequence_steps carries the typed steps. When primitive_type is anything
else, sequence_steps must be empty.
SequenceStep[] sequence_steps

3. **Register in CMakeLists / package.xml.** In `src/interfaces/CMakeLists.txt` add `msg/SequenceStep.msg` to `rosidl_generate_interfaces(...)`. Confirm `geometry_msgs` is already a `<depend>` in `package.xml`.

4. **Update llm_schema.yaml.** In `src/llm_gateway/config/llm_schema.yaml`:
   - Add `BLENDED_SEQUENCE` to the enum of allowed `primitive_type` values (location confirmed by W2 discovery's grep on the file).
   - Add a `sequence_steps` field schema (array of objects, each matching SequenceStep fields, with constraints: ≥2 items, first/last `blend_radius_m=0.0`, intermediates >0.0 and ≤ `drawing.blend_radius_m_max` from SSOT).

5. **Update primitive_blended_sequence.cpp**. The current line 825 raises the error message that proves the contract was missing. After Tasks 1–4 are merged, replace lines around 818–825 to consume `goal.sequence_steps` instead of failing. Specifically:
   - Read `goal.sequence_steps` from the ExecuteMotion goal.
   - For each step, build a `pilz_industrial_motion_planner::MotionSequenceItem` with the step's pose, blend_radius_m, planner_id.
   - Submit to MoveGroupSequence per the existing implementation in the rest of the file.
   - On invalid sequence (zero steps, or first/last blend_radius != 0): return failure with explicit reason; do NOT silently fall back.

6. **HMI compatibility (per F5).** After Tasks 1–5, run the HMI inventory diff (per W0.T9 re-verification rule):
```bash
   rg -n -e '/execute_motion|ExecuteMotion' hmi/
   rg -n -e 'primitive_type|sequence_steps' hmi/
```
   Verify HMI does NOT introspect ExecuteMotion's `sequence_steps` field today (it should not, since the field is new). If HMI consumed only fields that still exist, no HMI patch is needed. If HMI does need updates, list them in `MIGRATION-W2.md` and pair the change.

**Verification of W2.T0 specifically:**

| # | Check | Pass criterion |
|---|---|---|
| T0.1 | `cat src/interfaces/msg/SequenceStep.msg` | File exists, fields per Task 1 |
| T0.2 | `rg -n 'SequenceStep\[\] sequence_steps' src/interfaces/action/ExecuteMotion.action` | Hit; field added |
| T0.3 | `colcon build --packages-select interfaces` | Green; SequenceStep.msg generates Python and C++ headers |
| T0.4 | Unit test: build an ExecuteMotion goal with primitive_type=BLENDED_SEQUENCE and 3 sequence_steps; submit to mocked motion_core | `primitive_blended_sequence.cpp` consumes the steps without raising the line 825 error |
| T0.5 | Unit test: ExecuteMotion goal with primitive_type=BLENDED_SEQUENCE and 0 sequence_steps | Rejected with explicit reason (NOT the old "lacks sequence steps" error from line 825) |
| T0.6 | `python -m yamllint src/llm_gateway/config/llm_schema.yaml` | Pass; new BLENDED_SEQUENCE enum value present |
| T0.7 | HMI re-verification grep | No HMI breakage; or HMI patch included in same PR |

**No-conflict guarantee:** Tasks 1–5 are purely additive to the public interface (new message, new optional field, new enum value). Existing primitives (PTP, LIN, CIRC, CARTESIAN_PATH) are untouched. The line 825 replacement only changes behaviour when primitive_type=BLENDED_SEQUENCE, which is a new code path. No regression in non-drawing flows is possible.

**Stop signal:** all T0.* checks green. THEN proceed to W2.T1.

### W2.T1 — Add `BLENDED_SEQUENCE` to the JSON schema
**Precondition:** W2.T0 must be merged before this task starts. Without W2.T0 the contract is missing and emitting BLENDED_SEQUENCE produces the line 825 error from primitive_blended_sequence.cpp, masking any other progress.

File: the JSON schema used by `schema_validator.py` (location confirmed by discovery F; likely `src/llm_gateway/llm_gateway/schemas/` or referenced from `setup.py`).

The schema must accept a primitive payload of the form:

```json
{
  "primitive_type": "BLENDED_SEQUENCE",
  "frame_id": "base_link",
  "velocity_scale": 0.1,
  "acceleration_scale": 0.1,
  "sequence_steps": [
    {
      "primitive_type": "LIN",
      "target_pose": { "position": {...}, "orientation": {...} },
      "blend_radius_m": 0.0,
      "velocity_scale": 0.1
    },
    {
      "primitive_type": "LIN",
      "target_pose": {...},
      "blend_radius_m": 0.008,
      "velocity_scale": 0.1
    },
    ...,
    {
      "primitive_type": "LIN",
      "target_pose": {...},
      "blend_radius_m": 0.0,
      "velocity_scale": 0.1
    }
  ]
}
```

Constraints:

- `sequence_steps` length ≥ 2 (start + end at minimum)
- First and last items have `blend_radius_m == 0.0` (start/stop must be exact)
- All intermediate items have `blend_radius_m > 0.0`
- `blend_radius_m` upper bound from SSOT key `drawing.blend_radius_m_max` (default 0.015 m)
- Each step's `primitive_type` is one of `{"LIN", "PTP", "CIRC"}` — the C++ dispatcher handles mixed sequences

Validation tests in `src/llm_gateway/tests/test_schema_validator.py`:

- Valid 3-step, 5-step, 10-step BLENDED_SEQUENCE — accept
- 1-step BLENDED_SEQUENCE — reject (degenerate, use plain LIN)
- Intermediate `blend_radius_m == 0` — reject (would produce a mid-stop)
- First or last `blend_radius_m > 0` — reject (start/stop not exact)
- `blend_radius_m` over the SSOT max — reject

### W2.T2 — Rewire drawing geometry to emit BLENDED_SEQUENCE

File: `src/llm_gateway/llm_gateway/drawing_geometry.py` (729 LOC, on the file-size exception list)

Current behaviour (verified at `drawing_geometry.py:577`): emits `{"primitive_type": "CARTESIAN_PATH", "waypoints": [...]}`.

New behaviour: emit `{"primitive_type": "BLENDED_SEQUENCE", "sequence_steps": [...]}` where each step is a LIN primitive carrying one waypoint, with `blend_radius_m = 0.0` for first/last and `blend_radius_m = drawing.blend_radius_m` (from SSOT) for intermediates.

Add SSOT keys to `src/safety/config/safety_rules.yaml`:

```yaml
drawing:
  blend_radius_m: 0.008          # default for intermediate LIN items
  blend_radius_m_max: 0.015      # hard upper bound enforced by schema
  velocity_scale_default: 0.10   # used when caller does not specify
  fallback_workplane:
    mode: "base"                 # used when /get_current_pose times out
    pose:
      position: {x: 0.30, y: 0.0, z: 0.20}
      orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
```

Backward-compatibility plan: the `CARTESIAN_PATH` primitive emit code is wrapped with a feature flag `drawing.use_blended_sequence: true` (default true after W2). When false, old behaviour. This permits one-PR rollback if a hardware corner case appears.

After two weeks of stable W2 in CI, set `drawing.use_blended_sequence` to ignored (enforced true) and mark the legacy code path `# DEPRECATED: removal_date=<W2_merge+28d>, reason=replaced_by_BLENDED_SEQUENCE_in_W2`.

### W2.T3 — Update the normalizer

File: `src/llm_gateway/llm_gateway/normalizer.py` (line 218 currently maps `CARTESIAN_PATH → PILZ_LIN`).

New entries:

```python
PLANNER_DEFAULTS = {
    ...,
    "BLENDED_SEQUENCE": "PILZ_LIN",   # outer container; per-step planners override per item
}
```

Per-step normalization: each item in `sequence_steps` is normalized as if it were a standalone LIN/PTP/CIRC primitive (reuse existing functions). The agent must NOT duplicate normalization logic — call into the existing per-primitive normalize functions for each step.

Tests in `test_normalizer.py`:

- `BLENDED_SEQUENCE` with 5 LIN steps normalizes each step's pose and emits `planner_id: PILZ_LIN`
- Mixed sequence (`PTP` approach + `LIN` segments) normalizes per-item planner IDs

### W2.T4 — Update `goal_mapper` to map BLENDED_SEQUENCE to ExecuteMotion goal

File: `src/llm_gateway/llm_gateway/goal_mapper.py`.

Currently `goal_mapper.py:61` handles `CARTESIAN_PATH` with `waypoints_msg`. Add a `BLENDED_SEQUENCE` branch that maps `sequence_steps` to the ExecuteMotion action's expected fields. The action definition lives in `src/interfaces/action/` — discovery G must produce its name. Look for fields like `sequence_step` or `motion_sequence_items`. The C++ side (`primitive_blended_sequence.cpp:825`) uses a "typed SequenceStep API" — match that shape.

If the existing ExecuteMotion action does not have a `sequence_steps` field, the agent stops and reports. Adding a field to a ROS interface is a separate decision because it touches HMI (`hmi/backend/ros/adapter.py:99-100`) and `supervisor` audit logger (`supervisor/src/audit_logger.cpp`). Discovery output drives this branch.

### W2.T5 — Consolidate `_hydrate_draw_workplane` (3 → 1)

Three sites, verified:

- `src/llm_gateway/llm_gateway/command_pipeline.py:63` — canonical
- `src/llm_gateway/llm_gateway/llm_gateway_node.py:817` — thin wrapper, OK as-is, keep
- `hmi/backend/services/intent_resolution.py:411` — local reimplementation, REMOVE

Action:

1. In `command_pipeline.py:63`, ensure the canonical `hydrate_draw_workplane` handles the workplane-unavailable case gracefully:
   - If `pose_fetcher` returns `None` (service timeout): log WARN, fall back to `mode="base"` with the pose from SSOT key `drawing.fallback_workplane.pose`. Cache the result in a module-level dict keyed by request id, so repeated calls in the same request do not re-warn.
   - On exception: re-raise (do not swallow).
2. In `hmi/backend/services/intent_resolution.py`, deprecate `_hydrate_draw_workplane` (line 411): mark with `# DEPRECATED: removal_date=<W2_merge+28d>, reason=consolidated_to_llm_gateway_via_W5`. Replace its callers (line 144) with a call to a ROS service exposed by `llm_gateway`.
   - W2 does NOT yet remove the HMI implementation. That is W5's aggressive consolidation. W2 only marks the deprecation and ensures the canonical version handles the fallback case.
   - W2 DOES remove the duplicated logic where it diverged from the canonical (i.e. if `intent_resolution.py:411` has different fallback behaviour than `command_pipeline.py:63`, align them so the deprecation is purely about call indirection, not behaviour).

The HMI service call is added in W5. For W2, the HMI keeps its local copy with the deprecation tag and a `# TODO(W5): replace with ROS service call` comment.

### W2.T6 — CIRC degenerate arc check at the gateway

File: `src/llm_gateway/llm_gateway/semantic_validator.py` (the file that already validates CARTESIAN_PATH at line 195).

Add a check for CIRC: an arc through start, auxiliary, goal is degenerate if the three points are colinear. Compute:

```python
def _is_degenerate_arc(start, aux, goal, tolerance=1e-3):
    v1 = aux - start
    v2 = goal - start
    cross = np.cross(v1, v2)
    cross_norm = np.linalg.norm(cross)
    denom = np.linalg.norm(v1) * np.linalg.norm(v2)
    if denom < 1e-9:
        return True   # zero-length segments
    return cross_norm / denom < tolerance
```

If degenerate, reject with `"degenerate CIRC: aux is colinear with start-goal (cross/|v1||v2| = X)"` at the validator, before reaching the C++ planner. Tolerance from SSOT `circ.degenerate_tolerance` (default `1e-3`).

Tests:

- 3 colinear points → reject
- 3 points forming a 90-degree arc → accept
- 3 points with the auxiliary on the arc but very close to the chord midpoint → accept (not degenerate, just shallow)

### W2.T7 — Update tests, add new ones

The existing tests at `src/llm_gateway/tests/test_draw_shape.py:45-55` and `test_draw_text.py:90` assume `CARTESIAN_PATH` emit. Update them to expect `BLENDED_SEQUENCE`. Plus new tests:

- `test_draw_circle_blended.py`: prompt → drawing pipeline emits BLENDED_SEQUENCE with N steps, intermediate blend_radii > 0, first/last == 0
- `test_workplane_fallback.py`: pose_fetcher returns None → canonical hydrate_draw_workplane logs WARN and returns base-frame fallback pose, no exception
- `test_circ_degenerate.py`: colinear input → semantic_validator rejects
- An integration test in `src/llm_gateway/tests/test_integration.py` end-to-end: NL "draw a circle radius 5cm" → BLENDED_SEQUENCE with start, intermediate, end → validates → reaches goal_mapper → maps to ExecuteMotion goal with sequence_steps populated

Hardware/RViz validation deferred to W2 acceptance review by the human.
### W2.T8 — plan_only drawing fix (per Cascade C3; conditional on discovery)

**Why:** Cascade asserts that `llm_gateway_node.py:375` rejects commands with `execution_mode: "plan_only"`, which would mean `draw_shape` with plan_only always fails. We have not verified this line in our own discovery. **This task is conditional on discovery confirming the bug.**

**Task A (mandatory) — Verify the claim:**

```bash
sed -n '360,400p' src/llm_gateway/llm_gateway/llm_gateway_node.py
rg -n -e 'plan_only|execution_mode' src/llm_gateway/llm_gateway/llm_gateway_node.py
```

Paste the relevant lines into the PR. There are three possible outcomes:

1. **Cascade was correct** (line ~375 unconditionally rejects plan_only) → proceed to Task B.
2. **Cascade was partially correct** (plan_only is rejected only for some primitive types) → proceed to Task B with narrower scope.
3. **Cascade was wrong** (plan_only already works for draw primitives) → close W2.T8 with a note in MIGRATION-W2.md and skip to W3.

**Task B (conditional) — Add a plan_only path for draw primitives:**

When `execution_mode == "plan_only"` and `primitive_type` is one of `{"draw_shape", "draw_text", "BLENDED_SEQUENCE"}`:

1. Run the full schema validation, normalize, and goal-mapper logic (same as a real submission).
2. Instead of calling `/execute_motion`, publish the compiled goal to `/llm_debug` (or whatever debug topic exists per HMI inventory from W0.T9).
3. Return a structured response: `{"status": "PLAN_ONLY_OK", "compiled_goal": {...}, "ros_topic": "/llm_debug"}`.

Do NOT publish to `/execute_motion`. The whole point of plan_only is to inspect without executing.

**Tests:**

- `test_plan_only_draw_shape.py`: prompt → execution_mode=plan_only → response carries `PLAN_ONLY_OK` and the compiled BLENDED_SEQUENCE goal; `/execute_motion` is never called.
- `test_plan_only_does_not_execute.py`: same prompt twice; verify the action client mock is never invoked.

**No-conflict guarantee:** plan_only is a new branch in the dispatch logic. The existing `execution_mode != plan_only` path is unchanged. HMI is not affected unless it specifically uses plan_only (per W0.T9 inventory).
---

## Verification

| # | Check | Pass criterion |
|---|---|---|
| 1 | `rg -n '"primitive_type": "CARTESIAN_PATH"' src/llm_gateway/llm_gateway/drawing_geometry.py` | 0 hits (or 1 hit inside a `# DEPRECATED` block, with `drawing.use_blended_sequence == False` legacy path) |
| 2 | Schema validator: BLENDED_SEQUENCE with 5 LIN steps | Accepted |
| 3 | Schema validator: 1-step BLENDED_SEQUENCE | Rejected with `"sequence_steps must contain at least 2 items"` |
| 4 | NL prompt "draw a circle of radius 0.05 m" | Pipeline produces BLENDED_SEQUENCE primitive; sim execution shows continuous motion (no per-segment stop) |
| 5 | `pose_fetcher` mock returns `None` | `hydrate_draw_workplane` logs WARN, returns base-frame pose, no crash |
| 6 | CIRC with auxiliary at midpoint of start-goal segment | Rejected by semantic_validator with `"degenerate CIRC"` |
| 7 | Per-step LIN failure inside BLENDED_SEQUENCE (mock) | C++ dispatcher reports the failing step index; no silent fallback (W1 invariant holds) |
| 8 | `colcon test --packages-select llm_gateway primitives motion_core` | Green; updated tests pass; new tests pass |
| 9 | CI `safety-chain` | Green |
| 10 | CI `no-fallback-guard` | Green |
| 11 | CI `duplication` (jscpd) | No new duplications introduced; if `intent_resolution.py:411` is still duplicated, it is below threshold or has the DEPRECATED tag |
| 12 | RViz preview of a 5 cm circle | Smooth trajectory, all wrist joints within W1's `operational_joint_limits` |

---

## DON'T

- Do not remove `CARTESIAN_PATH` primitive support from `primitive_router_dispatch.cpp:716` or from `primitive_lin.cpp`. The primitive remains valid for non-drawing callers. W2 only stops drawing from emitting it.
- Do not delete `hmi/backend/services/intent_resolution.py:411` in this wave. Mark deprecated; W5 deletes after the ROS service replacement is in.
- Do not change the ExecuteMotion action interface (`src/interfaces/action/`) without an explicit interface-change PR with HMI sign-off. If the action lacks `sequence_steps`, stop and report.
- Do not pick a non-zero `blend_radius_m` for the first or last sequence step. Start and end must be exact poses; blending only on intermediates.
- Do not implement a "smart" fallback inside hydrate_draw_workplane that tries multiple fetchers. One pose_fetcher; if it fails, base-frame fallback. Keep it boring.
- Do not bundle file-size splits with this wave. `draw_router.py` (889 LOC) and `drawing_geometry.py` (729 LOC) stay god-files until W6. The exception list in W0 already permits this.
- Do not run on hardware before sim verification 4, 5, 6 are green.

---

## Output artefacts

- `src/llm_gateway/llm_gateway/drawing_geometry.py` — diff: emit BLENDED_SEQUENCE; legacy CARTESIAN_PATH path behind feature flag
- `src/llm_gateway/llm_gateway/normalizer.py` — diff: planner mapping for BLENDED_SEQUENCE; per-step normalization
- `src/llm_gateway/llm_gateway/goal_mapper.py` — diff: BLENDED_SEQUENCE branch
- `src/llm_gateway/llm_gateway/semantic_validator.py` — diff: CIRC degenerate check
- `src/llm_gateway/llm_gateway/command_pipeline.py` — diff: hydrate_draw_workplane fallback handling
- `src/llm_gateway/llm_gateway/schemas/<name>.json` — diff: BLENDED_SEQUENCE schema entry
- `hmi/backend/services/intent_resolution.py` — diff: DEPRECATED tag on local _hydrate_draw_workplane
- `src/safety/config/safety_rules.yaml` — diff: `drawing.*` SSOT keys, `circ.degenerate_tolerance`
- `src/llm_gateway/tests/test_schema_validator.py` — diff: BLENDED_SEQUENCE cases
- `src/llm_gateway/tests/test_normalizer.py` — diff: BLENDED_SEQUENCE cases
- `src/llm_gateway/tests/test_draw_shape.py` — diff: expect BLENDED_SEQUENCE
- `src/llm_gateway/tests/test_draw_text.py` — diff: expect BLENDED_SEQUENCE
- `src/llm_gateway/tests/test_draw_circle_blended.py` — new
- `src/llm_gateway/tests/test_workplane_fallback.py` — new
- `src/llm_gateway/tests/test_circ_degenerate.py` — new
- `src/llm_gateway/tests/test_integration.py` — diff: end-to-end BLENDED_SEQUENCE
- `MIGRATION-W2.md`

---

## Rollback procedure

Two-stage rollback because W2 changes the schema:

```bash
# Stage 1: feature flag (zero-downtime)
# Edit safety_rules.yaml: drawing.use_blended_sequence: false
# All drawing emits revert to CARTESIAN_PATH; W1's no-fallback-guard still active.
# This works for any installation that's already pulled W2 schema additions.

# Stage 2: revert PR (downtime — schema regresses)
git revert -m 1 <W2 merge commit>
# HMI must also revert any updates that consumed BLENDED_SEQUENCE.
# Coordinate with W5 if W5 has already shipped.
```

---

## Risk notes

- **Per-step LIN failure inside a long sequence:** the C++ dispatcher's behaviour on intermediate LIN failure determines whether the whole sequence fails or partial progress is allowed. This is a property of `primitive_blended_sequence.cpp` and not modifiable in W2. The agent must read that file's behaviour and document it in `MIGRATION-W2.md` so operators know what to expect.
- **Blend radius vs collision distance:** earlier reviews flagged that a `blend_radius` larger than the distance to a `CollisionObject` produces an invisible collision. In W2, drawing happens above a workplane with no scene obstacles — low risk. After W4 introduces `CollisionObject`s from perception, W4 (or a follow-up) must add a blend-radius-vs-clearance check.
- **HMI compatibility window:** until W5 lands, HMI's local `_hydrate_draw_workplane` runs alongside the canonical version. They must produce identical outputs for the same input. W2.T5 step (3) handles this; verify with a property-based test if behavioural divergence is suspected.
- **Sim ≠ hardware:** W2 verification is sim. Hardware drawing test happens after operator approval, with `velocity_scale ≤ 0.1`.

---

## Stop signal

End of W2. Do not proceed to W3 until:

- W2 PR merged.
- A sim drawing of a circle is visible in RViz with continuous motion (operator-recorded video or screenshot in the PR).
- HMI integration tests still pass (HMI is not yet calling the canonical workplane via ROS — that is W5 — but the deprecation tag is in place).

State explicitly: `End of W2. Awaiting review before W3.`

---

**Reliability tag:** `[VERIFIED]` for the file targets — `drawing_geometry.py:577`, `normalizer.py:218`, `command_pipeline.py:63`, `intent_resolution.py:411`, `semantic_validator.py:194-195` all confirmed in discovery. `[NEEDS-VALIDATION]` for W2.T4 (depends on the ExecuteMotion action's actual fields, which the agent must read at the start of the wave). `[NEEDS-VALIDATION]` for the per-step failure semantics described in the risk notes — that is the C++ dispatcher's existing behaviour, the agent reads and documents but does not change.
