# ReAct + Vision Composite Uplift — Design Spec

- **Date**: 2026-05-20
- **Branch**: `ws-deep-rebuild-3526`
- **Scope**: `llm_gateway`, `gp4_perception`, `safety`, integration tests
- **Status**: Draft (awaiting user review)

## 1. Background

A planning report (2026-05-20) proposed a sweeping rework of ReAct + D435i
integration. After auditing the current codebase, most of the report's
"missing items" are already implemented:

| Report claim | Repo reality |
|---|---|
| CIRC primitive missing | `src/primitives/src/primitive_circ.cpp` (PILZ CIRC) |
| Blended sequence missing | `primitive_blended_sequence.cpp` + Semantic IR `sequence` intent |
| Approach/Retract missing | `primitive_approach.cpp`, `primitive_retract.cpp` |
| Direct-command regex still hot | Narrowed to protective stop only; covered by `test_direct_review_regex.py` |
| ReAct Tool Registry missing | `react_planner.py` has 11 Tool classes (Query/Plan/Submit Motion, ComputeArc, Gripper*, GetPose, SetSpeed, WaitForState) |
| No ReAct timeout / loop counter | `IterationBudget` tiered (max_total=5, max_motion=3, max_readonly=10, max_repair=1, wall_clock_timeout_s=30) |
| D435i as VLM | Deterministic `gp4_perception` pkg with `scene_processor`, `query_perception_tool`, `calibration`, `scene_geometry`, `safety_guards` |
| No plan/pose cache | `_react_plan_cache` and `_get_cached_current_pose_snapshot` already wired |

The genuine gaps are scoped below; the rest of the report is treated as
already-completed work and is **out of scope**.

## 2. Goals

1. Let ReAct execute industrial pick/place flows in **≤ 5 ReAct iterations**
   by exposing composite tools that wrap the existing primitive backend.
2. Cut perception service round-trips per ReAct loop via a **scene snapshot
   cache** with explicit invalidation.
3. Keep the **fail-closed safety pipeline** intact: every motion still flows
   `LLM → Semantic IR → /validate_command → motion_core → hw_adapter`. No
   bypass, no new direct paths.
4. Surface the existing `sequence` IR to ReAct so multi-step plans can be
   committed in a single ToolCall when the LLM has the full picture.
5. Split the two oversized files (`llm_gateway_node.py` 2350 L,
   `react_planner.py` 2171 L) only **where the new code lands**, following
   the global 800-line guideline. No unrelated refactoring.

## 3. Non-Goals

- No VLM, no CLIP/GPT-4V, no raw images to LLM.
- No new ROS2 package or top-level folder.
- No changes to MotoROS2, MoveIt2 config, or hw_adapter wire format.
- No LLM fine-tuning dataset work (separate effort).
- No re-implementation of CIRC / blended / approach / retract primitives.
- No widening of safety bounds, joint limits, or velocity caps.
- No removal of the protective-stop regex shortcut (intentional).

## 4. Architecture (unchanged at the edges)

```
User → HMI/CLI → llm_gateway (ReAct + Semantic IR)
                     │
                     ├── Tools (read-only): get_current_pose, query_perception,
                     │                       wait_for_state, set_speed
                     │
                     ├── Tools (motion):    plan_motion, submit_motion,
                     │                       gripper_open/close, compute_arc_points
                     │
                     └── Tools (composite NEW): approach_object, retreat,
                                                 pick_object, place_object,
                                                 emit_sequence, refresh_scene
                          │
                          └── all composites emit Semantic IR; nothing skips
                              validate_command → motion_core → hw_adapter
```

Composite tools build Semantic IR (often `sequence` IR) and route it through
the same validate→plan→submit path the existing tools already use. They do
not call MoveIt2 or the hardware action server directly.

## 5. Detailed Design

### 5.1 Scene snapshot cache (`gp4_perception` + `llm_gateway`)

**Component**: `SceneSnapshotCache` inside `LLMGatewayNode` (no new file
unless `llm_gateway_node.py` is split).

- TTL: configurable, default `2.0 s`.
- Invalidation triggers (any of):
  - TTL expiry.
  - `refresh_scene` tool invoked.
  - `submit_motion` returned success with `tool_changed_world=True`
    (set by composites that move a grasped object).
  - Robot mode transitioned out of `IDLE` (we already block perception
    during motion at [react_planner.py:1575-1580](../../src/llm_gateway/llm_gateway/react_planner.py#L1575)).
- Cache key: `(class_filter, frame)` so per-cycle queries reuse results.
- On hit, `QueryPerceptionTool.invoke` returns the cached payload with
  `payload.cache_hit=True` for observability.

**Why TTL=2 s**: detector runs ~10 Hz; 2 s gives multiple frames of
freshness but stays well within a ReAct turn. Tunable via gateway param
`react.scene_cache_ttl_s`.

### 5.2 Composite ReAct tools (in `react_planner.py`)

All composites:
- Are marked `is_motion = True`, so they consume the `max_motion` budget
  (3 by default). Each composite counts as **one** motion iteration.
- Emit a single Semantic IR object (often `sequence`) and call the existing
  `plan_motion` + `submit_motion` flow internally, OR return the IR for
  the gateway to submit. Choice locked in §5.7.
- Validate inputs against `safety.config` thresholds before emitting IR.

| Tool | Inputs | Behavior |
|------|--------|----------|
| `approach_object` | `object_id` or `target_pose`, `offset_m` (default 0.05, capped by safety), `approach_axis` (default `-z_tool`) | Resolves object pose from cache (no extra perception call), emits LIN to `target + offset·axis` |
| `retreat` | `offset_m`, `axis` | Emits LIN back along axis by `offset_m` from current pose |
| `pick_object` | `object_id`, `approach_offset_m`, `grasp_descent_m`, `lift_m` | Emits `sequence` IR: approach → open_gripper → LIN descent → close_gripper → LIN lift |
| `place_object` | `target_pose` or `object_id`, `approach_offset_m`, `descent_m` | Emits `sequence` IR: approach → LIN descent → open_gripper → retreat |
| `emit_sequence` | `steps: [SemanticIR, ...]` | Pass-through helper: assembles a `sequence` IR for ReAct, validated against `validate_semantic_ir_contract` before submission |
| `refresh_scene` | none | Invalidates `SceneSnapshotCache` and triggers a fresh perception call on next `query_perception` |

`PickObjectTool` and `PlaceObjectTool` set `tool_changed_world=True` on
success so the cache is invalidated automatically.

### 5.3 Safety policy additions (`src/safety/config/safety_rules.yaml`)

Add (do not modify existing keys):

```yaml
composite_limits:
  max_sequence_length: 8           # steps per sequence IR from ReAct
  max_pick_approach_offset_m: 0.12 # caps approach_object/pick_object offset
  pick_descent_max_m: 0.06         # caps grasp_descent_m
  pick_lift_max_m: 0.10            # caps post-grasp lift
  place_descent_max_m: 0.06
  approach_axis_whitelist: ["-z_tool", "-z_base"]
```

`safety.command_validator` enforces these when it sees the composite IR.
Limits are deliberately tight; widening requires a new safety review per
[safety-first.md](../../.claude/rules/safety-first.md).

### 5.4 Semantic IR contract changes

Minimal addition: composites either emit `sequence` IR (already accepted)
or one of the existing single intents. **No new top-level intent.**
`validate_semantic_ir_contract` already recurses into `sequence.steps` —
that path stays unchanged.

What *is* added: an optional `metadata.source` field (`"composite_pick"`,
`"composite_place"`, etc.) for audit/observability only. The validator
ignores unknown metadata, so this is backward-compatible.

### 5.5 File splits (only where new code lands)

- `react_planner.py` (2171 L) → split tools into `react_planner/tools/`:
  - Move existing tool classes into per-tool modules
    (`motion.py`, `perception.py`, `gripper.py`, `state.py`, `composite.py`).
  - Keep `react_planner.py` as the orchestrator (`ReActAgent`,
    `IterationBudget`, `StateInjector`, `ToolRegistry`).
  - Target: each module ≤ 600 L; orchestrator ≤ 800 L.
- `llm_gateway_node.py` (2350 L) → extract perception-cache + composite
  IR helpers into `llm_gateway/composite_helpers.py`. Node file should
  drop below 1800 L after extraction; remaining size reduction is a
  separate effort and **out of scope**.

### 5.6 Tests (mandatory, before merge)

- **Unit**: each composite tool emits the expected `sequence` IR for
  representative inputs; safety-cap violations rejected with structured
  error.
- **Unit**: `SceneSnapshotCache` TTL expiry, manual invalidation, motion-
  mode invalidation, and `tool_changed_world` invalidation.
- **Contract**: existing `test_direct_review_regex.py` keeps passing.
  Add `test_composite_ir_contract.py` validating composite-emitted IR
  against `validate_semantic_ir_contract`.
- **Integration (sim)**: `test_pick_red_block_sim.py` runs
  `gp4_bringup sim.launch.py`, drives a fake perception detection of a
  red block, asserts ReAct completes the pick in ≤ 5 iterations.
- **Regression**: `colcon test --packages-select llm_gateway safety motion_core`
  stays green.

### 5.7 Internal vs. external composite execution

Composites **return Semantic IR** to the gateway, which then calls the
existing `plan_motion` + `submit_motion` path. They do not invoke
MoveIt2 or hw_adapter directly. Rationale:

- Keeps the audit log (one IR per user request, with composite metadata).
- Avoids duplicating the supervisor / quality-gate plumbing inside the
  tool layer.
- Lets the same validator catch composite mis-builds before any motion.

The downside (one extra hop) is negligible relative to perception/IK costs.

## 6. Error Handling & Fail-Closed Behavior

- If `query_perception` returns no match for `object_id`: composite tool
  fails with `object_not_found`, ReAct receives `ToolResult(ok=False)`,
  no motion emitted.
- If safety-cap violated: composite tool fails with
  `safety_cap_violation: <field>=<value> exceeds <limit>`, no motion.
- If `validate_command` rejects the emitted sequence IR: gateway returns
  `ReviewIntent` rejection with the validator's reason, ReAct gets the
  error and can repair (up to `max_repair=1`).
- If `submit_motion` fails mid-sequence: existing supervisor/quality-gate
  abort path applies. No additional fault tolerance in the composite
  layer — fail closed.

## 7. Observability

- Audit log line per composite invocation: tool name, resolved IR,
  cache_hit flag, safety check result.
- HMI System Log already renders Semantic IR + validation reason; the new
  `metadata.source` makes composite-driven motion distinguishable from
  user-issued single primitives.

## 8. Migration / Rollout

- Behind a gateway param `react.composite_tools_enabled` (default
  `true` in sim, `false` for first hardware run).
- Existing single-primitive flows untouched — pure additive feature set.
- README "primitives" section gets a "Composite ReAct flows" subsection;
  no contract changes to the public primitive list.

## 9. Out-of-Scope / Rejected from Report

The following report items are **rejected** as either already implemented
or unnecessary:

- Re-implementing CIRC, blended, approach/retract primitives.
- Removing all direct regex paths (protective stop must stay direct).
- Adding `ComputeArcPointsTool` (exists).
- Building a new gp4_perception package (exists).
- Adding ReAct loop timeout (`IterationBudget` already enforces it).
- LLM fine-tuning dataset work.
- Behavior-tree task planner.
- Switching TOTG → Ruckig (no evidence current TOTG is the bottleneck).
- Consolidating `get_current_pose` across packages (not actually duplicated;
  only present in `llm_gateway_node.py`).

## 10. Acceptance Criteria

1. Composite tools registered in `ToolRegistry`; `colcon build` green.
2. Pick/place sim test completes in ≤ 5 ReAct iterations.
3. Perception cache hit rate ≥ 50% in a 5-iteration pick flow (measured
   via audit log).
4. All existing unit/integration tests still green.
5. Safety caps prevent composite IR exceeding configured limits (covered
   by unit tests with explicit rejected cases).
6. No file exceeding 800 lines in the new code paths (split achieved for
   `react_planner.py`; `llm_gateway_node.py` partial reduction acceptable).
7. README and `.claude/rules/llm-gateway.md` updated to reflect composite
   tool surface.

## 11. Open Questions

- (Resolved §5.7) Composite execution model.
- (Resolved §5.1) Cache TTL default — 2.0 s, gateway-param tunable.
- None outstanding pre-implementation.
