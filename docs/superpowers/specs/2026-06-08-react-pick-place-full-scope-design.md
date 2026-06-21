# ReAct Pick/Place Full Scope Upgrade - Design Spec

- Date: 2026-06-08
- Branch: `upgrade-react-8626`
- Source plan: `/home/hieu2/Documents/super-react-plan-8626.md`
- Scope: `llm_gateway`, `safety`, `motion_core`/MoveIt integration points, gripper I/O adapter path, tests, docs
- Status: Implementation in progress; FactoryTask is now the LLM-facing output contract.

## 1. Purpose

The ReAct gateway exposes primitive motion and I/O intents, and the current upgrade adds semantic station grounding, a FactoryTask task-tree contract, scene-aware compilation, composite pick/place behavior, scene cache, gripper verification, optional MTC handoff, and closed-loop postcondition checks while preserving the existing fail-closed robot-control boundary.

The final system must let an operator express pick/place goals in natural language or a strict goal DSL, resolve named station regions and objects deterministically, generate safe candidate poses, execute composite pick/place through validated Semantic IR or an MTC-backed service path, verify world-state changes, and stop safely when any required runtime fact is missing.

## 2. Scope

This design covers all phases from the source plan:

1. Preflight inventory and runtime verification hooks.
2. Semantic station map and scene graph resolver.
3. Candidate pose generator and spatial resolver.
4. Skill library and task compiler.
5. Composite ReAct tools and scene snapshot cache.
6. Real gripper adapter with feedback gating.
7. Optional MTC pick/place server integration.
8. Closed-loop ReAct verification, cleanup decisions, tests, and docs.

Implementation should happen in waves, but the accepted target is the full Phase 1-8 end state. Early waves are not a reduced product definition.

## 3. Non-Goals

- No direct LLM-to-motion or direct LLM-to-I/O path.
- No hardcoded station coordinates outside configuration.
- No unverified hardware I/O execution. `VERIFY_CONFIG` is a required sentinel and must fail closed at runtime.
- No manual IK solver. Use existing planning and MoveIt capabilities.
- No VLM/VLA end-to-end controller in this phase.
- No deletion of drawing-related files in the behavior PR unless usage evidence proves they are unused and tests protect the removal.
- No widening of workspace, joint, velocity, acceleration, manipulability, or forbidden-zone limits without an explicit safety review.

## 4. Architecture

The edge architecture stays unchanged: LLM output is constrained to FactoryTask JSON, and motion still flows through Semantic IR, validation, planning, and supervised execution.

```text
operator text / goal DSL
  -> ReAct agent
  -> FactoryTask task tree
  -> TaskCompiler / TaskRuntime policy checks
  -> StationSceneGraph resolver
  -> candidate pose generator in react_planner.py
  -> composite tools / optional MTC client
  -> Semantic IR or validated MTC request
  -> validate_semantic_ir_contract and /validate_command
  -> motion_core / MoveIt / supervisor / hw_adapter
  -> scene refresh and postcondition verification
```

The semantic station map is separate from `src/safety/config/safety_rules.yaml`. Safety remains the authority for workspace bounds, forbidden zones, motion caps, joint limits, and guard behavior. The station map provides names, aliases, zones, object classes, and semantic geometry hints; it cannot authorize motion by itself.

## 5. Components

### 5.1 Preflight Inventory

Before each implementation wave, run the repo-required harness: current branch, `colcon list`, and `git status --short`. Before editing any symbol, run GitNexus impact analysis for that symbol. Before hardware-dependent work, record ROS graph availability with `ros2 node list`, `ros2 topic list`, `ros2 service list`, and `ros2 action list` when a runtime is active.

For tool offsets, verify the active TCP frame by comparing `tf2_echo base_link tool0` with `tf2_echo tool0 <ACTIVE_TCP_FRAME>`. If the active TCP cannot be verified, pose generation must not apply a guessed 0.12 m offset.

### 5.2 Station Semantic Map

Create `src/llm_gateway/config/station_semantic_map.yaml`. It contains station regions, zones, aliases, and object class mappings. Unknown measured values use the exact sentinel `VERIFY_CONFIG`; code must load the file but reject runtime planning for any geometry that still contains that sentinel.

Required initial concepts:

- Regions: `conveyor`, `fixture`, `drop_zone`, `inspection_zone` when available from station data.
- Objects: `white_workpiece` with English and Vietnamese/operator aliases.
- Geometry: frame id, box center/size or zone offset, default clearance, approach axis.
- Audit metadata: source, reviewed date, and whether geometry has been runtime verified.

This file must not be merged into `safety_rules.yaml`.

### 5.3 StationSceneGraph and Resolver

Add an internal `StationSceneGraph` in `intent_engine.py` unless file size forces a narrowly named helper module. It loads the semantic map and exposes:

- `resolve_region(name)` -> exact region or `needs_clarification`.
- `resolve_zone(region, zone)` -> exact zone or `needs_clarification`.
- `resolve_object(query)` -> object class/aliases and latest perception match or `needs_clarification`.
- `nearest_free_cell(region, object_size)` -> candidate semantic cell or `capability_unavailable` when occupancy data is missing.

Resolver behavior is strict. It never silently maps an unknown phrase to a default region. Ambiguous aliases return a clarification result with candidate names.

### 5.4 Candidate Pose Generator

Add `_generate_candidate_poses` in `react_planner.py` or a small composite helper if the file split is required. It is pure: input scene node, purpose, current pose, safety rules, and optional perception detections; output ranked candidate poses with rejection reasons.

Responsibilities:

- Select center, zone offset, or nearest free cell for approach, grasp, drop, inspect, retreat.
- Apply the active TCP offset exactly once through frame transforms, not repeated z-axis subtraction.
- Reject candidates outside `workspace_bounds` or inside `forbidden_zones`.
- Send candidate IR through existing plan validation where available instead of implementing IK manually.
- Score candidates by safety validity, clearance, distance, and alignment.

If no candidate survives, return `needs_clarification` or `capability_unavailable`; do not emit motion.

### 5.5 Skill Library and Task Compiler

Define a small skill registry in `intent_engine.py` for high-level robot tasks:

- `move_to_region`
- `approach_object`
- `pick_object`
- `place_object`
- `retreat`
- `set_gripper`
- `verify_grasp`
- `refresh_scene`
- `verify_postcondition`
- `go_home`

Add `compile_goal(goal_dsl) -> list[SkillCall]`. It accepts strict goal DSL such as `pick_and_place` with `object` and `destination`. It checks preconditions, required capabilities, and ambiguity before emitting skill calls. The LLM should use this DSL and skill library rather than expanding the frozen primitive intent set with arbitrary new intents.

### 5.6 Composite ReAct Tools

Add composite tools that emit existing Semantic IR, usually `sequence`, and validate with `validate_semantic_ir_contract` before returning:

- `approach_object`: resolve object and emit pre-grasp `absolute_move_lin`.
- `retreat`: emit reverse `absolute_move_lin` or `move_relative` using allowed axis semantics.
- `pick_object`: approach -> open gripper -> descend -> close gripper -> verify grasp -> lift.
- `place_object`: approach destination -> descend -> open gripper -> detach/verify -> retreat.
- `emit_sequence`: wrap child Semantic IR into one validated sequence.
- `refresh_scene`: invalidate scene cache and mark next perception query fresh.

Composite actions count as one ReAct motion iteration. Internal child steps are validated as sequence steps and still pass through the existing safety and supervisor path.

### 5.7 Scene Snapshot Cache

Add `_SceneSnapshotCache` to `llm_gateway_node.py` or a tiny helper module. Default TTL is 2.0 s. Key by `(class_filter, frame)`. `QueryPerceptionTool.invoke` checks the cache before calling `/perception/get_object_positions` and returns `payload.cache_hit=True` on reuse.

Invalidate the cache when:

- TTL expires.
- `refresh_scene` is called.
- Motion succeeds with `metadata.tool_changed_world=True`.
- Robot state leaves `IDLE`.

Perception remains blocked during motion. Cache hits must not bypass calibration validity, depth range, or stale-data checks already returned by perception.

### 5.8 Gripper Adapter and Feedback

Replace the current `gripper_open` and `gripper_close` stubs only after I/O addresses are verified. Configuration fields for output address, output values, feedback input address, active polarity, and timeout start as `VERIFY_CONFIG`. Runtime invocation fails closed while any required value is unresolved.

The adapter uses MotoROS2 `WriteSingleIO` and `ReadSingleIO` services if present. It must verify the robot is `IDLE` before sending I/O. `verify_grasp` waits for feedback within a bounded timeout. The planning scene attaches an object only after feedback confirms a grasp; detach follows confirmed open/place behavior.

### 5.9 MTC Pick/Place Path

Add an optional MTC-backed pick/place server in the MoveIt-facing package only when dependencies are available. It stages approach, open, descend, close, attach, lift, move, descend, open, detach, and retreat. It does not replace simple primitives; PTP, LIN, MOVE_REL, and CIRC remain valid standalone paths.

Composite tools may choose MTC when all required inputs are verified: object pose, destination region, gripper config, planning scene, collision model, and MTC service availability. If MTC is unavailable, the system may use the validated primitive sequence path. If both are unavailable, it returns `capability_unavailable`.

### 5.10 Closed-Loop ReAct

World-changing actions trigger `refresh_scene` and `verify_postcondition`. Pick verifies object attached/grasped or absent from the original table pose. Place verifies object appears in the destination region and is detached. Replan budget defaults to one repair attempt. If verification fails after the repair budget, halt the sequence and return a structured fail-closed error.

Iteration accounting:

- A composite pick/place call counts as one motion iteration.
- A repair attempt counts against `max_repair`.
- Read-only verification counts against the read-only budget.
- No loop may continue after the configured total or wall-clock budget.

## 6. Error Handling

All failures are structured and observable:

- `needs_clarification`: unresolved or ambiguous region/object/zone.
- `verify_config_required`: config contains `VERIFY_CONFIG` for a runtime-required field.
- `capability_unavailable`: gripper, MTC, perception, or occupancy function missing.
- `safety_rejected`: workspace, forbidden-zone, joint, motion-limit, or validator rejection.
- `postcondition_failed`: scene verification did not confirm the requested world state.
- `runtime_unavailable`: required ROS service/action/TF is missing.

Every safety rejection logs the blocked capability, input summary, and reason. Logs must not include secrets or raw prompt data beyond the minimal command summary needed for audit.

## 7. Testing Strategy

Tests encode intent and safety rules, not only field shape.

Required tests:

- Semantic map loader accepts valid map and rejects runtime use of `VERIFY_CONFIG` geometry.
- Resolver returns exact matches, aliases, ambiguity, and `needs_clarification`.
- Candidate generator applies TCP offset once and rejects workspace/forbidden-zone candidates.
- Composite tools emit expected Semantic IR and validate it through `validate_semantic_ir_contract`.
- Scene cache covers hit, TTL expiry, refresh invalidation, motion-state invalidation, and world-change invalidation.
- Gripper adapter fails closed on unresolved config, non-IDLE state, missing services, timeout, and feedback mismatch.
- MTC selection chooses MTC only when dependencies are available and falls back or fails closed otherwise.
- Closed-loop verifier retries once and halts when postcondition remains false.
- Integration sim test for `white_workpiece` pick/place completes within the ReAct iteration budget with cache hit rate at least 50% when repeated perception queries occur.
- Existing contract, ReAct, safety, and motion tests continue passing.

Repo-required verification remains: `colcon build --symlink-install`, `colcon test`, `colcon test-result --verbose`, and `git status --short` after implementation waves.

## 8. Documentation and Cleanup

Update README and LLM gateway rules/docs to describe:

- Station semantic map ownership.
- `VERIFY_CONFIG` fail-closed behavior.
- Composite pick/place commands.
- Scene cache behavior.
- Gripper verification requirements.
- MTC optional path and primitive fallback.
- Closed-loop postcondition and repair limits.

Cleanup of `drawing_geometry.py` and `macro_policy.yaml` is a separate evidence-based cleanup wave. Do not delete them in the behavior wave unless call graph and tests prove they are unused and the user approves the cleanup scope.

## 9. Implementation Waves

Wave 0: Preflight inventory, symbol impact analysis, runtime interface notes, and exact current-state checklist.

Wave 1: Semantic map, `StationSceneGraph`, strict resolver, and tests.

Wave 2: Candidate pose generator, TCP-offset handling, safety filtering, and tests.

Wave 3: Skill registry, goal compiler, composite tools, sequence validation, and iteration-budget accounting.

Wave 4: Scene snapshot cache and perception-query integration.

Wave 5: Gripper adapter configuration, I/O service integration, feedback verification, and fail-closed tests.

Wave 6: Optional MTC server/client path, selection policy, primitive fallback, and tests.

Wave 7: Closed-loop refresh, postcondition verification, repair budget, and integration sim test.

Wave 8: Documentation, cleanup audit, full build/test/report.

Each wave must be independently reviewable and must preserve unrelated dirty worktree changes.

## 10. Acceptance Criteria

The full objective is complete only when all of these are proven in the current state:

1. `station_semantic_map.yaml` exists and is loaded without coupling to `safety_rules.yaml`.
2. Strict region/object/zone resolution is covered by tests.
3. Candidate poses are generated, safety-filtered, and validated without manual IK.
4. Composite ReAct tools are registered and emit contract-valid Semantic IR.
5. Scene cache returns observable cache hits and invalidates on all required triggers.
6. Gripper open/close/verify fail closed until real config is verified, and use MotoROS2 I/O when verified.
7. MTC pick/place path is available when dependencies exist, with primitive fallback or structured failure when not.
8. Closed-loop postcondition verification refreshes perception, retries once, and halts safely on failure.
9. Pick/place sequence consumes one motion iteration while repairs consume repair budget.
10. Unit, contract, and integration tests cover the safety-critical behavior.
11. README and gateway docs describe the new semantic map, composites, gripper, MTC, and closed-loop behavior.
12. `colcon build --symlink-install`, `colcon test`, and `colcon test-result --verbose` pass for the implementation state.

## 11. Risks and Controls

- Unknown geometry: use `VERIFY_CONFIG`, reject runtime planning until measured.
- Unknown gripper I/O: fail closed until `WriteSingleIO`/`ReadSingleIO` service names, addresses, values, and polarity are verified.
- TCP offset double-application: centralize offset handling in the candidate generator and test it directly.
- File growth: keep new helpers narrow; split only the code touched by this plan.
- MTC dependency drift: gate MTC with dependency/service checks and retain primitive fallback.
- Hardware safety: default to simulation/plan-only verification before hardware, preserve human approval and supervisor gates.

## 12. User Review Gate

After this spec is committed, the next permitted step is user review. Only after written-spec approval should the workflow invoke `superpowers:writing-plans` to produce the implementation plan.
