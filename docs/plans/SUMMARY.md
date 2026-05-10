# SUMMARY — gp4_ws Rebuild Plan v3

**Audience:** human reviewer + AI coding agent
**Date:** 2026-05-03
**Working branch (after W0):** `ws-deep-rebuild-3526`
**Prior branch:** `chore/workspace-deep-clean-2026-04-26` (renamed in W0)
**Total estimated effort:** 25–32 working days, executed wave-by-wave

---

## Decisions locked in by the human

| ID | Decision | Implication |
|---|---|---|
| D1 | **Review and rebuild perception fresh** | W0 reviews commit `36520035` for reference only. W4 writes new code, does NOT cherry-pick. The existing 6 perception files (calibration_recorder, fiducial_detector, vision_pick_planner, etc.) are read as inspiration, not as the implementation. |
| D2 | **Rename branch + cull others** | W0 renames `chore/workspace-deep-clean-2026-04-26` → `ws-deep-rebuild-3526`. Keeps `main`, `super-fix`, `hmi-pro`. Deletes `rebuild-again-v2`, `rebuild-core-v2`, plus stale remotes after backup tags. |
| D3 | **HMI consolidation = aggressive** | W5 rewrites `hmi/backend/services/intent_*` and `supervisor_*` to call ROS services on the `llm_gateway` / `safety` / `supervisor` nodes instead of reimplementing logic locally. HMI frontend tests will need updates. |
| D4 | **One markdown file per wave, technical and detailed** | Each wave 2–4 pages: Goal · Discovery · Tasks · Verify · DON'T · Output · Rollback. AI agent reads ONE file, executes ONE wave, stops, awaits review. |
| D5 | **Short summary up front** | This document. |

---

## Verified facts about the current codebase (from discovery 2026-05-03)

These are the only facts I am willing to build wave plans on. Anything not on this list = the agent must rediscover at wave start.

| Area | Verified fact | Evidence |
|---|---|---|
| Packages | 12 ROS packages: `gp4_bringup`, `gp4_moveit_config`, `gp4_station`, `hw_adapter`, `interfaces`, `jog_pendant`, `llm_gateway`, `motion_core`, `motoros2_client_interface_dependencies`, `primitives`, `safety`, `supervisor` | `find src/ -maxdepth 2 -name package.xml` |
| Drawing pipeline | Drawing emits `CARTESIAN_PATH`, normalizes to `PILZ_LIN`, dispatches via `primitive_router_dispatch.cpp:716` calling `computeCartesianPath`. `BLENDED_SEQUENCE` primitive exists with `MotionSequenceRequest` but is **unused** by drawing | `drawing_geometry.py:577`, `normalizer.py:218`, `primitive_blended_sequence.cpp:439-445`, `primitive_dispatcher.cpp:71` |
| Silent fallback | LIN primitive on Pilz failure silently falls back to `computeCartesianPath` | `primitive_router_dispatch.cpp:857-895` |
| Velocity scale | `kDefaultVelocityScaling = 0.06` is intentional separation of concerns, NOT a bypass | `trajectory_post_processor.hpp:16`, `command_validator.py:62` (comment) |
| Joint guard | `WristFlipGuard` only checks per-step deltas (max 30° between consecutive points) and sign flips. **No absolute joint position guard exists.** | `wrist_flip_guard.cpp`, `quality_gate.cpp:67`, `test_wrist_flip_guard.cpp:40` |
| LLM | Uses `jsonschema` (not Pydantic). `IntentRouter` is a router, not a classifier. `temperature=0.0` is hardcoded. Both `/llm_intent` and `/llm_raw_command` topics already exist. | `setup.py:17`, `llm_config.py:177`, `llm_gateway_node.py:105,119` |
| Pilz pipeline | Already configured in MoveIt | `gp4_moveit_config/config/pilz_industrial_motion_planner_planning.yaml` |
| Bloat evidence | `_hydrate_draw_workplane` defined in 3 places; `intent_*` and `supervisor_*` modules in HMI mirror src/ Python code | `intent_resolution.py:411`, `command_pipeline.py:63`, `llm_gateway_node.py:817` |
| Vision history | A perception package was committed at `36520035` on `rebuild-again-v2`, marked "not-finished-rebuild". 13 source files, 4 launch files, 4 config YAMLs. Used as reference only. | `git show 36520035 --stat` |
| BLENDED_SEQUENCE plumbing | primitive_blended_sequence.cpp:825 raises "ExecuteMotion goal for BLENDED_SEQUENCE lacks sequence steps". ExecuteMotion.action and llm_schema.yaml have no support — interface contract is missing | primitive_blended_sequence.cpp:825, transcript discovery |
| HMI | Lives at `hmi/` next to `src/` in the same repo. React + FastAPI. Backend reimplements ROS-side logic locally. | `hmi/backend/services/`, `hmi/frontend/package.json` |

---

## Wave dependency graph

```
              W0 (governance + branch + perception review)
              │
              ├──► W1 (kill silent fallback + JointPositionGuard)
              │     │
              │     └──► W2 (drawing rewire to BLENDED_SEQUENCE)
              │           │
              │           └──► W3 (ReAct on /llm_intent)
              │                 │
              │                 └──► W4 (perception fresh build)
              │                       │
              │                       └──► W5 (HMI aggressive consolidation)
              │                             │
              │                             └──► W6 (first cleanup wave)
              │                                   │
              │                                   └──► W7 (T-axis tiered mode, optional)
```

W1 must come before W2 because emitting `BLENDED_SEQUENCE` while the silent fallback is still active masks the new code path.

W3 must come before W4 because the ReAct agent's tool registry needs `query_perception` as a registered (initially stub) tool before perception is wired in.

W5 must come after W3+W4 because aggressive HMI consolidation changes which ROS services HMI calls; if the services aren't stable yet, HMI breaks twice.

---

## Wave list at a glance

| Wave | Title | Effort | Risk | Status | Pain points addressed |
|---|---|---|---|---|---|
| [W0](W0_governance_branch_perception_review.md) | Governance, branch consolidation, perception review | 3–4 d | Zero runtime | ✅ COMPLETE | #5 (bloat root cause) |
| [W1](W1_kill_silent_fallback_joint_guard.md) | Kill silent CARTESIAN_PATH fallback, add JointPositionGuard | 3–4 d | Medium (sim required first) | ✅ COMPLETE | #4 (J4–J6 unsafe poses) |
| [W2](W2_drawing_rewire_blended_sequence.md) | Rewire drawing to BLENDED_SEQUENCE, consolidate workplane, CIRC degenerate check | 3–5 d | Medium | ✅ COMPLETE | #2 (CIRC/draw broken) |
| [W3](W3_react_agent_on_llm_intent.md) | ReAct agent on `/llm_intent`, chained commands, tool registry | 5–7 d | Medium | ✅ COMPLETE | #1 (LLM is not reasoning) |
| [W4](W4_perception_fresh_build.md) | RealSense D435i eye-to-hand, calibration, scene processor, query_perception | 5–7 d | High (hardware dep) | ✅ COMPLETE | #3 (vision missing) |
| [W5](W5_hmi_aggressive_consolidation.md) | HMI backend gives up local intent/supervisor logic, calls ROS services | 3–5 d | Medium-High | ✅ COMPLETE | #5 (HMI duplicates src/) |
| [W6](W6_first_cleanup_wave.md) | Hard-delete aged DEPRECATED, jscpd full audit, file-budget enforcement | 2–3 d | Low | ✅ COMPLETE | #5 (sustained anti-bloat) |
| [W7](W7_t_axis_tiered_mode.md) | joint_6_t default ±180° / extended ±455° opt-in with precondition gate, Mode enum, three-stage guard enforcement | 2–3 d | Low | ✅ COMPLETE | safety extension |
| [W8](W8_second_cleanup_wave.md) | Second cleanup wave (early execution on 2026-05-09) | 2–3 d | Low | ✅ SOFTWARE VERIFIED / HW BLOCKED | #5 (sustained anti-bloat) |

---

## Five-mechanism anti-bloat enforcement (installed in W0, used by every wave)

| Layer | Concrete artefact | When it runs | Bypassable? |
|---|---|---|---|
| 1. AGENTS.md | 60-line root file with hard rules | Read at session start | Yes — but every reply is prompted to acknowledge it |
| 2. Pre-commit | ruff, black, mypy, clang-format, jscpd, vulture, custom scripts | On `git commit` | Yes (`--no-verify`) — but CI re-runs |
| 3. CI | All pre-commit checks + colcon build + colcon test + safety chain validator | On `git push` to PR branch | No — required check |
| 4. PR template | "What I searched / reused / deprecated / HMI impact / branch hygiene" | On PR open | Yes if reviewer doesn't enforce — humans must reject empty templates |
| 5. Cleanup wave | W6, repeated every 2 weeks | Scheduled | No, on calendar |

If any one layer is removed, bloat returns within 6–8 weeks. All five must coexist.

---

## Hard rules carried forward from previous reviews

- No silent fallback in motion code. Errors fail loudly with primitive name and waypoint index.
- No magic numbers in `.py` / `.cpp` for safety/motion/perception values; YAML SSOT only.
- No new file when an equivalent exists (`rg`/`find` proof required in PR).
- No `pip install` of LLM packages without `requirements.txt` pinning + `--user` install + `import rclpy` smoke test.
- No `computeCartesianPath` for drawing. Banned in W2; CI guard added in W6.
- No reimplementation of ROS-side logic inside `hmi/backend/`. After W5, HMI calls ROS services.
- Calibration timestamps are runtime-filled, never hardcoded.
- Branches do not multiply. `ws-deep-rebuild-3526` is the working branch; only `main`, `super-fix`, `hmi-pro` survive alongside it.
- J5 stays at ±90° (1.571 rad). No widening without separate safety review.
- `park_safe` is deleted. Use only named states with status=`active` in `docs/audit/NAMED_STATE_AUDIT.md`.
- BLENDED_SEQUENCE requires the typed SequenceStep interface (W2.T0). Do not emit BLENDED_SEQUENCE before the interface is merged.
- Perception services use typed interfaces from W4.T0. Do not invent ad-hoc message shapes inside gp4_perception.
- JointPositionGuard runs at three stages (A pre-downsample, B QualityGate, C hw_adapter). All three are required.
---

## What this plan deliberately does NOT include

- LLM model selection or prompt engineering optimization (separate concern, after W3 is functional)
- Multi-robot scenarios (not in scope per `motoros2_config.yaml` — single arm)
- ROS 1 compatibility (Humble only)
- ISO 10218 compliance (this is research/lab use; software safety is one layer, not the only layer)
- Performance benchmarking beyond "not regress against baseline" (W6 only)
- Frontend UI/UX redesign (W5 keeps the existing surface; only backend services change)

---

## How to use this plan

1. Read this SUMMARY end to end (you are here).
2. Open `W0_governance_branch_perception_review.md`.
3. Hand the W0 file to the AI coding agent. Wait for output.
4. Review the agent's output against W0's "Verify" section.
5. Merge W0. Update SUMMARY's "Status" column (add it now if not already).
6. Repeat for W1, W2, …, W7.

Do not skip ahead. Wave dependencies are real — skipping creates rework.

---

**Reliability tag:** `[VERIFIED]` for the verified-facts table (every row has file:line evidence from discovery output 2026-05-03). `[NEEDS-VALIDATION]` for the effort estimates (they assume codebase is frozen during rebuild — every external commit shifts them). The 5-mechanism anti-bloat design is `[VERIFIED]` against software engineering principles, not against any specific tool config; agent must adapt commands to the local environment.

## Rebuild completion status (2026-05-10)

All waves W0–W8 are **software-verified**. The remaining blockers are physical hardware:

1. D435i hand-eye calibration (extrinsics.yaml still `<NOT_CALIBRATED>`).
2. YRC1000micro read-only hardware validation.
3. Hardware execution authorization.

The deprecated `docs/Rebuild_Agent_v2.md` (955-line pre-rebuild agent harness) has been removed.
See `docs/PROJECT_COMPLETION_REPORT.md` for the full completion report.

End of SUMMARY.
