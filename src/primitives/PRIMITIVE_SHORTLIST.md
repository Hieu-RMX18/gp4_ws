# GP4 Public Primitive Shortlist — Frozen for Thesis/Demo

> **Version:** 1.1 (B2 reconciled)
> **Date:** 2026-04-05
> **Owner package:** `primitives`
> **Last audited by:** B2 task — contract reconciliation

This is the **single source of truth** for which primitives are public,
optional, internal-only, or deferred in the GP4 thesis/demo system.

---

## Classification Table

| #  | Primitive Name         | Classification   | .hpp | .cpp | Dispatcher | Schema | llm_gateway | safety | motion_core | Test |
|----|------------------------|------------------|:----:|:----:|:----------:|:------:|:-----------:|:------:|:-----------:|:----:|
| 1  | `HOME`                 | **PUBLIC**        | ✅   | ✅   | ✅         | ✅     | ✅          | ✅     | ✅          | ✅   |
| 2  | `PTP`                  | **PUBLIC**        | ✅   | ✅   | ✅         | ✅     | ✅          | ✅     | ✅          | ✅   |
| 3  | `LIN`                  | **PUBLIC**        | ✅   | ✅   | ✅         | ✅     | ✅          | ✅     | ✅          | ✅   |
| 4  | `CIRC`                 | **INTERNAL¹**     | ✅   | ✅   | ✅         | ❌     | ❌          | ❌     | ❌²         | ✅   |
| 5  | `move_to_named_pose`   | **DEFERRED**      | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 6  | `move_joints`          | **DEFERRED³**     | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 7  | `move_joint`           | **DEFERRED**      | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 8  | `move_to_pose`         | **DEFERRED⁴**    | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 9  | `move_rel`             | **DEFERRED**      | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 10 | `rotate_end_effector`  | **DEFERRED**      | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 11 | `set_speed`            | **DEFERRED**      | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 12 | `set_tool`             | **DEFERRED**      | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 13 | `set_frame`            | **DEFERRED**      | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 14 | `wait`                 | **DEFERRED**      | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 15 | `stop`                 | **DEFERRED**      | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 16 | `approach`             | **INTERNAL**      | ✅   | ✅   | ✅         | ✅⁵    | ❌          | ❌     | ❌          | ✅   |
| 17 | `retract`              | **INTERNAL**      | ✅   | ✅   | ✅         | ✅⁵    | ❌          | ❌     | ❌          | ✅   |
| 18 | `blended_sequence`     | **INTERNAL**      | ✅   | ✅   | ✅         | ❌     | ❌          | ❌     | ❌          | ✅   |
| 19 | `open_gripper`         | **OPTIONAL⁶**    | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |
| 20 | `close_gripper`        | **OPTIONAL⁶**    | ❌   | ❌   | ❌         | ❌     | ❌          | ❌     | ❌          | ❌   |

### Explicitly Deferred (NOT public now)

| Primitive             | Reason                                                       |
|-----------------------|--------------------------------------------------------------|
| `move_to_object`      | Requires perception pipeline (not in scope)                  |
| `pick`                | Requires gripper + perception pipeline                       |
| `place`               | Requires gripper + perception pipeline                       |
| `push`                | Requires force/contact sensing                               |
| `scan_workspace`      | Requires perception pipeline                                 |
| `alarm_reset`         | Requires YRC1000micro alarm/reset ROS2 service (not wired)   |
| `set_payload`         | Requires MotoROS2 payload service (not wired)                |
| `sequence/blending DSL` | Internal only via `blended_sequence`; no public DSL yet    |

---

## Footnotes

1. **CIRC reclassified (B2):** CIRC has .cpp/.hpp/dispatcher/test but is NOT wired
   end-to-end. `motion_core_node.cpp` rejects it at the `is_supported_primitive()` gate.
   `llm_schema.yaml` does not include it. Reclassified from PUBLIC(gated) to INTERNAL.
   `command_schema.json` was deprecated to `.DEPRECATED` in B2.
2. **CIRC planning readiness:** Pilz CIRC planner resolution exists in
   `resolve_planner_selection()` and `planner_router.cpp` but is never reached.
   When CIRC is promoted, all layers must be updated per the checklist below.
3. **move_joints:** PTP with `joint_target[]` already covers joint-space targeting in
   the current contract. A separate `move_joints` primitive is semantically redundant
   unless it adds per-joint velocity/acceleration limits.
4. **move_to_pose:** LIN and PTP with `target_pose` already cover Cartesian targeting.
   A separate `move_to_pose` primitive only makes sense as a higher-level alias that
   auto-selects planner; deferred until alias/dispatch logic is designed.
5. **approach/retract:** Internal sub-primitives of `blended_sequence`.
   Not LLM-callable. Not in `llm_schema.yaml`.
6. **Gripper:** Optional. Requires Robotiq/YRC1000micro I/O service integration.
   No hardware driver wired yet.

---

## Current End-to-End Contract Reality

### What is FULLY wired today (LLM → safety → motion_core → hw_adapter):

| Primitive | LLM schema | safety allowed | motion_core plans | hw_adapter executes |
|-----------|:----------:|:--------------:|:-----------------:|:-------------------:|
| `HOME`    | ✅         | ✅             | ✅                | ✅                  |
| `PTP`     | ✅         | ✅             | ✅                | ✅                  |
| `LIN`     | ✅         | ✅             | ✅                | ✅                  |

### What exists in primitives/ but is NOT wired end-to-end:

| Primitive          | Primitive dispatcher | motion_core routing | LLM/safety support |
|--------------------|:--------------------:|:-------------------:|:------------------:|
| `CIRC`             | ✅                   | ❌ (rejected)       | ❌                 |
| `approach`         | ✅                   | ❌                  | ❌                 |
| `retract`          | ✅                   | ❌                  | ❌                 |
| `blended_sequence` | ✅                   | ❌                  | ❌                 |

---

## Schema/Interface Mismatches — Resolved in B2

1. ✅ **`command_schema.json` deprecated** — renamed to `.DEPRECATED`.
   Runtime uses only `llm_schema.yaml` via `SchemaValidator`.

2. ✅ **`motion_core_node.cpp` whitelist reconciled** —
   `is_supported_primitive()` now allows only `HOME, PTP, LIN`.
   CIRC abort block removed (unreachable after gate change).

3. ✅ **`semantic_validator.py`** — already correct: `{"HOME", "PTP", "LIN"}`.

4. ✅ **Contract consistency test added** —
   `tests/test_contract_consistency.py` enforces agreement across
   schema, semantic_validator, normalizer, and prompt_builder.

---

## What Would Be Required to Promote a New Public Primitive

For any primitive in the "DEFERRED" list to become public, ALL of these steps must be completed:

1. ☐ `.hpp` header in `include/primitives/`
2. ☐ `.cpp` implementation in `src/primitives/src/`
3. ☐ Registration in `primitives/CMakeLists.txt`
4. ☐ Wired into `primitive_dispatcher.cpp`
5. ☐ `PrimitiveType` enum entry in `primitive_types.hpp`
6. ☐ `from_string()` / `to_string()` updated
7. ☐ `llm_schema.yaml` updated with new enum value
8. ☐ ~~`command_schema.json`~~ deprecated in B2; skip this step
9. ☐ `prompt_builder.py` updated with new allowed value
10. ☐ `semantic_validator.py` `_ALLOWED_PRIMITIVES` updated
11. ☐ `normalizer.py` `_PLANNER_DEFAULTS` updated
12. ☐ `motion_core_node.cpp` `is_supported_primitive()` updated
13. ☐ `motion_core_node.cpp` `execute()` routing logic added
14. ☐ `ExecuteMotion.action` updated if new goal fields needed
15. ☐ `ValidateCommand.srv` updated if new request fields needed
16. ☐ Safety validation updated if new constraints needed
17. ☐ Unit test added in `primitives/test/`
18. ☐ Build verified: `colcon build --packages-select primitives interfaces motion_core`
