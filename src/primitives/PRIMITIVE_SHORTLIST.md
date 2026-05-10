# GP4 Public Primitive Shortlist — Frozen for Thesis/Demo

> **Version:** 1.2 (Agentic Stack reconciled)
> **Date:** 2026-04-06
> **Owner package:** `primitives` (contract shortlist) + `motion_core` (runtime planning/dispatch)
> **Last audited by:** Antigravity — Sprint: New Primitives Integration

This is the **single source of truth** for which primitives are public,
optional, internal-only, or deferred in the GP4 thesis/demo system.

---

## Classification Table

| #  | Primitive Name         | Classification   | .hpp | .cpp | Dispatcher | Schema | llm_gateway | safety | motion_core | Test |
|----|------------------------|------------------|:----:|:----:|:----------:|:------:|:-----------:|:------:|:-----------:|:----:|
| 1  | `HOME`                 | **PUBLIC**        | ✅   | ✅   | ✅         | ✅     | ✅          | ✅     | ✅          | ✅   |
| 2  | `PTP`                  | **PUBLIC**        | ✅   | ✅   | ✅         | ✅     | ✅          | ✅     | ✅          | ✅   |
| 3  | `LIN`                  | **PUBLIC**        | ✅   | ✅   | ✅         | ✅     | ✅          | ✅     | ✅          | ✅   |
| 4  | `MOVE_REL`             | **PUBLIC**        | ✅   | ✅   | ✅         | ✅     | ✅          | ✅     | ✅          | ✅   |
| 5  | `GET_POSE`             | **PUBLIC**        | ❌¹  | ❌¹  | ✅         | ✅     | ✅          | ✅     | ✅          | ✅   |
| 6  | `SET_SPEED`            | **PUBLIC**        | ❌¹  | ❌¹  | ❌¹        | ✅     | ✅          | ✅     | ✅          | ✅   |
| 7  | `WAIT`                 | **PUBLIC**        | ❌¹  | ❌¹  | ❌¹        | ✅     | ✅          | ✅     | ✅          | ✅   |
| 8  | `STOP`                 | **PUBLIC**        | ❌¹  | ❌¹  | ❌¹        | ✅     | ✅          | ✅     | ✅          | ✅   |
| 9  | `MOVE_JOINT`           | **PUBLIC**        | ❌¹  | ❌¹  | ❌¹        | ✅     | ✅          | ✅     | ✅          | ✅   |
| 10 | `MOVE_JOINTS`          | **PUBLIC**        | ❌¹  | ❌¹  | ❌¹        | ✅     | ✅          | ✅     | ✅          | ✅   |
| 11 | `IO_SET`               | **PUBLIC**        | ❌¹  | ❌¹  | ❌¹        | ✅     | ✅          | ✅     | ✅          | ✅   |
| 12 | `ALARM_RESET`          | **PUBLIC**        | ❌¹  | ❌¹  | ❌¹        | ✅     | ✅          | ✅     | ✅          | ✅   |
| 13 | `CIRC`                 | **PUBLIC**        | ✅   | ✅   | ✅         | ✅     | ✅          | ✅     | ✅          | ✅   |
| 14 | `approach`             | **INTERNAL**      | ✅   | ✅   | ✅         | ✅²    | ❌          | ❌     | ❌          | ✅   |
| 15 | `retract`              | **INTERNAL**      | ✅   | ✅   | ✅         | ✅²    | ❌          | ❌     | ❌          | ✅   |
| 16 | `blended_sequence`     | **INTERNAL**      | ✅   | ✅   | ✅         | ❌     | ❌          | ❌     | ❌          | ✅   |

---

## Footnotes

1. **Logical/Direct Primitives:** These primitives do not require a separate `.cpp/.hpp` in the `primitives` package because they are handled directly as logic branches in `motion_core_node.cpp` (e.g., WAIT, STOP) or delegated to existing planning pipelines (e.g., MOVE_JOINT → PTP) or ROS2 services (e.g., IO_SET → hw_adapter).
2. **approach/retract:** Internal sub-primitives of `blended_sequence`. Not LLM-callable. Not in `llm_schema.yaml`.

---

## Current End-to-End Contract Reality

### What is FULLY wired today (LLM → safety → motion_core → hw_adapter):

All 13 **PUBLIC** primitives listed above are fully integrated into the end-to-end stack and verified via regression tests.

| Primitive Group | Primitives | Planning Pipeline | Execution |
|-----------------|------------|-------------------|-----------|
| **Motion**      | `HOME`, `PTP`, `LIN`, `MOVE_REL` | Pilz Industrial Planner | MotoROS2 |
| **Arc Motion**  | `CIRC` | Pilz CIRC | MotoROS2 |
| **Joint Motion**| `MOVE_JOINT`, `MOVE_JOINTS` | Delegated to PTP Payload | MotoROS2 |
| **Logical**     | `WAIT`, `STOP`, `SET_SPEED` | In-place execution | n/a |
| **Maintenance** | `ALARM_RESET`, `IO_SET` | Delegated to HW Adapter | MotoROS2 (via Service) |
| **Query**       | `GET_POSE` | Direct State Query | n/a |

### Known ownership debt (explicitly tracked)

- Primitive contract truth is centralized here, but runtime planning logic is still split between
  `primitives` and `motion_core`.
- Keep behavior-compatible fixes explicit and avoid introducing duplicate primitive routing layers.

---

## What is DEFERRED (NOT public now)

| Primitive             | Reason                                                       | Post-W4 Status |
|-----------------------|--------------------------------------------------------------|----------------|
| `move_to_object`      | Requires calibrated perception                               | Software path exists via ReAct `query_perception` → `plan_motion`. Blocked by D435i hand-eye calibration (`<NOT_CALIBRATED>`). |
| `pick`                | Requires gripper + calibrated perception                     | Gripper tools are stub-only in ReAct (`capability_unavailable`). IO mapping unknown. |
| `place`               | Requires gripper + calibrated perception                     | Same as `pick`. |
| `push`                | Requires force/contact sensing                               | No hardware support. |
| `scan_workspace`      | Requires calibrated perception                               | `gp4_perception` scene processor built (W4). Blocked by calibration. |
| `rotate_end_effector` | Deferred until RPY-relative transform logic is audited       | Not started. |
| `set_tool`            | Requires tool frame offset support in hw_adapter             | Not started. Real tool not mounted. |
| `set_frame`           | Requires coordinate transform support in safety/motion_core  | Not started. |
| `set_payload`         | Requires MotoROS2 payload service (not wired)                | Not started. |
| `open/close_gripper`  | Requires Robotiq/YRC1000micro I/O service integration        | ReAct tools exist as stubs. IO mapping unknown. |

---

## MACRO Primitive Note

The `MACRO` primitive is defined in `llm_schema.yaml` and validated in `intent_engine.py`
(max 10 steps, recursive `$ref` to the root schema). It is expanded into individual
steps by the `IntentRouter` during semantic IR routing. There is no separate C++
primitive dispatcher for MACRO — it is handled as sequence expansion in the gateway.

---

## Contract Verification Summary (updated 2026-05-10)

✅ **2901 ROS tests passed, 0 failures** (colcon test-result).
✅ **190 HMI backend tests passed** (pytest).
✅ **llm_schema.yaml** reconciled with **motion_core** whitelist.
✅ **Contract consistency test** enforced across all gateway layers.
✅ **6 new ReAct multi-step reasoning tests** added and passing.
