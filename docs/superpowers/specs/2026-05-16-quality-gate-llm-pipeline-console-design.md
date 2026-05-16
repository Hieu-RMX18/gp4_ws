# GP4 Quality Gate, LLM Pipeline, System Log & Runtime Console — Design Spec

**Date:** 2026-05-16
**Scope:** 4 targeted changes across motion_core (C++), llm_gateway (Python), HMI (TypeScript), and tools (Python)

---

## 1. Quality Gate — Lower MOVE_REL threshold to 0.60

### Problem

The quality gate rejects MOVE_REL commands with `"cartesian fraction below minimum threshold for primitive"` because MOVE_REL falls through to the generic threshold `kMinimumCartesianFraction = 0.90`. Small relative moves near workspace edges often produce Cartesian fractions between 0.60–0.89, which are safe but currently rejected.

The quality gate is a planning quality check, NOT a safety constraint. Workspace bounds (Z_min=0.23m) and collision checking protect the robot independently.

### Changes

**`src/motion_core/include/motion_core/quality_gate.hpp`:**
- Add `static constexpr double kMinimumFractionMoveRel = 0.60;`

**`src/motion_core/src/guards/quality_gate.cpp` — `minimum_cartesian_fraction_for_primitive()`:**
- Add `if (primitive == "MOVE_REL") return kMinimumFractionMoveRel;` before the fallback return

**`src/motion_core/test/test_quality_gate.cpp`:**
- Update existing tests that assert MOVE_REL threshold = 0.90 to expect 0.60
- Add test: MOVE_REL with fraction=0.65 should pass
- Add test: MOVE_REL with fraction=0.55 should fail

### Threshold summary after change

| Primitive | Threshold |
|-----------|-----------|
| CARTESIAN_PATH | 0.95 |
| LIN | 0.90 |
| CIRC | 0.90 |
| MOVE_REL | 0.60 |
| Other (PTP, HOME) | 0.90 (generic) |

---

## 2. LLM Pipeline — Regex pre-parse + LLM validate

### Problem

Commands matching regex fast-path skip LLM entirely. The user wants to see LLM in the pipeline for every command, and regex should only be a pre-parse step with LLM validating/enriching the result.

### Current flow

```
intent_text → _direct_review_semantic_ir() match? → return immediately
                                                  → no match → ReAct/LLM
```

### New flow

```
intent_text → _direct_review_semantic_ir() match? 
  → YES: semantic_ir (source="regex_fast_path")
       → _llm_validate_regex_result(semantic_ir, intent_text)
       → LLM confirms/enriches intent (lightweight single-turn)
       → return enriched_ir (source="regex+llm_validated")
       → if LLM timeout/fail → fallback to regex result (source="regex_fast_path_only")
  → NO: ReAct/direct LLM (source="react" | "llm")
```

### Changes

**`src/llm_gateway/llm_gateway/llm_gateway_node.py` — `_generate_review_semantic_ir()`:**
- After regex match, call new `_llm_validate_regex_result(regex_ir, intent_text)` method
- Emit trace with `source` field at each decision point

**New method `_llm_validate_regex_result(self, regex_ir, intent_text)`:**
- Build lightweight validation prompt: "Given user intent '{intent_text}', confirm this parsed result is correct: {regex_ir}. Reply with the same JSON if correct, or corrected JSON if wrong."
- Call `self._llm_client.generate_response()` with short timeout (5s)
- If LLM confirms → merge any enrichments, set source="regex+llm_validated"
- If LLM timeout/error → use regex_ir as-is, set source="regex_fast_path_only"
- Emit trace events: `regex_pre_parsed`, `llm_validation_started`, `llm_validation_result`

**Trace event `source` field values:**
- `regex_fast_path_only` — regex matched, LLM validation skipped/failed
- `regex+llm_validated` — regex matched, LLM confirmed
- `llm` — direct LLM parse (no regex match)
- `react` — ReAct agent used

---

## 3. System Log (HMI) — Add source labels and expand capacity

### Problem

System Log shows max 14 entries with only timestamp + level + message. No indication of which node/layer produced the message.

### Changes

**`hmi/frontend/components/gp4-hmi/SystemLog.tsx`:**
- Add `source` column between level and message: `[HH:MM:SS] [LEVEL] [source] message`
- Source derived from message tag or a new `source` field on ChatMessage
- Style source in a muted color to not dominate

**`hmi/frontend/components/GP4HMI.tsx`:**
- Increase max log entries from 14 to 30
- Increase message slice from 11 to 25
- Add auto-scroll behavior (scroll to bottom on new entry)

**`hmi/shared/contracts.ts` — `ChatMessage`:**
- Add optional `source?: string` field (backward compatible)

**`hmi/backend/services/supervisor_views.py` — `_append_message()`:**
- Populate `source` field based on context (e.g., "gateway", "safety", "motion_core", "supervisor", "hw_adapter")

---

## 4. Runtime Console — Enrich trace data and improve output

### Problem

`tools/runtime_console.py` receives limited data via `/llm_debug` topic. Missing: parse source (regex vs LLM), safety validation details, quality gate results, planner decisions, cartesian_fraction values.

### Changes — Trace emission enrichment (llm_gateway_node.py)

**Add `source` to all trace events:**
- `_generate_review_semantic_ir()`: set `source` field on returned semantic_ir
- `_emit_trace()`: include `source` in event data when available

**Add safety validation trace:**
- After `/validate_command` response, emit trace: `safety_validated` with fields: `risk_level`, `accepted`, `blocking_reasons[]`, `confirmation_reasons[]`

**Add quality gate / execution result trace:**
- motion_core already returns execution results via ExecuteMotion action feedback
- In `_handle_execution_result()`: emit trace with `cartesian_fraction`, `planner_id`, `points`, `time_parameterization`, `budget_mitigation`, `ruckig_status`

### Changes — runtime_console.py output format

**Enhanced event line format:**
```
[HH:MM:SS.mmm] [node_name]: EVENT_NAME — summary
                  key1=value1  key2=value2  key3=value3
```

Example outputs:
```
[15:02:16.123] [gateway]: PARSE — source=regex+llm_validated intent=move_relative
                  delta_z=-0.03  frame=base_link  speed=0.05

[15:02:16.456] [safety]: VALIDATED — risk=medium accepted=true
                  workspace_ok=true  velocity_ok=true  forbidden_zone_ok=true

[15:02:17.001] [motion_core]: PLANNED — planner=PILZ_LIN primitive=MOVE_REL
                  points=90  cartesian_fraction=0.72  threshold=0.60  PASS

[15:02:17.500] [motion_core]: QUALITY_GATE — PASSED
                  fraction=0.72 >= threshold=0.60 (MOVE_REL)

[15:02:19.001] [hw_adapter]: DISPATCHED — segments=1 hw_time=9.84s
                  ruckig=applied  budget=none

[15:02:19.100] [supervisor]: COMPLETED — success
────────────────────────────────────────────────────────
  📊 fa2173842c  │ 6 events │ 2.98s total │ ✓ SUCCESS
────────────────────────────────────────────────────────
```

For failures:
```
[15:01:32.001] [motion_core]: QUALITY_GATE — FAILED
                  fraction=0.85 < threshold=0.90 (MOVE_REL)
                  WHY:    cartesian fraction below minimum threshold for primitive
                  ACTION: Threshold for MOVE_REL is 0.90. Consider lowering or using PTP.
```

**Structured detail lines:**
- After main event line, print key-value pairs on indented continuation lines
- Only for events with details (not for simple status changes)
- Color-coded: green for pass, red for fail, yellow for warn, cyan for info

**Per-command summary:**
- Keep existing summary block but enrich with:
  - Parse source (regex/llm/react)
  - Planner used
  - Cartesian fraction (if applicable)
  - Each pipeline stage with pass/fail status

### Changes — New subscriptions (if available)

No new ROS topics needed — all enrichment comes through `/llm_debug` traces. The gateway node will emit richer trace events that the console already subscribes to.

---

## Files to modify

| File | Language | Section |
|------|----------|---------|
| `src/motion_core/include/motion_core/quality_gate.hpp` | C++ | 1 |
| `src/motion_core/src/guards/quality_gate.cpp` | C++ | 1 |
| `src/motion_core/test/test_quality_gate.cpp` | C++ | 1 |
| `src/llm_gateway/llm_gateway/llm_gateway_node.py` | Python | 2, 4 |
| `hmi/frontend/components/gp4-hmi/SystemLog.tsx` | TypeScript | 3 |
| `hmi/frontend/components/GP4HMI.tsx` | TypeScript | 3 |
| `hmi/shared/contracts.ts` | TypeScript | 3 |
| `hmi/backend/services/supervisor_views.py` | Python | 3 |
| `tools/runtime_console.py` | Python | 4 |

---

## Testing

- **Quality gate:** `colcon test --packages-select motion_core --output-on-failure`
- **LLM gateway:** `cd src/llm_gateway && python -m pytest tests/ -v`
- **HMI:** Manual verification via dev server
- **Runtime console:** Manual verification via `python3 tools/runtime_console.py --mode both`

## Safety impact

- Quality gate change: reduces MOVE_REL threshold only. Workspace bounds, collision checking, and joint limits are unaffected. All other primitives keep existing thresholds.
- LLM pipeline change: adds validation step, does not remove any safety check. Fallback preserves current behavior.
- System Log / Runtime Console: display-only changes, no effect on execution pipeline.
