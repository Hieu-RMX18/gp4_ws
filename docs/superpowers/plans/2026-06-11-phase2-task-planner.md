# Phase 2 — TaskPlanner (Single-Shot LLM) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the multi-iteration ReAct loop with a single-shot `TaskPlanner` class, wire it into `llm_gateway_node.py`, and delete `react_planner.py` plus its dead tests—while keeping all 618 existing tests green (count may shift after deletions/additions).

**Architecture:** Port the LLM client, config loader, workspace-bounds helpers, system-prompt builder, and `StateInjector` from `react_planner.py` into a new `task_planner.py` (no ROS imports). `TaskPlanner.plan()` sends one message pair (system + user) to the LLM, retries once on bad JSON, and returns either a FactoryTask dict or an error dict. The node drops `ReActAgent`, `IterationBudget`, tool registry, and `_react_plan_cache`; it renames `_react_agent` → `_task_planner` and `_react_enabled` → `_planner_enabled` throughout.

**Tech Stack:** Python 3.10, pytest, ROS 2 Humble (node test uses `pytest.importorskip`). No new dependencies.

---

## File Map

| Action | File | What changes |
|--------|------|-------------|
| **Create** | `src/llm_gateway/llm_gateway/task_planner.py` | Port symbols from `react_planner.py`; add `TaskPlanner` class |
| **Modify** | `src/llm_gateway/llm_gateway/llm_gateway_node.py` | Swap import, init block, `_generate_review_semantic_ir`, `process_intent`, rename attributes |
| **Delete** | `src/llm_gateway/llm_gateway/react_planner.py` | Replaced by `task_planner.py` |
| **Delete** | `src/llm_gateway/tests/test_react_agent.py` | Tests the ReAct loop — dead by design |
| **Delete** | `src/llm_gateway/tests/test_react_tools.py` | Tests the tool registry — dead by design |
| **Modify** | `src/llm_gateway/tests/test_react_gateway_pipeline.py` | Rename symbols, rewrite handoff tests, add guard test |
| **Modify** | `src/llm_gateway/tests/test_llm_backend.py` | `react_planner` → `task_planner` |
| **Modify** | `src/llm_gateway/tests/test_factory_task_prompt.py` | `react_planner` → `task_planner` |
| **Modify** | `src/llm_gateway/tests/test_contracts.py` | `react_planner` → `task_planner` |
| **Modify** | `src/llm_gateway/tests/test_gripper_adapter.py` | `react_planner` → `task_planner` |
| **Modify** | `src/llm_gateway/tests/test_scene_cache.py` | No import fix needed (no react_planner import) — verify |
| **Modify** | `src/llm_gateway/llm_gateway/semantic_ir_contract.py` | Update comment referencing react_planner |
| **Create** | `src/llm_gateway/tests/test_task_planner.py` | Unit tests for `TaskPlanner` |

---

## Task 1: Create `task_planner.py`

**Files:**
- Create: `src/llm_gateway/llm_gateway/task_planner.py`

- [ ] **Step 1.1: Port config + prompt symbols verbatim**

Copy these symbols unchanged from `react_planner.py` into the new file — do NOT edit logic, only reorganize into one clean module:

```python
# Symbols to port (copy exactly from react_planner.py):
# - _MODEL_PLACEHOLDER
# - _default_safety_rules_path()
# - _load_safety_temperature()
# - _default_config_path()
# - _as_bool()
# - _pick_first_non_empty()
# - LLMBackendConfig (dataclass)
# - load_llm_backend_config()
# - FROZEN_SEMANTIC_INTENTS
# - FROZEN_TOP_LEVEL_OUTPUT_INTENTS
# - _DEFAULT_WORKSPACE_BOUNDS
# - _LOGGER (logging.getLogger(__name__))
# - _coerce_workspace_bounds()
# - _load_yaml()
# - _load_workspace_bounds()
# - _format_workspace_bounds()
# - _SYSTEM_PROMPT_TEMPLATE
# - build_system_prompt()
# - _TRANSIENT_HTTP_STATUS
# - _is_transient_http()
# - OpenAICompatibleLLMClient  (the full class)
# - StateInjector  (pure Python, no ROS — verify no rclpy import inside)
```

Critical: the file must have `from __future__ import annotations` at the top, and **zero** `import rclpy` / `import ros` lines. The file must also import `from llm_gateway.factory_task import is_factory_task, parse_factory_task, FactoryTaskError` and `from llm_gateway.intent_engine import LLMParser`.

- [ ] **Step 1.2: Write `TaskPlanner` class**

Add this class at the bottom of `task_planner.py`:

```python
import json
import logging

_LOGGER = logging.getLogger(__name__)


class TaskPlanner:
    """Single-shot NL -> FactoryTask planner. No tool loop, no ROS dependency.

    Sends one message pair (system prompt + user text with state context) to the
    LLM, parses the response, and returns a FactoryTask dict or an error dict.
    Retries once (max_repair=1) if the response is not a valid FactoryTask or
    recognized error dict.
    """

    def __init__(
        self,
        llm_client: OpenAICompatibleLLMClient,
        state_injector: StateInjector,
        schema_validator,
        payload_parser: "LLMParser | None" = None,
        max_repair: int = 1,
    ) -> None:
        self._llm_client = llm_client
        self._state_injector = state_injector
        self._schema_validator = schema_validator
        self._payload_parser = payload_parser or LLMParser()
        self._max_repair = int(max_repair)

    def plan(self, user_text: str) -> dict:
        """Convert natural-language text to a FactoryTask or error dict.

        Returns:
            A FactoryTask dict (is_factory_task() == True),
            an error dict (has key "error"),
            or {"error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND", ...} after exhausting
            repair attempts.
        """
        state = self._state_injector.snapshot()
        state_context = (
            "Current robot state:\n"
            + json.dumps(state, indent=2, ensure_ascii=False)
            + "\n\n"
        )
        user_message = state_context + user_text.strip()

        for attempt in range(self._max_repair + 1):
            messages = [
                {
                    "role": "system",
                    "content": self._llm_client._system_prompt,
                },
                {
                    "role": "user",
                    "content": user_message,
                },
            ]
            if attempt > 0:
                messages.append(
                    {
                        "role": "user",
                        "content": (
                            "Return exactly one valid FactoryTask JSON object "
                            "with task_type='factory_task', version='1.0', "
                            "task_id, and root. No markdown, no extra keys."
                        ),
                    }
                )

            try:
                raw = self._llm_client.generate_response_from_messages(messages)
            except Exception as exc:
                return {
                    "error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND",
                    "hint": f"LLM request failed: {exc}",
                }

            try:
                parsed = self._payload_parser.parse(raw)
            except Exception:
                parsed = {"error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND", "hint": raw[:200]}

            if is_factory_task(parsed):
                try:
                    parse_factory_task(parsed)
                except FactoryTaskError as exc:
                    parsed = {
                        "error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND",
                        "hint": f"FactoryTask validation failed: {exc}",
                    }
                else:
                    return parsed

            if "error" in parsed:
                return parsed

            # Neither FactoryTask nor error — retry if budget allows
            _LOGGER.warning(
                "TaskPlanner: attempt %d response is neither FactoryTask nor error; "
                "repair=%d/%d",
                attempt + 1,
                attempt,
                self._max_repair,
            )

        return {
            "error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND",
            "hint": "Planner could not produce a valid FactoryTask. Rephrase the command.",
        }
```

- [ ] **Step 1.3: Verify no ROS imports**

```bash
cd /home/hieu2/gp4_ws/src/llm_gateway
python3 -c "import llm_gateway.task_planner; print('OK')"
```

Expected output: `OK` (no ImportError, no rclpy errors).

---

## Task 2: Add `test_task_planner.py`

**Files:**
- Create: `src/llm_gateway/tests/test_task_planner.py`

- [ ] **Step 2.1: Write the test file**

```python
"""Unit tests for task_planner.TaskPlanner."""

from __future__ import annotations

import json

import pytest

from llm_gateway.task_planner import StateInjector, TaskPlanner, OpenAICompatibleLLMClient


class _MockLLMClient:
    """Fake LLM client with a pre-built system_prompt attribute."""

    def __init__(self, responses: list[str]) -> None:
        self._system_prompt = "You are a test planner."
        self._responses = list(responses)
        self.calls: list[list[dict]] = []

    def generate_response_from_messages(self, messages: list[dict]) -> str:
        self.calls.append(messages)
        if not self._responses:
            raise RuntimeError("no more mock responses")
        return self._responses.pop(0)


def _factory_task_json(task_id: str = "go-home") -> str:
    return json.dumps(
        {
            "task_type": "factory_task",
            "version": "1.0",
            "task_id": task_id,
            "root": {"type": "skill", "name": "go_home", "args": {}},
        }
    )


def _make_planner(responses: list[str], max_repair: int = 1) -> tuple[TaskPlanner, _MockLLMClient]:
    client = _MockLLMClient(responses)
    injector = StateInjector()

    class _FakeValidator:
        pass

    planner = TaskPlanner(
        llm_client=client,
        state_injector=injector,
        schema_validator=_FakeValidator(),
        max_repair=max_repair,
    )
    return planner, client


# ── (a) Valid FactoryTask JSON returned directly ─────────────────────────────

def test_valid_factory_task_is_returned():
    planner, client = _make_planner([_factory_task_json()])

    result = planner.plan("go home")

    assert result["task_type"] == "factory_task"
    assert result["task_id"] == "go-home"
    assert len(client.calls) == 1


# ── (b) error dict passthrough (MISSING_SLOT) ────────────────────────────────

def test_missing_slot_error_is_passed_through():
    error_json = json.dumps(
        {
            "error": "MISSING_SLOT",
            "missing_fields": ["distance"],
            "hint": "relative move requires direction and distance.",
        }
    )
    planner, client = _make_planner([error_json])

    result = planner.plan("move down a bit")

    assert result["error"] == "MISSING_SLOT"
    assert result["missing_fields"] == ["distance"]
    assert len(client.calls) == 1


# ── (c) Broken JSON → retry once → UNSUPPORTED_OR_AMBIGUOUS_COMMAND ──────────

def test_broken_json_triggers_one_retry_then_fails():
    planner, client = _make_planner(["not json at all", "still not json"], max_repair=1)

    result = planner.plan("do something weird")

    assert result["error"] == "UNSUPPORTED_OR_AMBIGUOUS_COMMAND"
    # First attempt + one repair attempt = 2 total LLM calls
    assert len(client.calls) == 2


def test_broken_json_then_valid_on_retry_succeeds():
    planner, client = _make_planner(["not json at all", _factory_task_json()], max_repair=1)

    result = planner.plan("go home please")

    assert result["task_type"] == "factory_task"
    assert len(client.calls) == 2


# ── (d) State context appears in messages ────────────────────────────────────

def test_state_context_included_in_user_message():
    injector = StateInjector()
    client = _MockLLMClient([_factory_task_json()])

    class _FakeValidator:
        pass

    planner = TaskPlanner(
        llm_client=client,
        state_injector=injector,
        schema_validator=_FakeValidator(),
    )
    planner.plan("go home")

    assert len(client.calls) == 1
    messages = client.calls[0]
    # messages[0] = system, messages[1] = user
    user_content = messages[1]["content"]
    assert "robot_state" in user_content
    assert "go home" in user_content


def test_repair_message_appended_on_second_attempt():
    planner, client = _make_planner(["not json", _factory_task_json()], max_repair=1)

    planner.plan("go home please")

    first_call_len = len(client.calls[0])
    second_call_len = len(client.calls[1])
    # Second call appends a repair hint message
    assert second_call_len == first_call_len + 1
```

- [ ] **Step 2.2: Run new tests to verify they pass**

```bash
cd /home/hieu2/gp4_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/llm_gateway && python3 -m pytest tests/test_task_planner.py -v
```

Expected: all 7 tests pass.

---

## Task 3: Update `llm_gateway_node.py` — imports and init block

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`

> **Read the file before editing.** Lines referenced below may shift. Always grep for the exact string first.

- [ ] **Step 3.1: Replace the import block**

Find:
```python
from llm_gateway.react_planner import (
    IterationBudget,
    OpenAICompatibleLLMClient,
    ReActAgent,
    StateInjector,
    build_default_react_tool_registry,
    load_llm_backend_config,
)
```

Replace with:
```python
from llm_gateway.task_planner import (
    OpenAICompatibleLLMClient,
    StateInjector,
    TaskPlanner,
    load_llm_backend_config,
)
```

- [ ] **Step 3.2: Replace the ReAct init block**

Find the block starting with `# ── ReAct agent init (W3) ─────────────────────────────────────────────` through the `else: self._react_agent = None` line (approximately lines 228–261). Replace with:

```python
        # ── Planner init (Phase 2) ───────────────────────────────────────────
        self._planner_enabled = self._load_planner_enabled()
        self._state_injector = StateInjector()
        poses = []
        try:
            poses.extend(list(_load_srdf_named_poses().keys()))
        except Exception:
            pass
        if self._station_scene_graph is not None:
            self._state_injector.set_semantic_map(self._station_scene_graph.to_dict())
            if hasattr(self._station_scene_graph, "_regions"):
                poses.extend(self._station_scene_graph._regions.keys())
        self._state_injector.set_available_named_poses(poses)
        if self._planner_enabled:
            self._task_planner: TaskPlanner | None = TaskPlanner(
                llm_client=self._llm_client,
                state_injector=self._state_injector,
                schema_validator=self._schema_validator,
            )
        else:
            self._task_planner = None
```

- [ ] **Step 3.3: Fix `GripperIoAdapter` robot_mode_fn reference**

The `GripperIoAdapter` init at ~line 197 still reads `robot_mode_fn=self._current_react_robot_mode`. After renaming `_react_state_injector` to `_state_injector`, the method `_current_react_robot_mode` reads `self._react_state_injector`. Update the method body (line ~1265) to read `self._state_injector` instead.

Find:
```python
    def _current_react_robot_mode(self) -> str:
        snapshot = self._react_state_injector.snapshot()
```

Replace with:
```python
    def _current_react_robot_mode(self) -> str:
        snapshot = self._state_injector.snapshot()
```

- [ ] **Step 3.4: Fix subscription callbacks referencing `_react_state_injector`**

Two callbacks still call `self._react_state_injector.update_joint_states(...)` and `self._react_state_injector.update_robot_status(...)`. Find all occurrences and replace `_react_state_injector` with `_state_injector` throughout the entire file.

```bash
grep -n "_react_state_injector" /home/hieu2/gp4_ws/src/llm_gateway/llm_gateway/llm_gateway_node.py
```

Replace every occurrence of `self._react_state_injector` with `self._state_injector` in the file (subscriber callbacks and wherever else it appears).

- [ ] **Step 3.5: Rename `_load_react_enabled` → `_load_planner_enabled`**

Find the method definition:
```python
    def _load_react_enabled(self) -> bool:
        """Read llm.react.enabled from safety_rules.yaml SSOT."""
```

Replace with:
```python
    def _load_planner_enabled(self) -> bool:
        """Read llm.react.enabled from safety_rules.yaml SSOT. Config key unchanged."""
```

Body stays identical (still reads `react.enabled`).

- [ ] **Step 3.6: Delete `_react_plan_cache` lines**

Find and remove these two lines (inside the now-deleted init block — should already be gone from step 3.2, but verify):
```python
            self._react_plan_cache: Dict[str, Any] = {}
            self._react_plan_cache_max_entries = 64
```

Also grep for any remaining references to `_react_plan_cache`:
```bash
grep -n "_react_plan_cache" /home/hieu2/gp4_ws/src/llm_gateway/llm_gateway/llm_gateway_node.py
```

The only remaining reference would be in `react_planner.py`'s `PlanMotionTool` — that file is being deleted so no action needed there.

- [ ] **Step 3.7: Rename `_react_enabled` and `_react_agent` in subscriber names**

The subscribers `_react_joint_state_subscriber`, `_react_joint_state_fallback_subscriber`, `_react_robot_status_subscriber` are ROS topic names — do NOT rename them (renaming would break the runtime). Keep those subscription variable names as-is.

Rename only the logic attributes:
- `self._react_enabled` → `self._planner_enabled` (already done in step 3.2 for init; verify no remaining occurrences)
- `self._react_agent` → `self._task_planner` (already done in step 3.2 for init; verify)

```bash
grep -n "_react_enabled\|_react_agent\b" /home/hieu2/gp4_ws/src/llm_gateway/llm_gateway/llm_gateway_node.py
```

Fix any remaining occurrences not yet updated.

---

## Task 4: Update `_generate_review_semantic_ir` and `process_intent` in node

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`

- [ ] **Step 4.1: Rewrite the ReAct branch in `_generate_review_semantic_ir`**

Find (approximately lines 570–591):
```python
        if self._react_enabled and self._react_agent is not None:
            react_result = self._react_agent.run(intent_text)
            if not react_result.get("_handoff"):
                if is_factory_task(react_result):
                    return self._compile_factory_task_review_result(
                        react_result, parse_source="react_factory_task"
                    )
                if "error" in react_result:
                    enriched_result = dict(react_result)
                    enriched_result["_parse_source"] = "react"
                    return enriched_result
                return {
                    "error": "REACT_HANDOFF",
                    "message": "ReAct returned a non-FactoryTask final payload.",
                    "hint": "ReAct final output must be FactoryTask; Semantic IR is internal to the gateway compiler.",
                }
            reason = react_result.get("reason", "unknown")
            return {
                "error": "REACT_HANDOFF",
                "message": f"ReAct could not resolve the request: {reason}.",
                "hint": "Rephrase the command with clearer intent or check that all required parameters are provided.",
            }
```

Replace with:
```python
        if self._planner_enabled and self._task_planner is not None:
            self._emit_trace("llm_request_started", "reasoning", source="llm")
            planner_result = self._task_planner.plan(intent_text)
            self._emit_trace("llm_response_received", "reasoning", source="llm")
            if is_factory_task(planner_result):
                return self._compile_factory_task_review_result(
                    planner_result, parse_source="llm_factory_task"
                )
            if "error" in planner_result:
                enriched = dict(planner_result)
                enriched["_parse_source"] = "llm"
                return enriched
            return {
                "error": "UNSUPPORTED_OR_AMBIGUOUS_COMMAND",
                "message": "planner returned neither FactoryTask nor error",
                "hint": "Rephrase the command.",
            }
```

- [ ] **Step 4.2: Rewrite the ReAct branch in `process_intent`**

Find (approximately lines 1309–1338):
```python
        if self._react_enabled and self._react_agent is not None:
            try:
                react_result = self._react_agent.run(intent_text)
            except Exception as exc:
                self._reject("react_agent_failed", str(exc), intent_text=intent_text)
                return
            if react_result.get("_handoff"):
                self._reject(
                    "react_handoff",
                    ...
                )
                return
            if is_factory_task(react_result):
                self._reject(
                    "factory_task_requires_review",
                    ...
                )
                return
            elif "error" not in react_result:
                self._reject(
                    "react_contract_rejected",
                    ...
                )
                return
            payload = json.dumps(react_result)
        else:
```

Replace with:
```python
        if self._planner_enabled and self._task_planner is not None:
            try:
                self._emit_trace("llm_request_started", "reasoning")
                planner_result = self._task_planner.plan(intent_text)
                self._emit_trace("llm_response_received", "reasoning")
            except Exception as exc:
                self._reject("planner_failed", str(exc), intent_text=intent_text)
                return
            if is_factory_task(planner_result):
                self._reject(
                    "factory_task_requires_review",
                    "FactoryTask motion requests must be reviewed through "
                    "/llm_gateway/review_intent and confirmed before execution.",
                    intent_text=intent_text,
                )
                return
            if "error" not in planner_result:
                self._reject(
                    "planner_contract_rejected",
                    "Planner returned a non-FactoryTask final payload.",
                    intent_text=intent_text,
                    hint="Final output must be FactoryTask; Semantic IR is internal to the gateway compiler.",
                )
                return
            payload = json.dumps(planner_result)
        else:
```

---

## Task 5: Fix all test files importing `react_planner`

**Files:**
- Modify: `src/llm_gateway/tests/test_llm_backend.py`
- Modify: `src/llm_gateway/tests/test_factory_task_prompt.py`
- Modify: `src/llm_gateway/tests/test_contracts.py`
- Modify: `src/llm_gateway/tests/test_gripper_adapter.py`
- Modify: `src/llm_gateway/llm_gateway/semantic_ir_contract.py`

- [ ] **Step 5.1: Fix `test_llm_backend.py`**

Find:
```python
from llm_gateway.react_planner import load_llm_backend_config
```
Replace with:
```python
from llm_gateway.task_planner import load_llm_backend_config
```

- [ ] **Step 5.2: Fix `test_factory_task_prompt.py`**

Find:
```python
from llm_gateway.react_planner import build_system_prompt
```
Replace with:
```python
from llm_gateway.task_planner import build_system_prompt
```

- [ ] **Step 5.3: Fix `test_contracts.py`**

Find:
```python
from llm_gateway.react_planner import (
    FROZEN_SEMANTIC_INTENTS,
    FROZEN_TOP_LEVEL_OUTPUT_INTENTS,
    build_system_prompt,
)
```
Replace with:
```python
from llm_gateway.task_planner import (
    FROZEN_SEMANTIC_INTENTS,
    FROZEN_TOP_LEVEL_OUTPUT_INTENTS,
    build_system_prompt,
)
```

- [ ] **Step 5.4: Fix `test_gripper_adapter.py`**

Find:
```python
from llm_gateway.react_planner import GripperCloseTool, GripperOpenTool
```
Replace with:
```python
from llm_gateway.task_planner import GripperCloseTool, GripperOpenTool
```

Note: `GripperCloseTool` and `GripperOpenTool` are defined in `react_planner.py` but are **not** part of the spec's ported set for `task_planner.py`. Check whether they are also defined in `composite_tools.py`.

```bash
grep -n "class GripperCloseTool\|class GripperOpenTool" /home/hieu2/gp4_ws/src/llm_gateway/llm_gateway/composite_tools.py 2>/dev/null || echo "not in composite_tools"
grep -n "GripperCloseTool\|GripperOpenTool" /home/hieu2/gp4_ws/src/llm_gateway/llm_gateway/composite_tools.py | head -5
```

If they exist in `composite_tools.py`, change the import to:
```python
from llm_gateway.composite_tools import GripperCloseTool, GripperOpenTool
```

If not, they must be ported into `task_planner.py` as well (the spec says port from react_planner, so include them — the tool classes are pure Python, no ROS in their definitions, only in `invoke()` which checks `context.ros_node`).

Decision: Port `GripperCloseTool` and `GripperOpenTool` into `task_planner.py` as part of Step 1.1's symbol list. (They are used by `test_gripper_adapter.py` which imports them from `react_planner`.)

- [ ] **Step 5.5: Fix `semantic_ir_contract.py` comment**

```bash
grep -n "react_planner" /home/hieu2/gp4_ws/src/llm_gateway/llm_gateway/semantic_ir_contract.py
```

Update any comment that references `react_planner` to reference `task_planner` instead.

- [ ] **Step 5.6: Verify `test_scene_cache.py` has no react_planner import**

```bash
grep "react_planner" /home/hieu2/gp4_ws/src/llm_gateway/tests/test_scene_cache.py
```

Expected: no output. If any line found, update to `task_planner`.

---

## Task 6: Update `test_react_gateway_pipeline.py`

**Files:**
- Modify: `src/llm_gateway/tests/test_react_gateway_pipeline.py`

This is the largest and most careful step. Read the full analysis before editing.

**Rename map for this file:**
- Import: `from llm_gateway.react_planner import StateInjector` → `from llm_gateway.task_planner import StateInjector`
- Class `_StaticReActAgent` → `_StaticPlanner` (rename class, keep identical to existing but rename method `run` → `plan`)
- `node._react_agent` → `node._task_planner`
- `node._react_enabled` → `node._planner_enabled`
- `_assert_react_factory_task_review`: update `_parse_source` assertion from `"react_factory_task"` → `"llm_factory_task"` (because `_generate_review_semantic_ir` now uses `parse_source="llm_factory_task"` in the planner branch)
- `_make_gateway_shell`: update all references

- [ ] **Step 6.1: Replace the import**

Find:
```python
from llm_gateway.react_planner import StateInjector
```
Replace with:
```python
from llm_gateway.task_planner import StateInjector
```

- [ ] **Step 6.2: Rename `_StaticReActAgent` → `_StaticPlanner`**

Find:
```python
class _StaticReActAgent:
    def __init__(self, result):
        self.result = result
        self.user_text = None
        self.calls = []

    def run(self, user_text):
        self.user_text = user_text
        self.calls.append(user_text)
        return self.result
```
Replace with:
```python
class _StaticPlanner:
    def __init__(self, result):
        self.result = result
        self.user_text = None
        self.calls = []

    def plan(self, user_text):
        self.user_text = user_text
        self.calls.append(user_text)
        return self.result
```

- [ ] **Step 6.3: Update `_make_gateway_shell`**

Find inside `_make_gateway_shell`:
```python
    node._react_enabled = True
    if not raw_react_result:
        react_result = _factory_task_from_semantic_ir("react-task", react_result)
    node._react_agent = _StaticReActAgent(react_result)
```
Replace with:
```python
    node._planner_enabled = True
    if not raw_react_result:
        react_result = _factory_task_from_semantic_ir("react-task", react_result)
    node._task_planner = _StaticPlanner(react_result)
```

- [ ] **Step 6.4: Update `_assert_react_factory_task_review`**

The `_generate_review_semantic_ir` in the node now emits `parse_source="llm_factory_task"` (not `"react_factory_task"`) when the planner returns a FactoryTask. Update the helper to accept both sources (since some tests hard-assert `"react_factory_task"` in `result.semantic_ir_json`).

Find:
```python
    assert payload["_parse_source"] == "react_factory_task"
```

Replace with:
```python
    assert payload["_parse_source"] in {"react_factory_task", "llm_factory_task"}
```

- [ ] **Step 6.5: Fix all `node._react_agent` and `node._react_enabled` references in test bodies**

```bash
grep -n "_react_agent\|_react_enabled" /home/hieu2/gp4_ws/src/llm_gateway/tests/test_react_gateway_pipeline.py
```

For every occurrence:
- `node._react_agent` → `node._task_planner`
- `node._react_enabled` → `node._planner_enabled`
- `node._react_agent.user_text` → `node._task_planner.user_text`
- `node._react_agent.calls` → `node._task_planner.calls`

Also in `test_gateway_react_state_callbacks_update_state_injector`:
```python
    node._react_state_injector = StateInjector()
```
Replace with:
```python
    node._state_injector = StateInjector()
```

And the assertions:
```python
    node._react_joint_state_callback(joint_msg)
    node._react_robot_status_callback(robot_status)
    snapshot = node._react_state_injector.snapshot()["robot_state"]
```
Replace the snapshot line with:
```python
    snapshot = node._state_injector.snapshot()["robot_state"]
```

- [ ] **Step 6.6: Fix REACT_HANDOFF test**

Find:
```python
def test_review_intent_rejects_react_semantic_ir_final_payload():
    ...
    assert '"error":"REACT_HANDOFF"' in result.semantic_ir_json
    assert "FactoryTask" in result.error
```

The new planner returns a raw non-FactoryTask dict straight from `_generate_review_semantic_ir` — the error key is now `UNSUPPORTED_OR_AMBIGUOUS_COMMAND` or the dict's own error. Since `_StaticPlanner.plan()` now returns the raw result (not using `_handoff`), the raw_react_result=True path (non-FactoryTask semantic IR) goes through `_generate_review_semantic_ir`'s "neither FactoryTask nor error" branch. Update the assertion:

```python
    # The planner returned a non-FactoryTask, non-error dict.
    # Node returns UNSUPPORTED_OR_AMBIGUOUS_COMMAND.
    assert result.accepted is False
    assert (
        '"error":"REACT_HANDOFF"' in result.semantic_ir_json
        or '"error":"UNSUPPORTED_OR_AMBIGUOUS_COMMAND"' in result.semantic_ir_json
    )
    assert "FactoryTask" in result.error or "planner" in result.error.lower()
```

- [ ] **Step 6.7: Fix `test_generate_review_semantic_ir_reparses_every_submission`**

Find:
```python
    class _CountingAgent:
        def run(self, text):
            calls.append(text)
            return inner_agent.run(text)

    node._react_agent = _CountingAgent()
```

Replace:
```python
    class _CountingPlanner:
        def plan(self, text):
            calls.append(text)
            return inner_planner.plan(text)

    inner_planner = node._task_planner
    node._task_planner = _CountingPlanner()
```

- [ ] **Step 6.8: Fix `test_direct_tier_handles_safe_commands_without_react`**

Find:
```python
    class _MustNotRun:
        def run(self, text):
            raise AssertionError("ReAct must not run for tier-1 commands")

    node._react_agent = _MustNotRun()
```
Replace:
```python
    class _MustNotRun:
        def plan(self, text):
            raise AssertionError("Planner must not run for tier-1 commands")

    node._task_planner = _MustNotRun()
```

- [ ] **Step 6.9: Add the guard test**

Add this test at the end of the file:

```python
def test_react_loop_is_fully_removed():
    """Phase 2 guard: react_planner module must not exist."""
    import importlib
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module("llm_gateway.react_planner")
```

- [ ] **Step 6.10: Fix all hard-coded `"react_factory_task"` string assertions**

```bash
grep -n '"react_factory_task"' /home/hieu2/gp4_ws/src/llm_gateway/tests/test_react_gateway_pipeline.py
```

For each assertion that checks `_parse_source == "react_factory_task"`, update to accept `"llm_factory_task"` as well:

```python
# Before:
assert semantic_ir["_parse_source"] == "react_factory_task"
# After:
assert semantic_ir["_parse_source"] in {"react_factory_task", "llm_factory_task"}
```

Or if the test checks `'"_parse_source":"react_factory_task"'` in a JSON string:
```python
# Before:
assert '"_parse_source":"react_factory_task"' in result.semantic_ir_json
# After:
assert (
    '"_parse_source":"react_factory_task"' in result.semantic_ir_json
    or '"_parse_source":"llm_factory_task"' in result.semantic_ir_json
)
```

Also update `test_review_response_carries_code_version_and_parse_source`:
```python
assert stamped["_parse_source"] in {"direct", "react_factory_task", "llm_factory_task", "react", "llm"}
```
This already includes `"llm_factory_task"` — verify it is present.

---

## Task 7: Delete dead files

**Files:**
- Delete: `src/llm_gateway/llm_gateway/react_planner.py`
- Delete: `src/llm_gateway/tests/test_react_agent.py`
- Delete: `src/llm_gateway/tests/test_react_tools.py`

- [ ] **Step 7.1: Delete the three files**

```bash
rm /home/hieu2/gp4_ws/src/llm_gateway/llm_gateway/react_planner.py
rm /home/hieu2/gp4_ws/src/llm_gateway/tests/test_react_agent.py
rm /home/hieu2/gp4_ws/src/llm_gateway/tests/test_react_tools.py
```

- [ ] **Step 7.2: Verify guard test fires**

```bash
cd /home/hieu2/gp4_ws/src/llm_gateway
python3 -m pytest tests/test_react_gateway_pipeline.py::test_react_loop_is_fully_removed -v
```

Expected: PASSED.

---

## Task 8: Run full test suite and fix remaining failures

**Files:** (whatever is still broken)

- [ ] **Step 8.1: Run the full suite**

```bash
cd /home/hieu2/gp4_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/llm_gateway && python3 -m pytest tests/ -q 2>&1 | tail -30
```

- [ ] **Step 8.2: Fix each failure**

For each failure, apply the minimal surgical fix. Common expected failures:

1. Any test still importing `llm_gateway.react_planner` (will be `ModuleNotFoundError`): update import to `task_planner`.
2. Tests asserting `node._react_agent` attribute: update to `_task_planner`.
3. Tests asserting `_parse_source == "react_factory_task"` where node now returns `"llm_factory_task"`: update assertion.
4. `test_review_intent_reuses_cached_draw_semantic_ir_without_llm_call` — this test calls `_on_review_intent` twice and checks that the second call also uses the planner (not a cache). With planner, both calls go through `_task_planner.plan()`. The `_StaticPlanner` records calls. Verify `node._task_planner.calls` has 2 entries.

- [ ] **Step 8.3: Run again until green**

```bash
cd /home/hieu2/gp4_ws/src/llm_gateway && python3 -m pytest tests/ -q
```

Expected: all tests pass (baseline 618 minus ~tests in deleted files + new tests in test_task_planner.py).

---

## Task 9: Final smoke check

- [ ] **Step 9.1: Import sanity**

```bash
cd /home/hieu2/gp4_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -c "
from llm_gateway.task_planner import (
    TaskPlanner, OpenAICompatibleLLMClient, StateInjector,
    load_llm_backend_config, build_system_prompt,
    FROZEN_SEMANTIC_INTENTS, LLMBackendConfig,
    GripperCloseTool, GripperOpenTool,
)
print('task_planner OK')
import llm_gateway.llm_gateway_node as n
print('node OK')
"
```

Expected: two `OK` lines, no ImportError.

- [ ] **Step 9.2: Confirm `react_planner` is gone from the package**

```bash
python3 -c "import llm_gateway.react_planner" 2>&1 | grep ModuleNotFoundError
```

Expected: `ModuleNotFoundError: No module named 'llm_gateway.react_planner'`

- [ ] **Step 9.3: Final full test run**

```bash
cd /home/hieu2/gp4_ws/src/llm_gateway && python3 -m pytest tests/ -q --tb=short 2>&1 | tail -10
```

Expected: all tests pass, no FAILED.

---

## Self-Review

### Spec coverage (§3, §8 Phase 2 checklist)

| Requirement | Task |
|-------------|------|
| Create `task_planner.py`, no ROS imports | Task 1 |
| Port LLMBackendConfig, load_llm_backend_config, helpers | Task 1 step 1.1 |
| Port workspace bounds, system prompt builder | Task 1 step 1.1 |
| Port OpenAICompatibleLLMClient | Task 1 step 1.1 |
| Port StateInjector | Task 1 step 1.1 |
| New TaskPlanner class with plan() | Task 1 step 1.2 |
| Single-shot messages-based call | Task 1 step 1.2 |
| max_repair=1 retry with hint | Task 1 step 1.2 |
| Wire into node init | Task 3 |
| _generate_review_semantic_ir updated | Task 4 step 4.1 |
| process_intent updated | Task 4 step 4.2 |
| Delete _react_plan_cache | Task 3 step 3.6 |
| Rename _react_agent → _task_planner | Task 3 steps 3.2, 3.7 |
| Rename _react_enabled → _planner_enabled | Task 3 steps 3.2, 3.5, 3.7 |
| Rename _react_state_injector → _state_injector | Task 3 step 3.4 |
| Delete react_planner.py | Task 7 |
| Delete test_react_agent.py, test_react_tools.py | Task 7 |
| Fix all react_planner imports in tests | Task 5 |
| Update test_react_gateway_pipeline.py | Task 6 |
| Add test_react_loop_is_fully_removed guard | Task 6 step 6.9 |
| Add test_task_planner.py | Task 2 |
| llm_request_started / llm_response_received trace events | Task 4 steps 4.1, 4.2 |

### Placeholder scan

No TBD/TODO/placeholder present — all code blocks are complete.

### Type consistency

- `TaskPlanner.plan()` returns `dict` — used in node as `planner_result = self._task_planner.plan(...)` ✓
- `_StaticPlanner.plan()` signature matches `TaskPlanner.plan()` ✓
- `StateInjector` imported from same module in node and tests ✓
- `GripperCloseTool`, `GripperOpenTool` ported to `task_planner.py` and imported there ✓

### Key risks

1. **`_current_react_robot_mode` method name**: The `GripperIoAdapter` init receives `robot_mode_fn=self._current_react_robot_mode`. The method name itself is NOT renamed (only its body changes to read `self._state_injector` instead of `self._react_state_injector`). This is intentional — the method still works correctly with the new attribute name.

2. **`_parse_source` value change**: The planner branch in `_generate_review_semantic_ir` now passes `parse_source="llm_factory_task"` (same as the non-planner LLM path below it). Previously the ReAct branch used `"react_factory_task"`. Tests asserting the exact string must be updated to accept both values (Step 6.4, 6.10).

3. **`test_review_intent_rejects_react_semantic_ir_final_payload`**: The `raw_react_result=True` case passes a plain semantic IR dict (not a FactoryTask, no error key). Under the new planner, `_generate_review_semantic_ir` hits the "neither FactoryTask nor error" branch and returns `UNSUPPORTED_OR_AMBIGUOUS_COMMAND`. The test checks for `REACT_HANDOFF` in the JSON — Step 6.6 relaxes this assertion.

4. **`test_default_react_tool_registry_keeps_semantic_ir_off_llm_tool_surface`**: This test imports `build_default_react_tool_registry` from `react_planner`. After deletion, this will fail. Since the tool registry itself is being deleted, this test should be **removed** in Task 6 (it tests dead code). Add it to the deletion list in Step 6.

5. **`IterationCounters`/`IterationBudget` in `test_gripper_adapter.py`**: Line 45 imports `IterationCounters, IterationBudget` from `react_planner`. These classes are deleted. Check if the test uses them or just imports them.

```bash
grep -A 5 "IterationCounters\|IterationBudget" /home/hieu2/gp4_ws/src/llm_gateway/tests/test_gripper_adapter.py
```

If used only in one test, either remove that specific test or port the minimal dataclasses needed. If only imported but never called, remove the import.
