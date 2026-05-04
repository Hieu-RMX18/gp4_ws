# W3 — ReAct Agent on `/llm_intent`, Chained Commands, Tool Registry

**Wave class:** LLM behaviour upgrade
**Risk:** Medium (changes LLM contract; old path stays as fallback)
**Estimated effort:** 5–7 working days
**Depends on:** W2 (drawing pipeline output is stable; ReAct's `submit_motion` tool relies on it)
**Unblocks:** W4 (perception's `query_perception` tool registers in the ReAct registry built here)

---

## Goal

The current LLM behaviour, verified at `temperature=0.0` (`llm_config.py:177`) with a fixed prompt builder + `IntentRouter` mapping, is structured extraction — not reasoning. The user has explicitly named the symptoms: no chained commands, no recovery from rejection, no state awareness.

W3 adds a ReAct loop **on top of** the existing `IntentRouter`. The router itself is correct (it maps semantic IR to primitives) and stays. What changes is how the semantic IR is generated: from single-shot template fill to a Thought → Tool → Observation → Thought → … → Final-Command loop with structured tool calls, real-time robot state injection, and a tiered iteration ceiling.

`/llm_raw_command` and `/llm_intent` topics already exist (`llm_gateway_node.py:105,119`). W3 changes the logic behind `/llm_intent` to invoke ReAct; `/llm_raw_command` remains the previous-behaviour fallback for one full deprecation cycle.

---

## Why this comes before W4

W4 registers `query_perception` as a ReAct tool. If W3 has not produced the tool registry, W4 has nothing to register into. Stub the tool in W3 (returns `{"error": "perception_not_yet_implemented"}`); W4 fills it in.

---

## Discovery (paste raw output)

```bash
# A. Confirm the existing LLM gateway entry points
rg -n "/llm_intent|/llm_raw_command" src/llm_gateway/

# B. Existing IntentRouter, prompt builder, schema validator
rg -l "IntentRouter|intent_router" src/llm_gateway/
sed -n '1,40p' src/llm_gateway/llm_gateway/intent_router.py
sed -n '1,40p' src/llm_gateway/llm_gateway/prompt_builder.py

# C. Where temperature is read and used
rg -n "temperature" src/llm_gateway/

# D. Existing schema_validator usage (jsonschema, NOT Pydantic)
rg -n "jsonschema|schema_validator" src/llm_gateway/llm_gateway/

# E. The LLM client and its config
sed -n '1,80p' src/llm_gateway/llm_gateway/llm_client.py
sed -n '90,200p' src/llm_gateway/llm_gateway/llm_config.py

# F. State sources we will inject
ros2 topic list 2>/dev/null | rg -e 'joint_states|robot_status'
ros2 topic info /yaskawa/robot_status -v 2>/dev/null
ros2 topic info /yaskawa/joint_states -v 2>/dev/null

# G. Existing services that tools will wrap
ros2 service list 2>/dev/null | rg -e 'pose|plan|validate|execute'
ros2 action list 2>/dev/null

# H. Where async send_goal patterns already exist (so we follow style)
rg -n "send_goal_async|action_client" src/llm_gateway/

# I. Capability discovery (gripper / IO)
rg -l "gripper|GripperCommand|io_set" src/ hmi/
ros2 action list 2>/dev/null | rg -i 'gripper|io'
```

If F or G return empty, the system is not running. The agent can proceed with code-only verification but must flag in the PR that runtime tests need to be re-run when the system is up.

---

## Tasks

### W3.T1 — ReAct module skeleton

New directory: `src/llm_gateway/llm_gateway/react/`. Files:

- `react/__init__.py`
- `react/agent.py` — ReAct loop driver (~150 LOC)
- `react/tool_registry.py` — registry, tool base class (~80 LOC)
- `react/tools/get_current_pose.py`
- `react/tools/plan_motion.py`
- `react/tools/submit_motion.py`
- `react/tools/wait_for_state.py`
- `react/tools/set_speed.py`
- `react/tools/query_perception.py`            # stub for W4
- `react/tools/gripper_open.py`                # stub if gripper absent
- `react/tools/gripper_close.py`               # stub if gripper absent
- `react/state_injector.py` — pulls live state from ROS, formats for prompt
- `react/iteration_budget.py` — enforces tiered limits

Tool base class:

```python
from dataclasses import dataclass
from typing import Any, ClassVar
import json

@dataclass
class ToolResult:
    ok: bool
    payload: dict | None = None
    error: str | None = None

    def to_observation(self) -> str:
        if self.ok:
            return json.dumps({"ok": True, "payload": self.payload})
        return json.dumps({"ok": False, "error": self.error})


class Tool:
    name: ClassVar[str]
    description: ClassVar[str]
    input_schema: ClassVar[dict]   # jsonschema dict, validated before invoke
    is_motion: ClassVar[bool] = False  # True for plan_motion, submit_motion, set_speed, gripper_*
    is_readonly: ClassVar[bool] = False  # True for get_current_pose, query_perception, wait_for_state

    def invoke(self, args: dict, context: "AgentContext") -> ToolResult:
        raise NotImplementedError
```

Use `jsonschema.validate` for tool input validation. Do NOT migrate to Pydantic. The codebase contract is jsonschema (`setup.py:17`).

### W3.T2 — Tool implementations (real, except W4 stubs)

`get_current_pose`: calls existing service. Read-only. Returns the pose under `base_link`. SSOT for service name: keep using whatever name is in current `IntentRouter` flow — discovery G confirms.

`plan_motion`: takes `target` (PoseStamped or JointPositions), `planner` (default from existing planner mapping), `velocity_scale`, `acceleration_scale`. Calls the existing validation chain `/validate_command`. Returns `{"plan_id": "...", "valid": True, "estimated_duration_s": X}`. Does NOT execute. Motion-class tool.

`submit_motion`: takes `plan_id` (returned by `plan_motion`). Calls the `/execute_motion` action via `send_goal_async`. Returns immediately with `{"status": "SUBMITTED", "goal_id": "..."}` or `{"status": "REJECTED", "reason": "..."}` or `{"status": "TIMEOUT"}`. Subscribes to `/execute_motion/_action/feedback` to publish progress on `/llm_gateway/react_status`. Motion-class tool.

`wait_for_state`: takes `state` (one of `IDLE`, `MOVING`, `PLANNING`, `FAULT`), `timeout_s`. Polls `/yaskawa/robot_status`. Returns `{"reached": bool, "current_state": ..., "elapsed_s": ...}`. Read-only.

`set_speed`: takes `velocity_scale` (0.0–1.0). Goes through the existing safety validator (W1's reinforced one). Returns `{"applied": True, "velocity_scale": X}` or rejection. Motion-class tool because it changes future motion behaviour.

`query_perception`: stub. Returns `{"error": "perception_not_yet_implemented", "wave": "W4"}` always. Read-only class. W4 fills in body.

`gripper_open` / `gripper_close`: capability-conditional. Discovery I determines presence. If absent (probable, given empty discovery output), stub returns `{"error": "capability_unavailable", "capability": "gripper"}`. If present, wraps the existing action.

**`compute_arc_points`** (per Cascade C2): a **local geometry tool**, not a ROS call.

Args:
- `center: Point` (in `base_link` frame)
- `radius_m: float` (must be > 0)
- `start_angle_rad: float`
- `sweep_angle_rad: float` (positive = counterclockwise; magnitude > 0)
- `plane_normal: Vec3` (unit vector; default = +Z = `{0,0,1}` for table-plane arcs)

Returns:
```json
{
  "ok": true,
  "payload": {
    "start_pose": { ... PoseStamped ... },
    "auxiliary_pose": { ... PoseStamped ... },
    "target_pose": { ... PoseStamped ... }
  }
}
```

Computes three poses on the arc: `start_angle`, `start_angle + sweep_angle/2` (auxiliary), `start_angle + sweep_angle`. All in the plane defined by `center` + `plane_normal`. Orientation of the EEF: tangent to the arc, normal pointing along `plane_normal`. The exact orientation convention is documented in the docstring.

**Class:** `is_motion=False`, `is_readonly=True` (no ROS interaction; pure computation; cannot fail unless inputs are degenerate).

**Validation rules** (raised as `ValidationError`, returned as `ToolResult.error`):
- `radius_m <= 0`: reject.
- `sweep_angle_rad == 0`: reject (degenerate, no arc).
- `|sweep_angle_rad| > 2π`: reject (request a fresh sweep, not a wrap).
- `plane_normal.norm() < 1e-6`: reject (degenerate normal).

**Why this tool:** the LLM cannot reliably compute auxiliary poses for CIRC commands from natural language alone ("draw an arc from A to B going through C, radius 5cm"). With this tool, the LLM calls `compute_arc_points` first, then `plan_motion` with the `CIRC` primitive supplied with the three poses returned by the tool. The tool is local Python — no token cost, no ROS dependency, no latency.

**Tests:**

- 90° arc at radius 0.05 m, center origin, +Z normal: returns three poses on the unit circle scaled to 0.05 m, 90° apart.
- 0° sweep: rejected.
- Negative radius: rejected.
- Plane normal pointing in +Y: returns three poses in the X–Z plane.

**Integration test (W3-W4):** end-to-end NL prompt "draw a 90 degree arc with 5 cm radius starting at the current pose, sweeping counterclockwise" — ReAct calls `get_current_pose` → `compute_arc_points` → `plan_motion(primitive=CIRC, …)` → `submit_motion`. The full chain validates without operator intervention.

Each tool has a unit test under `src/llm_gateway/tests/test_react_tools/test_<name>.py`.

### W3.T3 — Iteration budget

File: `react/iteration_budget.py`.

```python
from dataclasses import dataclass

@dataclass
class IterationBudget:
    max_total: int = 5
    max_motion: int = 3
    max_readonly: int = 10
    max_repair: int = 1

@dataclass
class IterationCounters:
    total: int = 0
    motion: int = 0
    readonly: int = 0
    repair: int = 0

    def can_invoke(self, tool: "Tool", budget: IterationBudget) -> tuple[bool, str]:
        if self.total >= budget.max_total:
            return False, f"max_total exceeded ({self.total}/{budget.max_total})"
        if tool.is_motion and self.motion >= budget.max_motion:
            return False, f"max_motion exceeded ({self.motion}/{budget.max_motion})"
        if tool.is_readonly and self.readonly >= budget.max_readonly:
            return False, f"max_readonly exceeded ({self.readonly}/{budget.max_readonly})"
        return True, ""

    def record(self, tool: "Tool") -> None:
        self.total += 1
        if tool.is_motion:
            self.motion += 1
        if tool.is_readonly:
            self.readonly += 1
```

SSOT keys in `safety_rules.yaml`:

```yaml
llm:
  react:
    max_total_iterations:    5
    max_motion_iterations:   3
    max_readonly_iterations: 10
    max_repair_iterations:   1
    wall_clock_timeout_s:    30
    temperature:             0.2     # was 0.0; controlled exploration for ReAct
```

Note `temperature` is now SSOT-driven. `llm_config.py` reads it from YAML instead of hardcoding `0.0` at line 177.

### W3.T4 — State injector

File: `react/state_injector.py`.

Subscribes to `/yaskawa/joint_states`, `/yaskawa/robot_status`. Maintains the latest snapshot. Builds a structured dict for inclusion in the LLM prompt:

```yaml
robot_state:
  joints_rad: [<6 values>]
  joint_names: [joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t]
  mode: <IDLE|PLANNING|MOVING|FAULT|ESTOPPED>
  active_alarms: [...]
  last_action:
    tool: "<name>"
    status: "<ok|rejected|timeout>"
    error: "<message or null>"
  velocity_scale_active: <float>
  capabilities:
    gripper: <true|false>
    perception: false   # W4 flips to true
```

QoS: discovery F should confirm whether `joint_states` uses reliable (default per MotoROS2 config) and `robot_status` uses sensor_data. Set subscriber QoS to match. Mismatch causes silent message drop — verified earlier as the dominant `ApproximateTimeSynchronizer`-style failure mode.

### W3.T5 — Agent loop

File: `react/agent.py`.

```python
class ReActAgent:
    def __init__(self, llm_client, tool_registry, state_injector, budget, schema_validator):
        ...

    def run(self, user_text: str, request_id: str) -> dict:
        """
        Returns the final structured command (semantic IR) in the same shape
        IntentRouter currently expects. The router downstream is untouched.
        """
        state = self._state_injector.snapshot()
        history = []  # list of (role, content)
        counters = IterationCounters()

        while True:
            allowed, reason = counters.can_invoke_any(self._budget)
            if not allowed:
                return self._handoff(reason, history)

            messages = self._build_prompt(user_text, state, history)
            llm_response = self._llm.complete(messages)

            tool_call = self._parse_tool_call(llm_response)
            if tool_call is None:
                # Final command path
                semantic_ir = self._extract_semantic_ir(llm_response)
                ok, err = self._validate_semantic_ir(semantic_ir)
                if not ok:
                    if counters.repair < self._budget.max_repair:
                        counters.repair += 1
                        history.append(("observation", f"validation_error: {err}"))
                        continue
                    return self._handoff(f"semantic_ir invalid after repair: {err}", history)
                return semantic_ir

            tool = self._tool_registry.get(tool_call.name)
            if tool is None:
                history.append(("observation", f"unknown_tool: {tool_call.name}"))
                continue

            allowed, reason = counters.can_invoke(tool, self._budget)
            if not allowed:
                return self._handoff(f"budget exceeded for {tool.name}: {reason}", history)

            try:
                tool_input_ok = self._validate_tool_input(tool, tool_call.args)
            except Exception as e:
                history.append(("observation", f"tool_input_invalid: {e}"))
                counters.repair += 1 if counters.repair < self._budget.max_repair else 0
                continue

            result = tool.invoke(tool_call.args, self._context())
            counters.record(tool)
            state = self._state_injector.snapshot()
            history.append(("tool_call", tool_call))
            history.append(("observation", result.to_observation()))

            # Wall-clock guard
            if self._clock.elapsed_s() > self._budget.wall_clock_timeout_s:
                return self._handoff("wall_clock_timeout", history)
```

Two important details:

- The agent's job is to produce the same shape of structured command that `IntentRouter` currently consumes. ReAct does not reimplement the router. The router is correct and stays.
- For chained commands ("go home, then draw a circle"), the LLM emits a `MACRO` semantic IR with a `steps` list — same shape that `BLENDED_SEQUENCE` taught us to use for primitives. Each step is a primitive command. The router maps macros to a sequence of `submit_motion` calls or to a single `BLENDED_SEQUENCE` if all steps are LIN. **Macro definition is content for the prompt, not new code.** The agent uses existing primitives.

### W3.T6 — Wire into `llm_gateway_node`

File: `src/llm_gateway/llm_gateway/llm_gateway_node.py` (currently 869 LOC; on the file-size exception list).

The handler for `/llm_intent` (currently using `IntentRouter` directly) is replaced with: first run ReAct → produce semantic IR → pass to `IntentRouter` (existing) → existing safety/validate/execute flow.

The `/llm_raw_command` handler stays unchanged.

Mark the legacy single-shot path on `/llm_intent` with `# DEPRECATED: removal_date=<W3_merge+28d>, reason=replaced_by_react_in_W3`. Keep the code; do not delete in this wave.

### W3.T7 — Prompt updates

The prompt builder must teach the LLM:

- The available tools (name, description, input schema)
- The robot state structure
- The macro / chain-of-primitives convention
- That every tool call must be one JSON object per turn
- That the final response (no tool call) is the semantic IR

Use OpenAI function-calling format if the LLM client supports it (most do); otherwise, a JSON-only response convention parsed by `_parse_tool_call`.

Concrete prompt chunk to add (excerpt; full prompt in `src/llm_gateway/llm_gateway/prompts/react_system.txt`):

```
You are a robot task planner. You DO NOT control the robot directly.
You produce a structured command that the safety system reviews and the
motion system executes.

Available tools (one tool call per response, return JSON):
- get_current_pose() -> { pose: PoseStamped }
- plan_motion({ target, planner, velocity_scale }) -> { plan_id, valid, estimated_duration_s }
- submit_motion({ plan_id }) -> { status, goal_id }
- wait_for_state({ state, timeout_s }) -> { reached, current_state }
- set_speed({ velocity_scale }) -> { applied }
- query_perception({ class_filter }) -> { detections }
- gripper_open() / gripper_close({ force })

When you have enough information, respond WITHOUT a tool call, with the
final command JSON. The command must validate against the schema. Macros
("go home, then draw a circle") are expressed as a single command:
{
  "primitive_type": "MACRO",
  "steps": [ <command1>, <command2>, ... ]
}

Each step is itself a valid primitive command.
```

### W3.T8 — Schema additions for MACRO

If the existing schema does not have a `MACRO` primitive, add it:

```json
{
  "type": "object",
  "properties": {
    "primitive_type": {"const": "MACRO"},
    "steps": {
      "type": "array",
      "minItems": 1,
      "maxItems": 10,
      "items": { "$ref": "#/$defs/primitive_command" }
    }
  },
  "required": ["primitive_type", "steps"]
}
```

Cap `maxItems` at 10 so a hallucinating LLM cannot generate a 1000-step plan.

`IntentRouter` decides per macro whether to expand into a sequence of `submit_motion` calls (default) or coalesce purely-LIN runs into a `BLENDED_SEQUENCE` for smoother execution. The expansion logic lives in the router, not in the LLM prompt.

### W3.T9 — HMI checklist (per D3)

W3 adds the `MACRO` primitive type and (later, in W5) reduces HMI's local logic. For W3:

- `hmi/backend/services/intent_constants.py:16,33,43,65` lists primitive types. Add `"MACRO"`.
- `hmi/backend/services/intent_normalization.py:141` lists primitives that pass through normalization. Add `"MACRO"` if HMI surfaces it.
- `hmi/backend/services/supervisor_validation.py:123,178` lists motion primitives. Add or leave (decide based on whether HMI exposes macros).
- HMI frontend tests must pass.

If HMI does not surface MACROs (only direct primitive commands), the changes are minimal. Document the decision in the PR.

### W3.T10 — Tests

- `test_react_agent_basic.py`: NL "go home" → ReAct calls `get_current_pose` (optional) and emits `{primitive_type: HOME}`.
- `test_react_agent_chained.py`: NL "go home, then draw a circle of 5cm" → ReAct emits `{primitive_type: MACRO, steps: [HOME, BLENDED_SEQUENCE]}`.
- `test_react_agent_safety_reject.py`: ReAct attempts `plan_motion` with `velocity_scale=2.0`, gets rejected by validator, replans with `velocity_scale=0.1`, succeeds.
- `test_react_agent_budget_exceeded.py`: forced loop → budget exceeded → handoff.
- `test_react_agent_query_perception_stub.py`: ReAct calls `query_perception` → stub returns error → ReAct gracefully proceeds without perception.
- `test_react_state_injection.py`: state injector populates joint values from a mocked `/yaskawa/joint_states` topic.
- `test_react_iteration_budget.py`: tiered counters increment correctly.

---

## Verification

| # | Check | Pass criterion |
|---|---|---|
| 1 | `rg -n "/llm_intent" src/llm_gateway/llm_gateway/llm_gateway_node.py` | Same line, but the handler now invokes `ReActAgent.run` |
| 2 | `rg -n "/llm_raw_command" src/llm_gateway/llm_gateway/llm_gateway_node.py` | Unchanged from W2 |
| 3 | NL prompt "go home" → `/llm_intent` | Returns valid HOME primitive command, single iteration |
| 4 | NL prompt "draw a circle of 5cm and return home" | Returns MACRO with two steps; downstream produces BLENDED_SEQUENCE + HOME |
| 5 | Forced safety reject (velocity_scale=2.0) | ReAct repairs once, then succeeds; counters: repair=1, motion≤3 |
| 6 | Forced budget overrun | Returns handoff with budget reason |
| 7 | `query_perception` called | Returns stub error; agent's loop continues |
| 8 | `time.sleep` audit on react_tools | `rg "time\.sleep|asyncio\.sleep" src/llm_gateway/llm_gateway/react/` returns 0 hits |
| 9 | jsonschema validation on every tool input/output | No raw exceptions escape into the LLM loop; all wrapped as observations |
| 10 | `temperature` from SSOT | `llm_config.py` reads YAML; `temperature=0.0` no longer hardcoded |
| 11 | `colcon test --packages-select llm_gateway` | All new tests pass; old tests pass; deprecation tag visible |
| 12 | HMI backend tests | Green |
| 13 | CI duplication check | No new duplicate blocks ≥30 LOC |
| 14 | compute_arc_points with sweep=0 | Rejected with degenerate_arc |
| 15 | NL prompt "draw a 90 deg arc, radius 5cm" | ReAct emits compute_arc_points then plan_motion(CIRC) then submit_motion; full chain validates |
---

## DON'T

- Do not delete `IntentRouter` or modify its routing logic. It works. ReAct sits on top.
- Do not let any tool publish directly to `/joint_trajectory_controller/*`. R3 is absolute.
- Do not rename `submit_motion` to `execute_motion`. The semantic distinction matters: the LLM submits to the safety chain, the chain decides execution.
- Do not migrate from `jsonschema` to `Pydantic`. The codebase contract is jsonschema. Pydantic adds dependency surface for no win.
- Do not hardcode `temperature=0.0`. SSOT.
- Do not allow `time.sleep` inside tools. All waits go through `wait_for_state` (which uses `Future.result(timeout=...)` against a polling loop scheduled by the executor).
- Do not let the LLM emit raw ROS topic names or service paths. Tools are the only ROS surface the LLM sees.
- Do not bundle the file-size split of `llm_gateway_node.py` (869 LOC) with this wave. W6.
- Do not let `compute_arc_points` issue ROS service calls. It must remain a pure local function. ROS interaction is reserved for tools marked `is_motion=True` or `is_readonly=True with side-effects`.
- Do not let the LLM try to compute arc auxiliary poses by hand from NL — always route through `compute_arc_points`.
---

## Output artefacts

- `src/llm_gateway/llm_gateway/react/` — all new files under this directory
- `src/llm_gateway/llm_gateway/llm_gateway_node.py` — diff: `/llm_intent` handler invokes ReActAgent
- `src/llm_gateway/llm_gateway/llm_config.py` — diff: temperature from YAML
- `src/llm_gateway/llm_gateway/prompts/react_system.txt` — new prompt template
- `src/safety/config/safety_rules.yaml` — diff: `llm.react.*` SSOT keys
- `src/llm_gateway/tests/test_react_*` — new tests
- `hmi/backend/services/intent_constants.py` — diff: add MACRO if surfaced
- `MIGRATION-W3.md`

---

## Rollback procedure

```bash
# Quickest: SSOT toggle
# Edit safety_rules.yaml: llm.react.enabled: false
# /llm_intent reverts to single-shot IntentRouter behaviour.
# /llm_raw_command was already unchanged.

# Full revert
git revert -m 1 <W3 merge commit>
```

The legacy `/llm_intent` path code is still in the file (deprecated, not deleted). Restoring is a feature flag away.

---

## Risk notes

- **LLM cost**: ReAct calls 2–5x more tokens than single-shot. For the GP4 use case (low-frequency commands), this is fine. If cost becomes an issue, tighten `max_total_iterations` or add caching of tool results within a request.
- **Chained command intent ambiguity**: "go home, then draw a circle" is unambiguous; "pick up the box and put it back" is not (which box? back where?). The `query_perception` stub keeps this in the W4 backlog. Until W4, the LLM tells the user it cannot resolve.
- **Schema-validation race**: in fast successive `/llm_intent` calls, two ReAct sessions could deplete each other's budget. Each request gets its own `IterationCounters` instance — confirm threading model in `llm_gateway_node.py`.
- **`temperature` change**: 0.2 (vs prior 0.0) changes test determinism. Tests pin temperature to `0.0` for reproducibility; production uses SSOT.
- **HMI surface change**: only if MACROs are exposed. If HMI does not show macros to the user, no frontend change needed.

---

## Stop signal

End of W3. Do not proceed to W4 until:

- W3 PR merged.
- Three NL prompts demonstrably pass through ReAct end-to-end (logs in PR): a simple primitive, a chained macro, a recovery-after-rejection.
- `/llm_raw_command` regression tests still pass (deprecated path is functional).
- HMI tests still pass.

State explicitly: `End of W3. Awaiting review before W4.`

---

**Reliability tag:** `[VERIFIED]` for the entry-point file:line targets in `llm_gateway_node.py` and the schema/jsonschema contract. `[NEEDS-VALIDATION]` for the tool-input schema details — the agent must read the existing `IntentRouter` semantic IR shape to ensure ReAct's output is contract-compatible. `[NEEDS-VALIDATION]` for the LLM client's function-calling support; if the chosen LLM cannot use OpenAI-style function calling, the prompt must use the JSON-only convention and the parser is slightly more involved.
