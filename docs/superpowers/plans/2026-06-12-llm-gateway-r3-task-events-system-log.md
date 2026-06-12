# LLM Gateway R3 — task_events + System Log Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a unified `/llm_gateway/task_events` stream (fixed schema from spec §6), bridge robot/safety/motion status into the same schema, and surface it in the HMI as a filterable System Log with a live robot-status strip and JSONL export.

**Architecture:** `task_runtime._publish_event` already emits the exact §6 schema dict. R2 wired it to a no-op `_runtime_event_sink`. R3 replaces that sink with a real `std_msgs/String` JSON publisher on `/llm_gateway/task_events`, adds a thin mapping from `robot_status`/`joint_states`/`_emit_trace` into the same schema, then has the HMI backend ROS adapter subscribe and forward to the existing WebSocket stream. Frontend adds a System Log panel.

**Tech Stack:** rclpy, std_msgs/String, FastAPI WS (`/api/hmi/stream`), React 18 + Vite + TS. Phase R3 of `docs/superpowers/specs/2026-06-12-llm-gateway-remediation-design.md` §6.

---

## Verified preconditions

- `task_runtime._publish_event` emits `{ts, level, source, category, event, detail, data}` already (spec §6 schema). Source is hard-coded `"runtime"`.
- After R2, the node has `_runtime_event_sink(event: dict)` (currently logs). It is passed as `event_callback` to `TaskRuntime`.
- Node already has `_emit_trace(...)` for parse/dispatch traces, and `_state_robot_status_callback` / `_state_joint_state_callback`.
- HMI backend: `hmi/backend/ros/adapter.py` (ROS adapter), `telemetry_bridge_service.py`, WS in `hmi/backend/api/app.py` (`/api/hmi/stream`). No `task_events` consumer exists yet.

## Event schema (fixed — do not vary)

```json
{"ts":"15:02:01.123","level":"INFO|WARN|ERR","source":"gateway|runtime|safety|motion|hw_adapter|perception|system",
 "category":"TASK|MOTION|PERCEPTION|HARDWARE|IO|SAFETY|SYSTEM","event":"short_event","detail":"human readable","data":{}}
```

## File structure

| File | Change |
|------|--------|
| `src/llm_gateway/llm_gateway/llm_gateway_node.py` | Add `/llm_gateway/task_events` publisher; `_runtime_event_sink` publishes; add `_emit_task_event(...)` helper; map robot_status/safety/motion into schema |
| `src/llm_gateway/tests/test_task_events.py` | Create — event publishes valid schema; mapping helpers produce valid events |
| `hmi/backend/ros/adapter.py` | Subscribe `/llm_gateway/task_events`, forward parsed dicts to the stream |
| `hmi/backend/api/app.py` / WS layer | Relay task events to `/api/hmi/stream` clients |
| `hmi/frontend/src/components/system-log/` | Create — SystemLog panel (filter/search/expand/export) + robot status strip |

---

## Task 1: `/llm_gateway/task_events` publisher in the node

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
- Test: `src/llm_gateway/tests/test_task_events.py`

- [ ] **Step 1: Write the failing test**

Create `src/llm_gateway/tests/test_task_events.py`:
```python
import json
from llm_gateway.llm_gateway_node import LLMGatewayNode


def test_runtime_event_sink_publishes_json_string():
    node = object.__new__(LLMGatewayNode)
    published = []
    class _Pub:
        def publish(self, msg): published.append(msg.data)
    node._task_events_pub = _Pub()

    node._runtime_event_sink({
        "ts": "10:00:00.000", "level": "INFO", "source": "runtime",
        "category": "TASK", "event": "task_start", "detail": "Starting", "data": {"task_id": "t"},
    })

    assert len(published) == 1
    decoded = json.loads(published[0])
    assert decoded["category"] == "TASK"
    assert decoded["event"] == "task_start"
    assert decoded["data"]["task_id"] == "t"


def test_emit_task_event_builds_valid_schema():
    node = object.__new__(LLMGatewayNode)
    published = []
    class _Pub:
        def publish(self, msg): published.append(msg.data)
    node._task_events_pub = _Pub()

    node._emit_task_event("SAFETY", "validate_rejected", "blocked by workspace bound",
                          level="WARN", source="safety", data={"rule": "x_max"})

    decoded = json.loads(published[0])
    assert set(decoded) == {"ts", "level", "source", "category", "event", "detail", "data"}
    assert decoded["level"] == "WARN" and decoded["source"] == "safety"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/hieu2/gp4_ws/src/llm_gateway && python -m pytest tests/test_task_events.py -q`
Expected: FAIL — `_runtime_event_sink` does not publish / `_emit_task_event` missing.

- [ ] **Step 3: Write minimal implementation**

In `llm_gateway_node.py`:
1. In `__init__`, create the publisher (next to other publishers):
```python
self._task_events_pub = self.create_publisher(String, "/llm_gateway/task_events", 10)
```
2. Replace the R2 no-op `_runtime_event_sink` with a publishing version:
```python
def _runtime_event_sink(self, event: dict) -> None:
    pub = getattr(self, "_task_events_pub", None)
    if pub is None:
        return
    msg = String()
    msg.data = json.dumps(event, ensure_ascii=False, separators=(",", ":"))
    pub.publish(msg)

def _emit_task_event(self, category, event, detail, *, level="INFO", source="gateway", data=None):
    import datetime
    self._runtime_event_sink({
        "ts": datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
        "level": level, "source": source, "category": category,
        "event": event, "detail": detail, "data": data or {},
    })
```
(`String` and `json` are already imported in the node; verify with `grep -n "import json" llm_gateway_node.py` and `grep -n "from std_msgs.msg import" llm_gateway_node.py`.)

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_task_events.py -q`
Expected: PASS (2 passed).

- [ ] **Step 5: Full suite + build + commit**

```bash
python -m pytest tests/ -q
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_task_events.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "feat(llm_gateway): R3 publish /llm_gateway/task_events with fixed schema"
```

---

## Task 2: Bridge robot_status / safety / motion into task_events

**Files:**
- Modify: `src/llm_gateway/llm_gateway/llm_gateway_node.py`
- Test: `src/llm_gateway/tests/test_task_events_bridge.py`

- [ ] **Step 1: Write the failing test**

Create `src/llm_gateway/tests/test_task_events_bridge.py`:
```python
import json
from llm_gateway.llm_gateway_node import LLMGatewayNode


class _CapturePub:
    def __init__(self): self.events = []
    def publish(self, msg): self.events.append(json.loads(msg.data))


def _node_with_pub():
    node = object.__new__(LLMGatewayNode)
    node._task_events_pub = _CapturePub()
    return node


def test_robot_status_alarm_emits_hardware_event():
    node = _node_with_pub()
    class _Status:
        in_error = True; error_code = 4012; e_stopped = False
        in_motion = False; servo_on = True; mode = 2
    node._emit_robot_status_event(_Status())
    ev = node._task_events_pub.events[-1]
    assert ev["category"] == "HARDWARE"
    assert ev["data"]["error_code"] == 4012
    assert ev["level"] == "ERR"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_task_events_bridge.py -q`
Expected: FAIL — `_emit_robot_status_event` missing.

- [ ] **Step 3: Write minimal implementation**

In `llm_gateway_node.py` add (and call it from the existing `_state_robot_status_callback`, only emitting on change to avoid spam — track last status tuple):
```python
def _emit_robot_status_event(self, status) -> None:
    in_error = bool(getattr(status, "in_error", False))
    e_stopped = bool(getattr(status, "e_stopped", False))
    level = "ERR" if (in_error or e_stopped) else "INFO"
    self._emit_task_event(
        "HARDWARE", "robot_status",
        f"alarm={getattr(status,'error_code',0)} estop={e_stopped} servo={getattr(status,'servo_on',None)}",
        level=level, source="hw_adapter",
        data={
            "error_code": int(getattr(status, "error_code", 0)),
            "e_stopped": e_stopped, "in_error": in_error,
            "in_motion": bool(getattr(status, "in_motion", False)),
            "servo_on": getattr(status, "servo_on", None),
            "mode": getattr(status, "mode", None),
        },
    )
```
Check the real `RobotStatus` field names first: `grep -nE "status\.\w+" llm_gateway_node.py` around `_state_robot_status_callback` and match them (the node already reads `e_stopped` at `:1188`). Wire a `_set_runtime_stop(True)` here too when `e_stopped` — this is the STOP source R2 referenced. Emit only when the status tuple changes from the last one.

- [ ] **Step 4: Run + full suite + build + commit**

```bash
python -m pytest tests/test_task_events_bridge.py tests/ -q
cd /home/hieu2/gp4_ws && colcon build --packages-select llm_gateway --symlink-install && source install/setup.bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add src/llm_gateway/llm_gateway/llm_gateway_node.py src/llm_gateway/tests/test_task_events_bridge.py
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "feat(llm_gateway): R3 bridge robot_status into task_events (HARDWARE)"
```

---

## Task 3: HMI backend — subscribe task_events and forward to WS

**Files:**
- Read first: `hmi/backend/ros/adapter.py`, `hmi/backend/api/app.py` (WS broadcast mechanism), `hmi/backend/services/telemetry_bridge_service.py`
- Modify: `hmi/backend/ros/adapter.py` (+ the WS relay layer)
- Test: `hmi/backend/tests/` (match existing backend test layout)

- [ ] **Step 1: Map the existing stream**

Run:
```bash
grep -rnE "create_subscription|add_subscriber|broadcast|stream|/api/hmi/stream|async def" hmi/backend/ros/adapter.py hmi/backend/api/app.py | head -40
```
Identify: (a) where the adapter creates ROS subscriptions, (b) the callback that pushes a dict onto the WS broadcast queue. Reuse that exact path — do not invent a new transport.

- [ ] **Step 2: Write the failing test**

Create a backend test (match the directory/framework the other `hmi/backend` tests use — `grep -rl "def test_" hmi/backend | head`). Test that a received `/llm_gateway/task_events` JSON string is parsed and enqueued to the stream:
```python
def test_task_event_message_is_forwarded_to_stream(fake_adapter):
    fake_adapter.on_task_event_msg(_string_msg('{"ts":"10:00:00.000","level":"INFO",'
        '"source":"runtime","category":"TASK","event":"task_start","detail":"x","data":{}}'))
    assert fake_adapter.last_stream_payload["channel"] == "task_event"
    assert fake_adapter.last_stream_payload["event"]["category"] == "TASK"
```
(Adapt fixture/constructor to the real adapter API discovered in Step 1.)

- [ ] **Step 3: Run test to verify it fails**

Run the backend test suite for that file. Expected: FAIL — `on_task_event_msg` missing.

- [ ] **Step 4: Implement the subscription + forward**

In `adapter.py`, add a `std_msgs/String` subscription to `/llm_gateway/task_events`; in its callback `json.loads` the data and push `{"channel": "task_event", "event": <dict>}` onto the same broadcast mechanism telemetry uses. Guard against malformed JSON (drop + log, never crash the adapter).

- [ ] **Step 5: Run + commit**

```bash
# run the backend test suite per its README/conftest
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add hmi/backend
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "feat(hmi): R3 forward /llm_gateway/task_events to the WS stream"
```

---

## Task 4: HMI frontend — System Log panel + status strip + export

**Files:**
- Read first: `hmi/frontend/src/` stream hook (how `/api/hmi/stream` messages are consumed)
- Create: `hmi/frontend/src/components/system-log/SystemLog.tsx`, `LogRow.tsx`, `RobotStatusStrip.tsx`, `system-log.css`
- Modify: the stream hook to route `channel === "task_event"` into a log store; mount `SystemLog` in the main view

- [ ] **Step 1: Map the stream hook**

Run: `grep -rnE "api/hmi/stream|onmessage|WebSocket|useStream|channel" hmi/frontend/src | head -30`
Identify the hook that receives WS messages and where to add a `task_event` branch.

- [ ] **Step 2: Log store + ingest**

Add a bounded log store (e.g. last 2000 events, ring buffer) fed by `channel === "task_event"`. Keep robot-status events (`category === "HARDWARE"`) ALSO mirrored into a separate `robotStatus` slice for the strip — do not mix the live strip with the scrolling log (spec §6).

- [ ] **Step 3: SystemLog panel**

`SystemLog.tsx`: virtualized list of `LogRow`; filter controls for `category` (TASK/MOTION/PERCEPTION/HARDWARE/IO/SAFETY/SYSTEM), `level` (INFO/WARN/ERR), `source`; text search over `event`+`detail`; click a row to expand its `data` JSON. Color rows by level. `RobotStatusStrip.tsx`: live TCP/joints/servo/alarm from the `robotStatus` slice, updated in place. Add an "Export JSONL" button that downloads the current filtered log as newline-delimited JSON.

- [ ] **Step 4: Verify in the running HMI**

Run backend + frontend (`README` commands), trigger a FactoryTask, confirm the log streams events live, filters work, the status strip updates independently, and export produces valid JSONL. Use the `run` skill / Playwright screenshot at 1440 width per the web testing rules.

- [ ] **Step 5: Commit**

```bash
PRE_COMMIT_ALLOW_NO_CONFIG=1 git add hmi/frontend
PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit -m "feat(hmi): R3 System Log panel + robot status strip + JSONL export"
npx gitnexus analyze
```

---

## Done criteria for R3

- [ ] `/llm_gateway/task_events` publishes the fixed §6 schema for runtime events; robot_status mapped into HARDWARE events; e_stop sets the R2 runtime stop flag.
- [ ] HMI backend forwards task events to `/api/hmi/stream`; malformed messages are dropped, not fatal.
- [ ] HMI System Log panel: filter (category/level/source) + search + expand `data` + JSONL export; separate live robot-status strip.
- [ ] llm_gateway suite green + build green; backend + frontend builds green; GitNexus reindexed.
- [ ] Docs: update `.claude/rules/llm-gateway.md` + root `CLAUDE.md` for the new `/llm_gateway/task_events` topic (per `when-to-update-claude-docs.md`).

After R3 lands, write/execute R4 (thin node + remove legacy dispatch).
