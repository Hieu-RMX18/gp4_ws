# GP4 HMI — Telemetry Bridge v1 + Supervisor Command Ingress v2

## Current state

- Telemetry bridge v1 remains frozen for runtime-state, freshness, heartbeat, and schema-version behavior.
- HMI v2 adds a supervisor-owned command ingress path on top of that baseline.
- Command ingress is operational on both sim and hardware.
- Hardware telemetry validation Phase A is **confirmed** (see `HARDWARE_TELEMETRY_VALIDATION.md`).
- Confirmed commands validate through `/validate_command` and dispatch `/execute_motion`.
- Full pipeline (HMI → safety → MoveIt → hw_adapter → YRC1000micro) commissioned and running on real hardware.
- Safety guards (JointPositionGuard, ManipulabilityGuard, WristFlipGuard) are wired into the motion pipeline downstream of HMI command dispatch.
- CI no longer masks HMI backend/frontend test failures; 7 pre-existing backend tests fail due to missing generated interfaces (not a new regression).

Primary architecture notes:

- `hmi/HARDWARE_TELEMETRY_VALIDATION.md`
- `docs/hmi/HMI_ROS_INTERFACES.md`

## Architecture

```
hmi/frontend (React 18 + Vite + TypeScript)
    │  REST + WebSocket via Vite proxy
    ▼
hmi/backend/api (FastAPI)
    │  rclpy native (NOT rosbridge)
    ▼
ROS 2 topics / services / actions
```

### Backend services

| Service | File | Responsibility |
|---------|------|---------------|
| Supervisor | `supervisor_service.py` | Top-level supervisor orchestrator |
| Supervisor Execution | `supervisor_execution.py` | Execute motion via `/execute_motion` action |
| Supervisor Lifecycle | `supervisor_lifecycle.py` | State machine for command lifecycle |
| Supervisor Sequence | `supervisor_sequence.py` | Multi-step sequence execution ("park" verb removed) |
| Supervisor Submission | `supervisor_submission.py` | ReviewIntent submission and command routing |
| Supervisor Validation | `supervisor_validation.py` | Pre-dispatch command validation |
| Supervisor Views | `supervisor_views.py` | Read-only supervisor state queries |
| Telemetry Bridge | `telemetry_bridge_service.py` | Telemetry snapshot, freshness, WebSocket stream |
| Jog Service | `jog_service.py` | Real-time joint jogging dispatch |
| Session Lock | `session_lock_service.py` | Operator session lease management |
| Audit | `audit_service.py` | Execution audit trail |
| Intent Resolution | `intent_resolution.py` | Natural language → structured intent |
| Intent Normalization | `intent_normalization.py` | Unit/defaults normalization |
| Intent Constants | `intent_constants.py` | Shared intent definitions |

### Frontend components

| Component | File | Description |
|-----------|------|-------------|
| GP4HMI | `GP4HMI.tsx` | Main HMI layout shell |
| Topbar | `gp4-hmi/Topbar.tsx` | Header bar with status indicators |
| ChatPanel | `gp4-hmi/ChatPanel.tsx` | Natural language command input |
| CommandComposer | `gp4-hmi/CommandComposer.tsx` | Structured command builder |
| CommandPipelinePanel | `gp4-hmi/CommandPipelinePanel.tsx` | Pipeline stage visualization |
| ControlLeasePanel | `gp4-hmi/ControlLeasePanel.tsx` | Operator lease acquire/release |
| JointMonitor | `gp4-hmi/JointMonitor.tsx` | Joint position readout |
| TcpPosePanel | `gp4-hmi/TcpPosePanel.tsx` | TCP XYZ/RPY display |
| TelemetrySources | `gp4-hmi/TelemetrySources.tsx` | Source freshness status |
| SystemMetrics | `gp4-hmi/SystemMetrics.tsx` | System health metrics |
| SystemLog | `gp4-hmi/SystemLog.tsx` | Real-time log viewer |
| QuickCommands | `gp4-hmi/QuickCommands.tsx` | Preset command shortcuts |
| JogPendant | `JogPendant.tsx` | Real-time joint jogging UI |
| RuntimeStateBanner | `RuntimeStateBanner.tsx` | Runtime state indicator |

## Backend dependencies

```bash
pip3 install --user -r hmi/requirements.txt
```

## Run the FastAPI bridge

ROS-aware local dev:

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
python3 -m uvicorn hmi.backend.api.app:app --host 127.0.0.1 --port 8000
```

Shortcut:

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
python3 -m hmi.backend.api
```

## Frontend dev server

```bash
cd /home/hieu2/gp4_ws/hmi/frontend
npm install
npm run dev
```

The Vite proxy forwards:

- `http://127.0.0.1:5173/api/*` -> `http://127.0.0.1:8000/api/*`
- `ws://127.0.0.1:5173/api/*` -> `ws://127.0.0.1:8000/api/*`

## Verification commands

REST:

```bash
curl "http://127.0.0.1:8000/api/hmi/snapshot?session_id=session-dev&operator_id=operator-dev"
curl "http://127.0.0.1:8000/api/hmi/runtime-state?session_id=session-dev&operator_id=operator-dev"
curl "http://127.0.0.1:8000/api/hmi/connection-state"
curl "http://127.0.0.1:8000/api/hmi/lease-state?session_id=session-dev&operator_id=operator-dev"
curl -X POST "http://127.0.0.1:8000/api/hmi/lease/acquire" \
  -H "Content-Type: application/json" \
  -d '{"sessionId":"session-dev","operatorId":"operator-dev","requestedRole":"controller"}'
```

WebSocket:

```bash
python3 - <<'PY'
import asyncio
import websockets

async def main() -> None:
    uri = "ws://127.0.0.1:8000/api/hmi/stream?session_id=session-dev&operator_id=operator-dev"
    async with websockets.connect(uri) as websocket:
        print(await websocket.recv())

asyncio.run(main())
PY
```

Hardware telemetry validation (read-only):

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
python3 hmi/tools/hardware_telemetry_validation.py \
  --duration-sec 120 \
  --output /tmp/gp4_hardware_telemetry_report.json
```

WebSocket heartbeat:

- server sends an explicit `heartbeat` event every `5 s` when no semantic snapshot changed
- client treats missing traffic for `15 s` as stale and closes the socket to force reconnect
- client reconnect uses bounded exponential backoff with jitter:
  - base `500 ms`
  - cap `8000 ms`

## Telemetry freshness fields

`/snapshot`, `/runtime-state`, and `/connection-state` include:

- `telemetryState`: `fresh | stale | unavailable`
- `telemetrySources[]` with:
  - `topic`
  - `lastSeenAt`
  - `freshnessThresholdSec`
  - `freshnessState`
  - `preferred`
  - `active`

Deterministic interpretation:

- `transportState=disconnected` => backend cannot currently trust ROS telemetry path
- `telemetryState=stale` => backend is up but one or more active sources are stale
- `runtime.systemState=SAFETY_BLOCKED` => safety gate is actively blocking while transport may still be up

### Sim-mode freshness baseline

In sim mode, the supervisor treats these as freshness-critical:

- `gateway_status`
- `readiness`
- `supervisor_alerts`
- the active sim joint-state source

These are non-blocking unless the backend explicitly marks them active:

- `llm_debug`
- `llm_command`

These stay visible but are not freshness-critical in sim mode:

- `robot_status`
- inactive joint-state fallback/primary sources

If any freshness-critical sim source is stale, command-capable actions fail closed.

## Storage policy

Telemetry snapshots are intentionally compact:

- only semantic changes are persisted
- heartbeat events are **not** persisted
- default retention:
  - keep last `50,000` telemetry snapshots
  - prune telemetry snapshots older than `7` days

Replay-oriented records remain separate:

- `commands`
- `command_events`
- `runtime_events`
- `state_transitions`

## Storage growth assumptions

For telemetry bridge v1:

- one telemetry snapshot row per semantic state change, not per poll tick
- one transition row per meaningful state/freshness change
- growth is dominated by operator-visible runtime changes, not idle polling

## Contract verification

Backend responses are validated by FastAPI response models and dedicated contract models in:

- `hmi/backend/api/contracts.py`

Schema compatibility policy:

- current supported schema: `telemetry.v1`
- backend emits `schemaVersion` on REST and WebSocket payloads
- frontend rejects incompatible schema versions explicitly

Unit verification:

```bash
# Backend tests (requires generated ROS interfaces in install/)
pytest hmi/backend -v

# Frontend build check
cd /home/hieu2/gp4_ws/hmi/frontend && npm run build
```

> **Note:** 7 backend tests currently fail due to missing generated ROS interfaces
> when run outside a full `colcon build` environment. This is a pre-existing issue
> that was previously masked by CI `|| true`. It is not a new regression.

## Runtime validation

The frozen read-only telemetry bridge v1 can be validated end-to-end in one command:

```bash
cd /home/hieu2/gp4_ws
hmi/tools/run_runtime_validation.sh
```

What the harness does:

- chooses isolated `ROS_DOMAIN_ID` values automatically
- starts and stops the FastAPI bridge per scenario
- starts and stops read-only ROS telemetry fixtures cleanly
- validates REST and WebSocket behavior with `hmi/tools/bridge_probe.py`
- writes separate logs for each scenario under a temporary log directory
- exits non-zero if any scenario fails

Expected runtime:

- about **2 minutes**
- the slowest scenario is the full disconnect check because it waits for the `30 s` LLM freshness window to expire

Validation components:

- `hmi/tools/ros_telemetry_fixture.py`
  - publishes read-only telemetry scenarios only
  - never opens `/llm_text_input`, `/validate_command`, or `/execute_motion`
- `hmi/tools/bridge_probe.py`
  - probes `GET /api/hmi/snapshot`
  - probes `WS /api/hmi/stream`
  - asserts expected bridge state from live runs

How to interpret failures:

- inspect the per-scenario log directory printed by the harness summary
- `bridge.log`
  - bridge startup failure
  - ROS callback exception
  - transport/runtime derivation problems
- `fixture.log`
  - ROS publisher startup issue
  - QoS mismatch
  - scenario timing/publish problems
- `*.err`
  - unmet probe expectations

Timing-sensitive scenarios:

- `partial_stale_readiness`
- `joint_failover`
- `lost_conn_recover`
- `full_disconnect_after_gap`
- `burst_joint_states_ws`

These rely on freshness windows and scheduler timing, so they are the first to show machine-load or timing jitter issues.

## Hardware telemetry validation

Phase A telemetry validation is **confirmed** on the live hardware stack.
Hardware command execution remains gated at runtime by mode selection,
controller lease, operator confirmation, command validation, and execution preflight.

See `hmi/HARDWARE_TELEMETRY_VALIDATION.md` for the full validation worksheet and
capture tool usage.

Capture command (read-only, does not enable execution):

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
python3 hmi/tools/hardware_telemetry_validation.py \
  --duration-sec 120 \
  --output /tmp/gp4_hardware_telemetry_report.json
```
