# HMI v2 command ingress on top of telemetry bridge v1

## Scope boundary

This document is the current architecture note for the HMI command-capable path.

- Telemetry bridge v1 remains the source of truth for runtime state, freshness, heartbeats, and schema versioning.
- HMI v2 adds only a supervisor-owned command ingress workflow.
- MotoROS2 and real hardware execution remain fail-closed by default.
- The backend execution boundary dispatches ROS motion through `/validate_command` and `/execute_motion` in:
  - `sim` mode (unchanged), and
  - `hardware` mode **only when dual gate + preflight pass**.
- The live hardware validation gate is tracked in `hmi/HARDWARE_TELEMETRY_VALIDATION.md`.

## Sim-mode freshness policy

The backend treats freshness as a supervisor gate, not just a UI hint.

### Freshness rules

- `transportState=disconnected` -> telemetry path unavailable; command-capable actions fail closed.
- `telemetryState=stale` -> at least one **active freshness-critical** source is stale; command-capable actions fail closed.
- `telemetryState=fresh` -> all active freshness-critical sources are fresh.
- Event-driven topics stay non-blocking unless they are explicitly marked active.

### Freshness-critical sources in sim mode

These sources are active in sim mode and must stay fresh before the supervisor will allow a command to reach confirmation/execution gating:

- `gateway_status`
- `readiness`
- `supervisor_alerts`
- the active joint-state source (`joint_states_fallback` in sim mode today)

### Optional or mode-dependent sources

These are visible for diagnostics but do not block sim-mode freshness unless they become the active source for the current mode:

- `robot_status` in sim mode
- `joint_states_primary` while sim is using the fallback topic

### Event-driven, non-blocking sources

These are intentionally non-blocking unless the backend explicitly marks them active in the future:

- `llm_debug`
- `llm_command`

## Supervisor-owned command workflow

```text
Frontend
-> POST /api/hmi/commands/intent
-> FastAPI transport boundary
-> SupervisorService
-> parse + validation + risk assessment
-> confirmation gate
-> backend-owned execution boundary
-> WorkspaceRosAdapter
-> /validate_command
-> /execute_motion
```

### Hard rules enforced in code

- The browser never calls ROS directly.
- The browser never chooses ROS topics, services, or actions.
- All lease, validation, confirmation, and execution decisions are backend-owned.
- The supervisor fails closed on:
  - missing or invalid control lease
  - `ESTOP`, `FAULT`, `LOST_CONN`, `SAFETY_BLOCKED`
  - stale freshness-critical telemetry
  - hardware dual-gate failure (`HMI_ENABLE_HARDWARE_COMMANDS` + `hmi/data/hardware_gate.json`)
  - hardware preflight failure (required source freshness, active preferred joint source, command interfaces ready)
  - unsupported or ambiguous intent
  - plan fingerprint mismatch
  - expired confirmation window
  - runtime mode mismatch or unknown mode

## API surface

- `POST /api/hmi/lease/acquire`
- `POST /api/hmi/lease/renew`
- `POST /api/hmi/lease/release`
- `POST /api/hmi/commands/intent`
- `GET /api/hmi/commands/{commandId}`
- `GET /api/hmi/commands`
- `POST /api/hmi/commands/{commandId}/confirm`
- `POST /api/hmi/commands/{commandId}/cancel`
- `GET /api/hmi/replay`
- `GET /api/hmi/replay/{commandId}`

No public endpoint exposes raw ROS execution.

## Command lifecycle

```text
RECEIVED
-> PARSING
-> VALIDATING
-> NEEDS_CONFIRMATION
-> CONFIRMED
-> EXECUTION_REQUESTED
-> EXECUTING
-> SUCCEEDED | FAILED | REJECTED | CANCELLED | EXPIRED
```

Current sim behavior:

- valid commands stop at `NEEDS_CONFIRMATION` until the operator confirms
- on confirm, the supervisor records `CONFIRMED` and `EXECUTION_REQUESTED`
- the sim execution adapter calls `/validate_command` first and fails closed if validation rejects the payload
- if validation passes, the adapter dispatches `/execute_motion` and the command transitions through `EXECUTING`
- the command ends in `SUCCEEDED`, `FAILED`, or `CANCELLED` from the action result

Approval is owned solely by the HMI supervisor lease/confirm flow. The `ExecuteMotion` action no longer carries an approval flag; any plan-only path goes through `ValidateCommand.srv` first.

## Capability model

The snapshot capability fields now distinguish between:

- `readOnly`
- `commandIngressAvailable`
- `confirmationAvailable`
- `executionAllowed`
- `replayAvailable`
- `simOnly`

Current default behavior:

- sim mode: command ingress and confirmation are available
- hardware mode: command ingress stays read-only until dual gate passes; then confirmation/execution become available
- unknown mode: bridge stays read-only
- `executionAllowed=true` when command ingress is available for the active mode

## WebSocket event policy

The stream still begins with a full `snapshot` and still sends `heartbeat` when idle.
Additional supervisor-owned events are additive:

- `lease_state`
- `command_lifecycle`
- `replay_updated`

Telemetry semantics from v1 are unchanged.

## Persistence and audit

The audit SQLite store now persists enough command state for review/debug:

- current lifecycle state
- summary label
- confirmation expiry
- plan fingerprint
- correlation id
- risk level
- execution result summary
- transition timeline and runtime events

## Residual risk

### Acceptable for v2 development

- sim command ingress, validation, confirmation, and audit trail
- hardware mode remains fail-closed until dual gate + preflight pass
- backend-owned execution adapter that validates and dispatches ROS motion without opening raw browser -> ROS control
- browser UX gating backed by backend enforcement

### Must fix before treating real hardware as fresh/executable

- verify hardware-mode freshness against live `robot_status` and the real preferred joint topic
- prove controller/mode/alarm/servo semantics on MotoROS2 under load
- validate the supervisor-owned execution adapter against the real hardware boundary before enabling hardware mode
- validate execution feedback, cancel, and fault handling end-to-end on hardware
- review lease/auth identity beyond local development assumptions
