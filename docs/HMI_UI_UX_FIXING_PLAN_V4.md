# GP4 HMI UI/UX Fixing Plan v4

This plan replaces the earlier UI-only plan with one that matches the current
codebase contracts and the safety logic already present in the HMI.

## Scope

- Target: `hmi/frontend` React 18 + TypeScript + custom CSS.
- Primary screen: `hmi/frontend/components/GP4HMI.tsx`.
- Secondary screen: `hmi/frontend/components/JogPendant.tsx`.
- Backend contracts: `hmi/backend/api/app.py`, `hmi/backend/api/contracts.py`,
  and `hmi/shared/contracts.ts`.
- Do not change backend behavior, ROS2 behavior, lease semantics, command
  validation, hardware gates, or execution rules as part of visual polish.

## Non-Negotiable Invariants

- All operator text, LLM output, backend stream payloads, and browser state are
  untrusted inputs. Localhost does not make data trusted.
- UI changes must not relax schema validation, lease checks, confirmation
  checks, runtime blocking, read-only mode, or hardware-gate behavior.
- Natural-language command text must never be treated as accepted, validated,
  executable, or executed until the backend lifecycle says so.
- Runtime state, connection state, E-stop, alarm, servo, lease, and blocking
  status must remain visible during reconnect and stale-data periods.
- Process-affecting commands must keep the existing multi-step safety boundary:
  submit, validate, confirm when required, execute, terminal state.
- Do not newly log or render `leaseToken`, raw backend payloads, socket URLs with
  identifiers, local file paths, or unnecessary `sessionId` / `operatorId`
  details in UI surfaces.

## Preserved Contracts

### WebSocket

- Endpoint: `/api/hmi/stream`.
- Required query params: `session_id`, `operator_id`.
- Server-emitted event types:
  - `snapshot`
  - `heartbeat`
  - `lease_state`
  - `command_lifecycle`
  - `sequence_lifecycle`
  - `replay_updated`
  - `jog_bridge_status`
- Frontend-only synthesized event:
  - `connection_state`
- `schemaVersion` is present on `snapshot` data and `heartbeat`; do not require
  it on every stream event.
- `telemetry.v1` is anchored in backend, frontend, and shared contract code.
  Keep that version stable unless performing an explicit contract migration.

### REST And Commands

- Preserve existing `/api/hmi` REST command, lease, replay, and jog routes.
- Toasts and button states must be based on accepted backend responses and
  stream lifecycle updates, not just local click handlers.
- `CommandLifecycleState` remains the authoritative source for command status:
  `RECEIVED`, `PARSING`, `VALIDATING`, `NEEDS_CONFIRMATION`, `CONFIRMED`,
  `EXECUTION_REQUESTED`, `EXECUTING`, `SUCCEEDED`, `FAILED`, `REJECTED`,
  `CANCELLED`, `EXPIRED`.

### Chat Message Contract

- `ChatMessage.origin` stays `system | operator | assistant`.
- Warning/error display is derived from `tag` and lifecycle state, not from a
  new message origin.
- Visual labels must not imply assistant text is robot telemetry or system
  truth. Rename display copy such as `GP4 Agent` / `robot` to an explicit
  assistant label.

## Architecture Rules

- Keep exactly one WebSocket connection owner per active view.
- Child components receive state and callbacks through props; they must not call
  `client.connect()`.
- Preserve existing extracted boundaries:
  - `useGP4Bridge.ts`
  - `bridgeClient.ts`
  - `RuntimeStateBanner.tsx`
  - `JogPendant.tsx`
- Refactor only still-inline panels inside `GP4HMI.tsx` unless a contract bug
  requires a narrower supporting change.
- Keep `App.tsx` responsible for top-level tab routing and view selection.
- Keep a single globally imported base/tokens stylesheet. Split feature-specific
  CSS only after command-tab and jog-tab smoke checks pass.

## Phase 0: Baseline And Guardrails

1. Capture current behavior before editing:
   - command tab loads and connects
   - jog tab loads and receives `jog_bridge_status`
   - command submit, confirm, cancel, and replay still work
   - reconnect displays fail-closed runtime status
2. Audit existing CSS tokens in `hmi/frontend/styles/gp4-hmi.css`.
   Extend the current variables instead of replacing them.
3. Add a short implementation checklist to the PR or commit notes:
   - no new WebSocket owner
   - no contract rename
   - no lease/hardware-gate bypass
   - no sensitive token or raw payload rendering

Verification:

- `npm run build` from `hmi/frontend`.
- Manual command-tab and jog-tab smoke check.

## Phase 1: Readability And Visual Hierarchy

Goal: improve scan speed without changing behavior.

Actions:

- Raise text sizes conservatively where the current UI is below operator-readable
  size. Keep dense operational panels compact but legible.
- Preserve neutral backgrounds. Reserve red/yellow alarm colors for abnormal
  conditions only.
- Add icons only where they improve recognition of existing states:
  connection, servo, E-stop, alarm, lease, confirm, cancel, replay, jog bridge.
- Every icon must have visible text or an accessible label. Do not replace
  safety-critical text with icon-only controls.
- Keep focus rings visible for keyboard users.

Avoid:

- decorative animations
- blinking text
- alarm colors for normal success decoration
- new marketing-style cards or hero sections

Verification:

- `npm run build`.
- Visual check at desktop size used by the HMI.
- Confirm all critical state text remains visible without relying on color only.

## Phase 2: Message And Status Presentation

Goal: make chat/status history clearer while preserving trust boundaries.

Actions:

- Style messages from `origin`:
  - `operator`: operator-entered request
  - `assistant`: LLM/assistant response, explicitly labeled as assistant
  - `system`: backend/system status
- Style warning/error states from `message.tag` or command lifecycle terminal
  states.
- Cap rendered message history or isolate it so long sessions do not keep
  increasing main-screen render cost.
- Keep runtime banners and command lifecycle status visually separate from
  assistant text.
- Do not show raw backend payloads in the chat, toast, or debug UI.

Verification:

- Existing command lifecycle messages still render.
- Assistant messages are not labeled as robot telemetry.
- Failed/rejected/cancelled command states are visually distinct and text-labeled.

## Phase 3: Loading, Stale Data, And Toasts

Goal: improve temporary feedback without hiding fail-closed state.

Actions:

- Use skeletons only before the first valid snapshot.
- After the first snapshot, preserve last-known values during reconnect and mark
  them stale/disconnected.
- Keep `RuntimeStateBanner` visible during reconnect, stale telemetry, E-stop,
  alarm, lease loss, and runtime blocking.
- Button loading states may reflect a pending local request, but copy must say
  `Submitting request` or equivalent until backend acceptance is known.
- Toasts must be edge-triggered and deduplicated:
  - `connected -> disconnected`
  - clear E-stop -> active E-stop
  - no alarm -> active alarm
  - command lifecycle terminal state changes
- Do not toast every repeated heartbeat, reconnect attempt, or latched fault.
- Prefer persistent banners for persistent unsafe states; use toasts for
  transient events.

Verification:

- Reconnect loop does not create repeated toast spam.
- Disconnect keeps last-known runtime data plus clear stale/disconnected status.
- Command success/error toast only appears after authoritative backend state.

## Phase 4 : Check Again Keep Codelines and Styles Clean,Fresh And Maintainable

## Phase 5: Component Refactor

Goal: reduce `GP4HMI.tsx` size without moving behavior across safety
boundaries.

Allowed extractions:

- `ChatPanel.tsx`
- `CommandComposer.tsx`
- `CommandLifecyclePanel.tsx`
- `JointMonitor.tsx`
- `TelemetryPanel.tsx`
- `ReplayPanel.tsx`
- `SystemLog.tsx`

Rules:

- Extract presentational components first.
- Pass already-derived props and callbacks from `GP4HMI.tsx`.
- Do not move WebSocket connection setup into extracted components.
- Do not change shared contract types to fit UI names.
- Do not refactor `useGP4Bridge.ts`, `bridgeClient.ts`, `JogPendant.tsx`, or
  backend services unless a specific regression demands it.

Verification:

- `npm run build`.
- Compare command submit, confirm, cancel, replay, lease, reconnect, and jog
  behavior before and after each extraction.

## Phase 6: Joint Bars And Telemetry Details

Goal: improve robot state readability without implying unsupported semantics.

Actions:

- Show joint position bars using the existing `minDeg`, `maxDeg`, and
  `positionDeg` values.
- Use warning/limit styling only when calculated from actual joint limits.
- Do not add movement direction arrows unless velocity or prior-position
  comparison is explicitly implemented and verified.
- Avoid pulse animations except for an operator-action-required state. Normal
  motion should remain visually quiet.
- Always show units in degrees.

Verification:

- Null joint positions render as unavailable, not as zero.
- Near-limit state is text-labeled and not color-only.
- No unsupported velocity or direction signal is displayed.

## Phase 7: Optional 3D Visualization

Defer this unless the operational screens are stable.

Rules:

- 3D is monitoring-only. It must not control motion.
- Use existing joint positions only.
- Clearly label approximate visualization if not using a verified robot model.
- Do not add `@react-three/fiber` or other heavy dependencies unless the extra
  bundle and maintenance cost is accepted.
- Failure or blank 3D view must not hide primary runtime state.

## Verification Matrix

Run after each phase that changes code:

- Frontend:
  - `cd hmi/frontend`
  - `npm run build`
- Backend contract and flow tests:
  - `pytest -q hmi/backend/tests/test_telemetry_bridge_v1.py`
  - `pytest -q hmi/backend/tests/test_jog_service.py`
  - `pytest -q hmi/backend/tests/test_supervisor_service.py`
  - `pytest -q hmi/backend/tests/test_command_e2e_sim.py`
- Browser smoke:
  - command tab connects with one active WebSocket owner
  - jog tab connects and displays jog bridge status
  - reconnect shows stale/disconnected state without blanking safety panels
  - command submit, confirm, cancel, and terminal lifecycle all display correctly
  - replay update still appears
  - shortcut behavior is scoped and does not fire while typing
  - toasts are deduplicated on repeated reconnect/fault events

## Acceptance Criteria

- Text is readable from the expected operator distance.
- Existing safety gates and backend contracts are unchanged.
- No duplicate WebSocket ownership is introduced.
- Assistant output is labeled as assistant, not robot truth.
- Warning/error display derives from tags or lifecycle states, not new contract
  roles.
- Loading states never hide runtime blocking, E-stop, alarm, lease, or connection
  state.
- Toasts are authoritative, deduplicated, and not used as the only indicator for
  persistent unsafe states.
- Keyboard shortcuts are scoped, cleaned up, and cannot bypass existing command
  gates.
- `GP4HMI.tsx` is reduced only through presentational extraction.
- Command tab and jog tab both pass smoke verification after every refactor.
