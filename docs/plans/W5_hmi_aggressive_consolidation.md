# W5 — HMI Aggressive Consolidation: Backend Calls ROS Services Instead of Reimplementing

**Wave class:** Architecture refactor (cross-cutting)
**Risk:** Medium-High (HMI behaviour shifts from in-process to RPC; frontend tests need updates)
**Estimated effort:** 3–5 working days
**Depends on:** W3 (ReAct on `/llm_intent` is stable), W4 (perception ROS surface is stable)
**Unblocks:** W6 (cleanup wave hard-deletes the deprecated HMI logic)

---

## Goal

Per decision D3 (aggressive). HMI's FastAPI backend currently reimplements logic that already lives in ROS-side packages (`llm_gateway`, `safety`, `supervisor`). Examples verified in discovery:

- `hmi/backend/services/intent_resolution.py:411` — local `_hydrate_draw_workplane` (canonical at `llm_gateway/command_pipeline.py:63`)
- `hmi/backend/services/intent_constants.py` — duplicates primitive list and planner mapping
- `hmi/backend/services/intent_normalization.py` — duplicates normalization logic
- `hmi/backend/services/supervisor_validation.py` — duplicates safety pre-checks
- `hmi/backend/services/supervisor_submission.py` — duplicates submission flow
- `hmi/backend/services/supervisor_execution.py` — duplicates execution-gate check

This is the bloat root cause: every change to ROS-side logic risks drift from HMI's local copy. W2 already aligned `_hydrate_draw_workplane`'s behaviour and added DEPRECATED tags. W5 replaces the HMI's local copies with ROS service / topic calls.

The frontend continues to call HTTP/WebSocket endpoints on the FastAPI backend; only the backend's internals change. Frontend test fixtures need updates because the backend's response timing changes (RPC latency replaces in-process call), but the response shape stays.

---

## Why this comes after W3 and W4

W5 routes HMI through ROS services on `llm_gateway`, `safety`, `supervisor`. If those services' contracts are still being defined (W3 introduces ReAct on `/llm_intent`; W4 adds perception services), HMI rerouting catches the moving target. Stabilizing W3 and W4 first means W5 ports against a fixed contract.

---

## Discovery (paste raw output)

```bash
# A. List every HMI backend service file we will touch
ls hmi/backend/services/
ls hmi/backend/ros/

# B. Map each duplicated module to its src/ counterpart
rg -l "intent_constants|intent_normalization|intent_resolution" hmi/backend/
rg -l "supervisor_validation|supervisor_submission|supervisor_execution" hmi/backend/

# C. Existing ROS surface that HMI calls
rg -n "/validate_command|/execute_motion|/llm_intent|/llm_raw_command" hmi/backend/

# D. Existing ROS adapter pattern in HMI
sed -n '95,140p' hmi/backend/ros/adapter.py
sed -n '155,180p' hmi/backend/ros/command_dispatch.py

# E. Frontend fixtures that depend on backend response shape
ls hmi/frontend/src/__fixtures__/ 2>/dev/null
ls hmi/frontend/tests/ 2>/dev/null

# F. Existing tests
ls hmi/backend/tests/

# G. WebSocket / SSE pathways (for streaming feedback)
rg -l "WebSocket|websocket|sse|EventSource" hmi/backend/

# H. What's already exposed as a ROS service that we can reuse vs what needs new services
rg -n "def.*service\|create_service\|Service\(" src/llm_gateway/ src/safety/ src/supervisor/
```

If discovery H reveals that some HMI-needed ROS services do not yet exist, W5 must either propose them (separate sub-decision) or scope down. The agent reports the gap before writing code.

---

## Tasks
**Precondition (per F5):** `docs/hmi/HMI_ROS_INTERFACES.md` from W0.T9 must be current and reflect every interface added in W2.T0, W3.T2 (compute_arc_points doesn't add a ROS interface — skip), and W4.T0. W5 cannot start until the HMI inventory is up to date for waves W2-W4.

Add to W5's discovery commands:

```bash
diff docs/hmi/HMI_ROS_INTERFACES.md <(git show main:docs/hmi/HMI_ROS_INTERFACES.md 2>/dev/null) | head -50
```

If the diff is empty (i.e. HMI inventory was never updated despite W2-W4 adding interfaces), STOP and run W0.T9 against the current branch.
### W5.T1 — Identify the HMI → ROS call surface

The agent produces a mapping table in `MIGRATION-W5.md`:

| HMI module (current) | Operation | New ROS surface |
|---|---|---|
| `intent_resolution.py:411` `_hydrate_draw_workplane` | Hydrate draw workplane | New service `/llm_gateway/hydrate_workplane` (call `command_pipeline.hydrate_draw_workplane` server-side) |
| `intent_constants.py` `PLANNER_DEFAULTS`, primitive lists | Static config | Single shared config file under `interfaces/` package; HMI imports via Python or fetches via `/llm_gateway/get_primitive_constants` |
| `intent_normalization.py` per-primitive normalize | Normalize primitive command | Existing `/validate_command` already does this server-side; HMI sends raw command to `/validate_command` and receives normalized form back |
| `supervisor_validation.py:57` `_validate_command` | Pre-flight validation | Use `/validate_command` directly. Remove HMI's local copy. |
| `supervisor_submission.py:223,500` `_validate_command` callers | Multi-step submission | Use `/validate_command` + `/execute_motion` action |
| `supervisor_execution.py:25` confirm gate | Confirm gate | New service `/supervisor/confirm_execution` (move logic into supervisor pkg) |
| Workplane fallback policy | Hard-coded fallback | Read from SSOT `safety_rules.yaml drawing.fallback_workplane.*` (W2 added this) |

If a ROS service is needed but does not exist, the W5 PR adds it on the corresponding ROS-side package, with clear scope (one service per module). The added services are the minimum to enable HMI rerouting; no speculative endpoints.

### W5.T2 — Create new ROS services where the table indicates

For each "new service" entry in W5.T1's table:

1. Define a service interface in `src/interfaces/srv/`. Naming: `HydrateWorkplane.srv`, `GetPrimitiveConstants.srv`, `ConfirmExecution.srv`, etc.
2. Implement the server in the ROS-side package that owns the logic:
   - `/llm_gateway/hydrate_workplane` → `llm_gateway/command_pipeline.py` already has the canonical function; add a service wrapper in `llm_gateway_node.py`.
   - `/llm_gateway/get_primitive_constants` → returns the constants from a single YAML in `interfaces/config/primitive_constants.yaml`.
   - `/supervisor/confirm_execution` → moves the confirm-gate logic from `hmi/backend/services/supervisor_execution.py:25` into the supervisor package.
3. Add tests for each new service on the ROS side.

The agent must NOT introduce Python module-level state to handle these services; they are stateless RPCs.

### W5.T3 — Rewrite HMI backend services as thin RPC clients

Each module in `hmi/backend/services/intent_*` and `supervisor_*` is rewritten to be a thin layer that:

1. Takes the same input as before (preserving the FastAPI route contract).
2. Calls the corresponding ROS service via `hmi/backend/ros/adapter.py`.
3. Returns the same shape as before.

The local logic functions are deleted from the same PR. Example:

`hmi/backend/services/intent_resolution.py` BEFORE:

```python
def _hydrate_draw_workplane(self, payload):
    # 80 lines of local logic, duplicated from llm_gateway
    ...
```

AFTER:

```python
def _hydrate_draw_workplane(self, payload):
    # Calls the canonical service. No local logic.
    response = self._ros_adapter.call_service(
        "/llm_gateway/hydrate_workplane",
        HydrateWorkplane.Request(payload=payload),
        timeout_s=5.0)
    if not response.success:
        raise WorkplaneHydrationError(response.error)
    return response.hydrated_payload
```

This is the aggressive part: the local copy goes away in the same PR that introduces the RPC. No deprecation period. The risk is mitigated by:

- The new ROS service is the canonical implementation called from llm_gateway elsewhere (not new code).
- A property-based test ensures the RPC produces identical outputs to the previous local function for a battery of inputs.

### W5.T4 — Update `hmi/backend/ros/adapter.py` for new services

`adapter.py:99-100` currently lists `/validate_command` and `/execute_motion`. Add the new services:

```python
'/validate_command',
'/execute_motion',
'/llm_gateway/hydrate_workplane',
'/llm_gateway/get_primitive_constants',
'/supervisor/confirm_execution',
```

The adapter's readiness state machine must include each new service; HMI is not "ready" until all are reachable.

`hmi/backend/ros/telemetry_snapshot.py:80-86` lists readiness flags. Extend.

### W5.T5 — Frontend test fixture updates

Frontend tests under `hmi/frontend/tests/` may have fixtures that simulate the previous in-process timing. RPC latency is on the order of 5–50 ms; in-process was sub-millisecond. Update fixtures:

- Increase test timeouts where applicable.
- Add mock RPC adapters where end-to-end tests stubbed in-process functions.
- Verify that loading-state UI elements remain hidden ≤200 ms (perceptual budget).

If the frontend has user-facing latency expectations, update them. Document any UX delta.

### W5.T6 — DEPRECATED → DELETED

W2 added DEPRECATED tags to:

- `hmi/backend/services/intent_resolution.py:411 _hydrate_draw_workplane`
- (any other tagged entries from W2)

W5 deletes them. Per R5 of `AGENTS.md`, deletion is allowed once `rg <symbol>` returns 0 hits. The agent runs `rg` for each deprecated symbol and pastes proof.

For symbols not yet tagged (the rest of intent_*/supervisor_*), the deletion is in the same PR as the RPC replacement. This is the aggressive choice; document each deletion in `MIGRATION-W5.md` with the RPC replacement reference.

### W5.T7 — Tests

ROS-side tests (added with the new services):

- `test_hydrate_workplane_service.py`: payload in → hydrated payload out, with all the cases the local HMI version handled (mode=base, mode=tool, fallback, etc.).
- `test_confirm_execution_service.py`: confirm gate behaviour.

HMI-side tests:

- `hmi/backend/tests/test_intent_resolution_via_rpc.py`: integration with a stubbed ROS adapter; same input/output contract as before.
- `hmi/backend/tests/test_supervisor_validation_via_rpc.py`: same.
- Property-based test (`hypothesis` for Python, or generated examples): for a battery of payloads, the new RPC pathway produces identical outputs to a captured baseline of the old local function. Capture this baseline BEFORE deletion.
- `hmi/backend/tests/test_adapter_readiness.py`: HMI is not ready until all 5 ROS services are reachable.

Frontend tests: existing test suite passes; updated fixtures merged.

### W5.T8 — Documentation

Update `hmi/ARCHITECTURE.vi.md` to reflect the new flow: HMI backend is a thin layer over ROS services. Update `hmi/HMI_V2_COMMAND_INGRESS.md` if it described the in-process pathway.

---

## Verification

| # | Check | Pass criterion |
|---|---|---|
| 1 | `rg -l "_hydrate_draw_workplane" hmi/backend/services/` | 0 hits (deleted; only test files may reference) |
| 2 | `rg -n "intent_normalization\|intent_resolution" hmi/backend/services/` | Files exist but contain only RPC wrappers, no local logic |
| 3 | New ROS services discoverable | `ros2 service list` shows `/llm_gateway/hydrate_workplane`, `/llm_gateway/get_primitive_constants`, `/supervisor/confirm_execution` |
| 4 | Property-based RPC equivalence test | Passes for ≥1000 random payloads; output identical to captured baseline |
| 5 | HMI backend integration tests | Green |
| 6 | HMI frontend integration tests | Green |
| 7 | E2E test: NL prompt via HMI → reaches /llm_intent (ReAct) → safety → execute_motion | Same final result as before W5; latency increase ≤ 50 ms per RPC step |
| 8 | `colcon test --packages-select llm_gateway supervisor interfaces` | Green; new service tests pass |
| 9 | `pytest hmi/backend/` | Green |
| 10 | `npm test --prefix hmi/frontend/` | Green; updated fixtures merged |
| 11 | `jscpd hmi/backend/ src/` | No new duplicate blocks ≥30 LOC; ideally several existing blocks disappear |
| 12 | Adapter readiness | HMI shows "not ready" if any ROS service is offline; exact service name in the readiness panel |

---

## DON'T

- Do not start W5 if W3 or W4 services are still being defined. Their contracts must be stable.
- Do not change the FastAPI route contract or the WebSocket message shape. The frontend stays untouched in user-visible terms.
- Do not skip the property-based equivalence test. Aggressive consolidation needs proof of behavioural equivalence.
- Do not introduce a "transition mode" that runs both local and RPC paths and compares. That doubles complexity for no compounding benefit.
- Do not restore any deleted local function as a "performance fallback". RPC latency in this domain is acceptable; if a benchmark proves otherwise, that is a separate optimization PR.
- Do not merge the new ROS services PR and the HMI rewrite PR in the wrong order. Services first, then HMI rewrite. Same wave, two PRs is fine; one giant PR is also fine if reviewers can handle it.
- Do not delete files outside the duplication scope. `hmi/backend/services/` has 14+ files; not all duplicate. Each deletion is justified in `MIGRATION-W5.md`.
- Do not change ROS topic or service authentication / authorization without explicit review.

---

## Output artefacts

- `src/interfaces/srv/HydrateWorkplane.srv` — new
- `src/interfaces/srv/GetPrimitiveConstants.srv` — new
- `src/interfaces/srv/ConfirmExecution.srv` — new
- `src/interfaces/CMakeLists.txt`, `package.xml` — diffs adding services
- `src/llm_gateway/llm_gateway/llm_gateway_node.py` — diff: hydrate_workplane service handler
- `src/llm_gateway/llm_gateway/...` — diff: get_primitive_constants service handler
- `src/llm_gateway/llm_gateway/services/` — new directory if separating service handlers from the god-node
- `src/supervisor/src/...` — diff: confirm_execution service handler
- `src/llm_gateway/tests/test_hydrate_workplane_service.py` — new
- `src/supervisor/test/test_confirm_execution_service.cpp` — new (or `.py` if Python service)
- `hmi/backend/services/intent_resolution.py` — diff: RPC wrapper, local function deleted
- `hmi/backend/services/intent_normalization.py` — diff
- `hmi/backend/services/intent_constants.py` — diff (or deletion if entirely served by RPC)
- `hmi/backend/services/supervisor_validation.py` — diff
- `hmi/backend/services/supervisor_submission.py` — diff
- `hmi/backend/services/supervisor_execution.py` — diff
- `hmi/backend/ros/adapter.py` — diff: new services in adapter list
- `hmi/backend/ros/telemetry_snapshot.py` — diff: new readiness flags
- `hmi/backend/tests/test_*` — new RPC and equivalence tests
- `hmi/frontend/tests/__fixtures__/` — diff: updated fixtures
- `hmi/ARCHITECTURE.vi.md` — diff
- `MIGRATION-W5.md`

---

## Rollback procedure

```bash
# Revert the HMI rewrite PR but keep the new ROS services
# (they are backward-compatible additions).
git revert -m 1 <hmi-rewrite PR commit>

# OR revert both, in the right order:
git revert -m 1 <hmi-rewrite PR commit>
git revert -m 1 <ros-services PR commit>

# If the new ROS services are stable but the HMI rewrite has a regression,
# disable the RPC path via a feature flag in HMI's config:
# hmi/backend/config.yaml:
#   use_ros_services_for_intent: false
# This requires keeping a thin shim in the HMI services that branches on the
# flag. Decide BEFORE merging whether to ship that shim or commit fully.
```

---

## Risk notes

- **Latency regression**: each new RPC adds 5–50 ms. If HMI workflow has 5 sequential RPCs, a request slows by 25–250 ms. Acceptable for command-issuing UI; not acceptable for streaming telemetry. Verify per-route.
- **Service unavailability**: if a new ROS service is down, HMI was previously self-sufficient. Now it isn't. The readiness panel must surface this; the FastAPI route returns 503 with "ros service `<name>` unavailable".
- **Behavioural drift caught late**: the property-based test catches divergence on tested inputs. Edge cases not in the test fall through. Mitigation: capture the baseline by running the OLD local function over a corpus of real production payloads before deletion. The tests then assert the new RPC matches.
- **Frontend tests are flaky under RPC latency**: increase Jest/Vitest timeout but do not make tests "wait until flaky". If a test times out, the underlying RPC is too slow — that is a real bug.
- **Race conditions**: HMI frontend may issue concurrent requests. Each RPC call is independent; the ROS service must be thread-safe (per ROS executor model, single-threaded executors are; multi-threaded need explicit locking). Verify per service.
- **Versioning**: the new services define a contract. Once HMI ships, that contract is binding. Treat each service interface as a public API. Schema changes require a version bump.

---

## Stop signal

End of W5. Do not proceed to W6 until:

- W5 PR(s) merged.
- HMI end-to-end tests pass.
- Property-based equivalence test pass count visible in CI.
- `jscpd` reports a measurable decrease in cross-package duplication.
- Operator confirms HMI behavioural parity (manual smoke test of common workflows).

State explicitly: `End of W5. Awaiting review before W6.`

---

**Reliability tag:** `[NEEDS-VALIDATION]` — depends on the discovery output for which HMI services map cleanly to existing ROS services vs need new ones. The mapping table in W5.T1 is a starting point informed by current discovery; the agent may need to refine after reading the actual implementations of `intent_normalization.py` and `supervisor_validation.py`. The aggressive choice (per D3) is locked in; the only adaptation is which services need to be newly created vs already exist.
