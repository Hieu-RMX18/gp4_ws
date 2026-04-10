# Hardware telemetry validation gate for HMI Phase A

## Current status

- Phase A is **not proven** in this session.
- The system remains **fail-closed** for hardware command execution.
- HMI v2 command ingress stays sim-only until this report is completed with live MotoROS2 evidence.

## Scope

This phase validates read-only hardware telemetry only. It does **not** enable execution.

Topics under review:

- `/gateway_status`
- `/hw_adapter/ready`
- `/supervisor/alerts`
- `/yaskawa/joint_states`
- `/joint_states`
- `/yaskawa/robot_status`

## Non-negotiable gate

Do **not** enable any hardware execution gate until all of these are proven on the real stack:

1. freshness thresholds are justified by measured publish timing and jitter
2. disconnect and reconnect behavior is measured
3. `robot_status` safety semantics are correlated with real controller state
4. joint source precedence is validated on hardware
5. audit trail and operator-visible blocked reasons remain explicit

## Read-only capture tool

Use:

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
python3 hmi/tools/hardware_telemetry_validation.py \
  --duration-sec 120 \
  --output /tmp/gp4_hardware_telemetry_report.json
```

What it records:

- message counts
- min/mean/max interval
- max observed gap
- freshness transitions (`unavailable -> fresh -> stale`)
- derived active joint source (`joint_states_primary` vs `joint_states_fallback`)
- observed readiness, supervisor alert, and `robot_status` field values

This tool is read-only. It subscribes only to telemetry topics.

## Required validation scenarios

Run these with the robot in a safe validation window and with plant approval.

### 1. Steady-state capture

Objective:

- capture at least 120 s of normal hardware telemetry
- confirm preferred joint source is `/yaskawa/joint_states`
- measure max gap and jitter for all sources

Expected evidence:

- all required sources produce messages
- no unexplained freshness oscillation
- fallback joint source is idle or clearly secondary

### 2. Preferred/fallback joint source behavior

Objective:

- prove hardware normally prefers `/yaskawa/joint_states`
- observe fallback behavior only when the preferred source becomes stale or absent

Expected evidence:

- active source starts as `joint_states_primary`
- fallback only becomes active when primary freshness is lost
- active source returns to primary after recovery

### 3. Disconnect / reconnect timing

Objective:

- measure stale detection timing and recovery timing for each freshness-critical source

Suggested method:

- pause or isolate the upstream publisher / agent in a controlled maintenance window
- resume it after stale is observed

Expected evidence:

- stale is detected within threshold plus scheduler tolerance
- recovery occurs after telemetry resumes
- no silent transport success while freshness-critical sources are stale

### 4. Safety semantics correlation

Objective:

- prove `robot_status` and readiness fields match real controller meaning

Minimum checks:

- E-stop active -> `robot_status.e_stopped == TRUE`
- controller/alarm fault -> `robot_status.in_error == TRUE` or error codes present
- servo or motion-disabled condition -> readiness becomes not ready
- AUTO vs MANUAL controller mode maps correctly

Expected evidence:

- every exercised controller condition is reflected in telemetry
- any unknown tri-state stays treated as not-ready / blocked

## Pass / fail worksheet

| Check | Required evidence | Status |
| --- | --- | --- |
| gateway_status timing measured | capture file + reviewed max gap/jitter | BLOCKED |
| readiness timing measured | capture file + reviewed max gap/jitter | BLOCKED |
| supervisor_alerts timing measured | capture file + reviewed max gap/jitter | BLOCKED |
| robot_status timing measured | capture file + reviewed max gap/jitter | BLOCKED |
| primary joint source proven | capture shows primary preferred on hardware | BLOCKED |
| fallback policy proven | capture shows fallback only on primary freshness loss | BLOCKED |
| disconnect timing proven | stale/recover transitions captured | BLOCKED |
| E-stop semantics proven | observed hardware state matches telemetry | BLOCKED |
| fault semantics proven | observed hardware alarm matches telemetry | BLOCKED |
| controller mode semantics proven | AUTO/MANUAL meaning confirmed | BLOCKED |

## Threshold review guidance

Do not change thresholds from the current HMI baseline unless the capture justifies it.

Current baseline under review:

- `gateway_status`: `30.0 s`
- `readiness`: `3.0 s`
- `supervisor_alerts`: `5.0 s`
- `robot_status`: `3.0 s`
- `joint_states_primary`: `3.0 s`
- `joint_states_fallback`: `3.0 s`

Only change a threshold when:

- live hardware capture shows repeated healthy gaps near or above the current threshold, or
- reconnect / drop behavior proves the threshold is too lax to remain safety-meaningful

Any change must be documented with:

- raw capture file
- observed max gap
- reason for new threshold
- risk tradeoff

## Required artifacts before Phase B+

Before moving past Phase A, store:

1. capture JSON from `hardware_telemetry_validation.py`
2. operator notes for each scenario
3. explicit pass/fail decision for each worksheet row
4. recommended threshold updates, if any

If any row remains unproven, hardware execution must stay disabled.
