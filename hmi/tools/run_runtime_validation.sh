#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PROBE_SCRIPT="${SCRIPT_DIR}/bridge_probe.py"
FIXTURE_SCRIPT="${SCRIPT_DIR}/ros_telemetry_fixture.py"

PORT="${HMI_VALIDATION_PORT:-18080}"
BASE_URL="http://127.0.0.1:${PORT}"
DOMAIN_BASE_DEFAULT=$((120 + ($$ % 40)))
DOMAIN_BASE="${HMI_VALIDATION_DOMAIN_BASE:-${DOMAIN_BASE_DEFAULT}}"
LOG_ROOT="${HMI_VALIDATION_LOG_ROOT:-$(mktemp -d "${TMPDIR:-/tmp}/gp4_hmi_runtime_validation.XXXXXX")}"

BRIDGE_PID=""
FIXTURE_PID=""
CURRENT_SCENARIO_DIR=""

declare -a PASSED_SCENARIOS=()
declare -a FAILED_SCENARIOS=()
declare -a TIMING_SENSITIVE_SCENARIOS=(
  "partial_stale_readiness"
  "joint_failover"
  "lost_conn_recover"
  "full_disconnect_after_gap"
  "burst_joint_states_ws"
)

cleanup_process() {
  local pid="${1:-}"
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! kill -0 "${pid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        return 0
      fi
      sleep 0.1
    done
    kill "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup_current_processes() {
  cleanup_process "${FIXTURE_PID}"
  cleanup_process "${BRIDGE_PID}"
  FIXTURE_PID=""
  BRIDGE_PID=""
}

on_exit() {
  cleanup_current_processes
}

trap on_exit EXIT INT TERM

note() {
  printf '[INFO] %s\n' "$*"
}

warn() {
  printf '[WARN] %s\n' "$*"
}

run_background_with_domain() {
  local domain_id="$1"
  local log_file="$2"
  shift 2
  (
    cd "${WORKSPACE_DIR}"
    export ROS_DOMAIN_ID="${domain_id}"
    set +u
    source install/setup.bash
    set -u
    exec "$@"
  ) >"${log_file}" 2>&1 &
  echo $!
}

wait_for_bridge_ready() {
  local scenario_dir="$1"
  local timeout_sec="${2:-15}"
  local deadline=$((SECONDS + timeout_sec))
  local out_file="${scenario_dir}/bridge_ready.json"
  local err_file="${scenario_dir}/bridge_ready.err"

  while (( SECONDS < deadline )); do
    if python3 "${PROBE_SCRIPT}" snapshot --base-url "${BASE_URL}" >"${out_file}" 2>"${err_file}"; then
      return 0
    fi
    if [[ -n "${BRIDGE_PID}" ]] && ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
      warn "Bridge process exited early. See ${scenario_dir}/bridge.log"
      return 1
    fi
    sleep 0.5
  done

  warn "Bridge did not become ready within ${timeout_sec}s. See ${scenario_dir}/bridge.log"
  return 1
}

wait_for_snapshot_expectations() {
  local scenario_dir="$1"
  local label="$2"
  local timeout_sec="$3"
  shift 3

  local deadline=$((SECONDS + timeout_sec))
  local out_file="${scenario_dir}/${label}.json"
  local err_file="${scenario_dir}/${label}.err"

  while (( SECONDS < deadline )); do
    if python3 "${PROBE_SCRIPT}" snapshot --base-url "${BASE_URL}" "$@" >"${out_file}" 2>"${err_file}"; then
      return 0
    fi
    sleep 0.5
  done

  return 1
}

run_stream_check() {
  local scenario_dir="$1"
  local label="$2"
  shift 2
  local out_file="${scenario_dir}/${label}.json"
  local err_file="${scenario_dir}/${label}.err"
  python3 "${PROBE_SCRIPT}" stream --base-url "${BASE_URL}" "$@" >"${out_file}" 2>"${err_file}"
}

scenario_is_timing_sensitive() {
  local scenario_name="$1"
  local item
  for item in "${TIMING_SENSITIVE_SCENARIOS[@]}"; do
    if [[ "${item}" == "${scenario_name}" ]]; then
      return 0
    fi
  done
  return 1
}

scenario_marker() {
  local scenario_name="$1"
  if scenario_is_timing_sensitive "${scenario_name}"; then
    printf ' [timing-sensitive]'
  fi
}

run_scenario() {
  local scenario_name="$1"
  local scenario_index="$2"
  local domain_id=$((DOMAIN_BASE + scenario_index))
  local scenario_dir="${LOG_ROOT}/$(printf '%02d_%s' "${scenario_index}" "${scenario_name}")"
  local bridge_log="${scenario_dir}/bridge.log"
  local fixture_log="${scenario_dir}/fixture.log"

  CURRENT_SCENARIO_DIR="${scenario_dir}"
  mkdir -p "${scenario_dir}"
  cleanup_current_processes

  note "Running scenario '${scenario_name}' on ROS_DOMAIN_ID=${domain_id}$(scenario_marker "${scenario_name}")"

  BRIDGE_PID="$(run_background_with_domain "${domain_id}" "${bridge_log}" python3 -m uvicorn hmi.backend.api.app:app --host 127.0.0.1 --port "${PORT}")"
  if ! wait_for_bridge_ready "${scenario_dir}" 20; then
    return 1
  fi

  case "${scenario_name}" in
    normal)
      FIXTURE_PID="$(run_background_with_domain "${domain_id}" "${fixture_log}" python3 "${FIXTURE_SCRIPT}" --scenario normal)"
      if ! wait_for_snapshot_expectations "${scenario_dir}" "snapshot" 10 \
        --expect transportState=connected \
        --expect telemetryState=fresh \
        --expect runtime.systemState=NORMAL; then
        return 1
      fi
      ;;
    safety_blocked)
      FIXTURE_PID="$(run_background_with_domain "${domain_id}" "${fixture_log}" python3 "${FIXTURE_SCRIPT}" --scenario safety_blocked)"
      if ! wait_for_snapshot_expectations "${scenario_dir}" "snapshot" 10 \
        --expect transportState=connected \
        --expect telemetryState=fresh \
        --expect runtime.systemState=SAFETY_BLOCKED; then
        return 1
      fi
      ;;
    estop_over_fault_and_blocked)
      FIXTURE_PID="$(run_background_with_domain "${domain_id}" "${fixture_log}" python3 "${FIXTURE_SCRIPT}" --scenario estop_over_fault_and_blocked)"
      if ! wait_for_snapshot_expectations "${scenario_dir}" "snapshot" 10 \
        --expect transportState=connected \
        --expect telemetryState=fresh \
        --expect runtime.systemState=ESTOP; then
        return 1
      fi
      ;;
    fault_over_blocked)
      FIXTURE_PID="$(run_background_with_domain "${domain_id}" "${fixture_log}" python3 "${FIXTURE_SCRIPT}" --scenario fault_over_blocked)"
      if ! wait_for_snapshot_expectations "${scenario_dir}" "snapshot" 10 \
        --expect transportState=connected \
        --expect telemetryState=fresh \
        --expect runtime.systemState=FAULT; then
        return 1
      fi
      ;;
    partial_stale_readiness)
      FIXTURE_PID="$(run_background_with_domain "${domain_id}" "${fixture_log}" python3 "${FIXTURE_SCRIPT}" --scenario partial_stale_readiness)"
      if ! wait_for_snapshot_expectations "${scenario_dir}" "snapshot" 12 \
        --expect transportState=connected \
        --expect telemetryState=stale \
        --expect runtime.systemState=NORMAL; then
        return 1
      fi
      ;;
    joint_failover)
      FIXTURE_PID="$(run_background_with_domain "${domain_id}" "${fixture_log}" python3 "${FIXTURE_SCRIPT}" --scenario joint_failover)"
      if ! wait_for_snapshot_expectations "${scenario_dir}" "snapshot" 12 \
        --expect transportState=connected \
        --expect telemetryState=fresh \
        --expect-summary activeJointSource=joint_states_fallback; then
        return 1
      fi
      ;;
    lost_conn_recover)
      FIXTURE_PID="$(run_background_with_domain "${domain_id}" "${fixture_log}" python3 "${FIXTURE_SCRIPT}" --scenario lost_conn_recover)"
      if ! wait_for_snapshot_expectations "${scenario_dir}" "mid_gap_snapshot" 12 \
        --expect transportState=connecting \
        --expect telemetryState=stale \
        --expect runtime.systemState=LOST_CONN; then
        return 1
      fi
      if ! wait_for_snapshot_expectations "${scenario_dir}" "recovered_snapshot" 8 \
        --expect transportState=connected \
        --expect telemetryState=fresh \
        --expect runtime.systemState=NORMAL; then
        return 1
      fi
      ;;
    full_disconnect_after_gap)
      FIXTURE_PID="$(run_background_with_domain "${domain_id}" "${fixture_log}" python3 "${FIXTURE_SCRIPT}" --scenario normal)"
      if ! wait_for_snapshot_expectations "${scenario_dir}" "baseline_snapshot" 10 \
        --expect transportState=connected \
        --expect telemetryState=fresh \
        --expect runtime.systemState=NORMAL; then
        return 1
      fi
      if ! wait_for_snapshot_expectations "${scenario_dir}" "full_disconnect_snapshot" 60 \
        --expect transportState=disconnected \
        --expect telemetryState=unavailable \
        --expect runtime.systemState=LOST_CONN; then
        return 1
      fi
      ;;
    burst_joint_states_ws)
      FIXTURE_PID="$(run_background_with_domain "${domain_id}" "${fixture_log}" python3 "${FIXTURE_SCRIPT}" --scenario burst_joint_states)"
      if ! wait_for_snapshot_expectations "${scenario_dir}" "baseline_snapshot" 10 \
        --expect transportState=connected \
        --expect telemetryState=fresh \
        --expect runtime.systemState=NORMAL; then
        return 1
      fi
      if ! run_stream_check "${scenario_dir}" "stream" \
        --duration-sec 6 \
        --min-snapshots 2 \
        --max-heartbeats 1 \
        --expect-last-snapshot transportState=connected \
        --expect-last-snapshot telemetryState=fresh \
        --expect-last-snapshot runtimeSystemState=NORMAL; then
        return 1
      fi
      ;;
    *)
      warn "Unknown scenario '${scenario_name}'"
      return 1
      ;;
  esac
}

record_result() {
  local scenario_name="$1"
  local status="$2"
  if [[ "${status}" == "PASS" ]]; then
    PASSED_SCENARIOS+=("${scenario_name}")
    printf '[PASS] %s%s\n' "${scenario_name}" "$(scenario_marker "${scenario_name}")"
  else
    FAILED_SCENARIOS+=("${scenario_name}")
    printf '[FAIL] %s%s\n' "${scenario_name}" "$(scenario_marker "${scenario_name}")"
    if [[ -n "${CURRENT_SCENARIO_DIR}" ]]; then
      printf '       logs: %s\n' "${CURRENT_SCENARIO_DIR}"
      if [[ -f "${CURRENT_SCENARIO_DIR}/bridge.log" ]]; then
        printf '       bridge tail:\n'
        tail -n 5 "${CURRENT_SCENARIO_DIR}/bridge.log" | sed 's/^/         /'
      fi
      if [[ -f "${CURRENT_SCENARIO_DIR}/fixture.log" ]]; then
        printf '       fixture tail:\n'
        tail -n 5 "${CURRENT_SCENARIO_DIR}/fixture.log" | sed 's/^/         /'
      fi
    fi
  fi
}

main() {
  local start_epoch
  start_epoch="$(date +%s)"
  local -a scenarios=(
    "normal"
    "safety_blocked"
    "estop_over_fault_and_blocked"
    "fault_over_blocked"
    "partial_stale_readiness"
    "joint_failover"
    "lost_conn_recover"
    "full_disconnect_after_gap"
    "burst_joint_states_ws"
  )

  note "Runtime validation log root: ${LOG_ROOT}"
  note "Base URL: ${BASE_URL}"
  note "ROS_DOMAIN_ID base: ${DOMAIN_BASE}"
  note "Expected runtime: about 2 minutes"

  local index=0
  local scenario_name
  for scenario_name in "${scenarios[@]}"; do
    if run_scenario "${scenario_name}" "${index}"; then
      record_result "${scenario_name}" "PASS"
    else
      record_result "${scenario_name}" "FAIL"
    fi
    cleanup_current_processes
    CURRENT_SCENARIO_DIR=""
    index=$((index + 1))
  done

  local end_epoch elapsed
  end_epoch="$(date +%s)"
  elapsed=$((end_epoch - start_epoch))

  printf '\n=== Runtime validation summary ===\n'
  printf 'Passed: %d\n' "${#PASSED_SCENARIOS[@]}"
  printf 'Failed: %d\n' "${#FAILED_SCENARIOS[@]}"
  printf 'Duration: %ds\n' "${elapsed}"
  printf 'Logs: %s\n' "${LOG_ROOT}"

  if ((${#FAILED_SCENARIOS[@]} > 0)); then
    printf 'Failed scenarios:\n'
    printf '  - %s\n' "${FAILED_SCENARIOS[@]}"
  fi

  printf 'Timing-sensitive scenarios:\n'
  printf '  - %s\n' "${TIMING_SENSITIVE_SCENARIOS[@]}"
  printf 'These scenarios depend on telemetry freshness windows and are the first to show scheduling or machine-load issues.\n'

  if ((${#FAILED_SCENARIOS[@]} > 0)); then
    exit 1
  fi
}

main "$@"
