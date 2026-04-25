#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DURATION_SEC="120"
LOG_DIR="${GP4_LOG_DIR:-}"

usage() {
  cat <<'USAGE'
Usage:
  hmi/tools/run_readonly_hardware_validation.sh [--duration-sec SEC] [--log-dir DIR]

Read-only hardware validation for the GP4/YRC1000micro stack.

Preconditions:
  - micro-ROS Agent is already running on UDP 8888.
  - gp4_bringup system.launch.py or hw.launch.py is already running.
  - This script does not send motion, alarm reset, IO, jog, or action goals.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:?missing value for --duration-sec}"
      shift 2
      ;;
    --log-dir)
      LOG_DIR="${2:?missing value for --log-dir}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${LOG_DIR}" ]]; then
  LOG_DIR="/tmp/gp4_hw_validation_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${LOG_DIR}"

cd "${WORKSPACE_DIR}"

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # ROS setup scripts may reference optional environment variables.
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

note() {
  printf '[INFO] %s\n' "$*"
}

capture() {
  local name="$1"
  shift
  note "Running: $*"
  "$@" 2>&1 | tee "${LOG_DIR}/${name}.log"
}

capture_allow_fail() {
  local name="$1"
  shift
  note "Running: $*"
  set +e
  "$@" 2>&1 | tee "${LOG_DIR}/${name}.log"
  local status=${PIPESTATUS[0]}
  set -e
  printf '%s\n' "${status}" > "${LOG_DIR}/${name}.exit_code"
  return 0
}

note "Read-only GP4 hardware validation"
note "Workspace: ${WORKSPACE_DIR}"
note "Log dir: ${LOG_DIR}"
note "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
note "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
note "Safety boundary: no motion/action/service goals are sent by this script."

capture git_status git status --short --branch
capture colcon_list colcon list

capture hw_nodes bash -lc 'ros2 node list | sort'
capture hw_topics bash -lc 'ros2 topic list | sort'
capture hw_services bash -lc 'ros2 service list | sort'
capture hw_actions bash -lc 'ros2 action list | sort'

capture_allow_fail move_group_allow_trajectory_execution \
  ros2 param get /move_group allow_trajectory_execution
capture_allow_fail motion_core_max_velocity_scale \
  ros2 param get /motion_core_node max_velocity_scale
capture_allow_fail motion_core_max_acceleration_scale \
  ros2 param get /motion_core_node max_acceleration_scale
capture_allow_fail hw_adapter_sim_mode \
  ros2 param get /hw_adapter_node sim_mode
capture_allow_fail hw_adapter_follow_joint_trajectory_action \
  ros2 param get /hw_adapter_node follow_joint_trajectory_action

capture_allow_fail execute_motion_action_info ros2 action info /execute_motion
capture_allow_fail dispatch_trajectory_action_info ros2 action info /hw_adapter/dispatch_trajectory
capture_allow_fail yaskawa_fjt_action_info ros2 action info /yaskawa/follow_joint_trajectory

capture_allow_fail yaskawa_joint_states_hz \
  timeout 20s ros2 topic hz /yaskawa/joint_states --window 50
capture_allow_fail yaskawa_robot_status_once \
  timeout 10s ros2 topic echo --once /yaskawa/robot_status
capture_allow_fail hw_adapter_ready_once \
  timeout 10s ros2 topic echo --once /hw_adapter/ready

REPORT_PATH="${LOG_DIR}/gp4_hardware_telemetry_report.json"
capture hardware_telemetry_capture \
  python3 hmi/tools/hardware_telemetry_validation.py \
    --duration-sec "${DURATION_SEC}" \
    --output "${REPORT_PATH}"

capture hardware_telemetry_sha256 sha256sum "${REPORT_PATH}"

note "Read-only capture complete. Review logs in: ${LOG_DIR}"
note "Human review is still required before any hardware execution gate is enabled."
