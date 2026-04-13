#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export PYTHONPATH="${WORKSPACE_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
export COLCON_TRACE="${COLCON_TRACE-}"
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

pytest "${WORKSPACE_DIR}/hmi/backend/tests/test_command_e2e_sim.py" -v -s "$@"
