#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export PYTHONPATH="${WORKSPACE_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
source "${WORKSPACE_DIR}/install/setup.bash"

pytest "${WORKSPACE_DIR}/hmi/backend/tests/test_command_e2e_sim.py" -v -s "$@"
