#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

export GP4_LLM_ENV_FILE="${GP4_LLM_ENV_FILE:-${WORKSPACE_DIR}/.env}"

if [[ -f "${GP4_LLM_ENV_FILE}" ]]; then
  set -a
  source "${GP4_LLM_ENV_FILE}"
  set +a
fi

exec python3 -m uvicorn hmi.backend.api.app:app \
  --host "${HMI_BACKEND_HOST:-127.0.0.1}" \
  --port "${HMI_BACKEND_PORT:-8000}"
