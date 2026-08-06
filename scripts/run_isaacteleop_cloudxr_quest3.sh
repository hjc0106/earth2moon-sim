#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${ISAAC_TELEOP_PYTHON:-${PROJECT_ROOT}/.venv/bin/python}"
CLOUDXR_INSTALL_DIR="${CLOUDXR_INSTALL_DIR:-${HOME}/.cloudxr}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    echo "[ERROR] Project Python not found: ${PYTHON_BIN}" >&2
    echo "Run ${SCRIPT_DIR}/install_minimal.sh first." >&2
    exit 1
fi

if ! "${PYTHON_BIN}" -c "import isaacteleop.cloudxr" >/dev/null 2>&1; then
    echo "[ERROR] Isaac Teleop CloudXR is not installed in ${PYTHON_BIN}." >&2
    echo "Run ${SCRIPT_DIR}/install_minimal.sh first." >&2
    exit 1
fi

exec "${PYTHON_BIN}" -m isaacteleop.cloudxr \
    --cloudxr-install-dir "${CLOUDXR_INSTALL_DIR}" \
    --accept-eula
