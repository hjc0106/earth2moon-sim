#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
LOCAL_SITE_PACKAGES="${PROJECT_ROOT}/.isaaclab_site/lib/python3.11/site-packages"
LOCAL_SOURCE_ROOT="${PROJECT_ROOT}/source/tiangong"
LOCAL_ASSET_ROOT="${TIANGONG_LOCAL_ASSET_ROOT:-${PROJECT_ROOT}/assets}"

export TIANGONG_ASSET_ROOT="${LOCAL_ASSET_ROOT}"
export TIANGONG_PROJECT_ASSETS_ROOT="${PROJECT_ROOT}/assets"
export TIANGONG_ISAAC_ASSET_ROOT="${TIANGONG_ISAAC_ASSET_ROOT:-}"

if [[ -n "${ISAAC_SIM_PYTHON:-}" ]]; then
    PYTHON_CMD="${ISAAC_SIM_PYTHON}"
elif [[ -x "${PROJECT_ROOT}/../../IsaacSim/python.sh" ]]; then
    PYTHON_CMD="${PROJECT_ROOT}/../../IsaacSim/python.sh"
elif [[ -x "${PROJECT_ROOT}/../IsaacSim/python.sh" ]]; then
    PYTHON_CMD="${PROJECT_ROOT}/../IsaacSim/python.sh"
else
    echo "[ERROR] Could not locate Isaac Sim python.sh." >&2
    echo "Set ISAAC_SIM_PYTHON or place IsaacSim next to this repository." >&2
    exit 1
fi

EXTRA_PYTHONPATH=()

if [[ -d "${LOCAL_SOURCE_ROOT}" ]]; then
    EXTRA_PYTHONPATH+=("${LOCAL_SOURCE_ROOT}")
fi

if [[ -d "${LOCAL_SITE_PACKAGES}" ]]; then
    EXTRA_PYTHONPATH+=("${LOCAL_SITE_PACKAGES}")
fi

if (( ${#EXTRA_PYTHONPATH[@]} > 0 )); then
    EXTRA_PATH="$(IFS=:; echo "${EXTRA_PYTHONPATH[*]}")"
    if [[ -n "${PYTHONPATH:-}" ]]; then
        export PYTHONPATH="${EXTRA_PATH}:${PYTHONPATH}"
    else
        export PYTHONPATH="${EXTRA_PATH}"
    fi
fi

exec "${PYTHON_CMD}" "$@"
