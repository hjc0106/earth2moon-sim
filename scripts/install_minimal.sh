#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
UV_BIN="${PROJECT_ROOT}/.tools/bin/uv"
VENV_PYTHON="${PROJECT_ROOT}/.venv/bin/python"

if grep -RIlq --include='*.usd' --include='*.usda' --include='*.dae' \
    '^version https://git-lfs.github.com/spec/v1$' "${PROJECT_ROOT}/assets"; then
    echo "[ERROR] Git LFS assets have not been checked out." >&2
    echo "Install git-lfs and run: git lfs pull origin main" >&2
    exit 1
fi

if [[ ! -x "${UV_BIN}" ]]; then
    echo "[ERROR] uv is missing: ${UV_BIN}" >&2
    echo "Install uv into .tools/bin first: https://docs.astral.sh/uv/" >&2
    exit 1
fi

if [[ ! -x "${VENV_PYTHON}" ]]; then
    "${UV_BIN}" venv --python 3.11 --seed "${PROJECT_ROOT}/.venv"
fi

"${UV_BIN}" pip install --python "${VENV_PYTHON}" \
    "isaacsim[all,extscache]==5.1.0" \
    --extra-index-url https://pypi.nvidia.com

"${UV_BIN}" pip install --python "${VENV_PYTHON}" \
    "isaacteleop[cloudxr]~=1.0.0" \
    --extra-index-url https://pypi.nvidia.com

OMNI_KIT_ACCEPT_EULA=YES "${VENV_PYTHON}" -c \
    "import importlib.metadata as m; import isaacsim; print('Isaac Sim', m.version('isaacsim'))"

echo "[OK] earth2moon runtime is ready."
