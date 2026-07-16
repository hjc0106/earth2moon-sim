#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CLOUDXR_LAUNCHER="${CLOUDXR_LAUNCHER:-/home/zjz/workspace/tiangong/mujoco_teleop/scripts/run_isaacteleop_cloudxr_quest3.sh}"
SCENE_LAUNCHER="${SCENE_LAUNCHER:-${SCRIPT_DIR}/run_with_isaaclab.sh}"
SCENE_ENTRY="${SCENE_ENTRY:-${SCRIPT_DIR}/keyboard_teleop_ranger_arm.py}"

HOST_IP="${HOST_IP:-172.18.4.85}"
PROXY_PORT="${PROXY_PORT:-48322}"
CLOUDXR_ENV_FILE="${CLOUDXR_ENV_FILE:-/home/zjz/.cloudxr/run/cloudxr.env}"
STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-20}"
HEADSET_CLIENT_BASE="${HEADSET_CLIENT_BASE:-https://nvidia.github.io/IsaacTeleop/client/}"
HEADSET_CODEC="${HEADSET_CODEC:-h264}"
HEADSET_IMMERSIVE_MODE="${HEADSET_IMMERSIVE_MODE:-vr}"
HEADSET_FRAME_RATE="${HEADSET_FRAME_RATE:-72}"
HEADSET_MAX_BITRATE_MBPS="${HEADSET_MAX_BITRATE_MBPS:-25}"
HEADSET_CLIENT_URL="${HEADSET_CLIENT_BASE}?serverIP=${HOST_IP}&port=${PROXY_PORT}&codec=${HEADSET_CODEC}&immersiveMode=${HEADSET_IMMERSIVE_MODE}&deviceFrameRate=${HEADSET_FRAME_RATE}&maxStreamingBitrateMbps=${HEADSET_MAX_BITRATE_MBPS}"
CLOUDXR_LOG_FILE="${CLOUDXR_LOG_FILE:-/tmp/earth2moon_cloudxr_quest3.log}"

if [[ ! -x "${CLOUDXR_LAUNCHER}" ]]; then
    echo "[ERROR] CloudXR launcher not found: ${CLOUDXR_LAUNCHER}" >&2
    exit 1
fi

if [[ ! -x "${SCENE_LAUNCHER}" ]]; then
    echo "[ERROR] Scene launcher not found: ${SCENE_LAUNCHER}" >&2
    exit 1
fi

if [[ ! -f "${SCENE_ENTRY}" ]]; then
    echo "[ERROR] Scene entry script not found: ${SCENE_ENTRY}" >&2
    exit 1
fi

cleanup() {
    if [[ -n "${CLOUDXR_PID:-}" ]]; then
        kill "${CLOUDXR_PID}" >/dev/null 2>&1 || true
        wait "${CLOUDXR_PID}" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

echo "[quest3] starting official IsaacTeleop CloudXR flow for earth2moon-sim..."
echo "[quest3] certificate URL: https://${HOST_IP}:${PROXY_PORT}/"
echo "[quest3] official headset URL: ${HEADSET_CLIENT_URL}"

HOST_IP="${HOST_IP}" PROXY_PORT="${PROXY_PORT}" \
    "${CLOUDXR_LAUNCHER}" >"${CLOUDXR_LOG_FILE}" 2>&1 &
CLOUDXR_PID=$!

deadline=$((SECONDS + STARTUP_WAIT_SEC))
while (( SECONDS < deadline )); do
    if [[ -f "${CLOUDXR_ENV_FILE}" ]] && grep -q "XR_RUNTIME_JSON" "${CLOUDXR_ENV_FILE}"; then
        break
    fi
    if ! kill -0 "${CLOUDXR_PID}" >/dev/null 2>&1; then
        echo "[ERROR] CloudXR server exited early. Log: ${CLOUDXR_LOG_FILE}" >&2
        tail -n 80 "${CLOUDXR_LOG_FILE}" >&2 || true
        exit 1
    fi
    sleep 1
done

if [[ ! -f "${CLOUDXR_ENV_FILE}" ]]; then
    echo "[ERROR] CloudXR env file not ready: ${CLOUDXR_ENV_FILE}" >&2
    echo "Check CloudXR log: ${CLOUDXR_LOG_FILE}" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "${CLOUDXR_ENV_FILE}"

if [[ -n "${XR_RUNTIME_JSON:-}" ]] && [[ -f "${XR_RUNTIME_JSON}" ]]; then
    python3 - "$XR_RUNTIME_JSON" <<'PY'
import json
import os
import sys

json_path = os.path.abspath(sys.argv[1])
json_dir = os.path.dirname(json_path)

with open(json_path, "r", encoding="utf-8") as f:
    payload = json.load(f)

runtime = payload.get("runtime", {})
library_path = runtime.get("library_path")
if not library_path:
    sys.exit(0)

if os.path.isabs(library_path):
    sys.exit(0)

candidates = [
    os.path.abspath(os.path.join(json_dir, library_path)),
    os.path.abspath(os.path.join(os.path.dirname(json_dir), library_path)),
]

resolved = next((candidate for candidate in candidates if os.path.exists(candidate)), None)
if resolved is None:
    sys.exit(0)

runtime["library_path"] = resolved
payload["runtime"] = runtime
with open(json_path, "w", encoding="utf-8") as f:
    json.dump(payload, f, indent=4)
    f.write("\n")

print(f"[quest3] normalized XR runtime library_path -> {resolved}")
PY
fi

echo "[quest3] CloudXR runtime ready."
echo "[quest3] opening earth2moon-sim scene in XR OpenXR experience..."
echo "[quest3] CloudXR log: ${CLOUDXR_LOG_FILE}"
echo "[quest3] project root: ${PROJECT_ROOT}"

exec "${SCENE_LAUNCHER}" "${SCENE_ENTRY}" --xr-openxr "$@"
