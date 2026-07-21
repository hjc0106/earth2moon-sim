#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec env SCENE_ENTRY="${SCRIPT_DIR}/keyboard_teleop_ranger_arm.py" \
    "${SCRIPT_DIR}/run_earth2moon_cloudxr_quest3.sh" \
    --enable-openxr-r1pro-vr \
    --ik-damping 0.025 \
    --ik-gain 0.75 \
    --ik-max-joint-step 0.12 \
    --ik-orientation-weight 0.60 \
    --ik-max-position-error 0.08 \
    --ik-max-orientation-error 0.20 \
    --gripper-speed 0.24 \
    --openxr-vr-position-scale 1.6 \
    --openxr-vr-forward-scale 1.5 \
    --openxr-vr-lift-scale 1.5 \
    --openxr-vr-rotation-scale 1.0 \
    --openxr-vr-rotation-alpha 1.0 \
    --openxr-vr-max-position-speed 4.0 \
    --openxr-vr-max-rotation-speed 6.0 \
    --openxr-vr-torso-speed 1.1 \
    --openxr-vr-base-speed 1.0 \
    --openxr-vr-base-yaw-speed 1.25 \
    --add-r1pro \
    --r1pro-physics \
    --r1pro-x 1.8 \
    --r1pro-y 0.0 \
    --r1pro-yaw 180.0 \
    --r1pro-init-pose-preset arms_forward_level \
    --ground-z -10.0 \
    --r1pro-z 0.0 \
    "$@"
