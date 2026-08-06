#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Minimal local scene: both project robots, no CloudXR, no external services,
# and no Isaac asset server. Extra CLI options supplied by the operator are
# appended unchanged.
exec "${SCRIPT_DIR}/run_with_isaaclab.sh" \
    "${SCRIPT_DIR}/keyboard_teleop_ranger_arm.py" \
    --headless \
    --no-show-robot-feedback \
    --add-r1pro \
    --r1pro-physics \
    "$@"
