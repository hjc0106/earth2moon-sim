#!/usr/bin/env bash

# Launch the VR (or keyboard) teleop driver for the Tiangong G1 pick-place task.
#
# This wraps scripts/vr_teleop_se3_agent.py the same way run_with_isaaclab.sh wraps the keyboard
# entry, but additionally puts IsaacLab's source packages (isaaclab, isaaclab_assets,
# isaaclab_tasks) and the sister tiangong / tiangong_tasks packages on PYTHONPATH so the teleop
# script can import them. It also points TIANGONG_ISAAC_ASSET_ROOT at the local Isaac/IsaacLab
# asset mirror so the G1 USD and kinematics URDF resolve offline (no Omniverse nucleus).
#
# Usage:
#   bash scripts/run_vr_teleop.sh                                     # VR motion controllers (default)
#   bash scripts/run_vr_teleop.sh --teleop_device keyboard             # keyboard fallback (no VR)
#   bash scripts/run_vr_teleop.sh --teleop_device motion_controllers \
#       --enable_pinocchio --record_joint_data                        # full VR pipeline + recording
#
# Any extra args after the script flags are forwarded to vr_teleop_se3_agent.py.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"

# Default IsaacLab source root: the sibling IsaacLab checkout used to run the upstream teleop demo.
ISAACLAB_ROOT_DEFAULT="${REPO_ROOT}/IsaacLab"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-${ISAACLAB_ROOT_DEFAULT}}"

# Default local Isaac/IsaacLab asset mirror (contains the G1 USD + kinematics URDF offline).
LOCAL_ASSET_ROOT_DEFAULT="/media/qylab/Data/hjc_space/data/issac_lab/isaacsim_assets"
LOCAL_ASSET_ROOT="${TIANGONG_ISAAC_ASSET_ROOT:-${LOCAL_ASSET_ROOT_DEFAULT}}"

source setup.env

# Asset-path env vars consumed by tiangong.utils.assets and tiangong_tasks.
export TIANGONG_ASSET_ROOT="${TIANGONG_LOCAL_ASSET_ROOT:-${PROJECT_ROOT}/assets}"
export TIANGONG_PROJECT_ASSETS_ROOT="${PROJECT_ROOT}/assets"
export TIANGONG_ISAAC_ASSET_ROOT="${LOCAL_ASSET_ROOT}"

# Locate the Isaac Sim Python interpreter (same logic as run_with_isaaclab.sh).
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

# Build PYTHONPATH: project packages first, then IsaacLab source packages (so the teleop script can
# import isaaclab / isaaclab_assets / isaaclab_tasks). If IsaacLab is pip-installed in the active
# env, these extra entries are harmless duplicates.
EXTRA_PYTHONPATH=(
    "${PROJECT_ROOT}/source/tiangong"
    "${PROJECT_ROOT}/source/tiangong_tasks"
)

if [[ -d "${ISAACLAB_ROOT}/source/isaaclab" ]]; then
    EXTRA_PYTHONPATH+=("${ISAACLAB_ROOT}/source/isaaclab")
else
    echo "[WARN] IsaacLab source not found at ${ISAACLAB_ROOT}/source/isaaclab." >&2
    echo "       Set ISAACLAB_ROOT to your IsaacLab checkout if isaaclab is not pip-installed." >&2
fi
if [[ -d "${ISAACLAB_ROOT}/source/isaaclab_assets" ]]; then
    EXTRA_PYTHONPATH+=("${ISAACLAB_ROOT}/source/isaaclab_assets")
fi
if [[ -d "${ISAACLAB_ROOT}/source/isaaclab_tasks" ]]; then
    EXTRA_PYTHONPATH+=("${ISAACLAB_ROOT}/source/isaaclab_tasks")
fi

LOCAL_SITE_PACKAGES="${PROJECT_ROOT}/.isaaclab_site/lib/python3.11/site-packages"
if [[ -d "${LOCAL_SITE_PACKAGES}" ]]; then
    EXTRA_PYTHONPATH+=("${LOCAL_SITE_PACKAGES}")
fi

EXTRA_PATH="$(IFS=:; echo "${EXTRA_PYTHONPATH[*]}")"
if [[ -n "${PYTHONPATH:-}" ]]; then
    export PYTHONPATH="${EXTRA_PATH}:${PYTHONPATH}"
else
    export PYTHONPATH="${EXTRA_PATH}"
fi

# The default task (Isaac-TiangongPickPlace-FixedBaseUpperBodyIK-G1-Abs-v0) is a Pink IK task that
# requires pinocchio to be imported BEFORE AppLauncher starts Isaac Sim (otherwise Isaac Sim loads
# its own incompatible pinocchio version). If the caller did not explicitly pass --enable_pinocchio
# or --no-pinocchio, inject --enable_pinocchio automatically.
need_pinocchio=yes
for arg in "$@"; do
    case "$arg" in
        --enable_pinocchio) need_pinocchio=no ;;
        --no-pinocchio) need_pinocchio=no ;;
    esac
done
if [[ "$need_pinocchio" == "yes" ]]; then
    set -- "$@" --enable_pinocchio
fi

# Forward all args (possibly augmented with --enable_pinocchio) to the teleop script.
exec "${PYTHON_CMD}" "${SCRIPT_DIR}/vr_teleop_se3_agent.py" "$@"
