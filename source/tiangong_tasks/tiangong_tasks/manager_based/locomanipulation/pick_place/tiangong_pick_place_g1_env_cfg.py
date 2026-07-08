# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Environment configuration for the G1 fixed-base upper-body IK pick-place task set inside the
Tiangong (Earth-to-Moon) space station scene, with VR motion-controller teleoperation.

This configuration inherits the upstream ``Isaac-PickPlace-FixedBaseUpperBodyIK-G1-Abs-v0`` task
(G1 humanoid + packing table + steering-wheel object + Pink IK + OpenXR motion-controller
retargeter) and relocates the manipulation setup onto the station's interior floor (z ≈ -10),
using the Tiangong space station USD as a non-cloned visual background.

The action space, observations, terminations, teleop devices, and Pink IK controller are all
inherited unchanged from the upstream task; only the scene geometry / asset paths / XR anchor
position are overridden. See the upstream file for the full MDP definition:

    isaaclab_tasks.manager_based.locomanipulation.pick_place.fixed_base_upper_body_ik_g1_env_cfg
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.devices.device_base import DevicesCfg
from isaaclab.devices.openxr import OpenXRDeviceCfg, XrCfg
from isaaclab.devices.openxr.retargeters.humanoid.unitree.trihand.g1_upper_body_motion_ctrl_retargeter import (
    G1TriHandUpperBodyMotionControllerRetargeterCfg,
)
from isaaclab.devices.openxr.retargeters.humanoid.unitree.trihand.g1_upper_body_retargeter import (
    G1TriHandUpperBodyRetargeterCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass

from isaaclab_tasks.manager_based.locomanipulation.pick_place.fixed_base_upper_body_ik_g1_env_cfg import (
    FixedBaseUpperBodyIKG1EnvCfg,
    FixedBaseUpperBodyIKG1SceneCfg,
)

# Asset-path utility from the sister tiangong package (the teleop keyboard workspace).
# The tiangong package is expected to be importable on PYTHONPATH when this task is registered.
try:
    from tiangong.utils.assets import TIANGONG_SPACE_STATION_ASSET_PATH
except Exception:  # pragma: no cover - fallback when tiangong package is not on path
    TIANGONG_SPACE_STATION_ASSET_PATH = os.environ.get(
        "TIANGONG_SPACE_STATION_ASSET_PATH",
        "",
    )

##
# Local offline asset root for Isaac/IsaacLab nucleus assets (G1 USD + kinematics URDF).
# Override with the TIANGONG_ISAAC_ASSET_ROOT environment variable; otherwise fall back to the
# known local mirror on this machine. The upstream task fetches these from the Omniverse nucleus,
# which is unavailable offline, so we hard-override the USD/URDF paths in __post_init__ below.
##
_LOCAL_ASSET_ROOT = os.environ.get(
    "TIANGONG_ISAAC_ASSET_ROOT",
    "/media/qylab/Data/hjc_space/data/issac_lab/isaacsim_assets",
)

# Vertical offset from the upstream (ground = z=0) frame to the Tiangong station interior floor.
# The station's GroundPlane sits at z = -10.0; shifting everything by this constant places the
# G1 pelvis, table, and object on the station floor.
GROUND_Z = -10.0


@configclass
class TiangongPickPlaceG1SceneCfg(FixedBaseUpperBodyIKG1SceneCfg):
    """Scene configuration that places the G1 pick-place setup inside the Tiangong space station.

    The station is loaded once as a non-cloned background prim (``/World/TiangongStation``), so the
    150 MB station USD is not duplicated per environment. The G1 robot, packing table, and rigid
    pick-place object are shifted down by :data:`GROUND_Z` so they rest on the station's interior
    floor (the station's own ``/World/GroundPlane`` provides the collision surface).
    """

    # Tiangong space station as a one-off visual background (NOT under {ENV_REGEX_NS}).
    station = AssetBaseCfg(
        prim_path="/World/TiangongStation",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=UsdFileCfg(
            usd_path=str(TIANGONG_SPACE_STATION_ASSET_PATH),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )

    # Shift the table, object, and robot down onto the station floor.
    # NOTE: these override the parent class field defaults; configclass supports re-assignment
    # of inherited fields in the class body. USD paths point at the local Isaac/IsaacLab asset
    # mirror (TIANGONG_ISAAC_ASSET_ROOT) so the task runs fully offline without an Omniverse nucleus.
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.55, GROUND_Z - 0.3), rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=UsdFileCfg(
            usd_path=f"{_LOCAL_ASSET_ROOT}/Assets/Isaac/5.1/Isaac/Props/PackingTable/packing_table.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )

    object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.45, GROUND_Z + 0.6996), rot=(1.0, 0.0, 0.0, 0.0)
        ),
        spawn=UsdFileCfg(
            usd_path=f"{_LOCAL_ASSET_ROOT}/Assets/Isaac/5.1/Isaac/IsaacLab/Mimic/pick_place_task/pick_place_assets/steering_wheel.usd",
            scale=(0.75, 0.75, 0.75),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        ),
    )

    def __post_init__(self):
        # Run the parent post-init first (it sets robot.fix_root_link = True).
        super().__post_init__()
        # Shift the G1 pelvis down onto the station floor (parent default is z = 0.75).
        self.robot.init_state.pos = (0.0, 0.0, GROUND_Z + 0.75)
        # Drop the parent GroundPlane: the Tiangong station USD already provides a collision
        # surface at z = -10. Keeping both would z-fight and double-collide.
        self.ground = None


@configclass
class TiangongPickPlaceG1EnvCfg(FixedBaseUpperBodyIKG1EnvCfg):
    """Configuration for the G1 fixed-base upper-body IK pick-place task inside the Tiangong station.

    Inherits the full MDP (actions, observations, terminations, teleop devices, Pink IK controller)
    from :class:`FixedBaseUpperBodyIKG1EnvCfg` and only swaps the scene and a few offline-critical
    settings (XR anchor height, local G1 USD / kinematics URDF paths).
    """

    scene: InteractiveSceneCfg = TiangongPickPlaceG1SceneCfg(
        num_envs=1, env_spacing=2.5, replicate_physics=True
    )

    def __post_init__(self):
        # NOTE: we intentionally do NOT call super().__post_init__() here. The parent post-init
        # calls retrieve_file_path() against ISAACLAB_NUCLEUS_DIR to fetch the G1 kinematics URDF,
        # which fails without an Omniverse nucleus server. Instead we replicate the parent's
        # post-init logic verbatim (decimation, sim.dt, teleop_devices) but point the URDF and G1
        # USD at the local asset mirror so the task runs fully offline.

        # --- general settings (copied from FixedBaseUpperBodyIKG1EnvCfg.__post_init__) ---
        self.decimation = 4
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 1 / 200  # 200Hz
        self.sim.render_interval = 2

        # --- Offline G1 kinematics URDF (replaces parent's retrieve_file_path(nucleus) call) ---
        local_urdf = (
            f"{_LOCAL_ASSET_ROOT}/Assets/Isaac/5.1/Isaac/IsaacLab/Controllers/"
            "LocomanipulationAssets/unitree_g1_kinematics_asset/g1_29dof_with_hand_only_kinematics.urdf"
        )
        self.actions.upper_body_ik.controller.urdf_path = local_urdf

        # --- G1 robot USD -> local mirror (avoids nucleus resolution at spawn time) ---
        local_g1_usd = f"{_LOCAL_ASSET_ROOT}/Assets/Isaac/5.1/Isaac/Robots/Unitree/G1/g1.usd"
        self.scene.robot.spawn.usd_path = local_g1_usd

        # --- teleop devices (copied verbatim from parent, minus the nucleus URDF dependency) ---
        # Both the "handtracking" (G1TriHandUpperBodyRetargeter) and "motion_controllers"
        # (G1TriHandUpperBodyMotionControllerRetargeter) OpenXR devices are registered so that
        # --teleop_device motion_controllers (the VR pipeline) works out of the box.
        self.teleop_devices = DevicesCfg(
            devices={
                "handtracking": OpenXRDeviceCfg(
                    retargeters=[
                        G1TriHandUpperBodyRetargeterCfg(
                            enable_visualization=True,
                            # OpenXR hand tracking has 26 joints per hand
                            num_open_xr_hand_joints=2 * 26,
                            sim_device=self.sim.device,
                            hand_joint_names=self.actions.upper_body_ik.hand_joint_names,
                        ),
                    ],
                    sim_device=self.sim.device,
                    xr_cfg=self.xr,
                ),
                "motion_controllers": OpenXRDeviceCfg(
                    retargeters=[
                        G1TriHandUpperBodyMotionControllerRetargeterCfg(
                            enable_visualization=True,
                            sim_device=self.sim.device,
                            hand_joint_names=self.actions.upper_body_ik.hand_joint_names,
                        ),
                    ],
                    sim_device=self.sim.device,
                    xr_cfg=self.xr,
                ),
            }
        )

        # Move the XR anchor down to the station floor so the VR viewpoint starts beside the G1
        # instead of floating at z = -0.45 (the upstream default, which is ground-relative).
        self.xr.anchor_pos = (0.0, 0.0, GROUND_Z - 0.45)
