# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Tiangong locomanipulation pick-place tasks.

Registers the VR-teleoperable G1 fixed-base upper-body IK pick-place task set inside the Tiangong
(Earth-to-Moon) space station scene. Importing this subpackage triggers the gym.register() call;
the teleop driver script imports this module to make the task available to gym.make().
"""

import gymnasium as gym

from . import tiangong_pick_place_g1_env_cfg

gym.register(
    id="Isaac-TiangongPickPlace-FixedBaseUpperBodyIK-G1-Abs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": tiangong_pick_place_g1_env_cfg.TiangongPickPlaceG1EnvCfg,
    },
    disable_env_checker=True,
)
