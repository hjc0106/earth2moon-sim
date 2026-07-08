# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""VR / multi-device teleoperation entry for the Earth-to-Moon (Tiangong) space station sim.

This is a minimally-adapted copy of IsaacLab's ``scripts/environments/teleoperation/teleop_se3_agent.py``.
The only changes vs. upstream are:
  1. Default ``--task`` is ``Isaac-TiangongPickPlace-FixedBaseUpperBodyIK-G1-Abs-v0`` (the Tiangong
     G1 pick-place task registered by the sister ``tiangong_tasks`` package).
  2. An extra ``import tiangong_tasks.manager_based.locomanipulation.pick_place`` runs alongside
     the upstream task imports so the Tiangong task's gym.register() fires.
Everything else (main loop, JointDataRecorder, OpenXR device factory, Pink IK wiring) is unchanged
from upstream so the full VR motion-controller -> G1 upper-body chain stays in sync with IsaacLab.

Supports multiple input devices (e.g., keyboard, spacemouse, gamepad) and devices
configured within the environment (including OpenXR-based hand tracking or motion
controllers)."""

"""Launch Isaac Sim Simulator first."""

import argparse
from collections.abc import Callable

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Teleoperation for Isaac Lab environments.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument(
    "--teleop_device",
    type=str,
    default="keyboard",
    help=(
        "Teleop device. Set here (legacy) or via the environment config. If using the environment config, pass the"
        " device key/name defined under 'teleop_devices' (it can be a custom name, not necessarily 'handtracking')."
        " Built-ins: keyboard, spacemouse, gamepad. Not all tasks support all built-ins."
    ),
)
parser.add_argument(
    "--task",
    type=str,
    default="Isaac-TiangongPickPlace-FixedBaseUpperBodyIK-G1-Abs-v0",
    help="Name of the task. Default is the Tiangong (Earth-to-Moon station) G1 pick-place task.",
)
parser.add_argument("--sensitivity", type=float, default=1.0, help="Sensitivity factor.")
parser.add_argument(
    "--enable_pinocchio",
    action="store_true",
    default=False,
    help="Enable Pinocchio.",
)
parser.add_argument(
    "--record_joint_data",
    action="store_true",
    default=False,
    help="Record robot joint feedback data during active teleoperation.",
)
parser.add_argument(
    "--record_dir",
    type=str,
    default="logs/teleop_joint_records",
    help="Directory used for joint recording sessions.",
)
parser.add_argument(
    "--record_every_n_steps",
    type=int,
    default=1,
    help="Record one sample every N active teleoperation environment steps.",
)
parser.add_argument(
    "--record_object_name",
    type=str,
    default="object",
    help="Rigid object name used to split episodes when it returns to its initial pose.",
)
parser.add_argument(
    "--record_object_return_pos_threshold",
    type=float,
    default=0.03,
    help="Position threshold in meters for considering the recorded object returned to its initial pose.",
)
parser.add_argument(
    "--record_object_leave_pos_threshold",
    type=float,
    default=0.06,
    help="Position threshold in meters for considering the recorded object has left its initial pose.",
)
parser.add_argument(
    "--record_object_return_rot_threshold",
    type=float,
    default=0.25,
    help="Rotation threshold in radians for considering the recorded object returned to its initial pose.",
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

app_launcher_args = vars(args_cli)

if args_cli.enable_pinocchio:
    # Import pinocchio before AppLauncher to force the use of the version installed by IsaacLab and
    # not the one installed by Isaac Sim pinocchio is required by the Pink IK controllers and the
    # GR1T2 retargeter
    import pinocchio  # noqa: F401
teleop_device_name = args_cli.teleop_device.lower()
if any(xr_device in teleop_device_name for xr_device in ("handtracking", "motion_controller", "openxr")):
    app_launcher_args["xr"] = True

# launch omniverse app
app_launcher = AppLauncher(app_launcher_args)
simulation_app = app_launcher.app

"""Rest everything follows."""


import logging
import json
import time
from datetime import datetime
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch

from isaaclab.devices import Se3Gamepad, Se3GamepadCfg, Se3Keyboard, Se3KeyboardCfg, Se3SpaceMouse, Se3SpaceMouseCfg
from isaaclab.devices.openxr import remove_camera_configs
from isaaclab.devices.teleop_device_factory import create_teleop_device
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.utils import parse_env_cfg

if args_cli.enable_pinocchio:
    import isaaclab_tasks.manager_based.locomanipulation.pick_place  # noqa: F401
    import isaaclab_tasks.manager_based.manipulation.pick_place  # noqa: F401
    # Earth-to-Moon (Tiangong) tasks: importing this subpackage runs gym.register() for
    # "Isaac-TiangongPickPlace-FixedBaseUpperBodyIK-G1-Abs-v0" (the default --task above).
    import tiangong_tasks.manager_based.locomanipulation.pick_place  # noqa: F401

# import logger
logger = logging.getLogger(__name__)


def _safe_path_name(value: str | None) -> str:
    """Convert task/device names into filesystem-friendly names."""
    value = "none" if value is None else str(value)
    return "".join(char if char.isalnum() or char in ("-", "_", ".") else "_" for char in value)


class JointDataRecorder:
    """Records robot joint feedback data for teleoperation episodes."""

    def __init__(
        self,
        env,
        task_name: str,
        teleop_device: str,
        output_dir: str,
        sample_every: int,
        object_name: str,
        object_return_pos_threshold: float,
        object_leave_pos_threshold: float,
        object_return_rot_threshold: float,
    ):
        self._env = env
        self._robot = env.scene["robot"]
        self._recorded_body_names = ["left_wrist_yaw_link", "right_wrist_yaw_link"]
        self._recorded_body_ids = [
            self._robot.body_names.index(body_name)
            for body_name in self._recorded_body_names
            if body_name in self._robot.body_names
        ]
        self._recorded_body_names = [self._robot.body_names[body_id] for body_id in self._recorded_body_ids]
        self._sample_every = max(1, sample_every)
        self._object_name = object_name
        self._object = env.scene.rigid_objects.get(object_name)
        self._object_return_pos_threshold = object_return_pos_threshold
        self._object_leave_pos_threshold = max(object_leave_pos_threshold, object_return_pos_threshold)
        self._object_return_rot_threshold = object_return_rot_threshold
        self._object_initial_pose_w: torch.Tensor | None = None
        self._object_has_left_initial_pose = False
        self._active_step_count = 0
        self._episode_index = -1
        self._episode_start_wall_time = 0.0
        self._buffers: dict[str, list] = {}

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_name = f"{timestamp}_{_safe_path_name(task_name)}_{_safe_path_name(teleop_device)}"
        self.session_dir = Path(output_dir).expanduser().resolve() / session_name
        self.session_dir.mkdir(parents=True, exist_ok=True)

        metadata = {
            "task": task_name,
            "teleop_device": teleop_device,
            "created_at": timestamp,
            "num_envs": env.num_envs,
            "step_dt": env.step_dt,
            "physics_dt": env.physics_dt,
            "decimation": env.cfg.decimation,
            "sample_every_active_steps": self._sample_every,
            "joint_names": list(self._robot.joint_names),
            "body_names": list(self._robot.body_names),
            "recorded_body_names": self._recorded_body_names,
            "episode_split": {
                "object_name": object_name if self._object is not None else None,
                "object_return_pos_threshold_m": self._object_return_pos_threshold,
                "object_leave_pos_threshold_m": self._object_leave_pos_threshold,
                "object_return_rot_threshold_rad": self._object_return_rot_threshold,
            },
            "format": {
                "joint_pos": "shape [num_samples, num_envs, num_joints], radians",
                "joint_vel": "shape [num_samples, num_envs, num_joints], radians/second",
                "joint_pos_target": "shape [num_samples, num_envs, num_joints], radians",
                "joint_vel_target": "shape [num_samples, num_envs, num_joints], radians/second",
                "object_root_state": "shape [num_samples, num_envs, 13], [pos, quat(wxyz), lin_vel, ang_vel]",
                "body_link_state": (
                    "shape [num_samples, num_envs, num_recorded_bodies, 13], "
                    "[pos, quat(wxyz), lin_vel, ang_vel]"
                ),
                "action": "shape [num_samples, num_envs, action_dim], teleop action sent to env.step",
            },
        }
        with open(self.session_dir / "metadata.json", "w", encoding="utf-8") as metadata_file:
            json.dump(metadata, metadata_file, indent=2)

        self.start_episode()

    def start_episode(self) -> None:
        self._episode_index += 1
        self._active_step_count = 0
        self._episode_start_wall_time = time.time()
        self._object_initial_pose_w = self._get_object_pose_w()
        self._object_has_left_initial_pose = False
        self._buffers = {
            "global_step": [],
            "episode_step": [],
            "sim_time_s": [],
            "wall_time_s": [],
            "action": [],
            "joint_pos": [],
            "joint_vel": [],
            "joint_pos_target": [],
            "joint_vel_target": [],
            "object_root_state": [],
            "body_link_state": [],
        }

    def record(self, action: torch.Tensor) -> bool:
        self._active_step_count += 1
        should_sample = self._active_step_count % self._sample_every == 0

        robot_data = self._robot.data
        global_step = int(getattr(self._env, "common_step_counter", self._active_step_count))
        episode_step = int(self._env.episode_length_buf[0].item()) if hasattr(self._env, "episode_length_buf") else 0

        if should_sample:
            self._buffers["global_step"].append(global_step)
            self._buffers["episode_step"].append(episode_step)
            self._buffers["sim_time_s"].append(global_step * self._env.step_dt)
            self._buffers["wall_time_s"].append(time.time() - self._episode_start_wall_time)
            self._buffers["action"].append(action.detach().cpu().numpy().copy())
            self._buffers["joint_pos"].append(robot_data.joint_pos.detach().cpu().numpy().copy())
            self._buffers["joint_vel"].append(robot_data.joint_vel.detach().cpu().numpy().copy())
            self._buffers["joint_pos_target"].append(robot_data.joint_pos_target.detach().cpu().numpy().copy())
            self._buffers["joint_vel_target"].append(robot_data.joint_vel_target.detach().cpu().numpy().copy())
            if self._recorded_body_ids:
                self._buffers["body_link_state"].append(
                    robot_data.body_link_state_w[:, self._recorded_body_ids].detach().cpu().numpy().copy()
                )
            if self._object is not None:
                self._buffers["object_root_state"].append(
                    self._object.data.root_state_w.detach().cpu().numpy().copy()
                )

        return self._should_split_on_object_return()

    def _get_object_pose_w(self) -> torch.Tensor | None:
        if self._object is None:
            return None
        return self._object.data.root_link_pose_w.detach().clone()

    def _should_split_on_object_return(self) -> bool:
        if self._object is None or self._object_initial_pose_w is None:
            return False
        if len(self._buffers.get("joint_pos", [])) == 0:
            return False

        object_pose_w = self._get_object_pose_w()
        if object_pose_w is None:
            return False

        pos_error = torch.linalg.norm(object_pose_w[:, :3] - self._object_initial_pose_w[:, :3], dim=-1)
        quat_dot = torch.abs(torch.sum(object_pose_w[:, 3:7] * self._object_initial_pose_w[:, 3:7], dim=-1))
        quat_dot = torch.clamp(quat_dot, 0.0, 1.0)
        rot_error = 2.0 * torch.acos(quat_dot)

        has_left = torch.any(pos_error > self._object_leave_pos_threshold).item()
        if has_left:
            self._object_has_left_initial_pose = True

        has_returned = torch.all(pos_error < self._object_return_pos_threshold).item() and torch.all(
            rot_error < self._object_return_rot_threshold
        ).item()
        return self._object_has_left_initial_pose and has_returned

    def close_episode(self) -> Path | None:
        if len(self._buffers.get("joint_pos", [])) == 0:
            return None

        episode_path = self.session_dir / f"episode_{self._episode_index:04d}.npz"
        np.savez_compressed(
            episode_path,
            joint_names=np.asarray(self._robot.joint_names, dtype=str),
            body_names=np.asarray(self._robot.body_names, dtype=str),
            recorded_body_names=np.asarray(self._recorded_body_names, dtype=str),
            global_step=np.asarray(self._buffers["global_step"], dtype=np.int64),
            episode_step=np.asarray(self._buffers["episode_step"], dtype=np.int64),
            sim_time_s=np.asarray(self._buffers["sim_time_s"], dtype=np.float64),
            wall_time_s=np.asarray(self._buffers["wall_time_s"], dtype=np.float64),
            action=np.stack(self._buffers["action"], axis=0),
            joint_pos=np.stack(self._buffers["joint_pos"], axis=0),
            joint_vel=np.stack(self._buffers["joint_vel"], axis=0),
            joint_pos_target=np.stack(self._buffers["joint_pos_target"], axis=0),
            joint_vel_target=np.stack(self._buffers["joint_vel_target"], axis=0),
            body_link_state=(
                np.stack(self._buffers["body_link_state"], axis=0)
                if len(self._buffers["body_link_state"]) > 0
                else np.empty((0, self._env.num_envs, 0, 13), dtype=np.float32)
            ),
            object_root_state=(
                np.stack(self._buffers["object_root_state"], axis=0)
                if len(self._buffers["object_root_state"]) > 0
                else np.empty((0, self._env.num_envs, 13), dtype=np.float32)
            ),
        )
        print(f"Saved joint recording episode: {episode_path}")
        return episode_path

    def close(self) -> None:
        self.close_episode()


def main() -> None:
    """
    Run teleoperation with an Isaac Lab manipulation environment.

    Creates the environment, sets up teleoperation interfaces and callbacks,
    and runs the main simulation loop until the application is closed.

    Returns:
        None
    """
    # parse configuration
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.env_name = args_cli.task
    if not isinstance(env_cfg, ManagerBasedRLEnvCfg):
        raise ValueError(
            "Teleoperation is only supported for ManagerBasedRLEnv environments. "
            f"Received environment config type: {type(env_cfg).__name__}"
        )
    # modify configuration
    env_cfg.terminations.time_out = None
    if "Lift" in args_cli.task:
        # set the resampling time range to large number to avoid resampling
        env_cfg.commands.object_pose.resampling_time_range = (1.0e9, 1.0e9)
        # add termination condition for reaching the goal otherwise the environment won't reset
        env_cfg.terminations.object_reached_goal = DoneTerm(func=mdp.object_reached_goal)

    if args_cli.xr:
        env_cfg = remove_camera_configs(env_cfg)
        env_cfg.sim.render.antialiasing_mode = "DLSS"

    try:
        # create environment
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
        # check environment name (for reach , we don't allow the gripper)
        if "Reach" in args_cli.task:
            logger.warning(
                f"The environment '{args_cli.task}' does not support gripper control. The device command will be"
                " ignored."
            )
    except Exception as e:
        logger.error(f"Failed to create environment: {e}")
        simulation_app.close()
        return

    # Flags for controlling teleoperation flow
    should_reset_recording_instance = False
    teleoperation_active = True

    # Callback handlers
    def reset_recording_instance() -> None:
        """
        Reset the environment to its initial state.

        Sets a flag to reset the environment on the next simulation step.

        Returns:
            None
        """
        nonlocal should_reset_recording_instance
        should_reset_recording_instance = True
        print("Reset triggered - Environment will reset on next step")

    def start_teleoperation() -> None:
        """
        Activate teleoperation control of the robot.

        Enables the application of teleoperation commands to the environment.

        Returns:
            None
        """
        nonlocal teleoperation_active
        teleoperation_active = True
        print("Teleoperation activated")

    def stop_teleoperation() -> None:
        """
        Deactivate teleoperation control of the robot.

        Disables the application of teleoperation commands to the environment.

        Returns:
            None
        """
        nonlocal teleoperation_active
        teleoperation_active = False
        print("Teleoperation deactivated")

    # Create device config if not already in env_cfg
    teleoperation_callbacks: dict[str, Callable[[], None]] = {
        "R": reset_recording_instance,
        "START": start_teleoperation,
        "STOP": stop_teleoperation,
        "RESET": reset_recording_instance,
    }

    # For hand tracking devices, add additional callbacks
    if args_cli.xr:
        # Default to inactive for hand tracking
        teleoperation_active = False
    else:
        # Always active for other devices
        teleoperation_active = True

    # Create teleop device from config if present, otherwise create manually
    teleop_interface = None
    try:
        if hasattr(env_cfg, "teleop_devices") and args_cli.teleop_device in env_cfg.teleop_devices.devices:
            teleop_interface = create_teleop_device(
                args_cli.teleop_device, env_cfg.teleop_devices.devices, teleoperation_callbacks
            )
        else:
            logger.warning(
                f"No teleop device '{args_cli.teleop_device}' found in environment config. Creating default."
            )
            # Create fallback teleop device
            sensitivity = args_cli.sensitivity
            if args_cli.teleop_device.lower() == "keyboard":
                teleop_interface = Se3Keyboard(
                    Se3KeyboardCfg(pos_sensitivity=0.05 * sensitivity, rot_sensitivity=0.05 * sensitivity)
                )
            elif args_cli.teleop_device.lower() == "spacemouse":
                teleop_interface = Se3SpaceMouse(
                    Se3SpaceMouseCfg(pos_sensitivity=0.05 * sensitivity, rot_sensitivity=0.05 * sensitivity)
                )
            elif args_cli.teleop_device.lower() == "gamepad":
                teleop_interface = Se3Gamepad(
                    Se3GamepadCfg(pos_sensitivity=0.1 * sensitivity, rot_sensitivity=0.1 * sensitivity)
                )
            else:
                logger.error(f"Unsupported teleop device: {args_cli.teleop_device}")
                logger.error("Configure the teleop device in the environment config.")
                env.close()
                simulation_app.close()
                return

            # Add callbacks to fallback device
            for key, callback in teleoperation_callbacks.items():
                try:
                    teleop_interface.add_callback(key, callback)
                except (ValueError, TypeError) as e:
                    logger.warning(f"Failed to add callback for key {key}: {e}")
    except Exception as e:
        logger.error(f"Failed to create teleop device: {e}")
        env.close()
        simulation_app.close()
        return

    if teleop_interface is None:
        logger.error("Failed to create teleop interface")
        env.close()
        simulation_app.close()
        return

    print(f"Using teleop device: {teleop_interface}")

    # reset environment
    env.reset()
    teleop_interface.reset()

    joint_recorder = None
    if args_cli.record_joint_data:
        try:
            joint_recorder = JointDataRecorder(
                env=env,
                task_name=args_cli.task,
                teleop_device=args_cli.teleop_device,
                output_dir=args_cli.record_dir,
                sample_every=args_cli.record_every_n_steps,
                object_name=args_cli.record_object_name,
                object_return_pos_threshold=args_cli.record_object_return_pos_threshold,
                object_leave_pos_threshold=args_cli.record_object_leave_pos_threshold,
                object_return_rot_threshold=args_cli.record_object_return_rot_threshold,
            )
            print(f"Recording joint feedback data to: {joint_recorder.session_dir}")
        except Exception as e:
            logger.error(f"Failed to initialize joint data recorder: {e}")
            env.close()
            simulation_app.close()
            return

    print("Teleoperation started. Press 'R' to reset the environment.")

    # simulate environment
    while simulation_app.is_running():
        try:
            # run everything in inference mode
            with torch.inference_mode():
                # get device command
                action = teleop_interface.advance()

                # Only apply teleop commands when active
                if teleoperation_active:
                    # process actions
                    actions = action.repeat(env.num_envs, 1)
                    # apply actions
                    env.step(actions)
                    if joint_recorder is not None:
                        object_returned = joint_recorder.record(actions)
                        if object_returned:
                            joint_recorder.close_episode()
                            joint_recorder.start_episode()
                            print("Object returned to initial pose; started a new recording episode")
                else:
                    env.sim.render()

                if should_reset_recording_instance:
                    if joint_recorder is not None:
                        joint_recorder.close_episode()
                    env.reset()
                    teleop_interface.reset()
                    if joint_recorder is not None:
                        joint_recorder.start_episode()
                    should_reset_recording_instance = False
                    print("Environment reset complete")
        except Exception as e:
            logger.error(f"Error during simulation step: {e}")
            break

    # close the simulator
    if joint_recorder is not None:
        joint_recorder.close()
    env.close()
    print("Environment closed")


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
