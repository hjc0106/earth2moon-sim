"""Minimal smoke test: build the Tiangong G1 pick-place env, reset, step a few times, exit.

Does NOT enter the infinite teleop loop. Verifies that the task registers, the scene (G1 +
packing table + steering wheel + Tiangong station background) builds, Pink IK initializes,
and env.reset()/env.step() work. Run with:

    bash scripts/run_vr_teleop.sh scripts/smoke_test_tiangong_env.py --headless
"""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--task", type=str, default="Isaac-TiangongPickPlace-FixedBaseUpperBodyIK-G1-Abs-v0")
parser.add_argument("--num_envs", type=int, default=1)
args_cli = parser.parse_args()

# This task needs pinocchio imported before AppLauncher (same as teleop_se3_agent.py).
import pinocchio  # noqa: F401

app_launcher_args = vars(args_cli)
app_launcher = AppLauncher(app_launcher_args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
import isaaclab_tasks.manager_based.locomanipulation.pick_place  # noqa: F401
import isaaclab_tasks.manager_based.manipulation.pick_place  # noqa: F401
import tiangong_tasks.manager_based.locomanipulation.pick_place  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg
from isaaclab.envs import ManagerBasedRLEnvCfg

def main():
    print(f"[smoke] Parsing env cfg for task: {args_cli.task}")
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.env_name = args_cli.task
    assert isinstance(env_cfg, ManagerBasedRLEnvCfg), f"Expected ManagerBasedRLEnvCfg, got {type(env_cfg)}"

    # Verify our overrides took effect
    print(f"[smoke] scene.robot.spawn.usd_path = {env_cfg.scene.robot.spawn.usd_path}")
    print(f"[smoke] scene.robot.init_state.pos = {env_cfg.scene.robot.init_state.pos}")
    print(f"[smoke] scene.packing_table.init_state.pos = {env_cfg.scene.packing_table.init_state.pos}")
    print(f"[smoke] scene.object.init_state.pos = {env_cfg.scene.object.init_state.pos}")
    print(f"[smoke] scene.station.prim_path = {env_cfg.scene.station.prim_path}")
    print(f"[smoke] scene.station.spawn.usd_path = {env_cfg.scene.station.spawn.usd_path}")
    print(f"[smoke] scene.ground = {env_cfg.scene.ground}")
    print(f"[smoke] actions.upper_body_ik.controller.urdf_path = {env_cfg.actions.upper_body_ik.controller.urdf_path}")
    print(f"[smoke] xr.anchor_pos = {env_cfg.xr.anchor_pos}")
    print(f"[smoke] teleop_devices keys = {list(env_cfg.teleop_devices.devices.keys())}")

    print("[smoke] Creating environment...")
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    print("[smoke] env.reset()...")
    obs, info = env.reset()
    print(f"[smoke] reset OK. obs keys: {list(obs.keys()) if isinstance(obs, dict) else type(obs)}")

    robot = env.scene["robot"]
    print(f"[smoke] robot.num_joints = {robot.num_joints}")
    print(f"[smoke] robot.body_names (first 5) = {robot.body_names[:5]}")
    print(f"[smoke] robot.data.root_pos_w = {robot.data.root_pos_w.cpu().numpy()}")

    obj = env.scene.rigid_objects.get("object")
    if obj is not None:
        print(f"[smoke] object root_pos_w = {obj.data.root_pos_w.cpu().numpy()}")

    print("[smoke] Stepping 10 times with zero actions...")
    action_dim = env.action_manager.total_action_dim
    print(f"[smoke] action_dim = {action_dim}")
    zero_action = torch.zeros(env.num_envs, action_dim, device=env.device)
    for i in range(10):
        with torch.inference_mode():
            obs, reward, terminated, truncated, info = env.step(zero_action)
    print(f"[smoke] 10 steps OK. robot root_pos after steps = {robot.data.root_pos_w.cpu().numpy()}")
    print(f"[smoke] object root_pos after steps = {obj.data.root_pos_w.cpu().numpy() if obj is not None else 'N/A'}")

    print("[smoke] SMOKE TEST PASSED")
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
