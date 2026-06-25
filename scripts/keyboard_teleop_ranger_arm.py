"""天宫 IsaacSim 键盘遥操作入口。

负责加载天宫场景、Ranger Arm、R1 Pro 和辅助展示资产，并把键盘输入分发给
对应机器人控制器。Ranger Arm IK、R1 Pro 底盘和地面贴合逻辑都在 teleop 模块中实现。
"""

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from isaacsim import SimulationApp


def _ensure_tiangong_on_path() -> None:
    """把本仓库源码路径加入 sys.path，便于脚本直接从仓库运行。"""
    script_path = Path(__file__).resolve()
    repo_root = script_path.parents[1]
    package_root = repo_root / "source" / "tiangong"
    for candidate in (package_root, repo_root):
        candidate_str = str(candidate)
        if candidate.exists() and candidate_str not in sys.path:
            sys.path.insert(0, candidate_str)


_ensure_tiangong_on_path()

from tiangong.utils.assets import CF2X_ASSET_PATH, TIANGONG_SPACE_STATION_ASSET_PATH, tkmodel_usd_path


def default_r1pro_usd_path() -> str:
    """返回 R1 Pro 默认 USD 路径，优先使用项目内已整理的 assets 版本。"""
    repo_root = Path(__file__).resolve().parents[1]
    candidates = (repo_root / "assets" / "r1pro" / "r1pro.usda",)
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return str(candidates[0])


def default_cf2x_usd_path() -> str:
    """返回 Crazyflie 默认 USD 路径。"""
    return str(Path(CF2X_ASSET_PATH).expanduser())


def default_drone_trajectory_paths() -> tuple[str, str]:
    """返回两架 Crazyflie 默认轨迹 CSV 路径。"""
    repo_root = Path(__file__).resolve().parents[1]
    return (
        str((repo_root / "assets" / "trajectories" / "quadrotor_demo_1.csv").resolve()),
        str((repo_root / "assets" / "trajectories" / "quadrotor_demo_2.csv").resolve()),
    )


@dataclass
class TeleopConfig:
    """基础遥操作速度配置。"""

    speed: float
    turn_rate: float
    lift_rate: float


class KeyboardState:
    """统一 IsaacSim 键盘事件和轮询状态，避免 viewport 焦点丢失时漏按键。"""

    def __init__(self, keys, appwindow, input_iface, carb_module, debug: bool = False):
        self._keys = {key: False for key in keys}
        self._pressed_once = set()
        self._poll_keys = {key: False for key in keys}
        self._appwindow = appwindow
        self._input = input_iface
        self._carb = carb_module
        self._debug = debug
        self._last_event_time = time.time()
        self._keyboard = self._appwindow.get_keyboard()
        self._event_handle = None

    def _event_callback(self, event, *args, **kwargs):
        self._last_event_time = time.time()
        if self._debug:
            self._carb.log_info(f"Keyboard event: input={event.input} type={event.type}")
        if event.input not in self._keys:
            return
        if event.type in (
            self._carb.input.KeyboardEventType.KEY_PRESS,
            self._carb.input.KeyboardEventType.KEY_REPEAT,
        ):
            if event.type == self._carb.input.KeyboardEventType.KEY_PRESS and not self._keys[event.input]:
                self._pressed_once.add(event.input)
            self._keys[event.input] = True
        elif event.type == self._carb.input.KeyboardEventType.KEY_RELEASE:
            self._keys[event.input] = False

    def connect(self):
        if self._keyboard is None:
            self._carb.log_warn("Keyboard device not available. Click the viewport or wait for UI to load.")
            return
        if self._event_handle is None:
            self._event_handle = self._input.subscribe_to_keyboard_events(self._keyboard, self._event_callback)

    def ensure_connected(self):
        if self._keyboard is None and self._appwindow is not None:
            self._keyboard = self._appwindow.get_keyboard()
        if self._keyboard is not None and self._event_handle is None:
            self._event_handle = self._input.subscribe_to_keyboard_events(self._keyboard, self._event_callback)

    def disconnect(self):
        if self._event_handle is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._event_handle)
            self._event_handle = None

    def pressed(self, key) -> bool:
        return bool(self._keys.get(key, False))

    def poll_pressed(self, key) -> bool:
        try:
            if self._keyboard is not None and self._input.get_keyboard_value(self._keyboard, key) > 0:
                return True
            return self._input.get_keyboard_value(None, key) > 0
        except Exception:  # noqa: BLE001
            return False

    def consume_pressed(self, key) -> bool:
        poll_pressed = self.poll_pressed(key)
        poll_was_pressed = bool(self._poll_keys.get(key, False))
        self._poll_keys[key] = poll_pressed
        if key in self._pressed_once:
            self._pressed_once.remove(key)
            return True
        if poll_pressed and not poll_was_pressed:
            return True
        return False

    def seconds_since_event(self) -> float:
        return time.time() - self._last_event_time


def _hide_prim(stage, usdgeom_module, prim_path: str) -> None:
    """从 stage 中移除指定 prim；参数保留 usdgeom_module 以兼容旧调用。"""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    stage.RemovePrim(prim_path)


def _prepare_r1pro_articulation_for_teleop(
    stage,
    gf_module,
    usd_module,
    usdgeom_module,
    usdphysics_module,
    physx_schema_module,
    sdf_module,
    scene_prim_path: str,
    carb_module,
    fixed_base: bool = False,
) -> None:
    """整理 R1 Pro USD 的 articulation/physics 设置，使其能被遥操作控制。

    官方 USD 和旧生成 USD 的 articulation 根节点位置不完全一致，这里会把根 API
    统一到场景根 prim，并根据参数选择自由轮式底盘或固定底座模式。
    """
    scene_prim = stage.GetPrimAtPath(scene_prim_path)
    if not scene_prim.IsValid():
        carb_module.log_warn(f"Cannot prepare missing r1pro prim: {scene_prim_path}")
        return

    def _set_attr(prim, name: str, value_type, value) -> None:
        attr = prim.GetAttribute(name)
        if not attr:
            attr = prim.CreateAttribute(name, value_type)
        attr.Set(value)

    def _copy_schema_attrs(source_prim, target_prim, schema_api) -> None:
        source_api = schema_api(source_prim)
        if not source_api:
            return
        target_api = schema_api(target_prim)
        if not target_api:
            target_api = schema_api.Apply(target_prim)
        for attr_name in source_api.GetSchemaAttributeNames():
            source_attr = source_prim.GetAttribute(attr_name)
            if not source_attr:
                continue
            value = source_attr.Get()
            if value is None:
                continue
            target_attr = target_prim.GetAttribute(attr_name)
            if target_attr:
                target_attr.Set(value)

    root_link_path = f"{scene_prim_path}/base_footprint_x"
    root_link_prim = stage.GetPrimAtPath(root_link_path)
    if not root_link_prim.IsValid():
        base_link_prim = stage.GetPrimAtPath(f"{scene_prim_path}/base_link")
        if not base_link_prim.IsValid():
            scene_prim_range = usd_module.PrimRange(scene_prim)
            for prim in scene_prim_range:
                if prim.IsValid() and prim.GetName() == "base_link" and prim.HasAPI(usdphysics_module.ArticulationRootAPI):
                    base_link_prim = prim
                    break
        if base_link_prim.IsValid() and base_link_prim.HasAPI(usdphysics_module.ArticulationRootAPI):
            for prim in usd_module.PrimRange(scene_prim):
                rigid_body = usdphysics_module.RigidBodyAPI(prim)
                if not rigid_body:
                    continue
                rigid_body.CreateRigidBodyEnabledAttr(True)
                rigid_body.CreateKinematicEnabledAttr(False)
                prim.CreateAttribute("physxRigidBody:disableGravity", sdf_module.ValueTypeNames.Bool).Set(False)
            carb_module.log_warn(f"r1pro official articulation prepared at {scene_prim_path} (wheel-driven base).")
            return
        carb_module.log_warn(f"Cannot fix missing r1pro root link: {root_link_path}")
        return

    # Put the articulation API on the parent prim so the composed r1pro can be
    # controlled as one articulation. The default path leaves the base free and
    # locks the synthetic base-footprint chain internally, so wheel contact can
    # move the robot instead of a world-fixed virtual base.
    if root_link_prim.HasAPI(usdphysics_module.ArticulationRootAPI):
        usdphysics_module.ArticulationRootAPI.Apply(scene_prim)
        root_link_prim.RemoveAPI(usdphysics_module.ArticulationRootAPI)
    elif not scene_prim.HasAPI(usdphysics_module.ArticulationRootAPI):
        usdphysics_module.ArticulationRootAPI.Apply(scene_prim)

    physx_articulation_source = None
    for candidate_path in (f"{scene_prim_path}/base_link", root_link_path, scene_prim_path):
        candidate_prim = stage.GetPrimAtPath(candidate_path)
        if candidate_prim.IsValid() and physx_schema_module.PhysxArticulationAPI(candidate_prim):
            physx_articulation_source = candidate_prim
            break
    if physx_articulation_source is not None:
        _copy_schema_attrs(physx_articulation_source, scene_prim, physx_schema_module.PhysxArticulationAPI)
        if physx_articulation_source != scene_prim:
            physx_articulation_source.RemoveAPI(physx_schema_module.PhysxArticulationAPI)
    elif not physx_schema_module.PhysxArticulationAPI(scene_prim):
        physx_schema_module.PhysxArticulationAPI.Apply(scene_prim)

    fixed_joint_path = f"{scene_prim_path}/teleop_world_fixed_joint"
    if fixed_base:
        fixed_joint = usdphysics_module.FixedJoint.Define(stage, fixed_joint_path)
        fixed_joint_prim = fixed_joint.GetPrim()
        fixed_joint_prim.SetActive(True)
        fixed_joint.CreateJointEnabledAttr(True)
        fixed_joint.CreateBody1Rel().SetTargets([sdf_module.Path(root_link_path)])
        root_pose = usdgeom_module.XformCache().GetLocalToWorldTransform(root_link_prim).RemoveScaleShear()
        fixed_joint.CreateLocalPos0Attr().Set(gf_module.Vec3f(root_pose.ExtractTranslation()))
        fixed_joint.CreateLocalRot0Attr().Set(gf_module.Quatf(root_pose.ExtractRotationQuat()))
        fixed_joint.CreateLocalPos1Attr().Set(gf_module.Vec3f(0.0))
        fixed_joint.CreateLocalRot1Attr().Set(gf_module.Quatf(1.0))
        fixed_joint.CreateBreakForceAttr().Set(3.4028235e38)
        fixed_joint.CreateBreakTorqueAttr().Set(3.4028235e38)
    else:
        fixed_joint_prim = stage.GetPrimAtPath(fixed_joint_path)
        if fixed_joint_prim.IsValid():
            fixed_joint_prim.SetActive(False)

    virtual_base_joints = (
        ("base_footprint_x", "base_footprint_x_joint", "linear", not fixed_base),
        ("base_footprint_y", "base_footprint_y_joint", "linear", not fixed_base),
        ("base_footprint_z", "base_footprint_z_joint", "linear", True),
        ("base_footprint_rx", "base_footprint_rx_joint", "angular", True),
        ("base_footprint_ry", "base_footprint_ry_joint", "angular", True),
        ("base_footprint_rz", "base_footprint_rz_joint", "angular", not fixed_base),
    )
    for body_name, joint_name, drive_name, should_lock in virtual_base_joints:
        joint_prim = stage.GetPrimAtPath(f"{scene_prim_path}/{body_name}/{joint_name}")
        if not joint_prim.IsValid():
            carb_module.log_warn(f"r1pro virtual base joint missing: {joint_name}")
            continue
        joint_prim.SetActive(True)
        usdphysics_module.DriveAPI.Apply(joint_prim, drive_name)
        _set_attr(joint_prim, "physics:jointEnabled", sdf_module.ValueTypeNames.Bool, True)
        if should_lock:
            _set_attr(joint_prim, "physics:lowerLimit", sdf_module.ValueTypeNames.Float, 0.0)
            _set_attr(joint_prim, "physics:upperLimit", sdf_module.ValueTypeNames.Float, 0.0)
            stiffness = 1000000.0
            damping = 100000.0
            max_force = 1000000.0
        else:
            for limit_attr_name in ("physics:lowerLimit", "physics:upperLimit"):
                limit_attr = joint_prim.GetAttribute(limit_attr_name)
                if limit_attr:
                    limit_attr.Clear()
            stiffness = 0.0
            damping = 10000.0
            max_force = 100000.0
        _set_attr(joint_prim, f"drive:{drive_name}:physics:targetPosition", sdf_module.ValueTypeNames.Float, 0.0)
        _set_attr(joint_prim, f"drive:{drive_name}:physics:targetVelocity", sdf_module.ValueTypeNames.Float, 0.0)
        _set_attr(joint_prim, f"drive:{drive_name}:physics:stiffness", sdf_module.ValueTypeNames.Float, stiffness)
        _set_attr(joint_prim, f"drive:{drive_name}:physics:damping", sdf_module.ValueTypeNames.Float, damping)
        _set_attr(joint_prim, f"drive:{drive_name}:physics:maxForce", sdf_module.ValueTypeNames.Float, max_force)
        _set_attr(joint_prim, f"drive:{drive_name}:physics:type", sdf_module.ValueTypeNames.Token, "force")
        state_attr_prefix = f"state:{drive_name}:physics"
        _set_attr(joint_prim, f"{state_attr_prefix}:position", sdf_module.ValueTypeNames.Float, 0.0)
        _set_attr(joint_prim, f"{state_attr_prefix}:velocity", sdf_module.ValueTypeNames.Float, 0.0)

    for prim in usd_module.PrimRange(scene_prim):
        rigid_body = usdphysics_module.RigidBodyAPI(prim)
        if not rigid_body:
            continue
        rigid_body.CreateRigidBodyEnabledAttr(True)
        rigid_body.CreateKinematicEnabledAttr(False)
        prim.CreateAttribute("physxRigidBody:disableGravity", sdf_module.ValueTypeNames.Bool).Set(False)
    mode = "fixed virtual base" if fixed_base else "free wheel-driven base"
    carb_module.log_warn(f"r1pro physics prepared at {scene_prim_path} ({mode}).")


ISAAC_ASSET_BROWSER_FOLDERS = (
    "Robots",
    "Environments",
    "IsaacLab",
    "Materials",
    "People",
    "Props",
    "Samples",
    "Sensors",
)


def _resolve_isaac_asset_roots(asset_root: str) -> tuple[str | None, str | None, str]:
    """解析可选 IsaacSim 资产镜像根目录，并生成 kit 可接受的 file URI。"""
    root = Path(asset_root).expanduser()
    version_root = root / "Assets" / "Isaac" / "5.1"
    candidates = (version_root, root)
    for candidate in candidates:
        isaac_root = candidate / "Isaac"
        if candidate.exists() and isaac_root.exists():
            resolved = candidate.resolve()
            return resolved.as_uri(), isaac_root.resolve().as_uri(), str(resolved)
    return None, None, str(root)


def main() -> None:
    """解析参数、启动 IsaacSim、加载场景，并进入键盘遥操作主循环。"""
    parser = argparse.ArgumentParser(description="Keyboard teleop for /World/ranger_arm.")
    parser.add_argument("--usd", type=str, default=tkmodel_usd_path(), help="Path to the USD scene.")
    parser.add_argument("--prim", type=str, default="/World/ranger_arm", help="Prim path to control.")
    parser.add_argument("--speed", type=float, default=0.05, help="Linear speed in meters per second.")
    parser.add_argument("--turn-rate", type=float, default=10.0, help="Yaw speed in degrees per second.")
    parser.add_argument("--lift-rate", type=float, default=0.05, help="Vertical speed in meters per second.")
    parser.add_argument(
        "--wheel-speed",
        type=float,
        default=3.0,
        help="Default wheel angular speed in radians per second for both robots.",
    )
    parser.add_argument(
        "--ranger-wheel-speed",
        type=float,
        default=4.0,
        help="Ranger Arm wheel angular speed in radians per second. Defaults to --wheel-speed.",
    )
    parser.add_argument(
        "--r1pro-wheel-speed",
        type=float,
        default=4.5,
        help="R1 Pro wheel angular speed in radians per second. Defaults to a faster target-reach value.",
    )
    parser.add_argument(
        "--wheel-turn-speed",
        type=float,
        default=0.8,
        help="Default turn wheel angular speed in radians per second when no steering joints are available.",
    )
    parser.add_argument(
        "--ranger-wheel-turn-speed",
        type=float,
        default=None,
        help="Ranger Arm turn wheel angular speed in radians per second. Defaults to --wheel-turn-speed.",
    )
    parser.add_argument("--left-wheel-sign", type=float, default=1.0, help="Direction multiplier for left wheels.")
    parser.add_argument("--right-wheel-sign", type=float, default=1.0, help="Direction multiplier for right wheels.")
    parser.add_argument("--front-steer-sign", type=float, default=1.0, help="Direction multiplier for front steering joints.")
    parser.add_argument("--rear-steer-sign", type=float, default=-1.0, help="Direction multiplier for rear steering joints.")
    parser.add_argument("--left-steer-sign", type=float, default=1.0, help="Direction multiplier for left steering joints.")
    parser.add_argument("--right-steer-sign", type=float, default=1.0, help="Direction multiplier for right steering joints.")
    parser.add_argument(
        "--ranger-active-steer",
        type=str,
        default="arm",
        choices=("arm", "front", "rear", "all"),
        help="Which Ranger Arm wheel pair can steer. 'arm' selects the pair closest to the arm end.",
    )
    parser.add_argument(
        "--drive-wheels",
        action="store_true",
        default=True,
        help="Drive physical wheel joints. This is enabled by default.",
    )
    parser.add_argument(
        "--move-prim",
        action="store_true",
        dest="deprecated_move_prim",
        help="Deprecated and ignored. Base motion now requires wheel DOFs.",
    )
    parser.add_argument(
        "--wheel-joints",
        type=str,
        default="",
        help="Comma-separated wheel joint prim paths to drive (optional).",
    )
    parser.add_argument(
        "--steer-joints",
        type=str,
        default="",
        help="Comma-separated steering joint prim paths to drive (optional).",
    )
    parser.add_argument(
        "--max-steer-rad",
        type=float,
        default=0.6,
        help="Ranger Arm max steering angle in radians when steering joints are present.",
    )
    parser.add_argument(
        "--r1pro-max-steer-rad",
        type=float,
        default=3.14159,
        help="R1 Pro max steering angle in radians. Keep near pi for side motion.",
    )
    parser.add_argument(
        "--print-joints",
        action="store_true",
        help="Print detected wheel/steering joints and exit.",
    )
    parser.add_argument(
        "--wrap-prim",
        action="store_true",
        help="Wrap the target prim under /World/ranger_arm_teleop for easier translation.",
    )
    parser.add_argument("--dt", type=float, default=1.0 / 60.0, help="Control timestep in seconds.")
    parser.add_argument(
        "--quit-on-esc",
        action="store_true",
        help="Quit when ESC is pressed (default: off).",
    )
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without a UI window.")
    parser.add_argument(
        "--keep-alive",
        action="store_true",
        help="Keep the app running even if is_running() reports false.",
    )
    parser.add_argument(
        "--debug-loop",
        action="store_true",
        help="Print a heartbeat log while the loop is running.",
    )
    parser.add_argument(
        "--debug-input",
        action="store_true",
        help="Log keyboard events and current pressed state.",
    )
    parser.add_argument(
        "--use-ui",
        action="store_true",
        help="Show a small UI panel with sliders for motion control.",
    )
    parser.add_argument(
        "--enable-arm-ik",
        action="store_true",
        default=True,
        help="Enable Ranger Arm arm teleop. Enabled by default; kept for backward compatibility.",
    )
    parser.add_argument(
        "--disable-arm-ik",
        action="store_false",
        dest="enable_arm_ik",
        help="Disable Ranger Arm arm teleop.",
    )
    parser.add_argument(
        "--ik-arm",
        choices=("left", "right", "both"),
        default="both",
        help="Which arm end effector to drive with IK.",
    )
    parser.add_argument(
        "--ik-speed",
        type=float,
        default=0.06,
        help="IK target translation speed in meters per second.",
    )
    parser.add_argument(
        "--ik-rotation-speed",
        type=float,
        default=0.8,
        help="IK target rotation speed in radians per second.",
    )
    parser.add_argument(
        "--ik-gain",
        type=float,
        default=1.0,
        help="Scale applied to each damped least-squares IK joint update.",
    )
    parser.add_argument(
        "--ik-damping",
        type=float,
        default=0.05,
        help="Damping factor for damped least-squares IK.",
    )
    parser.add_argument(
        "--ik-max-joint-step",
        type=float,
        default=0.05,
        help="Maximum IK joint position update per frame in radians.",
    )
    parser.add_argument(
        "--ik-hold-orientation",
        action="store_true",
        help="Also hold the initial end-effector orientation with a 6D pose IK error.",
    )
    parser.add_argument(
        "--left-ee-body",
        type=str,
        default="arm_left_link7",
        help="Rigid body name used as the left-arm IK end effector.",
    )
    parser.add_argument(
        "--right-ee-body",
        type=str,
        default="arm_right_link7",
        help="Rigid body name used as the right-arm IK end effector.",
    )
    parser.add_argument(
        "--gripper-speed",
        type=float,
        default=0.08,
        help="Gripper target speed in meters per second.",
    )
    parser.add_argument(
        "--gripper-stiffness",
        type=float,
        default=5000.0,
        help="Position-drive stiffness for gripper joints.",
    )
    parser.add_argument(
        "--gripper-damping",
        type=float,
        default=500.0,
        help="Position-drive damping for gripper joints.",
    )
    parser.add_argument(
        "--gripper-max-effort",
        type=float,
        default=500.0,
        help="Maximum drive effort for gripper joints.",
    )
    parser.add_argument(
        "--gripper-open-position",
        type=float,
        default=0.035,
        help="Open gripper joint target in meters.",
    )
    parser.add_argument(
        "--gripper-closed-position",
        type=float,
        default=0.0,
        help="Closed gripper joint target in meters.",
    )
    parser.add_argument(
        "--no-stage-animation",
        action="store_false",
        dest="play_stage_animation",
        help="Do not force-play the USD stage animation timeline.",
    )
    parser.add_argument(
        "--stage-animation-duration",
        type=float,
        default=250.0,
        help="Stage animation duration in USD timeCode units. Tiangong Space Station source samples are 0..250.",
    )
    parser.add_argument(
        "--stage-animation-rate",
        type=float,
        default=12.0,
        help="USD timeCode units advanced per wall-clock second for stage animation. 12 gives a slower 20.8s assembly.",
    )
    parser.add_argument(
        "--loop-stage-animation",
        action="store_true",
        help="Loop the stage animation instead of holding the final assembled pose.",
    )
    parser.add_argument(
        "--no-manual-stage-animation",
        action="store_false",
        dest="manual_stage_animation",
        help="Disable manual sampling of Tiangong station USD xform animation.",
    )
    parser.add_argument(
        "--station-animation-usd",
        type=str,
        default=TIANGONG_SPACE_STATION_ASSET_PATH,
        help="Source USD used as a fallback for Tiangong station xform animation samples.",
    )
    parser.add_argument(
        "--station-assembly-progress-scale",
        type=float,
        default=1.0,
        help="Translate progress multiplier for Tiangong assembly. Keep 1.0 to preserve the original final pose.",
    )
    parser.add_argument(
        "--ground-drones",
        action="store_true",
        default=True,
        help="Place /World/cf2x and /World/cf2x_01 near the ground.",
    )
    parser.add_argument(
        "--no-ground-drones",
        action="store_false",
        dest="ground_drones",
        help="Keep /World/cf2x and /World/cf2x_01 at their authored positions.",
    )
    parser.add_argument(
        "--drone-scale",
        type=float,
        default=4.0,
        help="Uniform scale applied to /World/cf2x and /World/cf2x_01 when grounding them.",
    )
    default_drone_traj_1, default_drone_traj_2 = default_drone_trajectory_paths()
    parser.add_argument(
        "--enable-drone-trajectories",
        action="store_true",
        default=True,
        help="Drive /World/cf2x and /World/cf2x_01 along their CSV trajectories.",
    )
    parser.add_argument(
        "--no-enable-drone-trajectories",
        action="store_false",
        dest="enable_drone_trajectories",
        help="Load the Crazyflie assets without animating them from CSV trajectories.",
    )
    parser.add_argument(
        "--cf2x-trajectory-1",
        type=str,
        default=default_drone_traj_1,
        help="CSV trajectory for /World/cf2x.",
    )
    parser.add_argument(
        "--cf2x-trajectory-2",
        type=str,
        default=default_drone_traj_2,
        help="CSV trajectory for /World/cf2x_01.",
    )
    parser.add_argument(
        "--drone-trajectory-loop",
        action="store_true",
        default=True,
        help="Loop Crazyflie trajectory playback.",
    )
    parser.add_argument(
        "--ground-z",
        type=float,
        default=-10.0,
        help="World Z height used as the contact floor for grounded assets and the local R1 Pro. Tiangong scene floor is near -10.",
    )
    parser.add_argument(
        "--add-r1pro",
        action="store_true",
        default=False,
        help="Add the project-local Galaxea R1 Pro USD to the scene.",
    )
    parser.add_argument(
        "--no-add-r1pro",
        action="store_false",
        dest="add_r1pro",
        help="Do not add the project-local Galaxea R1 Pro USD.",
    )
    parser.add_argument(
        "--r1pro-usd",
        type=str,
        default=default_r1pro_usd_path(),
        help="Path to the local r1pro USD asset.",
    )
    parser.add_argument(
        "--r1pro-prim",
        type=str,
        default="/World/r1pro",
        help="Prim path for the local r1pro robot.",
    )
    parser.add_argument("--r1pro-x", type=float, default=1.8, help="Ground X position for the local r1pro robot.")
    parser.add_argument("--r1pro-y", type=float, default=-1.6, help="Ground Y position for the local r1pro robot.")
    parser.add_argument("--r1pro-z", type=float, default=0.0, help="Z offset after bbox-grounding the local r1pro robot.")
    parser.add_argument("--r1pro-yaw", type=float, default=0.0, help="Ground yaw angle in degrees for the local r1pro robot.")
    parser.add_argument(
        "--r1pro-scale",
        type=float,
        default=1.55,
        help="Uniform scale applied to the local r1pro robot so its size is closer to ranger_arm.",
    )
    parser.add_argument(
        "--r1pro-display-only",
        action="store_true",
        default=True,
        help="Keep an existing r1pro visual in the scene but disable its PhysX articulation/rigid bodies.",
    )
    parser.add_argument(
        "--no-r1pro-display-only",
        action="store_false",
        dest="r1pro_display_only",
        help="Leave an existing r1pro PhysX setup untouched. Use only after the r1pro USD is made simulation-safe.",
    )
    parser.add_argument(
        "--r1pro-physics",
        action="store_true",
        help="Keep the adapted r1pro PhysX articulation enabled for later control experiments.",
    )
    parser.add_argument(
        "--r1pro-fixed-base",
        action="store_true",
        help="Pin r1pro to the world and drive its virtual base joints instead of wheel contact.",
    )
    parser.add_argument(
        "--enable-target-reach",
        action="store_true",
        help="Create three target cubes and move the active robot base toward the selected target area.",
    )
    parser.add_argument(
        "--target-points",
        type=str,
        default="",
        help="Semicolon-separated target coordinates, for example '2.75,-2.45,-8.85;3.10,-1.60,-8.75'.",
    )
    parser.add_argument(
        "--target-marker-root",
        type=str,
        default="/World/teleop_targets",
        help="Root prim path used for target reach marker cubes.",
    )
    parser.add_argument(
        "--target-marker-size",
        type=float,
        default=0.20,
        help="Visual cube size for target reach markers.",
    )
    parser.add_argument(
        "--target-height",
        type=float,
        default=1.15,
        help="Default target marker height above --ground-z, roughly matching gripper height.",
    )
    parser.add_argument(
        "--target-spread",
        type=float,
        default=0.85,
        help="Default lateral spread between generated target markers.",
    )
    parser.add_argument(
        "--target-forward-offset",
        type=float,
        default=0.95,
        help="Default X offset from the local R1 Pro start position to generated target markers.",
    )
    parser.add_argument(
        "--target-reach-speed",
        type=float,
        default=0.05,
        help="Maximum end-effector target movement speed in meters per second.",
    )
    parser.add_argument(
        "--target-reach-tolerance",
        type=float,
        default=0.03,
        help="Distance threshold in meters for considering a target reached.",
    )
    parser.add_argument(
        "--target-reach-hold-orientation",
        action="store_true",
        help="Hold current gripper orientation while approaching target points.",
    )
    parser.add_argument(
        "--no-target-base-motion",
        action="store_false",
        dest="target_base_motion",
        help="Disable automatic base motion while target reach is active.",
    )
    parser.add_argument(
        "--target-base-distance",
        type=float,
        default=1.4,
        help="Desired XY base distance to target area.",
    )
    parser.add_argument(
        "--target-base-command",
        type=float,
        default=0.45,
        help="Maximum normalized base command used by automatic target reach.",
    )
    parser.add_argument(
        "--clear-ui-selection",
        action="store_true",
        default=True,
        help="Continuously clear USD/Fabric selection to avoid stale instance-proxy manipulator errors.",
    )
    parser.add_argument(
        "--no-clear-ui-selection",
        action="store_false",
        dest="clear_ui_selection",
        help="Allow normal viewport/stage selection while teleop is running.",
    )
    parser.add_argument(
        "--isaac-asset-root",
        type=str,
        default=os.environ.get("TIANGONG_ISAAC_ASSET_ROOT", ""),
        help="Optional Isaac Sim asset mirror root. Leave empty to use project-local USD references only.",
    )
    parser.add_argument(
        "--no-isaac-asset-root-override",
        action="store_true",
        help="Do not override Isaac Sim /persistent/isaac/asset_root settings.",
    )
    parser.add_argument(
        "--enable-asset-browser",
        action="store_false",
        dest="disable_asset_browser",
        help="Enable the Isaac Sim Assets browser window. Disabled by default to avoid noisy thumbnail scans.",
    )
    parser.set_defaults(disable_asset_browser=True)
    args = parser.parse_args()

    sim_extra_args = []
    isaac_asset_root_display = None
    isaac_asset_root_uri = None
    isaac_asset_browser_root_uri = None
    if not args.no_isaac_asset_root_override and args.isaac_asset_root:
        isaac_asset_root_uri, isaac_asset_browser_root_uri, isaac_asset_root_display = _resolve_isaac_asset_roots(
            args.isaac_asset_root
        )
        if isaac_asset_root_uri:
            sim_extra_args.extend(
                [
                    f"--/persistent/isaac/asset_root/default={isaac_asset_root_uri}",
                    f"--/persistent/isaac/asset_root/cloud={isaac_asset_root_uri}",
                    "--/persistent/isaac/asset_root/timeout=1.0",
                ]
            )
        if isaac_asset_browser_root_uri:
            for index, folder in enumerate(ISAAC_ASSET_BROWSER_FOLDERS):
                sim_extra_args.append(
                    f"--/exts/isaacsim.asset.browser/folders/{index}={isaac_asset_browser_root_uri}/{folder}"
                )
            sim_extra_args.extend(
                [
                    "--/exts/isaacsim.asset.browser/data/hide_file_without_thumbnails=false",
                    "--/exts/isaacsim.asset.browser/data/timeout=1.0",
                    "--/exts/isaacsim.asset.browser/visible_after_startup=false",
                ]
            )
    if args.disable_asset_browser:
        sim_extra_args.extend(["--disable", "isaacsim.asset.browser"])

    sim_app = SimulationApp({"headless": args.headless, "renderer": "RTX", "extra_args": sim_extra_args})

    from tiangong.teleop import (
        BaseMotionCommand,
        DroneAssetController,
        ManipulatorMotionCommand,
        R1ProTeleopController,
        RangerArmTeleopController,
        TeleopDispatcher,
    )

    import carb  # noqa: WPS433
    import numpy as np  # noqa: WPS433
    import omni  # noqa: WPS433
    import omni.kit.app  # noqa: WPS433
    import omni.timeline  # noqa: WPS433
    import omni.usd  # noqa: WPS433
    import omni.ui as ui  # noqa: WPS433
    from omni.kit.viewport.utility import get_active_viewport  # noqa: WPS433
    from isaacsim.core.api import World  # noqa: WPS433
    from isaacsim.core.prims import SingleArticulation  # noqa: WPS433
    from isaacsim.core.utils.xforms import get_world_pose  # noqa: WPS433
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: WPS433

    if getattr(args, "deprecated_move_prim", False):
        carb.log_warn("--move-prim is deprecated and ignored; base motion now requires wheel/steer DOFs.")
    if args.enable_target_reach:
        carb.log_warn("Target reach markers and auto-follow are disabled in this startup flow; ignoring --enable-target-reach.")

    ui_state = {"forward": 0.0, "strafe": 0.0, "lift": 0.0, "yaw": 0.0}
    ui_models = {"forward": None, "strafe": None, "lift": None, "yaw": None}
    if args.use_ui:
        window = ui.Window("Teleop", width=300, height=220)
        with window.frame:
            with ui.VStack(spacing=6):
                ui.Label("Teleop sliders (use if keyboard focus fails)")
                with ui.HStack():
                    ui.Label("Forward", width=80)
                    f_model = ui.FloatDrag(min=-1.0, max=1.0, step=0.05)
                with ui.HStack():
                    ui.Label("Strafe", width=80)
                    s_model = ui.FloatDrag(min=-1.0, max=1.0, step=0.05)
                with ui.HStack():
                    ui.Label("Lift", width=80)
                    z_model = ui.FloatDrag(min=-1.0, max=1.0, step=0.05)
                with ui.HStack():
                    ui.Label("Yaw", width=80)
                    y_model = ui.FloatDrag(min=-1.0, max=1.0, step=0.05)
                ui_models["forward"] = f_model
                ui_models["strafe"] = s_model
                ui_models["lift"] = z_model
                ui_models["yaw"] = y_model

                def _sync_models(*_args):
                    ui_state["forward"] = float(f_model.model.get_value_as_float())
                    ui_state["strafe"] = float(s_model.model.get_value_as_float())
                    ui_state["lift"] = float(z_model.model.get_value_as_float())
                    ui_state["yaw"] = float(y_model.model.get_value_as_float())

                f_model.model.add_value_changed_fn(_sync_models)
                s_model.model.add_value_changed_fn(_sync_models)
                z_model.model.add_value_changed_fn(_sync_models)
                y_model.model.add_value_changed_fn(_sync_models)

                def _reset():
                    f_model.model.set_value(0.0)
                    s_model.model.set_value(0.0)
                    z_model.model.set_value(0.0)
                    y_model.model.set_value(0.0)
                    _sync_models()

                ui.Button("Reset", clicked_fn=_reset)

    settings = carb.settings.get_settings()
    settings.set("/app/window/exitOnEsc", False)
    settings.set("/app/window/quitOnClose", False)
    settings.set("/app/lifecycle/quitWhenIdle", False)
    if isaac_asset_root_display:
        carb.log_info(f"Using local Isaac Sim asset root: {isaac_asset_root_display}")
    if isaac_asset_root_uri:
        settings.set("/persistent/isaac/asset_root/default", isaac_asset_root_uri)
        settings.set("/persistent/isaac/asset_root/cloud", isaac_asset_root_uri)
    if isaac_asset_browser_root_uri:
        for index, folder in enumerate(ISAAC_ASSET_BROWSER_FOLDERS):
            settings.set(f"/exts/isaacsim.asset.browser/folders/{index}", f"{isaac_asset_browser_root_uri}/{folder}")

    carb.log_info(f"Opening USD: {args.usd}")
    omni.usd.get_context().open_stage(args.usd)

    stage = None
    for _ in range(200):
        sim_app.update()
        stage = omni.usd.get_context().get_stage()
        if stage is not None and stage.GetPrimAtPath("/World").IsValid():
            break
        time.sleep(0.01)

    if stage is None:
        carb.log_error("Failed to open USD stage.")
        sim_app.close()
        return
    my_world = World(stage_units_in_meters=1.0, backend="numpy")
    asset_controller = DroneAssetController(
        stage,
        sim_app,
        carb,
        Gf,
        Sdf,
        Usd,
        UsdGeom,
        UsdPhysics,
        ground_z=args.ground_z,
    )
    carb.log_info(f"Using teleop ground Z={args.ground_z:.4f} for grounded assets.")

    def _clear_stage_selection() -> None:
        if not args.clear_ui_selection:
            return
        try:
            selection = omni.usd.get_context().get_selection()
            selection.clear_selected_prim_paths(omni.usd.Selection.SourceType.ALL)
        except TypeError:
            try:
                omni.usd.get_context().get_selection().clear_selected_prim_paths()
            except Exception:
                pass
        except Exception:
            pass

    _clear_stage_selection()

    grounded_asset_poses = []

    def _make_transform_matrix(translation, rotation):
        transform = Gf.Matrix4d(1.0)
        rotation_x = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), float(rotation[0]))
        rotation_y = Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), float(rotation[1]))
        rotation_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), float(rotation[2]))
        transform.SetRotate(rotation_x * rotation_y * rotation_z)
        transform.SetTranslate(translation)
        return transform

    def _apply_grounded_asset_poses() -> None:
        for transform_op, scale_op, translation, rotation, scale in grounded_asset_poses:
            transform_op.Set(_make_transform_matrix(translation, rotation))
            scale_op.Set(Gf.Vec3f(scale, scale, scale))

    def _reset_to_single_transform_op(prim):
        xformable = UsdGeom.Xformable(prim)
        try:
            xformable.ClearXformOpOrder()
        except Exception:
            pass
        transform_op = None
        scale_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:transform:teleop_ground":
                transform_op = op
            if op.GetOpName() == "xformOp:scale:teleop_ground":
                scale_op = op
        if transform_op is None:
            transform_op = xformable.AddTransformOp(
                precision=UsdGeom.XformOp.PrecisionDouble,
                opSuffix="teleop_ground",
            )
        if scale_op is None:
            scale_op = xformable.AddScaleOp(
                precision=UsdGeom.XformOp.PrecisionFloat,
                opSuffix="teleop_ground",
            )
        xformable.SetXformOpOrder([transform_op, scale_op], True)
        return transform_op, scale_op


    def _disable_asset_physics(root_prim) -> None:
        prims = [prim for prim in Usd.PrimRange(root_prim) if prim.IsValid()]
        deactivate_paths = []
        for prim in prims:
            name = prim.GetName().lower()
            type_name = prim.GetTypeName()
            if type_name.startswith("Physics") or "joint" in name:
                deactivate_paths.append(str(prim.GetPath()))
                continue
            try:
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            except Exception:
                pass
            rigid_body = UsdPhysics.RigidBodyAPI(prim)
            if rigid_body:
                rigid_body.CreateRigidBodyEnabledAttr(False)
                rigid_body.CreateKinematicEnabledAttr(False)
            collision = UsdPhysics.CollisionAPI(prim)
            if collision:
                collision.CreateCollisionEnabledAttr(False)
                deactivate_paths.append(str(prim.GetPath()))
                continue
            if name == "collisions":
                deactivate_paths.append(str(prim.GetPath()))
                continue
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeVisible()
            prim.CreateAttribute("physxArticulation:articulationEnabled", Sdf.ValueTypeNames.Bool).Set(False)
        for prim_path in deactivate_paths:
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                prim.SetActive(False)
            prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
            prim.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool).Set(True)

    def _lock_asset_to_ground(prim_path, target_xy, rotation, scale=1.0) -> None:
        asset_prim = stage.GetPrimAtPath(prim_path)
        if not asset_prim.IsValid():
            carb.log_warn(f"Cannot ground missing asset prim: {prim_path}")
            return
        try:
            stage.Load(asset_prim.GetPath(), Usd.LoadWithDescendants)
        except TypeError:
            stage.Load(asset_prim.GetPath())
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Could not explicitly load {prim_path}: {exc}")
        for _ in range(3):
            sim_app.update()
        _disable_asset_physics(asset_prim)
        transform_op, scale_op = _reset_to_single_transform_op(asset_prim)
        transform_op.Set(_make_transform_matrix(Gf.Vec3d(target_xy[0], target_xy[1], 0.0), rotation))
        scale_op.Set(Gf.Vec3f(scale, scale, scale))
        sim_app.update()
        try:
            bbox_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            )
            aligned_range = bbox_cache.ComputeWorldBound(asset_prim).ComputeAlignedRange()
            center = aligned_range.GetMidpoint()
            min_z = aligned_range.GetMin()[2]
            max_z = aligned_range.GetMax()[2]
            current_translation = Gf.Vec3d(target_xy[0], target_xy[1], 0.0)
            translation = current_translation + Gf.Vec3d(target_xy[0] - center[0], target_xy[1] - center[1], 0.05 - min_z)
            carb.log_info(
                f"Grounded {prim_path}: center={center}, min_z={min_z:.4f}, max_z={max_z:.4f}, "
                f"translation={translation}"
            )
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Could not place {prim_path} on the ground from bbox: {exc}")
            translation = Gf.Vec3d(target_xy[0], target_xy[1], 0.05)
        transform_op.Set(_make_transform_matrix(translation, rotation))
        scale_op.Set(Gf.Vec3f(scale, scale, scale))
        grounded_asset_poses.append((transform_op, scale_op, translation, rotation, scale))

    def _place_asset_on_ground(prim_path, target_xy, rotation, keep_locked: bool, scale=1.0) -> None:
        asset_prim = stage.GetPrimAtPath(prim_path)
        if not asset_prim.IsValid():
            carb.log_warn(f"Cannot place missing asset prim: {prim_path}")
            return
        try:
            stage.Load(asset_prim.GetPath(), Usd.LoadWithDescendants)
        except TypeError:
            stage.Load(asset_prim.GetPath())
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Could not explicitly load {prim_path}: {exc}")
        for _ in range(3):
            sim_app.update()
        transform_op, scale_op = _reset_to_single_transform_op(asset_prim)
        transform_op.Set(_make_transform_matrix(Gf.Vec3d(target_xy[0], target_xy[1], 0.0), rotation))
        scale_op.Set(Gf.Vec3f(scale, scale, scale))
        sim_app.update()
        try:
            bbox_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            )
            aligned_range = bbox_cache.ComputeWorldBound(asset_prim).ComputeAlignedRange()
            center = aligned_range.GetMidpoint()
            min_z = aligned_range.GetMin()[2]
            translation = Gf.Vec3d(target_xy[0] - center[0] + target_xy[0], target_xy[1] - center[1] + target_xy[1], 0.05 - min_z)
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Could not place {prim_path} on the ground from bbox: {exc}")
            translation = Gf.Vec3d(target_xy[0], target_xy[1], 0.05)
        transform_op.Set(_make_transform_matrix(translation, rotation))
        scale_op.Set(Gf.Vec3f(scale, scale, scale))
        if keep_locked:
            grounded_asset_poses.append((transform_op, scale_op, translation, rotation, scale))

    def _make_existing_asset_display_only(prim_path) -> None:
        asset_prim = stage.GetPrimAtPath(prim_path)
        if not asset_prim.IsValid():
            return
        try:
            stage.Load(asset_prim.GetPath(), Usd.LoadWithDescendants)
        except TypeError:
            stage.Load(asset_prim.GetPath())
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Could not explicitly load {prim_path}: {exc}")
        for _ in range(3):
            sim_app.update()
        _disable_asset_physics(asset_prim)
        carb.log_warn(f"{prim_path} set to display-only; PhysX articulation/rigid bodies disabled.")

    def _ensure_drone_prims() -> list[tuple[str, float]]:
        drone_specs = [
            ("/World/cf2x", -2.0),
            ("/World/cf2x_01", 2.0),
        ]
        cf2x_path = Path(default_cf2x_usd_path()).expanduser()
        can_inject_missing = cf2x_path.exists()
        if not can_inject_missing:
            carb.log_warn(f"Crazyflie USD not found for missing-drone injection: {cf2x_path}")
        for drone_path, _offset_y in drone_specs:
            if stage.GetPrimAtPath(drone_path).IsValid():
                carb.log_info(f"Using existing drone prim: {drone_path}")
                continue
            if not can_inject_missing:
                carb.log_warn(f"Missing drone prim and no cf2x.usd available to inject it: {drone_path}")
                continue
            drone_prim = UsdGeom.Xform.Define(stage, drone_path).GetPrim()
            drone_prim.GetReferences().AddReference(str(cf2x_path))
            carb.log_info(f"Injected missing drone prim: {drone_path} <- {cf2x_path}")
        sim_app.update()
        return drone_specs

    def _load_drone_trajectory_csv(csv_path: str) -> list[tuple[float, float, float, float, float]]:
        path = Path(csv_path).expanduser()
        if not path.exists():
            carb.log_warn(f"Drone trajectory CSV not found: {path}")
            return []
        samples: list[tuple[float, float, float, float, float]] = []
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                try:
                    samples.append(
                        (
                            float(row["time"]),
                            float(row["x"]),
                            float(row["y"]),
                            float(row["z"]),
                            float(row["yaw_deg"]),
                        )
                    )
                except Exception as exc:  # noqa: BLE001
                    carb.log_warn(f"Skipping malformed drone trajectory row in {path}: {exc}")
        samples.sort(key=lambda sample: sample[0])
        return samples

    def _sample_drone_trajectory(
        samples: list[tuple[float, float, float, float, float]],
        elapsed_seconds: float,
        loop_enabled: bool,
    ) -> tuple[Gf.Vec3d, Gf.Vec3f] | None:
        if not samples:
            return None
        if len(samples) == 1:
            _time, x, y, z, yaw_deg = samples[0]
            return Gf.Vec3d(x, y, z), Gf.Vec3f(0.0, 0.0, yaw_deg)
        total_duration = max(samples[-1][0] - samples[0][0], 1e-6)
        sample_time = elapsed_seconds
        if loop_enabled:
            sample_time = samples[0][0] + ((elapsed_seconds - samples[0][0]) % total_duration)
        elif elapsed_seconds >= samples[-1][0]:
            _time, x, y, z, yaw_deg = samples[-1]
            return Gf.Vec3d(x, y, z), Gf.Vec3f(0.0, 0.0, yaw_deg)
        for left, right in zip(samples, samples[1:]):
            left_t, left_x, left_y, left_z, left_yaw = left
            right_t, right_x, right_y, right_z, right_yaw = right
            if left_t <= sample_time <= right_t:
                alpha = 0.0 if right_t <= left_t else (sample_time - left_t) / (right_t - left_t)
                x = left_x + (right_x - left_x) * alpha
                y = left_y + (right_y - left_y) * alpha
                z = left_z + (right_z - left_z) * alpha
                yaw_deg = left_yaw + (right_yaw - left_yaw) * alpha
                return Gf.Vec3d(x, y, z), Gf.Vec3f(0.0, 0.0, yaw_deg)
        _time, x, y, z, yaw_deg = samples[-1]
        return Gf.Vec3d(x, y, z), Gf.Vec3f(0.0, 0.0, yaw_deg)

    def _yaw_degrees_from_quat_wxyz(quat_wxyz) -> float:
        quat = _quat_wxyz_to_gf(quat_wxyz)
        rotation = Gf.Rotation(quat)
        matrix = Gf.Matrix3d(rotation.GetMatrix())
        forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
        return math.degrees(math.atan2(float(forward[1]), float(forward[0])))

    def _ensure_drone_rotor_visuals(drone_prim) -> list[dict]:
        rotor_keywords = ("rotor", "prop", "motor")
        rotor_entries = []
        used_paths = set()
        for prim in Usd.PrimRange(drone_prim):
            if not prim.IsValid():
                continue
            prim_name = prim.GetName().lower()
            if not any(keyword in prim_name for keyword in rotor_keywords):
                continue
            if not prim.IsA(UsdGeom.Xformable):
                continue
            prim_path = str(prim.GetPath())
            if prim_path in used_paths:
                continue
            xformable = UsdGeom.Xformable(prim)
            rotate_op = None
            for op in xformable.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:rotateZ:teleop_rotor_spin":
                    rotate_op = op
                    break
            if rotate_op is None:
                rotate_op = xformable.AddRotateZOp(
                    precision=UsdGeom.XformOp.PrecisionFloat,
                    opSuffix="teleop_rotor_spin",
                )
            used_paths.add(prim_path)
            rotor_entries.append({"path": prim_path, "rotate_op": rotate_op})
        rotor_entries.sort(key=lambda entry: entry["path"])
        for index, entry in enumerate(rotor_entries):
            entry["direction"] = 1.0 if index % 2 == 0 else -1.0
            entry["angle_deg"] = 0.0
        return rotor_entries[:4]

    def _apply_drone_manual_pose(state: dict) -> None:
        state["transform_op"].Set(
            _make_transform_matrix(
                state["translation"],
                Gf.Vec3f(
                    float(state.get("pitch_deg", 0.0)),
                    float(state.get("roll_deg", 0.0)),
                    float(state["yaw_deg"]),
                ),
            )
        )
        state["scale_op"].Set(Gf.Vec3f(args.drone_scale, args.drone_scale, args.drone_scale))
        for rotor_state in state.get("rotors", []):
            rotor_state["rotate_op"].Set(float(rotor_state["angle_deg"]))

    if args.ground_drones:
        for drone_path, offset_y in _ensure_drone_prims():
            if not stage.GetPrimAtPath(drone_path).IsValid():
                continue
            asset_controller.lock_asset_to_ground(
                drone_path,
                (0.0, offset_y),
                Gf.Vec3f(0.0, 0.0, 80.1330337524414),
                scale=args.drone_scale,
            )

    drone_trajectory_states = []
    if args.enable_drone_trajectories:
        for drone_path, csv_path in (
            ("/World/cf2x", args.cf2x_trajectory_1),
            ("/World/cf2x_01", args.cf2x_trajectory_2),
        ):
            drone_prim = stage.GetPrimAtPath(drone_path)
            if not drone_prim.IsValid():
                carb.log_warn(f"Cannot enable trajectory for missing drone prim: {drone_path}")
                continue
            samples = _load_drone_trajectory_csv(csv_path)
            if not samples:
                continue
            _disable_asset_physics(drone_prim)
            transform_op, scale_op = _reset_to_single_transform_op(drone_prim)
            initial_pose = _sample_drone_trajectory(samples, samples[0][0], args.drone_trajectory_loop)
            if initial_pose is None:
                continue
            translation, rotation = initial_pose
            transform_op.Set(_make_transform_matrix(translation, rotation))
            scale_op.Set(Gf.Vec3f(args.drone_scale, args.drone_scale, args.drone_scale))
            drone_trajectory_states.append(
                {
                    "name": drone_path.rsplit("/", 1)[-1],
                    "path": drone_path,
                    "samples": samples,
                    "transform_op": transform_op,
                    "scale_op": scale_op,
                    "translation": translation,
                    "yaw_deg": float(rotation[2]),
                    "pitch_deg": 0.0,
                    "roll_deg": 0.0,
                    "manual_override": False,
                    "rotors": _ensure_drone_rotor_visuals(drone_prim),
                }
            )
            carb.log_info(f"Drone trajectory ready: {drone_path} <- {csv_path}")
    drone_trajectory_start_time = time.time()

    drone_states_by_path = {state["path"]: state for state in drone_trajectory_states}
    for drone_path in ("/World/cf2x", "/World/cf2x_01"):
        if drone_path in drone_states_by_path:
            continue
        drone_prim = stage.GetPrimAtPath(drone_path)
        if not drone_prim.IsValid():
            continue
        _disable_asset_physics(drone_prim)
        transform_op, scale_op = _reset_to_single_transform_op(drone_prim)
        try:
            translation_np, rotation_quat = _get_world_pose_safe_for_path(drone_path)
            translation = Gf.Vec3d(*np.asarray(translation_np, dtype=np.float64).reshape(-1)[:3])
            yaw_deg = _yaw_degrees_from_quat_wxyz(rotation_quat)
        except Exception:
            translation = Gf.Vec3d(0.0, 0.0, args.ground_z + 1.0)
            yaw_deg = 0.0
        state = {
            "name": drone_path.rsplit("/", 1)[-1],
            "path": drone_path,
            "samples": [],
            "transform_op": transform_op,
            "scale_op": scale_op,
            "translation": translation,
            "yaw_deg": yaw_deg,
            "pitch_deg": 0.0,
            "roll_deg": 0.0,
            "manual_override": True,
            "rotors": _ensure_drone_rotor_visuals(drone_prim),
        }
        _apply_drone_manual_pose(state)
        drone_states_by_path[drone_path] = state

    def _apply_drone_trajectories() -> None:
        if not drone_states_by_path:
            return
        elapsed_seconds = time.time() - drone_trajectory_start_time
        for state in drone_states_by_path.values():
            if state.get("manual_override", False) or not state.get("samples"):
                rotor_speed_deg = float(state.get("rotor_speed_deg_per_sec", 540.0))
                for rotor_state in state.get("rotors", []):
                    rotor_state["angle_deg"] = (
                        float(rotor_state.get("angle_deg", 0.0))
                        + rotor_speed_deg * float(rotor_state["direction"]) * args.dt
                    ) % 360.0
                _apply_drone_manual_pose(state)
                continue
            pose = _sample_drone_trajectory(state["samples"], elapsed_seconds, args.drone_trajectory_loop)
            if pose is None:
                continue
            translation, rotation = pose
            state["translation"] = translation
            state["yaw_deg"] = float(rotation[2])
            state["pitch_deg"] = 0.0
            state["roll_deg"] = 0.0
            state["rotor_speed_deg_per_sec"] = 720.0
            for rotor_state in state.get("rotors", []):
                rotor_state["angle_deg"] = (
                    float(rotor_state.get("angle_deg", 0.0))
                    + float(state["rotor_speed_deg_per_sec"]) * float(rotor_state["direction"]) * args.dt
                ) % 360.0
            _apply_drone_manual_pose(state)

    if args.r1pro_display_only and args.r1pro_prim != "/World/r1pro":
        asset_controller.make_display_only("/World/r1pro")

    r1pro_scene_prim_path = args.r1pro_prim

    def _resolve_r1pro_articulation_prim_path(scene_prim_path: str) -> str:
        candidate_paths = [f"{scene_prim_path}/base_link", f"{scene_prim_path}/base_footprint_x", scene_prim_path]
        for path in candidate_paths:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                return path
        root_prim = stage.GetPrimAtPath(scene_prim_path)
        if root_prim.IsValid():
            for prim in Usd.PrimRange(root_prim):
                if prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    return str(prim.GetPath())
        return f"{scene_prim_path}/base_link"

    r1pro_articulation_prim_path = _resolve_r1pro_articulation_prim_path(r1pro_scene_prim_path)
    robot_camera_aliases = {"r1pro": {}, "ranger_arm": {}}
    robot_camera_rigs = {}
    active_camera_control = {"robot_name": None, "alias": None}
    active_control_context = {"kind": "robot", "path": None}

    def _get_world_pose_safe_for_path(prim_path: str):
        try:
            return get_world_pose(prim_path, fabric=True)
        except Exception:
            return get_world_pose(prim_path)

    def _quat_wxyz_to_gf(quat_wxyz) -> Gf.Quatf:
        quat_array = np.asarray(quat_wxyz, dtype=np.float32).reshape(-1)[:4]
        return Gf.Quatf(float(quat_array[0]), Gf.Vec3f(float(quat_array[1]), float(quat_array[2]), float(quat_array[3])))

    def _deactivate_existing_cameras(scene_root_path: str, preserve_paths: set[str] | None = None):
        root_prim = stage.GetPrimAtPath(scene_root_path)
        if not root_prim.IsValid():
            return
        preserve_paths = preserve_paths or set()
        target_paths = []
        seen_paths = set()
        for prim in Usd.PrimRange(root_prim):
            if not prim.IsValid() or prim.GetTypeName() != "Camera":
                continue
            prim_path = str(prim.GetPath())
            if "/teleop_camera_rigs/" in prim_path:
                continue
            parent_prim = prim.GetParent()
            target_prim = parent_prim if parent_prim.IsValid() and str(parent_prim.GetPath()).startswith(scene_root_path) else prim
            target_path = str(target_prim.GetPath())
            if prim_path in preserve_paths or target_path in preserve_paths:
                continue
            if target_path in seen_paths:
                continue
            seen_paths.add(target_path)
            target_paths.append(target_path)
        for target_path in target_paths:
            target_prim = stage.GetPrimAtPath(target_path)
            if target_prim.IsValid():
                target_prim.SetActive(False)

    def _clear_existing_robot_cameras(robot_name: str, scene_root_path: str, preserve_paths: set[str] | None = None):
        _deactivate_existing_cameras(scene_root_path, preserve_paths=preserve_paths)
        rig_root_path = f"/World/teleop_camera_rigs/{robot_name}"
        if stage.GetPrimAtPath(rig_root_path).IsValid():
            stage.RemovePrim(rig_root_path)
        robot_camera_aliases[robot_name] = {}
        for key in [rig_key for rig_key in robot_camera_rigs if rig_key[0] == robot_name]:
            robot_camera_rigs.pop(key, None)

    def _build_r1pro_usd_camera_aliases(scene_prim_path: str) -> dict[str, str]:
        camera_candidates = {
            "head_top": [
                f"{scene_prim_path}/Root/r1_pro_with_gripper/zed_link/teleop_head_top/Camera",
                f"{scene_prim_path}/r1_pro_with_gripper/zed_link/teleop_head_top/Camera",
            ],
            "left_gripper": [
                f"{scene_prim_path}/Root/r1_pro_with_gripper/left_realsense_link/teleop_left_gripper/Camera",
                f"{scene_prim_path}/r1_pro_with_gripper/left_realsense_link/teleop_left_gripper/Camera",
            ],
            "right_gripper": [
                f"{scene_prim_path}/Root/r1_pro_with_gripper/right_realsense_link/teleop_right_gripper/Camera",
                f"{scene_prim_path}/r1_pro_with_gripper/right_realsense_link/teleop_right_gripper/Camera",
            ],
        }
        aliases = {}
        for alias, candidates in camera_candidates.items():
            for camera_path in candidates:
                if stage.GetPrimAtPath(camera_path).IsValid():
                    aliases[alias] = camera_path
                    break
        return aliases

    def _upsert_follow_camera(
        robot_name: str,
        scene_root_path: str,
        alias: str,
        link_path: str,
        translate: tuple[float, float, float],
        rotate_xyz: tuple[float, float, float],
        focal_length: float,
    ) -> str | None:
        link_prim = stage.GetPrimAtPath(link_path)
        if not link_prim.IsValid():
            return None
        rig_root_path = f"/World/teleop_camera_rigs/{robot_name}"
        UsdGeom.Xform.Define(stage, "/World/teleop_camera_rigs")
        UsdGeom.Xform.Define(stage, rig_root_path)
        camera_xform_path = f"{rig_root_path}/{alias}"
        camera_prim_path = f"{camera_xform_path}/Camera"
        camera_xform = UsdGeom.Xform.Define(stage, camera_xform_path)
        xformable = UsdGeom.Xformable(camera_xform.GetPrim())
        xformable.ClearXformOpOrder()
        translate_op = xformable.AddTranslateOp()
        rotate_op = xformable.AddOrientOp()
        translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
        rotate_op.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        camera = UsdGeom.Camera.Define(stage, camera_prim_path)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 500.0))
        camera.GetFocalLengthAttr().Set(focal_length)
        camera.GetHorizontalApertureAttr().Set(20.955)
        camera.GetVerticalApertureAttr().Set(15.2908)
        local_rotation = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), rotate_xyz[0])
        local_rotation *= Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), rotate_xyz[1])
        local_rotation *= Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rotate_xyz[2])
        robot_camera_rigs[(robot_name, alias)] = {
            "camera_path": camera_prim_path,
            "body_path": link_path,
            "local_offset": Gf.Vec3d(*translate),
            "base_local_rotation": Gf.Quatf(
                float(local_rotation.GetQuat().GetReal()),
                Gf.Vec3f(
                    float(local_rotation.GetQuat().GetImaginary()[0]),
                    float(local_rotation.GetQuat().GetImaginary()[1]),
                    float(local_rotation.GetQuat().GetImaginary()[2]),
                ),
            ),
            "adjust_yaw_deg": 0.0,
            "adjust_pitch_deg": 0.0,
            "translate_op": translate_op,
            "rotate_op": rotate_op,
        }
        return camera_prim_path

    def _build_r1pro_camera_aliases(scene_prim_path: str) -> dict[str, str]:
        link_candidates = {
            "head_top": [
                ("/Root/r1_pro_with_gripper/zed_link", (-1.20, 0.0, 0.45), (0.0, 0.0, 180.0), 18.0),
                ("/r1_pro_with_gripper/zed_link", (-1.20, 0.0, 0.45), (0.0, 0.0, 180.0), 18.0),
            ],
            "left_gripper": [
                ("/Root/r1_pro_with_gripper/left_realsense_link", (-0.12, 0.0, 0.02), (0.0, 0.0, 180.0), 16.0),
                ("/Root/r1_pro_with_gripper/left_gripper_link", (-0.12, 0.0, 0.02), (0.0, 0.0, 180.0), 16.0),
                ("/r1_pro_with_gripper/left_realsense_link", (-0.12, 0.0, 0.02), (0.0, 0.0, 180.0), 16.0),
                ("/r1_pro_with_gripper/left_gripper_link", (-0.12, 0.0, 0.02), (0.0, 0.0, 180.0), 16.0),
            ],
            "right_gripper": [
                ("/Root/r1_pro_with_gripper/right_realsense_link", (-0.12, 0.0, 0.02), (0.0, 0.0, 180.0), 16.0),
                ("/Root/r1_pro_with_gripper/right_gripper_link", (-0.12, 0.0, 0.02), (0.0, 0.0, 180.0), 16.0),
                ("/r1_pro_with_gripper/right_realsense_link", (-0.12, 0.0, 0.02), (0.0, 0.0, 180.0), 16.0),
                ("/r1_pro_with_gripper/right_gripper_link", (-0.12, 0.0, 0.02), (0.0, 0.0, 180.0), 16.0),
            ],
        }
        aliases = {}
        for alias, candidates in link_candidates.items():
            for suffix, translate, rotate_xyz, focal_length in candidates:
                camera_path = _upsert_follow_camera(
                    "r1pro",
                    scene_prim_path,
                    alias,
                    f"{scene_prim_path}{suffix}",
                    translate,
                    rotate_xyz,
                    focal_length,
                )
                if camera_path is not None and stage.GetPrimAtPath(camera_path).IsValid():
                    aliases[alias] = camera_path
                    break
        return aliases

    def _build_ranger_camera_aliases(scene_prim_path: str) -> dict[str, str]:
        head_candidates = [
            ("/base_footprint", (0.18, 0.0, 1.45), (6.0, 0.0, 0.0), 18.0),
            ("/base_link", (0.18, 0.0, 1.45), (6.0, 0.0, 0.0), 18.0),
            ("/chassis_link", (0.18, 0.0, 1.45), (6.0, 0.0, 0.0), 18.0),
            ("/body", (0.18, 0.0, 1.45), (6.0, 0.0, 0.0), 18.0),
        ]
        link_candidates = {
            "head_top": head_candidates,
        }
        aliases = {}
        for alias, candidates in link_candidates.items():
            for suffix, translate, rotate_xyz, focal_length in candidates:
                camera_path = _upsert_follow_camera(
                    "ranger_arm",
                    scene_prim_path,
                    alias,
                    f"{scene_prim_path}{suffix}",
                    translate,
                    rotate_xyz,
                    focal_length,
                )
                if camera_path is not None and stage.GetPrimAtPath(camera_path).IsValid():
                    aliases[alias] = camera_path
                    break
        return aliases

    def _build_drone_camera_aliases(drone_name: str, scene_prim_path: str) -> dict[str, str]:
        camera_path = _upsert_follow_camera(
            drone_name,
            scene_prim_path,
            "chase",
            scene_prim_path,
            (-1.2, 0.0, 0.45),
            (12.0, 0.0, 0.0),
            18.0,
        )
        if camera_path is not None and stage.GetPrimAtPath(camera_path).IsValid():
            return {"chase": camera_path}
        return {}

    def _update_robot_camera_rigs():
        for rig in robot_camera_rigs.values():
            try:
                body_pos, body_quat = _get_world_pose_safe_for_path(rig["body_path"])
            except Exception:
                continue
            body_pos = np.asarray(body_pos, dtype=np.float64).reshape(-1)[:3]
            body_quat_gf = _quat_wxyz_to_gf(body_quat)
            body_rotation = Gf.Rotation(body_quat_gf)
            world_position = body_rotation.TransformDir(rig["local_offset"]) + Gf.Vec3d(*body_pos)
            adjustment = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), float(rig["adjust_yaw_deg"]))
            adjustment *= Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), float(rig["adjust_pitch_deg"]))
            adjustment_quat = Gf.Quatf(
                float(adjustment.GetQuat().GetReal()),
                Gf.Vec3f(
                    float(adjustment.GetQuat().GetImaginary()[0]),
                    float(adjustment.GetQuat().GetImaginary()[1]),
                    float(adjustment.GetQuat().GetImaginary()[2]),
                ),
            )
            world_rotation = body_quat_gf * rig["base_local_rotation"] * adjustment_quat
            rig["translate_op"].Set(world_position)
            rig["rotate_op"].Set(world_rotation)

    def _validate_robot_camera_rigs(robot_name: str):
        camera_aliases = robot_camera_aliases.get(robot_name, {})
        if not camera_aliases:
            return
        for alias, camera_path in camera_aliases.items():
            rig = robot_camera_rigs.get((robot_name, alias))
            if rig is None:
                continue
            try:
                body_pos, body_quat = _get_world_pose_safe_for_path(rig["body_path"])
                camera_pos, _ = _get_world_pose_safe_for_path(camera_path)
            except Exception as exc:  # noqa: BLE001
                carb.log_warn(f"Failed to validate {robot_name} camera {alias}: {exc}")
                continue
            body_pos = np.asarray(body_pos, dtype=np.float64).reshape(-1)[:3]
            camera_pos = np.asarray(camera_pos, dtype=np.float64).reshape(-1)[:3]
            body_quat_gf = _quat_wxyz_to_gf(body_quat)
            expected_position = Gf.Rotation(body_quat_gf).TransformDir(rig["local_offset"]) + Gf.Vec3d(*body_pos)
            expected_position_np = np.array(
                [float(expected_position[0]), float(expected_position[1]), float(expected_position[2])],
                dtype=np.float64,
            )
            position_error = float(np.linalg.norm(camera_pos - expected_position_np))
            carb.log_info(
                f"{robot_name} camera validated: {alias}, body={rig['body_path']}, camera={camera_path}, "
                f"position_error={position_error:.6f}"
            )

    def _switch_robot_camera(camera_alias: str) -> bool:
        active_robot = dispatcher.active_name if dispatcher is not None and dispatcher.active_name is not None else "r1pro"
        camera_path = robot_camera_aliases.get(active_robot, {}).get(camera_alias)
        if not camera_path:
            carb.log_warn(f"{active_robot} camera '{camera_alias}' is unavailable on current stage.")
            return False
        viewport = get_active_viewport()
        if viewport is None:
            carb.log_warn("No active viewport found; cannot switch camera.")
            return False
        viewport.camera_path = camera_path
        active_camera_control["robot_name"] = active_robot
        active_camera_control["alias"] = camera_alias
        active_camera = viewport.camera_path.pathString if hasattr(viewport.camera_path, "pathString") else str(viewport.camera_path)
        carb.log_info(f"Viewport switched to {active_robot} camera: {camera_alias} -> {active_camera}")
        return True

    def _switch_named_camera(robot_name: str, camera_alias: str = "chase") -> bool:
        camera_path = robot_camera_aliases.get(robot_name, {}).get(camera_alias)
        if not camera_path:
            carb.log_warn(f"{robot_name} camera '{camera_alias}' is unavailable on current stage.")
            return False
        viewport = get_active_viewport()
        if viewport is None:
            carb.log_warn("No active viewport found; cannot switch camera.")
            return False
        viewport.camera_path = camera_path
        active_camera_control["robot_name"] = robot_name
        active_camera_control["alias"] = camera_alias
        active_camera = viewport.camera_path.pathString if hasattr(viewport.camera_path, "pathString") else str(viewport.camera_path)
        carb.log_info(f"Viewport switched to {robot_name} camera: {camera_alias} -> {active_camera}")
        return True

    def _adjust_active_camera(delta_yaw_deg: float, delta_pitch_deg: float) -> bool:
        robot_name = active_camera_control.get("robot_name")
        alias = active_camera_control.get("alias")
        if not robot_name or not alias:
            return False
        rig = robot_camera_rigs.get((robot_name, alias))
        if rig is None:
            return False
        rig["adjust_yaw_deg"] = float(rig["adjust_yaw_deg"]) + float(delta_yaw_deg)
        rig["adjust_pitch_deg"] = max(-85.0, min(85.0, float(rig["adjust_pitch_deg"]) + float(delta_pitch_deg)))
        return True

    if args.add_r1pro:
        r1pro_path = Path(args.r1pro_usd).expanduser()
        if r1pro_path.exists():
            r1pro_prim = stage.GetPrimAtPath(args.r1pro_prim)
            if r1pro_prim.IsValid() and args.r1pro_physics:
                existing_articulation_path = _resolve_r1pro_articulation_prim_path(args.r1pro_prim)
                if stage.GetPrimAtPath(existing_articulation_path).IsValid() and stage.GetPrimAtPath(
                    existing_articulation_path
                ).HasAPI(UsdPhysics.ArticulationRootAPI):
                    carb.log_warn(f"Using existing r1pro prim for physics test: {args.r1pro_prim}")
                    r1pro_scene_prim_path = args.r1pro_prim
                else:
                    r1pro_scene_prim_path = f"{args.r1pro_prim}_teleop"
                    carb.log_warn(
                        f"Existing prim {args.r1pro_prim} is display-only/non-articulated; "
                        f"loading controllable r1pro at {r1pro_scene_prim_path}."
                    )
                    _hide_prim(stage, UsdGeom, args.r1pro_prim)
            if not r1pro_prim.IsValid() or r1pro_scene_prim_path != args.r1pro_prim:
                r1pro_prim = UsdGeom.Xform.Define(stage, r1pro_scene_prim_path).GetPrim()
                r1pro_prim.GetReferences().AddReference(str(r1pro_path))
                sim_app.update()
            r1pro_articulation_prim_path = _resolve_r1pro_articulation_prim_path(r1pro_scene_prim_path)
            if args.r1pro_physics:
                asset_controller.place_asset_on_ground(
                    r1pro_scene_prim_path,
                    (args.r1pro_x, args.r1pro_y),
                    Gf.Vec3f(0.0, 0.0, args.r1pro_yaw),
                    keep_locked=False,
                    z_offset=args.r1pro_z,
                    scale=args.r1pro_scale,
                )
                _prepare_r1pro_articulation_for_teleop(
                    stage,
                    Gf,
                    Usd,
                    UsdGeom,
                    UsdPhysics,
                    PhysxSchema,
                    Sdf,
                    r1pro_scene_prim_path,
                    carb,
                    fixed_base=args.r1pro_fixed_base,
                )
                r1pro_articulation_prim_path = _resolve_r1pro_articulation_prim_path(r1pro_scene_prim_path)
            else:
                asset_controller.lock_asset_to_ground(
                    r1pro_scene_prim_path,
                    (args.r1pro_x, args.r1pro_y),
                    Gf.Vec3f(0.0, 0.0, args.r1pro_yaw),
                    scale=args.r1pro_scale,
                )
        else:
            carb.log_warn(f"r1pro USD not found: {r1pro_path}")
    elif args.r1pro_display_only and not args.r1pro_physics:
        asset_controller.make_display_only(args.r1pro_prim)

    r1pro_usd_camera_aliases = _build_r1pro_usd_camera_aliases(r1pro_scene_prim_path)
    _clear_existing_robot_cameras(
        "r1pro",
        r1pro_scene_prim_path,
        preserve_paths=set(r1pro_usd_camera_aliases.values()),
    )
    robot_camera_aliases["r1pro"] = r1pro_usd_camera_aliases or _build_r1pro_camera_aliases(r1pro_scene_prim_path)
    asset_controller.apply_locked_poses()
    _clear_stage_selection()

    sim_app.update()
    prim_path = args.prim
    wrapper_path = "/World/ranger_arm_teleop"
    prim = stage.GetPrimAtPath(prim_path)
    wrapped_prim_path = f"{wrapper_path}/ranger_arm"
    if not prim.IsValid() and stage.GetPrimAtPath(wrapped_prim_path).IsValid():
        carb.log_info(f"Prim {prim_path} not found; using existing wrapped prim {wrapped_prim_path}.")
        prim_path = wrapped_prim_path
        prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        carb.log_error(f"Prim not found: {args.prim}")
        sim_app.close()
        return

    if args.wrap_prim:
        wrapper = stage.GetPrimAtPath(wrapper_path)
        if not wrapper.IsValid():
            wrapper = UsdGeom.Xform.Define(stage, wrapper_path).GetPrim()
        if stage.GetPrimAtPath(wrapped_prim_path).IsValid():
            prim_path = wrapped_prim_path
            prim = stage.GetPrimAtPath(prim_path)
        elif prim_path == "/World/ranger_arm":
            try:
                namespace_editor = Usd.NamespaceEditor(stage)
                if hasattr(namespace_editor, "MovePrim") and namespace_editor.MovePrim(prim_path, wrapped_prim_path):
                    namespace_editor.ApplyEdits()
                    prim_path = wrapped_prim_path
                    prim = stage.GetPrimAtPath(prim_path)
                else:
                    carb.log_warn("NamespaceEditor cannot move prim in this Isaac version. Keeping original prim path.")
            except Exception as exc:  # noqa: BLE001
                carb.log_warn(f"Failed to wrap prim: {exc}")

    def _detect_joints(root_prim):
        wheels = []
        steering = []
        for prim in Usd.PrimRange(root_prim):
            name = prim.GetName().lower()
            type_name = prim.GetTypeName()
            is_joint = "Joint" in type_name or "Physics" in type_name
            if not is_joint:
                continue
            is_steering = "steer" in name or "steering" in name
            is_wheel = "wheel" in name and not is_steering
            if is_steering:
                steering.append(str(prim.GetPath()))
            elif is_wheel:
                wheels.append(str(prim.GetPath()))
        return wheels, steering

    wheel_joint_paths = []
    steering_joint_paths = []
    wheel_joints_explicit = bool(args.wheel_joints)
    steer_joints_explicit = bool(args.steer_joints)
    if args.drive_wheels:
        if wheel_joints_explicit:
            wheel_joint_paths = [p.strip() for p in args.wheel_joints.split(",") if p.strip()]
        else:
            # Try to auto-detect wheel joints under the robot prim.
            root = stage.GetPrimAtPath("/World/ranger_arm_teleop/ranger_arm")
            if not root.IsValid():
                root = stage.GetPrimAtPath("/World/ranger_arm")
            if root.IsValid():
                wheel_joint_paths, steering_joint_paths = _detect_joints(root)
        if steer_joints_explicit:
            steering_joint_paths = [p.strip() for p in args.steer_joints.split(",") if p.strip()]
        if wheel_joints_explicit and not wheel_joint_paths:
            carb.log_warn("No Ranger Arm wheel joints found. Base motion requires wheel joints.")
    robot_articulation = None
    r1pro_articulation = None
    wheel_indices = np.array([], dtype=np.int32)
    steering_indices = np.array([], dtype=np.int32)
    wheel_control_paths = []
    steering_control_paths = []
    ranger_controller = None
    r1pro_controller = None
    dispatcher = None
    timeline = omni.timeline.get_timeline_interface()
    animation_start_seconds = 0.0
    animation_end_seconds = max(args.stage_animation_duration, 0.0)
    animation_current_seconds = animation_start_seconds
    last_animation_wall_time = time.time()
    animation_time_codes_per_second = stage.GetTimeCodesPerSecond() or 24.0
    stage_animation_finished = False
    stage_animation_ops = []
    stage_animation_samples = []
    arm_ik_enabled = False
    active_ik_mode = args.ik_arm

    def _cache_stage_animation_op(prim_path, op, samples) -> None:
        # Block weaker payload time samples so the manual player is the only writer.
        try:
            op.GetAttr().Block()
            _clear_stage_selection()
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Could not block original animation on {prim_path}.{op.GetOpName()}: {exc}")
        stage_animation_ops.append((prim_path, op.GetOpName(), op.GetOpType()))
        stage_animation_samples.append(samples)

    def _get_stage_animation_op(prim_path, op_name, op_type):
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return None
        if prim.IsInstanceProxy():
            station = stage.GetPrimAtPath("/World/Tiangong_Space_Station")
            if station.IsValid() and station.IsInstanceable():
                station.SetInstanceable(False)
                sim_app.update()
                _clear_stage_selection()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid() or prim.IsInstanceProxy():
                return None
        xformable_prim = UsdGeom.Xformable(prim)
        if not xformable_prim:
            return None
        op = next(
            (xform_op for xform_op in xformable_prim.GetOrderedXformOps() if xform_op.GetOpName() == op_name),
            None,
        )
        if op is None:
            op = next(
                (xform_op for xform_op in xformable_prim.GetOrderedXformOps() if xform_op.GetOpType() == op_type),
                None,
            )
        return op

    def _sample_stage_animation_value(op_type, samples, time_code_value):
        if op_type == UsdGeom.XformOp.TypeTranslate and len(samples) >= 2:
            start_time, start_value = samples[0]
            end_time, end_value = samples[-1]
            duration = max(end_time - start_time, 1e-6)
            progress = (time_code_value - start_time) / duration
            progress = max(0.0, min(args.station_assembly_progress_scale, progress))
            return start_value + (end_value - start_value) * progress
        sample_time = min(max(time_code_value, samples[0][0]), samples[-1][0])
        _sample_time, value = min(samples, key=lambda item: abs(item[0] - sample_time))
        return value

    def _apply_stage_animation_pose(time_code_value) -> None:
        for (prim_path, op_name, op_type), samples in zip(stage_animation_ops, stage_animation_samples):
            op = _get_stage_animation_op(prim_path, op_name, op_type)
            if op is None:
                continue
            value = _sample_stage_animation_value(op_type, samples, time_code_value)
            if value is not None:
                op.Set(value)

    def _collect_stage_animation_ops() -> None:
        nonlocal animation_time_codes_per_second
        if not args.manual_stage_animation:
            return
        stage_animation_ops.clear()
        stage_animation_samples.clear()
        station = stage.GetPrimAtPath("/World/Tiangong_Space_Station")
        if not station.IsValid():
            carb.log_warn("Manual stage animation skipped: /World/Tiangong_Space_Station not found.")
            return
        if station.IsInstanceable():
            station.SetInstanceable(False)
            sim_app.update()
            _clear_stage_selection()

        try:
            stage.Load(station.GetPath(), Usd.LoadWithDescendants)
        except TypeError:
            stage.Load(station.GetPath())
        for _ in range(3):
            sim_app.update()

        for prim in Usd.PrimRange(station):
            xformable_prim = UsdGeom.Xformable(prim)
            if not xformable_prim:
                continue
            for op in xformable_prim.GetOrderedXformOps():
                times = op.GetTimeSamples()
                if times:
                    samples = [(sample_time, op.Get(Usd.TimeCode(sample_time))) for sample_time in times]
                    _cache_stage_animation_op(str(prim.GetPath()), op, samples)

        if not stage_animation_ops:
            source_path = Path(args.station_animation_usd).expanduser()
            if source_path.exists():
                source_stage = Usd.Stage.Open(str(source_path))
                if source_stage is None:
                    carb.log_warn(f"Could not open station animation USD: {source_path}")
                else:
                    animation_time_codes_per_second = source_stage.GetTimeCodesPerSecond() or animation_time_codes_per_second
                    source_root = "/World"
                    target_root = str(station.GetPath())
                    for source_prim in source_stage.Traverse():
                        source_xformable = UsdGeom.Xformable(source_prim)
                        if not source_xformable:
                            continue
                        source_prim_path = str(source_prim.GetPath())
                        if not source_prim_path.startswith(source_root):
                            continue
                        target_path = target_root + source_prim_path[len(source_root) :]
                        target_prim = stage.GetPrimAtPath(target_path)
                        if not target_prim.IsValid():
                            continue
                        target_xformable = UsdGeom.Xformable(target_prim)
                        if not target_xformable:
                            continue
                        target_ops = target_xformable.GetOrderedXformOps()
                        for source_op in source_xformable.GetOrderedXformOps():
                            times = source_op.GetTimeSamples()
                            if not times:
                                continue
                            target_op = next(
                                (op for op in target_ops if op.GetOpName() == source_op.GetOpName()),
                                None,
                            )
                            if target_op is None:
                                target_op = next(
                                    (op for op in target_ops if op.GetOpType() == source_op.GetOpType()),
                                    None,
                                )
                            if target_op is None:
                                continue
                            samples = [
                                (sample_time, source_op.Get(Usd.TimeCode(sample_time)))
                                for sample_time in times
                            ]
                            _cache_stage_animation_op(target_path, target_op, samples)
            else:
                carb.log_warn(f"Station animation USD not found: {source_path}")

        message = (
            f"Manual stage animation ops: {len(stage_animation_ops)} "
            f"(station loaded={station.IsLoaded()}, source={args.station_animation_usd})"
        )
        carb.log_info(message)
        print(message, flush=True)
        if stage_animation_ops:
            _apply_stage_animation_pose(animation_start_seconds)

    def _play_stage_animation() -> None:
        nonlocal animation_start_seconds, animation_end_seconds, animation_current_seconds, last_animation_wall_time
        nonlocal stage_animation_finished
        if not args.play_stage_animation:
            return
        start_time_code = stage.GetStartTimeCode()
        end_time_code = stage.GetEndTimeCode()
        time_codes_per_second = animation_time_codes_per_second
        if args.stage_animation_duration > 0.0:
            animation_start_seconds = 0.0
            animation_end_seconds = args.stage_animation_duration
        elif end_time_code > start_time_code:
            animation_start_seconds = start_time_code
            animation_end_seconds = end_time_code
        if animation_end_seconds > animation_start_seconds:
            animation_current_seconds = animation_start_seconds
            stage_animation_finished = False
            if stage_animation_ops:
                _apply_stage_animation_pose(animation_current_seconds)
            carb.log_info(
                f"Stage animation timeline: {animation_start_seconds:.3f} -> "
                f"{animation_end_seconds:.3f} timeCodes, tps={time_codes_per_second:.3f}, "
                f"rate={args.stage_animation_rate:.3f} timeCodes/s"
            )
        timeline.play()
        timeline.commit()
        last_animation_wall_time = time.time()

    def _advance_stage_animation() -> None:
        nonlocal animation_current_seconds, last_animation_wall_time, stage_animation_finished
        if not args.play_stage_animation or animation_end_seconds <= animation_start_seconds:
            return
        if stage_animation_finished:
            return
        now = time.time()
        elapsed = now - last_animation_wall_time
        last_animation_wall_time = now
        animation_current_seconds += elapsed * args.stage_animation_rate
        duration = animation_end_seconds - animation_start_seconds
        if animation_current_seconds > animation_end_seconds and args.loop_stage_animation:
            animation_current_seconds = animation_start_seconds + (animation_current_seconds - animation_end_seconds) % duration
        elif animation_current_seconds > animation_end_seconds:
            animation_current_seconds = animation_end_seconds
            stage_animation_finished = True
        _apply_stage_animation_pose(animation_current_seconds)

    _collect_stage_animation_ops()
    _play_stage_animation()
    robot_root_path = "/World/ranger_arm_teleop/ranger_arm"
    if args.drive_wheels or args.enable_arm_ik:
        if not stage.GetPrimAtPath(robot_root_path).IsValid():
            robot_root_path = "/World/ranger_arm"
        try:
            if timeline.is_stopped():
                timeline.play()
            for _ in range(10):
                sim_app.update()
            robot_articulation = SingleArticulation(
                robot_root_path,
                name="ranger_arm_teleop_articulation",
                reset_xform_properties=False,
            )
            my_world.scene.add(robot_articulation)
            my_world.reset()
            for _ in range(3):
                asset_controller.apply_locked_poses()
                my_world.step(render=True)
            robot_articulation.initialize(my_world.physics_sim_view)
            _play_stage_animation()

            def _joint_name(path: str) -> str:
                return path.rsplit("/", 1)[-1]

            def _dof_is_steering(name: str) -> bool:
                lower_name = name.lower()
                return "steer" in lower_name or "steering" in lower_name

            def _dof_is_wheel(name: str) -> bool:
                lower_name = name.lower()
                return "wheel" in lower_name and not _dof_is_steering(name)

            def _path_for_dof(name: str) -> str:
                return f"{robot_root_path}/{name}"

            def _joint_indices(paths):
                indices = []
                control_paths = []
                for path in paths:
                    name = _joint_name(path)
                    try:
                        indices.append(robot_articulation.get_dof_index(name))
                        control_paths.append(path)
                    except Exception as exc:  # noqa: BLE001
                        carb.log_warn(f"Could not find articulation DOF for joint {name}: {exc}")
                return control_paths, np.array(indices, dtype=np.int32)

            if args.drive_wheels:
                if not wheel_joints_explicit:
                    wheel_joint_paths = [_path_for_dof(name) for name in robot_articulation.dof_names if _dof_is_wheel(name)]
                if not steer_joints_explicit:
                    steering_joint_paths = [
                        _path_for_dof(name) for name in robot_articulation.dof_names if _dof_is_steering(name)
                    ]

                wheel_control_paths, wheel_indices = _joint_indices(wheel_joint_paths)
                steering_control_paths, steering_indices = _joint_indices(steering_joint_paths)
            carb.log_info(f"Articulation DOFs: {list(robot_articulation.dof_names)}")
            carb.log_info(f"Wheel control paths: {wheel_control_paths}")
            carb.log_info(f"Steering control paths: {steering_control_paths}")
            carb.log_info(f"Wheel DOF indices: {wheel_indices.tolist()}")
            carb.log_info(f"Steering DOF indices: {steering_indices.tolist()}")
            if args.print_joints:
                sim_app.close()
                return
            if args.drive_wheels and wheel_indices.size == 0:
                carb.log_warn("No Ranger Arm wheel DOF indices found in articulation. Base motion requires wheel DOFs.")
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Failed to initialize ranger_arm articulation control: {exc}")
            arm_ik_enabled = False
    if stage.GetPrimAtPath(robot_root_path).IsValid():
        _clear_existing_robot_cameras("ranger_arm", robot_root_path)
        robot_camera_aliases["ranger_arm"] = _build_ranger_camera_aliases(robot_root_path)
    for drone_name in ("cf2x", "cf2x_01"):
        drone_scene_path = f"/World/{drone_name}"
        if stage.GetPrimAtPath(drone_scene_path).IsValid():
            _clear_existing_robot_cameras(drone_name, drone_scene_path)
            robot_camera_aliases[drone_name] = _build_drone_camera_aliases(drone_name, drone_scene_path)
    _update_robot_camera_rigs()
    _validate_robot_camera_rigs("r1pro")
    _validate_robot_camera_rigs("ranger_arm")
    _validate_robot_camera_rigs("cf2x")
    _validate_robot_camera_rigs("cf2x_01")

    if args.add_r1pro and args.r1pro_physics and stage.GetPrimAtPath(r1pro_articulation_prim_path).IsValid():
        try:
            if timeline.is_stopped():
                timeline.play()
            r1pro_articulation = SingleArticulation(
                r1pro_articulation_prim_path,
                name="r1pro_teleop_articulation",
                reset_xform_properties=False,
            )
            my_world.scene.add(r1pro_articulation)
            my_world.reset()
            for _ in range(3):
                asset_controller.apply_locked_poses()
                my_world.step(render=True)
            if robot_articulation is not None:
                robot_articulation.initialize(my_world.physics_sim_view)
            r1pro_articulation.initialize(my_world.physics_sim_view)
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Failed to initialize r1pro articulation control: {exc}")
            r1pro_articulation = None

    if robot_articulation is not None:
        try:
            ranger_controller = RangerArmTeleopController(
                robot_articulation,
                carb,
                robot_root_path,
                get_world_pose,
                wheel_control_paths,
                wheel_indices,
                steering_control_paths,
                steering_indices,
                args,
            )
            arm_ik_enabled = ranger_controller.arm_ik_enabled
            active_ik_mode = ranger_controller.active_target_mode
            if arm_ik_enabled:
                carb.log_warn(
                    f"Ranger arm IK ready. Active arm: {active_ik_mode}. "
                    "W/S drive base forward/back, A/D steer base left/right; "
                    "TAB switches left/right/both; use I/K, J/L, U/O, T/G, F/H, R/Y, 7/8 for 7-DOF joints; "
                    "M/N open/close gripper."
                )
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Failed to build ranger_arm teleop controller: {exc}")
            ranger_controller = None
            arm_ik_enabled = False

    if r1pro_articulation is not None:
        try:
            r1pro_body_root_path = r1pro_scene_prim_path
            articulation_prim_name = r1pro_articulation_prim_path.rstrip("/").split("/")[-1]
            if articulation_prim_name in {"base_link", "base_footprint_x"}:
                r1pro_body_root_path = "/".join(r1pro_articulation_prim_path.rstrip("/").split("/")[:-1])

            r1pro_controller = R1ProTeleopController(
                r1pro_articulation,
                carb,
                get_world_pose,
                args,
                r1pro_body_root_path,
            )
            carb.log_warn(
                f"r1pro teleop ready. Active gripper target: {r1pro_controller.active_target_mode}. "
                "Use F1 or 2 to switch robot, W/S forward-back, A/D steer, Q/E yaw (r1pro only), "
                "TAB to switch left/right/both arms, 7/8 for arm joint7, 5/6 for torso yaw, "
                "M/N to open/close, F6/F7/F8 to switch robot cameras, F9/F10 to switch drone cameras."
            )
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Failed to build r1pro teleop controller: {exc}")
            r1pro_controller = None

    for robot_name, camera_aliases in robot_camera_aliases.items():
        for alias, camera_path in camera_aliases.items():
            carb.log_info(f"{robot_name} camera ready: {alias} -> {camera_path}")

    dispatcher = TeleopDispatcher([ranger_controller, r1pro_controller])
    if r1pro_controller is not None and dispatcher.set_active("r1pro"):
        carb.log_info("Active teleop controller switched to: r1pro")
    target_reach = None
    time_code = Usd.TimeCode.Default()
    cfg = TeleopConfig(speed=args.speed, turn_rate=args.turn_rate, lift_rate=args.lift_rate)

    def _keyboard_inputs(*names):
        return [key for name in names if (key := getattr(carb.input.KeyboardInput, name, None)) is not None]

    keys = [
        carb.input.KeyboardInput.W,
        carb.input.KeyboardInput.A,
        carb.input.KeyboardInput.S,
        carb.input.KeyboardInput.D,
        carb.input.KeyboardInput.UP,
        carb.input.KeyboardInput.DOWN,
        carb.input.KeyboardInput.LEFT,
        carb.input.KeyboardInput.RIGHT,
        carb.input.KeyboardInput.Q,
        carb.input.KeyboardInput.E,
        carb.input.KeyboardInput.J,
        carb.input.KeyboardInput.L,
        carb.input.KeyboardInput.F1,
        carb.input.KeyboardInput.F2,
        carb.input.KeyboardInput.F6,
        carb.input.KeyboardInput.F7,
        carb.input.KeyboardInput.F8,
        carb.input.KeyboardInput.F9,
        carb.input.KeyboardInput.F10,
        carb.input.KeyboardInput.KEY_1,
        carb.input.KeyboardInput.KEY_2,
        carb.input.KeyboardInput.KEY_3,
        carb.input.KeyboardInput.KEY_4,
        carb.input.KeyboardInput.KEY_5,
        carb.input.KeyboardInput.KEY_7,
        carb.input.KeyboardInput.KEY_8,
        carb.input.KeyboardInput.KEY_6,
    ]
    if dispatcher is not None and dispatcher.active_name is not None:
        keys.extend(
            [
                carb.input.KeyboardInput.I,
                carb.input.KeyboardInput.K,
                carb.input.KeyboardInput.U,
                carb.input.KeyboardInput.O,
                carb.input.KeyboardInput.M,
                carb.input.KeyboardInput.N,
                carb.input.KeyboardInput.TAB,
                carb.input.KeyboardInput.T,
                carb.input.KeyboardInput.G,
                carb.input.KeyboardInput.F,
                carb.input.KeyboardInput.H,
                carb.input.KeyboardInput.R,
                carb.input.KeyboardInput.Y,
                carb.input.KeyboardInput.Z,
                carb.input.KeyboardInput.X,
                carb.input.KeyboardInput.C,
                carb.input.KeyboardInput.V,
                carb.input.KeyboardInput.B,
                carb.input.KeyboardInput.P,
            ]
        )
    app_window = omni.appwindow.get_default_app_window()
    input_iface = carb.input.acquire_input_interface()
    # Wait briefly for UI/keyboard to become available.
    keyboard_device = None
    for _ in range(200):
        keyboard_device = app_window.get_keyboard() if app_window is not None else None
        if keyboard_device is not None:
            break
        sim_app.update()
        time.sleep(0.01)

    keyboard = KeyboardState(
        keys,
        app_window,
        input_iface,
        carb,
        args.debug_input,
    )
    if keyboard_device is None:
        carb.log_warn("Keyboard device is still None after waiting. Input events may not be received.")
    keyboard.connect()

    if dispatcher is not None and dispatcher.active_name is not None:
        carb.log_info(
            f"Teleop ready. Active controller: {dispatcher.active_name}. "
            "Use F1 or 1/2 to switch robot; W/S drive forward-back; A/D steer base; "
            "TAB switches left/right/both; I/K J/L U/O T/G F/H R/Y 7/8 drive 7-DOF arm joints; "
            "5/6 torso yaw; "
            "M/N gripper; F6/F7/F8 switch active robot cameras; F9/F10 switch drone cameras; "
            "3/4 select drone teleop, F2 returns to robot teleop."
        )
    else:
        carb.log_info("Teleop ready. Focus the viewport and use W/S/A/D for base. ESC to quit.")
    app = omni.kit.app.get_app()
    if app is None:
        carb.log_error("Failed to acquire omni.kit.app. Exiting.")
        return

    try:
        carb.log_info("Entering teleop loop.")
        last_heartbeat = time.time()
        last_selection_clear = 0.0
        while True:
            _advance_stage_animation()
            asset_controller.apply_locked_poses()
            _apply_drone_trajectories()
            if robot_articulation is not None or r1pro_articulation is not None:
                my_world.step(render=True)
            else:
                sim_app.update()
            _update_robot_camera_rigs()
            asset_controller.apply_locked_poses()
            _apply_drone_trajectories()
            time.sleep(0.001)

            if args.clear_ui_selection and time.time() - last_selection_clear > 1.0:
                _clear_stage_selection()
                last_selection_clear = time.time()

            if args.debug_loop and (time.time() - last_heartbeat) > 2.0:
                carb.log_info("Teleop loop heartbeat.")
                last_heartbeat = time.time()
            if args.debug_input and keyboard.seconds_since_event() > 5.0:
                keyboard.ensure_connected()

            if getattr(app, "is_exiting", None) and app.is_exiting() and not args.keep_alive:
                carb.log_warn("App is exiting; shutting down teleop loop.")
                break
            if args.play_stage_animation and timeline.is_stopped():
                timeline.play()

            if args.quit_on_esc and keyboard.pressed(carb.input.KeyboardInput.ESCAPE):
                break

            if keyboard.consume_pressed(carb.input.KeyboardInput.F1):
                active_name = dispatcher.cycle_active()
                if active_name is not None:
                    carb.log_warn(f"Active teleop controller switched to: {active_name}")
                else:
                    carb.log_warn("No available teleop controller is registered.")
            key_1_pressed = keyboard.consume_pressed(carb.input.KeyboardInput.KEY_1)
            if key_1_pressed:
                active_control_context["kind"] = "robot"
                active_control_context["path"] = None
                if dispatcher.set_active("ranger_arm"):
                    carb.log_warn("Active teleop controller switched to: ranger_arm")
                else:
                    carb.log_warn(f"ranger_arm controller is unavailable. Registered controllers: {dispatcher.names()}")
            key_2_pressed = keyboard.consume_pressed(carb.input.KeyboardInput.KEY_2)
            if key_2_pressed:
                active_control_context["kind"] = "robot"
                active_control_context["path"] = None
                if dispatcher.set_active("r1pro"):
                    carb.log_warn("Active teleop controller switched to: r1pro")
                else:
                    carb.log_warn(f"r1pro controller is unavailable. Registered controllers: {dispatcher.names()}")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F2):
                active_control_context["kind"] = "robot"
                active_control_context["path"] = None
                carb.log_warn(f"Keyboard control returned to robot teleop: {dispatcher.active_name}")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F6):
                _switch_robot_camera("head_top")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F7):
                _switch_robot_camera("left_gripper")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F8):
                _switch_robot_camera("right_gripper")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F9):
                _switch_named_camera("cf2x", "chase")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F10):
                _switch_named_camera("cf2x_01", "chase")
            if keyboard.consume_pressed(carb.input.KeyboardInput.KEY_3):
                drone_state = drone_states_by_path.get("/World/cf2x")
                if drone_state is not None:
                    drone_state["manual_override"] = True
                    active_control_context["kind"] = "drone"
                    active_control_context["path"] = "/World/cf2x"
                    carb.log_warn("Drone teleop selected: cf2x. Use W/S A/D Q/E J/L to move.")
                else:
                    carb.log_warn("cf2x is unavailable for manual teleop.")
            if keyboard.consume_pressed(carb.input.KeyboardInput.KEY_4):
                drone_state = drone_states_by_path.get("/World/cf2x_01")
                if drone_state is not None:
                    drone_state["manual_override"] = True
                    active_control_context["kind"] = "drone"
                    active_control_context["path"] = "/World/cf2x_01"
                    carb.log_warn("Drone teleop selected: cf2x_01. Use W/S A/D Q/E J/L to move.")
                else:
                    carb.log_warn("cf2x_01 is unavailable for manual teleop.")
            if keyboard.consume_pressed(carb.input.KeyboardInput.TAB):
                target_mode = dispatcher.cycle_target_mode()
                if target_mode is not None:
                    carb.log_warn(f"{dispatcher.active_name} active target switched to: {target_mode}")

            camera_yaw_step = 4.0
            camera_pitch_step = 3.0
            if keyboard.pressed(carb.input.KeyboardInput.LEFT) or keyboard.poll_pressed(carb.input.KeyboardInput.LEFT):
                _adjust_active_camera(-camera_yaw_step, 0.0)
            if keyboard.pressed(carb.input.KeyboardInput.RIGHT) or keyboard.poll_pressed(carb.input.KeyboardInput.RIGHT):
                _adjust_active_camera(camera_yaw_step, 0.0)
            if keyboard.pressed(carb.input.KeyboardInput.UP) or keyboard.poll_pressed(carb.input.KeyboardInput.UP):
                _adjust_active_camera(0.0, -camera_pitch_step)
            if keyboard.pressed(carb.input.KeyboardInput.DOWN) or keyboard.poll_pressed(carb.input.KeyboardInput.DOWN):
                _adjust_active_camera(0.0, camera_pitch_step)

            forward = float(
                keyboard.pressed(carb.input.KeyboardInput.W)
                or keyboard.poll_pressed(carb.input.KeyboardInput.W)
            ) - float(
                keyboard.pressed(carb.input.KeyboardInput.S)
                or keyboard.poll_pressed(carb.input.KeyboardInput.S)
            )
            strafe = float(
                keyboard.pressed(carb.input.KeyboardInput.D)
                or keyboard.poll_pressed(carb.input.KeyboardInput.D)
            ) - float(
                keyboard.pressed(carb.input.KeyboardInput.A)
                or keyboard.poll_pressed(carb.input.KeyboardInput.A)
            )
            lift = float(
                keyboard.pressed(carb.input.KeyboardInput.E) or keyboard.poll_pressed(carb.input.KeyboardInput.E)
            ) - float(
                keyboard.pressed(carb.input.KeyboardInput.Q) or keyboard.poll_pressed(carb.input.KeyboardInput.Q)
            )
            drone_yaw = float(
                keyboard.pressed(carb.input.KeyboardInput.L) or keyboard.poll_pressed(carb.input.KeyboardInput.L)
            ) - float(
                keyboard.pressed(carb.input.KeyboardInput.J) or keyboard.poll_pressed(carb.input.KeyboardInput.J)
            )
            active_controller_name = dispatcher.active_name if dispatcher is not None else None
            ranger_arm_is_active = active_controller_name == "ranger_arm"
            r1pro_is_active = active_controller_name == "r1pro"
            yaw = 0.0
            drone_control_active = active_control_context["kind"] == "drone" and active_control_context["path"] in drone_states_by_path
            if drone_control_active:
                yaw = drone_yaw
            elif r1pro_is_active:
                yaw = float(
                    keyboard.pressed(carb.input.KeyboardInput.E) or keyboard.poll_pressed(carb.input.KeyboardInput.E)
                ) - float(
                    keyboard.pressed(carb.input.KeyboardInput.Q) or keyboard.poll_pressed(carb.input.KeyboardInput.Q)
                )
                lift = 0.0
            elif ranger_arm_is_active:
                yaw = -strafe
                strafe = 0.0
                lift = 0.0
            if args.use_ui:
                # Always read live slider values to avoid missed callbacks.
                if ui_models["forward"] is not None:
                    forward = float(ui_models["forward"].model.get_value_as_float())
                if ui_models["strafe"] is not None:
                    strafe = float(ui_models["strafe"].model.get_value_as_float())
                if ui_models["lift"] is not None:
                    lift = float(ui_models["lift"].model.get_value_as_float())
                if ui_models["yaw"] is not None:
                    yaw = float(ui_models["yaw"].model.get_value_as_float())

            if drone_control_active:
                active_drone_state = drone_states_by_path[active_control_context["path"]]
                linear_step = 1.2 * args.dt
                vertical_step = 0.8 * args.dt
                yaw_step = 75.0 * args.dt
                yaw_rad = math.radians(float(active_drone_state["yaw_deg"]))
                forward_dir = np.array([math.cos(yaw_rad), math.sin(yaw_rad)], dtype=np.float64)
                right_dir = np.array([-math.sin(yaw_rad), math.cos(yaw_rad)], dtype=np.float64)
                planar_delta = (forward * forward_dir + strafe * right_dir) * linear_step
                active_drone_state["translation"] = Gf.Vec3d(
                    float(active_drone_state["translation"][0]) + float(planar_delta[0]),
                    float(active_drone_state["translation"][1]) + float(planar_delta[1]),
                    float(active_drone_state["translation"][2]) + lift * vertical_step,
                )
                active_drone_state["yaw_deg"] = float(active_drone_state["yaw_deg"]) + yaw * yaw_step
                target_pitch_deg = -10.0 * forward
                target_roll_deg = -8.0 * strafe
                active_drone_state["pitch_deg"] = 0.8 * float(active_drone_state.get("pitch_deg", 0.0)) + 0.2 * target_pitch_deg
                active_drone_state["roll_deg"] = 0.8 * float(active_drone_state.get("roll_deg", 0.0)) + 0.2 * target_roll_deg
                control_effort = min(1.0, abs(forward) + abs(strafe) + abs(lift) + abs(yaw))
                active_drone_state["rotor_speed_deg_per_sec"] = 720.0 + 1080.0 * control_effort
                _apply_drone_manual_pose(active_drone_state)
                forward = 0.0
                strafe = 0.0
                lift = 0.0
                yaw = 0.0

            arm_target_delta = np.zeros(3, dtype=np.float32)
            arm_rotation_delta = np.zeros(3, dtype=np.float32)
            ranger_joint_delta = np.zeros(7, dtype=np.float32)
            torso_delta = np.zeros(4, dtype=np.float32)
            joint7_delta = 0.0
            gripper_delta = 0.0
            arm_motion_has_input = False
            arm_rotation_has_input = False
            ranger_joint_has_input = False
            joint7_has_input = False
            torso_has_input = False
            if dispatcher is not None and dispatcher.active_name is not None:
                ik_forward = float(
                    keyboard.pressed(carb.input.KeyboardInput.I) or keyboard.poll_pressed(carb.input.KeyboardInput.I)
                ) - float(
                    keyboard.pressed(carb.input.KeyboardInput.K) or keyboard.poll_pressed(carb.input.KeyboardInput.K)
                )
                ik_strafe = float(
                    keyboard.pressed(carb.input.KeyboardInput.L) or keyboard.poll_pressed(carb.input.KeyboardInput.L)
                ) - float(
                    keyboard.pressed(carb.input.KeyboardInput.J) or keyboard.poll_pressed(carb.input.KeyboardInput.J)
                )
                ik_lift = float(
                    keyboard.pressed(carb.input.KeyboardInput.U) or keyboard.poll_pressed(carb.input.KeyboardInput.U)
                ) - float(
                    keyboard.pressed(carb.input.KeyboardInput.O) or keyboard.poll_pressed(carb.input.KeyboardInput.O)
                )
                gripper_delta = (
                    float(
                        keyboard.pressed(carb.input.KeyboardInput.M)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.M)
                    )
                    - float(
                        keyboard.pressed(carb.input.KeyboardInput.N)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.N)
                    )
                ) * args.gripper_speed * args.dt
                arm_roll = float(
                    keyboard.pressed(carb.input.KeyboardInput.T) or keyboard.poll_pressed(carb.input.KeyboardInput.T)
                ) - float(
                    keyboard.pressed(carb.input.KeyboardInput.G) or keyboard.poll_pressed(carb.input.KeyboardInput.G)
                )
                arm_pitch = float(
                    keyboard.pressed(carb.input.KeyboardInput.F) or keyboard.poll_pressed(carb.input.KeyboardInput.F)
                ) - float(
                    keyboard.pressed(carb.input.KeyboardInput.H) or keyboard.poll_pressed(carb.input.KeyboardInput.H)
                )
                arm_yaw = float(
                    keyboard.pressed(carb.input.KeyboardInput.R) or keyboard.poll_pressed(carb.input.KeyboardInput.R)
                ) - float(
                    keyboard.pressed(carb.input.KeyboardInput.Y) or keyboard.poll_pressed(carb.input.KeyboardInput.Y)
                )
                if ranger_arm_is_active and arm_ik_enabled:
                    ranger_joint_delta = np.array(
                        [
                            ik_forward,
                            ik_strafe,
                            ik_lift,
                            arm_roll,
                            arm_pitch,
                            arm_yaw,
                            0.0,
                        ],
                        dtype=np.float32,
                    )
                    ranger_joint_delta[:3] *= args.ik_speed * args.dt * 8.0
                    ranger_joint_delta[3:6] *= args.ik_rotation_speed * args.dt * 2.0
                elif r1pro_is_active:
                    arm_target_delta = np.array([ik_forward, ik_strafe, ik_lift], dtype=np.float32) * args.ik_speed * args.dt
                    arm_rotation_delta = (
                        np.array([arm_roll, arm_pitch, arm_yaw], dtype=np.float32) * args.ik_rotation_speed * args.dt
                    )
                if ranger_arm_is_active and arm_ik_enabled:
                    joint7_axis = float(
                        keyboard.pressed(carb.input.KeyboardInput.KEY_7)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.KEY_7)
                    ) - float(
                        keyboard.pressed(carb.input.KeyboardInput.KEY_8)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.KEY_8)
                    )
                    ranger_joint_delta[6] = joint7_axis * args.ik_rotation_speed * args.dt * 2.0
                elif r1pro_is_active:
                    joint7_axis = float(
                        keyboard.pressed(carb.input.KeyboardInput.KEY_7)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.KEY_7)
                    ) - float(
                        keyboard.pressed(carb.input.KeyboardInput.KEY_8)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.KEY_8)
                    )
                    joint7_delta = joint7_axis * args.ik_rotation_speed * args.dt
                if r1pro_is_active:
                    torso_1 = float(
                        keyboard.pressed(carb.input.KeyboardInput.Z) or keyboard.poll_pressed(carb.input.KeyboardInput.Z)
                    ) - float(
                        keyboard.pressed(carb.input.KeyboardInput.X) or keyboard.poll_pressed(carb.input.KeyboardInput.X)
                    )
                    torso_2 = float(
                        keyboard.pressed(carb.input.KeyboardInput.C) or keyboard.poll_pressed(carb.input.KeyboardInput.C)
                    ) - float(
                        keyboard.pressed(carb.input.KeyboardInput.V) or keyboard.poll_pressed(carb.input.KeyboardInput.V)
                    )
                    torso_3 = float(
                        keyboard.pressed(carb.input.KeyboardInput.B) or keyboard.poll_pressed(carb.input.KeyboardInput.B)
                    ) - float(
                        keyboard.pressed(carb.input.KeyboardInput.P) or keyboard.poll_pressed(carb.input.KeyboardInput.P)
                    )
                    torso_4 = float(
                        keyboard.pressed(carb.input.KeyboardInput.KEY_5)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.KEY_5)
                    ) - float(
                        keyboard.pressed(carb.input.KeyboardInput.KEY_6)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.KEY_6)
                    )
                    torso_delta = (
                        np.array([torso_1, torso_2, torso_3, torso_4], dtype=np.float32)
                        * args.ik_rotation_speed
                        * args.dt
                    )
                arm_motion_has_input = bool(np.any(arm_target_delta))
                arm_rotation_has_input = bool(np.any(arm_rotation_delta))
                ranger_joint_has_input = bool(np.any(ranger_joint_delta))
                joint7_has_input = joint7_delta != 0.0
                torso_has_input = bool(np.any(torso_delta))

            base_has_input = forward != 0.0 or strafe != 0.0 or lift != 0.0 or yaw != 0.0
            arm_has_input = bool(
                arm_motion_has_input
                or arm_rotation_has_input
                or ranger_joint_has_input
                or joint7_has_input
                or torso_has_input
                or gripper_delta != 0.0
            )
            if args.debug_input and (base_has_input or arm_has_input):
                carb.log_info(
                    f"Input state f={forward} s={strafe} z={lift} yaw={yaw} "
                    f"ik_delta={arm_target_delta.tolist()} rot_delta={arm_rotation_delta.tolist()} "
                    f"joint7_delta={joint7_delta:.4f} gripper_delta={gripper_delta:.4f}"
                )

            active_controller = dispatcher.active_controller() if dispatcher is not None else None
            if drone_control_active:
                active_controller = None
            if (
                active_controller is None
                and not base_has_input
                and not arm_has_input
            ):
                continue

            if timeline.is_stopped():
                timeline.play()
            should_dispatch = bool(
                base_has_input
                or arm_has_input
                or active_controller is not None
            )
            if should_dispatch:
                dispatcher.step(
                    BaseMotionCommand(forward=forward, strafe=strafe, lift=lift, yaw=yaw),
                    ManipulatorMotionCommand(
                        delta_xyz=arm_target_delta if arm_motion_has_input else None,
                        delta_rot=arm_rotation_delta if arm_rotation_has_input else None,
                        joint_delta=ranger_joint_delta if ranger_joint_has_input else None,
                        torso_delta=torso_delta if torso_has_input else None,
                        joint7_delta=joint7_delta if joint7_has_input else 0.0,
                        gripper_delta=gripper_delta,
                    ),
                )
                _update_robot_camera_rigs()
                _apply_drone_trajectories()
    except Exception:
        carb.log_error("Unhandled exception in teleop loop.")
        import traceback  # noqa: WPS433

        carb.log_error(traceback.format_exc())
    finally:
        keyboard.disconnect()
        sim_app.close()


if __name__ == "__main__":
    main()
