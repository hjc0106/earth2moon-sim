"""天宫 IsaacSim 键盘遥操作入口。

负责加载天宫场景、Ranger Arm、R1 Pro 和辅助展示资产，并把键盘输入分发给
对应机器人控制器。Ranger Arm IK、R1 Pro 底盘和地面贴合逻辑都在 teleop 模块中实现。
"""

import argparse
import csv
import json
import math
import os
import socket
import sys
import threading
import time
from collections import deque
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


def _quat_conjugate(quat_wxyz):
    """返回四元数共轭。"""
    import numpy as np  # noqa: WPS433

    quat = np.asarray(quat_wxyz, dtype=np.float32).reshape(4)
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=np.float32)


def _quat_multiply(lhs_wxyz, rhs_wxyz):
    """四元数相乘，输入输出均为 wxyz。"""
    import numpy as np  # noqa: WPS433

    w0, x0, y0, z0 = np.asarray(lhs_wxyz, dtype=np.float32).reshape(4)
    w1, x1, y1, z1 = np.asarray(rhs_wxyz, dtype=np.float32).reshape(4)
    return np.array(
        [
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        ],
        dtype=np.float32,
    )


def _normalize_quat(quat_wxyz):
    """归一化四元数。"""
    import numpy as np  # noqa: WPS433

    quat = np.asarray(quat_wxyz, dtype=np.float32).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return quat / norm


def _rotate_vec_by_quat(quat_wxyz, vec_xyz):
    """用四元数旋转三维向量。"""
    import numpy as np  # noqa: WPS433

    quat = _normalize_quat(quat_wxyz)
    vec = np.asarray(vec_xyz, dtype=np.float32).reshape(3)
    vec_quat = np.array([0.0, vec[0], vec[1], vec[2]], dtype=np.float32)
    rotated = _quat_multiply(_quat_multiply(quat, vec_quat), _quat_conjugate(quat))
    return rotated[1:].astype(np.float32)


def _yaw_only_quat(quat_wxyz):
    """仅保留四元数的 yaw 分量，便于把 VR 位移对齐到机器人朝向。"""
    import numpy as np  # noqa: WPS433

    quat = _normalize_quat(quat_wxyz)
    w, x, y, z = [float(v) for v in quat]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    half_yaw = 0.5 * yaw
    return np.array([math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)], dtype=np.float32)


def _scale_relative_quat(delta_quat_wxyz, scale: float):
    """把相对旋转四元数按比例放大/缩小。"""
    import numpy as np  # noqa: WPS433

    delta_quat = _normalize_quat(delta_quat_wxyz)
    w = float(np.clip(delta_quat[0], -1.0, 1.0))
    angle = 2.0 * math.acos(w)
    sin_half = math.sqrt(max(1.0 - w * w, 0.0))
    if sin_half < 1e-6 or abs(angle) < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    axis = delta_quat[1:4] / sin_half
    scaled_angle = angle * float(scale)
    half = 0.5 * scaled_angle
    return _normalize_quat(
        np.array(
            [math.cos(half), axis[0] * math.sin(half), axis[1] * math.sin(half), axis[2] * math.sin(half)],
            dtype=np.float32,
        )
    )


def _mirror_pose_across_xz(pos_xyz, quat_wxyz):
    """按官方 mirror teleop 路线，把位姿关于 XZ 平面镜像一次。

    这一步对应 `r1pro_control/robot_control_ros2/dual_arm_tele_mirror.py`
    里的 `M_H @ T @ M_H`：
    - 位置: 只翻转 y
    - 旋转: 翻转与横向相关的 roll / yaw 方向
    """
    import numpy as np  # noqa: WPS433

    pos = np.asarray(pos_xyz, dtype=np.float32).reshape(3).copy()
    quat = _normalize_quat(quat_wxyz)
    pos[1] *= -1.0
    mirrored_quat = np.array([quat[0], -quat[1], quat[2], -quat[3]], dtype=np.float32)
    return pos.astype(np.float32), _normalize_quat(mirrored_quat)


def _compose_pose(base_pos, base_quat_wxyz, local_pos, local_quat_wxyz):
    """组合两个位姿，返回世界位姿。"""
    import numpy as np  # noqa: WPS433

    base_pos = np.asarray(base_pos, dtype=np.float32).reshape(3)
    base_quat = _normalize_quat(base_quat_wxyz)
    local_pos = np.asarray(local_pos, dtype=np.float32).reshape(3)
    local_quat = _normalize_quat(local_quat_wxyz)
    world_pos = base_pos + _rotate_vec_by_quat(base_quat, local_pos)
    world_quat = _normalize_quat(_quat_multiply(base_quat, local_quat))
    return world_pos.astype(np.float32), world_quat.astype(np.float32)


def _inverse_pose(pos_xyz, quat_wxyz):
    """返回位姿逆。"""
    import numpy as np  # noqa: WPS433

    pos = np.asarray(pos_xyz, dtype=np.float32).reshape(3)
    quat = _normalize_quat(quat_wxyz)
    inv_quat = _quat_conjugate(quat)
    inv_pos = _rotate_vec_by_quat(inv_quat, -pos)
    return inv_pos.astype(np.float32), inv_quat.astype(np.float32)


def _relative_pose(base_pos, base_quat_wxyz, world_pos, world_quat_wxyz):
    """计算世界位姿在基座位姿下的相对位姿。"""
    inv_pos, inv_quat = _inverse_pose(base_pos, base_quat_wxyz)
    return _compose_pose(inv_pos, inv_quat, world_pos, world_quat_wxyz)


def _retarget_openxr_motion_controller_pose(pos_xyz, quat_wxyz):
    """对 OpenXR motion controller pose 做固定坐标系修正。"""
    import numpy as np  # noqa: WPS433

    controller_pos = np.asarray(pos_xyz, dtype=np.float32).reshape(3)
    controller_quat = _normalize_quat(quat_wxyz)
    # 这里只修正控制器朝向，不旋转绝对位置。
    # 之前把 position 也乘上固定四元数，会把世界坐标轴一并扭掉，
    # 直接造成左右平移反向和末端跟随不自然。
    #
    # 这里使用“当前控制器姿态 × 固定局部外参”，更接近官方 wrist/raw EE pose
    # 的处理语义；如果改成左乘，会更像把世界坐标先整体旋掉，末端转向通常就会发飘。
    correction_quat = np.array([0.5358, -0.4619, 0.5358, 0.4619], dtype=np.float32)
    corrected_pos = controller_pos.copy()
    corrected_quat = _normalize_quat(_quat_multiply(controller_quat, correction_quat))
    return corrected_pos.astype(np.float32), corrected_quat.astype(np.float32)


def _query_openxr_controller(input_device):
    """读取单个 OpenXR 手柄，输出 2x7 数组。"""
    import numpy as np  # noqa: WPS433

    if input_device is None:
        return np.array([])

    # 使用物理追踪空间 pose，避免 XR 相机锚点和机器人底盘运动反向污染手柄增量。
    pose = input_device.get_pose("")
    position = pose.ExtractTranslation()
    quat = pose.ExtractRotationQuat()

    thumbstick_x = 0.0
    thumbstick_y = 0.0
    trigger = 0.0
    squeeze = 0.0
    button_0 = 0.0
    button_1 = 0.0

    if input_device.has_input_gesture("thumbstick", "x"):
        thumbstick_x = float(input_device.get_input_gesture_value("thumbstick", "x"))
    if input_device.has_input_gesture("thumbstick", "y"):
        thumbstick_y = float(input_device.get_input_gesture_value("thumbstick", "y"))
    if input_device.has_input_gesture("trigger", "value"):
        trigger = float(input_device.get_input_gesture_value("trigger", "value"))
    if input_device.has_input_gesture("squeeze", "value"):
        squeeze = float(input_device.get_input_gesture_value("squeeze", "value"))

    if input_device.has_input_gesture("x", "click") or input_device.has_input_gesture("y", "click"):
        if input_device.has_input_gesture("x", "click"):
            button_0 = float(input_device.get_input_gesture_value("x", "click"))
        if input_device.has_input_gesture("y", "click"):
            button_1 = float(input_device.get_input_gesture_value("y", "click"))
    else:
        if input_device.has_input_gesture("a", "click"):
            button_0 = float(input_device.get_input_gesture_value("a", "click"))
        if input_device.has_input_gesture("b", "click"):
            button_1 = float(input_device.get_input_gesture_value("b", "click"))

    pose_row = [
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(quat.GetReal()),
        float(quat.GetImaginary()[0]),
        float(quat.GetImaginary()[1]),
        float(quat.GetImaginary()[2]),
    ]
    input_row = [thumbstick_x, thumbstick_y, trigger, squeeze, button_0, button_1, 0.0]
    return np.array([pose_row, input_row], dtype=np.float32)


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


def default_xr_openxr_experience_path() -> str:
    """返回 Isaac Sim XR OpenXR experience 路径。"""
    exp_path = os.environ.get("EXP_PATH", "").strip()
    if exp_path:
        candidate = Path(exp_path) / "isaacsim.exp.base.xr.openxr.kit"
        if candidate.exists():
            return str(candidate)
    repo_root = Path(__file__).resolve().parents[3]
    candidate = repo_root / "IsaacSim" / "apps" / "isaacsim.exp.base.xr.openxr.kit"
    return str(candidate)


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


class RobotSwitchCommandServer:
    """接收外部机器人切换命令，复用现有 teleop dispatcher 切换逻辑。"""

    def __init__(self, host: str, port: int, carb_module):
        self._host = host
        self._port = int(port)
        self._carb = carb_module
        self._queue = deque()
        self._lock = threading.Lock()
        self._control_state = {}
        self._control_timestamp = 0.0
        self._running = False
        self._thread = None
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self._host, self._port))
        self._socket.settimeout(0.25)

    @property
    def address(self) -> tuple[str, int]:
        return self._host, self._port

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._serve, name="vr-robot-switch", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        try:
            self._socket.close()
        except OSError:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def pop_command(self) -> str | None:
        with self._lock:
            if not self._queue:
                return None
            return self._queue.popleft()

    def get_control_state(self, max_age_s: float = 0.3) -> dict[str, float]:
        with self._lock:
            if not self._control_state:
                return {}
            if max_age_s > 0.0 and (time.time() - self._control_timestamp) > max_age_s:
                return {}
            return dict(self._control_state)

    def _serve(self) -> None:
        while self._running:
            try:
                payload, _addr = self._socket.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            command, control_state = self._normalize_payload(payload)
            if control_state is not None:
                with self._lock:
                    self._control_state = control_state
                    self._control_timestamp = time.time()
            if command is None:
                continue
            with self._lock:
                self._queue.append(command)

    @staticmethod
    def _normalize_payload(payload: bytes) -> tuple[str | None, dict[str, float] | None]:
        try:
            text = payload.decode("utf-8").strip()
        except UnicodeDecodeError:
            return None, None
        if not text:
            return None, None
        command = text
        control_state = None
        if text.startswith("{"):
            try:
                decoded = json.loads(text)
            except json.JSONDecodeError:
                decoded = None
            if isinstance(decoded, dict):
                control_payload = decoded.get("control")
                if isinstance(control_payload, dict):
                    control_state = RobotSwitchCommandServer._sanitize_control_state(control_payload)
                command = decoded.get("command") or decoded.get("action") or decoded.get("robot") or ""
        command = str(command).strip().lower().replace("-", "_")
        alias_map = {
            "1": "ranger_arm",
            "2": "r1pro",
            "cycle_robot": "cycle",
            "cycle_active": "cycle",
            "switch": "cycle",
            "return": "return_robot",
            "robot": "return_robot",
        }
        command = alias_map.get(command, command)
        if command in {"cycle", "ranger_arm", "r1pro", "return_robot"}:
            return command, control_state
        return None, control_state

    @staticmethod
    def _sanitize_control_state(payload: dict) -> dict[str, float]:
        fields = (
            "base_forward",
            "base_strafe",
            "base_lift",
            "base_yaw",
            "ik_forward",
            "ik_strafe",
            "ik_lift",
            "arm_roll",
            "arm_pitch",
            "arm_yaw",
            "joint7",
            "torso_1",
            "torso_2",
            "torso_3",
            "torso_4",
            "gripper",
            "left_arm_forward",
            "left_arm_strafe",
            "left_arm_lift",
            "left_arm_roll",
            "left_arm_pitch",
            "left_arm_yaw",
            "left_joint7",
            "left_gripper",
            "right_arm_forward",
            "right_arm_strafe",
            "right_arm_lift",
            "right_arm_roll",
            "right_arm_pitch",
            "right_arm_yaw",
            "right_joint7",
            "right_gripper",
        )
        state = {}
        for field in fields:
            raw_value = payload.get(field, 0.0)
            try:
                numeric = float(raw_value)
            except (TypeError, ValueError):
                numeric = 0.0
            state[field] = max(-1.0, min(1.0, numeric))
        return state


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
            carb_module.log_warn(f"r1pro articulation prepared at {scene_prim_path} (wheel-driven base).")
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
        "--log-joint-positions",
        action="store_true",
        help="Log the active robot's current joint positions after teleop input is applied.",
    )
    parser.add_argument(
        "--joint-position-log-interval",
        type=float,
        default=0.25,
        help="Minimum seconds between --log-joint-positions lines.",
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
        "--show-robot-feedback",
        action="store_true",
        default=True,
        help="Show an Isaac Sim window with active robot pose, joint positions, and measured joint efforts.",
    )
    parser.add_argument(
        "--no-show-robot-feedback",
        action="store_false",
        dest="show_robot_feedback",
        help="Hide the robot feedback window.",
    )
    parser.add_argument(
        "--robot-feedback-joint-limit",
        type=int,
        default=0,
        help="Maximum number of non-wheel joints shown in the robot feedback window. Use 0 to show all.",
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
        "--ik-orientation-weight",
        type=float,
        default=0.55,
        help="IK 中末端姿态误差相对位置误差的权重。",
    )
    parser.add_argument(
        "--ik-max-position-error",
        type=float,
        default=0.08,
        help="单帧送入 IK 的最大末端位置误差（米）。",
    )
    parser.add_argument(
        "--ik-max-orientation-error",
        type=float,
        default=0.20,
        help="单帧送入 IK 的最大末端姿态误差（弧度）。",
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
    parser.add_argument("--r1pro-y", type=float, default=0.0, help="Ground Y position for the local r1pro robot.")
    parser.add_argument("--r1pro-z", type=float, default=0.0, help="Z offset after bbox-grounding the local r1pro robot.")
    parser.add_argument("--r1pro-yaw", type=float, default=180.0, help="Ground yaw angle in degrees for the local r1pro robot.")
    parser.add_argument(
        "--r1pro-init-pose-preset",
        type=str,
        default="arms_forward_level",
        choices=("none", "arms_forward_level"),
        help="Initial r1pro joint preset. 'arms_forward_level' tries to place both arms forward with upper arms raised and forearms level.",
    )
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
    parser.add_argument(
        "--enable-vr-switch-udp",
        action="store_true",
        default=True,
        help="Listen for external VR robot-switch commands over UDP. Enabled by default.",
    )
    parser.add_argument(
        "--no-enable-vr-switch-udp",
        action="store_false",
        dest="enable_vr_switch_udp",
        help="Disable the UDP robot-switch listener.",
    )
    parser.add_argument(
        "--vr-switch-udp-host",
        type=str,
        default="0.0.0.0",
        help="Bind address for external VR robot-switch UDP commands.",
    )
    parser.add_argument(
        "--vr-switch-udp-port",
        type=int,
        default=35678,
        help="UDP port for external VR robot-switch commands.",
    )
    parser.add_argument(
        "--xr-openxr",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Launch this teleop scene with the Isaac Sim XR OpenXR experience.",
    )
    parser.add_argument(
        "--xr-experience",
        type=str,
        default=default_xr_openxr_experience_path(),
        help="Path to the Isaac Sim XR OpenXR .kit experience file.",
    )
    parser.add_argument(
        "--enable-openxr-r1pro-vr",
        action="store_true",
        default=False,
        help="直接读取 Quest3 OpenXR 手柄，在原场景中控制 r1pro 双臂末端和夹爪。",
    )
    parser.add_argument(
        "--openxr-vr-position-scale",
        type=float,
        default=1.0,
        help="Quest3 手柄位移映射到 r1pro 末端位移时的缩放系数。",
    )
    parser.add_argument(
        "--openxr-vr-forward-scale",
        type=float,
        default=1.6,
        help="在基础位移缩放上，额外放大手柄沿 r1pro 前向的位移。",
    )
    parser.add_argument(
        "--openxr-vr-lift-scale",
        type=float,
        default=1.6,
        help="在基础位移缩放上，额外放大手柄竖直抬升的位移。",
    )
    parser.add_argument(
        "--openxr-vr-rotation-scale",
        type=float,
        default=1.35,
        help="Quest3 手柄姿态映射到 r1pro 末端姿态时的旋转增益。",
    )
    parser.add_argument(
        "--openxr-vr-rotation-alpha",
        type=float,
        default=1.0,
        help="Quest3 手柄姿态写入 r1pro 末端目标时的平滑系数，越小越慢。",
    )
    parser.add_argument(
        "--openxr-vr-max-position-speed",
        type=float,
        default=1.5,
        help="VR 末端目标每秒允许的最大平移速度，抑制追踪跳变。",
    )
    parser.add_argument(
        "--openxr-vr-max-rotation-speed",
        type=float,
        default=4.0,
        help="VR 末端目标每秒允许的最大旋转速度（弧度/秒）。",
    )
    parser.add_argument(
        "--openxr-vr-torso-speed",
        type=float,
        default=0.9,
        help="Quest3 手柄按键控制 r1pro 腰部前两个关节时的速度系数。",
    )
    parser.add_argument(
        "--openxr-vr-base-speed",
        type=float,
        default=1.0,
        help="Quest3 左手柄摇杆映射到底盘前进/后退时的缩放系数。",
    )
    parser.add_argument(
        "--openxr-vr-base-yaw-speed",
        type=float,
        default=0.85,
        help="Quest3 左手柄摇杆映射到底盘转向时的缩放系数。",
    )
    parser.add_argument(
        "--openxr-vr-stick-deadband",
        type=float,
        default=0.08,
        help="Quest3 摇杆死区，小于该值时忽略输入。",
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

    sim_experience = ""
    if args.xr_openxr:
        sim_experience = str(Path(args.xr_experience).expanduser())
        if not Path(sim_experience).exists():
            raise FileNotFoundError(f"XR OpenXR experience not found: {sim_experience}")

    sim_app = SimulationApp(
        {"headless": args.headless, "renderer": "RTX", "extra_args": sim_extra_args},
        experience=sim_experience,
    )

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
    from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: WPS433
    from isaacsim.core.utils.types import ArticulationAction  # noqa: WPS433
    from isaacsim.core.utils.xforms import get_world_pose  # noqa: WPS433
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: WPS433

    if args.xr_openxr:
        settings = carb.settings.get_settings()
        settings.set_float("/persistent/xr/profile/ar/render/nearPlane", 0.15)
        settings.set_string("/persistent/xr/profile/ar/anchorMode", "custom anchor")
        settings.set_string("/xrstage/profile/ar/customAnchor", "/World/XRAnchor")

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
    if args.xr_openxr and not stage.GetPrimAtPath("/World/XRAnchor").IsValid():
        SingleXFormPrim(
            "/World/XRAnchor",
            position=np.array([0.0, 0.0, 0.0], dtype=np.float32),
            orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        )
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
    overview_camera_state = {"path": None}
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
        embedded_camera_candidates = [
            f"{scene_prim_path}/Root/r1_pro_with_gripper/zed_link/teleop_head_top/Camera",
            f"{scene_prim_path}/r1_pro_with_gripper/zed_link/teleop_head_top/Camera",
        ]
        for camera_path in embedded_camera_candidates:
            camera_prim = stage.GetPrimAtPath(camera_path)
            if not camera_prim.IsValid():
                continue
            camera_xform = camera_prim.GetParent()
            rotate_attr = camera_xform.GetAttribute("xformOp:rotateXYZ")
            if rotate_attr.IsValid():
                # USD Camera 沿局部 -Z 观察；该资产的 180 度局部偏航对准机体前方。
                rotate_attr.Set(Gf.Vec3f(0.0, 180.0, 0.0))
            return {"head_top": camera_path}

        head_candidates = [
            ("/Root/r1_pro_with_gripper/zed_link", (0.02, 0.0, 0.08), (0.0, 180.0, 0.0), 18.0),
            ("/r1_pro_with_gripper/zed_link", (0.02, 0.0, 0.08), (0.0, 180.0, 0.0), 18.0),
        ]
        aliases = {}
        for suffix, translate, rotate_xyz, focal_length in head_candidates:
            camera_path = _upsert_follow_camera(
                "r1pro",
                scene_prim_path,
                "head_top",
                f"{scene_prim_path}{suffix}",
                translate,
                rotate_xyz,
                focal_length,
            )
            if camera_path is not None and stage.GetPrimAtPath(camera_path).IsValid():
                aliases["head_top"] = camera_path
                break
        return aliases

    def _build_ranger_camera_aliases(scene_prim_path: str) -> dict[str, str]:
        head_candidates = [
            ("/chassis_link", (0.22, 0.0, 2.02), (22.0, 0.0, 0.0), 15.0),
            ("/body", (0.22, 0.0, 2.02), (22.0, 0.0, 0.0), 15.0),
            ("/base_link", (0.22, 0.0, 2.02), (22.0, 0.0, 0.0), 15.0),
            ("/base_footprint", (0.22, 0.0, 2.02), (22.0, 0.0, 0.0), 15.0),
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
                    rig = robot_camera_rigs.get(("ranger_arm", alias))
                    if rig is not None and alias == "head_top":
                        # Ranger 的车体正面是 +X；USD Camera 沿局部 -Z 观察。
                        rig["base_local_rotation"] = Gf.Quatf(
                            0.5,
                            Gf.Vec3f(0.5, -0.5, -0.5),
                        )
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
        if args.xr_openxr:
            _request_xr_camera_anchor(camera_path)
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
        carb.log_info(f"Viewport switched to {robot_name} camera: {camera_alias} -> {camera_path}")
        if args.xr_openxr:
            _request_xr_camera_anchor(camera_path)
        return True

    def _request_xr_camera_anchor(camera_path: str) -> bool:
        try:
            from omni.kit.xr.core import XRCore  # noqa: WPS433
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"XR anchor request skipped; XRCore unavailable: {exc}")
            return False

        try:
            xrcore = XRCore.get_singleton()
            camera_prim = stage.GetPrimAtPath(camera_path)
            if not camera_prim.IsValid():
                raise RuntimeError(f"camera prim not found on stage: {camera_path}")
            view_pose = UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            stage_anchor_path = camera_path
            robot_name = active_camera_control.get("robot_name")
            alias = active_camera_control.get("alias")
            rig = robot_camera_rigs.get((robot_name, alias)) if robot_name and alias else None
            if rig is not None:
                stage_anchor_path = rig["body_path"]
            xrcore.schedule_teleport_to_view(stage_anchor_path, view_pose)
            carb.log_info(f"XR teleport requested for camera: {camera_path}, anchor: {stage_anchor_path}")
            return True
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Failed to request XR teleport for {camera_path}: {exc}")
            return False

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

    _clear_existing_robot_cameras(
        "r1pro",
        r1pro_scene_prim_path,
    )
    r1pro_camera_aliases = _build_r1pro_camera_aliases(r1pro_scene_prim_path)
    robot_camera_aliases["r1pro"] = dict(r1pro_camera_aliases)
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
    overview_viewport = get_active_viewport()
    if overview_viewport is not None:
        overview_camera_path = overview_viewport.camera_path
        overview_camera_state["path"] = (
            overview_camera_path.pathString
            if hasattr(overview_camera_path, "pathString")
            else str(overview_camera_path)
        )
        carb.log_info(f"Startup overview camera recorded: {overview_camera_state['path']}")

    if args.add_r1pro and args.r1pro_physics and stage.GetPrimAtPath(r1pro_articulation_prim_path).IsValid():
        try:
            if timeline.is_stopped():
                timeline.play()
            carb.log_warn(
                f"Initializing r1pro articulation control from prim: {r1pro_articulation_prim_path}"
            )
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
            try:
                current_pos, _current_quat = r1pro_articulation.get_world_pose()
                yaw_rad = math.radians(float(args.r1pro_yaw))
                desired_quat = np.array([math.cos(0.5 * yaw_rad), 0.0, 0.0, math.sin(0.5 * yaw_rad)], dtype=np.float32)
                desired_pos = np.array(
                    [float(args.r1pro_x), float(args.r1pro_y), float(np.asarray(current_pos, dtype=np.float32)[2])],
                    dtype=np.float32,
                )
                r1pro_articulation.set_world_pose(position=desired_pos, orientation=desired_quat)
                for _ in range(3):
                    my_world.step(render=True)
                applied_pos, applied_quat = r1pro_articulation.get_world_pose()
                carb.log_warn(
                    "Applied r1pro articulation world pose override: "
                    f"pos={np.asarray(applied_pos, dtype=np.float32).tolist()}, "
                    f"quat={np.asarray(applied_quat, dtype=np.float32).tolist()}, yaw_deg={args.r1pro_yaw}"
                )
            except Exception as exc:  # noqa: BLE001
                carb.log_warn(f"Failed to force r1pro articulation world pose override: {exc}")
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
                "Use F1 for overview, 1/2 to switch robot, W/S forward-back, A/D steer, Q/E yaw (r1pro only), "
                "TAB to switch left/right/both arms, 7/8 for arm joint7, 5/6 for torso yaw, "
                "M/N to open/close, F6 to switch the active robot head camera, F9/F10 to switch drone cameras."
            )
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Failed to build r1pro teleop controller: {exc}")
            r1pro_controller = None
    elif args.add_r1pro and args.r1pro_physics:
        carb.log_warn(
            "r1pro articulation prim is invalid, controller will not be created: "
            f"{r1pro_articulation_prim_path}"
        )

    openxr_vr_bridge = None
    openxr_vr_adapters = {}
    openxr_xr_core = None
    openxr_vr_calibrated = False
    openxr_vr_last_calibrate_pressed = False
    openxr_vr_calibration_pose = {}
    openxr_vr_calibration_task_pose = {}
    openxr_head_limit_reference = {}
    openxr_head_device_alignment = {}

    if args.enable_openxr_r1pro_vr:
        if not args.xr_openxr:
            carb.log_warn("Ignoring --enable-openxr-r1pro-vr because --xr-openxr is not enabled.")
        elif r1pro_controller is None and ranger_controller is None:
            carb.log_warn("Ignoring OpenXR robot VR control because no robot controller is available.")
        else:
            try:
                from omni.kit.xr.core import XRCore  # noqa: WPS433
                from tiangong.vr_teleop import (  # noqa: WPS433
                    R1ProVRAdapter,
                    R1ProVRAdapterConfig,
                    VRBimanualBridge,
                    VRControllerState,
                )

                openxr_xr_core = XRCore.get_singleton()
                if openxr_xr_core is None:
                    carb.log_warn("XRCore singleton unavailable; OpenXR r1pro VR control disabled.")
                else:
                    openxr_vr_bridge = VRBimanualBridge()
                    openxr_vr_bridge.mapper.cfg.position_scale = args.openxr_vr_position_scale
                    adapter_config = R1ProVRAdapterConfig(
                        position_alpha=1.0,
                        rotation_alpha=args.openxr_vr_rotation_alpha,
                        gripper_alpha=1.0,
                        control_dt=args.dt,
                        max_position_speed=args.openxr_vr_max_position_speed,
                        max_rotation_speed=args.openxr_vr_max_rotation_speed,
                    )
                    for robot_name, controller in (
                        ("ranger_arm", ranger_controller),
                        ("r1pro", r1pro_controller),
                    ):
                        if controller is None or not getattr(controller, "arm_ik_tasks", None):
                            continue
                        openxr_vr_adapters[robot_name] = R1ProVRAdapter(controller, adapter_config)
                    carb.log_warn(
                        f"OpenXR robot VR control ready: {list(openxr_vr_adapters)}. "
                        "F3 选择 ranger_arm，F4 选择 r1pro；切换后把双手柄摆到期望起始姿态，"
                        "再同时按住左手柄 Y 和右手柄 B 完成当前机器人标定。"
                        " trigger=按住闭合/松开张开, squeeze=保留且不参与机械臂控制, "
                        "ranger_arm: 左摇杆前后=行驶、左右=转向；"
                        "r1pro: 左摇杆前后/左右=底盘移动、右摇杆左右=底盘自旋, "
                        "左手柄 X/Y=腰部关节1负/正, "
                        "右手柄 A/B=腰部关节2负/正, 左右手柄姿态分别控制左右臂末端。"
                    )
            except Exception as exc:  # noqa: BLE001
                carb.log_warn(f"Failed to initialize OpenXR r1pro VR control: {exc}")
                openxr_vr_bridge = None
                openxr_vr_adapters.clear()
                openxr_xr_core = None

    for robot_name, camera_aliases in robot_camera_aliases.items():
        for alias, camera_path in camera_aliases.items():
            carb.log_info(f"{robot_name} camera ready: {alias} -> {camera_path}")

    dispatcher = TeleopDispatcher([ranger_controller, r1pro_controller])
    last_joint_position_log_time = 0.0
    last_feedback_window_update_time = 0.0
    feedback_label = None
    if args.show_robot_feedback and not args.headless:
        feedback_window = ui.Window("Robot Feedback", width=520, height=460)
        with feedback_window.frame:
            with ui.VStack(spacing=4):
                feedback_label = ui.Label("Waiting for robot feedback...", word_wrap=True)

    def _format_joint_position_snapshot(robot_name: str, snapshot: dict[str, float]) -> str:
        values = ", ".join(f"{name}={value:.5f}" for name, value in snapshot.items())
        return f"{robot_name} joint_positions: {values}"

    def _format_feedback_window_text(feedback: dict) -> str:
        if not feedback:
            return "No active robot feedback."
        position = np.asarray(feedback.get("position", []), dtype=np.float32).reshape(-1)
        quat = np.asarray(feedback.get("quat", []), dtype=np.float32).reshape(-1)
        joint_positions = feedback.get("joint_positions", {})
        joint_efforts = feedback.get("joint_efforts", {})
        joint_units = feedback.get("joint_units", {})
        joint_limit = int(args.robot_feedback_joint_limit)
        lines = [f"Robot: {feedback.get('name', 'unknown')}"]
        if position.size >= 3:
            lines.append(f"Base position xyz (m): {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}")
        if quat.size >= 4:
            lines.append(f"Base orientation quat wxyz: {quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}")
        lines.append("")
        lines.append("Non-wheel joints: position / measured effort")
        if not joint_positions:
            lines.append("  No joint positions available.")
            return "\n".join(lines)
        filtered_joint_positions = {
            name: joint_position
            for name, joint_position in joint_positions.items()
            if "wheel" not in name.lower()
        }
        if not filtered_joint_positions:
            lines.append("  No non-wheel joints available.")
            return "\n".join(lines)
        for index, (name, joint_position) in enumerate(filtered_joint_positions.items()):
            if joint_limit > 0 and index >= joint_limit:
                lines.append(f"  ... {len(filtered_joint_positions) - joint_limit} more non-wheel joints")
                break
            effort = joint_efforts.get(name)
            position_unit, effort_unit = joint_units.get(name, ("rad", "N*m"))
            effort_text = "n/a" if effort is None else f"{effort:.4f} {effort_unit}"
            lines.append(f"  {name}: pos={joint_position:.4f} {position_unit}, effort={effort_text}")
        if not joint_efforts:
            lines.append("")
            lines.append("Measured effort unavailable from this articulation/API.")
        return "\n".join(lines)

    def _update_feedback_window() -> None:
        if feedback_label is None:
            return
        feedback = dispatcher.active_robot_feedback_snapshot()
        feedback_label.text = _format_feedback_window_text(feedback)

    def _activate_robot(name: str, source: str) -> None:
        active_control_context["kind"] = "robot"
        active_control_context["path"] = None
        if dispatcher.set_active(name):
            carb.log_warn(f"{source}: active teleop controller switched to: {name}")
        else:
            carb.log_warn(f"{source}: {name} controller is unavailable. Registered controllers: {dispatcher.names()}")

    def _active_openxr_controller():
        """返回当前可接收 Quest3 双臂末端目标的机器人控制器。"""
        active_name = dispatcher.active_name if dispatcher is not None else None
        if active_name not in openxr_vr_adapters:
            return None
        return dispatcher.active_controller()

    def _select_openxr_robot(name: str, source: str) -> None:
        """同时切换机器人控制、头部相机，并清除上一机器人的 VR 零位。"""
        nonlocal openxr_vr_calibrated, openxr_vr_last_calibrate_pressed
        _activate_robot(name, source)
        if dispatcher.active_name != name:
            return
        _switch_named_camera(name, "head_top")
        openxr_vr_calibrated = False
        openxr_vr_last_calibrate_pressed = False
        openxr_vr_calibration_pose.clear()
        openxr_vr_calibration_task_pose.clear()
        openxr_head_limit_reference.clear()
        carb.log_warn(f"{source}: switched OpenXR teleop to {name}; press Y+B to calibrate this robot.")

    def _switch_to_overview_camera(source: str) -> bool:
        """恢复启动时的全场景相机，并解除机器人头部 XR 锚定。"""
        nonlocal openxr_vr_calibrated, openxr_vr_last_calibrate_pressed
        camera_path = overview_camera_state.get("path")
        if not camera_path:
            carb.log_warn(f"{source}: startup overview camera was not recorded.")
            return False
        viewport = get_active_viewport()
        if viewport is None:
            carb.log_warn(f"{source}: no active viewport found; cannot restore overview camera.")
            return False
        viewport.camera_path = camera_path
        active_camera_control["robot_name"] = None
        active_camera_control["alias"] = None
        openxr_vr_calibrated = False
        openxr_vr_last_calibrate_pressed = False
        openxr_vr_calibration_pose.clear()
        openxr_vr_calibration_task_pose.clear()
        openxr_head_limit_reference.clear()
        if args.xr_openxr:
            _request_xr_camera_anchor(camera_path)
        carb.log_warn(f"{source}: restored startup overview camera: {camera_path}")
        return True

    def _cycle_robot(source: str) -> None:
        active_control_context["kind"] = "robot"
        active_control_context["path"] = None
        active_name = dispatcher.cycle_active()
        if active_name is not None:
            carb.log_warn(f"{source}: active teleop controller switched to: {active_name}")
        else:
            carb.log_warn(f"{source}: no available teleop controller is registered.")

    def _merge_axis(keyboard_value: float, vr_value: float) -> float:
        return float(max(-1.0, min(1.0, float(keyboard_value) + float(vr_value))))

    def _apply_r1pro_initial_pose_preset() -> None:
        if r1pro_controller is None or not getattr(r1pro_controller, "available", False):
            return
        if args.r1pro_init_pose_preset == "none":
            return
        if args.r1pro_init_pose_preset != "arms_forward_level":
            return

        # 该模型零位时双臂沿 -Z 下垂；第四关节绕局部 Y 轴转 -90 度后，
        # 上臂保持竖直，小臂由 -Z 转到机体 +X，形成确定的水平前伸姿态。
        tasks = list(getattr(r1pro_controller, "arm_ik_tasks", []))
        if not tasks:
            return

        moved = False
        for task in tasks:
            side = task.get("side")
            if side not in {"left", "right"}:
                continue
            arm_target = np.array([0.0, 0.0, 0.0, -0.5 * math.pi, 0.0, 0.0, 0.0], dtype=np.float32)
            arm_target = r1pro_controller._clamp_joint_positions(task["joint_indices"], arm_target)
            task["joint_target"] = arm_target.copy()
            task["rest_joint_positions"] = arm_target.copy()
            r1pro_controller.articulation.apply_action(
                ArticulationAction(
                    joint_positions=arm_target,
                    joint_indices=task["joint_indices"],
                )
            )
            moved = True

        if getattr(r1pro_controller, "torso_indices", np.array([], dtype=np.int32)).size:
            torso_target = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
            torso_target = r1pro_controller._clamp_joint_positions(r1pro_controller.torso_indices, torso_target)
            r1pro_controller.torso_target = np.asarray(torso_target, dtype=np.float32)
            r1pro_controller.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.asarray(r1pro_controller.torso_target, dtype=np.float32),
                    joint_indices=r1pro_controller.torso_indices,
                )
            )

        if moved:
            for _ in range(80):
                for task in tasks:
                    r1pro_controller.articulation.apply_action(
                        ArticulationAction(
                            joint_positions=np.asarray(task["joint_target"], dtype=np.float32),
                            joint_indices=task["joint_indices"],
                        )
                    )
                if hasattr(r1pro_controller, "_apply_gripper_targets"):
                    r1pro_controller._apply_gripper_targets(tasks)
                my_world.step(render=True)
            for task in getattr(r1pro_controller, "arm_ik_tasks", []):
                task["gripper_target"] = np.asarray(task["gripper_open"], dtype=np.float32).copy()
                task["joint_target"] = np.asarray(
                    r1pro_controller.articulation.get_joint_positions(task["joint_indices"]),
                    dtype=np.float32,
                )
                task["rest_joint_positions"] = np.asarray(task["joint_target"], dtype=np.float32).copy()
            if hasattr(r1pro_controller, "_apply_gripper_targets"):
                r1pro_controller._apply_gripper_targets(r1pro_controller.arm_ik_tasks)
            for _ in range(2):
                my_world.step(render=True)
            r1pro_controller.sync_ik_targets()
            carb.log_warn(
                "Applied r1pro initial pose preset: arms_forward_level "
                "(双臂第四关节固定为 -90 度，上臂竖直、小臂水平前伸，并已同步 IK/夹爪目标)."
            )

    def _apply_vr_side_gripper(controller, tasks, gripper_delta: float) -> bool:
        if not tasks or gripper_delta == 0.0:
            return False
        for task in tasks:
            closed = np.asarray(task["gripper_closed"], dtype=np.float32)
            opened = np.asarray(task["gripper_open"], dtype=np.float32)
            current_target = np.asarray(task["gripper_target"], dtype=np.float32)
            direction = np.sign(opened - closed).astype(np.float32)
            if np.isscalar(direction):
                direction = np.array(1.0 if float(direction) != 0.0 else 1.0, dtype=np.float32)
            else:
                direction[direction == 0.0] = 1.0
            minimum = np.minimum(closed, opened)
            maximum = np.maximum(closed, opened)
            task["gripper_target"] = np.clip(current_target + gripper_delta * direction, minimum, maximum)
        if hasattr(controller, "_apply_gripper_targets"):
            controller._apply_gripper_targets(tasks)
        elif hasattr(controller, "ik_solver") and controller.ik_solver is not None:
            controller.ik_solver.apply_gripper_targets(tasks)
        return True

    if r1pro_controller is not None:
        try:
            _apply_r1pro_initial_pose_preset()
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Failed to apply r1pro initial pose preset: {exc}")

    def _apply_vr_bimanual_override(controller, control_state: dict[str, float]) -> bool:
        tasks = getattr(controller, "arm_ik_tasks", None)
        if not tasks:
            return False

        per_side = {
            "left": {
                "forward": float(control_state.get("left_arm_forward", 0.0)),
                "strafe": float(control_state.get("left_arm_strafe", 0.0)),
                "lift": float(control_state.get("left_arm_lift", 0.0)),
                "roll": float(control_state.get("left_arm_roll", 0.0)),
                "pitch": float(control_state.get("left_arm_pitch", 0.0)),
                "yaw": float(control_state.get("left_arm_yaw", 0.0)),
                "joint7": float(control_state.get("left_joint7", 0.0)),
                "gripper": float(control_state.get("left_gripper", 0.0)),
            },
            "right": {
                "forward": float(control_state.get("right_arm_forward", 0.0)),
                "strafe": float(control_state.get("right_arm_strafe", 0.0)),
                "lift": float(control_state.get("right_arm_lift", 0.0)),
                "roll": float(control_state.get("right_arm_roll", 0.0)),
                "pitch": float(control_state.get("right_arm_pitch", 0.0)),
                "yaw": float(control_state.get("right_arm_yaw", 0.0)),
                "joint7": float(control_state.get("right_joint7", 0.0)),
                "gripper": float(control_state.get("right_gripper", 0.0)),
            },
        }

        consumed = False
        for side, side_control in per_side.items():
            side_tasks = [task for task in tasks if task.get("side") == side]
            if not side_tasks:
                continue
            planar_active = any(
                abs(side_control[key]) > 1e-4
                for key in ("forward", "strafe", "lift", "roll", "pitch", "yaw", "joint7")
            )
            gripper_active = abs(side_control["gripper"]) > 1e-4
            if not planar_active and not gripper_active:
                continue
            consumed = True
            if getattr(controller, "name", "") == "r1pro":
                delta_xyz = np.array(
                    [side_control["forward"], side_control["strafe"], side_control["lift"]],
                    dtype=np.float32,
                ) * args.ik_speed * args.dt
                delta_rot = np.array(
                    [side_control["roll"], side_control["pitch"], side_control["yaw"]],
                    dtype=np.float32,
                ) * args.ik_rotation_speed * args.dt
                joint7_delta = side_control["joint7"] * args.ik_rotation_speed * args.dt
                controller._update_direct_arm_joint_targets(side_tasks, delta_xyz, delta_rot, joint7_delta)
                controller._hold_direct_arm_joint_targets(side_tasks)
                controller.sync_ik_targets()
            else:
                joint_delta = np.array(
                    [
                        side_control["forward"],
                        side_control["strafe"],
                        side_control["lift"],
                        side_control["roll"],
                        side_control["pitch"],
                        side_control["yaw"],
                        side_control["joint7"],
                    ],
                    dtype=np.float32,
                )
                joint_delta[:3] *= args.ik_speed * args.dt * 8.0
                joint_delta[3:6] *= args.ik_rotation_speed * args.dt * 2.0
                joint_delta[6] *= args.ik_rotation_speed * args.dt * 2.0
                controller._update_direct_arm_joint_targets(
                    side_tasks,
                    None,
                    None,
                    0.0,
                    explicit_joint_delta=joint_delta,
                )
                controller._hold_direct_arm_joint_targets(side_tasks)
            _apply_vr_side_gripper(controller, side_tasks, side_control["gripper"] * args.gripper_speed * args.dt)
        return consumed

    def _read_openxr_vr_states():
        if openxr_xr_core is None:
            return None, None
        left_device = openxr_xr_core.get_input_device("/user/hand/left")
        right_device = openxr_xr_core.get_input_device("/user/hand/right")
        left_data = _query_openxr_controller(left_device)
        right_data = _query_openxr_controller(right_device)
        if left_data.size == 0 or right_data.size == 0:
            return None, None
        left_pos, left_quat = _retarget_openxr_motion_controller_pose(left_data[0, :3], left_data[0, 3:7])
        right_pos, right_quat = _retarget_openxr_motion_controller_pose(right_data[0, :3], right_data[0, 3:7])
        left_state = VRControllerState(
            position=left_pos,
            quat_wxyz=left_quat,
            thumbstick_x=float(left_data[1, 0]),
            thumbstick_y=float(left_data[1, 1]),
            trigger=float(left_data[1, 2]),
            squeeze=float(left_data[1, 3]),
            button_0=float(left_data[1, 4]),
            button_1=float(left_data[1, 5]),
        )
        right_state = VRControllerState(
            position=right_pos,
            quat_wxyz=right_quat,
            thumbstick_x=float(right_data[1, 0]),
            thumbstick_y=float(right_data[1, 1]),
            trigger=float(right_data[1, 2]),
            squeeze=float(right_data[1, 3]),
            button_0=float(right_data[1, 4]),
            button_1=float(right_data[1, 5]),
        )
        return left_state, right_state

    def _openxr_controller_local_pose(side: str, state, base_pos, base_yaw_quat):
        """把 Quest 物理追踪坐标转换到当前机器人的作业坐标。"""
        if state is None:
            return None, None
        del base_pos, base_yaw_quat
        controller_pos = np.asarray(state.position, dtype=np.float32)
        controller_quat = np.asarray(state.quat_wxyz, dtype=np.float32)
        del side
        # OpenXR: +X 向右、+Y 向上、-Z 向前。
        # 当前 VR 里让两台车都以机体 +X 作为“视角前方/作业前方”，
        # 这样头部相机、底盘前进和双臂手柄推前的方向保持一致。
        tracking_to_robot_quat = np.array([0.5, 0.5, -0.5, -0.5], dtype=np.float32)
        local_pos = _rotate_vec_by_quat(tracking_to_robot_quat, controller_pos)
        local_quat = _normalize_quat(
            _quat_multiply(
                _quat_multiply(tracking_to_robot_quat, controller_quat),
                _quat_conjugate(tracking_to_robot_quat),
            )
        )
        return local_pos.astype(np.float32), local_quat.astype(np.float32)

    def _capture_openxr_vr_calibration(left_state, right_state) -> None:
        controller = _active_openxr_controller()
        if controller is None:
            return
        openxr_vr_calibration_pose["robot_name"] = dispatcher.active_name
        openxr_vr_calibration_pose["left_pos"] = np.asarray(left_state.position, dtype=np.float32).copy()
        openxr_vr_calibration_pose["right_pos"] = np.asarray(right_state.position, dtype=np.float32).copy()
        openxr_vr_calibration_pose["left_quat"] = _normalize_quat(left_state.quat_wxyz)
        openxr_vr_calibration_pose["right_quat"] = _normalize_quat(right_state.quat_wxyz)
        base_pos, base_quat = controller.get_base_world_pose()
        openxr_vr_calibration_pose["base_pos"] = np.asarray(base_pos, dtype=np.float32).copy()
        openxr_vr_calibration_pose["base_quat"] = _normalize_quat(base_quat)
        openxr_vr_calibration_pose["base_yaw_quat"] = _yaw_only_quat(base_quat)
        for task in controller.arm_ik_tasks:
            current_pos, current_quat = controller._get_world_pose_safe(task["body_path"])
            current_pos = np.asarray(current_pos, dtype=np.float32)
            current_quat = _normalize_quat(np.asarray(current_quat, dtype=np.float32))
            openxr_vr_calibration_task_pose[f"{task['side']}_pos"] = current_pos
            openxr_vr_calibration_task_pose[f"{task['side']}_quat"] = current_quat
            ee_local_pos = _rotate_vec_by_quat(
                _quat_conjugate(openxr_vr_calibration_pose["base_yaw_quat"]),
                current_pos - openxr_vr_calibration_pose["base_pos"],
            )
            ee_local_quat = _normalize_quat(
                _quat_multiply(
                    _quat_conjugate(openxr_vr_calibration_pose["base_yaw_quat"]),
                    current_quat,
                )
            )
            state = {"left": left_state, "right": right_state}[task["side"]]
            controller_local_pos, controller_local_quat = _openxr_controller_local_pose(
                task["side"],
                state,
                openxr_vr_calibration_pose["base_pos"],
                openxr_vr_calibration_pose["base_yaw_quat"],
            )
            openxr_vr_calibration_task_pose[f"{task['side']}_local_pos"] = ee_local_pos
            openxr_vr_calibration_task_pose[f"{task['side']}_local_quat"] = ee_local_quat
            openxr_vr_calibration_task_pose[f"{task['side']}_controller_local_pos"] = controller_local_pos
            openxr_vr_calibration_task_pose[f"{task['side']}_controller_local_quat"] = controller_local_quat

    def _refresh_openxr_vr_calibration_for_side(side: str, state) -> bool:
        """只刷新单侧 VR 零位，避免开底盘的同侧手柄把机械臂目标拖丢。"""
        if not openxr_vr_calibrated:
            return False
        controller = _active_openxr_controller()
        if controller is None or state is None:
            return False
        if openxr_vr_calibration_pose.get("robot_name") != dispatcher.active_name:
            return False
        task = next((item for item in controller.arm_ik_tasks if item.get("side") == side), None)
        if task is None:
            return False
        base_pos_now, base_quat_now = controller.get_base_world_pose()
        base_pos_now = np.asarray(base_pos_now, dtype=np.float32)
        base_yaw_quat_now = _yaw_only_quat(base_quat_now)
        current_pos, current_quat = controller._get_world_pose_safe(task["body_path"])
        current_pos = np.asarray(current_pos, dtype=np.float32)
        current_quat = _normalize_quat(np.asarray(current_quat, dtype=np.float32))
        ee_local_pos = _rotate_vec_by_quat(_quat_conjugate(base_yaw_quat_now), current_pos - base_pos_now)
        ee_local_quat = _normalize_quat(_quat_multiply(_quat_conjugate(base_yaw_quat_now), current_quat))
        controller_local_pos, controller_local_quat = _openxr_controller_local_pose(
            side,
            state,
            base_pos_now,
            base_yaw_quat_now,
        )
        openxr_vr_calibration_pose["base_pos"] = base_pos_now.copy()
        openxr_vr_calibration_pose["base_quat"] = _normalize_quat(base_quat_now)
        openxr_vr_calibration_pose["base_yaw_quat"] = base_yaw_quat_now.copy()
        openxr_vr_calibration_task_pose[f"{side}_pos"] = current_pos
        openxr_vr_calibration_task_pose[f"{side}_quat"] = current_quat
        openxr_vr_calibration_task_pose[f"{side}_local_pos"] = ee_local_pos
        openxr_vr_calibration_task_pose[f"{side}_local_quat"] = ee_local_quat
        openxr_vr_calibration_task_pose[f"{side}_controller_local_pos"] = controller_local_pos
        openxr_vr_calibration_task_pose[f"{side}_controller_local_quat"] = controller_local_quat
        return True

    def _get_active_xr_anchor_target_pose():
        """读取当前遥操作机器人的头部相机，不依赖桌面 viewport 状态。"""
        robot_name = dispatcher.active_name if dispatcher is not None else None
        camera_path = robot_camera_aliases.get(robot_name, {}).get("head_top")
        if not camera_path:
            return None, None
        camera_prim = stage.GetPrimAtPath(camera_path)
        if not camera_prim.IsValid():
            return None, None
        world_transform = UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = world_transform.ExtractTranslation()
        quat = world_transform.ExtractRotationQuat()
        return (
            np.array([float(position[0]), float(position[1]), float(position[2])], dtype=np.float32),
            _normalize_quat(
                np.array(
                    [
                        float(quat.GetReal()),
                        float(quat.GetImaginary()[0]),
                        float(quat.GetImaginary()[1]),
                        float(quat.GetImaginary()[2]),
                    ],
                    dtype=np.float32,
                )
            ),
        )

    def _match_openxr_head_device_to_world_pose(target_pos, target_quat_wxyz) -> bool:
        """把物理头显设备锚定到机器人头部，同时保留头显自身的转动。"""
        if openxr_xr_core is None:
            return False
        target_pos = np.asarray(target_pos, dtype=np.float32).reshape(3)
        target_quat = _normalize_quat(target_quat_wxyz)
        device_pose = Gf.Matrix4d(1.0)
        device_pose.SetTranslateOnly(Gf.Vec3d(float(target_pos[0]), float(target_pos[1]), float(target_pos[2])))
        device_pose.SetRotateOnly(
            Gf.Quatd(
                float(target_quat[0]),
                Gf.Vec3d(float(target_quat[1]), float(target_quat[2]), float(target_quat[3])),
            )
        )
        try:
            profile = openxr_head_device_alignment.get("profile")
            display_device = openxr_head_device_alignment.get("device")
            if profile is None or display_device is None:
                profile = openxr_xr_core.get_profile("vr")
                display_device = profile.get_device("xrdisplaydevice0") if profile is not None else None
                if profile is None or display_device is None:
                    return False
                openxr_head_device_alignment["profile"] = profile
                openxr_head_device_alignment["device"] = display_device
            profile.set_physical_world_to_world_anchor_transform_to_match_xr_device(device_pose, display_device)
            return True
        except Exception as exc:  # noqa: BLE001
            if not openxr_head_device_alignment.get("warned"):
                carb.log_warn(f"Failed to align physical XR head device: {exc}")
                openxr_head_device_alignment["warned"] = True
            return False

    def _capture_openxr_head_limit_reference() -> None:
        if openxr_xr_core is None:
            return
        target_pos, target_quat = _get_active_xr_anchor_target_pose()
        if target_pos is None or target_quat is None:
            return
        head_device = openxr_xr_core.get_input_device("/user/head")
        if head_device is None:
            return
        hmd = head_device.get_pose("")
        position = np.array(list(hmd.ExtractTranslation()), dtype=np.float32)
        quat_gf = hmd.ExtractRotationQuat()
        quat = _normalize_quat(
            np.array(
                [
                    float(quat_gf.GetReal()),
                    float(quat_gf.GetImaginary()[0]),
                    float(quat_gf.GetImaginary()[1]),
                    float(quat_gf.GetImaginary()[2]),
                ],
                dtype=np.float32,
            )
        )
        openxr_head_limit_reference["local_pos"] = position.copy()
        openxr_head_limit_reference["local_quat"] = quat.copy()
        if not _match_openxr_head_device_to_world_pose(target_pos, target_quat):
            view_pose = Gf.Matrix4d(1.0)
            view_pose.SetTranslateOnly(Gf.Vec3d(float(target_pos[0]), float(target_pos[1]), float(target_pos[2])))
            view_pose.SetRotateOnly(
                Gf.Quatd(
                    float(target_quat[0]),
                    Gf.Vec3d(float(target_quat[1]), float(target_quat[2]), float(target_quat[3])),
                )
            )
            openxr_xr_core.schedule_set_camera(view_pose)

    def _update_openxr_head_anchor_with_limits() -> bool:
        if not args.xr_openxr or openxr_xr_core is None:
            return False
        target_pos, target_quat = _get_active_xr_anchor_target_pose()
        if target_pos is None or target_quat is None:
            return False
        if "local_pos" not in openxr_head_limit_reference or "local_quat" not in openxr_head_limit_reference:
            _capture_openxr_head_limit_reference()
            return False
        head_device = openxr_xr_core.get_input_device("/user/head")
        if head_device is None:
            return False
        # 物理设备 pose 不受 schedule_set_camera 影响，适合计算真实转头增量。
        hmd = head_device.get_pose("")
        head_quat_gf = hmd.ExtractRotationQuat()
        head_physical_quat = _normalize_quat(
            np.array(
                [
                    float(head_quat_gf.GetReal()),
                    float(head_quat_gf.GetImaginary()[0]),
                    float(head_quat_gf.GetImaginary()[1]),
                    float(head_quat_gf.GetImaginary()[2]),
                ],
                dtype=np.float32,
            )
        )

        reference_physical_quat = _normalize_quat(openxr_head_limit_reference["local_quat"])
        head_rotation_delta = _normalize_quat(
            _quat_multiply(_quat_conjugate(reference_physical_quat), head_physical_quat)
        )
        desired_head_quat = _normalize_quat(_quat_multiply(target_quat, head_rotation_delta))

        # 物理设备锚点跟随 r1pro 头部；真实头部平移不再带走视点，转头仍按相对姿态生效。
        if not _match_openxr_head_device_to_world_pose(target_pos, desired_head_quat):
            view_pose = Gf.Matrix4d(1.0)
            view_pose.SetTranslateOnly(Gf.Vec3d(float(target_pos[0]), float(target_pos[1]), float(target_pos[2])))
            view_pose.SetRotateOnly(
                Gf.Quatd(
                    float(desired_head_quat[0]),
                    Gf.Vec3d(
                        float(desired_head_quat[1]),
                        float(desired_head_quat[2]),
                        float(desired_head_quat[3]),
                    ),
                )
            )
            openxr_xr_core.schedule_set_camera(view_pose)
        return True

    def _apply_openxr_vr_calibration(raw_output, left_state, right_state):
        if not openxr_vr_calibrated:
            return raw_output
        controller = _active_openxr_controller()
        if controller is None:
            return raw_output
        if openxr_vr_calibration_pose.get("robot_name") != dispatcher.active_name:
            return raw_output
        state_by_side = {"left": left_state, "right": right_state}
        base_pos_now, base_quat_now = controller.get_base_world_pose()
        base_pos_now = np.asarray(base_pos_now, dtype=np.float32)
        base_yaw_quat_now = _yaw_only_quat(base_quat_now)
        for target in (raw_output.command.left, raw_output.command.right):
            side = target.side
            state = state_by_side[side]
            if state is None:
                continue
            reference_local_pos = openxr_vr_calibration_task_pose[f"{side}_local_pos"]
            reference_local_quat = openxr_vr_calibration_task_pose[f"{side}_local_quat"]
            controller_reference_pos = openxr_vr_calibration_task_pose[f"{side}_controller_local_pos"]
            controller_reference_quat = openxr_vr_calibration_task_pose[f"{side}_controller_local_quat"]

            controller_local_pos_now, controller_local_quat_now = _openxr_controller_local_pose(
                side,
                state,
                base_pos_now,
                base_yaw_quat_now,
            )

            # 平移和旋转分别按标定零位求相对量。这样手柄原地旋转不会因为
            # controller->EE 的平移外参产生额外位移，底盘移动时目标也保持在机器人坐标系内。
            controller_delta_pos = np.asarray(controller_local_pos_now, dtype=np.float32) - np.asarray(
                controller_reference_pos, dtype=np.float32
            )
            controller_axis_scale = np.array(
                [args.openxr_vr_forward_scale, 1.0, args.openxr_vr_lift_scale],
                dtype=np.float32,
            )
            target_local_pos = np.asarray(reference_local_pos, dtype=np.float32) + (
                controller_delta_pos * float(args.openxr_vr_position_scale) * controller_axis_scale
            )
            target_delta_quat = _quat_multiply(
                np.asarray(controller_local_quat_now, dtype=np.float32),
                _quat_conjugate(np.asarray(controller_reference_quat, dtype=np.float32)),
            )
            target_delta_quat = _scale_relative_quat(target_delta_quat, float(args.openxr_vr_rotation_scale))
            target_local_quat = _normalize_quat(
                _quat_multiply(target_delta_quat, np.asarray(reference_local_quat, dtype=np.float32))
            )
            target.position = base_pos_now + _rotate_vec_by_quat(base_yaw_quat_now, target_local_pos)
            target.quat_wxyz = _normalize_quat(_quat_multiply(base_yaw_quat_now, target_local_quat))
        raw_output.ee_action = np.concatenate(
            [
                raw_output.command.left.position,
                raw_output.command.left.quat_wxyz,
                raw_output.command.right.position,
                raw_output.command.right.quat_wxyz,
            ]
        ).astype(np.float32)
        return raw_output

    def _apply_openxr_vr_gripper_hold(raw_output, left_state, right_state) -> None:
        state_by_side = {"left": left_state, "right": right_state}
        for target in (raw_output.command.left, raw_output.command.right):
            state = state_by_side[target.side]
            if state is None:
                continue
            gripping = float(state.trigger) > 0.35
            target.gripper = -1.0 if gripping else 1.0
        raw_output.gripper_action = np.array(
            [float(raw_output.command.left.gripper), float(raw_output.command.right.gripper)],
            dtype=np.float32,
        )
        raw_output.ee_action = np.concatenate(
            [
                raw_output.command.left.position,
                raw_output.command.left.quat_wxyz,
                raw_output.command.right.position,
                raw_output.command.right.quat_wxyz,
            ]
        ).astype(np.float32)

    def _apply_openxr_vr_torso(left_state, right_state) -> bool:
        if dispatcher.active_name != "r1pro" or r1pro_controller is None or left_state is None or right_state is None:
            return False
        torso_axis_1 = float(left_state.button_1 > 0.5) - float(left_state.button_0 > 0.5)
        torso_axis_2 = float(right_state.button_1 > 0.5) - float(right_state.button_0 > 0.5)
        if abs(torso_axis_1) < 1e-4 and abs(torso_axis_2) < 1e-4:
            return False
        torso_delta = np.zeros(4, dtype=np.float32)
        torso_delta[0] = torso_axis_1 * float(args.openxr_vr_torso_speed) * float(args.dt)
        torso_delta[1] = torso_axis_2 * float(args.openxr_vr_torso_speed) * float(args.dt)
        r1pro_controller._apply_torso_target(torso_delta)
        return True

    def _build_openxr_vr_base_control(left_state, right_state) -> dict[str, float]:
        if left_state is None or right_state is None:
            return {}

        def _axis(value: float) -> float:
            value = float(np.clip(value, -1.0, 1.0))
            return 0.0 if abs(value) < float(args.openxr_vr_stick_deadband) else value

        left_stick_x = _axis(left_state.thumbstick_x)
        left_stick_y = _axis(left_state.thumbstick_y)
        right_stick_x = _axis(right_state.thumbstick_x)

        forward = left_stick_y * float(args.openxr_vr_base_speed)
        if dispatcher.active_name == "ranger_arm":
            strafe = 0.0
            yaw = -left_stick_x * float(args.openxr_vr_base_yaw_speed)
        else:
            strafe = -left_stick_x * float(args.openxr_vr_base_speed)
            yaw = -right_stick_x * float(args.openxr_vr_base_yaw_speed)

        if abs(forward) < 1e-4 and abs(strafe) < 1e-4 and abs(yaw) < 1e-4:
            return {}

        return {
            "base_forward": float(np.clip(forward, -1.0, 1.0)),
            "base_strafe": float(np.clip(strafe, -1.0, 1.0)),
            "base_yaw": float(np.clip(yaw, -1.0, 1.0)),
        }

    vr_switch_server = None
    if args.enable_vr_switch_udp:
        try:
            vr_switch_server = RobotSwitchCommandServer(
                args.vr_switch_udp_host,
                args.vr_switch_udp_port,
                carb,
            )
            vr_switch_server.start()
            carb.log_info(
                "VR robot switch UDP listener ready on "
                f"{args.vr_switch_udp_host}:{args.vr_switch_udp_port} "
                "(commands: cycle, ranger_arm, r1pro)."
            )
        except OSError as exc:
            carb.log_warn(
                "Failed to start VR robot switch UDP listener on "
                f"{args.vr_switch_udp_host}:{args.vr_switch_udp_port}: {exc}"
            )

    initial_robot_name = "r1pro" if r1pro_controller is not None else "ranger_arm"
    if dispatcher.set_active(initial_robot_name):
        carb.log_info(f"Active teleop controller switched to: {initial_robot_name}")
    target_reach = None
    time_code = Usd.TimeCode.Default()
    cfg = TeleopConfig(speed=args.speed, turn_rate=args.turn_rate, lift_rate=args.lift_rate)

    def _keyboard_inputs(*names):
        return [key for name in names if (key := getattr(carb.input.KeyboardInput, name, None)) is not None]

    keypad_key_1 = getattr(carb.input.KeyboardInput, "NUMPAD_1", None)
    keypad_key_2 = getattr(carb.input.KeyboardInput, "NUMPAD_2", None)

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
        carb.input.KeyboardInput.F3,
        carb.input.KeyboardInput.F4,
        carb.input.KeyboardInput.F6,
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
    if keypad_key_1 is not None:
        keys.append(keypad_key_1)
    if keypad_key_2 is not None:
        keys.append(keypad_key_2)
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

    if args.enable_openxr_r1pro_vr:
        carb.log_warn("OpenXR VR ready; startup keeps the overview camera. Press F3/F4 to enter a robot head view.")

    if dispatcher is not None and dispatcher.active_name is not None:
        carb.log_info(
            f"Teleop ready. Active controller: {dispatcher.active_name}. "
            "F1 restores overview; 1/2 switch robot; W/S drive forward-back; A/D steer base; "
            "TAB switches left/right/both; I/K J/L U/O T/G F/H R/Y 7/8 drive 7-DOF arm joints; "
            "5/6 torso yaw; "
            "M/N gripper; F6 switch active robot head camera; F3/F4 force robot head cameras; "
            "F9/F10 switch drone cameras; "
            "3/4 select drone teleop, F2 returns to robot teleop. "
            f"External VR switch UDP: {args.vr_switch_udp_host}:{args.vr_switch_udp_port}."
        )
        carb.log_warn(
            "Keyboard robot switch: 主键盘 1= ranger_arm, 2= r1pro; "
            "若有小键盘，也支持 NUMPAD_1 / NUMPAD_2。"
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

            external_command = vr_switch_server.pop_command() if vr_switch_server is not None else None
            while external_command is not None:
                if external_command == "cycle":
                    _cycle_robot("VR switch")
                elif external_command == "ranger_arm":
                    _activate_robot("ranger_arm", "VR switch")
                elif external_command == "r1pro":
                    _activate_robot("r1pro", "VR switch")
                elif external_command == "return_robot":
                    active_control_context["kind"] = "robot"
                    active_control_context["path"] = None
                    carb.log_warn(f"VR switch: control returned to robot teleop: {dispatcher.active_name}")
                external_command = vr_switch_server.pop_command() if vr_switch_server is not None else None

            if keyboard.consume_pressed(carb.input.KeyboardInput.F1):
                _switch_to_overview_camera("Keyboard F1")
            key_1_pressed = keyboard.consume_pressed(carb.input.KeyboardInput.KEY_1)
            if keypad_key_1 is not None:
                key_1_pressed = keyboard.consume_pressed(keypad_key_1) or key_1_pressed
            if key_1_pressed:
                _activate_robot("ranger_arm", "Keyboard")
            key_2_pressed = keyboard.consume_pressed(carb.input.KeyboardInput.KEY_2)
            if keypad_key_2 is not None:
                key_2_pressed = keyboard.consume_pressed(keypad_key_2) or key_2_pressed
            if key_2_pressed:
                _activate_robot("r1pro", "Keyboard")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F2):
                active_control_context["kind"] = "robot"
                active_control_context["path"] = None
                carb.log_warn(f"Keyboard control returned to robot teleop: {dispatcher.active_name}")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F6):
                _switch_robot_camera("head_top")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F9):
                _switch_named_camera("cf2x", "chase")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F10):
                _switch_named_camera("cf2x_01", "chase")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F3):
                _select_openxr_robot("ranger_arm", "Keyboard F3")
            if keyboard.consume_pressed(carb.input.KeyboardInput.F4):
                _select_openxr_robot("r1pro", "Keyboard F4")
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

            vr_control = vr_switch_server.get_control_state() if vr_switch_server is not None else {}
            openxr_vr_consumed = False
            openxr_output = None
            openxr_vr_base_control = {}
            openxr_left_state = None
            openxr_right_state = None
            openxr_vr_active_adapter = None
            active_openxr_controller = _active_openxr_controller()
            if (
                openxr_vr_bridge is not None
                and active_openxr_controller is not None
                and dispatcher.active_name in openxr_vr_adapters
            ):
                openxr_vr_active_adapter = openxr_vr_adapters[dispatcher.active_name]
                openxr_left_state, openxr_right_state = _read_openxr_vr_states()
                if openxr_left_state is not None and openxr_right_state is not None:
                    calibrate_pressed = bool(
                        openxr_left_state.button_1 > 0.5 and openxr_right_state.button_1 > 0.5
                    )
                    if calibrate_pressed and not openxr_vr_last_calibrate_pressed:
                        active_openxr_controller.sync_ik_targets()
                        _capture_openxr_vr_calibration(openxr_left_state, openxr_right_state)
                        _capture_openxr_head_limit_reference()
                        openxr_vr_calibrated = True
                        base_yaw_quat = openxr_vr_calibration_pose.get("base_yaw_quat")
                        carb.log_warn(
                            f"OpenXR VR 标定完成：Quest3 双手柄已按当前 {dispatcher.active_name} 双末端姿态建立零位对齐。"
                            f"base_yaw_quat={base_yaw_quat.tolist() if base_yaw_quat is not None else 'n/a'}"
                        )
                    openxr_vr_last_calibrate_pressed = calibrate_pressed
                    if openxr_vr_calibrated:
                        _update_openxr_head_anchor_with_limits()
                        openxr_vr_base_control = _build_openxr_vr_base_control(openxr_left_state, openxr_right_state)
                        if (
                            dispatcher.active_name == "ranger_arm"
                            and (
                                abs(openxr_vr_base_control.get("base_forward", 0.0)) > 1e-4
                                or abs(openxr_vr_base_control.get("base_yaw", 0.0)) > 1e-4
                            )
                        ):
                            _refresh_openxr_vr_calibration_for_side("left", openxr_left_state)
                        openxr_output = openxr_vr_bridge.build(openxr_left_state, openxr_right_state)
                        openxr_output = _apply_openxr_vr_calibration(
                            openxr_output,
                            openxr_left_state,
                            openxr_right_state,
                        )
                        # 标定零位必须原样保留。旧肩部裁剪会在手柄不动时也改写目标，
                        # 导致双臂持续向头顶和关节限位运动；安全边界由 IK 误差限幅和关节限位负责。
                        _apply_openxr_vr_gripper_hold(openxr_output, openxr_left_state, openxr_right_state)
                        openxr_vr_consumed = _apply_openxr_vr_torso(openxr_left_state, openxr_right_state) or openxr_vr_consumed

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
            if vr_control:
                forward = _merge_axis(forward, vr_control.get("base_forward", 0.0))
                strafe = _merge_axis(strafe, vr_control.get("base_strafe", 0.0))
                lift = _merge_axis(lift, vr_control.get("base_lift", 0.0))
                yaw = _merge_axis(yaw, vr_control.get("base_yaw", 0.0))
            if openxr_vr_base_control and (r1pro_is_active or ranger_arm_is_active):
                forward = _merge_axis(forward, openxr_vr_base_control.get("base_forward", 0.0))
                if r1pro_is_active:
                    strafe = _merge_axis(strafe, openxr_vr_base_control.get("base_strafe", 0.0))
                yaw = _merge_axis(yaw, openxr_vr_base_control.get("base_yaw", 0.0))
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
                if vr_control:
                    ik_forward = _merge_axis(ik_forward, vr_control.get("ik_forward", 0.0))
                    ik_strafe = _merge_axis(ik_strafe, vr_control.get("ik_strafe", 0.0))
                    ik_lift = _merge_axis(ik_lift, vr_control.get("ik_lift", 0.0))
                    arm_roll = _merge_axis(arm_roll, vr_control.get("arm_roll", 0.0))
                    arm_pitch = _merge_axis(arm_pitch, vr_control.get("arm_pitch", 0.0))
                    arm_yaw = _merge_axis(arm_yaw, vr_control.get("arm_yaw", 0.0))
                    gripper_delta += float(vr_control.get("gripper", 0.0)) * args.gripper_speed * args.dt
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
                    if vr_control:
                        joint7_axis = _merge_axis(joint7_axis, vr_control.get("joint7", 0.0))
                    ranger_joint_delta[6] = joint7_axis * args.ik_rotation_speed * args.dt * 2.0
                elif r1pro_is_active:
                    joint7_axis = float(
                        keyboard.pressed(carb.input.KeyboardInput.KEY_7)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.KEY_7)
                    ) - float(
                        keyboard.pressed(carb.input.KeyboardInput.KEY_8)
                        or keyboard.poll_pressed(carb.input.KeyboardInput.KEY_8)
                    )
                    if vr_control:
                        joint7_axis = _merge_axis(joint7_axis, vr_control.get("joint7", 0.0))
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
                    if vr_control:
                        torso_1 = _merge_axis(torso_1, vr_control.get("torso_1", 0.0))
                        torso_2 = _merge_axis(torso_2, vr_control.get("torso_2", 0.0))
                        torso_3 = _merge_axis(torso_3, vr_control.get("torso_3", 0.0))
                        torso_4 = _merge_axis(torso_4, vr_control.get("torso_4", 0.0))
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
            vr_bimanual_consumed = False
            if active_controller is not None and vr_control:
                vr_bimanual_consumed = _apply_vr_bimanual_override(active_controller, vr_control)
            if (
                active_controller is None
                and not base_has_input
                and not arm_has_input
                and not vr_bimanual_consumed
                and not openxr_vr_consumed
            ):
                continue

            if timeline.is_stopped():
                timeline.play()
            should_dispatch = bool(
                base_has_input
                or arm_has_input
                or active_controller is not None
            )
            if (vr_bimanual_consumed or openxr_vr_consumed) and not base_has_input and not arm_has_input:
                should_dispatch = False
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
            # VR IK 必须最后下发，避免底盘或键盘命令触发的 direct-joint hold
            # 在同一帧覆盖双手柄生成的末端目标。
            if openxr_output is not None and openxr_vr_active_adapter is not None:
                openxr_vr_consumed = bool(openxr_vr_active_adapter.apply(openxr_output)) or openxr_vr_consumed
            if should_dispatch:
                if args.log_joint_positions and (base_has_input or arm_has_input):
                    now = time.time()
                    if now - last_joint_position_log_time >= max(args.joint_position_log_interval, 0.0):
                        snapshot = dispatcher.active_joint_position_snapshot()
                        if snapshot:
                            robot_name = dispatcher.active_name or "unknown"
                            message = _format_joint_position_snapshot(robot_name, snapshot)
                            carb.log_info(message)
                            print(message, flush=True)
                        last_joint_position_log_time = now
                if feedback_label is not None:
                    now = time.time()
                    if now - last_feedback_window_update_time >= 0.2:
                        _update_feedback_window()
                        last_feedback_window_update_time = now
                _update_robot_camera_rigs()
                _apply_drone_trajectories()
    except Exception:
        carb.log_error("Unhandled exception in teleop loop.")
        import traceback  # noqa: WPS433

        carb.log_error(traceback.format_exc())
    finally:
        if vr_switch_server is not None:
            vr_switch_server.stop()
        keyboard.disconnect()
        sim_app.close()


if __name__ == "__main__":
    main()
