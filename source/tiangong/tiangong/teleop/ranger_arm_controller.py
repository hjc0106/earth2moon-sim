"""Ranger Arm 遥操作控制器。

本控制器负责 Ranger Arm 底盘轮/转向关节和机械臂目标选择；
具体 IK 求解委托给 ranger_arm_ik.RangerArmIKSolver。
"""

from __future__ import annotations

import numpy as np
from isaacsim.core.utils.types import ArticulationAction

from .ranger_arm_ik import RangerArmIKSolver


class RangerArmTeleopController:
    """把统一遥操作命令转换成 Ranger Arm 的底盘动作和双臂 IK 动作。"""

    name = "ranger_arm"

    def __init__(
        self,
        articulation,
        carb,
        root_path: str,
        get_world_pose,
        wheel_control_paths,
        wheel_indices,
        steering_control_paths,
        steering_indices,
        args,
    ):
        self.articulation = articulation
        self.carb = carb
        self.root_path = root_path
        self.get_world_pose = get_world_pose
        self.wheel_control_paths = wheel_control_paths
        self.wheel_indices = wheel_indices
        self.steering_control_paths = steering_control_paths
        self.steering_indices = steering_indices
        self.args = args
        self._base_motion_warned = False
        self.available = articulation is not None
        self.active_target_mode = args.ik_arm
        self.arm_ik_enabled = False
        self.arm_ik_tasks = []
        self.ik_solver = None
        if self.available:
            self._configure_base_drives()
        if self.available and args.enable_arm_ik:
            self._initialize_ik()
        if self.available:
            message = (
                f"Ranger Arm base drive ready: wheel_dofs={self.wheel_indices.size}, "
                f"steer_dofs={self.steering_indices.size}, "
                f"wheel_names={self._joint_names(self.wheel_control_paths)}, "
                f"steer_names={self._joint_names(self.steering_control_paths)}, "
                f"active_steer_names={self._joint_names(self._active_steering_paths())}."
            )
            self.carb.log_warn(message)
            print(message, flush=True)
            if self.arm_ik_enabled:
                self.carb.log_warn("Ranger Arm arm teleop uses direct 7-DOF joint targets; IK solver interface remains available.")
                for task in self.arm_ik_tasks:
                    self.carb.log_warn(f"Ranger Arm {task['side']} arm joints: {task['joint_names']}")
            else:
                self.carb.log_warn("Ranger Arm arm teleop is disabled or failed to initialize; only base control is active.")

    def _joint_names(self, paths) -> list[str]:
        """把关节路径压缩成名字，便于启动日志核对 DOF 分类。"""
        return [str(path).rsplit("/", 1)[-1] for path in paths]

    def _configure_base_drives(self):
        """配置 Ranger Arm 轮子和转向 DOF，确保速度/位置 action 能生效。"""
        try:
            if self.wheel_indices.size:
                self.articulation._articulation_view.set_gains(
                    kps=np.zeros((1, self.wheel_indices.size), dtype=np.float32),
                    kds=np.full((1, self.wheel_indices.size), 250.0, dtype=np.float32),
                    joint_indices=self.wheel_indices,
                )
                self.articulation._articulation_view.set_max_efforts(
                    values=np.full((1, self.wheel_indices.size), 500.0, dtype=np.float32),
                    joint_indices=self.wheel_indices,
                )
            if self.steering_indices.size:
                self.articulation._articulation_view.set_gains(
                    kps=np.full((1, self.steering_indices.size), 1500.0, dtype=np.float32),
                    kds=np.full((1, self.steering_indices.size), 150.0, dtype=np.float32),
                    joint_indices=self.steering_indices,
                )
                self.articulation._articulation_view.set_max_efforts(
                    values=np.full((1, self.steering_indices.size), 300.0, dtype=np.float32),
                    joint_indices=self.steering_indices,
                )
        except Exception as exc:
            self.carb.log_warn(f"Could not configure Ranger Arm base drives: {exc}")

    def _initialize_ik(self):
        """初始化独立 IK 求解器，并缓存左右臂 task。"""
        self.ik_solver = RangerArmIKSolver(self.articulation, self.root_path, self.get_world_pose, self.args)
        self.arm_ik_enabled = self.ik_solver.initialize()
        self.arm_ik_tasks = self.ik_solver.tasks
        self._configure_arm_drives()

    def _configure_arm_drives(self):
        """给双臂 7 自由度关节配置位置驱动参数，用于直接关节控制。"""
        if not self.arm_ik_tasks:
            return
        arm_indices = np.unique(np.concatenate([task["joint_indices"] for task in self.arm_ik_tasks])).astype(np.int32)
        try:
            self.articulation._articulation_view.set_gains(
                kps=np.full((1, arm_indices.size), 12000.0, dtype=np.float32),
                kds=np.full((1, arm_indices.size), 1200.0, dtype=np.float32),
                joint_indices=arm_indices,
            )
            self.articulation._articulation_view.set_max_efforts(
                values=np.full((1, arm_indices.size), 2000.0, dtype=np.float32),
                joint_indices=arm_indices,
            )
        except Exception as exc:
            self.carb.log_warn(f"Could not configure Ranger Arm arm drives: {exc}")

    def _get_world_pose_safe(self, prim_path: str):
        """优先用 fabric 获取 prim 世界位姿，失败时回退普通 USD 查询。"""
        try:
            return self.get_world_pose(prim_path, fabric=True)
        except Exception:
            return self.get_world_pose(prim_path)

    def _to_pose_arrays(self, position, quat):
        """把 IsaacSim/tensor 返回值规整成 numpy 位姿数组。"""
        return np.asarray(position, dtype=np.float32).reshape(-1)[:3], np.asarray(quat, dtype=np.float32).reshape(-1)[:4]

    def get_base_world_pose(self):
        """读取 Ranger Arm 物理 articulation 的实时底座位姿。

        目标区域控制必须读取 PhysX 中的 live pose；USD 根 prim 的 xform
        通常是加载时的 authored 值，轮子带动车体运动后不会同步变化。
        """
        try:
            position, quat = self.articulation.get_world_pose()
            return self._to_pose_arrays(position, quat)
        except Exception:
            pass
        try:
            positions, quats = self.articulation._articulation_view.get_world_poses()
            return self._to_pose_arrays(np.asarray(positions)[0], np.asarray(quats)[0])
        except Exception:
            pass
        for body_name in ("base_footprint", "base_link", "chassis_link", "body"):
            try:
                return self._to_pose_arrays(*self._get_world_pose_safe(f"{self.root_path}/{body_name}"))
            except Exception:
                continue
        return self._to_pose_arrays(*self._get_world_pose_safe(self.root_path))

    def cycle_target_mode(self) -> str:
        """在左臂、右臂、双臂之间循环切换当前 IK 目标。"""
        modes = ("left", "right", "both")
        self.active_target_mode = modes[(modes.index(self.active_target_mode) + 1) % len(modes)]
        self.sync_ik_targets()
        return self.active_target_mode

    def _active_tasks(self):
        """返回当前控制模式下需要接收机械臂命令的 IK task。"""
        if self.active_target_mode == "both":
            return list(self.arm_ik_tasks)
        return [task for task in self.arm_ik_tasks if task["side"] == self.active_target_mode]

    def sync_ik_targets(self):
        """把所有 IK 目标重置为当前末端实际位姿。"""
        if self.ik_solver is not None:
            self.ik_solver.sync_targets()

    def _wheel_x_from_name(self, joint_path: str) -> float:
        """按关节名估算该轮位于底盘机械臂端还是另一端。"""
        name = joint_path.lower()
        if "rear" in name or "back" in name or "rl" in name or "rr" in name:
            return -1.0
        if "front" in name or "fl" in name or "fr" in name:
            return 1.0
        return 0.0

    def _yaw_from_quat(self, quat) -> float:
        """从 wxyz 四元数提取底座 yaw。"""
        quat = np.asarray(quat, dtype=np.float32).reshape(-1)
        if quat.size < 4:
            return 0.0
        w, x, y, z = [float(value) for value in quat[:4]]
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def _arm_end_sign(self) -> float:
        """判断机械臂更靠近底盘本地 +X 还是 -X 一端。"""
        offsets = []
        try:
            base_position, base_quat = self.get_base_world_pose()
        except Exception:
            return -1.0
        yaw = self._yaw_from_quat(base_quat)
        cos_yaw = float(np.cos(yaw))
        sin_yaw = float(np.sin(yaw))
        for task in self.arm_ik_tasks:
            try:
                ee_position, _ = self._get_world_pose_safe(task["body_path"])
            except Exception:
                continue
            delta_xy = np.asarray(ee_position, dtype=np.float32)[:2] - np.asarray(base_position, dtype=np.float32)[:2]
            local_x = cos_yaw * float(delta_xy[0]) + sin_yaw * float(delta_xy[1])
            if abs(local_x) > 0.05:
                offsets.append(local_x)
        if not offsets:
            return -1.0
        return 1.0 if float(np.mean(offsets)) >= 0.0 else -1.0

    def _active_steering_paths(self):
        """返回真正可转向的两个 Ranger Arm 转向关节路径。"""
        paths = list(self.steering_control_paths)
        if len(paths) <= 2:
            return paths
        mode = getattr(self.args, "ranger_active_steer", "arm")
        if mode == "all":
            return paths
        if mode == "front":
            selected = [path for path in paths if self._wheel_x_from_name(path) > 0.0]
        elif mode == "rear":
            selected = [path for path in paths if self._wheel_x_from_name(path) < 0.0]
        else:
            arm_sign = self._arm_end_sign()
            selected = [path for path in paths if self._wheel_x_from_name(path) * arm_sign > 0.0]
        if len(selected) >= 2:
            return selected[:2]
        if selected:
            return selected
        return paths[:2]

    def _side_sign_from_name(self, joint_path: str) -> float:
        """读取左右轮速度符号修正。"""
        name = joint_path.lower()
        if "left" in name or "lf" in name or "fl" in name or "rl" in name:
            return float(self.args.left_wheel_sign)
        if "right" in name or "rf" in name or "fr" in name or "rr" in name:
            return float(self.args.right_wheel_sign)
        return 1.0

    def _steer_sign_from_name(self, joint_path: str) -> float:
        """读取转向关节符号修正。"""
        name = joint_path.lower()
        steer_sign = 1.0
        if "front" in name or "fl" in name or "fr" in name:
            steer_sign *= self.args.front_steer_sign
        if "rear" in name or "back" in name or "rl" in name or "rr" in name:
            steer_sign *= self.args.rear_steer_sign
        if "left" in name or "lf" in name or "fl" in name or "rl" in name:
            steer_sign *= self.args.left_steer_sign
        if "right" in name or "rf" in name or "fr" in name or "rr" in name:
            steer_sign *= self.args.right_steer_sign
        return float(steer_sign)

    def _steer_pair_sign(self, active_steering) -> float:
        """同一对转向轮使用同一个符号，保证两个转向命令一致。"""
        if not active_steering:
            return 1.0
        name = str(active_steering[0]).lower()
        if "rear" in name or "back" in name or "rl" in name or "rr" in name:
            return float(self.args.rear_steer_sign)
        if "front" in name or "fl" in name or "fr" in name:
            return float(self.args.front_steer_sign)
        return 1.0

    def _apply_base_motion(self, base_command):
        """按两转向轮同角度、四滚动轮同速度的 Ranger Arm 底盘模型发送 action。"""
        if self.wheel_indices.size == 0 or not getattr(self.args, "drive_wheels", True):
            if not self._base_motion_warned and (
                base_command.forward != 0.0 or base_command.strafe != 0.0 or base_command.yaw != 0.0
            ):
                self.carb.log_warn("Ranger Arm base motion skipped: no wheel DOFs available to drive.")
                self._base_motion_warned = True
            return
        wheel_speed = float(
            self.args.ranger_wheel_speed if self.args.ranger_wheel_speed is not None else self.args.wheel_speed
        )
        forward = float(base_command.forward)
        yaw = float(base_command.yaw)
        steering_angle = float(np.clip(yaw, -1.0, 1.0) * self.args.max_steer_rad)
        if self.steering_indices.size > 0:
            active_steering_paths = self._active_steering_paths()
            active_steering = set(active_steering_paths)
            steering_target = steering_angle * self._steer_pair_sign(active_steering_paths)
            steering_positions = []
            for joint_path in self.steering_control_paths:
                if joint_path in active_steering:
                    steering_positions.append(steering_target)
                else:
                    steering_positions.append(0.0)
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.array(steering_positions, dtype=np.float32),
                    joint_indices=self.steering_indices,
                )
            )

        wheel_velocities = []
        turn_gain = 0.6
        for joint_path in self.wheel_control_paths:
            axis_sign = self._side_sign_from_name(joint_path)
            name = str(joint_path).lower()
            lr_sign = -1.0 if ("left" in name or "lf" in name or "fl" in name or "rl" in name) else 1.0
            velocity = forward * wheel_speed * axis_sign
            if abs(forward) < 1e-5 and abs(yaw) > 1e-5:
                velocity = yaw * wheel_speed * turn_gain * lr_sign * axis_sign
            wheel_velocities.append(velocity)
        self.articulation.apply_action(
            ArticulationAction(
                joint_velocities=np.array(wheel_velocities, dtype=np.float32),
                joint_indices=self.wheel_indices,
            )
        )

    def _clamp_joint_positions(self, joint_indices, joint_positions):
        """按关节限制裁剪 Ranger Arm 双臂目标。"""
        dof_properties = self.articulation.dof_properties
        lower = dof_properties["lower"][joint_indices]
        upper = dof_properties["upper"][joint_indices]
        has_limits = dof_properties["hasLimits"][joint_indices]
        clamped = np.array(joint_positions, dtype=np.float32, copy=True)
        clamped[has_limits] = np.clip(clamped[has_limits], lower[has_limits], upper[has_limits])
        return clamped

    def _update_direct_arm_joint_targets(
        self,
        tasks,
        delta_xyz,
        delta_rot,
        joint7_delta=0.0,
        explicit_joint_delta=None,
    ) -> bool:
        """更新 Ranger Arm 直接关节目标。

        优先使用显式 7 关节增量；若未提供，再兼容旧的 delta_xyz / delta_rot 映射。
        """
        if explicit_joint_delta is not None:
            joint_delta = np.asarray(explicit_joint_delta, dtype=np.float32).reshape(-1)[:7]
            if joint_delta.size < 7:
                joint_delta = np.pad(joint_delta, (0, 7 - joint_delta.size))
        else:
            joint_delta = np.zeros(7, dtype=np.float32)
            if delta_xyz is not None and np.any(delta_xyz):
                delta_xyz = np.asarray(delta_xyz, dtype=np.float32)
                joint_delta[0] += float(delta_xyz[0]) * 8.0
                joint_delta[1] += float(delta_xyz[1]) * 8.0
                joint_delta[2] += float(delta_xyz[2]) * 8.0
            if delta_rot is not None and np.any(delta_rot):
                delta_rot = np.asarray(delta_rot, dtype=np.float32)
                joint_delta[3] += float(delta_rot[0]) * 2.0
                joint_delta[4] += float(delta_rot[1]) * 2.0
                joint_delta[5] += float(delta_rot[2]) * 2.0
            if joint7_delta != 0.0:
                joint_delta[6] += float(joint7_delta) * 2.0
        if not np.any(joint_delta):
            return False
        for task in tasks:
            joint_target = self._clamp_joint_positions(task["joint_indices"], task["joint_target"] + joint_delta)
            task["joint_target"] = np.array(joint_target, dtype=np.float32)
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.array(task["joint_target"], dtype=np.float32),
                    joint_indices=task["joint_indices"],
                )
            )
            current_pos, current_quat = self._get_world_pose_safe(task["body_path"])
            task["target_pos"] = np.array(current_pos, dtype=np.float32)
            task["target_quat"] = np.array(current_quat, dtype=np.float32)
        return True

    def _hold_direct_arm_joint_targets(self, tasks) -> None:
        """持续发送缓存的双臂关节目标，保持手臂停在目标位置。"""
        for task in tasks:
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.array(task["joint_target"], dtype=np.float32),
                    joint_indices=task["joint_indices"],
                )
            )

    def step(self, base_command, manipulator_command) -> None:
        """执行一帧遥操作：先底盘，再夹爪和机械臂 IK。"""
        if not self.available:
            return
        self._apply_base_motion(base_command)
        if not self.arm_ik_enabled:
            return
        tasks = self._active_tasks()
        self.ik_solver.apply_gripper_targets(tasks)
        delta_xyz = manipulator_command.delta_xyz
        delta_rot = manipulator_command.delta_rot
        joint_delta = manipulator_command.joint_delta
        gripper_input_active = manipulator_command.gripper_delta != 0.0
        if gripper_input_active:
            for task in tasks:
                task["gripper_target"] = float(
                    np.clip(
                        task["gripper_target"] + manipulator_command.gripper_delta,
                        task["gripper_closed"],
                        task["gripper_open"],
                    )
                )
        self._update_direct_arm_joint_targets(
            tasks,
            delta_xyz,
            delta_rot,
            manipulator_command.joint7_delta,
            explicit_joint_delta=joint_delta,
        )
        self._hold_direct_arm_joint_targets(tasks)
