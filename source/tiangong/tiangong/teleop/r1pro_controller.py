"""Galaxea R1 Pro 遥操作控制器。

负责 R1 Pro 轮式底盘、转向、躯干、双臂和夹爪的键盘遥操作。
当前默认使用直接关节控制驱动双臂，同时保留 Jacobian IK 路径用于后续切换。
"""

from __future__ import annotations

import numpy as np
from isaacsim.core.utils.types import ArticulationAction


class R1ProTeleopController:
    """把统一遥操作命令转换成 R1 Pro 底盘、躯干、手臂和夹爪动作。"""

    name = "r1pro"

    def __init__(self, articulation, carb, get_world_pose, args, scene_root_path: str):
        self.articulation = articulation
        self.carb = carb
        self.get_world_pose = get_world_pose
        self.args = args
        self.scene_root_path = scene_root_path.rstrip("/")
        self.base_speed = args.speed
        self.turn_rate_rad = np.deg2rad(args.turn_rate)
        self.available = articulation is not None
        self._base_motion_warned = False
        self.active_target_mode = "both"
        self.base_indices = np.array([], dtype=np.int32)
        self.wheel_indices = np.array([], dtype=np.int32)
        self.steer_indices = np.array([], dtype=np.int32)
        self.torso_indices = np.array([], dtype=np.int32)
        self.torso_target = np.array([], dtype=np.float32)
        self.arm_ik_tasks = []
        self.arm_ik_enabled = False
        self.direct_joint_control = True
        self._r1pro_wheel_xy = np.array(
            [
                [0.16897, 0.28],
                [0.16897, -0.28],
                [-0.32703, 0.0],
            ],
            dtype=np.float32,
        )
        if self.available:
            self._initialize()

    def _initialize(self):
        """解析 R1 Pro 的关键 DOF，并初始化双臂 task 与关节驱动参数。"""
        base_joint_names = (
            "base_footprint_x_joint",
            "base_footprint_y_joint",
            "base_footprint_rz_joint",
        )
        base_indices = []
        for joint_name in base_joint_names:
            try:
                base_indices.append(self.articulation.get_dof_index(joint_name))
            except Exception:
                pass
        self.base_indices = np.array(base_indices, dtype=np.int32)
        wheel_indices = []
        for joint_name in (f"wheel_motor_joint{index}" for index in range(1, 4)):
            try:
                wheel_indices.append(self.articulation.get_dof_index(joint_name))
            except Exception:
                pass
        self.wheel_indices = np.array(wheel_indices, dtype=np.int32)
        steer_indices = []
        for joint_name in (f"steer_motor_joint{index}" for index in range(1, 4)):
            try:
                steer_indices.append(self.articulation.get_dof_index(joint_name))
            except Exception:
                pass
        self.steer_indices = np.array(steer_indices, dtype=np.int32)
        torso_indices = []
        for joint_name in (f"torso_joint{index}" for index in range(1, 5)):
            try:
                torso_indices.append(self.articulation.get_dof_index(joint_name))
            except Exception:
                self.carb.log_warn(f"r1pro torso DOF unavailable: {joint_name}")
        self.torso_indices = np.array(torso_indices, dtype=np.int32)
        if self.torso_indices.size:
            self.torso_target = np.array(self.articulation.get_joint_positions(self.torso_indices), dtype=np.float32)
        jacobian_shape = self.articulation._articulation_view.get_jacobian_shape()
        self.arm_ik_tasks = [
            task
            for task in (
                self._make_arm_ik_task("left", ("left_eef_link", "left_gripper_link"), jacobian_shape),
                self._make_arm_ik_task("right", ("right_eef_link", "right_gripper_link"), jacobian_shape),
            )
            if task is not None
        ]
        self.arm_ik_enabled = bool(self.arm_ik_tasks)
        self._configure_joint_drives()
        self.carb.log_warn(
            f"r1pro direct joint teleop enabled for arms/grippers. "
            f"wheel_dofs={self.wheel_indices.size}, steer_dofs={self.steer_indices.size}."
        )

    def _get_world_pose_safe(self, prim_path: str):
        """优先用 fabric 获取位姿，失败时回退到普通 USD 查询。"""
        try:
            return self.get_world_pose(prim_path, fabric=True)
        except Exception:
            return self.get_world_pose(prim_path)

    def _to_pose_arrays(self, position, quat):
        """把 IsaacSim/tensor 返回值规整成 numpy 位姿数组。"""
        return np.asarray(position, dtype=np.float32).reshape(-1)[:3], np.asarray(quat, dtype=np.float32).reshape(-1)[:4]

    def get_base_world_pose(self):
        """读取 R1 Pro 物理 articulation 的实时底座位姿。

        目标区域控制不能使用静态 USD 根节点 xform；轮式底盘移动后，
        PhysX articulation/root link 才是距离计算需要的实时定位来源。
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
        for body_name in ("base_link", "base_footprint_x", "base_footprint"):
            try:
                return self._to_pose_arrays(*self._get_world_pose_safe(f"{self.scene_root_path}/{body_name}"))
            except Exception:
                continue
        return self._to_pose_arrays(*self._get_world_pose_safe(self.scene_root_path))

    def _make_arm_ik_task(self, side: str, body_names, jacobian_shape):
        """为单侧 R1 Pro 手臂创建 IK/直接关节控制所需的索引和目标缓存。"""
        joint_names = [f"{side}_arm_joint{index}" for index in range(1, 8)]
        gripper_names = [f"{side}_gripper_finger_joint1", f"{side}_gripper_finger_joint2"]
        try:
            joint_indices = np.array([self.articulation.get_dof_index(name) for name in joint_names], dtype=np.int32)
            gripper_indices = np.array([self.articulation.get_dof_index(name) for name in gripper_names], dtype=np.int32)
        except Exception as exc:
            self.carb.log_warn(f"r1pro {side} arm/gripper DOFs unavailable: {exc}")
            return None
        body_name = None
        body_index = None
        for candidate in body_names:
            try:
                body_index = self.articulation._articulation_view.get_body_index(candidate)
                if body_index < 0:
                    continue
                body_name = candidate
                break
            except Exception:
                continue
        if body_name is None or body_index is None:
            self.carb.log_warn(f"r1pro {side} arm body unavailable: {tuple(body_names)}")
            return None
        floating_base = jacobian_shape[-1] == self.articulation.num_dof + 6
        jacobian_body_index = body_index if floating_base else body_index - 1
        jacobian_joint_offset = 6 if floating_base else 0
        body_path = f"{self.scene_root_path}/{body_name}"
        target_pos, target_quat = self._get_world_pose_safe(body_path)
        joint_positions = self.articulation.get_joint_positions(joint_indices)
        gripper_positions = self.articulation.get_joint_positions(gripper_indices)
        dof_properties = self.articulation.dof_properties
        lower = dof_properties["lower"][gripper_indices]
        upper = dof_properties["upper"][gripper_indices]
        has_limits = dof_properties["hasLimits"][gripper_indices]
        gripper_closed = np.zeros(gripper_indices.shape, dtype=np.float32)
        gripper_open = np.full(gripper_indices.shape, 0.05, dtype=np.float32)
        if np.any(has_limits):
            gripper_closed[has_limits] = np.clip(0.0, lower[has_limits], upper[has_limits])
            lower_distance = np.abs(lower[has_limits] - gripper_closed[has_limits])
            upper_distance = np.abs(upper[has_limits] - gripper_closed[has_limits])
            gripper_open[has_limits] = np.where(upper_distance >= lower_distance, upper[has_limits], lower[has_limits])
        gripper_positions = np.clip(
            gripper_positions,
            np.minimum(gripper_closed, gripper_open),
            np.maximum(gripper_closed, gripper_open),
        )
        return {
            "side": side,
            "body_path": body_path,
            "joint_indices": joint_indices,
            "gripper_indices": gripper_indices,
            "jacobian_body_index": jacobian_body_index,
            "jacobian_joint_columns": joint_indices + jacobian_joint_offset,
            "target_pos": np.array(target_pos, dtype=np.float32),
            "target_quat": np.array(target_quat, dtype=np.float32),
            "joint_target": np.array(joint_positions, dtype=np.float32),
            "gripper_closed": gripper_closed,
            "gripper_open": gripper_open,
            "gripper_target": np.array(gripper_positions, dtype=np.float32),
        }

    def _configure_joint_drives(self):
        """设置底盘、转向、躯干、手臂和夹爪的 position/velocity drive 参数。"""
        try:
            if self.torso_indices.size:
                self.articulation._articulation_view.set_gains(
                    kps=np.full((1, self.torso_indices.size), 15000.0, dtype=np.float32),
                    kds=np.full((1, self.torso_indices.size), 1500.0, dtype=np.float32),
                    joint_indices=self.torso_indices,
                )
                self.articulation._articulation_view.set_max_efforts(
                    values=np.full((1, self.torso_indices.size), 3000.0, dtype=np.float32),
                    joint_indices=self.torso_indices,
                )
            if self.base_indices.size:
                self.articulation._articulation_view.set_gains(
                    kps=np.zeros((1, self.base_indices.size), dtype=np.float32),
                    kds=np.full((1, self.base_indices.size), 10000.0, dtype=np.float32),
                    joint_indices=self.base_indices,
                )
                self.articulation._articulation_view.set_max_efforts(
                    values=np.full((1, self.base_indices.size), 100000.0, dtype=np.float32),
                    joint_indices=self.base_indices,
                )
            if self.arm_ik_tasks:
                arm_indices = np.unique(np.concatenate([task["joint_indices"] for task in self.arm_ik_tasks])).astype(np.int32)
                gripper_indices = np.unique(np.concatenate([task["gripper_indices"] for task in self.arm_ik_tasks])).astype(
                    np.int32
                )
                self.articulation._articulation_view.set_gains(
                    kps=np.full((1, arm_indices.size), 12000.0, dtype=np.float32),
                    kds=np.full((1, arm_indices.size), 1200.0, dtype=np.float32),
                    joint_indices=arm_indices,
                )
                self.articulation._articulation_view.set_max_efforts(
                    values=np.full((1, arm_indices.size), 2000.0, dtype=np.float32),
                    joint_indices=arm_indices,
                )
                self.articulation._articulation_view.set_gains(
                    kps=np.full((1, gripper_indices.size), self.args.gripper_stiffness, dtype=np.float32),
                    kds=np.full((1, gripper_indices.size), self.args.gripper_damping, dtype=np.float32),
                    joint_indices=gripper_indices,
                )
                self.articulation._articulation_view.set_max_efforts(
                    values=np.full((1, gripper_indices.size), self.args.gripper_max_effort, dtype=np.float32),
                    joint_indices=gripper_indices,
                )
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
            if self.steer_indices.size:
                self.articulation._articulation_view.set_gains(
                    kps=np.full((1, self.steer_indices.size), 2500.0, dtype=np.float32),
                    kds=np.full((1, self.steer_indices.size), 250.0, dtype=np.float32),
                    joint_indices=self.steer_indices,
                )
                self.articulation._articulation_view.set_max_efforts(
                    values=np.full((1, self.steer_indices.size), 500.0, dtype=np.float32),
                    joint_indices=self.steer_indices,
                )
        except Exception as exc:
            self.carb.log_warn(f"Could not configure r1pro joint gains: {exc}")

    def cycle_target_mode(self) -> str:
        """在左臂、右臂、双臂之间循环切换当前控制目标。"""
        modes = ("left", "right", "both")
        self.active_target_mode = modes[(modes.index(self.active_target_mode) + 1) % len(modes)]
        self.sync_ik_targets()
        return self.active_target_mode

    def _active_tasks(self):
        """返回当前目标模式对应的手臂 task。"""
        if self.active_target_mode == "both":
            return list(self.arm_ik_tasks)
        return [task for task in self.arm_ik_tasks if task["side"] == self.active_target_mode]

    def sync_ik_targets(self):
        """把缓存的末端目标刷新为当前真实末端位姿。"""
        for task in self.arm_ik_tasks:
            current_pos, current_quat = self._get_world_pose_safe(task["body_path"])
            task["target_pos"] = np.array(current_pos, dtype=np.float32)
            task["target_quat"] = np.array(current_quat, dtype=np.float32)

    def _quat_conjugate(self, quat):
        return np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=np.float32)

    def _quat_multiply(self, lhs, rhs):
        w0, x0, y0, z0 = lhs
        w1, x1, y1, z1 = rhs
        return np.array(
            [
                w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
                w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
                w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
                w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
            ],
            dtype=np.float32,
        )

    def _axis_angle_error(self, current_quat, target_quat):
        quat_error = self._quat_multiply(target_quat, self._quat_conjugate(current_quat))
        if quat_error[0] < 0.0:
            quat_error = -quat_error
        vector_norm = float(np.linalg.norm(quat_error[1:4]))
        if vector_norm < 1e-6:
            return np.zeros(3, dtype=np.float32)
        angle = 2.0 * np.arctan2(vector_norm, float(np.clip(quat_error[0], -1.0, 1.0)))
        return quat_error[1:4] / vector_norm * angle

    def _quat_from_axis_angle(self, axis, angle):
        axis = np.asarray(axis, dtype=np.float32)
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1e-6 or abs(angle) < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        axis = axis / axis_norm
        half_angle = angle * 0.5
        sin_half = np.sin(half_angle)
        return np.array([np.cos(half_angle), axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half], dtype=np.float32)

    def _normalize_quat(self, quat):
        quat = np.asarray(quat, dtype=np.float32)
        quat_norm = float(np.linalg.norm(quat))
        if quat_norm < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        return quat / quat_norm

    def _apply_target_rotation(self, tasks, delta_rot):
        if delta_rot is None:
            return
        delta_rot = np.asarray(delta_rot, dtype=np.float32)
        if not np.any(delta_rot):
            return
        for task in tasks:
            delta_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            for axis, angle in (
                (np.array([1.0, 0.0, 0.0], dtype=np.float32), float(delta_rot[0])),
                (np.array([0.0, 1.0, 0.0], dtype=np.float32), float(delta_rot[1])),
                (np.array([0.0, 0.0, 1.0], dtype=np.float32), float(delta_rot[2])),
            ):
                delta_quat = self._quat_multiply(self._quat_from_axis_angle(axis, angle), delta_quat)
            task["target_quat"] = self._normalize_quat(self._quat_multiply(task["target_quat"], delta_quat))

    def _clamp_joint_positions(self, joint_indices, joint_positions):
        dof_properties = self.articulation.dof_properties
        lower = dof_properties["lower"][joint_indices]
        upper = dof_properties["upper"][joint_indices]
        has_limits = dof_properties["hasLimits"][joint_indices]
        clamped = np.array(joint_positions, dtype=np.float32, copy=True)
        clamped[has_limits] = np.clip(clamped[has_limits], lower[has_limits], upper[has_limits])
        return clamped

    def _damped_least_squares_delta(self, jacobian, error):
        jacobian_t = jacobian.T
        damping_matrix = np.eye(jacobian.shape[0], dtype=np.float32) * (self.args.ik_damping**2)
        try:
            delta = jacobian_t @ np.linalg.solve(jacobian @ jacobian_t + damping_matrix, error)
        except np.linalg.LinAlgError:
            delta = np.linalg.pinv(jacobian) @ error
        delta = delta * self.args.ik_gain
        max_step = max(self.args.ik_max_joint_step, 0.0)
        if max_step > 0.0:
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > max_step:
                delta = delta * (max_step / delta_norm)
        return delta

    def _apply_gripper_targets(self, tasks):
        """将夹爪目标位置发送给当前活动 task。"""
        for task in tasks:
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.array(task["gripper_target"], dtype=np.float32),
                    joint_indices=task["gripper_indices"],
                )
            )

    def _apply_arm_ik_targets(self, tasks):
        """通过 Jacobian DLS 求解双臂 IK，并发送关节位置目标。"""
        jacobians = self.articulation._articulation_view.get_jacobians()
        if jacobians is None:
            return
        jacobians = np.asarray(jacobians)
        for task in tasks:
            current_pos, current_quat = self._get_world_pose_safe(task["body_path"])
            current_pos = np.array(current_pos, dtype=np.float32)
            current_quat = np.array(current_quat, dtype=np.float32)
            position_error = task["target_pos"] - current_pos
            orientation_error = self._axis_angle_error(current_quat, task["target_quat"])
            ik_error = np.concatenate([position_error, orientation_error]).astype(np.float32)
            jacobian = jacobians[0, task["jacobian_body_index"], 0:6, :][:, task["jacobian_joint_columns"]]
            if jacobian.shape[0] != ik_error.shape[0]:
                continue
            joint_pos = self.articulation.get_joint_positions(task["joint_indices"])
            delta_joint_pos = self._damped_least_squares_delta(jacobian, ik_error)
            joint_target = self._clamp_joint_positions(task["joint_indices"], joint_pos + delta_joint_pos)
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.array(joint_target, dtype=np.float32),
                    joint_indices=task["joint_indices"],
                )
            )

    def _update_direct_arm_joint_targets(self, tasks, delta_xyz, delta_rot, joint7_delta=0.0) -> bool:
        """把键盘末端增量映射为直接关节增量，当前用于更稳定的 R1 Pro 手臂控制。"""
        joint_delta = np.zeros(7, dtype=np.float32)
        if delta_xyz is not None and np.any(delta_xyz):
            delta_xyz = np.asarray(delta_xyz, dtype=np.float32)
            joint_delta[1] += float(delta_xyz[0]) * 8.0
            joint_delta[0] += float(delta_xyz[1]) * 8.0
            joint_delta[2] += float(delta_xyz[2]) * 8.0
        if delta_rot is not None and np.any(delta_rot):
            delta_rot = np.asarray(delta_rot, dtype=np.float32)
            joint_delta[6] += float(delta_rot[0]) * 2.0
            joint_delta[5] += float(delta_rot[1]) * 2.0
            joint_delta[4] += float(delta_rot[2]) * 2.0
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
        return True

    def _hold_direct_arm_joint_targets(self, tasks) -> None:
        """持续发送缓存的手臂关节目标，避免 PhysX 驱动掉目标。"""
        for task in tasks:
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.array(task["joint_target"], dtype=np.float32),
                    joint_indices=task["joint_indices"],
                )
            )

    def _apply_base_motion(self, base_command):
        """只驱动真实轮/转向 DOF；缺失时不平移 prim 或虚拟底座。"""
        if self.wheel_indices.size and getattr(self.args, "drive_wheels", True):
            self._apply_wheel_motion(base_command)
            return
        if not self._base_motion_warned and (
            base_command.forward != 0.0 or base_command.strafe != 0.0 or base_command.yaw != 0.0
        ):
            self.carb.log_warn("R1 Pro base motion skipped: no wheel DOFs available to drive.")
            self._base_motion_warned = True

    def _apply_wheel_motion(self, base_command):
        """根据三轮底盘几何计算每个轮子的转向角和轮速。"""
        if self.wheel_indices.size:
            forward = float(base_command.forward)
            strafe = float(base_command.strafe)
            yaw = float(base_command.yaw)
            r1pro_wheel_speed = getattr(self.args, "r1pro_wheel_speed", None)
            wheel_speed = float(r1pro_wheel_speed if r1pro_wheel_speed is not None else getattr(self.args, "wheel_speed", 3.0))
            wheel_xy = self._r1pro_wheel_xy[: self.wheel_indices.size]
            wheel_vectors = np.zeros((self.wheel_indices.size, 2), dtype=np.float32)
            wheel_vectors[:, 0] = forward - yaw * wheel_xy[:, 1]
            wheel_vectors[:, 1] = strafe + yaw * wheel_xy[:, 0]
            velocities = np.linalg.norm(wheel_vectors, axis=1).astype(np.float32) * wheel_speed
            reverse = wheel_vectors[:, 0] < 0.0
            velocities[reverse] *= -1.0
            self.articulation.apply_action(
                ArticulationAction(joint_velocities=velocities, joint_indices=self.wheel_indices)
            )
        if self.steer_indices.size:
            forward = float(base_command.forward)
            strafe = float(base_command.strafe)
            yaw = float(base_command.yaw)
            wheel_xy = self._r1pro_wheel_xy[: self.steer_indices.size]
            steer_vectors = np.zeros((self.steer_indices.size, 2), dtype=np.float32)
            steer_vectors[:, 0] = forward - yaw * wheel_xy[:, 1]
            steer_vectors[:, 1] = strafe + yaw * wheel_xy[:, 0]
            steer_target = np.arctan2(steer_vectors[:, 1], steer_vectors[:, 0]).astype(np.float32)
            reverse = steer_vectors[:, 0] < 0.0
            steer_target[reverse] = steer_target[reverse] - np.sign(steer_target[reverse]) * np.pi
            idle = np.linalg.norm(steer_vectors, axis=1) < 1e-6
            steer_target[idle] = 0.0
            max_steer = abs(float(getattr(self.args, "r1pro_max_steer_rad", np.pi)))
            if max_steer < 1e-6:
                max_steer = np.pi
            steer_target = np.clip(steer_target[: self.steer_indices.size], -max_steer, max_steer)
            self.articulation.apply_action(
                ArticulationAction(joint_positions=steer_target, joint_indices=self.steer_indices)
            )

    def _apply_torso_target(self, torso_delta) -> None:
        """累积并发送躯干关节目标。"""
        if self.torso_indices.size == 0:
            return
        if torso_delta is not None and np.any(torso_delta):
            torso_delta = np.asarray(torso_delta, dtype=np.float32)
            delta = np.zeros(self.torso_indices.shape, dtype=np.float32)
            copy_count = min(delta.size, torso_delta.size)
            delta[:copy_count] = torso_delta[:copy_count]
            self.torso_target = self._clamp_joint_positions(self.torso_indices, self.torso_target + delta)
        self.articulation.apply_action(
            ArticulationAction(
                joint_positions=np.array(self.torso_target, dtype=np.float32),
                joint_indices=self.torso_indices,
            )
        )

    def step(self, base_command, manipulator_command) -> None:
        """执行一帧 R1 Pro 遥操作命令。"""
        if not self.available:
            return
        self._apply_base_motion(base_command)
        tasks = self._active_tasks()
        delta_xyz = manipulator_command.delta_xyz
        delta_rot = manipulator_command.delta_rot
        joint7_delta = manipulator_command.joint7_delta
        self._apply_torso_target(manipulator_command.torso_delta)
        if manipulator_command.gripper_delta != 0.0:
            for task in tasks:
                gripper_closed = np.asarray(task["gripper_closed"], dtype=np.float32)
                gripper_open = np.asarray(task["gripper_open"], dtype=np.float32)
                gripper_direction = np.sign(gripper_open - gripper_closed).astype(np.float32)
                gripper_direction[gripper_direction == 0.0] = 1.0
                gripper_min = np.minimum(gripper_closed, gripper_open)
                gripper_max = np.maximum(gripper_closed, gripper_open)
                task["gripper_target"] = np.clip(
                    np.asarray(task["gripper_target"], dtype=np.float32)
                    + manipulator_command.gripper_delta * gripper_direction,
                    gripper_min,
                    gripper_max,
                )
        self._apply_gripper_targets(tasks)
        if self.direct_joint_control:
            moved = self._update_direct_arm_joint_targets(tasks, delta_xyz, delta_rot, joint7_delta)
            self._hold_direct_arm_joint_targets(tasks)
            if moved:
                self.sync_ik_targets()
            return
        if delta_xyz is not None and np.any(delta_xyz):
            for task in tasks:
                task["target_pos"] = task["target_pos"] + delta_xyz
        else:
            self.sync_ik_targets()
        self._apply_target_rotation(tasks, delta_rot)
        if (
            (delta_xyz is not None and np.any(delta_xyz))
            or (delta_rot is not None and np.any(delta_rot))
            or manipulator_command.gripper_delta != 0.0
        ):
            self._apply_arm_ik_targets(tasks)
