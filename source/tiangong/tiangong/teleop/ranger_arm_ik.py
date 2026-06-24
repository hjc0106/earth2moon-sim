"""Ranger Arm 双臂差分 IK 求解模块。

本文件只负责 IK 任务创建、目标同步、夹爪目标和 Jacobian DLS 求解；
底盘移动、按键映射和控制器切换由 ranger_arm_controller.py 及 dispatcher.py 负责。
"""

from __future__ import annotations

import numpy as np
from isaacsim.core.utils.types import ArticulationAction


class RangerArmIKSolver:
    """封装 Ranger Arm 左右 7 自由度机械臂的差分 IK 求解。"""

    def __init__(self, articulation, root_path: str, get_world_pose, args):
        self.articulation = articulation
        self.root_path = root_path
        self.get_world_pose = get_world_pose
        self.args = args
        self.tasks = []
        self.enabled = False

    def initialize(self) -> bool:
        """根据 articulation 构建左右臂 IK task，并配置夹爪驱动参数。"""
        jacobian_shape = self.articulation._articulation_view.get_jacobian_shape()
        self.tasks = [
            self._make_task(
                side,
                self.args.left_ee_body if side == "left" else self.args.right_ee_body,
                jacobian_shape,
            )
            for side in ("left", "right")
        ]
        self.enabled = bool(self.tasks)
        self.configure_gripper_drives()
        return self.enabled

    def _get_world_pose_safe(self, prim_path: str):
        """优先用 fabric 查询位姿，失败时回退到普通 USD 查询。"""
        try:
            return self.get_world_pose(prim_path, fabric=True)
        except Exception:
            return self.get_world_pose(prim_path)

    def _make_task(self, side: str, body_name: str, jacobian_shape):
        """创建单侧机械臂 IK task，缓存关节、末端、Jacobian 列和夹爪限制。"""
        joint_names = [f"arm_{side}_joint{index}" for index in range(1, 8)]
        gripper_names = [f"gripper_{side}_joint", f"gripper_{side}_joint_mimic"]
        joint_indices = np.array([self.articulation.get_dof_index(name) for name in joint_names], dtype=np.int32)
        gripper_indices = np.array([self.articulation.get_dof_index(name) for name in gripper_names], dtype=np.int32)
        body_index = self.articulation._articulation_view.get_body_index(body_name)
        floating_base = jacobian_shape[-1] == self.articulation.num_dof + 6
        jacobian_body_index = body_index if floating_base else body_index - 1
        jacobian_joint_offset = 6 if floating_base else 0
        body_path = f"{self.root_path}/{body_name}"
        target_pos, target_quat = self._get_world_pose_safe(body_path)
        joint_positions = self.articulation.get_joint_positions(joint_indices)
        gripper_positions = self.articulation.get_joint_positions(gripper_indices)
        dof_properties = self.articulation.dof_properties
        lower = dof_properties["lower"][gripper_indices]
        upper = dof_properties["upper"][gripper_indices]
        has_limits = dof_properties["hasLimits"][gripper_indices]
        limited_lower = lower[has_limits]
        limited_upper = upper[has_limits]
        gripper_min = float(np.max(limited_lower)) if limited_lower.size else float(self.args.gripper_closed_position)
        gripper_max = float(np.min(limited_upper)) if limited_upper.size else float(self.args.gripper_open_position)
        gripper_closed = float(np.clip(self.args.gripper_closed_position, gripper_min, gripper_max))
        gripper_open = float(np.clip(self.args.gripper_open_position, gripper_min, gripper_max))
        if gripper_open < gripper_closed:
            gripper_closed, gripper_open = gripper_open, gripper_closed
        return {
            "side": side,
            "body_name": body_name,
            "body_path": body_path,
            "joint_names": joint_names,
            "joint_indices": joint_indices,
            "gripper_names": gripper_names,
            "gripper_indices": gripper_indices,
            "jacobian_body_index": jacobian_body_index,
            "jacobian_joint_columns": joint_indices + jacobian_joint_offset,
            "target_pos": np.array(target_pos, dtype=np.float32),
            "target_quat": np.array(target_quat, dtype=np.float32),
            "joint_target": np.array(joint_positions, dtype=np.float32),
            "gripper_closed": gripper_closed,
            "gripper_open": gripper_open,
            "gripper_target": float(np.clip(np.mean(gripper_positions), gripper_closed, gripper_open)),
        }

    def configure_gripper_drives(self) -> None:
        """给左右夹爪 DOF 设置位置驱动刚度、阻尼和最大力。"""
        if not self.tasks:
            return
        gripper_indices = np.unique(np.concatenate([task["gripper_indices"] for task in self.tasks])).astype(np.int32)
        self.articulation._articulation_view.set_gains(
            kps=np.full((1, gripper_indices.size), self.args.gripper_stiffness, dtype=np.float32),
            kds=np.full((1, gripper_indices.size), self.args.gripper_damping, dtype=np.float32),
            joint_indices=gripper_indices,
        )
        self.articulation._articulation_view.set_max_efforts(
            values=np.full((1, gripper_indices.size), self.args.gripper_max_effort, dtype=np.float32),
            joint_indices=gripper_indices,
        )

    def sync_targets(self) -> None:
        """把 IK 目标同步到当前末端位姿，通常只在切换控制对象时调用。"""
        for task in self.tasks:
            current_pos, current_quat = self._get_world_pose_safe(task["body_path"])
            task["target_pos"] = np.array(current_pos, dtype=np.float32)
            task["target_quat"] = np.array(current_quat, dtype=np.float32)

    def apply_gripper_targets(self, tasks) -> None:
        """把 task 中缓存的夹爪目标位置发送给 articulation。"""
        for task in tasks:
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.full(task["gripper_indices"].shape, task["gripper_target"], dtype=np.float32),
                    joint_indices=task["gripper_indices"],
                )
            )

    def rotate_targets(self, tasks, delta_rot) -> None:
        """按 XYZ 小角度增量旋转当前 IK 目标姿态。"""
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
                delta_quat = quat_multiply(quat_from_axis_angle(axis, angle), delta_quat)
            task["target_quat"] = normalize_quat(quat_multiply(task["target_quat"], delta_quat))

    def apply_targets(self, tasks, use_orientation: bool) -> None:
        """读取 Jacobian 并求解关节增量，将末端持续拉向目标位姿。"""
        jacobians = self.articulation._articulation_view.get_jacobians()
        if jacobians is None:
            return
        jacobians = np.asarray(jacobians)
        for task in tasks:
            current_pos, current_quat = self._get_world_pose_safe(task["body_path"])
            current_pos = np.array(current_pos, dtype=np.float32)
            current_quat = np.array(current_quat, dtype=np.float32)
            position_error = task["target_pos"] - current_pos
            orientation_error = axis_angle_error(current_quat, task["target_quat"]) if use_orientation else None
            if orientation_error is None:
                ik_error = position_error
                jacobian = jacobians[0, task["jacobian_body_index"], 0:3, :][:, task["jacobian_joint_columns"]]
            else:
                ik_error = np.concatenate([position_error, orientation_error]).astype(np.float32)
                jacobian = jacobians[0, task["jacobian_body_index"], 0:6, :][:, task["jacobian_joint_columns"]]
            if jacobian.shape[0] != ik_error.shape[0]:
                continue
            if np.linalg.norm(ik_error) < 1e-4:
                continue
            joint_pos = self.articulation.get_joint_positions(task["joint_indices"])
            delta_joint_pos = damped_least_squares_delta(
                jacobian,
                ik_error,
                damping=self.args.ik_damping,
                gain=self.args.ik_gain,
                max_step=self.args.ik_max_joint_step,
            )
            joint_target = self._clamp_joint_positions(task["joint_indices"], joint_pos + delta_joint_pos)
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.array(joint_target, dtype=np.float32),
                    joint_indices=task["joint_indices"],
                )
            )

    def _clamp_joint_positions(self, joint_indices, joint_positions):
        """按 USD/PhysX 中的 DOF 限制裁剪关节目标。"""
        dof_properties = self.articulation.dof_properties
        lower = dof_properties["lower"][joint_indices]
        upper = dof_properties["upper"][joint_indices]
        has_limits = dof_properties["hasLimits"][joint_indices]
        clamped = np.array(joint_positions, dtype=np.float32, copy=True)
        clamped[has_limits] = np.clip(clamped[has_limits], lower[has_limits], upper[has_limits])
        return clamped


def damped_least_squares_delta(jacobian, error, damping: float, gain: float, max_step: float):
    """使用阻尼最小二乘法计算 dq，并限制单步关节范数。"""
    jacobian_t = jacobian.T
    damping_matrix = np.eye(jacobian.shape[0], dtype=np.float32) * (float(damping) ** 2)
    try:
        delta = jacobian_t @ np.linalg.solve(jacobian @ jacobian_t + damping_matrix, error)
    except np.linalg.LinAlgError:
        delta = np.linalg.pinv(jacobian) @ error
    delta = delta * float(gain)
    max_step = max(float(max_step), 0.0)
    if max_step > 0.0:
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > max_step:
            delta = delta * (max_step / delta_norm)
    return delta


def quat_conjugate(quat):
    """返回四元数共轭，格式为 IsaacSim 常用的 [w, x, y, z]。"""
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=np.float32)


def quat_multiply(lhs, rhs):
    """计算两个 [w, x, y, z] 四元数的乘积。"""
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


def axis_angle_error(current_quat, target_quat):
    """把当前姿态到目标姿态的误差转换为轴角向量。"""
    quat_error = quat_multiply(target_quat, quat_conjugate(current_quat))
    if quat_error[0] < 0.0:
        quat_error = -quat_error
    vector_norm = float(np.linalg.norm(quat_error[1:4]))
    if vector_norm < 1e-6:
        return np.zeros(3, dtype=np.float32)
    angle = 2.0 * np.arctan2(vector_norm, float(np.clip(quat_error[0], -1.0, 1.0)))
    return quat_error[1:4] / vector_norm * angle


def quat_from_axis_angle(axis, angle):
    """由旋转轴和角度生成 [w, x, y, z] 四元数。"""
    axis = np.asarray(axis, dtype=np.float32)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-6 or abs(angle) < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    axis = axis / axis_norm
    half_angle = angle * 0.5
    sin_half = np.sin(half_angle)
    return np.array(
        [np.cos(half_angle), axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half],
        dtype=np.float32,
    )


def normalize_quat(quat):
    """归一化四元数；异常接近零时回退到单位姿态。"""
    quat = np.asarray(quat, dtype=np.float32)
    quat_norm = float(np.linalg.norm(quat))
    if quat_norm < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return quat / quat_norm
