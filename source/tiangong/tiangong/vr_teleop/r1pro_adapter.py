"""从 VR 末端目标到 R1 Pro 控制器的独立桥接层。

这是机器人控制侧的第一步具体接入：

- 输入侧：``tiangong.vr_teleop`` 生成的通用双臂 VR 目标
- 机器人侧：现有的 ``R1ProTeleopController``

它与旧启动脚本保持独立，方便我们单独迭代 VR 遥操作，而不影响键盘遥操作。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .bridge import VRBridgeOutput


def _quat_slerp(quat_a_wxyz: np.ndarray, quat_b_wxyz: np.ndarray, alpha: float) -> np.ndarray:
    """对两个四元数做球面线性插值。"""
    quat_a = np.asarray(quat_a_wxyz, dtype=np.float32).reshape(4)
    quat_b = np.asarray(quat_b_wxyz, dtype=np.float32).reshape(4)
    quat_a = quat_a / max(float(np.linalg.norm(quat_a)), 1e-8)
    quat_b = quat_b / max(float(np.linalg.norm(quat_b)), 1e-8)
    dot = float(np.dot(quat_a, quat_b))
    if dot < 0.0:
        quat_b = -quat_b
        dot = -dot
    alpha = float(np.clip(alpha, 0.0, 1.0))
    if dot > 0.9995:
        blended = quat_a + alpha * (quat_b - quat_a)
        return blended / max(float(np.linalg.norm(blended)), 1e-8)
    theta_0 = float(np.arccos(np.clip(dot, -1.0, 1.0)))
    sin_theta_0 = float(np.sin(theta_0))
    if sin_theta_0 < 1e-6:
        return quat_a
    theta = theta_0 * alpha
    sin_theta = float(np.sin(theta))
    s0 = float(np.sin(theta_0 - theta) / sin_theta_0)
    s1 = float(sin_theta / sin_theta_0)
    return (s0 * quat_a + s1 * quat_b).astype(np.float32)


@dataclass(slots=True)
class R1ProVRAdapterConfig:
    """VR 目标映射到 R1 Pro 控制器的配置。"""

    position_alpha: float = 1.0
    rotation_alpha: float = 1.0
    gripper_alpha: float = 1.0
    control_dt: float = 1.0 / 60.0
    max_position_speed: float = 1.5
    max_rotation_speed: float = 4.0
    sync_missing_side: bool = False


class R1ProVRAdapter:
    """把 VR 桥接层输出写入现有 ``R1ProTeleopController``。

    说明：
    - 这个适配器假设控制器已经完成初始化。
    - 手臂末端目标复用控制器现有的 IK 路径。
    - 夹爪值直接映射到左右夹爪目标。
    """

    def __init__(self, controller, cfg: R1ProVRAdapterConfig | None = None):
        self.controller = controller
        self.cfg = cfg or R1ProVRAdapterConfig()

    def apply(self, output: VRBridgeOutput) -> bool:
        """把双臂 VR 命令应用到 R1 Pro 控制器。

        返回：
            只要至少有一侧手臂目标被更新并推送到控制器，就返回 ``True``。
        """
        if self.controller is None or not getattr(self.controller, "available", False):
            return False
        tasks = getattr(self.controller, "arm_ik_tasks", None)
        if not tasks:
            return False

        updated = False
        command = output.command
        side_to_target = {
            "left": command.left,
            "right": command.right,
        }

        for task in tasks:
            side = task.get("side")
            target = side_to_target.get(side)
            if target is None:
                if self.cfg.sync_missing_side:
                    self._sync_task_to_current_pose(task)
                continue
            self._apply_task_target(task, target.position, target.quat_wxyz, target.gripper)
            updated = True

        if updated:
            self.controller._apply_arm_ik_targets(tasks)
            self.controller._apply_gripper_targets(tasks)
        return updated

    def _apply_task_target(
        self,
        task: dict,
        target_pos: np.ndarray,
        target_quat_wxyz: np.ndarray,
        target_gripper: float,
    ) -> None:
        """根据单侧 VR 目标更新单个控制 task。"""
        try:
            current_pos, current_quat = self.controller._get_world_pose_safe(task["body_path"])
            current_pos = np.asarray(current_pos, dtype=np.float32).reshape(3)
            current_quat = self.controller._normalize_quat(np.asarray(current_quat, dtype=np.float32).reshape(4))
        except Exception:
            current_pos = np.asarray(task["target_pos"], dtype=np.float32).reshape(3)
            current_quat = self.controller._normalize_quat(np.asarray(task["target_quat"], dtype=np.float32).reshape(4))
        desired_pos = np.asarray(target_pos, dtype=np.float32).reshape(3)
        alpha = float(np.clip(self.cfg.position_alpha, 0.0, 1.0))
        position_delta = alpha * (desired_pos - current_pos)
        max_position_step = max(float(self.cfg.max_position_speed), 0.0) * max(float(self.cfg.control_dt), 0.0)
        position_delta_norm = float(np.linalg.norm(position_delta))
        if max_position_step > 0.0 and position_delta_norm > max_position_step:
            position_delta *= max_position_step / position_delta_norm
        task["target_pos"] = current_pos + position_delta
        desired_quat = self.controller._normalize_quat(np.asarray(target_quat_wxyz, dtype=np.float32).reshape(4))
        rotation_alpha = float(np.clip(self.cfg.rotation_alpha, 0.0, 1.0))
        quat_dot = float(np.clip(abs(np.dot(current_quat, desired_quat)), 0.0, 1.0))
        angular_distance = 2.0 * float(np.arccos(quat_dot))
        max_rotation_step = max(float(self.cfg.max_rotation_speed), 0.0) * max(float(self.cfg.control_dt), 0.0)
        if max_rotation_step > 0.0 and angular_distance > max_rotation_step:
            rotation_alpha = min(rotation_alpha, max_rotation_step / angular_distance)
        task["target_quat"] = self.controller._normalize_quat(_quat_slerp(current_quat, desired_quat, rotation_alpha))

        close_weight = 0.5 * (1.0 - float(np.clip(target_gripper, -1.0, 1.0)))
        gripper_closed = np.asarray(task["gripper_closed"], dtype=np.float32)
        gripper_open = np.asarray(task["gripper_open"], dtype=np.float32)
        desired_gripper = gripper_open + close_weight * (gripper_closed - gripper_open)
        grip_alpha = float(np.clip(self.cfg.gripper_alpha, 0.0, 1.0))
        task["gripper_target"] = (
            (1.0 - grip_alpha) * np.asarray(task["gripper_target"], dtype=np.float32)
            + grip_alpha * np.asarray(desired_gripper, dtype=np.float32)
        )

    def _sync_task_to_current_pose(self, task: dict) -> None:
        """把单个 task 刷新为当前真实末端位姿。"""
        current_pos, current_quat = self.controller._get_world_pose_safe(task["body_path"])
        task["target_pos"] = np.asarray(current_pos, dtype=np.float32)
        task["target_quat"] = np.asarray(current_quat, dtype=np.float32)
