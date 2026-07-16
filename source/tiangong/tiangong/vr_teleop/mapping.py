"""独立的 VR 手柄到末端目标的映射辅助模块。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .types import ArmSide, BimanualVRCommand, EndEffectorTarget, VRControllerState


def _normalize_quat(quat_wxyz: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat_wxyz, dtype=np.float32).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    return quat / norm


@dataclass(slots=True)
class VRMotionMappingConfig:
    """手柄到末端目标映射的配置。"""

    position_scale: float = 1.0
    gripper_deadband: float = 0.05


class VRMotionMapper:
    """把双手柄状态映射成双臂笛卡尔目标。"""

    def __init__(self, cfg: VRMotionMappingConfig | None = None):
        self.cfg = cfg or VRMotionMappingConfig()

    def map_controller(self, side: ArmSide, state: VRControllerState) -> EndEffectorTarget:
        """把单个手柄状态映射成单侧末端目标。"""
        position = np.asarray(state.position, dtype=np.float32).reshape(3) * float(self.cfg.position_scale)
        quat = _normalize_quat(state.quat_wxyz)
        gripper = self._map_gripper(state.trigger, state.squeeze)
        return EndEffectorTarget(side=side, position=position, quat_wxyz=quat, gripper=gripper)

    def map_bimanual(self, left_state: VRControllerState, right_state: VRControllerState) -> BimanualVRCommand:
        """把左右手柄状态组合成双臂命令。"""
        left_target = self.map_controller("left", left_state)
        right_target = self.map_controller("right", right_state)
        return BimanualVRCommand(
            left=left_target,
            right=right_target,
            metadata={
                "left_trigger": float(left_state.trigger),
                "left_squeeze": float(left_state.squeeze),
                "right_trigger": float(right_state.trigger),
                "right_squeeze": float(right_state.squeeze),
            },
        )

    def _map_gripper(self, trigger: float, squeeze: float) -> float:
        """把 trigger/squeeze 映射为单个夹爪控制值。

        约定：
        - `1.0` 表示张开
        - `-1.0` 表示闭合
        - trigger 负责夹爪闭合
        - squeeze 预留给上层做暂停/模式切换，不直接参与夹爪
        """
        trigger = float(np.clip(trigger, 0.0, 1.0))
        _ = float(np.clip(squeeze, 0.0, 1.0))
        if trigger < float(self.cfg.gripper_deadband):
            return 1.0
        return -1.0
