"""独立的最小 VR 控制桥接层。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .mapping import VRMotionMapper, VRMotionMappingConfig
from .types import BimanualVRCommand, VRControllerState


@dataclass(slots=True)
class VRBridgeOutput:
    """独立 VR 桥接层的输出。"""

    ee_action: np.ndarray
    gripper_action: np.ndarray
    command: BimanualVRCommand


class VRBimanualBridge:
    """从双手柄状态构建适合 IK 使用的双臂动作向量。"""

    def __init__(self, mapper: VRMotionMapper | None = None):
        self.mapper = mapper or VRMotionMapper(VRMotionMappingConfig())

    def build(self, left_state: VRControllerState, right_state: VRControllerState) -> VRBridgeOutput:
        """根据左右手柄状态构建末端和夹爪动作。"""
        command = self.mapper.map_bimanual(left_state, right_state)
        ee_action = np.concatenate(
            [
                np.asarray(command.left.position, dtype=np.float32),
                np.asarray(command.left.quat_wxyz, dtype=np.float32),
                np.asarray(command.right.position, dtype=np.float32),
                np.asarray(command.right.quat_wxyz, dtype=np.float32),
            ]
        ).astype(np.float32)
        gripper_action = np.array([command.left.gripper, command.right.gripper], dtype=np.float32)
        return VRBridgeOutput(ee_action=ee_action, gripper_action=gripper_action, command=command)
