"""独立 VR 遥操作模块共用的数据结构。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np


ArmSide = Literal["left", "right"]


@dataclass(slots=True)
class VRControllerState:
    """单个 VR 手柄的位姿和模拟量按键状态。"""

    position: np.ndarray
    quat_wxyz: np.ndarray
    trigger: float = 0.0
    squeeze: float = 0.0
    thumbstick_x: float = 0.0
    thumbstick_y: float = 0.0
    button_0: float = 0.0
    button_1: float = 0.0


@dataclass(slots=True)
class EndEffectorTarget:
    """单侧末端执行器的目标位姿。"""

    side: ArmSide
    position: np.ndarray
    quat_wxyz: np.ndarray
    gripper: float = 0.0


@dataclass(slots=True)
class BimanualVRCommand:
    """由左右两个手柄生成的双臂组合命令。"""

    left: EndEffectorTarget
    right: EndEffectorTarget
    metadata: dict[str, float] = field(default_factory=dict)
