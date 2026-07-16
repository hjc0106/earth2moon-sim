"""独立 VR 遥操作辅助模块。

这个包故意不依赖 ``tiangong.teleop``，
这样在不加载 Isaac Sim 运行时的情况下也可以单独导入和测试。
"""

from .bridge import VRBimanualBridge, VRBridgeOutput
from .mapping import VRMotionMapper, VRMotionMappingConfig
from .r1pro_adapter import R1ProVRAdapter, R1ProVRAdapterConfig
from .types import BimanualVRCommand, EndEffectorTarget, VRControllerState

__all__ = [
    "BimanualVRCommand",
    "EndEffectorTarget",
    "R1ProVRAdapter",
    "R1ProVRAdapterConfig",
    "VRBimanualBridge",
    "VRBridgeOutput",
    "VRControllerState",
    "VRMotionMapper",
    "VRMotionMappingConfig",
]
