"""遥操作命令分发模块。

键盘入口只生成统一的底盘和机械臂命令；本模块负责把命令交给当前激活的
机器人控制器，从而让 Ranger Arm 与 R1 Pro 共享一套按键语义。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class BaseMotionCommand:
    """底盘运动命令：前后、横移、升降和偏航。"""

    forward: float = 0.0
    strafe: float = 0.0
    lift: float = 0.0
    yaw: float = 0.0


@dataclass
class ManipulatorMotionCommand:
    """机械臂/躯干/夹爪命令：末端平移旋转、躯干、腕部和夹爪增量。"""

    delta_xyz: object | None = None
    delta_rot: object | None = None
    joint_delta: object | None = None
    torso_delta: object | None = None
    joint7_delta: float = 0.0
    gripper_delta: float = 0.0


class TeleopDispatcher:
    """维护可用控制器列表，并把每帧命令分发给当前激活控制器。"""

    def __init__(self, controllers):
        self._controllers = {controller.name: controller for controller in controllers if controller is not None}
        self._active_name = next(iter(self._controllers), None)

    @property
    def active_name(self) -> str | None:
        return self._active_name

    def names(self) -> list[str]:
        """返回已注册控制器名称。"""
        return list(self._controllers)

    def has_controller(self, name: str) -> bool:
        """检查指定控制器是否存在且可用。"""
        controller = self._controllers.get(name)
        return bool(controller and controller.available)

    def set_active(self, name: str) -> bool:
        """切换当前激活控制器，失败时保持原状态。"""
        if not self.has_controller(name):
            return False
        self._active_name = name
        return True

    def active_controller(self):
        """返回当前激活控制器对象。"""
        if self._active_name is None:
            return None
        return self._controllers.get(self._active_name)

    def cycle_target_mode(self) -> str | None:
        """委托当前控制器切换左右臂/双臂目标。"""
        controller = self.active_controller()
        if controller is None:
            return None
        return controller.cycle_target_mode()

    def cycle_active(self) -> str | None:
        """在所有可用机器人控制器之间循环切换。"""
        available_names = [name for name, controller in self._controllers.items() if controller.available]
        if not available_names:
            return None
        if self._active_name not in available_names:
            self._active_name = available_names[0]
            return self._active_name
        current_index = available_names.index(self._active_name)
        self._active_name = available_names[(current_index + 1) % len(available_names)]
        return self._active_name

    def step(self, base_command: BaseMotionCommand, manipulator_command: ManipulatorMotionCommand) -> None:
        """把统一命令发送给当前激活控制器执行。"""
        controller = self.active_controller()
        if controller is None:
            return
        controller.step(base_command, manipulator_command)

    def active_joint_position_snapshot(self) -> dict[str, float]:
        """读取当前激活机器人所有 DOF 的实时关节位置。"""
        controller = self.active_controller()
        if controller is None or not hasattr(controller, "joint_position_snapshot"):
            return {}
        return controller.joint_position_snapshot()

    def active_robot_feedback_snapshot(self) -> dict:
        """读取当前激活机器人的位姿、关节位置和关节力矩反馈。"""
        controller = self.active_controller()
        if controller is None or not hasattr(controller, "robot_feedback_snapshot"):
            return {}
        return controller.robot_feedback_snapshot()
