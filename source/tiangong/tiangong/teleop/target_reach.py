"""目标点与机器人靠近控制。

本模块提供一层机器人无关的目标点抽象：用 stage 中的小方块表示目标点，
当前阶段主循环只调用底座靠近目标区域；末端夹爪逼近代码保留给后续恢复。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from isaacsim.core.utils.types import ArticulationAction


@dataclass(frozen=True)
class TargetPoint:
    """一个可视化目标点。"""

    name: str
    prim_path: str
    position: np.ndarray
    color: tuple[float, float, float]


@dataclass
class TargetReachStatus:
    """一次目标逼近 step 的结果。"""

    active: bool
    robot_name: str | None = None
    arm_names: tuple[str, ...] = ()
    target_name: str | None = None
    min_distance: float | None = None
    reached: bool = False


@dataclass
class TargetBaseStatus:
    """当前机器人底座与目标点之间的实时状态。"""

    active: bool
    robot_name: str | None = None
    target_name: str | None = None
    robot_position: np.ndarray | None = None
    robot_quat: np.ndarray | None = None
    target_position: np.ndarray | None = None
    distance_xy: float | None = None
    within_distance: bool = False


def parse_target_points(spec: str, default_points: tuple[tuple[float, float, float], ...]) -> list[np.ndarray]:
    """解析 `x,y,z;x,y,z` 格式目标点；为空时返回默认点。"""
    if not spec.strip():
        return [np.array(point, dtype=np.float32) for point in default_points]
    points = []
    for raw_point in spec.split(";"):
        values = [float(value.strip()) for value in raw_point.split(",") if value.strip()]
        if len(values) != 3:
            raise ValueError(f"Invalid target point '{raw_point}'. Expected x,y,z.")
        points.append(np.array(values, dtype=np.float32))
    return points


class TargetMarkerManager:
    """在 USD stage 中创建和维护目标点小方块。"""

    DEFAULT_COLORS = (
        (1.0, 0.15, 0.1),
        (0.1, 0.65, 1.0),
        (0.2, 0.9, 0.35),
    )

    def __init__(self, stage, gf_module, usdgeom_module, usdphysics_module, root_path: str, marker_size: float):
        self._stage = stage
        self._Gf = gf_module
        self._UsdGeom = usdgeom_module
        self._UsdPhysics = usdphysics_module
        self.root_path = root_path.rstrip("/")
        self.marker_size = float(marker_size)
        self.targets: list[TargetPoint] = []

    def create_targets(self, positions: list[np.ndarray]) -> list[TargetPoint]:
        """按给定坐标创建目标小方块，并返回目标描述。"""
        self.targets = []
        self._UsdGeom.Xform.Define(self._stage, self.root_path)
        for index, position in enumerate(positions):
            name = f"target_{index + 1}"
            prim_path = f"{self.root_path}/{name}"
            color = self.DEFAULT_COLORS[index % len(self.DEFAULT_COLORS)]
            self._create_cube(prim_path, position, color)
            self.targets.append(TargetPoint(name=name, prim_path=prim_path, position=position, color=color))
        return list(self.targets)

    def _create_cube(self, prim_path: str, position: np.ndarray, color: tuple[float, float, float]) -> None:
        """创建或更新一个目标 marker：可选中的碰撞方块加竖直提示柱。"""
        self._define_colored_cube(
            prim_path,
            position,
            color,
            np.array([1.0, 1.0, 1.0], dtype=np.float32),
            collision_enabled=True,
        )
        self._define_colored_cube(
            f"{prim_path}/pillar",
            np.array([0.0, 0.0, self.marker_size * 1.75], dtype=np.float32),
            color,
            np.array([0.28, 0.28, 3.5], dtype=np.float32),
            collision_enabled=False,
        )

    def _define_colored_cube(
        self,
        prim_path: str,
        position: np.ndarray,
        color: tuple[float, float, float],
        scale: np.ndarray,
        collision_enabled: bool,
    ) -> None:
        """创建一个带 displayColor 的 cube prim。"""
        cube = self._UsdGeom.Cube.Define(self._stage, prim_path)
        prim = cube.GetPrim()
        cube.CreateSizeAttr(self.marker_size)
        gprim = self._UsdGeom.Gprim(prim)
        gprim.CreateDisplayColorAttr([self._Gf.Vec3f(*color)])
        gprim.CreateDisplayOpacityAttr([0.9])
        if collision_enabled:
            collision = self._UsdPhysics.CollisionAPI.Apply(prim)
            collision.CreateCollisionEnabledAttr(True)
        xformable = self._UsdGeom.Xformable(prim)
        try:
            xformable.ClearXformOpOrder()
        except Exception:
            pass
        translate_op = xformable.AddTranslateOp()
        scale_op = xformable.AddScaleOp()
        translate_op.Set(self._Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
        scale_op.Set(self._Gf.Vec3f(float(scale[0]), float(scale[1]), float(scale[2])))
        xformable.SetXformOpOrder([translate_op, scale_op], True)


class EndEffectorTargetReacher:
    """把当前激活机器人的末端 task 拉向目标点。"""

    def __init__(self, get_world_pose, carb, tolerance: float, max_step: float, hold_orientation: bool):
        self._get_world_pose = get_world_pose
        self._carb = carb
        self.tolerance = float(tolerance)
        self.max_step = float(max_step)
        self.hold_orientation = bool(hold_orientation)
        self._base_tracking_states: dict[tuple[str | None, str], dict[str, object]] = {}

    def step(self, controller, target: TargetPoint) -> TargetReachStatus:
        """对当前控制器执行一帧末端目标逼近。"""
        if controller is None or not getattr(controller, "available", False):
            return TargetReachStatus(active=False)
        tasks = self._active_tasks(controller)
        if not tasks:
            return TargetReachStatus(active=False, robot_name=getattr(controller, "name", None), target_name=target.name)

        target_position = self.target_position(target)
        min_distance = None
        moved = False
        for task in tasks:
            current_pos, current_quat = self._get_world_pose_safe(task["body_path"])
            current_pos = np.array(current_pos, dtype=np.float32)
            error = target_position - current_pos
            distance = float(np.linalg.norm(error))
            min_distance = distance if min_distance is None else min(min_distance, distance)
            if distance <= self.tolerance:
                continue
            step = error
            if self.max_step > 0.0 and distance > self.max_step:
                step = error / distance * self.max_step
            task["target_pos"] = current_pos + step
            if self.hold_orientation:
                task["target_quat"] = np.array(current_quat, dtype=np.float32)
            moved = True

        if moved:
            self._apply_controller_targets(controller, tasks)

        return TargetReachStatus(
            active=True,
            robot_name=getattr(controller, "name", None),
            arm_names=tuple(str(task.get("side", "unknown")) for task in tasks),
            target_name=target.name,
            min_distance=min_distance,
            reached=min_distance is not None and min_distance <= self.tolerance,
        )

    def target_position(self, target: TargetPoint) -> np.ndarray:
        """读取目标 marker 当前世界坐标，支持用户用鼠标拖动后实时更新。"""
        position, _ = self._get_world_pose_safe(target.prim_path)
        return np.array(position, dtype=np.float32)

    def _yaw_from_quat(self, quat) -> float:
        """从 wxyz 四元数提取绕世界 Z 轴的 yaw。"""
        quat = np.asarray(quat, dtype=np.float32)
        if quat.size < 4:
            return 0.0
        w, x, y, z = [float(value) for value in quat[:4]]
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def _world_delta_to_local_xy(self, delta_xy: np.ndarray, root_quat) -> np.ndarray:
        """把世界 XY 方向转换成机器人底座本地 forward/side 方向。"""
        yaw = self._yaw_from_quat(root_quat)
        cos_yaw = float(np.cos(yaw))
        sin_yaw = float(np.sin(yaw))
        return np.array(
            [
                cos_yaw * float(delta_xy[0]) + sin_yaw * float(delta_xy[1]),
                -sin_yaw * float(delta_xy[0]) + cos_yaw * float(delta_xy[1]),
            ],
            dtype=np.float32,
        )

    def _wrap_angle(self, angle: float) -> float:
        """把角度规整到 [-pi, pi]。"""
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _arm_front_sign(self, controller, status: TargetBaseStatus) -> float:
        """根据末端夹爪相对底座的位置判断双臂朝向是本地 +X 还是 -X。"""
        offsets = []
        for task in self._active_tasks(controller):
            body_path = task.get("body_path")
            if not body_path:
                continue
            try:
                ee_position, _ = self._get_world_pose_safe(body_path)
            except Exception:
                continue
            ee_position = np.asarray(ee_position, dtype=np.float32)
            local_offset = self._world_delta_to_local_xy(ee_position[:2] - status.robot_position[:2], status.robot_quat)
            if float(np.linalg.norm(local_offset)) > 0.05:
                offsets.append(local_offset)
        if not offsets:
            return 1.0
        mean_offset = np.mean(np.stack(offsets), axis=0)
        if abs(float(mean_offset[0])) < 0.05:
            return 1.0
        return 1.0 if float(mean_offset[0]) >= 0.0 else -1.0

    def _update_base_tracking_state(self, controller, target: TargetPoint, status: TargetBaseStatus, heading_error: float):
        """根据上一帧效果自适应修正底盘前进和转向符号。"""
        key = (getattr(controller, "name", None), target.name)
        state = self._base_tracking_states.setdefault(
            key,
            {
                "forward_sign": 1.0,
                "yaw_sign": 1.0,
                "distance_worse_count": 0,
                "heading_worse_count": 0,
                "distance_worse_total": 0.0,
                "heading_worse_total": 0.0,
                "last_distance": None,
                "last_heading_error": None,
                "last_target_position": None,
                "last_forward": 0.0,
                "last_yaw": 0.0,
            },
        )
        last_distance = state.get("last_distance")
        last_heading_error = state.get("last_heading_error")
        last_target_position = state.get("last_target_position")
        target_shift = 0.0
        if last_target_position is not None and status.target_position is not None:
            target_shift = float(np.linalg.norm(status.target_position[:2] - np.asarray(last_target_position)[:2]))
        target_is_stable = target_shift < 0.05

        if last_distance is not None and target_is_stable and abs(float(state.get("last_forward", 0.0))) > 0.05:
            distance_delta = float(status.distance_xy) - float(last_distance)
            if distance_delta > 0.001:
                state["distance_worse_count"] = int(state.get("distance_worse_count", 0)) + 1
                state["distance_worse_total"] = float(state.get("distance_worse_total", 0.0)) + distance_delta
            elif distance_delta < -0.001:
                state["distance_worse_count"] = 0
                state["distance_worse_total"] = 0.0
            if int(state.get("distance_worse_count", 0)) >= 45 or float(state.get("distance_worse_total", 0.0)) > 0.08:
                state["forward_sign"] = -float(state.get("forward_sign", 1.0))
                state["distance_worse_count"] = 0
                state["distance_worse_total"] = 0.0

        if last_heading_error is not None and target_is_stable and abs(float(state.get("last_yaw", 0.0))) > 0.05:
            heading_delta = abs(float(heading_error)) - abs(float(last_heading_error))
            if heading_delta > 0.001:
                state["heading_worse_count"] = int(state.get("heading_worse_count", 0)) + 1
                state["heading_worse_total"] = float(state.get("heading_worse_total", 0.0)) + heading_delta
            elif heading_delta < -0.001:
                state["heading_worse_count"] = 0
                state["heading_worse_total"] = 0.0
            if int(state.get("heading_worse_count", 0)) >= 45 or float(state.get("heading_worse_total", 0.0)) > 0.08:
                state["yaw_sign"] = -float(state.get("yaw_sign", 1.0))
                state["heading_worse_count"] = 0
                state["heading_worse_total"] = 0.0
        return state

    def base_command(self, controller, target: TargetPoint, desired_distance: float, max_command: float):
        """根据目标和机器人根节点 XY 距离生成底座靠近命令。

        当前阶段只让底座进入目标区域，机械臂自动触及目标暂不调用。
        """
        status = self.base_status(controller, target, desired_distance)
        if not status.active or status.distance_xy is None:
            return 0.0, 0.0, 0.0
        delta_xy = status.target_position[:2] - status.robot_position[:2]
        distance_xy = float(status.distance_xy)
        if distance_xy < 1e-6:
            return 0.0, 0.0, 0.0
        local_delta = self._world_delta_to_local_xy(delta_xy, status.robot_quat)
        local_direction = local_delta / distance_xy
        arm_front_sign = self._arm_front_sign(controller, status)
        target_angle = float(np.arctan2(float(local_direction[1]), float(local_direction[0])))
        front_angle = 0.0 if arm_front_sign >= 0.0 else float(np.pi)
        heading_error = self._wrap_angle(target_angle - front_angle)
        abs_heading = abs(heading_error)
        heading_tolerance = 0.22 if getattr(controller, "name", "") == "ranger_arm" else 0.3
        if status.within_distance and abs_heading <= heading_tolerance:
            return 0.0, 0.0, 0.0

        distance_error = max(distance_xy - float(desired_distance), 0.0)
        command_scale = min(distance_error / max(float(desired_distance), 1e-6), 1.0)
        command_scale *= max(float(max_command), 0.0)
        if not status.within_distance:
            command_scale = max(command_scale, min(float(max_command) * 0.35, 0.18))
        state = self._update_base_tracking_state(controller, target, status, heading_error)
        yaw_scale = command_scale if not status.within_distance else max(float(max_command) * 0.45, 0.15)
        yaw = float(np.clip(heading_error / (np.pi * 0.5), -1.0, 1.0) * yaw_scale)
        yaw *= float(state.get("yaw_sign", 1.0))

        if getattr(controller, "name", "") == "ranger_arm":
            drive_direction = arm_front_sign * float(state.get("forward_sign", 1.0))
            steering_direction = 1.0 if drive_direction >= 0.0 else -1.0
            yaw = float(np.clip(heading_error / 0.65, -1.0, 1.0) * steering_direction)
            yaw *= float(state.get("yaw_sign", 1.0))
            heading_scale = max(float(np.cos(min(abs_heading, np.pi * 0.5))), 0.25)
            if abs_heading > np.pi * 0.65:
                heading_scale = 0.18
            if status.within_distance:
                forward_magnitude = 0.0
            else:
                forward_magnitude = command_scale
            forward = forward_magnitude * heading_scale * drive_direction
            state["last_forward"] = float(forward)
            state["last_yaw"] = float(yaw)
            state["last_distance"] = float(status.distance_xy)
            state["last_heading_error"] = float(heading_error)
            state["last_target_position"] = np.array(status.target_position, dtype=np.float32)
            return float(forward), 0.0, yaw

        translation_scale = 0.35 if abs_heading > 1.0 else 1.0
        if status.within_distance:
            translation_scale = 0.0
        forward = float(local_direction[0] * command_scale * translation_scale * float(state.get("forward_sign", 1.0)))
        strafe = float(local_direction[1] * command_scale * translation_scale * float(state.get("forward_sign", 1.0)))
        state["last_forward"] = float(np.linalg.norm([forward, strafe]))
        state["last_yaw"] = float(yaw)
        state["last_distance"] = float(status.distance_xy)
        state["last_heading_error"] = float(heading_error)
        state["last_target_position"] = np.array(status.target_position, dtype=np.float32)
        return forward, strafe, yaw

    def base_status(self, controller, target: TargetPoint, desired_distance: float) -> TargetBaseStatus:
        """读取机器人底座与目标点的实时坐标和 XY 距离。"""
        root_path = getattr(controller, "scene_root_path", None) or getattr(controller, "root_path", None)
        if not root_path:
            return TargetBaseStatus(active=False, robot_name=getattr(controller, "name", None), target_name=target.name)
        if hasattr(controller, "get_base_world_pose"):
            base_position, base_quat = controller.get_base_world_pose()
        else:
            base_position, base_quat = self._get_world_pose_safe(root_path)
        base_position = np.array(base_position, dtype=np.float32)
        base_quat = np.array(base_quat, dtype=np.float32)
        target_position = self.target_position(target)
        delta_xy = target_position[:2] - base_position[:2]
        distance_xy = float(np.linalg.norm(delta_xy))
        return TargetBaseStatus(
            active=True,
            robot_name=getattr(controller, "name", None),
            target_name=target.name,
            robot_position=base_position,
            robot_quat=base_quat,
            target_position=target_position,
            distance_xy=distance_xy,
            within_distance=distance_xy <= max(float(desired_distance), 0.0),
        )

    def _get_world_pose_safe(self, prim_path: str):
        """优先使用 fabric 查询末端位姿，失败时回退普通查询。"""
        try:
            return self._get_world_pose(prim_path, fabric=True)
        except Exception:
            return self._get_world_pose(prim_path)

    def _active_tasks(self, controller):
        """读取控制器当前生效的左/右臂 task。"""
        if hasattr(controller, "_active_tasks"):
            return list(controller._active_tasks())
        return list(getattr(controller, "arm_ik_tasks", []))

    def _apply_controller_targets(self, controller, tasks) -> None:
        """根据控制器类型调用已有 IK 应用逻辑。"""
        if getattr(controller, "name", "") == "ranger_arm" and getattr(controller, "ik_solver", None) is not None:
            controller.ik_solver.apply_targets(tasks, use_orientation=self.hold_orientation)
            return
        if getattr(controller, "name", "") == "r1pro":
            self._apply_r1pro_targets(controller, tasks)
            return
        self._carb.log_warn(f"Target reach is not supported for controller: {getattr(controller, 'name', None)}")

    def _apply_r1pro_targets(self, controller, tasks) -> None:
        """驱动 R1 Pro 末端靠近目标。

        R1 Pro 当前主控制路径是 direct_joint_control，因此优先复用它的
        关节增量映射；只有关闭 direct_joint_control 时才走预留 Jacobian IK。
        """
        if getattr(controller, "direct_joint_control", False):
            for task in tasks:
                current_pos, _ = self._get_world_pose_safe(task["body_path"])
                current_pos = np.array(current_pos, dtype=np.float32)
                delta_xyz = np.asarray(task["target_pos"], dtype=np.float32) - current_pos
                controller._update_direct_arm_joint_targets([task], delta_xyz, None, 0.0)
            controller._hold_direct_arm_joint_targets(tasks)
            return

        jacobians = controller.articulation._articulation_view.get_jacobians()
        if jacobians is None:
            return
        jacobians = np.asarray(jacobians)
        for task in tasks:
            current_pos, current_quat = self._get_world_pose_safe(task["body_path"])
            current_pos = np.array(current_pos, dtype=np.float32)
            current_quat = np.array(current_quat, dtype=np.float32)
            position_error = task["target_pos"] - current_pos
            orientation_error = controller._axis_angle_error(current_quat, task["target_quat"])
            ik_error = np.concatenate([position_error, orientation_error]).astype(np.float32)
            jacobian = jacobians[0, task["jacobian_body_index"], 0:6, :][:, task["jacobian_joint_columns"]]
            if jacobian.shape[0] != ik_error.shape[0]:
                continue
            joint_pos = controller.articulation.get_joint_positions(task["joint_indices"])
            delta_joint_pos = controller._damped_least_squares_delta(jacobian, ik_error)
            joint_target = controller._clamp_joint_positions(task["joint_indices"], joint_pos + delta_joint_pos)
            task["joint_target"] = np.array(joint_target, dtype=np.float32)
            controller.articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.array(task["joint_target"], dtype=np.float32),
                    joint_indices=task["joint_indices"],
                )
            )


class TargetReachCoordinator:
    """管理目标点、当前目标索引和自动逼近开关。"""

    def __init__(self, targets: list[TargetPoint], reacher: EndEffectorTargetReacher, enabled: bool):
        self.targets = targets
        self.reacher = reacher
        self.enabled = bool(enabled)
        self.active_index = 0
        self._last_reached = False

    @property
    def active_target(self) -> TargetPoint | None:
        """返回当前选中的目标点。"""
        if not self.targets:
            return None
        return self.targets[self.active_index]

    def set_active_index(self, index: int) -> TargetPoint | None:
        """切换当前目标点。"""
        if not self.targets:
            return None
        self.active_index = max(0, min(int(index), len(self.targets) - 1))
        self._last_reached = False
        return self.active_target

    def toggle_enabled(self) -> bool:
        """开关自动目标逼近模式。"""
        self.enabled = not self.enabled
        self._last_reached = False
        return self.enabled

    def step(self, controller) -> TargetReachStatus:
        """若已启用，则对当前目标执行一帧逼近。"""
        target = self.active_target
        if not self.enabled or target is None:
            return TargetReachStatus(active=False)
        status = self.reacher.step(controller, target)
        if status.reached and not self._last_reached:
            self._last_reached = True
        elif not status.reached:
            self._last_reached = False
        return status

    def base_command(self, controller, desired_distance: float, max_command: float):
        """为当前目标生成底座靠近命令。"""
        target = self.active_target
        if not self.enabled or target is None:
            return 0.0, 0.0, 0.0
        return self.reacher.base_command(controller, target, desired_distance, max_command)

    def base_status(self, controller, desired_distance: float) -> TargetBaseStatus:
        """返回当前目标下机器人底座与目标点的实时状态。"""
        target = self.active_target
        if not self.enabled or target is None:
            return TargetBaseStatus(active=False)
        return self.reacher.base_status(controller, target, desired_distance)

    def current_target_position(self) -> np.ndarray | None:
        """读取当前目标 marker 的实时世界坐标。"""
        target = self.active_target
        if target is None:
            return None
        return self.reacher.target_position(target)
