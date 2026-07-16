# Teleop Cameras

本文档说明当前天宫仿真里 4 个机器人对象的相机精简方案，以及现有键盘切换和无人机控制方式。

## 当前保留方案

当前场景中的 4 个机器人对象是：

- `ranger_arm`
- `r1pro`
- `cf2x`
- `cf2x_01`

相机保留规则如下：

- `ranger_arm` 保留 1 个相机
  - `head_top`
- `r1pro` 保留 3 个相机
  - `head_top`
  - `left_gripper`
  - `right_gripper`
- 两架无人机各保留 1 个相机
  - `chase`

总数为 6 个视角：

- `ranger_arm.head_top`
- `r1pro.head_top`
- `r1pro.left_gripper`
- `r1pro.right_gripper`
- `cf2x.chase`
- `cf2x_01.chase`

## 当前切换键

轮式机器人相机切换依赖当前激活机器人：

- `F1`：返回启动时的全场景视角
- `1`：切到 `ranger_arm`
- `2`：切到 `r1pro`
- `F6`：当前激活轮式机器人的 `head_top`
- `F3`：切到 `ranger_arm.head_top`，同时选择 Ranger VR 控制
- `F4`：切到 `r1pro.head_top`，同时选择 R1 Pro VR 控制

无人机相机直接绑定：

- `F9`：切到 `cf2x.chase`
- `F10`：切到 `cf2x_01.chase`

切到任一相机后，统一使用方向键微调当前镜头：

- `↑`：抬头
- `↓`：低头
- `←`：向左转
- `→`：向右转

当前实现里，方向键优先用于镜头调整；轮式机器人底盘移动统一使用 `W/A/S/D`。

## 当前实现位置

主逻辑在 [scripts/keyboard_teleop_ranger_arm.py](/home/zjz/workspace/tiangong/earth2moon-sim/scripts/keyboard_teleop_ranger_arm.py:1431)。

对应构建函数：

- `r1pro`：`_build_r1pro_camera_aliases`
- `ranger_arm`：`_build_ranger_camera_aliases`
- `cf2x` / `cf2x_01`：`_build_drone_camera_aliases`

当前无人机相机是跟随相机，不是机体内置传感器。
当前所有保留相机都支持“跟随 + 方向键微调视角”。

## 当前无人机控制

当前两架无人机已经接入一套独立于轮式机器人的轻量控制层。

控制模式切换：

- `3`：接管 `cf2x`
- `4`：接管 `cf2x_01`
- `F2`：退出无人机控制，回到当前地面机器人控制

无人机手动控制键位：

- `W/S`：前后
- `A/D`：左右
- `Q/E`：下降/上升
- `J/L`：偏航左/右

当前行为说明：

- 默认仍可按 CSV 轨迹运行
- 当按下 `3` 或 `4` 后，对应无人机会切换到手动接管
- 手动控制按机体当前朝向移动，不是固定世界坐标平移
- 手动飞行时会附带轻微俯仰/横滚姿态变化
- 旋翼会持续旋转，输入越大旋转越快

当前这套仍属于运动学控制，不是真实四旋翼动力学飞控。

## 现阶段边界

当前这版已经支持：

- 两架无人机按 CSV 轨迹运行
- 两架无人机独立相机切换
- 两架无人机键盘手动接管
- 无人机相机 rig 调试方块已移除

当前这版还不支持：

- 基于推力/质量/姿态闭环的真实四旋翼动力学飞控
- 在 `dispatcher` 内把无人机并入与轮式机器人完全相同的控制器激活语义
