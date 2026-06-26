# 更新日志

在此记录实际的代码、脚本、资产与文档变更。纯规划讨论不写入本文件。

## 2026-06-26 — 旋翼无人机 6-DOF 物理飞控

### 概述

将四旋翼控制从纯运动学（teleport prim）升级为基于 Isaac Sim PhysX 的 6-DOF 物理飞控，采用级联 SE3 几何控制器实现轨迹跟踪，并新增轨迹可视化、APF 自主避障框架与慢速椭圆测试轨迹。

### 物理飞控实现原理

**控制器架构：级联 SE3 几何控制器（Lee et al. 2010）**

1. **位置环（PD）**：计算期望加速度
   - `a_des = Kp_pos × (setpoint_pos − pos) + Kd_pos × (setpoint_vel − vel) + gravity`
   - 期望推力向量 `F_des = mass × a_des`

2. **姿态期望构造**：从 `F_des` 方向确定期望 body z 轴 `b3_des = F_des / |F_des|`，结合期望 yaw 构造完整期望旋转矩阵 `R_des`

3. **姿态环（SO(3) PD）**：
   - 旋转误差 `e_R = ½(R_desᵀR − RᵀR_des)^∨`
   - 角速度误差 `e_Ω = Ω_body − RᵀR_des·Ω_des`
   - 体力矩 `τ_body = −K_att·e_R − K_omega·e_Ω`，clip 到 ±max_torque

4. **推力分配**：`thrust = clip(F_des · b3_current, 0, max_thrust)`，沿当前 body z 轴施加世界系力

5. **翻转恢复**：当 `b3_z < 0`（body 倒置）时施加世界系 z 轴恢复推力 + 恢复力矩

**PhysX 施力路径**：
- 优先使用 `omni.physx.get_physx_simulation_interface().apply_force_at_pos(stage_id, body_token, force, body_pos, "Force")` + `apply_torque()`
- body_token 通过 `PhysicsSchemaTools.sdfPathToInt(path)` 编码，stage_id 通过 `UsdUtils.StageCache.Get().GetId(stage).ToLongInt()` 获取
- 备选路径：`RigidPrim.apply_forces_and_torques_at_pos()`

**关键工程问题与修复**：
- 移除 cf2x 的 `ArticulationRootAPI`，使 body 成为纯自由刚体，外力直接生效
- 设 root prim（而非 body）的 USD transform 到轨迹起点，避免 joint body transform 不一致导致 PhysX snap 冲击翻转机体
- Isaac Sim 5.1 类名为 `RigidPrim`（非 `RigidPrimView`）
- `PhysicsSchemaTools` 从 `pxr` 命名空间导入
- 施力移到 `my_world.step()` 之前，力在同帧物理积分生效
- 4 个旋翼桨叶关闭碰撞，仅保留 body 碰撞，避免自碰撞弹飞

### 代码修改内容

**`source/tiangong/tiangong/teleop/quadrotor_controller.py`**：
- 新增 `QuadrotorPhysicsTrajectoryController` 类（~450 行），接口与原 `QuadrotorTrajectoryController` 一致，可被同一个 `QuadrotorFleetController` 包装
- 新增 `sample_quadrotor_trajectory_state()` 函数，从航点列表采样位置/速度/yaw/yaw_rate
- `_enable_asset_physics()`：移除 ArticulationRootAPI，启用 body 刚体物理 + 重力，关闭桨叶碰撞
- `_set_initial_usd_pose()`：设 root prim transform 到轨迹起点
- `_add_rotor_drives()`：用 PhysicsDriveAPI 给 4 个旋翼加角速度驱动（视觉旋转）
- `_read_dynamics_parameters()`：从 USD 读取 body 质量/惯量，计算 max_thrust = 4×mass×g
- `_compute_control()`：SE3 级联控制器核心
- `_apply_wrench_physx()`：通过 omni.physx 底层接口施加世界系力/力矩
- `_create_trajectory_visualization()`：每架无人机独立的橙色（预期）+ 绿色（实际）轨迹线
- APF 避障框架：`_init_obstacle_camera()` / `_depth_to_world_points()` / `_compute_apf_repulsion()`
- 新增 `initialize_physics_view()` / `reset_to_setpoint()` 公共方法

**`source/tiangong/tiangong/teleop/__init__.py`**：
- 导出 `QuadrotorPhysicsTrajectoryController` 和 `sample_quadrotor_trajectory_state`

**`scripts/keyboard_teleop_ranger_arm.py`**：
- 新增 `--quadrotor-physics/--no-quadrotor-physics`（默认开）及增益调参 CLI
- 新增 `--quadrotor-obstacle-avoidance` 及 APF 参数 CLI
- finalize 块：`play()` → `initialize_physics_view()` → `reset_to_setpoint()` → step 稳定
- 主循环：`update()` 移到 `step()` 之前，力在同帧生效
- 测试障碍物：`--quadrotor-obstacle-avoidance` 时在轨迹中段生成 0.3m kinematic 立方体

**`assets/trajectories/quadrotor/1-4.csv`**：
- 改为慢速椭圆轨迹（0.37 m/s，26s，8 段圆弧每段 3s + 2s 起始悬停）
- 4 条轨迹分别覆盖东北/西北/东南/西南象限，空间分离互不交叉

**`README.md` / `docs/ROBOT_OPERATION_MANUAL.md`**：
- 更新物理飞控说明、CLI 参数表、测试命令

### 验证

- 纯 numpy 闭环刚体仿真：轨迹 1 两圈循环，最大跟踪误差 0.085 m，最大倾角 4.6°
- Isaac Sim 实测：4 机同时沿 4 条椭圆轨迹飞行，橙色预期线与绿色实际线基本重合，循环后稳定悬停
- 诊断日志确认：`physics_handle_valid=True`，`via=PhysX`，悬停推力 0.277N

### 测试命令

```bash
# 单机
bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --add-quadrotor --quadrotor-count 1 \
  --quadrotor-trajectory-dir assets/trajectories/quadrotor \
  --quadrotor-scale 1.0 --quadrotor-loop-trajectory

# 4 机编队
bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --add-quadrotor --quadrotor-count 4 \
  --quadrotor-trajectory-dir assets/trajectories/quadrotor \
  --quadrotor-scale 1.0 --quadrotor-loop-trajectory \
  --no-show-quadrotor-camera-window
```

### 备注

- APF 自主避障已实现但暂时关闭（`--quadrotor-obstacle-avoidance` 可开启），深度相机检测场景结构产生干扰需进一步调优
- 物理飞控建议 `--quadrotor-scale 1.0`，动力学使用 USD 标注的 Crazyflie 质量/惯量（~28g）


## 2026-06-24

- 摘要：暴露四旋翼数量参数，并将编队轨迹改为从目录按序号读取。
- 文件：
  - `source/tiangong/tiangong/teleop/quadrotor_controller.py`
  - `scripts/keyboard_teleop_ranger_arm.py`
  - `assets/trajectories/quadrotor/1.csv`
  - `assets/trajectories/quadrotor/2.csv`
  - `assets/trajectories/quadrotor/3.csv`
  - `assets/trajectories/quadrotor/4.csv`
  - `README.md`
  - `update.md`
- 验证：已通过 `python -m compileall scripts source/tiangong/tiangong`。
- 备注：使用 `--quadrotor-count N` 与 `--quadrotor-trajectory-dir assets/trajectories/quadrotor`，脚本按 `{dir}/1.csv` … `{dir}/N.csv` 加载轨迹。已移除 `--quadrotor-trajectory` 与 `--quadrotor-trajectories`。

- 摘要：新增四旋翼编队模式，包含分离的俯瞰轨迹，以及合并显示全部机载相机的外部 Matplotlib RGB-D 窗口。
- 文件：
  - `source/tiangong/tiangong/teleop/quadrotor_controller.py`
  - `source/tiangong/tiangong/teleop/__init__.py`
  - `scripts/keyboard_teleop_ranger_arm.py`
  - `assets/trajectories/quadrotor_demo_1.csv`
  - `assets/trajectories/quadrotor_demo_2.csv`
  - `assets/trajectories/quadrotor_demo_3.csv`
  - `assets/trajectories/quadrotor_demo_4.csv`
  - `README.md`
  - `update.md`
- 验证：已通过 `python -m compileall scripts source/tiangong/tiangong`。
- 备注：`--quadrotor-count` 默认为 `4`；编号轨迹分别在东北/西北/东南/西南象限俯瞰飞行。外部窗口以 4×2 网格显示 RGB/Depth；四旋翼遥操作激活时按 `F6` 可在 IsaacSim 视口中循环切换相机。

- 摘要：提高示例四旋翼轨迹飞行高度，并新增可配置的相机下俯角，用于俯瞰 RGB-D 观测。
- 文件：
  - `assets/trajectories/quadrotor_demo.csv`
  - `source/tiangong/tiangong/teleop/quadrotor_controller.py`
  - `scripts/keyboard_teleop_ranger_arm.py`
  - `README.md`
  - `update.md`
- 验证：已通过 `python -m compileall scripts source/tiangong/tiangong`。
- 备注：示例轨迹 `z` 由约 `-7.6` 调整为 `-0.5`（相对 `--ground-z -10.0` 约 9.5 m）。新增 `--quadrotor-camera-pitch`，默认 `90.0` 垂直向下俯瞰；低高度前视飞行可设为 `0`。

- 摘要：收紧外部查看器逻辑，仅在收到实时四旋翼相机 RGB/Depth 帧时更新，并在出现有效帧时明确打日志。
- 文件：
  - `source/tiangong/tiangong/teleop/quadrotor_controller.py`
  - `scripts/keyboard_teleop_ranger_arm.py`
- 验证：已通过 `python -m compileall scripts source/tiangong/tiangong`。
- 备注：Matplotlib 窗口在 `/World/quadrotor/rgbd_camera/Camera` 产出有效 RGB 或 depth 帧之前保持等待状态，随后记录 `displaying live camera frames`。

- 摘要：外部四旋翼相机查看器在启动阶段收到空帧时不再自行禁用，改为继续等待有效帧。
- 文件：
  - `source/tiangong/tiangong/teleop/quadrotor_controller.py`
- 验证：已通过 `python -m compileall scripts source/tiangong/tiangong`；用户日志显示查看器在收到空 `(0,)` 帧前已成功初始化。
- 备注：查看器会等待合法的 H×W×3 RGB 与 H×W depth 帧后再更新 Matplotlib 图像。

- 摘要：当 Matplotlib 默认后端不可交互时，强制外部四旋翼查看器使用 GUI 后端。
- 文件：
  - `source/tiangong/tiangong/teleop/quadrotor_controller.py`
  - `scripts/keyboard_teleop_ranger_arm.py`
- 验证：已通过 `python -m compileall scripts source/tiangong/tiangong`；本地环境检查显示修复前 Matplotlib 默认为无窗口 `agg` 后端。
- 备注：查看器会尝试 `TkAgg`，使用 `/tmp/matplotlib` 作为 Matplotlib 缓存目录，并在跳过时记录日志。

- 摘要：新增四旋翼相机的外部 Matplotlib RGB-D 查看器，同时保留 IsaacSim 视口用于场景展示。
- 文件：
  - `source/tiangong/tiangong/teleop/quadrotor_controller.py`
  - `source/tiangong/tiangong/teleop/__init__.py`
  - `scripts/keyboard_teleop_ranger_arm.py`
  - `README.md`
  - `docs/ROBOT_OPERATION_MANUAL.md`
- 验证：已通过 `python -m compileall scripts source/tiangong/tiangong`；IsaacSim 相机冒烟测试仍待在本地 GUI 会话中完成。
- 备注：默认通过 `--show-quadrotor-camera-window` 启用外部窗口；可用 `--no-show-quadrotor-camera-window` 关闭。

- 摘要：启动后支持自动在 IsaacSim 视口中可视化四旋翼 RGB-D 相机。
- 文件：
  - `scripts/keyboard_teleop_ranger_arm.py`
  - `README.md`
  - `docs/ROBOT_OPERATION_MANUAL.md`
- 验证：IsaacSim 视口冒烟测试待完成；代码已通过现有脚本编译检查。
- 备注：使用 `--no-show-quadrotor-camera` 可保持原视口相机；也可按 `F6` 手动切换到当前机器人相机。

- 摘要：从 IsaacSim 资产源刷新项目内 Crazyflie 四旋翼 USD 目录。
- 文件：
  - `assets/Assets/Isaac/5.1/Isaac/Robots/Bitcraze/Crazyflie/`
- 验证：刷新后确认 `cf2x.usd` 与 `configuration/cf2x_robot_schema.usd` 存在；目录大小为 `312K`。
- 备注：以覆盖方式从 `/home/qylab/hjc_space/data/issac_lab/isaacsim_assets/Assets/Isaac/5.1/Isaac/Robots/Bitcraze/Crazyflie` 同步。

- 摘要：将 IsaacSim Crazyflie 四旋翼 USD 资产复制到项目内 `assets/` 目录。
- 文件：
  - `assets/Assets/Isaac/5.1/Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd`
  - `assets/Assets/Isaac/5.1/Isaac/Robots/Bitcraze/Crazyflie/configuration/cf2x_robot_schema.usd`
  - `assets/Assets/Isaac/5.1/Isaac/Robots/Bitcraze/Crazyflie/.thumbs/`
- 验证：确认项目资产下已存在复制的 `cf2x.usd`，目录大小为 `312K`。
- 备注：默认 `CF2X_ASSET_PATH` 已指向该项目内路径，因此默认四旋翼 USD 无需改代码。

- 摘要：完成四旋翼集成初版，包括 kinematic 轨迹跟踪、RGB-D 相机 prim 搭建与示例轨迹。
- 文件：
  - `source/tiangong/tiangong/teleop/quadrotor_controller.py`
  - `source/tiangong/tiangong/teleop/__init__.py`
  - `scripts/keyboard_teleop_ranger_arm.py`
  - `assets/trajectories/quadrotor_demo.csv`
  - `README.md`
  - `docs/ROBOT_OPERATION_MANUAL.md`
  - `docs/CALL_FLOW.md`
  - `update.md`
- 验证：已通过 `python -m compileall scripts source/tiangong/tiangong`；已通过 `assets/trajectories/quadrotor_demo.csv` 的独立 CSV 轨迹解析检查。
- 备注：四旋翼 v1 为 kinematic 模式，不模拟旋翼动力学、ROS 发布或图像落盘。IsaacSim 冒烟测试仍待在本地 IsaacSim 会话（需 Crazyflie 资产可用）中完成。
