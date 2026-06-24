# Ranger Arm 机械臂控制说明

本文档说明当前 `Ranger Arm` 双臂控制方式、7 自由度关节命名，以及保留下来的 IK 接口位置。

## 当前控制方式

当前 `Ranger Arm` 机械臂已经调整为和 `R1 Pro` 一致的控制思路：

- 保留 `IK` 求解模块和相关接口
- 但日常遥操作默认不再直接用 Jacobian IK 驱动双臂
- 实际控制改为：直接更新每个机械臂的 7 个关节目标位置

也就是说，现在键盘输入直接对应 7 个手臂关节，再持续发送关节位置目标。

这样做的好处是：

- 行为更稳定
- 更接近 `R1 Pro` 当前控制方式
- 不会出现末端目标被反复重置、手臂收敛不稳定的问题

## 调用链

当前主调用链如下：

1. `scripts/keyboard_teleop_ranger_arm.py` 加载天宫场景，并创建 `RangerArmTeleopController`
2. `RangerArmTeleopController` 负责：
   - Ranger Arm 底盘轮速/转向
   - 左臂 / 右臂 / 双臂切换
   - 双臂 7 关节目标更新
   - 夹爪目标更新
3. `RangerArmIKSolver` 仍然会初始化并维护：
   - 左右臂 task
   - 末端 body 路径
   - Jacobian 索引
   - 夹爪限制
   - 当前末端目标缓存
4. 但正常遥操作时，`RangerArmTeleopController.step()` 直接更新 `joint_target`
5. 然后持续发送：

```text
ArticulationAction(joint_positions=joint_target, joint_indices=arm_joint_indices)
```

## 机械臂关节

Ranger Arm 左右机械臂都按 7 自由度组织：

### 左臂 7 关节

```text
arm_left_joint1
arm_left_joint2
arm_left_joint3
arm_left_joint4
arm_left_joint5
arm_left_joint6
arm_left_joint7
```

### 右臂 7 关节

```text
arm_right_joint1
arm_right_joint2
arm_right_joint3
arm_right_joint4
arm_right_joint5
arm_right_joint6
arm_right_joint7
```

这些关节通过 `SingleArticulation.get_dof_index()` 解析为 articulation DOF 索引，并缓存到 task 中。

## 夹爪关节

每侧夹爪包含两个 DOF：

```text
gripper_left_joint
gripper_left_joint_mimic

gripper_right_joint
gripper_right_joint_mimic
```

夹爪依旧走位置驱动，受以下参数控制：

```text
--gripper-stiffness
--gripper-damping
--gripper-max-effort
--gripper-open-position
--gripper-closed-position
```

## 键盘输入与 7 关节映射

当前键盘输入仍然保留原有语义：

- 底盘：`W/S` 前后，`A/D` 左右转向
- `I/K`：第 1 关节
- `J/L`：第 2 关节
- `U/O`：第 3 关节
- `T/G`：第 4 关节
- `F/H`：第 5 关节
- `R/Y`：第 6 关节
- `3/4`：第 7 关节
- `M/N`：夹爪开合

底层执行方式现在就是直接生成 7 关节增量，不再经过末端 XYZ / 姿态再换算一次。

当前直接关节控制映射逻辑位于：

[ranger_arm_controller.py](/home/zjz/workspace/tiangong/tiangong/source/tiangong/tiangong/teleop/ranger_arm_controller.py)

函数：

- `_update_direct_arm_joint_targets()`
- `_hold_direct_arm_joint_targets()`

## 为什么还保留 IK 接口

虽然当前双臂控制改成直接关节控制，但 `IK` 接口没有删除，仍然保留在：

[ranger_arm_ik.py](/home/zjz/workspace/tiangong/tiangong/source/tiangong/tiangong/teleop/ranger_arm_ik.py)

保留内容包括：

- `RangerArmIKSolver`
- 左右臂 task 创建
- Jacobian body index / joint column 缓存
- 末端目标 `target_pos / target_quat`
- `damped_least_squares_delta()`
- 四元数与姿态误差辅助函数

保留这些接口的目的，是为了以后如果要恢复：

- 末端目标点自动逼近
- 基于目标坐标的末端闭环控制
- 视觉伺服 / 抓取前对位

就不用再重新搭建 IK 数据结构。

## 当前建议理解

可以把现在的 Ranger Arm 机械臂控制理解成：

```text
外部接口层：仍保留 IK task / 末端目标 / Jacobian 信息
执行层：默认走 7 自由度直接关节位置控制
```

因此当前最重要的结论是：

- 每个机械臂按 **7 个自由度关节** 控制
- 不是直接靠 Jacobian IK 每帧求解来驱动
- IK 模块仍然保留，供后续恢复末端控制能力

## 推荐启动

```bash
cd /home/zjz/workspace/tiangong/tiangong

bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --add-r1pro \
  --r1pro-physics \
  --ground-z -10.0 \
  --r1pro-z 0.0 \
  --r1pro-scale 1.55
```
