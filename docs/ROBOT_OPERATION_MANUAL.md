# Robot Operation Manual

本文档说明如何在本项目中启动天宫场景、Ranger Arm 和 Galaxea R1 Pro，并解释两台机器人的键盘控制方式。

## 资产位置

本项目的默认运行路径已经改为项目内资产，除 IsaacSim 安装本体外，不需要再依赖 `/home/zjz/workspace/isaacsim_assets` 或外层 `tkmodel.usd`。

```text
assets/
├── tiangong_scene/
│   ├── tkmodel.usd
│   ├── Tiangong Space Station.usd
│   ├── configuration/
│   │   └── tkmodel_physics.usd
│   └── materials/
└── r1pro/
    ├── r1pro.usda
    └── usd/
        ├── left_arm_link6.usd
        └── textures/
```

默认解析：

- 主场景：`assets/tiangong_scene/tkmodel.usd`
- 天宫动画采样：`assets/tiangong_scene/Tiangong Space Station.usd`
- Ranger Arm 物理资产：`assets/tiangong_scene/configuration/tkmodel_physics.usd`
- R1 Pro 官方资产：`assets/r1pro/r1pro.usda`

## 启动命令

从项目根目录执行：

```bash
cd /home/zjz/workspace/tiangong/tiangong

bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --add-r1pro \
  --r1pro-physics \
  --ground-z -10.0 \
  --r1pro-z 0.0 \
  --r1pro-scale 1.55
```

推荐先用低速启动，避免 R1 Pro 轮速太快摔倒：

```bash
cd /home/zjz/workspace/tiangong/tiangong

bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --add-r1pro \
  --r1pro-physics \
  --ground-z -10.0 \
  --r1pro-z 0.0 \
  --r1pro-scale 1.55 \
  --r1pro-wheel-speed 4.5 \
  --ranger-wheel-speed 3.0 \
  --ranger-wheel-turn-speed 0.4 \
  --ik-speed 0.035 \
  --ik-rotation-speed 0.45 \
  --gripper-speed 0.04
```


## 速度参数

| 参数 | 默认值 | 推荐低速 | 影响对象 | 说明 |
| --- | ---: | ---: | --- | --- |
| `--wheel-speed` | `3.0` | `1.5` | Ranger Arm / R1 Pro | 两台机器人共用的默认轮子角速度，单位 `rad/s`。 |
| `--ranger-wheel-speed` | `4.0` | `3.0` | Ranger Arm | Ranger Arm 前进/后退轮速，会覆盖 `--wheel-speed`；目标点模式下可适当高于 R1 Pro。 |
| `--ranger-wheel-turn-speed` | 跟随 `--wheel-turn-speed` | `0.4` | Ranger Arm | 兼容保留参数；当前 Ranger Arm 主要通过机械臂端两轮同向转向实现转弯。 |
| `--wheel-turn-speed` | `0.8` | `0.4` | Ranger Arm | 兼容保留参数；无转向 DOF 时才会作为差速兜底参考。 |
| `--max-steer-rad` | `0.6` | `0.4` 到 `0.6` | Ranger Arm | Ranger Arm 转向关节最大角度，单位 `rad`。 |
| `--left-wheel-sign` / `--right-wheel-sign` | `1.0` / `1.0` | 按模型调整 | Ranger Arm | 左右轮速度方向修正。如果按 `W` 后反向走，把对应符号取反。 |
| `--r1pro-wheel-speed` | `4.5` | `4.0` 到 `4.5` | R1 Pro | R1 Pro `wheel_motor_joint1..3` 轮子角速度；目标靠近模式下需要高于 4 才不容易在停止圈外推不动。 |
| `--r1pro-max-steer-rad` | `3.14159` | `3.14159` | R1 Pro | R1 Pro `steer_motor_joint1..3` 最大转向角。保持接近 `pi` 可支持横移。 |
| `--speed` | `0.05` | `0.03` | 保留参数 | 当前底座移动只走轮子/转向 DOF，不再用平移 prim 兜底。 |
| `--turn-rate` | `10.0` | `5.0` | 保留参数 | 当前底座移动只走轮子/转向 DOF，不再用平移 prim 兜底。 |
| `--ik-speed` | `0.06` | `0.035` | 两台机器人机械臂 | Ranger Arm 直接关节控制的前 3 轴步进速度参考；R1 Pro 用作末端目标平移速度。 |
| `--ik-rotation-speed` | `0.8` | `0.45` | 两台机器人机械臂 / R1 Pro 躯干 | Ranger Arm 直接关节控制的后 4 轴步进速度参考；R1 Pro 用作姿态和部分关节步进速度。 |
| `--gripper-speed` | `0.08` | `0.04` | 两台机器人夹爪 | 夹爪开合目标变化速度，单位 `m/s`。 |
| `--ground-z` | `-10.0` | `-10.0` | 贴地资产 / R1 Pro | 天宫场景地面平面在 `Z=-10` 附近，R1 Pro 以它为贴地基准。 |
| `--r1pro-scale` | `1.55` | `1.4` 到 `1.7` | R1 Pro | R1 Pro 整体缩放比例，用于让体量更接近 Ranger Arm，并保持显示与物理一起缩放。 |
| `--target-reach-speed` | `0.05` | `0.03` 到 `0.05` | 两台机器人末端 | 当前阶段机械臂自动触及已暂停；该参数保留给后续恢复末端逼近。 |
| `--target-reach-tolerance` | `0.03` | `0.03` | 两台机器人末端 | 当前阶段机械臂自动触及已暂停；该参数保留给后续恢复末端到达判定。 |
| `--target-height` | `1.15` | `1.0` 到 `1.3` | 目标点 | 默认目标点相对 `--ground-z` 的高度，用于接近夹爪水平高度。 |
| `--target-base-distance` | `1.4` | `1.2` 到 `1.4` | 两台机器人底座辅助 | 目标点模式下底座靠近目标区域的停止距离。 |

R1 Pro 当前使用官方 Galaxea USD 的底盘关节：

- 转向关节：`steer_motor_joint1`、`steer_motor_joint2`、`steer_motor_joint3`
- 轮子关节：`wheel_motor_joint1`、`wheel_motor_joint2`、`wheel_motor_joint3`

所以 R1 Pro 的底盘速度主要看 `--r1pro-wheel-speed`，不是 `--speed`。

### R1 Pro 底座转向实现

R1 Pro 底座不是平移 prim。键盘输入会先转换成每个轮组的二维速度向量：

- `W/S` 改变前后速度分量。
- Ranger Arm 激活时，`A/D` 控制可转向轮左/右打角；R1 Pro 激活时，`A/D` 改变横向速度分量。
- `Q/E` 改变绕 Z 轴旋转分量。

脚本根据三组轮子在底盘上的位置，给每个轮组计算目标方向：

- 方向角写入 `steer_motor_joint1..3`，让轮子先转向。
- 速度大小写入 `wheel_motor_joint1..3`，让轮子滚动。
- 如果轮组目标方向超过前进半平面，会自动反向轮速并缩短转向角，避免轮子大幅绕圈。

因此调慢 R1 Pro 时优先调 `--r1pro-wheel-speed`；想限制转向角再调 `--r1pro-max-steer-rad`。

## 控制焦点

启动 IsaacSim 后，先点击 viewport，让窗口获得键盘焦点。没有焦点时按键事件可能不会进入脚本。

## 机器人切换

| 按键 | 功能 |
| --- | --- |
| `F1` | 在可控机器人之间循环切换 |
| `1` | 切换到 Ranger Arm |
| `2` | 切换到 R1 Pro |
| `TAB` | 切换当前机械臂目标：左臂、右臂、双臂 |
| `7` / `8` / `9` | 选择第 1 / 2 / 3 个目标点小方块 |
| `0` | 开关目标点自动逼近 |
| `ESC` | 退出脚本 |

启动后脚本默认会优先切换到 R1 Pro。如果按键没有反应，先按 `2` 再操作底盘。

## 目标点逼近

启用 `--enable-target-reach` 后，脚本会在 `/World/teleop_targets` 下创建三个目标点 marker。每个 marker 包含一个带碰撞的 20cm 方块和一根同色竖直提示柱。当前阶段先暂停机械臂自动触及，只让当前激活机器人底座靠近选中的目标区域。

默认目标点会放在 R1 Pro 初始位置前方，并分散在左右两侧，高度接近夹爪水平。也可以手动指定：

```bash
--target-points "2.75,-2.45,-8.85;3.10,-1.60,-8.75;2.75,-0.75,-8.85"
```

操作方式：

- 可用鼠标在 viewport/stage 中选中目标方块并拖动，自动逼近会读取方块当前世界坐标。
- `7`、`8`、`9`：切换三个目标点，主键盘数字和小键盘数字都支持。
- `0`：暂停或恢复自动逼近，主键盘数字和小键盘数字都支持。
- `TAB`：切换左臂、右臂、双臂。
- 当前阶段暂停机械臂自动触及目标，只处理底座移动到目标区域；手臂仍可手动遥操作。
- 目标点较远且没有手动底盘输入时，脚本会给当前激活机器人的底座一个小的靠近命令，让机器人进入目标区域；默认在距离目标约 `1.4m` 且机械臂所在一端基本面向目标时停止。
- Ranger Arm 和 R1 Pro 的底座移动都只驱动轮子/转向 DOF；如果日志提示找不到 wheel DOF，需要先修正 USD 关节名或启动参数，不会用平移 prim 代替。
- 日志会低频显示当前机器人与目标方块的实时 XY 距离和是否进入 `--target-base-distance`。机器人定位来自 PhysX articulation/root link 的 live pose，不再读取静态 USD 根节点 xform。

完整调用流程见 [CALL_FLOW.md](CALL_FLOW.md)。

## Ranger Arm 控制

Ranger Arm 是天宫场景中的移动机械臂平台。

### 底盘/平台

| 按键 | 功能 |
| --- | --- |
| `W` / `↑` | 前进 |
| `S` / `↓` | 后退 |
| `A` / `←` | 底盘左转 |
| `D` / `→` | 底盘右转 |

### 机械臂控制

| 按键 | 功能 |
| --- | --- |
| `I` / `K` | 第 1 关节正/负方向 |
| `J` / `L` | 第 2 关节正/负方向 |
| `U` / `O` | 第 3 关节正/负方向 |
| `T` / `G` | 第 4 关节正/负方向 |
| `F` / `H` | 第 5 关节正/负方向 |
| `R` / `Y` | 第 6 关节正/负方向 |
| `3` / `4` | 第 7 关节正/负方向 |
| `M` | 打开夹爪 |
| `N` | 关闭夹爪 |

Ranger Arm 当前每个机械臂都按 7 个自由度关节控制：

```text
arm_left_joint1..7
arm_right_joint1..7
```

项目中仍然保留了 `IK` 求解器接口与 Jacobian 数据结构，但当前遥操作执行层已经切换成与 `R1 Pro` 一致的直接关节位置控制。

## R1 Pro 控制

R1 Pro 使用官方 Galaxea USD，底盘通过自带关节控制：

- 转向关节：`steer_motor_joint1..3`
- 轮子关节：`wheel_motor_joint1..3`

这意味着底盘移动来自轮子转向和滚动，不是平移整个 prim。

### 底盘

| 按键 | 功能 |
| --- | --- |
| `W` / `↑` | 前进 |
| `S` / `↓` | 后退 |
| `A` / `←` | 底盘左转/左打轮 |
| `D` / `→` | 底盘右转/右打轮 |
| `Q` | 左转 |
| `E` | 右转 |

### 机械臂与夹爪

| 按键 | 功能 |
| --- | --- |
| `TAB` | 切换左臂、右臂、双臂 |
| `I/K` | 当前臂目标前/后 |
| `J/L` | 当前臂目标左/右 |
| `U/O` | 当前臂目标上/下 |
| `T/G` | 手腕 Roll 正/负方向，对应 `arm_joint7` |
| `F/H` | 手腕 Pitch 正/负方向，对应 `arm_joint6` |
| `R/Y` | 手腕 Yaw 正/负方向，对应 `arm_joint5` |
| `7/8` | 第 7 轴正/负方向 |
| `M/N` | 夹爪开/合 |

R1 Pro 夹爪的两个 finger joint 限位方向相反：

- `left/right_gripper_finger_joint1`：闭合 `0`，打开约 `+0.05`
- `left/right_gripper_finger_joint2`：闭合 `0`，打开约 `-0.05`

脚本会分别给两个 finger 写入相反方向的目标位置，所以 `M` 是打开，`N` 是闭合。若开合不明显，可以提高 `--gripper-speed`，例如 `--gripper-speed 0.08`。

### 躯干

| 按键 | 功能 |
| --- | --- |
| `Z/X` | `torso_joint1` 正/负方向 |
| `C/V` | `torso_joint2` 正/负方向 |
| `B/P` | `torso_joint3` 正/负方向 |
| `5/6` | `torso_joint4` 腰部 Z 轴旋转正/负方向 |

R1 Pro 官方 `torso.py` 使用 4 个躯干关节：

- `torso_joint1`：Y 轴俯仰
- `torso_joint2`：Y 轴俯仰
- `torso_joint3`：Y 轴俯仰，官方 URDF 轴向为 `0 -1 0`
- `torso_joint4`：Z 轴旋转，也就是腰部旋转关节

`left_arm_base_joint` 和 `right_arm_base_joint` 是固定关节，不是可控 DOF。两侧机械臂真正可控关节是 `left/right_arm_joint1..7`。

R1 Pro 和 Ranger Arm 现在都统一采用“缓存关节目标并持续发送”的方式控制手臂；区别在于：

- `R1 Pro` 本来就是 7 关节直接控制
- `Ranger Arm` 保留 IK 接口，但当前执行层也已切换成 7 关节直接控制

## 外部资产路径

默认命令已经不需要外部资产根。如果你手动传了 `--isaac-asset-root` 或设置了 `TIANGONG_ISAAC_ASSET_ROOT`，脚本会使用该外部镜像。想强制项目内资产，取消该环境变量：

```bash
unset TIANGONG_ISAAC_ASSET_ROOT
```

## 可选：运行 Galaxea 官方 Demo

本项目仍保留官方 demo 运行入口：

```bash
bash scripts/run_galaxea_demo.sh wheel
```

官方 `wheel` demo 会直接运行 `source/wheel_move.py`，用于对照底盘轮速和转向关节行为。
