# Tiangong IsaacSim Workspace

这个工作区用于在 IsaacSim/Isaac Lab 中加载天宫场景、Ranger Arm、Galaxea R1 Pro 和四旋翼无人机，并提供键盘遥操作、目标点靠近与轨迹飞行验证入口。

## 目录说明

- `assets/`：当前唯一保留的项目资产目录，包含天宫场景、Ranger Arm 物理场景、R1 Pro USD 和 Crazyflie 四旋翼 USD。
- `assets/trajectories/quadrotor/`：按序号存放四旋翼轨迹（`1.csv`、`2.csv` …），由 `--quadrotor-count` 决定读取数量。
- `scripts/keyboard_teleop_ranger_arm.py`：主遥操作入口，支持 Ranger Arm、R1 Pro、四旋翼轨迹飞行和目标点靠近。
- `scripts/run_with_isaaclab.sh`：使用本地 IsaacSim/Isaac Lab Python 执行项目脚本。
- `source/tiangong/tiangong/teleop/`：底盘控制、目标点靠近、Ranger Arm IK、R1 Pro 控制等核心代码。
- `source/tiangong/tiangong/utils/`：本地资产路径解析工具。
- `docs/`：操作手册、调用流程和 IK 文档。

已清理掉的内容包括外层重复资产、OmniGibson 原始资产、Galaxea 官方 demo 缓存、IsaacLab RL 模板环境和 Python 缓存。

## IsaacSim 准备

`scripts/run_with_isaaclab.sh` 会自动把 `source/tiangong` 加入 `PYTHONPATH`，并设置 `TIANGONG_PROJECT_ASSETS_ROOT` 指向项目内 `assets/`。请始终通过该脚本启动，避免 USD 场景路径解析错误。

### 方式 A：pip 安装（推荐）

在 conda/venv 中安装 `isaacsim`、`isaacsim-rl` 等包后，**没有** `python.sh`，需把 `ISAAC_SIM_PYTHON` 指向当前环境的 `python`：

```bash
conda activate isalab   # 换成你安装了 isaacsim 的环境名

export ISAAC_SIM_PYTHON=$(which python)
$ISAAC_SIM_PYTHON -c "import isaacsim; print('ok')"
```

验证 `isaacsim-rl` 已安装：

```bash
python -m pip show isaacsim-rl
```

### 方式 B：独立安装（Omniverse / 解压包）

独立安装的 Isaac Sim 自带 `python.sh`：

```bash
export ISAAC_SIM_PYTHON=/path/to/IsaacSim/python.sh
$ISAAC_SIM_PYTHON --version
```

也可将 `IsaacSim` 目录放在本仓库同级，脚本会自动查找 `../IsaacSim/python.sh`。

## 启动遥操作

```bash
cd /path/to/earth2moon-sim

bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --enable-arm-ik \
  --add-r1pro \
  --r1pro-physics \
  --enable-target-reach \
  --ground-z -10.0 \
  --r1pro-z 0.0 \
  --target-base-distance 1.4 \
  --r1pro-wheel-speed 4.5 \
  --ranger-wheel-speed 3.0 \
  --ik-speed 0.035 \
  --ik-rotation-speed 0.45 \
  --gripper-speed 0.04
```

pip 安装时完整示例：

```bash
conda activate isalab
export ISAAC_SIM_PYTHON=$(which python)

cd /path/to/earth2moon-sim
bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --enable-arm-ik \
  --add-r1pro \
  --r1pro-physics \
  --enable-target-reach
```

R1 Pro 默认只从 `assets/r1pro/r1pro.usda` 加载。Ranger Arm 和天宫场景默认只从 `assets/tiangong_scene/` 加载。

### 四旋翼无人机轨迹验证

四旋翼默认使用项目内 Crazyflie `cf2x.usd`（`assets/Crazyflie/` 或 `assets/Assets/Isaac/.../Crazyflie/`）。若本地没有该资产，可通过 `--quadrotor-usd` 或 `--isaac-asset-root` 指向 IsaacSim 资产根目录。

默认启用 **物理 6-DOF 飞控**（`--quadrotor-physics`，默认开）：保留 Crazyflie 关节体的刚体物理（重力、碰撞、四个旋翼转动副），通过级联 SE3 几何控制器（位置 PD → 期望推力向量；姿态 PD → 机体系力矩）驱动 `body` 刚体跟踪世界坐标系轨迹。暂停时在当前设定点悬停，`F4` 重置瞬移回起点并清零速度。需要旧的纯运动学模式（直接搬 prim、无飞行动力学）时加 `--no-quadrotor-physics`。物理模式建议 `--quadrotor-scale 1.0`（动力学用 USD 标注的 Crazyflie 质量/惯量，约 28 g，与视觉缩放无关）。

无人机数量由 **`--quadrotor-count`** 控制（默认 `4`）。每架机的轨迹从 **`--quadrotor-trajectory-dir`** 按序号读取：`1.csv`、`2.csv` … 直到 `N.csv`（也支持 `01.csv` 或 `.json`）。

默认轨迹目录 `assets/trajectories/quadrotor/` 已包含 4 条俯瞰轨迹，分别覆盖东北、西北、东南、西南象限，高度约 `Z = -0.5`（相对 `--ground-z -10.0` 约 9.5 m），彼此空间分离；相机默认 `--quadrotor-camera-pitch 90` 向下俯视。

单机物理飞行测试（读取预设轨迹 `1.csv`，推荐先这样验证飞控）：

```bash
bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --add-quadrotor \
  --quadrotor-count 1 \
  --quadrotor-trajectory-dir assets/trajectories/quadrotor \
  --quadrotor-scale 1.0 \
  --quadrotor-loop-trajectory \
  --quadrotor-camera-window-hz 10 \
  --quadrotor-depth-max 20
```

多机编队示例：

```bash
bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --add-quadrotor \
  --quadrotor-count 4 \
  --quadrotor-trajectory-dir assets/trajectories/quadrotor \
  --quadrotor-scale 1.0 \
  --quadrotor-loop-trajectory \
  --quadrotor-camera-window-hz 10 \
  --quadrotor-depth-max 20
```

调整数量示例：只飞 2 架时设 `--quadrotor-count 2`，脚本会读取目录下的 `1.csv` 和 `2.csv`；增加到 6 架时准备 `1.csv` … `6.csv` 即可。

轨迹目录结构：

```text
assets/trajectories/quadrotor/
├── 1.csv
├── 2.csv
├── 3.csv
└── 4.csv
```

轨迹 CSV 格式（世界坐标系，单位：米 / 度）：

```csv
time,x,y,z,yaw_deg
0.0,4.0,1.0,-0.5,0.0
```

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `--quadrotor-count` | `4` | 生成四旋翼数量；对应读取 `{trajectory-dir}/1..N.csv`。 |
| `--quadrotor-trajectory-dir` | `assets/trajectories/quadrotor` | 按序号存放轨迹文件的目录。 |
| `--ground-z` | `-10.0` | 场景地面高度；轨迹 `z` 与目标点高度均以此为基准。 |
| `--quadrotor-scale` | `1.0` | 四旋翼模型缩放；物理飞控建议 `1.0`。 |
| `--quadrotor-physics` | 开启 | 物理 6-DOF SE3 飞控；`--no-quadrotor-physics` 切回纯运动学。 |
| `--quadrotor-physics-kp-pos` | `6.0` | 位置比例增益（每米误差产生的期望加速度）。 |
| `--quadrotor-physics-kd-pos` | `3.0` | 位置微分增益（每 m/s 速度误差产生的期望加速度）。 |
| `--quadrotor-physics-k-att` | `8e-3` | 姿态误差增益 → 机体力矩。 |
| `--quadrotor-physics-k-omega` | `1.5e-3` | 角速度误差增益 → 机体力矩。 |
| `--quadrotor-physics-max-thrust` | `4×悬停` | 最大总推力（N）；默认为 4 倍悬停推力。 |
| `--quadrotor-physics-max-torque` | `0.01` | 单轴最大机体力矩（N·m）。 |
| `--quadrotor-physics-rotor-spin` | `1200.0` | 旋翼视觉目标转速（rad/s；m1/m3 与 m2/m4 反向）。 |
| `--quadrotor-camera-pitch` | `90.0` | 相机俯仰角（度）；`90` 为垂直向下俯瞰，`0` 为水平前视。 |
| `--quadrotor-camera-window-hz` | `10.0` | 外部 Matplotlib RGB-D 窗口刷新率。 |
| `--quadrotor-depth-max` | `20.0` | 深度图显示上限（米）；俯瞰高度较高时可适当增大。 |
| `--quadrotor-loop-trajectory` | 关闭 | 轨迹播完后循环；示例命令建议开启。 |

默认会打开外部 Matplotlib 窗口：单机时 1×2（RGB + Depth），多机时 N×2 网格同时显示全部相机。IsaacSim viewport 继续用于场景展示。不需要外部窗口时加 `--no-show-quadrotor-camera-window`；需要在 IsaacSim 内查看某一路相机时，按 `F2` 切到四旋翼后再按 `F6` 循环切换。

低高度前视单机飞行：使用 `--quadrotor-count 1`，修改 `assets/trajectories/quadrotor/1.csv` 中 `z` 为 `-7` 附近，并加 `--quadrotor-camera-pitch 0`。

## 键盘控制

完整说明见 [docs/ROBOT_OPERATION_MANUAL.md](docs/ROBOT_OPERATION_MANUAL.md)。

- `F1`：在可控机器人之间循环切换。
- `F2`：切到四旋翼无人机编队。
- `F3`：暂停或恢复全部四旋翼轨迹。
- `F4`：重置全部四旋翼轨迹。
- `F6`：切换到当前机器人相机；四旋翼激活且数量大于 1 时，在 IsaacSim viewport 中循环切换各机 RGB-D 相机。
- `1`：切到 Ranger Arm。
- `2`：切到 R1 Pro。
- `7`、`8`、`9`：切换目标点。
- `0`：暂停或恢复目标点靠近。
- `W/S` 或 `↑/↓`：前进/后退。
- `A/D` 或 `←/→`：Ranger Arm 转向；R1 Pro 横移/方向控制按当前控制器逻辑执行。
- `Q/E`：R1 Pro 底盘转向；四旋翼激活且轨迹暂停时升降。
- `Z/X`：四旋翼激活且轨迹暂停时偏航。
- `TAB`：切换左臂、右臂、双臂目标。
- `I/K`、`J/L`、`U/O`：移动当前机械臂目标。
- `T/G`、`F/H`、`R/Y`：旋转当前机械臂目标。
- `3/4`：R1 Pro 末端第 7 轴。
- `5/6`：R1 Pro 躯干 yaw。
- `M/N`：夹爪开/合。
- `ESC`：退出。

## 资产边界

当前运行不依赖这些历史目录：

- 外层 `omnigibson-robot-assets/`
- 外层 `configuration/`
- 外层 `materials/`
- 外层 `tkmodel.usd`
- `.local_galaxea/`
- `.local_isaac_assets/`

如果后续需要重新运行 Galaxea 官方 demo 或重新生成 OmniGibson 资产，需要重新下载对应官方资源；当前项目主流程不需要它们。
