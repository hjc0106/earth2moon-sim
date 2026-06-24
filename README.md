# Tiangong IsaacSim Workspace

这个工作区用于在 IsaacSim/Isaac Lab 中加载天宫场景、Ranger Arm 和 Galaxea R1 Pro，并提供键盘遥操作和目标点靠近入口。

## 目录说明

- `assets/`：当前唯一保留的项目资产目录，包含天宫场景、Ranger Arm 物理场景和 R1 Pro USD。
- `scripts/keyboard_teleop_ranger_arm.py`：主遥操作入口，支持 Ranger Arm、R1 Pro 和目标点靠近。
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

## 键盘控制

完整说明见 [docs/ROBOT_OPERATION_MANUAL.md](docs/ROBOT_OPERATION_MANUAL.md)。

- `F1`：在可控机器人之间循环切换。
- `1`：切到 Ranger Arm。
- `2`：切到 R1 Pro。
- `7`、`8`、`9`：切换目标点。
- `0`：暂停或恢复目标点靠近。
- `W/S` 或 `↑/↓`：前进/后退。
- `A/D` 或 `←/→`：Ranger Arm 转向；R1 Pro 横移/方向控制按当前控制器逻辑执行。
- `Q/E`：R1 Pro 底盘转向；Ranger Arm 普通模式下用于升降。
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
