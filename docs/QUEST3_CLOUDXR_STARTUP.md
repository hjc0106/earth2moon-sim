# Quest3 CloudXR 手动启动

本文档记录 `earth2moon-sim` 场景接入 Quest3 的手动启动顺序和对应命令。

注意：

- 建议在非 conda 终端里启动 `Isaac Sim` 相关脚本。
- 当前如果直接在 `conda` 终端里执行 `run_with_isaaclab.sh`，可能会在进入脚本前就报环境问题。

当前机器上的关键地址：

- Host IP: `172.18.4.85`
- CloudXR WSS 端口: `48322`
- CloudXR backend 端口: `49100`
- Quest3 访问地址: `https://172.18.4.85:48322/client/`

## 启动顺序

推荐顺序如下：

1. 启动 CloudXR 服务
2. 启动 `earth2moon-sim` XR 场景
3. 在 Isaac Sim 里确认 XR/VR 已启用
4. 在 Quest3 上连接 `172.18.4.85:48322`

## 方式一：推荐，一条命令起场景

这个方式最简单。脚本会先拉起 CloudXR，再启动 `earth2moon-sim` 的 XR 场景。

```bash
cd /home/zjz/workspace/tiangong/earth2moon-sim
./scripts/run_scene_r1pro_vr_quest3.sh
```

如果需要目标点参数，也可以继续追加，但当前 XR 启动流里 `--enable-target-reach` 会被忽略。

## 方式二：完全手动，分两个终端

### 终端 1：启动 CloudXR

```bash
cd /home/zjz/workspace/IsaacTeleop

source .venv/bin/activate

/home/zjz/workspace/tiangong/mujoco_teleop/scripts/run_isaacteleop_cloudxr_quest3.sh
```

正常情况下会看到类似输出：

```text
CloudXR runtime: running
CloudXR WSS proxy: running
Activate CloudXR environment in another terminal: source /home/zjz/.cloudxr/run/cloudxr.env
```

这个终端不要关。

### 终端 2：启动 earth2moon-sim XR 场景

```bash
source /home/zjz/.cloudxr/run/cloudxr.env

cd /home/zjz/workspace/tiangong/earth2moon-sim

bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --xr-openxr \
  --enable-openxr-r1pro-vr \
  --add-r1pro \
  --r1pro-physics \
  --r1pro-yaw 180.0 \
  --r1pro-init-pose-preset arms_forward_level \
  --ground-z -10.0 \
  --r1pro-z 0.0
```

如果你想加低速参数，建议用这个版本：

```bash
source /home/zjz/.cloudxr/run/cloudxr.env

cd /home/zjz/workspace/tiangong/earth2moon-sim

bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --xr-openxr \
  --enable-arm-ik \
  --add-r1pro \
  --r1pro-physics \
  --ground-z -10.0 \
  --r1pro-z 0.0 \
  --r1pro-wheel-speed 4.5 \
  --ranger-wheel-speed 3.0 \
  --ik-speed 0.035 \
  --ik-rotation-speed 0.45 \
  --gripper-speed 0.04
```

## Quest3 连接步骤

当上面两个终端都正常后：

1. 打开 Quest3 里的 CloudXR/Isaac Teleop client
2. 访问 `https://172.18.4.85:48322/client/`
3. 如果需要，也可以直接填服务地址 `172.18.4.85` 和端口 `48322`
4. 连接后，再看 Isaac Sim 里是否需要点 `Start VR`

## 场景里的相机切换

当前 `earth2moon-sim` 已有机器人相机切换逻辑：

- `F1`: 返回启动时的全场景视角
- `1`: 切到 `ranger_arm`
- `2`: 切到 `r1pro`
- `F6`: 切当前激活机器人的 `head_top`
- `F3`: 选择 `ranger_arm` 并切到头部 XR 视角
- `F4`: 选择 `r1pro` 并切到头部 XR 视角

建议测试两个机器人头部视角时按：

```text
F3 -> Ranger Arm
F4 -> R1 Pro
F1 -> 返回全场景
```

## 停止命令

如果要手动停服务：

```bash
pkill -f 'python -m isaacteleop.cloudxr'
pkill -f 'scripts/keyboard_teleop_ranger_arm.py --xr-openxr'
```

## 常见问题

### 1. Quest3 显示 disconnect

先检查：

- CloudXR 服务是否还在跑
- `earth2moon-sim` 场景是否真的用 `--xr-openxr` 启动
- 防火墙是否放行 `48322/tcp` 和 `49100/tcp`

### 2. Quest3 进了场景，但不是机器人头部视角

使用 `F3/F4` 进入对应机器人头部 XR 视角；只按 `1/2` 只会切换控制器，不会切换相机。切换后需要重新按一次左手柄 `Y` + 右手柄 `B` 标定。

### 3. `--enable-target-reach` 没效果

当前 XR 启动流里，这个参数会被忽略，这是现有脚本行为，不是启动命令写错。
