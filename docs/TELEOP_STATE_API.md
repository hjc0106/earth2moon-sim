# Teleop State API

启动 `scripts/keyboard_teleop_ranger_arm.py` 后，默认会启动 HTTP 服务：`http://127.0.0.1:8211/api/v1`。接口输出为 UTF-8 JSON，未使用鉴权；默认只绑定本机回环地址。

状态快照由 Isaac Sim 主线程在每个仿真帧发布，HTTP 线程只读取已发布的 JSON 数据，不会跨线程读取 USD 或 PhysX。除“切换视角”外，接口均为只读，不能控制机器人运动或写入关节值。

## 启动与接入

默认无需额外参数。可用以下参数调整：

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `--enable-state-api` / `--no-enable-state-api` | `bool` | `true` | 启用/关闭接口服务。 |
| `--state-api-host` | `string` | `127.0.0.1` | 服务监听地址。 |
| `--state-api-port` | `integer` | `8211` | 服务监听端口。 |
| `--state-api-image-width` | `integer` | `640` | 单张相机图像宽度（像素）。 |
| `--state-api-image-height` | `integer` | `480` | 单张相机图像高度（像素）。 |
| `--state-api-image-fps` | `number` | `10.0` | 图像缓存最大刷新频率，不是视频流帧率保证。 |
| `--pace-control-loop` / `--no-pace-control-loop` | `bool` | `true` | 以 `--dt` 节拍限制遥操作循环，防止固定步长的 VR/键盘增量在过高 FPS 下被重复叠加。 |

其他使用方应先请求 `GET /cameras` 获取当前场景真实存在的机器人名与相机别名；不要硬编码 USD Camera 路径。机器人可用名称通常是 `ranger_arm`、`r1pro`、`cf2x`、`cf2x_01`。

## 通用约定

- Base URL：`http://<host>:<port>/api/v1`
- `Content-Type`：响应均为 `application/json; charset=utf-8`；POST 请求必须发送 `application/json`。
- `updated_at`：`number`，Unix 时间戳，单位秒；用于判断状态新鲜度。
- `position`：`number[3]`，世界坐标 `[x, y, z]`，单位米。
- `quat_wxyz`：`number[4]`，四元数 `[w, x, y, z]`。
- `joints`：`object<string, number>`，键是 articulation DOF 名称，值为实时关节位置。
- 旋转关节位置单位为 `rad`，平移关节位置单位为 `m`；每个关节的具体单位可从 `joint_units` 读取。

## 接口定义

### GET `/`

发现接口。无请求体。

成功响应 `200 OK`：

```json
{
  "robots": ["ranger_arm", "r1pro", "cf2x", "cf2x_01"],
  "endpoints": ["/robots", "/cameras"]
}
```

字段类型：`robots` 为 `string[]`；`endpoints` 为相对路径 `string[]`。

### GET `/robots`

读取四个机器人当前状态。无请求体。

成功响应 `200 OK`：

```json
{
  "updated_at": 1785123456.12,
  "robots": {
    "r1pro": {
      "kind": "ground_robot",
      "prim_path": "/World/r1pro",
      "position": [1.8, 0.0, -10.0],
      "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
      "joints": {"torso_joint1": 0.0, "left_arm_joint1": 0.12},
      "joint_units": {
        "torso_joint1": {"position": "rad", "effort": "N*m"},
        "left_arm_joint1": {"position": "rad", "effort": "N*m"}
      }
    },
    "cf2x": {
      "kind": "drone",
      "prim_path": "/World/cf2x",
      "position": [0.0, -2.0, -8.0],
      "yaw_deg": 80.13,
      "joints": {},
      "joint_units": {}
    }
  }
}
```

`robots` 是 `object<string, RobotState>`。`RobotState.kind` 为 `"ground_robot"` 或 `"drone"`；`prim_path` 为 USD prim 的 `string` 路径。地面机器人提供 `quat_wxyz`，无人机提供 `yaw_deg: number`。无人机目前没有以 articulation 方式发布的关节，因此其 `joints` 和 `joint_units` 为空对象。

### GET `/robots/{robot_name}/joints`

读取单台机器人的实时关节位置。

| 路径参数 | 类型 | 必填 | 示例 |
| --- | --- | --- | --- |
| `robot_name` | `string` | 是 | `ranger_arm`、`r1pro` |

成功响应 `200 OK`：

```json
{
  "name": "ranger_arm",
  "joints": {
    "arm_left_joint1": 0.15,
    "arm_right_joint1": -0.05
  },
  "updated_at": 1785123456.12
}
```

`name` 为 `string`，`joints` 为 `object<string, number>`，`joint_units` 为 `object<string, {position: string, effort: string}>`，用于标识每个关节的位置和力/力矩单位。请求不存在的机器人时返回 `404 Not Found`：

```json
{"error": "Unknown robot: unknown_robot"}
```

### GET `/cameras`

查询每个机器人当前可用的相机别名及 USD Camera 路径。无请求体。

成功响应 `200 OK`：

```json
{
  "updated_at": 1785123456.12,
  "cameras": {
    "ranger_arm": {"head_top": "/World/teleop_camera_rigs/ranger_arm/head_top/Camera"},
    "r1pro": {"head_top": "/World/.../Camera", "left_gripper": "/World/.../Camera"},
    "cf2x": {"chase": "/World/teleop_camera_rigs/cf2x/chase/Camera"},
    "cf2x_01": {"chase": "/World/teleop_camera_rigs/cf2x_01/chase/Camera"}
  }
}
```

`cameras` 为 `object<string, object<string, string>>`：第一层 key 是机器人名，第二层 key 是相机别名，value 是 USD Camera prim 路径。相机是否存在取决于实际加载的资产；调用方必须以本接口返回值为准。

### GET `/cameras/{robot_name}/{camera_alias}/image`

读取指定相机最新的一帧 RGB 图像。该接口直接返回 JPEG 二进制，不是 JSON；响应头 `Content-Type` 为 `image/jpeg`，`X-Frame-Timestamp` 为该帧的 Unix 时间戳（秒）。

| 路径参数 | 类型 | 必填 | 示例 |
| --- | --- | --- | --- |
| `robot_name` | `string` | 是 | `ranger_arm`、`r1pro`、`cf2x`、`cf2x_01` |
| `camera_alias` | `string` | 是 | `head_top`、`left_gripper`、`right_gripper`、`chase` |

例如，获取 Ranger Arm 头部第一视角图像：

```text
GET /api/v1/cameras/ranger_arm/head_top/image
```

获取 R1 Pro 头部第一视角图像：

```text
GET /api/v1/cameras/r1pro/head_top/image
```

成功响应为 `200 OK` 和 JPEG 数据。渲染管线尚未产出首帧、相机不存在或 Replicator 图像采集不可用时返回 `503 Service Unavailable`：

```json
{"error": "Camera image is not ready"}
```

调用方应先通过 `/cameras` 确认相机别名存在；随后可按需要轮询本接口。它提供的是“最新帧”，不保证每次请求获得不同帧，也不适用于低延迟视频传输；连续视频需求应另行使用 WebRTC、RTSP 或 MJPEG。

### GET `/relative-poses/{reference_robot}/{target_robot}`

读取目标机器人在参考机器人车体坐标系中的相对位姿。当前发布 `r1pro/ranger_arm`（Ranger Arm 相对于 R1 Pro）与 `ranger_arm/r1pro`（R1 Pro 相对于 Ranger Arm）两个方向。

```text
GET /api/v1/relative-poses/r1pro/ranger_arm
```

成功响应 `200 OK`：

```json
{
  "reference": "r1pro",
  "target": "ranger_arm",
  "position_m": [1.25, -0.40, 0.0],
  "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
  "updated_at": 1785123456.12
}
```

`position_m: number[3]` 的坐标轴与 R1 Pro 车体坐标系一致；`quat_wxyz: number[4]` 表示 Ranger Arm 相对于 R1 Pro 的旋转。相对位姿尚未可用时返回 `404 Not Found`。

### GET `/telemetry`

读取遥操作性能与 VR 同步诊断指标。所有时间均为毫秒，数值为主循环最近最多 120 帧的滑动统计；它反映实际执行节拍，而非 Isaac Sim 配置的理论 `--dt`。

成功响应 `200 OK`：

```json
{
  "updated_at": 1785123456.12,
  "telemetry": {
    "sample_count": 120,
    "target_control_hz": 60.0,
    "loop_fps": 58.7,
    "loop_frame_ms_mean": 17.04,
    "loop_frame_ms_p95": 22.30,
    "simulation_step_ms_mean": 14.82,
    "simulation_step_ms_max": 21.76,
    "camera_image_target_fps": 10.0,
    "openxr_enabled": true,
    "openxr_calibrated": true,
    "udp_vr_control_age_ms": null,
    "active_control_kind": "robot",
    "active_robot": "r1pro"
  }
}
```

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `sample_count` | `integer` | 已统计帧数，启动初期小于 120。 |
| `target_control_hz` | `number` | `1 / --dt`，控制算法的目标频率，不代表实际帧率。 |
| `control_loop_paced` | `boolean` | 是否启用了 `--dt` 节拍限制；默认应为 `true`。 |
| `loop_fps` | `number` | 实际主循环平均频率。 |
| `loop_frame_ms_mean` / `loop_frame_ms_p95` | `number` | 主循环平均帧耗时及 P95 帧耗时；P95 明显偏大表示存在抖动。 |
| `simulation_step_ms_mean` / `simulation_step_ms_max` | `number` | physics + render 的平均/最大单步耗时。 |
| `camera_image_target_fps` | `number` | HTTP 图像缓存配置的最高刷新率；0 表示相机图像采集未启用。 |
| `openxr_enabled` / `openxr_calibrated` | `boolean` | OpenXR VR 是否启用、是否已完成标定。 |
| `udp_vr_control_age_ms` | `number \| null` | 最近一次外部 UDP VR 控制包的年龄；`null` 表示未收到过控制包。持续大于约 300 ms 时该控制输入已按过期处理。 |
| `active_control_kind` / `active_robot` | `string \| null` | 当前控制上下文及当前机器人。 |

VR 末端控制既会受低帧率/抖动影响，也会受“实际循环频率高于固定 `--dt` 目标频率”影响：后者会令每个按固定 `--dt` 计算的增量被更频繁地叠加。默认启用 `--pace-control-loop`，使循环不快于 `1 / --dt`。建议先观察：若目标为 60 Hz，平均帧耗时应接近 16.7 ms，P95 也应尽量稳定；若需 Quest 3 的 72/90 Hz 体验，则应相应降低渲染负载并将 `--dt` 设为 `1/72` 或 `1/90`，而不是关闭节拍限制。

### POST `/active-camera`

请求在 Isaac Sim 的当前 viewport 中切换视角。该接口只切换本地仿真 UI 的 active camera，不返回图像流，也不影响机器人控制。

请求体：

```json
{"robot": "cf2x", "camera": "chase"}
```

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `robot` | `string` | 是 | 必须是 `/cameras` 中存在的机器人名。 |
| `camera` | `string` | 是 | 必须是该机器人下存在的相机别名。 |

成功响应 `202 Accepted`：

```json
{"robot": "cf2x", "camera": "chase"}
```

`202` 表示命令已进入队列，实际切换由下一次仿真主循环处理。请求格式错误返回 `400 Bad Request`，机器人名或相机别名不存在返回 `404 Not Found`。所有未知路径均返回 `404`，错误格式均为 `{"error": "..."}`。

## 调用示例

### curl

```bash
# 读取 R1 Pro 所有关节位置
curl --fail http://127.0.0.1:8211/api/v1/robots/r1pro/joints

# 查询四个机器人目前可用的相机
curl --fail http://127.0.0.1:8211/api/v1/cameras

# 保存 Ranger Arm 头部第一视角 JPEG；查看实时画面可周期性重复请求此 URL
curl --fail http://127.0.0.1:8211/api/v1/cameras/ranger_arm/head_top/image \
  --output ranger_head.jpg

# 获取 Ranger Arm 在 R1 Pro 车体系下的相对位姿
curl --fail http://127.0.0.1:8211/api/v1/relative-poses/r1pro/ranger_arm

# 查看实际循环 FPS、P95 帧耗时和 VR 输入时效
curl --fail http://127.0.0.1:8211/api/v1/telemetry

# 切换到第一架无人机的 chase 视角
curl --fail-with-body -X POST http://127.0.0.1:8211/api/v1/active-camera \
  -H 'Content-Type: application/json' \
  -d '{"robot":"cf2x","camera":"chase"}'
```

### Python

```python
import requests

base_url = "http://127.0.0.1:8211/api/v1"

# 读取关节；生产环境请设置连接/读取超时并检查 HTTP 状态。
response = requests.get(f"{base_url}/robots/r1pro/joints", timeout=(1, 2))
response.raise_for_status()
joints: dict[str, float] = response.json()["joints"]
print(joints["left_arm_joint1"])

# 先发现可用视角，再发起切换。
cameras: dict[str, dict[str, str]] = requests.get(f"{base_url}/cameras", timeout=(1, 2)).json()["cameras"]
if "chase" in cameras.get("cf2x", {}):
    response = requests.post(
        f"{base_url}/active-camera",
        json={"robot": "cf2x", "camera": "chase"},
        timeout=(1, 2),
    )
    response.raise_for_status()  # 成功时为 202

# 获取头部第一视角 JPEG，并交给 OpenCV/Pillow/视觉模型处理。
image_response = requests.get(
    f"{base_url}/cameras/ranger_arm/head_top/image",
    timeout=(1, 2),
)
image_response.raise_for_status()
with open("ranger_head.jpg", "wb") as image_file:
    image_file.write(image_response.content)

# Ranger Arm 相对 R1 Pro 的位置和姿态。
relative_pose = requests.get(
    f"{base_url}/relative-poses/r1pro/ranger_arm", timeout=(1, 2)
).json()
print(relative_pose["position_m"])

# 诊断 VR 同步：不要只看 target_control_hz，要看实际 loop_fps 与 P95 帧耗时。
telemetry = requests.get(f"{base_url}/telemetry", timeout=(1, 2)).json()["telemetry"]
print(telemetry["loop_fps"], telemetry["loop_frame_ms_p95"])
```

## 部署与安全

如需让其他机器访问，可显式传入 `--state-api-host 0.0.0.0`，例如：

```bash
bash scripts/run_with_isaaclab.sh scripts/keyboard_teleop_ranger_arm.py \
  --state-api-host 0.0.0.0 --state-api-port 8211
```

该服务没有认证、TLS、访问频率限制或命令权限控制。仅应部署在受信任的内网，或通过反向代理、防火墙、VPN 和认证层保护；不要直接暴露到公网。
