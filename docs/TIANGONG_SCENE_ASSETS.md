# 天宫场景资产依赖说明

本文档说明 `assets/tiangong_scene` 目录下场景资产的主要依赖关系，重点梳理：

- `tkmodel.usd`
- `configuration/*.usd`
- `materials/mdl/*`
- `materials/textures/*`
- `textures/*`

目的是让后续清理、替换、裁剪资产时，能快速判断哪些文件是入口，哪些文件是中间层，哪些文件只是底层贴图或建模源文件。

## 1. 总体调用链

当前天宫场景资产的主链路可以概括为：

```text
tkmodel.usd
  ├─ Tiangong Space Station.usd
  └─ ranger_arm_teleop/ranger_arm
       ├─ configuration/tkmodel_physics.usd
       │    └─ subLayer: configuration/tkmodel_base.usd
       ├─ variant Physics=None
       │    └─ reference: configuration/tkmodel_base.usd
       ├─ variant Physics=PhysX
       │    └─ payload: configuration/tkmodel_physics.usd
       └─ variant Sensor=Sensors
            └─ payload: configuration/tkmodel_sensor.usd

configuration/tkmodel_base.usd
  └─ materials/mdl/Base/OmniPBR.mdl
  └─ materials/mdl/Base/OmniPBR_Opacity.mdl
       └─ 继续读取 materials/textures/* 与 textures/* 中的网格/贴图资源
```

## 2. 入口文件说明

### 2.1 `tkmodel.usd`

文件：
[tkmodel.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/tkmodel.usd)

它是整个天宫地面场景的总入口，负责把两个大块内容拼起来：

- 空间站主体：`Tiangong Space Station.usd`
- 地面机器人 `ranger_arm_teleop/ranger_arm`

当前关键引用关系：

- `payload -> ./Tiangong Space Station.usd`
- `payload -> ./configuration/tkmodel_physics.usd`
- `reference -> ./configuration/tkmodel_base.usd`
- `payload -> ./configuration/tkmodel_sensor.usd`

也就是说，`tkmodel.usd` 只负责“组装场景”，不直接承载完整的 Ranger Arm 几何、碰撞和材质细节。

### 2.2 `Tiangong Space Station.usd`

文件：
[Tiangong Space Station.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/Tiangong%20Space%20Station.usd)

它主要是空间站静态场景资产入口，底层会继续使用 `textures/` 里的大量贴图。

如果只是调 `ranger_arm` 机器人本体，这个文件一般不用改。

## 3. Ranger Arm 分层结构

### 3.1 `configuration/tkmodel_base.usd`

文件：
[tkmodel_base.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/configuration/tkmodel_base.usd)

这是 `ranger_arm` 的“基础描述层”，主要包含：

- 可视几何 `visuals`
- 部分碰撞几何引用
- `Looks` 材质定义
- 轮子、转向轮、双臂、夹爪的视觉层级

这层本质上是“机器人长什么样”。

当前材质依赖已改为项目内相对路径：

- `../materials/mdl/Base/OmniPBR.mdl`
- `../materials/mdl/Base/OmniPBR_Opacity.mdl`

所以如果后面看到机器人贴图或材质异常，优先检查这个文件和 `materials/` 目录。

### 3.2 `configuration/tkmodel_physics.usd`

文件：
[tkmodel_physics.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/configuration/tkmodel_physics.usd)

它是 `ranger_arm` 的物理层，主要负责：

- 通过 `subLayer` 叠加 `tkmodel_base.usd`
- 添加 articulation/rigid body/collision 等物理相关内容
- 给各个 link 的 `collisions` 挂接碰撞层

可以把它理解成：

```text
tkmodel_base.usd = 长相与材质
tkmodel_physics.usd = 在 base 之上补齐物理属性
```

如果后面是“能看见机器人但不能被控制”或“轮子不动、碰撞失效”，优先查这一层。

### 3.3 `configuration/tkmodel_sensor.usd`

文件：
[tkmodel_sensor.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/configuration/tkmodel_sensor.usd)

这是传感器变体层，当前依赖较轻。只有在使用 `Sensor=Sensors` 这个 variant 时才会走到它。

如果当前主要做遥操作和底盘/机械臂控制，这层通常不是第一排查对象。

## 4. 材质与贴图层

### 4.1 `materials/mdl/Base`

目录：
[/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/materials/mdl/Base](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/materials/mdl/Base)

当前包含：

- [OmniPBR.mdl](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/materials/mdl/Base/OmniPBR.mdl)
- [OmniPBR_Opacity.mdl](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/materials/mdl/Base/OmniPBR_Opacity.mdl)

这两个文件现在已经本地化到项目内，作用是提供 `tkmodel_base.usd` 使用的 MDL 材质定义。

如果未来要进一步去 Isaac Sim 依赖，这一层就是关键缓冲层。

### 4.2 `materials/textures`

目录：
[/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/materials/textures](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/materials/textures)

这个目录名称里虽然有 `textures`，但实际不只是贴图，还混有一些网格和源文件：

- `fl.dae`
- `fr.dae`
- `ranger_base.dae`
- `wheel_assembly_a.dae`
- `wheel_assembly_b.dae`
- `base_add_on.STL`
- `base_texture.png`
- `wheel_texture.png`
- `ranger_base.blend`
- `change_color.py`
- `reduce_mesh.py`

这层更接近“机器人本体建模资源仓”。

建议这样理解：

- `.dae` / `.STL`：几何来源
- `.png`：机器人本体局部贴图
- `.blend`：Blender 源文件
- `*.py`：建模/贴图处理脚本，不参与仿真运行主链路

后面如果只想保留“运行所需最小集”，这个目录是最值得继续梳理和裁剪的一层。

### 4.3 `textures`

目录：
[/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/textures](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/textures)

这一层主要是天宫场景和部分材质贴图资源，数量最多，典型包括：

- 空间站地板贴图
- 控制面板贴图
- 金属面板法线/粗糙度/金属度贴图
- 标识类贴图，如 `flag.png`、`logo.png`

如果空间站场景能加载但“外观发白、发黑、没有贴图”，通常问题就在这里。

这层对 `Tiangong Space Station.usd` 的影响通常大于对 `ranger_arm` 本体的影响。

## 5. 后续清理时的判断方法

### 可以优先保留的核心文件

- [tkmodel.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/tkmodel.usd)
- [Tiangong Space Station.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/Tiangong%20Space%20Station.usd)
- [tkmodel_base.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/configuration/tkmodel_base.usd)
- [tkmodel_physics.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/configuration/tkmodel_physics.usd)
- [tkmodel_sensor.usd](/home/zjz/workspace/tiangong/tiangong/assets/tiangong_scene/configuration/tkmodel_sensor.usd)
- `materials/mdl/Base/*`

### 可以重点核查、谨慎裁剪的目录

- `materials/textures/`
- `textures/`

原因是这两层很可能存在：

- 已不再被引用的旧网格
- 仅用于建模加工的中间文件
- 可合并或降采样的贴图

### 不建议直接动的层

- `configuration/*.usd`
- `tkmodel.usd`

因为这些文件一旦误删字段，通常不是“贴图丢了”，而是会直接导致：

- payload 失效
- articulation 丢失
- 碰撞失效
- 机器人不可控

## 6. 当前整理后的状态

当前 `assets/tiangong_scene` 已完成这些清理：

- 删除了场景中的两个外部 `cf2x` payload
- 把 `ranger_arm` 材质中的绝对 `mdl` 路径改为项目内相对路径
- 资产主链路中不再残留 `/home/...`、`file:`、`workspace/isaacsim_assets` 这类外部绝对引用

所以当前可以把 `assets/tiangong_scene` 视为一个基本自洽的项目内资产包。


