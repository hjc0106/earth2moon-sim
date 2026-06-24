"""场景展示资产的固定、贴地和手动位姿控制工具。

主要用于把天宫空间站、无人机、R1 Pro 展示资产等非当前控制主体稳定放到地面
或指定 prim 附近，避免加载后受物理影响漂移。
"""

from __future__ import annotations

import math


class DroneAssetController:
    """管理辅助资产的显示模式、锁定位姿和地面贴合。"""

    GROUND_Z = 0.0

    def __init__(
        self,
        stage,
        sim_app,
        carb,
        gf_module,
        sdf_module,
        usd_module,
        usdgeom_module,
        usdphysics_module,
        ground_z=0.0,
    ):
        self._stage = stage
        self._sim_app = sim_app
        self._carb = carb
        self._Gf = gf_module
        self._Sdf = sdf_module
        self._Usd = usd_module
        self._UsdGeom = usdgeom_module
        self._UsdPhysics = usdphysics_module
        self.GROUND_Z = float(ground_z)
        self._grounded_asset_poses = []
        self._locked_asset_poses = {}

    def _make_transform_matrix(self, translation, rotation):
        """根据平移和 XYZ 欧拉角生成 USD transform 矩阵。"""
        transform = self._Gf.Matrix4d(1.0)
        rotation_x = self._Gf.Rotation(self._Gf.Vec3d(1.0, 0.0, 0.0), float(rotation[0]))
        rotation_y = self._Gf.Rotation(self._Gf.Vec3d(0.0, 1.0, 0.0), float(rotation[1]))
        rotation_z = self._Gf.Rotation(self._Gf.Vec3d(0.0, 0.0, 1.0), float(rotation[2]))
        transform.SetRotate(rotation_x * rotation_y * rotation_z)
        transform.SetTranslate(translation)
        return transform

    def apply_locked_poses(self) -> None:
        """每帧重放锁定位姿，保证展示资产不被仿真或 stage 更新带偏。"""
        for transform_op, scale_op, translation, rotation, scale in self._grounded_asset_poses:
            transform_op.Set(self._make_transform_matrix(translation, rotation))
            scale_op.Set(self._Gf.Vec3f(scale, scale, scale))
        for prim_path, pose in self._locked_asset_poses.items():
            pose["transform_op"].Set(self._make_transform_matrix(pose["translation"], pose["rotation"]))
            pose["scale_op"].Set(self._Gf.Vec3f(pose["scale"], pose["scale"], pose["scale"]))
            if pose.get("snap_to_ground", False):
                self._snap_locked_pose_to_ground(prim_path, pose)

    def _reset_to_ground_ops(self, prim):
        """清理并重建本模块专用的 transform/scale xformOp。"""
        xformable = self._UsdGeom.Xformable(prim)
        try:
            xformable.ClearXformOpOrder()
        except Exception:
            pass
        transform_op = None
        scale_op = None
        for op in xformable.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:transform:teleop_ground":
                transform_op = op
            if op.GetOpName() == "xformOp:scale:teleop_ground":
                scale_op = op
        if transform_op is None:
            transform_op = xformable.AddTransformOp(
                precision=self._UsdGeom.XformOp.PrecisionDouble,
                opSuffix="teleop_ground",
            )
        if scale_op is None:
            scale_op = xformable.AddScaleOp(
                precision=self._UsdGeom.XformOp.PrecisionFloat,
                opSuffix="teleop_ground",
            )
        xformable.SetXformOpOrder([transform_op, scale_op], True)
        return transform_op, scale_op

    def _disable_asset_physics(self, root_prim) -> None:
        """关闭展示资产下的 articulation、刚体和碰撞，让它只作为可视模型。"""
        for prim in self._Usd.PrimRange(root_prim):
            if not prim.IsValid():
                continue
            name = prim.GetName().lower()
            type_name = prim.GetTypeName()
            if type_name.startswith("Physics") or "joint" in name:
                prim.SetActive(False)
                continue
            try:
                if prim.HasAPI(self._UsdPhysics.ArticulationRootAPI):
                    prim.RemoveAPI(self._UsdPhysics.ArticulationRootAPI)
            except Exception:
                pass
            rigid_body = self._UsdPhysics.RigidBodyAPI(prim)
            if rigid_body:
                rigid_body.CreateRigidBodyEnabledAttr(False)
                rigid_body.CreateKinematicEnabledAttr(False)
            collision = self._UsdPhysics.CollisionAPI(prim)
            if collision:
                collision.CreateCollisionEnabledAttr(False)
                prim.SetActive(False)
                continue
            if name == "collisions":
                prim.SetActive(False)
                continue
            imageable = self._UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeVisible()
            prim.CreateAttribute("physxArticulation:articulationEnabled", self._Sdf.ValueTypeNames.Bool).Set(False)
            prim.CreateAttribute("physxArticulation:enabledSelfCollisions", self._Sdf.ValueTypeNames.Bool).Set(False)
            prim.CreateAttribute("physxRigidBody:disableGravity", self._Sdf.ValueTypeNames.Bool).Set(True)

    def _compute_ground_translation(self, asset_prim, target_xy):
        """根据整体包围盒计算使资产最低点贴到 GROUND_Z 的平移。"""
        bbox_cache = self._UsdGeom.BBoxCache(
            self._Usd.TimeCode.Default(),
            [self._UsdGeom.Tokens.default_, self._UsdGeom.Tokens.render, self._UsdGeom.Tokens.proxy],
        )
        aligned_range = bbox_cache.ComputeWorldBound(asset_prim).ComputeAlignedRange()
        center = aligned_range.GetMidpoint()
        min_z = aligned_range.GetMin()[2]
        return self._Gf.Vec3d(target_xy[0] - center[0], target_xy[1] - center[1], self.GROUND_Z - min_z)

    def _compute_contact_min_z(self, asset_prim):
        """优先用轮子接触 link 估计最低点，缺失时回退到整体包围盒。"""
        bbox_cache = self._UsdGeom.BBoxCache(
            self._Usd.TimeCode.Default(),
            [self._UsdGeom.Tokens.default_, self._UsdGeom.Tokens.render, self._UsdGeom.Tokens.proxy],
        )
        contact_names = {"wheel_motor_link1", "wheel_motor_link2", "wheel_motor_link3"}
        contact_min_z = []
        for prim in self._Usd.PrimRange(asset_prim):
            if not prim.IsValid() or prim.GetName() not in contact_names:
                continue
            aligned_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not aligned_range.IsEmpty():
                contact_min_z.append(aligned_range.GetMin()[2])
        if contact_min_z:
            return min(contact_min_z)
        aligned_range = bbox_cache.ComputeWorldBound(asset_prim).ComputeAlignedRange()
        return aligned_range.GetMin()[2]

    def _snap_locked_pose_to_ground(self, prim_path, pose) -> None:
        """重新计算锁定资产的 Z 偏移，使其持续贴在配置地面高度。"""
        asset_prim = self._stage.GetPrimAtPath(prim_path)
        if not asset_prim.IsValid():
            return
        try:
            min_z = self._compute_contact_min_z(asset_prim)
        except Exception:
            return
        z_offset = float(pose.get("z_offset", 0.0))
        dz = self.GROUND_Z - min_z + z_offset
        if abs(dz) < 1e-5:
            return
        translation = pose["translation"]
        pose["translation"] = self._Gf.Vec3d(translation[0], translation[1], translation[2] + dz)
        pose["transform_op"].Set(self._make_transform_matrix(pose["translation"], pose["rotation"]))

    def lock_asset_to_ground(self, prim_path, target_xy, rotation, scale=1.0) -> None:
        """把资产加载为展示模型，并固定到给定 XY 的地面位置。"""
        asset_prim = self._stage.GetPrimAtPath(prim_path)
        if not asset_prim.IsValid():
            self._carb.log_warn(f"Cannot ground missing asset prim: {prim_path}")
            return
        try:
            self._stage.Load(asset_prim.GetPath(), self._Usd.LoadWithDescendants)
        except TypeError:
            self._stage.Load(asset_prim.GetPath())
        except Exception as exc:
            self._carb.log_warn(f"Could not explicitly load {prim_path}: {exc}")
        for _ in range(3):
            self._sim_app.update()
        self._disable_asset_physics(asset_prim)
        transform_op, scale_op = self._reset_to_ground_ops(asset_prim)
        transform_op.Set(self._make_transform_matrix(self._Gf.Vec3d(target_xy[0], target_xy[1], 0.0), rotation))
        scale_op.Set(self._Gf.Vec3f(scale, scale, scale))
        self._sim_app.update()
        try:
            translation = self._compute_ground_translation(asset_prim, target_xy)
        except Exception as exc:
            self._carb.log_warn(f"Could not place {prim_path} on the ground from bbox: {exc}")
            translation = self._Gf.Vec3d(target_xy[0], target_xy[1], self.GROUND_Z)
        transform_op.Set(self._make_transform_matrix(translation, rotation))
        scale_op.Set(self._Gf.Vec3f(scale, scale, scale))
        self._grounded_asset_poses.append((transform_op, scale_op, translation, rotation, scale))

    def lock_asset_to_pose(self, prim_path, translation, rotation, scale=1.0) -> None:
        """把资产固定到指定世界位姿。"""
        asset_prim = self._stage.GetPrimAtPath(prim_path)
        if not asset_prim.IsValid():
            self._carb.log_warn(f"Cannot lock missing asset prim: {prim_path}")
            return
        try:
            self._stage.Load(asset_prim.GetPath(), self._Usd.LoadWithDescendants)
        except TypeError:
            self._stage.Load(asset_prim.GetPath())
        except Exception as exc:
            self._carb.log_warn(f"Could not explicitly load {prim_path}: {exc}")
        for _ in range(3):
            self._sim_app.update()
        transform_op, scale_op = self._reset_to_ground_ops(asset_prim)
        translation = self._Gf.Vec3d(translation[0], translation[1], translation[2])
        transform_op.Set(self._make_transform_matrix(translation, rotation))
        scale_op.Set(self._Gf.Vec3f(scale, scale, scale))
        self._grounded_asset_poses.append((transform_op, scale_op, translation, rotation, scale))

    def lock_asset_to_prim_pose(self, prim_path, source_prim_path, z_offset=0.0) -> bool:
        """复制 source prim 的世界位姿，并把资产底部修正到地面。"""
        asset_prim = self._stage.GetPrimAtPath(prim_path)
        source_prim = self._stage.GetPrimAtPath(source_prim_path)
        if not asset_prim.IsValid() or not source_prim.IsValid():
            self._carb.log_warn(f"Cannot lock {prim_path} to missing source prim {source_prim_path}")
            return False
        try:
            self._stage.Load(asset_prim.GetPath(), self._Usd.LoadWithDescendants)
            self._stage.Load(source_prim.GetPath(), self._Usd.LoadWithDescendants)
        except TypeError:
            self._stage.Load(asset_prim.GetPath())
            self._stage.Load(source_prim.GetPath())
        except Exception as exc:
            self._carb.log_warn(f"Could not explicitly load {prim_path} or {source_prim_path}: {exc}")
        for _ in range(3):
            self._sim_app.update()
        transform_op, scale_op = self._reset_to_ground_ops(asset_prim)
        xform_cache = self._UsdGeom.XformCache(self._Usd.TimeCode.Default())
        matrix = xform_cache.GetLocalToWorldTransform(source_prim)
        translation = matrix.ExtractTranslation()
        rotation = matrix.ExtractRotation().Decompose(
            self._Gf.Vec3d(1.0, 0.0, 0.0),
            self._Gf.Vec3d(0.0, 1.0, 0.0),
            self._Gf.Vec3d(0.0, 0.0, 1.0),
        )
        transform_op.Set(self._make_transform_matrix(translation, rotation))
        scale_op.Set(self._Gf.Vec3f(1.0, 1.0, 1.0))
        self._sim_app.update()
        try:
            min_z = self._compute_contact_min_z(asset_prim)
            translation = self._Gf.Vec3d(
                translation[0],
                translation[1],
                translation[2] + (self.GROUND_Z - min_z) + float(z_offset),
            )
        except Exception as exc:
            self._carb.log_warn(f"Could not ground {prim_path} after copying {source_prim_path} pose: {exc}")
            translation = self._Gf.Vec3d(translation[0], translation[1], translation[2] + float(z_offset))
        transform_op.Set(self._make_transform_matrix(translation, rotation))
        scale_op.Set(self._Gf.Vec3f(1.0, 1.0, 1.0))
        self._locked_asset_poses[prim_path] = {
            "transform_op": transform_op,
            "scale_op": scale_op,
            "translation": self._Gf.Vec3d(translation),
            "rotation": self._Gf.Vec3f(rotation),
            "scale": 1.0,
            "snap_to_ground": True,
            "z_offset": float(z_offset),
        }
        return True

    def move_locked_asset_pose(self, prim_path, forward, strafe, yaw, speed, turn_rate_rad, dt) -> bool:
        """按局部前后/横移/偏航增量移动已锁定资产。"""
        pose = self._locked_asset_poses.get(prim_path)
        if pose is None:
            return False
        translation = pose["translation"]
        rotation = pose["rotation"]
        yaw_rad = math.radians(float(rotation[2]))
        distance_forward = float(forward) * float(speed) * float(dt)
        distance_strafe = float(strafe) * float(speed) * float(dt)
        dx = distance_forward * math.cos(yaw_rad) - distance_strafe * math.sin(yaw_rad)
        dy = distance_forward * math.sin(yaw_rad) + distance_strafe * math.cos(yaw_rad)
        pose["translation"] = self._Gf.Vec3d(translation[0] + dx, translation[1] + dy, translation[2])
        pose["rotation"] = self._Gf.Vec3f(
            rotation[0],
            rotation[1],
            rotation[2] + math.degrees(float(yaw) * float(turn_rate_rad) * float(dt)),
        )
        if pose.get("snap_to_ground", False):
            pose["transform_op"].Set(self._make_transform_matrix(pose["translation"], pose["rotation"]))
            pose["scale_op"].Set(self._Gf.Vec3f(pose["scale"], pose["scale"], pose["scale"]))
            self._snap_locked_pose_to_ground(prim_path, pose)
        return True

    def place_asset_on_ground(self, prim_path, target_xy, rotation, keep_locked: bool, scale=1.0, z_offset=0.0) -> None:
        """把资产临时或持续放置到地面，可附加 z_offset 修正。"""
        asset_prim = self._stage.GetPrimAtPath(prim_path)
        if not asset_prim.IsValid():
            self._carb.log_warn(f"Cannot place missing asset prim: {prim_path}")
            return
        try:
            self._stage.Load(asset_prim.GetPath(), self._Usd.LoadWithDescendants)
        except TypeError:
            self._stage.Load(asset_prim.GetPath())
        except Exception as exc:
            self._carb.log_warn(f"Could not explicitly load {prim_path}: {exc}")
        for _ in range(3):
            self._sim_app.update()
        transform_op, scale_op = self._reset_to_ground_ops(asset_prim)
        transform_op.Set(self._make_transform_matrix(self._Gf.Vec3d(target_xy[0], target_xy[1], 0.0), rotation))
        scale_op.Set(self._Gf.Vec3f(scale, scale, scale))
        self._sim_app.update()
        try:
            min_z = self._compute_contact_min_z(asset_prim)
            translation = self._Gf.Vec3d(target_xy[0], target_xy[1], self.GROUND_Z - min_z)
            translation = self._Gf.Vec3d(translation[0], translation[1], translation[2] + float(z_offset))
        except Exception as exc:
            self._carb.log_warn(f"Could not place {prim_path} on the ground from bbox: {exc}")
            translation = self._Gf.Vec3d(target_xy[0], target_xy[1], self.GROUND_Z + float(z_offset))
        transform_op.Set(self._make_transform_matrix(translation, rotation))
        scale_op.Set(self._Gf.Vec3f(scale, scale, scale))
        if keep_locked:
            self._locked_asset_poses[prim_path] = {
                "transform_op": transform_op,
                "scale_op": scale_op,
                "translation": self._Gf.Vec3d(translation),
                "rotation": self._Gf.Vec3f(rotation),
                "scale": scale,
                "snap_to_ground": True,
                "z_offset": float(z_offset),
            }

    def make_display_only(self, prim_path) -> None:
        """仅关闭资产物理，不改变其当前显示位姿。"""
        asset_prim = self._stage.GetPrimAtPath(prim_path)
        if not asset_prim.IsValid():
            return
        try:
            self._stage.Load(asset_prim.GetPath(), self._Usd.LoadWithDescendants)
        except TypeError:
            self._stage.Load(asset_prim.GetPath())
        except Exception as exc:
            self._carb.log_warn(f"Could not explicitly load {prim_path}: {exc}")
        for _ in range(3):
            self._sim_app.update()
        self._disable_asset_physics(asset_prim)
        self._carb.log_warn(f"{prim_path} set to display-only; PhysX articulation/rigid bodies disabled.")
