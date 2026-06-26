"""Quadrotor asset loading, RGB-D camera setup, and trajectory tracking."""

from __future__ import annotations

import csv
import json
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class QuadrotorWaypoint:
    """One world-frame quadrotor trajectory waypoint."""

    time: float
    position: np.ndarray
    yaw_deg: float


def load_quadrotor_trajectory(path: str) -> list[QuadrotorWaypoint]:
    """Load a CSV or JSON world-frame quadrotor trajectory."""
    trajectory_path = Path(path).expanduser()
    if not trajectory_path.exists():
        raise FileNotFoundError(f"Quadrotor trajectory not found: {trajectory_path}")
    suffix = trajectory_path.suffix.lower()
    if suffix == ".csv":
        return _load_quadrotor_trajectory_csv(trajectory_path)
    if suffix == ".json":
        return _load_quadrotor_trajectory_json(trajectory_path)
    raise ValueError(f"Unsupported quadrotor trajectory format: {trajectory_path.suffix}")


def _load_quadrotor_trajectory_csv(path: Path) -> list[QuadrotorWaypoint]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"time", "x", "y", "z", "yaw_deg"}
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"Quadrotor CSV trajectory missing columns: {sorted(missing)}")
        waypoints = [
            QuadrotorWaypoint(
                time=float(row["time"]),
                position=np.array([float(row["x"]), float(row["y"]), float(row["z"])], dtype=np.float32),
                yaw_deg=float(row["yaw_deg"]),
            )
            for row in reader
        ]
    return _validate_quadrotor_trajectory(waypoints)


def _load_quadrotor_trajectory_json(path: Path) -> list[QuadrotorWaypoint]:
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if isinstance(payload, dict):
        raw_waypoints = payload.get("waypoints")
    elif isinstance(payload, list):
        raw_waypoints = payload
    else:
        raw_waypoints = None
    if not isinstance(raw_waypoints, list):
        raise ValueError("Quadrotor JSON trajectory must contain a 'waypoints' list.")
    waypoints = []
    for waypoint in raw_waypoints:
        if "position" not in waypoint:
            raise ValueError("Quadrotor JSON waypoint missing 'position'.")
        position = waypoint["position"]
        if len(position) != 3:
            raise ValueError("Quadrotor JSON waypoint position must be [x, y, z].")
        waypoints.append(
            QuadrotorWaypoint(
                time=float(waypoint["time"]),
                position=np.array(position, dtype=np.float32),
                yaw_deg=float(waypoint.get("yaw_deg", 0.0)),
            )
        )
    return _validate_quadrotor_trajectory(waypoints)


def _validate_quadrotor_trajectory(waypoints: list[QuadrotorWaypoint]) -> list[QuadrotorWaypoint]:
    if not waypoints:
        raise ValueError("Quadrotor trajectory must contain at least one waypoint.")
    previous_time = None
    for waypoint in waypoints:
        if previous_time is not None and waypoint.time <= previous_time:
            raise ValueError("Quadrotor trajectory times must be strictly increasing.")
        previous_time = waypoint.time
    return waypoints


class QuadrotorTrajectoryController:
    """Kinematic quadrotor controller compatible with TeleopDispatcher."""

    name = "quadrotor"

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
        prim_path: str,
        asset_path: str,
        initial_position,
        initial_yaw_deg: float,
        scale: float,
        trajectory: list[QuadrotorWaypoint] | None = None,
        loop_trajectory: bool = False,
        autostart: bool = True,
        manual_speed: float = 0.5,
        manual_yaw_rate_deg: float = 45.0,
        manual_dt: float = 1.0 / 60.0,
        camera_resolution: tuple[int, int] = (640, 480),
        camera_pitch_deg: float = 90.0,
        label: str = "",
    ):
        self._stage = stage
        self._sim_app = sim_app
        self._carb = carb
        self._Gf = gf_module
        self._Sdf = sdf_module
        self._Usd = usd_module
        self._UsdGeom = usdgeom_module
        self._UsdPhysics = usdphysics_module
        self.prim_path = prim_path.rstrip("/")
        self.label = label.strip() or self.prim_path.rsplit("/", maxsplit=1)[-1]
        self.asset_path = str(Path(asset_path).expanduser()) if asset_path else ""
        self.scale = float(scale)
        self.trajectory = list(trajectory or [])
        self.loop_trajectory = bool(loop_trajectory)
        self.playing = bool(autostart and self.trajectory)
        self.manual_speed = float(manual_speed)
        self.manual_yaw_rate_deg = float(manual_yaw_rate_deg)
        self.manual_dt = float(manual_dt)
        self.camera_resolution = camera_resolution
        self.camera_pitch_deg = float(camera_pitch_deg)
        self.active_target_mode = "trajectory"
        self.available = False
        self.camera_path = f"{self.prim_path}/rgbd_camera/Camera"
        self._trajectory_time = 0.0
        self._position = np.array(initial_position, dtype=np.float32)
        self._yaw_deg = float(initial_yaw_deg)
        self._translate_op = None
        self._orient_op = None
        self._scale_op = None
        self._initialize_asset()

    def _initialize_asset(self) -> None:
        asset_path = Path(self.asset_path).expanduser()
        if not asset_path.exists():
            self._carb.log_warn(f"Quadrotor USD not found: {asset_path}")
            return
        prim = self._stage.GetPrimAtPath(self.prim_path)
        if not prim.IsValid():
            prim = self._UsdGeom.Xform.Define(self._stage, self.prim_path).GetPrim()
            prim.GetReferences().AddReference(str(asset_path))
        try:
            self._stage.Load(prim.GetPath(), self._Usd.LoadWithDescendants)
        except TypeError:
            self._stage.Load(prim.GetPath())
        except Exception as exc:
            self._carb.log_warn(f"Could not explicitly load quadrotor {self.prim_path}: {exc}")
        for _ in range(3):
            self._sim_app.update()
        self._disable_asset_physics(prim)
        self._reset_pose_ops(prim)
        self._create_rgbd_camera()
        self._apply_pose()
        self.available = True
        self._carb.log_warn(
            f"Quadrotor ready: prim={self.prim_path}, camera={self.camera_path}, "
            f"trajectory_waypoints={len(self.trajectory)}"
        )

    def _disable_asset_physics(self, root_prim) -> None:
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
            prim.CreateAttribute("physxRigidBody:disableGravity", self._Sdf.ValueTypeNames.Bool).Set(True)

    def _reset_pose_ops(self, prim) -> None:
        xformable = self._UsdGeom.Xformable(prim)
        try:
            xformable.ClearXformOpOrder()
        except Exception:
            pass
        self._translate_op = xformable.AddTranslateOp()
        self._orient_op = xformable.AddOrientOp()
        self._scale_op = xformable.AddScaleOp()
        xformable.SetXformOpOrder([self._translate_op, self._orient_op, self._scale_op], True)

    def _create_rgbd_camera(self) -> None:
        camera_root_path = f"{self.prim_path}/rgbd_camera"
        camera_root = self._UsdGeom.Xform.Define(self._stage, camera_root_path)
        camera_root_xform = self._UsdGeom.Xformable(camera_root.GetPrim())
        try:
            camera_root_xform.ClearXformOpOrder()
        except Exception:
            pass
        camera_root_xform.AddTranslateOp().Set(self._Gf.Vec3d(0.12, 0.0, 0.03))
        camera_root_xform.AddRotateXYZOp().Set(
            self._Gf.Vec3f(float(self.camera_pitch_deg), 0.0, 0.0)
        )
        camera = self._UsdGeom.Camera.Define(self._stage, self.camera_path)
        camera.GetClippingRangeAttr().Set(self._Gf.Vec2f(0.01, 250.0))
        camera.GetFocalLengthAttr().Set(12.0)
        camera.GetHorizontalApertureAttr().Set(20.955)
        camera.GetVerticalApertureAttr().Set(15.2908)
        camera_prim = camera.GetPrim()
        camera_prim.CreateAttribute("tiangong:rgbd:enabled", self._Sdf.ValueTypeNames.Bool).Set(True)
        camera_prim.CreateAttribute("tiangong:rgbd:width", self._Sdf.ValueTypeNames.Int).Set(
            int(self.camera_resolution[0])
        )
        camera_prim.CreateAttribute("tiangong:rgbd:height", self._Sdf.ValueTypeNames.Int).Set(
            int(self.camera_resolution[1])
        )

    def _yaw_quat(self):
        yaw_rotation = self._Gf.Rotation(self._Gf.Vec3d(0.0, 0.0, 1.0), self._yaw_deg)
        yaw_quat = yaw_rotation.GetQuat()
        return self._Gf.Quatf(
            float(yaw_quat.GetReal()),
            self._Gf.Vec3f(
                float(yaw_quat.GetImaginary()[0]),
                float(yaw_quat.GetImaginary()[1]),
                float(yaw_quat.GetImaginary()[2]),
            ),
        )

    def _apply_pose(self) -> None:
        if self._translate_op is None or self._orient_op is None or self._scale_op is None:
            return
        self._translate_op.Set(
            self._Gf.Vec3d(float(self._position[0]), float(self._position[1]), float(self._position[2]))
        )
        self._orient_op.Set(self._yaw_quat())
        self._scale_op.Set(self._Gf.Vec3f(self.scale, self.scale, self.scale))

    def _sample_trajectory(self, time_value: float) -> tuple[np.ndarray, float]:
        if not self.trajectory:
            return np.array(self._position, dtype=np.float32), self._yaw_deg
        if len(self.trajectory) == 1 or time_value <= self.trajectory[0].time:
            first = self.trajectory[0]
            return np.array(first.position, dtype=np.float32), first.yaw_deg
        if time_value >= self.trajectory[-1].time:
            last = self.trajectory[-1]
            return np.array(last.position, dtype=np.float32), last.yaw_deg
        for left, right in zip(self.trajectory[:-1], self.trajectory[1:]):
            if left.time <= time_value <= right.time:
                alpha = (time_value - left.time) / max(right.time - left.time, 1e-6)
                position = left.position + (right.position - left.position) * alpha
                yaw = float(left.yaw_deg + (right.yaw_deg - left.yaw_deg) * alpha)
                return np.array(position, dtype=np.float32), yaw
        last = self.trajectory[-1]
        return np.array(last.position, dtype=np.float32), last.yaw_deg

    def update(self, dt: float) -> None:
        """Advance trajectory playback by one frame."""
        if not self.available or not self.playing or not self.trajectory:
            return
        self._trajectory_time += max(float(dt), 0.0)
        end_time = self.trajectory[-1].time
        if self.loop_trajectory and end_time > 0.0:
            self._trajectory_time = self._trajectory_time % end_time
        elif self._trajectory_time >= end_time:
            self._trajectory_time = end_time
            self.playing = False
        self._position, self._yaw_deg = self._sample_trajectory(self._trajectory_time)
        self._apply_pose()

    def reset_trajectory(self) -> None:
        """Reset playback to the first trajectory waypoint."""
        self._trajectory_time = 0.0
        if self.trajectory:
            self._position, self._yaw_deg = self._sample_trajectory(0.0)
            self._apply_pose()

    def toggle_playing(self) -> bool:
        """Pause or resume trajectory playback."""
        if not self.trajectory:
            self.playing = False
            return False
        self.playing = not self.playing
        return self.playing

    def cycle_target_mode(self) -> str:
        """Keep dispatcher target switching harmless for the quadrotor."""
        return self.active_target_mode

    def get_base_world_pose(self):
        """Return the current kinematic pose for target/status utilities."""
        quat = self._yaw_quat()
        return (
            np.array(self._position, dtype=np.float32),
            np.array(
                [
                    float(quat.GetReal()),
                    float(quat.GetImaginary()[0]),
                    float(quat.GetImaginary()[1]),
                    float(quat.GetImaginary()[2]),
                ],
                dtype=np.float32,
            ),
        )

    def step(self, base_command, manipulator_command) -> None:
        """Apply manual kinematic control while trajectory playback is paused."""
        if not self.available or self.playing:
            return
        forward = float(base_command.forward)
        strafe = float(base_command.strafe)
        lift = float(base_command.lift)
        yaw_axis = float(base_command.yaw)
        if forward == 0.0 and strafe == 0.0 and lift == 0.0 and yaw_axis == 0.0:
            return
        yaw_rad = np.deg2rad(self._yaw_deg)
        dx = forward * np.cos(yaw_rad) - strafe * np.sin(yaw_rad)
        dy = forward * np.sin(yaw_rad) + strafe * np.cos(yaw_rad)
        self._position = self._position + np.array(
            [dx, dy, lift], dtype=np.float32
        ) * self.manual_speed * self.manual_dt
        self._yaw_deg += yaw_axis * self.manual_yaw_rate_deg * self.manual_dt
        self._apply_pose()


def sample_quadrotor_trajectory_state(
    waypoints: list[QuadrotorWaypoint],
    time_value: float,
    fallback_position,
    fallback_yaw_deg: float,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Sample (position, velocity, yaw_deg, yaw_rate_deg/s) from a waypoint list.

    Velocity is computed analytically from the active segment; yaw rate is the
    segment yaw slope. Clamped to the first/last waypoint outside the range.
    """
    if not waypoints:
        return (
            np.array(fallback_position, dtype=np.float32),
            np.zeros(3, dtype=np.float32),
            float(fallback_yaw_deg),
            0.0,
        )
    if len(waypoints) == 1 or time_value <= waypoints[0].time:
        first = waypoints[0]
        return (np.array(first.position, dtype=np.float32), np.zeros(3, dtype=np.float32), first.yaw_deg, 0.0)
    if time_value >= waypoints[-1].time:
        last = waypoints[-1]
        return (np.array(last.position, dtype=np.float32), np.zeros(3, dtype=np.float32), last.yaw_deg, 0.0)
    for left, right in zip(waypoints[:-1], waypoints[1:]):
        if left.time <= time_value <= right.time:
            span = max(right.time - left.time, 1e-6)
            alpha = (time_value - left.time) / span
            position = left.position + (right.position - left.position) * alpha
            velocity = (right.position - left.position) / span
            yaw = float(left.yaw_deg + (right.yaw_deg - left.yaw_deg) * alpha)
            yaw_rate = float((right.yaw_deg - left.yaw_deg) / span)
            return (np.array(position, dtype=np.float32), np.array(velocity, dtype=np.float32), yaw, yaw_rate)
    last = waypoints[-1]
    return (np.array(last.position, dtype=np.float32), np.zeros(3, dtype=np.float32), last.yaw_deg, 0.0)


class QuadrotorPhysicsTrajectoryController:
    """Physics-based 6-DOF quadrotor flight controller.

    Unlike :class:`QuadrotorTrajectoryController` (which kinematically teleports
    the prim), this keeps the Crazyflie articulation's rigid-body physics active
    (gravity, collisions, revolute rotor joints) and drives the body link to
    follow a world-frame trajectory using a cascaded SE3 geometric controller:

        position PD  -> desired acceleration / thrust vector
        attitude PD  -> body torque (Lee et al. geometric SO(3) controller)

    Total thrust is applied as a world-frame force along the current body z-axis
    at the center of mass; the desired torque is applied as a world-frame torque.
    The four rotor joints are spun via angular velocity drives for visual realism
    (aerodynamic thrust is modeled by the applied body force, not the rotors).

    The public interface mirrors :class:`QuadrotorTrajectoryController` so the
    same :class:`QuadrotorFleetController` can wrap either controller.
    """

    name = "quadrotor"

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
        prim_path: str,
        asset_path: str,
        initial_position,
        initial_yaw_deg: float,
        scale: float,
        trajectory: list[QuadrotorWaypoint] | None = None,
        loop_trajectory: bool = False,
        autostart: bool = True,
        manual_speed: float = 0.5,
        manual_yaw_rate_deg: float = 45.0,
        manual_dt: float = 1.0 / 60.0,
        camera_resolution: tuple[int, int] = (640, 480),
        camera_pitch_deg: float = 90.0,
        label: str = "",
        world=None,
        gravity: float = 9.81,
        mass: float | None = None,
        inertia: np.ndarray | None = None,
        kp_pos: float = 6.0,
        kd_pos: float = 3.0,
        k_att: float = 8.0e-3,
        k_omega: float = 1.5e-3,
        max_thrust: float | None = None,
        max_torque: float = 0.01,
        rotor_spin_velocity: float = 1200.0,
        obstacle_avoidance: bool = False,
        apf_range: float = 1.5,
        apf_strength: float = 2.0,
        apf_min_distance: float = 0.3,
        apf_depth_step: int = 12,
        apf_focal_length: float = 12.0,
    ):
        self._stage = stage
        self._sim_app = sim_app
        self._carb = carb
        self._Gf = gf_module
        self._Sdf = sdf_module
        self._Usd = usd_module
        self._UsdGeom = usdgeom_module
        self._UsdPhysics = usdphysics_module
        self.prim_path = prim_path.rstrip("/")
        self.label = label.strip() or self.prim_path.rsplit("/", maxsplit=1)[-1]
        self.asset_path = str(Path(asset_path).expanduser()) if asset_path else ""
        self.scale = float(scale)
        self.trajectory = list(trajectory or [])
        self.loop_trajectory = bool(loop_trajectory)
        self._autostart = bool(autostart)
        self.manual_speed = float(manual_speed)
        self.manual_yaw_rate_deg = float(manual_yaw_rate_deg)
        self.manual_dt = float(manual_dt)
        self.camera_resolution = camera_resolution
        self.camera_pitch_deg = float(camera_pitch_deg)
        self.active_target_mode = "trajectory"
        self.available = False
        self._world = world
        self._gravity = float(gravity)
        self._kp_pos = float(kp_pos)
        self._kd_pos = float(kd_pos)
        self._k_att = float(k_att)
        self._k_omega = float(k_omega)
        self._max_torque = float(max_torque)
        self._rotor_spin_velocity = float(rotor_spin_velocity)
        self._obstacle_avoidance = bool(obstacle_avoidance)
        self._apf_range = float(apf_range)
        self._apf_strength = float(apf_strength)
        self._apf_min_distance = float(apf_min_distance)
        self._apf_depth_step = int(apf_depth_step)
        self._apf_focal_length = float(apf_focal_length)
        self._obstacle_camera = None
        self._obstacle_camera_initialized = False
        self.camera_path = f"{self.prim_path}/body/rgbd_camera/Camera"
        self._trajectory_time = 0.0
        self._initial_position = np.array(initial_position, dtype=np.float32)
        self._initial_yaw_deg = float(initial_yaw_deg)
        self._setpoint_pos = np.array(self._initial_position, dtype=np.float32)
        self._setpoint_yaw_deg = float(self._initial_yaw_deg)
        self._setpoint_vel = np.zeros(3, dtype=np.float32)
        self._setpoint_yaw_rate = 0.0
        self._body_prim_path = f"{self.prim_path}/body"
        self._joint_names = ["m1_joint", "m2_joint", "m3_joint", "m4_joint"]
        self._view = None
        self._view_initialized = False
        self._physics_reset_done = False
        self._warned = set()
        self._diag_logged = False
        self._diag_state_logged = False
        self._periodic_log_count = 0
        self._apf_log_count = 0
        self._physx_sim_interface = None
        self._physx_stage_id = None
        self._physx_body_token = None
        self._mass = None
        self._inertia = None
        self._max_thrust = None
        self._initialize_asset(mass_override=mass, inertia_override=inertia, max_thrust_override=max_thrust)
        self.playing = bool(self._autostart and self.trajectory)

    # ---- asset / physics setup -------------------------------------------------

    def _initialize_asset(self, mass_override, inertia_override, max_thrust_override) -> None:
        asset_path = Path(self.asset_path).expanduser()
        if not asset_path.exists():
            self._carb.log_warn(f"Quadrotor USD not found: {asset_path}")
            return
        prim = self._stage.GetPrimAtPath(self.prim_path)
        if not prim.IsValid():
            prim = self._UsdGeom.Xform.Define(self._stage, self.prim_path).GetPrim()
            prim.GetReferences().AddReference(str(asset_path))
        try:
            self._stage.Load(prim.GetPath(), self._Usd.LoadWithDescendants)
        except TypeError:
            self._stage.Load(prim.GetPath())
        except Exception as exc:
            self._carb.log_warn(f"Could not explicitly load quadrotor {self.prim_path}: {exc}")
        for _ in range(3):
            self._sim_app.update()
        self._enable_asset_physics(prim)
        self._set_initial_usd_pose()
        self._add_rotor_drives()
        self._read_dynamics_parameters(mass_override, inertia_override, max_thrust_override)
        self._create_rgbd_camera()
        self._create_trajectory_visualization()
        self.available = True
        self._carb.log_warn(
            f"Quadrotor (physics) ready: prim={self.prim_path}, body={self._body_prim_path}, "
            f"camera={self.camera_path}, mass={self._mass:.4f} kg, "
            f"max_thrust={self._max_thrust:.3f} N, waypoints={len(self.trajectory)}"
        )

    def _enable_asset_physics(self, root_prim) -> None:
        prop_names = {"m1_prop", "m2_prop", "m3_prop", "m4_prop"}
        try:
            if root_prim.HasAPI(self._UsdPhysics.ArticulationRootAPI):
                root_prim.RemoveAPI(self._UsdPhysics.ArticulationRootAPI)
                self._carb.log_warn(
                    f"Removed ArticulationRootAPI from {self.prim_path} so body acts as a free rigid body "
                    "(external forces apply directly via RigidPrimView / PhysX)."
                )
        except Exception as exc:
            self._carb.log_warn(f"Could not remove ArticulationRootAPI from {self.prim_path}: {exc}")
        for prim in self._Usd.PrimRange(root_prim):
            if not prim.IsValid():
                continue
            type_name = prim.GetTypeName()
            prim_name = prim.GetName()
            if type_name.startswith("PhysicsJoint"):
                prim.SetActive(True)
                continue
            is_prop = prim_name in prop_names
            rigid_body = self._UsdPhysics.RigidBodyAPI(prim)
            if rigid_body:
                rigid_body.CreateRigidBodyEnabledAttr(True)
                rigid_body.CreateKinematicEnabledAttr(False)
                prim.CreateAttribute("physxRigidBody:disableGravity", self._Sdf.ValueTypeNames.Bool).Set(False)
            collision = self._UsdPhysics.CollisionAPI(prim)
            if collision:
                collision.CreateCollisionEnabledAttr(not is_prop)

    def _set_initial_usd_pose(self) -> None:
        root_prim = self._stage.GetPrimAtPath(self.prim_path)
        if not root_prim.IsValid():
            return
        try:
            xformable = self._UsdGeom.Xformable(root_prim)
            try:
                xformable.ClearXformOpOrder()
            except Exception:
                pass
            translate_op = xformable.AddTranslateOp()
            orient_op = xformable.AddOrientOp()
            yaw_quat = self._yaw_quat_gf(self._setpoint_yaw_deg)
            translate_op.Set(self._Gf.Vec3d(*[float(v) for v in self._setpoint_pos]))
            orient_op.Set(yaw_quat)
            xformable.SetXformOpOrder([translate_op, orient_op], True)
        except Exception as exc:
            self._carb.log_warn(f"Could not set initial USD pose for root: {exc}")

    def _yaw_quat_gf(self, yaw_deg: float):
        yaw_rotation = self._Gf.Rotation(self._Gf.Vec3d(0.0, 0.0, 1.0), float(yaw_deg))
        yaw_quat = yaw_rotation.GetQuat()
        return self._Gf.Quatf(
            float(yaw_quat.GetReal()),
            self._Gf.Vec3f(
                float(yaw_quat.GetImaginary()[0]),
                float(yaw_quat.GetImaginary()[1]),
                float(yaw_quat.GetImaginary()[2]),
            ),
        )

    def _add_rotor_drives(self) -> None:
        spin = self._rotor_spin_velocity
        directions = {"m1_joint": 1.0, "m2_joint": -1.0, "m3_joint": 1.0, "m4_joint": -1.0}
        drive_cls = getattr(self._UsdPhysics, "PhysicsDriveAPI", None) or getattr(
            self._UsdPhysics, "DriveAPI", None
        )
        for joint_name in self._joint_names:
            joint_prim = self._stage.GetPrimAtPath(f"{self.prim_path}/{joint_name}")
            if not joint_prim.IsValid():
                continue
            target_velocity = float(spin * directions[joint_name])
            applied = False
            if drive_cls is not None:
                try:
                    drive = drive_cls.Apply(joint_prim, "angular")
                    drive.CreateStiffnessAttr().Set(0.0)
                    drive.CreateDampingAttr().Set(1.0e-4)
                    drive.CreateMaxForceAttr().Set(1.0e-3)
                    drive.CreateTargetVelocityAttr().Set(target_velocity)
                    applied = True
                except Exception as exc:
                    self._carb.log_warn(f"Could not add rotor drive API for {joint_name}: {exc}")
            if not applied:
                try:
                    joint_prim.CreateAttribute(
                        "drive:angular:physics:targetVelocity", self._Sdf.ValueTypeNames.Float
                    ).Set(target_velocity)
                    joint_prim.CreateAttribute(
                        "drive:angular:physics:damping", self._Sdf.ValueTypeNames.Float
                    ).Set(1.0e-4)
                    joint_prim.CreateAttribute(
                        "drive:angular:physics:stiffness", self._Sdf.ValueTypeNames.Float
                    ).Set(0.0)
                    joint_prim.CreateAttribute(
                        "drive:angular:physics:maxForce", self._Sdf.ValueTypeNames.Float
                    ).Set(1.0e-3)
                except Exception as exc:
                    self._carb.log_warn(f"Could not set rotor drive attrs for {joint_name}: {exc}")

    def _read_dynamics_parameters(self, mass_override, inertia_override, max_thrust_override) -> None:
        body_prim = self._stage.GetPrimAtPath(self._body_prim_path)
        body_mass = 0.025
        body_inertia = np.array([1.6572e-5, 1.6656e-5, 2.9262e-5], dtype=np.float32)
        if body_prim.IsValid():
            mass_attr = body_prim.GetAttribute("physics:mass")
            if mass_attr and mass_attr.Get() is not None:
                body_mass = float(mass_attr.Get())
            inertia_attr = body_prim.GetAttribute("physics:diagonalInertia")
            if inertia_attr and inertia_attr.Get() is not None:
                body_inertia = np.array(inertia_attr.Get(), dtype=np.float32)
        prop_mass_total = 0.0
        for prop_name in ("m1_prop", "m2_prop", "m3_prop", "m4_prop"):
            prop_prim = self._stage.GetPrimAtPath(f"{self.prim_path}/{prop_name}")
            if prop_prim.IsValid():
                prop_mass_attr = prop_prim.GetAttribute("physics:mass")
                if prop_mass_attr and prop_mass_attr.Get() is not None:
                    prop_mass_total += float(prop_mass_attr.Get())
        if mass_override is not None:
            self._mass = float(mass_override)
        else:
            self._mass = float(body_mass + prop_mass_total)
        if inertia_override is not None:
            self._inertia = np.array(inertia_override, dtype=np.float32)
        else:
            self._inertia = body_inertia
        if max_thrust_override is not None:
            self._max_thrust = float(max_thrust_override)
        else:
            self._max_thrust = float(4.0 * self._mass * self._gravity)

    def _create_rgbd_camera(self) -> None:
        camera_root_path = f"{self._body_prim_path}/rgbd_camera"
        camera_root = self._UsdGeom.Xform.Define(self._stage, camera_root_path)
        camera_root_xform = self._UsdGeom.Xformable(camera_root.GetPrim())
        try:
            camera_root_xform.ClearXformOpOrder()
        except Exception:
            pass
        offset = 0.12 * self.scale
        camera_root_xform.AddTranslateOp().Set(self._Gf.Vec3d(offset, 0.0, 0.03 * self.scale))
        camera_root_xform.AddRotateXYZOp().Set(
            self._Gf.Vec3f(float(self.camera_pitch_deg), 0.0, 0.0)
        )
        camera = self._UsdGeom.Camera.Define(self._stage, self.camera_path)
        camera.GetClippingRangeAttr().Set(self._Gf.Vec2f(0.01, 250.0))
        camera.GetFocalLengthAttr().Set(12.0)
        camera.GetHorizontalApertureAttr().Set(20.955)
        camera.GetVerticalApertureAttr().Set(15.2908)
        camera_prim = camera.GetPrim()
        camera_prim.CreateAttribute("tiangong:rgbd:enabled", self._Sdf.ValueTypeNames.Bool).Set(True)
        camera_prim.CreateAttribute("tiangong:rgbd:width", self._Sdf.ValueTypeNames.Int).Set(
            int(self.camera_resolution[0])
        )
        camera_prim.CreateAttribute("tiangong:rgbd:height", self._Sdf.ValueTypeNames.Int).Set(
            int(self.camera_resolution[1])
        )

    def _create_trajectory_visualization(self) -> None:
        self._actual_path_points: list[np.ndarray] = []
        self._actual_path_max = 2000
        self._viz_path_update_counter = 0
        viz_root = f"/World/trajectory_viz_{self.label}"
        desired_path = f"{viz_root}/desired_trajectory"
        actual_path = f"{viz_root}/actual_trajectory"
        desired_curve = self._UsdGeom.BasisCurves.Define(self._stage, desired_path)
        desired_curve.CreateTypeAttr().Set("linear")
        desired_curve.CreateWrapAttr().Set("nonperiodic")
        desired_curve.GetCurveVertexCountsAttr().Set([0])
        color_attr = desired_curve.CreateDisplayColorAttr()
        color_attr.Set([self._Gf.Vec3f(1.0, 0.5, 0.0)])
        width_attr = desired_curve.CreateWidthsAttr()
        width_attr.Set([0.02])
        actual_curve = self._UsdGeom.BasisCurves.Define(self._stage, actual_path)
        actual_curve.CreateTypeAttr().Set("linear")
        actual_curve.CreateWrapAttr().Set("nonperiodic")
        actual_curve.GetCurveVertexCountsAttr().Set([0])
        actual_color_attr = actual_curve.CreateDisplayColorAttr()
        actual_color_attr.Set([self._Gf.Vec3f(0.0, 1.0, 0.0)])
        actual_width_attr = actual_curve.CreateWidthsAttr()
        actual_width_attr.Set([0.015])
        self._desired_curve = desired_curve
        self._actual_curve = actual_curve
        if self.trajectory:
            dense_points = self._sample_dense_trajectory_points(num_per_segment=20)
            gf_points = [self._Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in dense_points]
            self._desired_curve.GetPointsAttr().Set(gf_points)
            self._desired_curve.GetCurveVertexCountsAttr().Set([len(gf_points)])
            self._carb.log_warn(
                f"Trajectory viz[{self.label}]: desired path drawn with {len(gf_points)} points (orange, width=0.02m), "
                f"actual path will be drawn in green (width=0.015m)."
            )

    def _sample_dense_trajectory_points(self, num_per_segment: int = 20) -> list[np.ndarray]:
        if not self.trajectory:
            return []
        if len(self.trajectory) == 1:
            return [np.array(self.trajectory[0].position, dtype=np.float32)]
        points: list[np.ndarray] = []
        for left, right in zip(self.trajectory[:-1], self.trajectory[1:]):
            for i in range(num_per_segment):
                alpha = i / num_per_segment
                p = left.position + (right.position - left.position) * alpha
                points.append(np.array(p, dtype=np.float32))
        points.append(np.array(self.trajectory[-1].position, dtype=np.float32))
        return points

    def _update_actual_path_viz(self, pos: np.ndarray) -> None:
        self._actual_path_points.append(np.array(pos, dtype=np.float32))
        if len(self._actual_path_points) > self._actual_path_max:
            self._actual_path_points = self._actual_path_points[-self._actual_path_max:]
        self._viz_path_update_counter += 1
        if self._viz_path_update_counter % 10 != 0:
            return
        gf_points = [self._Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in self._actual_path_points]
        self._actual_curve.GetPointsAttr().Set(gf_points)
        self._actual_curve.GetCurveVertexCountsAttr().Set([len(gf_points)])

    def _warn_once(self, key: str, message: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self._carb.log_warn(message)

    # ---- rigid body view / state ----------------------------------------------

    def _ensure_view(self) -> bool:
        if self._view is None:
            try:
                from isaacsim.core.prims import RigidPrim as RigidPrimView
            except Exception as exc:
                self._carb.log_warn(f"RigidPrim unavailable for quadrotor physics: {exc}")
                return False
            try:
                self._view = RigidPrimView(prim_paths_expr=[self._body_prim_path], name=f"{self.label}_body_view")
            except Exception as exc:
                self._carb.log_warn(f"Failed to create RigidPrim for {self._body_prim_path}: {exc}")
                self._view = None
                return False
        if not self._view_initialized:
            initialized = False
            if self._world is not None:
                try:
                    self._view.initialize(self._world.physics_sim_view)
                    initialized = True
                except TypeError:
                    initialized = False
                except Exception as exc:
                    self._warn_once("view_init_simview", f"Quadrotor view initialize(physics_sim_view) failed: {exc}")
            if not initialized:
                try:
                    self._view.initialize()
                    initialized = True
                except Exception as exc:
                    self._warn_once("view_init", f"Quadrotor view initialize() failed: {exc}")
                    return False
            self._view_initialized = True
            self._reset_physics_state()
            self._carb.log_warn(
                f"Quadrotor physics view initialized: prim={self.prim_path}, "
                f"physics_handle_valid={self._view.is_physics_handle_valid()}, "
                f"hover_thrust={self._mass * self._gravity:.4f} N, max_thrust={self._max_thrust:.4f} N"
            )
        return True

    def _physics_handle_valid(self) -> bool:
        return self._view is not None and self._view.is_physics_handle_valid()

    def initialize_physics_view(self) -> bool:
        """Eagerly build the rigid body view and reset the body to the setpoint.

        Call after ``my_world.reset()`` so the physics simulation view exists;
        this wires the controller's PhysX tensor handle and teleports the body
        to the first trajectory waypoint with zero velocity before the main
        loop starts applying forces.
        """
        return self._ensure_view()

    def reset_to_setpoint(self) -> None:
        """Teleport the body to the current setpoint with zero velocity.

        Safe to call after ``initialize_physics_view`` and a few physics steps
        to overcome any drift accumulated while the handle was being set up.
        """
        if self._view is not None and self._physics_handle_valid():
            self._reset_physics_state()

    def _reset_physics_state(self) -> None:
        if self._view is None or not self._view_initialized:
            return
        try:
            velocities = np.zeros((1, 6), dtype=np.float32)
            self._view.set_velocities(velocities)
            verify_pos, _ = self._view.get_world_poses(usd=False)
            self._carb.log_warn(
                f"Quadrotor physics reset[{self.label}]: velocities zeroed, "
                f"current pos={np.round(np.asarray(verify_pos[0]),3).tolist()}"
            )
        except Exception as exc:
            self._carb.log_warn(f"Quadrotor physics reset failed for {self.prim_path}: {exc}")
        self._physics_reset_done = True

    def _read_state(self):
        positions, orientations = self._view.get_world_poses(usd=False)
        linear = self._view.get_linear_velocities()
        angular = self._view.get_angular_velocities()
        pos = np.array(positions[0], dtype=np.float32)
        quat_wxyz = np.array(orientations[0], dtype=np.float32)
        vel = np.array(linear[0], dtype=np.float32)
        omega_world = np.array(angular[0], dtype=np.float32)
        return pos, quat_wxyz, vel, omega_world

    def _read_state_safe(self):
        if self._physics_handle_valid():
            try:
                return self._read_state()
            except Exception as exc:
                self._warn_once("state_read", f"Quadrotor state read failed for {self.prim_path}: {exc}")
        pos = np.array(self._setpoint_pos, dtype=np.float32)
        quat_wxyz = self._yaw_to_quat_wxyz(self._setpoint_yaw_deg)
        vel = np.zeros(3, dtype=np.float32)
        omega_world = np.zeros(3, dtype=np.float32)
        try:
            from isaacsim.core.utils.xforms import get_world_pose  # noqa: PLC0415

            pose, quat = get_world_pose(self._body_prim_path, fabric=True)
            pos = np.array(pose, dtype=np.float32)
            quat_wxyz = np.array(quat, dtype=np.float32)
        except Exception:
            try:
                body_prim = self._stage.GetPrimAtPath(self._body_prim_path)
                xformable = self._UsdGeom.Xformable(body_prim)
                matrix = xformable.ComputeLocalToWorldTransform(0)
                t = matrix.ExtractTranslation()
                pos = np.array([t[0], t[1], t[2]], dtype=np.float32)
            except Exception:
                pass
        return pos, quat_wxyz, vel, omega_world

    # ---- rotation helpers ------------------------------------------------------

    @staticmethod
    def _quat_wxyz_to_rot(quat_wxyz) -> np.ndarray:
        w, x, y, z = float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])
        n = w * w + x * x + y * y + z * z
        if n < 1e-12:
            return np.eye(3, dtype=np.float32)
        s = 2.0 / n
        return np.array(
            [
                [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
                [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _yaw_to_quat_wxyz(yaw_deg: float) -> np.ndarray:
        yaw = np.deg2rad(float(yaw_deg))
        half = 0.5 * yaw
        return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=np.float32)

    @staticmethod
    def _rotation_error_vee(rot_desired: np.ndarray, rot: np.ndarray) -> np.ndarray:
        mismatch = 0.5 * (rot_desired.T @ rot - rot.T @ rot_desired)
        return np.array([mismatch[2, 1], mismatch[0, 2], mismatch[1, 0]], dtype=np.float32)

    @staticmethod
    def _desired_rotation_from_thrust_and_yaw(thrust_vec: np.ndarray, yaw_deg: float) -> np.ndarray:
        norm = float(np.linalg.norm(thrust_vec))
        if norm < 1e-6:
            b3 = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        else:
            b3 = (thrust_vec / norm).astype(np.float32)
        yaw = np.deg2rad(float(yaw_deg))
        forward_partial = np.array([np.cos(yaw), np.sin(yaw), 0.0], dtype=np.float32)
        cross = np.cross(b3, forward_partial)
        cross_norm = float(np.linalg.norm(cross))
        if cross_norm < 1e-6:
            forward_partial = np.array([1.0, 0.0, 0.0], dtype=np.float32)
            cross = np.cross(b3, forward_partial)
            cross_norm = float(np.linalg.norm(cross))
        b2 = (cross / cross_norm).astype(np.float32)
        b1 = np.cross(b2, b3).astype(np.float32)
        return np.stack([b1, b2, b3], axis=1).astype(np.float32)

    # ---- obstacle avoidance (APF via depth camera) ----------------------------

    def _init_obstacle_camera(self) -> bool:
        if not self._obstacle_avoidance:
            return False
        if self._obstacle_camera_initialized:
            return self._obstacle_camera is not None
        self._obstacle_camera_initialized = True
        try:
            from isaacsim.sensors.camera import Camera  # noqa: PLC0415

            self._obstacle_camera = Camera(
                prim_path=self.camera_path,
                resolution=self.camera_resolution,
            )
            self._obstacle_camera.initialize()
            self._obstacle_camera.add_distance_to_image_plane_to_frame()
            self._carb.log_warn(
                f"APF obstacle avoidance camera initialized: {self.camera_path}, "
                f"range={self._apf_range}m, strength={self._apf_strength}"
            )
            return True
        except Exception as exc:
            self._warn_once("apf_camera", f"APF obstacle camera init failed: {exc}")
            self._obstacle_camera = None
            return False

    def _read_depth(self) -> np.ndarray | None:
        if self._obstacle_camera is None:
            return None
        try:
            depth = self._obstacle_camera.get_depth()
            if depth is not None:
                return np.asarray(depth, dtype=np.float32)
            frame = self._obstacle_camera.get_current_frame()
            depth = frame.get("distance_to_image_plane", frame.get("depth"))
            if depth is not None:
                return np.asarray(depth, dtype=np.float32)
        except Exception as exc:
            self._warn_once("apf_depth", f"APF depth read failed: {exc}")
        return None

    def _depth_to_world_points(self, depth: np.ndarray, body_pos: np.ndarray, rot: np.ndarray) -> np.ndarray:
        h, w = depth.shape[:2]
        step = max(self._apf_depth_step, 1)
        rows = np.arange(0, h, step)
        cols = np.arange(0, w, step)
        grid_r, grid_c = np.meshgrid(rows, cols, indexing="ij")
        d = depth[grid_r, grid_c].astype(np.float32)
        mask = (d > 0.05) & (d < self._apf_range) & np.isfinite(d)
        if not np.any(mask):
            return np.empty((0, 3), dtype=np.float32)
        d_valid = d[mask]
        cx, cy = w * 0.5, h * 0.5
        fx = self._apf_focal_length * w / 20.955
        fy = self._apf_focal_length * h / 15.2908
        u = grid_c[mask].astype(np.float32)
        v = grid_r[mask].astype(np.float32)
        x_cam = (u - cx) * d_valid / fx
        y_cam = (v - cy) * d_valid / fy
        z_cam = -d_valid
        points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)
        camera_offset = np.array([0.12 * self.scale, 0.0, 0.03 * self.scale], dtype=np.float32)
        pitch_rad = np.deg2rad(self.camera_pitch_deg)
        cos_p, sin_p = np.cos(pitch_rad), np.sin(pitch_rad)
        pitch_rot = np.array(
            [[1, 0, 0], [0, cos_p, -sin_p], [0, sin_p, cos_p]], dtype=np.float32
        )
        points_body = (pitch_rot @ points_cam.T).T + camera_offset
        points_world = (rot @ points_body.T).T + body_pos
        points_world = points_world.astype(np.float32)
        relative = points_world - body_pos[np.newaxis, :]
        rel_dist = np.linalg.norm(relative, axis=1)
        height_diff = relative[:, 2]
        keep = (rel_dist > 0.15) & (rel_dist < self._apf_range) & (np.abs(height_diff) < 0.8)
        return points_world[keep]

    def _compute_apf_repulsion(self, body_pos: np.ndarray, obstacle_points: np.ndarray) -> np.ndarray:
        if len(obstacle_points) == 0:
            return np.zeros(3, dtype=np.float32)
        diff = body_pos[np.newaxis, :] - obstacle_points
        dist = np.linalg.norm(diff, axis=1)
        dist = np.maximum(dist, self._apf_min_distance)
        mask = dist < self._apf_range
        if not np.any(mask):
            return np.zeros(3, dtype=np.float32)
        d_valid = dist[mask]
        diff_valid = diff[mask]
        if len(d_valid) > 30:
            nearest_idx = np.argsort(d_valid)[:30]
            d_valid = d_valid[nearest_idx]
            diff_valid = diff_valid[nearest_idx]
        directions = diff_valid / d_valid[:, np.newaxis]
        eta = self._apf_strength
        range_val = self._apf_range
        repulsion_mag = eta * ((1.0 / d_valid - 1.0 / range_val) ** 2)
        repulsion_mag = np.minimum(repulsion_mag, 5.0)
        repulsion = directions * repulsion_mag[:, np.newaxis]
        total_repulsion = repulsion.sum(axis=0)
        total_repulsion[2] = 0.0
        max_rep = 4.0
        norm = float(np.linalg.norm(total_repulsion))
        if norm > max_rep:
            total_repulsion = total_repulsion * (max_rep / norm)
        return total_repulsion.astype(np.float32)

    # ---- control ---------------------------------------------------------------

    def _compute_control(self, pos, quat_wxyz, vel, omega_world) -> tuple[np.ndarray, np.ndarray]:
        rot = self._quat_wxyz_to_rot(quat_wxyz)
        gravity_vec = np.array([0.0, 0.0, self._gravity], dtype=np.float32)
        accel_des = (
            self._kp_pos * (self._setpoint_pos - pos)
            + self._kd_pos * (self._setpoint_vel - vel)
            + gravity_vec
        )
        if self._obstacle_avoidance and self._obstacle_camera is not None:
            depth = self._read_depth()
            if depth is not None and depth.ndim == 2:
                obstacle_points = self._depth_to_world_points(depth, pos, rot)
                repulsion = self._compute_apf_repulsion(pos, obstacle_points)
                accel_des = accel_des + repulsion
                self._apf_log_count += 1
                if self._apf_log_count % 120 == 0:
                    self._carb.log_warn(
                        f"APF[{self.label}]: obstacles={len(obstacle_points)} "
                        f"repulsion={np.round(repulsion,3).tolist()} "
                        f"|accel|={np.linalg.norm(accel_des):.2f}"
                    )
        accel_norm = float(np.linalg.norm(accel_des))
        max_accel = 25.0
        if accel_norm > max_accel:
            accel_des = accel_des * (max_accel / accel_norm)
        force_des = (self._mass * accel_des).astype(np.float32)
        rot_desired = self._desired_rotation_from_thrust_and_yaw(force_des, self._setpoint_yaw_deg)
        b3_current = rot[:, 2]
        thrust = float(np.dot(force_des, b3_current))
        thrust = max(0.0, min(thrust, self._max_thrust))
        omega_body = (rot.T @ omega_world).astype(np.float32)
        omega_des_world = np.array([0.0, 0.0, np.deg2rad(self._setpoint_yaw_rate)], dtype=np.float32)
        omega_des_body = (rot.T @ rot_desired @ omega_des_world).astype(np.float32)
        rot_error = self._rotation_error_vee(rot_desired, rot)
        omega_error = omega_body - omega_des_body
        torque_body = (-self._k_att * rot_error - self._k_omega * omega_error).astype(np.float32)
        torque_body = np.clip(torque_body, -self._max_torque, self._max_torque)
        world_force = (thrust * b3_current).astype(np.float32)
        b3_z = float(b3_current[2])
        if b3_z < 0.0:
            recover = self._mass * self._gravity * min(1.0, 1.0 + b3_z)
            world_force = world_force + np.array([0.0, 0.0, recover], dtype=np.float32)
            recover_torque = np.array([-5.0 * b3_current[1], 5.0 * b3_current[0], 0.0], dtype=np.float32)
            torque_body = torque_body + np.clip(recover_torque, -self._max_torque, self._max_torque)
        world_torque = (rot @ torque_body).astype(np.float32)
        return world_force, world_torque

    def _apply_wrench(self, world_force: np.ndarray, world_torque: np.ndarray, body_pos: np.ndarray | None = None) -> None:
        applied = False
        method = "none"
        try:
            self._apply_wrench_physx(world_force, world_torque, body_pos)
            applied = True
            method = "PhysX"
        except Exception as exc:
            self._warn_once("apply_wrench_physx", f"Quadrotor apply wrench (PhysX) failed for {self.prim_path}: {exc}")
        if not applied and self._physics_handle_valid():
            try:
                self._view.apply_forces_and_torques_at_pos(
                    forces=np.array([world_force], dtype=np.float32),
                    torques=np.array([world_torque], dtype=np.float32),
                    is_global=True,
                )
                applied = True
                method = "RigidPrimView"
            except Exception as exc:
                self._warn_once("apply_wrench_view", f"Quadrotor apply wrench (RigidPrimView) failed: {exc}")
        if not applied:
            self._warn_once(
                "handle_invalid",
                f"Quadrotor could not apply force for {self.prim_path}; body will free-fall.",
            )
            return
        if not self._diag_logged:
            self._diag_logged = True
            self._carb.log_warn(
                f"Quadrotor control diag[{self.label}]: first wrench applied "
                f"force={np.round(world_force,4).tolist()} torque={np.round(world_torque,5).tolist()} "
                f"via={method}"
            )

    def _apply_wrench_physx(self, world_force: np.ndarray, world_torque: np.ndarray, body_pos: np.ndarray | None = None) -> None:
        if self._physx_sim_interface is None:
            try:
                import omni.physx  # noqa: PLC0415
                from pxr import UsdUtils, PhysicsSchemaTools  # noqa: PLC0415

                self._physx_sim_interface = omni.physx.get_physx_simulation_interface()
                self._physx_stage_id = UsdUtils.StageCache.Get().GetId(self._stage).ToLongInt()
                self._physx_body_token = PhysicsSchemaTools.sdfPathToInt(self._body_prim_path)
            except Exception as exc:
                raise RuntimeError(f"omni.physx simulation interface unavailable: {exc}") from exc
        fx, fy, fz = float(world_force[0]), float(world_force[1]), float(world_force[2])
        tx, ty, tz = float(world_torque[0]), float(world_torque[1]), float(world_torque[2])
        if body_pos is not None:
            px, py, pz = float(body_pos[0]), float(body_pos[1]), float(body_pos[2])
        else:
            px, py, pz = float(self._setpoint_pos[0]), float(self._setpoint_pos[1]), float(self._setpoint_pos[2])
        self._physx_sim_interface.apply_force_at_pos(
            self._physx_stage_id,
            self._physx_body_token,
            self._carb.Float3(fx, fy, fz),
            self._carb.Float3(px, py, pz),
            "Force",
        )
        self._physx_sim_interface.apply_torque(
            self._physx_stage_id,
            self._physx_body_token,
            self._carb.Float3(tx, ty, tz),
        )

    # ---- public interface (mirrors QuadrotorTrajectoryController) --------------

    def update(self, dt: float) -> None:
        if not self.available:
            return
        self._ensure_view()
        if self._obstacle_avoidance:
            self._init_obstacle_camera()
        pos, quat_wxyz, vel, omega_world = self._read_state_safe()
        if not self._diag_state_logged:
            self._diag_state_logged = True
            self._carb.log_warn(
                f"Quadrotor state diag[{self.label}]: pos={np.round(pos,3).tolist()} "
                f"quat_wxyz={np.round(quat_wxyz,4).tolist()} vel={np.round(vel,3).tolist()} "
                f"setpoint={np.round(self._setpoint_pos,3).tolist()} handle={self._physics_handle_valid()}"
            )
        if self.playing and self.trajectory:
            self._trajectory_time += max(float(dt), 0.0)
            end_time = self.trajectory[-1].time
            if self.loop_trajectory and end_time > 0.0:
                self._trajectory_time = self._trajectory_time % end_time
            elif self._trajectory_time >= end_time:
                self._trajectory_time = end_time
                self.playing = False
            pos_des, vel_des, yaw_des, yaw_rate_des = sample_quadrotor_trajectory_state(
                self.trajectory, self._trajectory_time, self._setpoint_pos, self._setpoint_yaw_deg
            )
            self._setpoint_pos = pos_des
            self._setpoint_vel = vel_des
            self._setpoint_yaw_deg = yaw_des
            self._setpoint_yaw_rate = yaw_rate_des
        else:
            self._setpoint_vel = np.zeros(3, dtype=np.float32)
            self._setpoint_yaw_rate = 0.0
        world_force, world_torque = self._compute_control(pos, quat_wxyz, vel, omega_world)
        self._apply_wrench(world_force, world_torque, body_pos=pos)
        self._update_actual_path_viz(pos)
        self._periodic_log_count += 1
        if self._periodic_log_count % 120 == 0:
            rot = self._quat_wxyz_to_rot(quat_wxyz)
            b3 = rot[:, 2]
            self._carb.log_warn(
                f"Quadrotor periodic[{self.label}]: pos={np.round(pos,3).tolist()} "
                f"vel={np.round(vel,3).tolist()} b3={np.round(b3,3).tolist()} "
                f"force={np.round(world_force,4).tolist()} sp={np.round(self._setpoint_pos,3).tolist()}"
            )

    def reset_trajectory(self) -> None:
        self._trajectory_time = 0.0
        if self.trajectory:
            pos_des, vel_des, yaw_des, yaw_rate_des = sample_quadrotor_trajectory_state(
                self.trajectory, 0.0, self._setpoint_pos, self._setpoint_yaw_deg
            )
            self._setpoint_pos = pos_des
            self._setpoint_vel = vel_des
            self._setpoint_yaw_deg = yaw_des
            self._setpoint_yaw_rate = yaw_rate_des
        if self._view is not None and self._view_initialized:
            self._reset_physics_state()

    def toggle_playing(self) -> bool:
        if not self.trajectory:
            self.playing = False
            return False
        self.playing = not self.playing
        return self.playing

    def cycle_target_mode(self) -> str:
        return self.active_target_mode

    def get_base_world_pose(self):
        pos = np.array(self._setpoint_pos, dtype=np.float32)
        quat = self._yaw_to_quat_wxyz(self._setpoint_yaw_deg)
        return pos, quat

    def step(self, base_command, manipulator_command) -> None:
        if not self.available:
            return
        forward = float(base_command.forward)
        strafe = float(base_command.strafe)
        lift = float(base_command.lift)
        yaw_axis = float(base_command.yaw)
        if forward == 0.0 and strafe == 0.0 and lift == 0.0 and yaw_axis == 0.0:
            return
        if self.playing:
            self.playing = False
            self._setpoint_vel = np.zeros(3, dtype=np.float32)
            self._setpoint_yaw_rate = 0.0
        yaw_rad = np.deg2rad(self._setpoint_yaw_deg)
        dx = forward * np.cos(yaw_rad) - strafe * np.sin(yaw_rad)
        dy = forward * np.sin(yaw_rad) + strafe * np.cos(yaw_rad)
        self._setpoint_pos = self._setpoint_pos + np.array([dx, dy, lift], dtype=np.float32) * (
            self.manual_speed * self.manual_dt
        )
        self._setpoint_yaw_deg += yaw_axis * self.manual_yaw_rate_deg * self.manual_dt


def resolve_quadrotor_trajectory_paths(count: int, trajectory_dir: str) -> list[str]:
    """Resolve one trajectory file per quadrotor from a numbered directory."""
    if count < 1:
        raise ValueError("Quadrotor count must be at least 1.")
    directory = Path(trajectory_dir).expanduser()
    if not directory.is_dir():
        raise FileNotFoundError(f"Quadrotor trajectory directory not found: {directory}")
    paths: list[str] = []
    for index in range(1, count + 1):
        candidates = (
            directory / f"{index}.csv",
            directory / f"{index}.json",
            directory / f"{index:02d}.csv",
            directory / f"{index:02d}.json",
        )
        for candidate in candidates:
            if candidate.exists():
                paths.append(str(candidate))
                break
        else:
            expected = ", ".join(candidate.name for candidate in candidates)
            raise FileNotFoundError(
                f"Missing quadrotor trajectory for index {index} in {directory}. Expected one of: {expected}"
            )
    return paths


def resolve_quadrotor_prim_paths(base_prim: str, count: int) -> list[str]:
    """Resolve USD prim paths for a quadrotor fleet."""
    base = base_prim.rstrip("/")
    if count == 1:
        return [base]
    return [f"{base}_{index}" for index in range(1, count + 1)]


class QuadrotorFleetController:
    """Manage multiple kinematic quadrotors as one teleop controller."""

    name = "quadrotor"

    def __init__(self, controllers: list[QuadrotorTrajectoryController]):
        self.controllers = [controller for controller in controllers if controller.available]
        self.available = bool(self.controllers)
        self.active_target_mode = "trajectory"
        self._active_index = 0

    @property
    def playing(self) -> bool:
        if not self.controllers:
            return False
        return bool(self.controllers[0].playing)

    @playing.setter
    def playing(self, value: bool) -> None:
        for controller in self.controllers:
            controller.playing = bool(value)

    @property
    def camera_path(self) -> str:
        return self.active_controller.camera_path

    @property
    def camera_resolution(self) -> tuple[int, int]:
        return self.active_controller.camera_resolution

    @property
    def active_controller(self) -> QuadrotorTrajectoryController:
        return self.controllers[self._active_index]

    def camera_aliases(self) -> dict[str, str]:
        aliases = {f"head_top_{index + 1}": controller.camera_path for index, controller in enumerate(self.controllers)}
        if len(self.controllers) == 1:
            aliases["head_top"] = self.controllers[0].camera_path
        return aliases

    def cycle_viewport_camera(self) -> str | None:
        if not self.controllers:
            return None
        self._active_index = (self._active_index + 1) % len(self.controllers)
        return f"head_top_{self._active_index + 1}"

    def update(self, dt: float) -> None:
        for controller in self.controllers:
            controller.update(dt)

    def reset_trajectory(self) -> None:
        for controller in self.controllers:
            controller.reset_trajectory()

    def toggle_playing(self) -> bool:
        if not self.controllers:
            return False
        new_state = not self.controllers[0].playing
        for controller in self.controllers:
            controller.playing = new_state
        return new_state

    def cycle_target_mode(self) -> str:
        return self.active_target_mode

    def get_base_world_pose(self):
        return self.active_controller.get_base_world_pose()

    def step(self, base_command, manipulator_command) -> None:
        self.active_controller.step(base_command, manipulator_command)


class QuadrotorMatplotlibCameraViewer:
    """External Matplotlib RGB/Depth viewer for the quadrotor camera."""

    def __init__(
        self,
        camera_path: str,
        carb,
        resolution: tuple[int, int] = (640, 480),
        update_period: float = 0.1,
        depth_max: float = 20.0,
    ):
        self.camera_path = camera_path
        self._carb = carb
        self.resolution = (int(resolution[0]), int(resolution[1]))
        self.update_period = max(float(update_period), 1e-3)
        self.depth_max = max(float(depth_max), 1e-3)
        self.enabled = False
        self._last_update = 0.0
        self._camera = None
        self._plt = None
        self._figure = None
        self._rgb_image = None
        self._depth_image = None
        self._empty_frame_warned = False
        self._has_valid_frame = False

    def initialize(self) -> bool:
        """Create the Isaac camera sensor and Matplotlib window."""
        try:
            os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
            import matplotlib  # noqa: PLC0415

            if matplotlib.get_backend().lower() == "agg":
                matplotlib.use("TkAgg", force=True)
            import matplotlib.pyplot as plt  # noqa: PLC0415
            from isaacsim.sensors.camera import Camera  # noqa: PLC0415
        except Exception as exc:  # noqa: BLE001
            self._carb.log_warn(
                f"Quadrotor Matplotlib viewer unavailable: {exc}. "
                "Install/enable a GUI Matplotlib backend such as TkAgg, or run without --headless."
            )
            return False
        try:
            self._camera = Camera(prim_path=self.camera_path, resolution=self.resolution)
            self._camera.initialize()
            self._camera.add_distance_to_image_plane_to_frame()
            self._plt = plt
            plt.ion()
            self._figure, axes = plt.subplots(1, 2, num="Quadrotor RGB-D Camera", figsize=(10, 4))
            axes[0].set_title("Quadrotor Camera RGB (waiting)")
            axes[1].set_title("Quadrotor Camera Depth (waiting)")
            for axis in axes:
                axis.set_axis_off()
            self._rgb_image = axes[0].imshow(np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8))
            self._depth_image = axes[1].imshow(
                np.zeros((self.resolution[1], self.resolution[0]), dtype=np.float32),
                cmap="viridis",
                vmin=0.0,
                vmax=self.depth_max,
            )
            self._figure.tight_layout()
            self._figure.show()
            plt.show(block=False)
            self.enabled = True
            self._carb.log_warn(f"Quadrotor Matplotlib RGB-D viewer ready for {self.camera_path}.")
            return True
        except Exception as exc:  # noqa: BLE001
            self._carb.log_warn(f"Failed to initialize quadrotor Matplotlib viewer: {exc}")
            self.enabled = False
            return False

    def _read_rgb_depth(self):
        """Read RGB and depth from the Isaac camera sensor, falling back to current_frame keys."""
        rgb = self._camera.get_rgb()
        depth = self._camera.get_depth()
        if rgb is not None and depth is not None:
            return rgb, depth
        try:
            frame = self._camera.get_current_frame()
        except Exception:
            return rgb, depth
        if rgb is None:
            rgb = frame.get("rgb", frame.get("rgba"))
        if depth is None:
            depth = frame.get("distance_to_image_plane", frame.get("depth"))
        return rgb, depth

    def update(self, now: float) -> None:
        """Refresh the external RGB-D window if a new frame is available."""
        if not self.enabled or self._camera is None or self._plt is None:
            return
        if now - self._last_update < self.update_period:
            return
        self._last_update = now
        try:
            rgb, depth = self._read_rgb_depth()
            updated = False
            if rgb is not None:
                rgb_array = np.asarray(rgb)
                if rgb_array.ndim != 3 or rgb_array.shape[2] < 3:
                    if not self._empty_frame_warned:
                        self._carb.log_warn(
                            f"Quadrotor RGB frame not ready yet; received shape {rgb_array.shape}."
                        )
                        self._empty_frame_warned = True
                    return
                self._empty_frame_warned = False
                rgb_array = rgb_array[:, :, :3]
                if rgb_array.dtype != np.uint8:
                    rgb_array = np.clip(rgb_array, 0, 255).astype(np.uint8)
                self._rgb_image.set_data(rgb_array)
                updated = True
            if depth is not None:
                depth_array = np.asarray(depth, dtype=np.float32)
                if depth_array.ndim != 2:
                    if not self._empty_frame_warned:
                        self._carb.log_warn(
                            f"Quadrotor depth frame not ready yet; received shape {depth_array.shape}."
                        )
                        self._empty_frame_warned = True
                    return
                self._empty_frame_warned = False
                depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=self.depth_max, neginf=0.0)
                self._depth_image.set_data(np.clip(depth_array, 0.0, self.depth_max))
                updated = True
            if not updated:
                if not self._empty_frame_warned:
                    self._carb.log_warn("Quadrotor camera frames are not ready yet; waiting for IsaacSim sensor output.")
                    self._empty_frame_warned = True
                return
            if not self._has_valid_frame:
                axes = self._figure.axes
                if len(axes) >= 2:
                    axes[0].set_title("Quadrotor Camera RGB")
                    axes[1].set_title("Quadrotor Camera Depth")
                self._carb.log_warn("Quadrotor Matplotlib viewer is displaying live camera frames.")
                self._has_valid_frame = True
            self._figure.canvas.draw_idle()
            self._figure.canvas.flush_events()
            self._plt.pause(0.001)
        except Exception as exc:  # noqa: BLE001
            self._carb.log_warn(f"Quadrotor Matplotlib viewer update failed: {exc}")
            self.enabled = False


class QuadrotorMatplotlibFleetCameraViewer:
    """External Matplotlib RGB/Depth viewer for multiple quadrotor cameras."""

    def __init__(
        self,
        camera_specs: list[tuple[str, str]],
        carb,
        resolution: tuple[int, int] = (320, 240),
        update_period: float = 0.1,
        depth_max: float = 20.0,
    ):
        self.camera_specs = list(camera_specs)
        self._carb = carb
        self.resolution = (int(resolution[0]), int(resolution[1]))
        self.update_period = max(float(update_period), 1e-3)
        self.depth_max = max(float(depth_max), 1e-3)
        self.enabled = False
        self._last_update = 0.0
        self._cameras = []
        self._plt = None
        self._figure = None
        self._rgb_images = []
        self._depth_images = []
        self._empty_frame_warned = False
        self._has_valid_frame = False

    def initialize(self) -> bool:
        """Create Isaac camera sensors and one Matplotlib window for the fleet."""
        if not self.camera_specs:
            return False
        try:
            os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
            import matplotlib  # noqa: PLC0415

            if matplotlib.get_backend().lower() == "agg":
                matplotlib.use("TkAgg", force=True)
            import matplotlib.pyplot as plt  # noqa: PLC0415
            from isaacsim.sensors.camera import Camera  # noqa: PLC0415
        except Exception as exc:  # noqa: BLE001
            self._carb.log_warn(
                f"Quadrotor fleet Matplotlib viewer unavailable: {exc}. "
                "Install/enable a GUI Matplotlib backend such as TkAgg, or run without --headless."
            )
            return False
        try:
            self._cameras = []
            for camera_path, _label in self.camera_specs:
                camera = Camera(prim_path=camera_path, resolution=self.resolution)
                camera.initialize()
                camera.add_distance_to_image_plane_to_frame()
                self._cameras.append(camera)
            self._plt = plt
            plt.ion()
            row_count = len(self.camera_specs)
            self._figure, axes = plt.subplots(
                row_count,
                2,
                num="Quadrotor Fleet RGB-D",
                figsize=(10, max(2.5 * row_count, 4.0)),
            )
            if row_count == 1:
                axes = np.array([axes])
            self._rgb_images = []
            self._depth_images = []
            for row_index, (_camera_path, label) in enumerate(self.camera_specs):
                rgb_axis = axes[row_index, 0]
                depth_axis = axes[row_index, 1]
                rgb_axis.set_title(f"{label} RGB (waiting)")
                depth_axis.set_title(f"{label} Depth (waiting)")
                rgb_axis.set_axis_off()
                depth_axis.set_axis_off()
                self._rgb_images.append(
                    rgb_axis.imshow(np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8))
                )
                self._depth_images.append(
                    depth_axis.imshow(
                        np.zeros((self.resolution[1], self.resolution[0]), dtype=np.float32),
                        cmap="viridis",
                        vmin=0.0,
                        vmax=self.depth_max,
                    )
                )
            self._figure.tight_layout()
            self._figure.show()
            plt.show(block=False)
            self.enabled = True
            self._carb.log_warn(
                f"Quadrotor fleet Matplotlib RGB-D viewer ready for {len(self.camera_specs)} cameras."
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self._carb.log_warn(f"Failed to initialize quadrotor fleet Matplotlib viewer: {exc}")
            self.enabled = False
            return False

    def _read_rgb_depth(self, camera):
        rgb = camera.get_rgb()
        depth = camera.get_depth()
        if rgb is not None and depth is not None:
            return rgb, depth
        try:
            frame = camera.get_current_frame()
        except Exception:
            return rgb, depth
        if rgb is None:
            rgb = frame.get("rgb", frame.get("rgba"))
        if depth is None:
            depth = frame.get("distance_to_image_plane", frame.get("depth"))
        return rgb, depth

    def update(self, now: float) -> None:
        """Refresh all fleet camera tiles when new frames are available."""
        if not self.enabled or not self._cameras or self._plt is None:
            return
        if now - self._last_update < self.update_period:
            return
        self._last_update = now
        try:
            updated_any = False
            for row_index, camera in enumerate(self._cameras):
                rgb, depth = self._read_rgb_depth(camera)
                label = self.camera_specs[row_index][1]
                if rgb is not None:
                    rgb_array = np.asarray(rgb)
                    if rgb_array.ndim == 3 and rgb_array.shape[2] >= 3:
                        rgb_array = rgb_array[:, :, :3]
                        if rgb_array.dtype != np.uint8:
                            rgb_array = np.clip(rgb_array, 0, 255).astype(np.uint8)
                        self._rgb_images[row_index].set_data(rgb_array)
                        updated_any = True
                if depth is not None:
                    depth_array = np.asarray(depth, dtype=np.float32)
                    if depth_array.ndim == 2:
                        depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=self.depth_max, neginf=0.0)
                        self._depth_images[row_index].set_data(np.clip(depth_array, 0.0, self.depth_max))
                        updated_any = True
            if not updated_any:
                if not self._empty_frame_warned:
                    self._carb.log_warn("Quadrotor fleet camera frames are not ready yet; waiting for sensor output.")
                    self._empty_frame_warned = True
                return
            self._empty_frame_warned = False
            if not self._has_valid_frame:
                for row_index, (_camera_path, label) in enumerate(self.camera_specs):
                    axes = self._figure.axes
                    rgb_axis = axes[row_index * 2]
                    depth_axis = axes[row_index * 2 + 1]
                    rgb_axis.set_title(f"{label} RGB")
                    depth_axis.set_title(f"{label} Depth")
                self._carb.log_warn("Quadrotor fleet Matplotlib viewer is displaying live camera frames.")
                self._has_valid_frame = True
            self._figure.canvas.draw_idle()
            self._figure.canvas.flush_events()
            self._plt.pause(0.001)
        except Exception as exc:  # noqa: BLE001
            self._carb.log_warn(f"Quadrotor fleet Matplotlib viewer update failed: {exc}")
            self.enabled = False
