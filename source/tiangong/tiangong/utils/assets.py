"""项目本地资产路径解析工具。

统一管理天宫场景、Ranger Arm、R1 Pro 以及 IsaacSim 内置资产的默认路径。
支持通过环境变量覆盖资产根目录，便于在不同机器上复用同一套脚本。
"""

from __future__ import annotations

import os
from pathlib import Path


def _default_repo_root() -> Path:
    """根据当前文件位置推导仓库根目录。"""
    # source/tiangong/tiangong/utils/assets.py -> outer repo root is parents[5]
    return Path(__file__).resolve().parents[5]


TIANGONG_REPO_ROOT = Path(os.environ.get("TIANGONG_REPO_ROOT", _default_repo_root())).resolve()
"""仓库根目录绝对路径。"""

TIANGONG_WORKSPACE_ROOT = TIANGONG_REPO_ROOT.parent
"""共享 workspace 根目录绝对路径。"""

TIANGONG_PROJECT_ASSETS_ROOT = Path(
    os.environ.get("TIANGONG_PROJECT_ASSETS_ROOT", TIANGONG_REPO_ROOT / "assets")
).resolve()
"""本项目内置 assets 目录绝对路径。"""

TIANGONG_ASSET_ROOT = Path(
    os.environ.get("TIANGONG_ASSET_ROOT", TIANGONG_PROJECT_ASSETS_ROOT)
).resolve()
"""优先使用的本地资产根目录绝对路径。"""


def local_asset_path(*parts: str) -> str:
    """返回配置资产根目录下的绝对路径。"""
    return str(TIANGONG_ASSET_ROOT.joinpath(*parts).resolve())


def tkmodel_usd_path() -> str:
    """返回默认天宫主场景 USD 路径。"""
    return str((TIANGONG_PROJECT_ASSETS_ROOT / "tiangong_scene" / "tkmodel.usd").resolve())


TIANGONG_SPACE_STATION_ASSET_PATH = str(
    (TIANGONG_PROJECT_ASSETS_ROOT / "tiangong_scene" / "Tiangong Space Station.usd").resolve()
)
CF2X_ASSET_PATH = local_asset_path("Assets", "Isaac", "5.1", "Isaac", "Robots", "Bitcraze", "Crazyflie", "cf2x.usd")
RANGER_ARM_ASSET_PATH = local_asset_path(
    "Assets", "Isaac", "5.1", "Isaac", "Robots", "Clearpath", "RidgebackFranka", "ridgeback_franka.usd"
)
RANGER_ARM_CONFIG_ASSET_PATH = str(
    (TIANGONG_PROJECT_ASSETS_ROOT / "tiangong_scene" / "configuration" / "tkmodel_physics.usd").resolve()
)

ASSET_PRIM_PATHS = {
    "tiangong_space_station": {
        "asset_path": TIANGONG_SPACE_STATION_ASSET_PATH,
        "prim_path": "/World/Tiangong_Space_Station",
    },
    "cf2x": {
        "asset_path": CF2X_ASSET_PATH,
        "prim_path": "/World/cf2x",
    },
    "cf2x_01": {
        "asset_path": CF2X_ASSET_PATH,
        "prim_path": "/World/cf2x_01",
    },
    "ranger_arm": {
        "asset_path": RANGER_ARM_CONFIG_ASSET_PATH,
        "prim_path": "/World/ranger_arm",
        "asset_prim_path": "/ranger_arm",
    },
}
"""天宫场景使用的本地 USD 路径与 stage prim 路径映射。"""
