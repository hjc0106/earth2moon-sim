"""线程安全的遥操作 HTTP 接口。

HTTP 工作线程只读取仿真主线程发布的 JSON 快照，绝不直接访问 Isaac Sim
对象，从而避免 Kit/PhysX 跨线程调用。
"""

from __future__ import annotations

import copy
import json
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


class TeleopStateApiServer:
    """在 ``/api/v1`` 下提供机器人、相机和性能状态接口。

    ``publish`` 只能由仿真线程调用。POST 请求仅将命令加入队列，仿真线程通过
    ``pop_command`` 取出并执行，避免 HTTP 工作线程直接操作 viewport。
    """

    API_PREFIX = "/api/v1"

    def __init__(self, host: str, port: int):
        self._host = host
        self._port = int(port)
        self._lock = threading.Lock()
        self._snapshot: dict[str, Any] = {
            "robots": {}, "cameras": {}, "relative_poses": {}, "telemetry": {}, "updated_at": None
        }
        self._images: dict[tuple[str, str], dict[str, Any]] = {}
        self._commands: deque[dict[str, str]] = deque()
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    @property
    def address(self) -> tuple[str, int]:
        return self._host, self._port

    def start(self) -> None:
        if self._server is not None:
            return
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format, *_args):  # noqa: N802
                return

            def _reply(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
                body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _image_reply(self, image: dict[str, Any]) -> None:
                body = image["data"]
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", image["content_type"])
                self.send_header("Content-Length", str(len(body)))
                self.send_header("X-Frame-Timestamp", str(image["updated_at"]))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):  # noqa: N802
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                snapshot = owner.snapshot()
                if path == owner.API_PREFIX:
                    self._reply(
                        HTTPStatus.OK,
                        {
                            "robots": list(snapshot["robots"]),
                            "endpoints": [
                                "/robots",
                                "/robots/{robot_name}/joints",
                                "/cameras",
                                "/cameras/{robot_name}/{camera_alias}/image",
                                "/relative-poses/{reference_robot}/{target_robot}",
                                "/telemetry",
                            ],
                        },
                    )
                    return
                if path == f"{owner.API_PREFIX}/robots":
                    self._reply(HTTPStatus.OK, {"robots": snapshot["robots"], "updated_at": snapshot["updated_at"]})
                    return
                if path == f"{owner.API_PREFIX}/cameras":
                    self._reply(HTTPStatus.OK, {"cameras": snapshot["cameras"], "updated_at": snapshot["updated_at"]})
                    return
                if path == f"{owner.API_PREFIX}/telemetry":
                    self._reply(HTTPStatus.OK, {"telemetry": snapshot["telemetry"], "updated_at": snapshot["updated_at"]})
                    return
                if path.startswith(f"{owner.API_PREFIX}/cameras/") and path.endswith("/image"):
                    parts = path[len(f"{owner.API_PREFIX}/cameras/") : -len("/image")].strip("/").split("/")
                    if len(parts) != 2:
                        self._reply(HTTPStatus.NOT_FOUND, {"error": "Not found"})
                        return
                    image = owner.image(parts[0], parts[1])
                    if image is None:
                        self._reply(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "Camera image is not ready"})
                    else:
                        self._image_reply(image)
                    return
                if path.startswith(f"{owner.API_PREFIX}/relative-poses/"):
                    parts = path[len(f"{owner.API_PREFIX}/relative-poses/") :].strip("/").split("/")
                    if len(parts) != 2:
                        self._reply(HTTPStatus.NOT_FOUND, {"error": "Not found"})
                        return
                    pose = snapshot["relative_poses"].get(f"{parts[0]}/{parts[1]}")
                    if pose is None:
                        self._reply(HTTPStatus.NOT_FOUND, {"error": "Relative pose is unavailable"})
                    else:
                        self._reply(HTTPStatus.OK, {**pose, "updated_at": snapshot["updated_at"]})
                    return
                if path.startswith(f"{owner.API_PREFIX}/robots/") and path.endswith("/joints"):
                    name = path[len(f"{owner.API_PREFIX}/robots/") : -len("/joints")].strip("/")
                    robot = snapshot["robots"].get(name)
                    if robot is None:
                        self._reply(HTTPStatus.NOT_FOUND, {"error": f"Unknown robot: {name}"})
                    else:
                        self._reply(
                            HTTPStatus.OK,
                            {
                                "name": name,
                                "joints": robot.get("joints", {}),
                                "joint_units": robot.get("joint_units", {}),
                                "updated_at": snapshot["updated_at"],
                            },
                        )
                    return
                self._reply(HTTPStatus.NOT_FOUND, {"error": "Not found"})

            def do_POST(self):  # noqa: N802
                path = self.path.split("?", 1)[0].rstrip("/")
                if path != f"{owner.API_PREFIX}/active-camera":
                    self._reply(HTTPStatus.NOT_FOUND, {"error": "Not found"})
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    payload = json.loads(self.rfile.read(length).decode("utf-8"))
                    robot = str(payload["robot"])
                    camera = str(payload["camera"])
                except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                    self._reply(HTTPStatus.BAD_REQUEST, {"error": "Expected JSON: {robot, camera}"})
                    return
                if camera not in owner.snapshot().get("cameras", {}).get(robot, {}):
                    self._reply(HTTPStatus.NOT_FOUND, {"error": "Unknown robot or camera alias"})
                    return
                with owner._lock:
                    owner._commands.append({"type": "set_camera", "robot": robot, "camera": camera})
                self._reply(HTTPStatus.ACCEPTED, {"robot": robot, "camera": camera})

        self._server = ThreadingHTTPServer((self._host, self._port), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, name="teleop-state-api", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._server is None:
            return
        self._server.shutdown()
        self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._server = None
        self._thread = None

    def publish(
        self,
        robots: dict[str, dict[str, Any]],
        cameras: dict[str, dict[str, str]],
        relative_poses: dict[str, dict[str, Any]] | None = None,
        telemetry: dict[str, Any] | None = None,
    ) -> None:
        """原子发布可 JSON 序列化的仿真状态。"""
        with self._lock:
            self._snapshot = {
                "robots": copy.deepcopy(robots),
                "cameras": copy.deepcopy(cameras),
                "relative_poses": copy.deepcopy(relative_poses or {}),
                "telemetry": copy.deepcopy(telemetry or {}),
                "updated_at": time.time(),
            }

    def publish_image(self, robot: str, camera: str, data: bytes, content_type: str = "image/jpeg") -> None:
        """发布一帧由仿真线程编码完成的相机图像。"""
        with self._lock:
            self._images[(robot, camera)] = {
                "data": bytes(data),
                "content_type": content_type,
                "updated_at": time.time(),
            }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._snapshot)

    def image(self, robot: str, camera: str) -> dict[str, Any] | None:
        with self._lock:
            image = self._images.get((robot, camera))
            return copy.deepcopy(image) if image is not None else None

    def pop_command(self) -> dict[str, str] | None:
        with self._lock:
            return self._commands.popleft() if self._commands else None
