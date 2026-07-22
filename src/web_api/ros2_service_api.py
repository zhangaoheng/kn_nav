#!/usr/bin/env python3
"""Expose the KN navigation ROS 2 services through a FastAPI HTTP API.

Run this script from a shell where ROS 2 and this workspace have been sourced::

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/tools/ros2_service_api.py --host 127.0.0.1 --port 8000

FastAPI's interactive API documentation is then available at ``/docs``.
Set ``KN_NAV_API_KEY`` to require the same value in the ``X-API-Key`` HTTP
header.  Authentication is strongly recommended when binding to a non-loopback
address.
"""

import argparse
import asyncio
from contextlib import asynccontextmanager
from dataclasses import dataclass
import hmac
import json
import math
import os
from pathlib import Path
import threading
from typing import Any, Dict, Optional, Tuple

try:
    import uvicorn
    from fastapi import APIRouter, Depends, FastAPI, Header, HTTPException
    from pydantic import BaseModel, Field
except ImportError as error:  # pragma: no cover - depends on the deployment host
    raise SystemExit(
        "FastAPI dependencies are missing. Install them with: "
        "python3 -m pip install fastapi uvicorn"
    ) from error


@dataclass(frozen=True)
class Settings:
    host: str = "127.0.0.1"
    port: int = 8000
    service_timeout: float = 30.0
    service_wait_timeout: float = 1.0
    points_file: str = str(Path(__file__).with_name("cache.json"))
    api_key: str = ""

    @classmethod
    def from_environment(cls) -> "Settings":
        return cls(
            host=os.environ.get("KN_NAV_API_HOST", "127.0.0.1"),
            port=int(os.environ.get("KN_NAV_API_PORT", "8000")),
            service_timeout=float(os.environ.get("KN_NAV_SERVICE_TIMEOUT", "30.0")),
            service_wait_timeout=float(
                os.environ.get("KN_NAV_SERVICE_WAIT_TIMEOUT", "1.0")
            ),
            points_file=os.environ.get(
                "KN_NAV_POINTS_FILE",
                str(Path(__file__).with_name("cache.json")),
            ),
            api_key=os.environ.get("KN_NAV_API_KEY", ""),
        )


class QuaternionPose(BaseModel):
    """A map-frame pose represented by position and quaternion."""

    x: float = Field(..., description="Map-frame X position in metres")
    y: float = Field(..., description="Map-frame Y position in metres")
    z: float = Field(..., description="Map-frame Z position in metres")
    qx: float = Field(..., description="Quaternion X")
    qy: float = Field(..., description="Quaternion Y")
    qz: float = Field(..., description="Quaternion Z")
    qw: float = Field(..., description="Quaternion W")


class SavePointRequest(BaseModel):
    name: str = Field(..., description="Unique navigation point name")
    overwrite: bool = Field(False, description="Replace an existing point")


class NavigationGoalRequest(BaseModel):
    """A saved point name, or a direct planar map-frame goal."""

    name: Optional[str] = Field(None, description="Saved navigation point name")
    x: Optional[float] = Field(None, description="Map-frame X position in metres")
    y: Optional[float] = Field(None, description="Map-frame Y position in metres")
    z: float = Field(0.0, description="Map-frame Z position in metres")
    yaw: float = Field(0.0, description="Target yaw in radians")


class BridgeEnableRequest(BaseModel):
    data: bool = Field(..., description="True to arm, false to disarm the bridge")


def _ensure_finite(values: Dict[str, float]) -> None:
    invalid = [name for name, value in values.items() if not math.isfinite(value)]
    if invalid:
        raise HTTPException(
            status_code=422,
            detail=f"Non-finite numeric fields are not allowed: {', '.join(invalid)}",
        )


def _fill_pose_request(request: Any, pose: QuaternionPose) -> None:
    values = {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "qx": pose.qx,
        "qy": pose.qy,
        "qz": pose.qz,
        "qw": pose.qw,
    }
    _ensure_finite(values)
    for name, value in values.items():
        setattr(request, name, float(value))


def _normalise_quaternion_pose(
    pose: QuaternionPose,
) -> Tuple[QuaternionPose, float]:
    values = {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "qx": pose.qx,
        "qy": pose.qy,
        "qz": pose.qz,
        "qw": pose.qw,
    }
    _ensure_finite(values)
    norm = math.sqrt(
        pose.qx * pose.qx
        + pose.qy * pose.qy
        + pose.qz * pose.qz
        + pose.qw * pose.qw
    )
    if norm < 1e-6:
        raise HTTPException(status_code=422, detail="Quaternion norm is too small")
    return (
        QuaternionPose(
            x=pose.x,
            y=pose.y,
            z=pose.z,
            qx=pose.qx / norm,
            qy=pose.qy / norm,
            qz=pose.qz / norm,
            qw=pose.qw / norm,
        ),
        norm,
    )


def _quaternion_pose_to_dict(pose: QuaternionPose) -> Dict[str, float]:
    return {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "qx": pose.qx,
        "qy": pose.qy,
        "qz": pose.qz,
        "qw": pose.qw,
    }


def _quaternion_to_rpy(
    qx: float, qy: float, qz: float, qw: float
) -> Dict[str, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return {
        "roll": roll,
        "pitch": pitch,
        "yaw": math.atan2(siny_cosp, cosy_cosp),
    }


def _normalise_point_name(name: str) -> str:
    normalised = name.strip()
    if not normalised:
        raise HTTPException(status_code=422, detail="Point name must not be empty")
    if len(normalised) > 128:
        raise HTTPException(
            status_code=422, detail="Point name must not exceed 128 characters"
        )
    if any(ord(character) < 32 for character in normalised):
        raise HTTPException(
            status_code=422, detail="Point name must not contain control characters"
        )
    return normalised


class NavigationPointStore:
    """Thread-safe JSON storage for named navigation points."""

    def __init__(self, file_path: str):
        self.path = Path(file_path).expanduser().resolve()
        self._lock = threading.Lock()
        self._initialise_file()

    def _initialise_file(self) -> None:
        """Create a missing store and repair an empty store as an empty object."""
        with self._lock:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                try:
                    with self.path.open("x", encoding="utf-8") as handle:
                        handle.write("{}\n")
                    return
                except FileExistsError:
                    pass

                with self.path.open("r", encoding="utf-8") as handle:
                    content = handle.read()
                if not content.strip():
                    with self.path.open("w", encoding="utf-8") as handle:
                        handle.write("{}\n")
            except OSError as error:
                raise RuntimeError(
                    f"Unable to initialise navigation points file {self.path}: {error}"
                ) from error

    def _load_unlocked(self) -> Dict[str, Dict[str, Any]]:
        if not self.path.exists():
            return {}
        try:
            with self.path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError) as error:
            raise HTTPException(
                status_code=500,
                detail=f"Unable to read navigation points file: {error}",
            ) from error
        if not isinstance(data, dict):
            raise HTTPException(
                status_code=500,
                detail="Navigation points file must contain a JSON object",
            )
        invalid = [name for name, point in data.items() if not isinstance(point, dict)]
        if invalid:
            raise HTTPException(
                status_code=500,
                detail=(
                    "Navigation points must be JSON objects; invalid entries: "
                    + ", ".join(str(name) for name in invalid)
                ),
            )
        return data

    def list_points(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            points = self._load_unlocked()
            return {name: dict(points[name]) for name in sorted(points)}

    def get_point(self, name: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            point = self._load_unlocked().get(name)
            return dict(point) if isinstance(point, dict) else None

    def save_point(
        self, name: str, point: Dict[str, Any], overwrite: bool
    ) -> None:
        with self._lock:
            points = self._load_unlocked()
            if name in points and not overwrite:
                raise HTTPException(
                    status_code=409,
                    detail=f"Navigation point already exists: {name}",
                )
            points[name] = point
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.path.with_suffix(self.path.suffix + ".tmp")
                with temporary.open("w", encoding="utf-8") as handle:
                    json.dump(points, handle, indent=2, sort_keys=True)
                    handle.write("\n")
                temporary.replace(self.path)
            except OSError as error:
                raise HTTPException(
                    status_code=500,
                    detail=f"Unable to write navigation points file: {error}",
                ) from error


class RosServiceGateway:
    """Own a ROS node and spin it on a background thread."""

    SERVICE_NAMES = {
        "relocalize": "/open3d_loc/relocalize",
        "get_pose": "/open3d_loc/get_pose",
        "publish_goal": "/open3d_loc/publish_goal",
        "bridge_enable": "/go2_cmd_vel_bridge/enable",
    }

    def __init__(self, settings: Settings):
        try:
            import rclpy
            from open3d_loc.srv import GetPose, PublishGoal, Relocalize
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from std_srvs.srv import SetBool
        except ImportError as error:
            raise RuntimeError(
                "ROS 2 Python interfaces are unavailable. Source /opt/ros/humble/"
                "setup.bash and the workspace install/setup.bash before starting "
                "this API."
            ) from error

        self.settings = settings
        self._rclpy = rclpy
        self._context = Context()
        rclpy.init(args=None, context=self._context)
        self._node = Node(
            f"kn_nav_fastapi_gateway_{os.getpid()}", context=self._context
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name="kn-nav-ros2-executor",
            daemon=True,
        )

        self.service_types = {
            "relocalize": Relocalize,
            "get_pose": GetPose,
            "publish_goal": PublishGoal,
            "bridge_enable": SetBool,
        }
        self.clients = {
            key: self._node.create_client(self.service_types[key], service_name)
            for key, service_name in self.SERVICE_NAMES.items()
        }

    def start(self) -> None:
        self._spin_thread.start()
        self._node.get_logger().info("KN navigation FastAPI ROS gateway started")

    def stop(self) -> None:
        self._executor.shutdown(timeout_sec=2.0)
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        if self._context.ok():
            self._context.shutdown()

    def status(self) -> Dict[str, Any]:
        services = {
            self.SERVICE_NAMES[key]: client.service_is_ready()
            for key, client in self.clients.items()
        }
        return {
            "success": self._context.ok() and self._spin_thread.is_alive(),
            "node": self._node.get_name(),
            "services": services,
        }

    async def call(
        self, key: str, request: Any, timeout: Optional[float] = None
    ) -> Any:
        client = self.clients[key]
        service_name = self.SERVICE_NAMES[key]
        loop = asyncio.get_running_loop()

        ready_deadline = loop.time() + self.settings.service_wait_timeout
        while not client.service_is_ready():
            if loop.time() >= ready_deadline:
                raise HTTPException(
                    status_code=503,
                    detail=f"ROS 2 service is unavailable: {service_name}",
                )
            await asyncio.sleep(0.05)

        try:
            future = client.call_async(request)
        except Exception as error:
            raise HTTPException(
                status_code=502,
                detail=f"Failed to call ROS 2 service {service_name}: {error}",
            ) from error

        deadline = loop.time() + (
            self.settings.service_timeout if timeout is None else timeout
        )
        while not future.done():
            if loop.time() >= deadline:
                future.cancel()
                raise HTTPException(
                    status_code=504,
                    detail=f"Timed out waiting for ROS 2 service: {service_name}",
                )
            await asyncio.sleep(0.01)

        try:
            response = future.result()
        except Exception as error:
            raise HTTPException(
                status_code=502,
                detail=f"ROS 2 service call failed for {service_name}: {error}",
            ) from error
        if response is None:
            raise HTTPException(
                status_code=502,
                detail=f"ROS 2 service returned no response: {service_name}",
            )
        return response

    def request(self, key: str) -> Any:
        return self.service_types[key].Request()


def create_app(settings: Settings) -> FastAPI:
    @asynccontextmanager
    async def lifespan(application: FastAPI):
        gateway = RosServiceGateway(settings)
        application.state.point_store = NavigationPointStore(settings.points_file)
        application.state.relocalize_lock = asyncio.Lock()
        gateway.start()
        application.state.ros_gateway = gateway
        try:
            yield
        finally:
            gateway.stop()

    application = FastAPI(
        title="KN Navigation ROS 2 Service API",
        version="2.0.0",
        description=(
            "Five core APIs: relocalize, save a navigation point, publish a goal, "
            "enable the velocity bridge, and list saved navigation points. "
            "All poses use the map frame and yaw values use radians."
        ),
        lifespan=lifespan,
    )

    def gateway() -> RosServiceGateway:
        return application.state.ros_gateway

    def point_store() -> NavigationPointStore:
        return application.state.point_store

    def relocalize_lock() -> Any:
        return application.state.relocalize_lock

    def require_api_key(x_api_key: Optional[str] = Header(default=None)) -> None:
        if settings.api_key and (
            x_api_key is None
            or not hmac.compare_digest(x_api_key, settings.api_key)
        ):
            raise HTTPException(status_code=401, detail="Invalid or missing X-API-Key")

    router = APIRouter(prefix="/api", dependencies=[Depends(require_api_key)])

    @application.get("/", tags=["system"])
    async def root() -> Dict[str, Any]:
        return {
            "name": "KN Navigation ROS 2 Service API",
            "docs": "/docs",
            "health": "/api/health",
            "functions": [
                "relocalize",
                "save_navigation_point",
                "publish_navigation_goal",
                "enable_velocity_bridge",
                "list_navigation_points",
            ],
        }

    @router.get("/health", tags=["system"], summary="Check ROS gateway health")
    async def health(ros: RosServiceGateway = Depends(gateway)) -> Dict[str, Any]:
        return ros.status()

    @router.get(
        "/services", tags=["system"], summary="List mapped ROS services and readiness"
    )
    async def services(ros: RosServiceGateway = Depends(gateway)) -> Dict[str, Any]:
        status = ros.status()
        return {"success": status["success"], "services": status["services"]}

    @router.post(
        "/open3d_loc/relocalize",
        tags=["localization"],
        summary="1. Relocalize and return success or failure",
    )
    async def relocalize(
        pose: QuaternionPose,
        ros: RosServiceGateway = Depends(gateway),
        lock: Any = Depends(relocalize_lock),
    ) -> Dict[str, Any]:
        normalised_pose, original_norm = _normalise_quaternion_pose(pose)
        sent_pose = _quaternion_pose_to_dict(normalised_pose)
        if lock.locked():
            return {
                "success": False,
                "message": "another relocalize request is running",
                "pose": sent_pose,
            }

        request = ros.request("relocalize")
        _fill_pose_request(request, normalised_pose)
        async with lock:
            response = await ros.call("relocalize", request)
        return {
            "success": response.success,
            "message": response.message,
            "pose": sent_pose,
            "input_quaternion_norm": original_norm,
            "quaternion_normalized": not math.isclose(
                original_norm, 1.0, rel_tol=1e-9, abs_tol=1e-9
            ),
        }

    @router.post(
        "/navigation/points",
        tags=["navigation"],
        summary="2. Save the current localization pose as a named JSON point",
    )
    async def save_navigation_point(
        command: SavePointRequest,
        ros: RosServiceGateway = Depends(gateway),
        store: NavigationPointStore = Depends(point_store),
    ) -> Dict[str, Any]:
        name = _normalise_point_name(command.name)
        response = await ros.call("get_pose", ros.request("get_pose"))
        if not response.success:
            return {"success": False, "message": response.message}

        values = {
            "x": response.x,
            "y": response.y,
            "z": response.z,
            "qx": response.qx,
            "qy": response.qy,
            "qz": response.qz,
            "qw": response.qw,
        }
        _ensure_finite(values)
        rpy = _quaternion_to_rpy(
            response.qx, response.qy, response.qz, response.qw
        )
        point: Dict[str, Any] = {
            "frame_id": "map",
            "child_frame_id": "base_link",
            **values,
            **rpy,
        }
        store.save_point(name, point, command.overwrite)
        return {
            "success": True,
            "message": f"Navigation point saved: {name}",
            "point": {"name": name, **point},
        }

    @router.get(
        "/navigation/points",
        tags=["navigation"],
        summary="5. Return all saved navigation points",
    )
    async def list_navigation_points(
        store: NavigationPointStore = Depends(point_store),
    ) -> Dict[str, Any]:
        stored_points = store.list_points()
        points = []
        for name, point in stored_points.items():
            item = dict(point)
            item["name"] = name
            points.append(item)
        return {"success": True, "count": len(points), "points": points}

    @router.post(
        "/navigation/goal",
        tags=["navigation"],
        summary="3. Publish a saved point or direct XYZ/yaw navigation goal",
    )
    async def publish_navigation_goal(
        goal: NavigationGoalRequest,
        ros: RosServiceGateway = Depends(gateway),
        store: NavigationPointStore = Depends(point_store),
    ) -> Dict[str, Any]:
        point_name: Optional[str] = None
        source = "coordinates"
        if goal.name is not None:
            if goal.x is not None or goal.y is not None:
                raise HTTPException(
                    status_code=422,
                    detail="Provide either name or coordinates, not both",
                )
            point_name = _normalise_point_name(goal.name)
            point = store.get_point(point_name)
            if point is None:
                raise HTTPException(
                    status_code=404,
                    detail=f"Navigation point not found: {point_name}",
                )
            try:
                x = float(point["x"])
                y = float(point["y"])
                z = float(point.get("z", 0.0))
                yaw = float(point["yaw"])
            except (KeyError, TypeError, ValueError) as error:
                raise HTTPException(
                    status_code=500,
                    detail=f"Saved navigation point is invalid: {point_name}",
                ) from error
            source = "saved_point"
        else:
            if goal.x is None or goal.y is None:
                raise HTTPException(
                    status_code=422,
                    detail="x and y are required when name is not provided",
                )
            x, y, z, yaw = goal.x, goal.y, goal.z, goal.yaw

        _ensure_finite({"x": x, "y": y, "z": z, "yaw": yaw})
        half_yaw = 0.5 * yaw
        pose = QuaternionPose(
            x=x,
            y=y,
            z=z,
            qx=0.0,
            qy=0.0,
            qz=math.sin(half_yaw),
            qw=math.cos(half_yaw),
        )
        request = ros.request("publish_goal")
        _fill_pose_request(request, pose)
        response = await ros.call("publish_goal", request)
        goal_result: Dict[str, Any] = {
            "source": source,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
        }
        if point_name is not None:
            goal_result["name"] = point_name
        return {
            "success": response.success,
            "message": response.message,
            "goal": goal_result,
        }

    @router.post(
        "/go2_cmd_vel_bridge/enable",
        tags=["robot"],
        summary="4. Enable or disable robot motion through the velocity bridge",
    )
    async def bridge_enable(
        command: BridgeEnableRequest,
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        request = ros.request("bridge_enable")
        request.data = command.data
        response = await ros.call("bridge_enable", request)
        return {"success": response.success, "message": response.message}

    application.include_router(router)
    return application


def _parse_args() -> argparse.Namespace:
    defaults = Settings.from_environment()
    parser = argparse.ArgumentParser(
        description="Expose KN navigation ROS 2 services through FastAPI"
    )
    parser.add_argument("--host", default=defaults.host)
    parser.add_argument("--port", type=int, default=defaults.port)
    parser.add_argument(
        "--service-timeout", type=float, default=defaults.service_timeout
    )
    parser.add_argument(
        "--service-wait-timeout", type=float, default=defaults.service_wait_timeout
    )
    parser.add_argument(
        "--points-file",
        default=defaults.points_file,
        help="JSON file used to store named navigation points",
    )
    parser.add_argument("--log-level", default="info")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("--port must be between 1 and 65535")
    if args.service_timeout <= 0.0 or args.service_wait_timeout <= 0.0:
        raise SystemExit("service timeouts must be greater than zero")
    if not args.points_file.strip():
        raise SystemExit("--points-file must not be empty")

    environment = Settings.from_environment()
    settings = Settings(
        host=args.host,
        port=args.port,
        service_timeout=args.service_timeout,
        service_wait_timeout=args.service_wait_timeout,
        points_file=args.points_file,
        api_key=environment.api_key,
    )
    if settings.host not in ("127.0.0.1", "localhost", "::1") and not settings.api_key:
        print(
            "WARNING: API is exposed beyond loopback without KN_NAV_API_KEY set.",
            flush=True,
        )

    uvicorn.run(
        create_app(settings),
        host=settings.host,
        port=settings.port,
        log_level=args.log_level,
    )


# This supports `uvicorn src.tools.ros2_service_api:app` when the repository is
# imported as a Python namespace, while direct execution remains the usual path.
app = create_app(Settings.from_environment())


if __name__ == "__main__":
    main()
