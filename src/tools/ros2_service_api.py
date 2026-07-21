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
import math
import os
import threading
from typing import Any, Dict, Optional

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
            api_key=os.environ.get("KN_NAV_API_KEY", ""),
        )


class QuaternionPose(BaseModel):
    """A map-frame pose represented by position and quaternion."""

    x: float = Field(..., description="Map-frame X position in metres")
    y: float = Field(..., description="Map-frame Y position in metres")
    z: float = Field(0.0, description="Map-frame Z position in metres")
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


class NavigationGoal(BaseModel):
    """Convenient planar navigation target for HTTP clients."""

    x: float = Field(..., description="Map-frame X position in metres")
    y: float = Field(..., description="Map-frame Y position in metres")
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


def _pose_to_dict(pose: Any) -> Dict[str, Any]:
    orientation = pose.orientation
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return {
        "position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
        },
        "orientation": {
            "qx": orientation.x,
            "qy": orientation.y,
            "qz": orientation.z,
            "qw": orientation.w,
        },
        "yaw": math.atan2(siny_cosp, cosy_cosp),
    }


class RosServiceGateway:
    """Own a ROS node and spin it on a background thread."""

    SERVICE_NAMES = {
        "relocalize": "/open3d_loc/relocalize",
        "get_pose": "/open3d_loc/get_pose",
        "publish_goal": "/open3d_loc/publish_goal",
        "pose_deviation": "/open3d_loc/pose_deviation",
        "map_save": "/map_save",
        "bridge_enable": "/go2_cmd_vel_bridge/enable",
    }

    def __init__(self, settings: Settings):
        try:
            import rclpy
            from open3d_loc.srv import GetPose, PoseDeviation, PublishGoal, Relocalize
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from std_srvs.srv import SetBool, Trigger
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
            "pose_deviation": PoseDeviation,
            "map_save": Trigger,
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
        gateway.start()
        application.state.ros_gateway = gateway
        try:
            yield
        finally:
            gateway.stop()

    application = FastAPI(
        title="KN Navigation ROS 2 Service API",
        version="1.0.0",
        description=(
            "HTTP facade for the navigation services documented in src/service.md. "
            "All poses use the map coordinate frame and yaw values use radians."
        ),
        lifespan=lifespan,
    )

    def gateway() -> RosServiceGateway:
        return application.state.ros_gateway

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

    async def call_publish_goal(
        pose: QuaternionPose, ros: RosServiceGateway
    ) -> Dict[str, Any]:
        request = ros.request("publish_goal")
        _fill_pose_request(request, pose)
        response = await ros.call("publish_goal", request)
        return {"success": response.success, "message": response.message}

    @router.post(
        "/navigation/goal",
        tags=["navigation"],
        summary="Send a map-frame navigation goal using XYZ and yaw",
    )
    async def navigation_goal(
        goal: NavigationGoal, ros: RosServiceGateway = Depends(gateway)
    ) -> Dict[str, Any]:
        _ensure_finite({"x": goal.x, "y": goal.y, "z": goal.z, "yaw": goal.yaw})
        half_yaw = 0.5 * goal.yaw
        pose = QuaternionPose(
            x=goal.x,
            y=goal.y,
            z=goal.z,
            qx=0.0,
            qy=0.0,
            qz=math.sin(half_yaw),
            qw=math.cos(half_yaw),
        )
        return await call_publish_goal(pose, ros)

    @router.post(
        "/open3d_loc/publish_goal",
        tags=["navigation"],
        summary="Send a navigation goal using a quaternion pose",
    )
    async def publish_goal(
        pose: QuaternionPose, ros: RosServiceGateway = Depends(gateway)
    ) -> Dict[str, Any]:
        return await call_publish_goal(pose, ros)

    @router.post(
        "/open3d_loc/relocalize",
        tags=["localization"],
        summary="Trigger Open3D relocalization from an initial pose",
    )
    async def relocalize(
        pose: QuaternionPose, ros: RosServiceGateway = Depends(gateway)
    ) -> Dict[str, Any]:
        request = ros.request("relocalize")
        _fill_pose_request(request, pose)
        response = await ros.call("relocalize", request)
        return {"success": response.success, "message": response.message}

    @router.get(
        "/open3d_loc/get_pose",
        tags=["localization"],
        summary="Get the latest Open3D localization pose",
    )
    async def get_pose(
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        response = await ros.call("get_pose", ros.request("get_pose"))
        result: Dict[str, Any] = {
            "success": response.success,
            "message": response.message,
        }
        if response.success:
            siny_cosp = 2.0 * (response.qw * response.qz + response.qx * response.qy)
            cosy_cosp = 1.0 - 2.0 * (
                response.qy * response.qy + response.qz * response.qz
            )
            result["pose"] = {
                "x": response.x,
                "y": response.y,
                "z": response.z,
                "qx": response.qx,
                "qy": response.qy,
                "qz": response.qz,
                "qw": response.qw,
                "yaw": math.atan2(siny_cosp, cosy_cosp),
            }
        return result

    @router.post(
        "/open3d_loc/pose_deviation",
        tags=["localization"],
        summary="Compare the current localization pose with a reference pose",
    )
    async def pose_deviation(
        pose: QuaternionPose, ros: RosServiceGateway = Depends(gateway)
    ) -> Dict[str, Any]:
        request = ros.request("pose_deviation")
        _fill_pose_request(request, pose)
        response = await ros.call("pose_deviation", request)
        result: Dict[str, Any] = {
            "success": response.success,
            "message": response.message,
        }
        if response.success:
            result.update(
                {
                    "current_pose": _pose_to_dict(response.current_pose),
                    "error_x": response.error_x,
                    "error_y": response.error_y,
                    "distance_xy": response.distance_xy,
                    "yaw_error_rad": response.yaw_error_rad,
                    "yaw_error_deg": response.yaw_error_deg,
                }
            )
        return result

    @router.post(
        "/map/save", tags=["map"], summary="Request FAST-LIO to save its map"
    )
    async def save_map(
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        response = await ros.call("map_save", ros.request("map_save"))
        return {"success": response.success, "message": response.message}

    @router.post(
        "/go2_cmd_vel_bridge/enable",
        tags=["robot"],
        summary="Arm or disarm the Unitree velocity bridge",
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
    parser.add_argument("--log-level", default="info")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("--port must be between 1 and 65535")
    if args.service_timeout <= 0.0 or args.service_wait_timeout <= 0.0:
        raise SystemExit("service timeouts must be greater than zero")

    environment = Settings.from_environment()
    settings = Settings(
        host=args.host,
        port=args.port,
        service_timeout=args.service_timeout,
        service_wait_timeout=args.service_wait_timeout,
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
