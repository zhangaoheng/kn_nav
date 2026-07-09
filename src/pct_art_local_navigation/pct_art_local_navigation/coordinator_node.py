#!/usr/bin/env python3
"""Coordinate a PCT global path with rolling ART local plans."""

from copy import deepcopy
import math
from typing import Optional, Sequence

from art_planner_msgs.action import PlanToGoal
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from pct_art_local_navigation.coordinator_logic import (
    MapGeometry,
    Point2,
    closest_progress_index,
    distance,
    local_goal_z_slope_rejection_reason,
    point_inside_map,
    select_local_goal,
    select_local_goal_without_map,
    tangent_yaw,
    validate_local_path,
)
from pct_art_local_navigation.path_smoothing import (
    PathSmoothingConfig,
    smooth_local_path,
    validate_smoothed_path,
)


class CoordinatorState:
    WAIT_GLOBAL_PATH = 'WAIT_GLOBAL_PATH'
    WAIT_MAP = 'WAIT_MAP'
    PLANNING = 'PLANNING'
    TRACKING = 'TRACKING'
    BLOCKED = 'BLOCKED'
    GOAL_REACHED = 'GOAL_REACHED'


def quaternion_to_yaw(quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class PctArtCoordinator(Node):
    """Select local goals on a PCT path and gate ART paths for tracking."""

    def __init__(self):
        super().__init__('pct_art_coordinator')
        self._declare_parameters()
        self._read_parameters()

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._local_path_pub = self.create_publisher(Path, self.local_path_topic, latched_qos)
        self._local_goal_pub = self.create_publisher(
            PoseStamped, self.local_goal_topic, latched_qos
        )
        self._status_pub = self.create_publisher(String, self.status_topic, latched_qos)
        self._final_approach_pub = self.create_publisher(
            Bool, '/pct_art_local_navigation/final_approach', latched_qos
        )

        self.create_subscription(Path, self.global_path_topic, self._global_path_cb, 10)
        self.create_subscription(Path, self.art_path_topic, self._art_path_cb, 10)
        self.create_subscription(GridMap, self.map_topic, self._map_cb, 1)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._art_client = ActionClient(self, PlanToGoal, self.art_action_name)

        self._global_path_msg: Optional[Path] = None
        self._global_points = []
        self._global_zs = []
        self._progress_index: Optional[int] = None
        self._map_geometry: Optional[MapGeometry] = None
        self._map_receive_time = None
        self._robot_point: Optional[Point2] = None
        self._robot_z: Optional[float] = None
        self._robot_yaw = 0.0
        self._local_goal_msg: Optional[PoseStamped] = None
        self._local_goal_index: Optional[int] = None
        self._last_art_path_time = None
        self._last_local_path_msg: Optional[Path] = None
        self._last_local_path_publish_time = None
        self._motion_path_active = False
        self._final_approach_active: Optional[bool] = None

        self._goal_handle = None
        self._send_future = None
        self._cancel_future = None
        self._pending_goal: Optional[PoseStamped] = None
        self._last_goal_request_time = None

        self._state = ''
        self._state_reason = ''
        self._set_state(CoordinatorState.WAIT_GLOBAL_PATH, 'waiting for /pct_path')
        self._publish_final_approach(False)
        self.create_timer(1.0 / self.update_rate, self._update)
        self.create_timer(1.0 / self.local_path_republish_rate, self._republish_local_path)

        self.get_logger().info(
            'PCT→ART coordinator ready: '
            f'global={self.global_path_topic}, ART={self.art_path_topic}, '
            f'local={self.local_path_topic}'
        )

    def _declare_parameters(self):
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('global_path_topic', '/pct_path')
        self.declare_parameter('art_path_topic', '/art_planner/path')
        self.declare_parameter('map_topic', '/traversability_map')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('local_goal_topic', '/local_goal')
        self.declare_parameter(
            'status_topic', '/pct_art_local_navigation/status'
        )
        self.declare_parameter('art_action_name', '/art_planner/plan_to_goal')
        self.declare_parameter('require_map_for_goal_selection', True)
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('lookahead_distance', 3.5)
        self.declare_parameter('minimum_goal_distance', 0.8)
        self.declare_parameter('maximum_goal_distance', 4.0)
        self.declare_parameter('maximum_local_goal_z_delta', 0.8)
        self.declare_parameter('maximum_local_goal_slope', 0.35)
        self.declare_parameter('map_edge_margin', 0.8)
        self.declare_parameter('advance_distance', 1.5)
        self.declare_parameter('goal_update_distance', 0.8)
        self.declare_parameter('goal_update_min_period', 0.5)
        self.declare_parameter('empty_art_path_grace_period', 0.75)
        self.declare_parameter('goal_reached_distance', 0.35)
        self.declare_parameter('goal_yaw_tolerance', 0.175)
        self.declare_parameter('progress_backtrack_points', 10)
        self.declare_parameter('progress_forward_search_points', 200)
        self.declare_parameter('maximum_path_start_distance', 1.0)
        self.declare_parameter('maximum_path_goal_distance', 0.8)
        self.declare_parameter('final_maximum_path_goal_distance', 0.25)
        self.declare_parameter('map_timeout', 1.0)
        self.declare_parameter('localization_timeout', 0.5)
        self.declare_parameter('art_path_timeout', 2.0)
        self.declare_parameter('local_path_republish_rate', 10.0)
        self.declare_parameter('local_path_reuse_timeout', 0.8)
        self.declare_parameter('tf_timeout', 0.1)
        self.declare_parameter('enable_path_smoothing', True)
        self.declare_parameter('path_min_point_spacing', 0.10)
        self.declare_parameter('path_resample_spacing', 0.10)
        self.declare_parameter('path_collinear_angle_threshold', 0.08)
        self.declare_parameter('path_smoothing_iterations', 20)
        self.declare_parameter('path_smoothing_data_weight', 0.45)
        self.declare_parameter('path_smoothing_smooth_weight', 0.20)
        self.declare_parameter('path_max_deviation', 0.15)

    def _read_parameters(self):
        def get(name):
            return self.get_parameter(name).value

        self.global_frame = get('global_frame')
        self.robot_frame = get('robot_frame')
        self.global_path_topic = get('global_path_topic')
        self.art_path_topic = get('art_path_topic')
        self.map_topic = get('map_topic')
        self.local_path_topic = get('local_path_topic')
        self.local_goal_topic = get('local_goal_topic')
        self.status_topic = get('status_topic')
        self.art_action_name = get('art_action_name')
        self.require_map_for_goal_selection = bool(get('require_map_for_goal_selection'))
        self.update_rate = float(get('update_rate'))
        self.lookahead_distance = float(get('lookahead_distance'))
        self.minimum_goal_distance = float(get('minimum_goal_distance'))
        self.maximum_goal_distance = float(get('maximum_goal_distance'))
        self.maximum_local_goal_z_delta = float(get('maximum_local_goal_z_delta'))
        self.maximum_local_goal_slope = float(get('maximum_local_goal_slope'))
        self.map_edge_margin = float(get('map_edge_margin'))
        self.advance_distance = float(get('advance_distance'))
        self.goal_update_distance = float(get('goal_update_distance'))
        self.goal_update_min_period = float(get('goal_update_min_period'))
        self.empty_art_path_grace_period = float(get('empty_art_path_grace_period'))
        self.goal_reached_distance = float(get('goal_reached_distance'))
        self.goal_yaw_tolerance = float(get('goal_yaw_tolerance'))
        self.progress_backtrack_points = int(get('progress_backtrack_points'))
        self.progress_forward_search_points = int(get('progress_forward_search_points'))
        self.maximum_path_start_distance = float(get('maximum_path_start_distance'))
        self.maximum_path_goal_distance = float(get('maximum_path_goal_distance'))
        self.final_maximum_path_goal_distance = float(
            get('final_maximum_path_goal_distance')
        )
        self.map_timeout = float(get('map_timeout'))
        self.localization_timeout = float(get('localization_timeout'))
        self.art_path_timeout = float(get('art_path_timeout'))
        self.local_path_republish_rate = float(get('local_path_republish_rate'))
        self.local_path_reuse_timeout = float(get('local_path_reuse_timeout'))
        self.tf_timeout = float(get('tf_timeout'))
        self.enable_path_smoothing = bool(get('enable_path_smoothing'))
        self.path_min_point_spacing = float(get('path_min_point_spacing'))
        self.path_resample_spacing = float(get('path_resample_spacing'))
        self.path_collinear_angle_threshold = float(
            get('path_collinear_angle_threshold')
        )
        self.path_smoothing_iterations = int(get('path_smoothing_iterations'))
        self.path_smoothing_data_weight = float(get('path_smoothing_data_weight'))
        self.path_smoothing_smooth_weight = float(get('path_smoothing_smooth_weight'))
        self.path_max_deviation = float(get('path_max_deviation'))
        self.path_smoothing_config = PathSmoothingConfig(
            enabled=self.enable_path_smoothing,
            min_point_spacing=self.path_min_point_spacing,
            resample_spacing=self.path_resample_spacing,
            collinear_angle_threshold=self.path_collinear_angle_threshold,
            iterations=self.path_smoothing_iterations,
            data_weight=self.path_smoothing_data_weight,
            smooth_weight=self.path_smoothing_smooth_weight,
            max_deviation=self.path_max_deviation,
        )

        positive = {
            'update_rate': self.update_rate,
            'lookahead_distance': self.lookahead_distance,
            'minimum_goal_distance': self.minimum_goal_distance,
            'maximum_goal_distance': self.maximum_goal_distance,
            'map_edge_margin': self.map_edge_margin,
            'advance_distance': self.advance_distance,
            'goal_update_distance': self.goal_update_distance,
            'goal_update_min_period': self.goal_update_min_period,
            'empty_art_path_grace_period': self.empty_art_path_grace_period,
            'goal_reached_distance': self.goal_reached_distance,
            'goal_yaw_tolerance': self.goal_yaw_tolerance,
            'map_timeout': self.map_timeout,
            'localization_timeout': self.localization_timeout,
            'art_path_timeout': self.art_path_timeout,
            'local_path_republish_rate': self.local_path_republish_rate,
            'local_path_reuse_timeout': self.local_path_reuse_timeout,
            'tf_timeout': self.tf_timeout,
            'final_maximum_path_goal_distance': self.final_maximum_path_goal_distance,
            'path_min_point_spacing': self.path_min_point_spacing,
            'path_resample_spacing': self.path_resample_spacing,
            'path_max_deviation': self.path_max_deviation,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and greater than zero')
        if not (
            self.minimum_goal_distance
            <= self.lookahead_distance
            <= self.maximum_goal_distance
        ):
            raise ValueError('goal distances must satisfy minimum <= lookahead <= maximum')
        if (
            not math.isfinite(self.maximum_local_goal_z_delta)
            or self.maximum_local_goal_z_delta < 0.0
        ):
            raise ValueError('maximum_local_goal_z_delta must be finite and non-negative')
        if (
            not math.isfinite(self.maximum_local_goal_slope)
            or self.maximum_local_goal_slope < 0.0
        ):
            raise ValueError('maximum_local_goal_slope must be finite and non-negative')
        if self.goal_yaw_tolerance > math.pi:
            raise ValueError('goal_yaw_tolerance must not exceed pi radians')
        if not (
            math.isfinite(self.path_collinear_angle_threshold)
            and 0.0 <= self.path_collinear_angle_threshold <= math.pi
        ):
            raise ValueError('path_collinear_angle_threshold must be in [0, pi]')
        if self.path_smoothing_iterations < 0:
            raise ValueError('path_smoothing_iterations must be non-negative')
        if (
            not math.isfinite(self.path_smoothing_data_weight)
            or self.path_smoothing_data_weight < 0.0
            or not math.isfinite(self.path_smoothing_smooth_weight)
            or self.path_smoothing_smooth_weight < 0.0
        ):
            raise ValueError('path smoothing weights must be finite and non-negative')

    def _global_path_cb(self, message: Path):
        if not message.poses:
            self._global_path_msg = None
            self._global_points = []
            self._global_zs = []
            self._progress_index = None
            self._local_goal_msg = None
            self._last_local_path_msg = None
            self._last_local_path_publish_time = None
            self._publish_final_approach(False)
            self._stop(CoordinatorState.WAIT_GLOBAL_PATH, 'received empty global path')
            self._cancel_active_goal()
            return
        if message.header.frame_id != self.global_frame:
            self._stop(
                CoordinatorState.BLOCKED,
                f'global path frame {message.header.frame_id!r} is not {self.global_frame!r}',
            )
            return

        points = [Point2(p.pose.position.x, p.pose.position.y) for p in message.poses]
        z_values = [p.pose.position.z for p in message.poses]
        if any(not math.isfinite(p.x) or not math.isfinite(p.y) for p in points):
            self._stop(CoordinatorState.BLOCKED, 'global path contains non-finite points')
            return
        if any(not math.isfinite(z) for z in z_values):
            self._stop(CoordinatorState.BLOCKED, 'global path contains non-finite z')
            return

        if self._motion_path_active:
            self._publish_empty_path()
        else:
            self._publish_final_approach(False)
        self._global_path_msg = message
        self._global_points = points
        self._global_zs = z_values
        self._progress_index = None
        self._local_goal_msg = None
        self._local_goal_index = None
        self._last_art_path_time = None
        self._last_local_path_msg = None
        self._last_local_path_publish_time = None
        self._cancel_active_goal()
        self._set_state(CoordinatorState.PLANNING, 'new global path received')

    def _map_cb(self, message: GridMap):
        if message.header.frame_id != self.global_frame:
            self._stop(
                CoordinatorState.BLOCKED,
                f'GridMap frame {message.header.frame_id!r} is not {self.global_frame!r}',
            )
            return
        info = message.info
        if info.length_x <= 0.0 or info.length_y <= 0.0:
            self._stop(CoordinatorState.BLOCKED, 'GridMap has invalid dimensions')
            return
        self._map_geometry = MapGeometry(
            center_x=info.pose.position.x,
            center_y=info.pose.position.y,
            yaw=quaternion_to_yaw(info.pose.orientation),
            length_x=info.length_x,
            length_y=info.length_y,
        )
        self._map_receive_time = self.get_clock().now()

    def _art_path_cb(self, message: Path):
        if self._local_goal_msg is None or self._robot_point is None:
            return
        if message.header.frame_id != self.global_frame:
            self._stop(
                CoordinatorState.BLOCKED,
                f'ART path frame {message.header.frame_id!r} is not {self.global_frame!r}',
            )
            return

        points = [Point2(p.pose.position.x, p.pose.position.y) for p in message.poses]
        if len(points) < 2 and self._local_goal_transition_active():
            self.get_logger().info(
                'Ignoring empty ART path while local goal is being updated',
                throttle_duration_sec=1.0,
            )
            return

        local_goal = Point2(
            self._local_goal_msg.pose.position.x,
            self._local_goal_msg.pose.position.y,
        )
        final_goal = self._is_final_local_goal()
        maximum_goal_distance = (
            self.final_maximum_path_goal_distance
            if final_goal
            else self.maximum_path_goal_distance
        )
        valid, reason = validate_local_path(
            points,
            self._robot_point,
            local_goal,
            self.maximum_path_start_distance,
            maximum_goal_distance,
        )
        if not valid:
            self._stop(CoordinatorState.BLOCKED, reason)
            return

        output_points = points
        path_kind = 'local planner'
        if self.enable_path_smoothing:
            if self.require_map_for_goal_selection and self._map_geometry is None:
                self._stop(CoordinatorState.WAIT_MAP, 'cannot smooth path without GridMap')
                return
            output_points = smooth_local_path(points, self.path_smoothing_config)
            valid, reason = validate_local_path(
                output_points,
                self._robot_point,
                local_goal,
                self.maximum_path_start_distance,
                maximum_goal_distance,
            )
            if not valid:
                self._stop(CoordinatorState.BLOCKED, f'smoothed path invalid: {reason}')
                return
            if self._map_geometry is not None:
                valid, reason = validate_smoothed_path(
                    output_points,
                    points,
                    self._map_geometry,
                    self.map_edge_margin,
                    self.path_max_deviation,
                )
                if not valid:
                    self._stop(CoordinatorState.BLOCKED, reason)
                    return
            path_kind = 'smoothed local planner'

        if self.enable_path_smoothing:
            output_message = self._make_local_path_message(message, output_points)
        else:
            message.header.stamp = self.get_clock().now().to_msg()
            output_message = message
        self._publish_final_approach(final_goal)
        self._local_path_pub.publish(output_message)
        self._last_art_path_time = self.get_clock().now()
        self._last_local_path_msg = output_message
        self._last_local_path_publish_time = self._last_art_path_time
        self._motion_path_active = True
        self._set_state(
            CoordinatorState.TRACKING,
            f'tracking {path_kind} path with {len(output_points)} poses',
        )

    def _make_local_path_message(
        self, source: Path, points: Sequence[Point2]
    ) -> Path:
        message = Path()
        message.header.frame_id = self.global_frame
        message.header.stamp = self.get_clock().now().to_msg()

        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = message.header.frame_id
            pose.header.stamp = message.header.stamp
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = self._interpolate_z_on_source_path(point, source)
            if index == len(points) - 1:
                pose.pose.orientation = source.poses[-1].pose.orientation
            else:
                yaw = tangent_yaw(points, index)
                pose.pose.orientation.z = math.sin(0.5 * yaw)
                pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        return message

    def _interpolate_z_on_source_path(self, point: Point2, source: Path) -> float:
        if not source.poses:
            return 0.0
        if len(source.poses) == 1:
            z = source.poses[0].pose.position.z
            return z if math.isfinite(z) else 0.0

        best_distance = float('inf')
        best_z = 0.0
        for index in range(len(source.poses) - 1):
            start = source.poses[index].pose.position
            end = source.poses[index + 1].pose.position
            start_point = Point2(start.x, start.y)
            end_point = Point2(end.x, end.y)
            dx = end_point.x - start_point.x
            dy = end_point.y - start_point.y
            length_sq = dx * dx + dy * dy
            if length_sq <= 0.0:
                ratio = 0.0
                projected = start_point
            else:
                ratio = (
                    (point.x - start_point.x) * dx
                    + (point.y - start_point.y) * dy
                ) / length_sq
                ratio = max(0.0, min(1.0, ratio))
                projected = Point2(
                    start_point.x + ratio * dx,
                    start_point.y + ratio * dy,
                )

            projected_distance = distance(point, projected)
            if projected_distance < best_distance:
                z0 = start.z if math.isfinite(start.z) else 0.0
                z1 = end.z if math.isfinite(end.z) else z0
                best_distance = projected_distance
                best_z = z0 + ratio * (z1 - z0)
        return best_z

    def _local_goal_failure_reason(self, require_map: bool, robot: Point2) -> str:
        if not self._global_points or self._progress_index is None:
            return 'planner range'

        saw_range_candidate = False
        saw_map_candidate = not require_map
        saw_z_rejection = False
        saw_slope_rejection = False
        arc_distance = 0.0
        for index in range(self._progress_index + 1, len(self._global_points)):
            arc_distance += distance(
                self._global_points[index - 1], self._global_points[index]
            )
            if arc_distance > self.maximum_goal_distance:
                break
            if arc_distance < self.minimum_goal_distance:
                continue
            saw_range_candidate = True
            if require_map and not self._point_is_inside_current_map(
                self._global_points[index]
            ):
                continue
            saw_map_candidate = True
            xy_distance = distance(robot, self._global_points[index])
            rejection = local_goal_z_slope_rejection_reason(
                self._global_zs,
                index,
                self._robot_z,
                self.maximum_local_goal_z_delta,
                xy_distance,
                self.maximum_local_goal_slope,
            )
            if rejection is None:
                continue
            saw_z_rejection = saw_z_rejection or rejection == 'z window'
            saw_slope_rejection = saw_slope_rejection or rejection == 'slope window'

        if not saw_range_candidate:
            return 'planner range'
        if require_map and not saw_map_candidate:
            return 'GridMap'
        if saw_slope_rejection:
            return 'slope window'
        if saw_z_rejection:
            return 'z window'
        return 'planner range'

    def _point_is_inside_current_map(self, point: Point2) -> bool:
        if self._map_geometry is None:
            return False
        return point_inside_map(point, self._map_geometry, self.map_edge_margin)

    def _republish_local_path(self):
        if (
            not self._motion_path_active
            or self._last_local_path_msg is None
            or self._last_local_path_publish_time is None
        ):
            return
        now = self.get_clock().now()
        age = (now - self._last_local_path_publish_time).nanoseconds * 1e-9
        if age < 0.0 or age > self.local_path_reuse_timeout:
            return

        message = self._crop_path_for_robot(self._last_local_path_msg)
        message.header.stamp = now.to_msg()
        for pose in message.poses:
            pose.header.stamp = message.header.stamp
        self._local_path_pub.publish(message)

    def _crop_path_for_robot(self, source: Path) -> Path:
        message = Path()
        message.header.frame_id = source.header.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        if not source.poses:
            return message
        if self._robot_point is None or len(source.poses) <= 2:
            message.poses = [deepcopy(pose) for pose in source.poses]
            return message

        nearest_index = min(
            range(len(source.poses)),
            key=lambda index: distance(
                self._robot_point,
                Point2(
                    source.poses[index].pose.position.x,
                    source.poses[index].pose.position.y,
                ),
            ),
        )
        start_index = min(nearest_index, max(0, len(source.poses) - 2))
        message.poses = [deepcopy(pose) for pose in source.poses[start_index:]]
        return message

    def _update(self):
        if not self._global_points:
            if self._state == CoordinatorState.GOAL_REACHED:
                return
            self._stop(CoordinatorState.WAIT_GLOBAL_PATH, 'waiting for global path')
            return

        now = self.get_clock().now()
        if self.require_map_for_goal_selection:
            if self._map_geometry is None or self._map_receive_time is None:
                self._stop(CoordinatorState.WAIT_MAP, 'waiting for traversability map')
                return
            if (now - self._map_receive_time).nanoseconds * 1e-9 > self.map_timeout:
                self._stop(CoordinatorState.WAIT_MAP, 'traversability map timed out')
                return

        robot = self._lookup_robot_point(now)
        if robot is None:
            self._stop(CoordinatorState.BLOCKED, 'robot localization unavailable or stale')
            return
        self._robot_point = robot

        goal_distance = distance(robot, self._global_points[-1])
        goal_yaw = quaternion_to_yaw(
            self._global_path_msg.poses[-1].pose.orientation
        )
        goal_yaw_error = normalize_angle(goal_yaw - self._robot_yaw)
        if (
            goal_distance <= self.goal_reached_distance
            and abs(goal_yaw_error) <= self.goal_yaw_tolerance
        ):
            self._complete_global_goal(goal_distance, goal_yaw_error)
            return
        if goal_distance <= self.goal_reached_distance:
            final_index = len(self._global_points) - 1
            if (
                self._local_goal_index != final_index
                and self._goal_update_period_elapsed(now)
            ):
                final_goal = self._make_local_goal(final_index)
                self._local_goal_msg = final_goal
                self._local_goal_index = final_index
                self._local_goal_pub.publish(final_goal)
                self._last_goal_request_time = now
                self._request_art_goal(final_goal)
            self._set_state(
                CoordinatorState.TRACKING
                if self._motion_path_active else CoordinatorState.PLANNING,
                f'aligning final yaw: error={goal_yaw_error:.3f} rad',
            )
            return

        if self._is_final_local_goal():
            if (
                self._motion_path_active
                and self._last_art_path_time is not None
                and (now - self._last_art_path_time).nanoseconds * 1e-9
                > self.art_path_timeout
            ):
                self._stop(CoordinatorState.BLOCKED, 'ART path timed out')
                return
            if self._pending_goal is not None and self._goal_handle is None:
                self._send_pending_goal()
            self._set_state(
                CoordinatorState.TRACKING
                if self._motion_path_active else CoordinatorState.PLANNING,
                f'holding final approach: distance={goal_distance:.3f} m, '
                f'yaw_error={goal_yaw_error:.3f} rad',
            )
            return

        self._progress_index = closest_progress_index(
            self._global_points,
            robot,
            self._progress_index,
            self.progress_backtrack_points,
            self.progress_forward_search_points,
        )
        if self.require_map_for_goal_selection:
            selection = select_local_goal(
                self._global_points,
                self._progress_index,
                self._map_geometry,
                self.map_edge_margin,
                self.lookahead_distance,
                self.minimum_goal_distance,
                self.maximum_goal_distance,
                self._global_zs,
                self._robot_z,
                self.maximum_local_goal_z_delta,
                robot,
                self.maximum_local_goal_slope,
            )
            no_selection_reason = (
                'no valid PCT local goal inside '
                f'{self._local_goal_failure_reason(True, robot)}'
            )
        else:
            selection = select_local_goal_without_map(
                self._global_points,
                self._progress_index,
                robot,
                self.lookahead_distance,
                self.minimum_goal_distance,
                self.maximum_goal_distance,
                self._global_zs,
                self._robot_z,
                self.maximum_local_goal_z_delta,
                self.maximum_local_goal_slope,
            )
            no_selection_reason = (
                'no valid PCT local goal inside '
                f'{self._local_goal_failure_reason(False, robot)}'
            )
        if selection is None:
            self._stop(CoordinatorState.BLOCKED, no_selection_reason)
            return

        candidate = self._make_local_goal(selection.index)
        candidate_point = Point2(candidate.pose.position.x, candidate.pose.position.y)
        request_goal = self._local_goal_msg is None
        if self._local_goal_msg is not None:
            current_goal = Point2(
                self._local_goal_msg.pose.position.x,
                self._local_goal_msg.pose.position.y,
            )
            request_goal = (
                distance(current_goal, robot) <= self.advance_distance
                or distance(current_goal, candidate_point) >= self.goal_update_distance
            )

        if request_goal and self._goal_update_period_elapsed(now):
            self._local_goal_msg = candidate
            self._local_goal_index = selection.index
            self._local_goal_pub.publish(candidate)
            self._last_goal_request_time = now
            self._request_art_goal(candidate)
            self._set_state(
                CoordinatorState.PLANNING,
                f'planning to PCT waypoint {selection.index}',
            )

        if (
            self._motion_path_active
            and self._last_art_path_time is not None
            and (now - self._last_art_path_time).nanoseconds * 1e-9
            > self.art_path_timeout
        ):
            self._stop(CoordinatorState.BLOCKED, 'ART path timed out')

        if self._pending_goal is not None and self._goal_handle is None:
            self._send_pending_goal()

    def _lookup_robot_point(self, now) -> Optional[Point2]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exception:
            self.get_logger().warn(
                f'TF lookup failed: {exception}', throttle_duration_sec=2.0
            )
            return None

        stamp = rclpy.time.Time.from_msg(transform.header.stamp)
        if stamp.nanoseconds > 0:
            age = (now - stamp).nanoseconds * 1e-9
            if age < 0.0 or age > self.localization_timeout:
                return None
        self._robot_yaw = quaternion_to_yaw(transform.transform.rotation)
        self._robot_z = transform.transform.translation.z
        return Point2(
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _make_local_goal(self, index: int) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        source_pose = self._global_path_msg.poses[index].pose
        goal.pose.position.x = source_pose.position.x
        goal.pose.position.y = source_pose.position.y
        goal.pose.position.z = source_pose.position.z
        if index == len(self._global_points) - 1:
            goal.pose.orientation = source_pose.orientation
        else:
            yaw = tangent_yaw(self._global_points, index)
            goal.pose.orientation.z = math.sin(0.5 * yaw)
            goal.pose.orientation.w = math.cos(0.5 * yaw)
        return goal

    def _complete_global_goal(self, goal_distance: float, yaw_error: float):
        # Clearing the task below latches GOAL_REACHED, so this stop path is
        # published exactly once for the completed task.
        self._publish_empty_path()
        self._pending_goal = None
        self._cancel_active_goal()
        self._global_path_msg = None
        self._global_points = []
        self._global_zs = []
        self._progress_index = None
        self._local_goal_msg = None
        self._local_goal_index = None
        self._last_art_path_time = None
        self._last_local_path_msg = None
        self._last_local_path_publish_time = None
        self._last_goal_request_time = None
        self._set_state(
            CoordinatorState.GOAL_REACHED,
            f'global goal reached: distance={goal_distance:.3f} m, '
            f'yaw_error={yaw_error:.3f} rad',
        )

    def _goal_update_period_elapsed(self, now) -> bool:
        if self._last_goal_request_time is None:
            return True
        return (
            (now - self._last_goal_request_time).nanoseconds * 1e-9
            >= self.goal_update_min_period
        )

    def _local_goal_transition_active(self) -> bool:
        if (
            self._pending_goal is not None
            or self._send_future is not None
            or self._cancel_future is not None
        ):
            return True
        if self._last_goal_request_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self._last_goal_request_time
        ).nanoseconds * 1e-9
        return 0.0 <= elapsed <= self.empty_art_path_grace_period

    def _request_art_goal(self, goal: PoseStamped):
        self._pending_goal = goal
        if self._goal_handle is not None:
            self._cancel_active_goal()
        elif self._send_future is None:
            self._send_pending_goal()

    def _send_pending_goal(self):
        if self._pending_goal is None or self._send_future is not None:
            return
        if not self._art_client.server_is_ready():
            self.get_logger().warn(
                f'Waiting for ART action server {self.art_action_name}',
                throttle_duration_sec=2.0,
            )
            return
        message = PlanToGoal.Goal()
        message.goal = self._pending_goal
        self._pending_goal = None
        self._send_future = self._art_client.send_goal_async(
            message, feedback_callback=self._art_feedback_cb
        )
        self._send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self._send_future = None
        try:
            goal_handle = future.result()
        except Exception as exception:  # noqa: BLE001 - ROS future errors vary
            self._stop(CoordinatorState.BLOCKED, f'ART goal send failed: {exception}')
            return
        if not goal_handle.accepted:
            self._stop(CoordinatorState.BLOCKED, 'ART rejected local goal')
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, handle=goal_handle: self._goal_result_cb(result, handle)
        )
        if self._pending_goal is not None:
            self._cancel_active_goal()

    def _goal_result_cb(self, future, goal_handle):
        if self._goal_handle is goal_handle:
            self._goal_handle = None
            self._cancel_future = None
        if self._pending_goal is not None:
            self._send_pending_goal()

    def _cancel_active_goal(self):
        if self._goal_handle is None or self._cancel_future is not None:
            return
        handle = self._goal_handle
        self._cancel_future = handle.cancel_goal_async()
        self._cancel_future.add_done_callback(
            lambda future, goal_handle=handle: self._cancel_done_cb(future, goal_handle)
        )

    def _cancel_done_cb(self, future, goal_handle):
        self._cancel_future = None
        if self._goal_handle is goal_handle:
            self._goal_handle = None
        if self._pending_goal is not None:
            self._send_pending_goal()

    def _art_feedback_cb(self, feedback_message):
        status = feedback_message.feedback.status
        failures = {
            PlanToGoal.Feedback.INVALID_START: 'local planner reports invalid start',
            PlanToGoal.Feedback.INVALID_GOAL: 'local planner reports invalid local goal',
            PlanToGoal.Feedback.NO_SOLUTION: 'local planner found no local path',
            PlanToGoal.Feedback.NO_GOAL_TF: 'local planner cannot transform local goal',
            PlanToGoal.Feedback.NO_MAP: 'local planner has no map',
            PlanToGoal.Feedback.NO_ROBOT_TF: 'local planner cannot obtain robot TF',
        }
        if status in failures:
            self._stop(CoordinatorState.BLOCKED, failures[status])

    def _stop(self, state: str, reason: str):
        if self._motion_path_active:
            self._publish_empty_path()
        if state in (
            CoordinatorState.WAIT_GLOBAL_PATH,
            CoordinatorState.WAIT_MAP,
            CoordinatorState.BLOCKED,
            CoordinatorState.GOAL_REACHED,
        ):
            self._pending_goal = None
            self._local_goal_msg = None
            self._local_goal_index = None
            self._publish_final_approach(False)
            self._cancel_active_goal()
        self._set_state(state, reason)

    def _publish_empty_path(self):
        message = Path()
        message.header.frame_id = self.global_frame
        message.header.stamp = self.get_clock().now().to_msg()
        self._local_path_pub.publish(message)
        self._motion_path_active = False
        self._last_art_path_time = None
        self._last_local_path_msg = None
        self._last_local_path_publish_time = None
        self._publish_final_approach(False)

    def _is_final_local_goal(self) -> bool:
        return bool(
            self._global_points
            and self._local_goal_index is not None
            and self._local_goal_index == len(self._global_points) - 1
        )

    def _publish_final_approach(self, active: bool):
        if active == self._final_approach_active:
            return
        self._final_approach_active = active
        message = Bool()
        message.data = active
        self._final_approach_pub.publish(message)

    def _set_state(self, state: str, reason: str):
        if state == self._state and reason == self._state_reason:
            return
        self._state = state
        self._state_reason = reason
        message = String()
        message.data = f'{state}: {reason}'
        self._status_pub.publish(message)
        if state == CoordinatorState.BLOCKED:
            self.get_logger().error(message.data)
        else:
            self.get_logger().info(message.data)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PctArtCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and rclpy.ok():
            node._publish_empty_path()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
