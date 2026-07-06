import os
import math

from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration

from pct_art_local_navigation.coordinator_logic import Point2
from pct_art_local_navigation.coordinator_node import (
    CoordinatorState,
    PctArtCoordinator,
    quaternion_to_yaw,
)


class PublisherRecorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_path(xs, final_yaw=0.0):
    return make_xy_path([(float(x), 0.0) for x in xs], final_yaw)


def make_xy_path(points, final_yaw=0.0):
    message = Path()
    message.header.frame_id = 'map'
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    message.poses[-1].pose.orientation.z = math.sin(0.5 * final_yaw)
    message.poses[-1].pose.orientation.w = math.cos(0.5 * final_yaw)
    return message


def make_empty_path():
    message = Path()
    message.header.frame_id = 'map'
    return message


def make_map():
    message = GridMap()
    message.header.frame_id = 'map'
    message.info.length_x = 10.0
    message.info.length_y = 10.0
    message.info.resolution = 0.1
    message.info.pose.orientation.w = 1.0
    return message


def test_node_selects_goal_and_forwards_only_valid_art_path():
    os.environ['ROS_LOG_DIR'] = '/tmp/pct_art_local_navigation_test_logs'
    rclpy.init()
    node = PctArtCoordinator()
    try:
        requested_goals = []
        local_paths = PublisherRecorder()
        local_goals = PublisherRecorder()
        final_approach = PublisherRecorder()
        node._request_art_goal = requested_goals.append
        node._local_path_pub = local_paths
        node._local_goal_pub = local_goals
        node._final_approach_pub = final_approach
        node._lookup_robot_point = lambda now: Point2(0.0, 0.0)

        node._map_cb(make_map())
        node._global_path_cb(make_path([0.0, 1.0, 2.0, 3.0, 3.5, 4.0]))
        node._update()

        assert len(requested_goals) == 1
        assert requested_goals[0].pose.position.x == 4.0
        assert len(local_goals.messages) == 1
        assert node._state == CoordinatorState.PLANNING

        valid_art_path = make_path([0.1, 1.0, 2.0, 4.0])
        node._art_path_cb(valid_art_path)
        assert len(local_paths.messages) == 1
        assert final_approach.messages[-1].data is True
        assert len(local_paths.messages[-1].poses) > 4
        assert math.isclose(local_paths.messages[-1].poses[0].pose.position.x, 0.1)
        assert math.isclose(local_paths.messages[-1].poses[-1].pose.position.x, 4.0)
        assert node._state == CoordinatorState.TRACKING

        invalid_art_path = make_path([2.0, 4.0])
        node._art_path_cb(invalid_art_path)
        assert len(local_paths.messages) == 2
        assert not local_paths.messages[-1].poses
        assert final_approach.messages[-1].data is False
        assert node._state == CoordinatorState.BLOCKED
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_selects_goal_without_traversability_map():
    os.environ['ROS_LOG_DIR'] = '/tmp/pct_art_local_navigation_test_logs'
    rclpy.init()
    node = PctArtCoordinator()
    try:
        requested_goals = []
        local_goals = PublisherRecorder()
        node.require_map_for_goal_selection = False
        node._request_art_goal = requested_goals.append
        node._local_goal_pub = local_goals
        node._lookup_robot_point = lambda now: Point2(0.0, 0.0)

        node._global_path_cb(make_path([0.0, 1.0, 2.0, 3.0, 4.0]))
        node._update()

        assert len(requested_goals) == 1
        assert len(local_goals.messages) == 1
        assert requested_goals[0].pose.position.x == 4.0
        assert node._state == CoordinatorState.PLANNING
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_smoothing_can_be_disabled_to_forward_original_art_path():
    os.environ['ROS_LOG_DIR'] = '/tmp/pct_art_local_navigation_test_logs'
    rclpy.init()
    node = PctArtCoordinator()
    try:
        local_paths = PublisherRecorder()
        node._local_path_pub = local_paths
        node._local_goal_msg = PoseStamped()
        node._local_goal_msg.pose.position.x = 1.0
        node._robot_point = Point2(0.0, 0.0)
        node.enable_path_smoothing = False

        art_path = make_path([0.0, 0.5, 1.0], final_yaw=math.pi / 2.0)
        node._art_path_cb(art_path)

        assert len(local_paths.messages) == 1
        assert len(local_paths.messages[-1].poses) == 3
        assert local_paths.messages[-1] is art_path
        assert math.isclose(
            quaternion_to_yaw(local_paths.messages[-1].poses[-1].pose.orientation),
            math.pi / 2.0,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_empty_art_path_is_ignored_while_goal_is_updating():
    os.environ['ROS_LOG_DIR'] = '/tmp/pct_art_local_navigation_test_logs'
    rclpy.init()
    node = PctArtCoordinator()
    try:
        local_paths = PublisherRecorder()
        node._local_path_pub = local_paths
        node._local_goal_msg = PoseStamped()
        node._local_goal_msg.pose.position.x = 1.0
        node._robot_point = Point2(0.0, 0.0)
        node._last_goal_request_time = node.get_clock().now()
        node._state = CoordinatorState.TRACKING

        node._art_path_cb(make_empty_path())

        assert not local_paths.messages
        assert node._state == CoordinatorState.TRACKING
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_empty_art_path_blocks_when_not_switching_goal():
    os.environ['ROS_LOG_DIR'] = '/tmp/pct_art_local_navigation_test_logs'
    rclpy.init()
    node = PctArtCoordinator()
    try:
        local_paths = PublisherRecorder()
        node._local_path_pub = local_paths
        node._local_goal_msg = PoseStamped()
        node._local_goal_msg.pose.position.x = 1.0
        node._robot_point = Point2(0.0, 0.0)
        node._state = CoordinatorState.TRACKING

        node._art_path_cb(make_empty_path())

        assert node._state == CoordinatorState.BLOCKED
        assert 'fewer than two poses' in node._state_reason
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_republishes_recent_local_path_cropped_to_robot():
    os.environ['ROS_LOG_DIR'] = '/tmp/pct_art_local_navigation_test_logs'
    rclpy.init()
    node = PctArtCoordinator()
    try:
        local_paths = PublisherRecorder()
        node._local_path_pub = local_paths
        node._motion_path_active = True
        node._robot_point = Point2(1.1, 0.0)
        node._last_local_path_msg = make_path([0.0, 1.0, 2.0, 3.0])
        node._last_local_path_publish_time = node.get_clock().now()

        node._republish_local_path()

        assert len(local_paths.messages) == 1
        assert len(local_paths.messages[-1].poses) == 3
        assert math.isclose(local_paths.messages[-1].poses[0].pose.position.x, 1.0)
        assert math.isclose(local_paths.messages[-1].poses[-1].pose.position.x, 3.0)

        node._last_local_path_publish_time = (
            node.get_clock().now()
            - Duration(seconds=node.local_path_reuse_timeout + 0.1)
        )
        node._republish_local_path()
        assert len(local_paths.messages) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_smoothed_local_path_preserves_final_yaw_and_rejects_map_escape():
    os.environ['ROS_LOG_DIR'] = '/tmp/pct_art_local_navigation_test_logs'
    rclpy.init()
    node = PctArtCoordinator()
    try:
        local_paths = PublisherRecorder()
        node._local_path_pub = local_paths
        node._map_cb(make_map())
        node._local_goal_msg = PoseStamped()
        node._local_goal_msg.pose.position.x = 2.0
        node._robot_point = Point2(0.0, 0.0)

        art_path = make_xy_path(
            [(0.0, 0.0), (0.8, 0.3), (1.4, -0.2), (2.0, 0.0)],
            final_yaw=math.pi / 2.0,
        )
        node._art_path_cb(art_path)

        assert len(local_paths.messages) == 1
        assert len(local_paths.messages[-1].poses) > len(art_path.poses)
        assert math.isclose(
            quaternion_to_yaw(local_paths.messages[-1].poses[-1].pose.orientation),
            math.pi / 2.0,
        )

        node._local_goal_msg.pose.position.x = 4.6
        escaping_path = make_xy_path([(0.0, 0.0), (4.0, 0.0), (4.6, 0.0)])
        node._art_path_cb(escaping_path)
        assert len(local_paths.messages) == 2
        assert not local_paths.messages[-1].poses
        assert node._state == CoordinatorState.BLOCKED
        assert 'outside GridMap' in node._state_reason
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_completed_goal_is_cleared_and_does_not_resume_after_moving():
    os.environ['ROS_LOG_DIR'] = '/tmp/pct_art_local_navigation_test_logs'
    rclpy.init()
    node = PctArtCoordinator()
    try:
        requested_goals = []
        local_paths = PublisherRecorder()
        node._request_art_goal = requested_goals.append
        node._local_path_pub = local_paths
        node._local_goal_pub = PublisherRecorder()
        node._lookup_robot_point = lambda now: Point2(0.0, 0.0)

        node._map_cb(make_map())
        node._global_path_cb(
            make_path([0.0, 1.0, 2.0, 3.0, 4.0], final_yaw=math.pi / 2.0)
        )
        node._update()
        assert len(requested_goals) == 1
        assert math.isclose(
            quaternion_to_yaw(requested_goals[0].pose.orientation),
            math.pi / 2.0,
        )

        node._art_path_cb(
            make_path([0.1, 1.0, 2.0, 4.0], final_yaw=math.pi / 2.0)
        )
        node._lookup_robot_point = lambda now: Point2(4.0, 0.0)
        node._robot_yaw = 0.0
        node._update()
        assert node._state == CoordinatorState.TRACKING
        assert node._global_points

        node._robot_yaw = math.pi / 2.0
        node._update()
        assert node._state == CoordinatorState.GOAL_REACHED
        assert not node._global_points
        assert node._global_path_msg is None
        assert not local_paths.messages[-1].poses

        node._lookup_robot_point = lambda now: Point2(6.0, 0.0)
        node._robot_yaw = 0.0
        node._update()
        assert node._state == CoordinatorState.GOAL_REACHED
        assert len(requested_goals) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_final_path_must_end_close_to_global_goal():
    os.environ['ROS_LOG_DIR'] = '/tmp/pct_art_local_navigation_test_logs'
    rclpy.init()
    node = PctArtCoordinator()
    try:
        requested_goals = []
        local_paths = PublisherRecorder()
        final_approach = PublisherRecorder()
        node._request_art_goal = requested_goals.append
        node._local_path_pub = local_paths
        node._local_goal_pub = PublisherRecorder()
        node._final_approach_pub = final_approach
        node._lookup_robot_point = lambda now: Point2(0.0, 0.0)

        node._map_cb(make_map())
        node._global_path_cb(make_path([0.0, 1.0, 2.0, 3.0, 4.0]))
        node._update()

        assert node._local_goal_index == len(node._global_points) - 1
        assert len(requested_goals) == 1

        node._art_path_cb(make_path([0.0, 1.0, 2.0, 3.7]))

        assert node._state == CoordinatorState.BLOCKED
        assert 'ends 0.30 m from local goal' in node._state_reason
        assert not local_paths.messages
    finally:
        node.destroy_node()
        rclpy.shutdown()
