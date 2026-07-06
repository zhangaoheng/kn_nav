"""ROS-independent geometry and validation helpers for the coordinator."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class Point2:
    x: float
    y: float


@dataclass(frozen=True)
class MapGeometry:
    center_x: float
    center_y: float
    yaw: float
    length_x: float
    length_y: float


@dataclass(frozen=True)
class LocalGoalSelection:
    index: int
    arc_distance: float


def z_delta_ok(
    path_z: Optional[Sequence[float]],
    index: int,
    robot_z: Optional[float],
    maximum_z_delta: Optional[float],
) -> bool:
    if path_z is None or robot_z is None or maximum_z_delta is None:
        return True
    if maximum_z_delta <= 0.0:
        return True
    if not 0 <= index < len(path_z):
        return False
    goal_z = path_z[index]
    return (
        math.isfinite(goal_z)
        and math.isfinite(robot_z)
        and abs(goal_z - robot_z) <= maximum_z_delta
    )


def z_slope_ok(
    path_z: Optional[Sequence[float]],
    index: int,
    robot_z: Optional[float],
    maximum_z_delta: Optional[float],
    xy_distance: Optional[float],
    maximum_slope: Optional[float],
) -> bool:
    if not z_delta_ok(path_z, index, robot_z, maximum_z_delta):
        return False
    if (
        path_z is None
        or robot_z is None
        or maximum_slope is None
        or maximum_slope <= 0.0
    ):
        return True
    if not 0 <= index < len(path_z):
        return False
    goal_z = path_z[index]
    if not math.isfinite(goal_z) or not math.isfinite(robot_z):
        return False
    dz = abs(goal_z - robot_z)
    if dz <= 0.0:
        return True
    if xy_distance is None or not math.isfinite(xy_distance) or xy_distance <= 1e-6:
        return False
    return dz / xy_distance <= maximum_slope


def local_goal_z_slope_rejection_reason(
    path_z: Optional[Sequence[float]],
    index: int,
    robot_z: Optional[float],
    maximum_z_delta: Optional[float],
    xy_distance: Optional[float],
    maximum_slope: Optional[float],
) -> Optional[str]:
    if z_slope_ok(
        path_z,
        index,
        robot_z,
        maximum_z_delta,
        xy_distance,
        maximum_slope,
    ):
        return None
    if not z_delta_ok(path_z, index, robot_z, maximum_z_delta):
        return 'z window'
    return 'slope window'


def distance(a: Point2, b: Point2) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def point_inside_map(point: Point2, geometry: MapGeometry, margin: float) -> bool:
    """Return whether a world-frame point is inside the rotated map inset."""
    usable_half_x = 0.5 * geometry.length_x - margin
    usable_half_y = 0.5 * geometry.length_y - margin
    if usable_half_x <= 0.0 or usable_half_y <= 0.0:
        return False

    dx = point.x - geometry.center_x
    dy = point.y - geometry.center_y
    cos_yaw = math.cos(geometry.yaw)
    sin_yaw = math.sin(geometry.yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    return abs(local_x) <= usable_half_x and abs(local_y) <= usable_half_y


def closest_progress_index(
    path: Sequence[Point2],
    robot: Point2,
    previous_index: Optional[int],
    backtrack_points: int,
    forward_search_points: int,
) -> int:
    """Find the closest point in a bounded window around prior progress."""
    if not path:
        raise ValueError('path must not be empty')
    if previous_index is None:
        start = 0
        end = len(path)
    else:
        previous_index = max(0, min(previous_index, len(path) - 1))
        start = max(0, previous_index - max(0, backtrack_points))
        end = min(len(path), previous_index + max(1, forward_search_points) + 1)

    return min(range(start, end), key=lambda index: distance(path[index], robot))


def select_local_goal(
    path: Sequence[Point2],
    progress_index: int,
    geometry: MapGeometry,
    edge_margin: float,
    desired_distance: float,
    minimum_distance: float,
    maximum_distance: float,
    path_z: Optional[Sequence[float]] = None,
    robot_z: Optional[float] = None,
    maximum_z_delta: Optional[float] = None,
    robot: Optional[Point2] = None,
    maximum_slope: Optional[float] = None,
) -> Optional[LocalGoalSelection]:
    """Select an in-map path point closest to the desired arc lookahead."""
    if not path or not 0 <= progress_index < len(path):
        return None
    if not 0.0 <= minimum_distance <= desired_distance <= maximum_distance:
        raise ValueError('lookahead distances must be non-negative and ordered')

    candidates = []
    arc_distance = 0.0
    for index in range(progress_index + 1, len(path)):
        arc_distance += distance(path[index - 1], path[index])
        if arc_distance > maximum_distance:
            break
        xy_distance = distance(robot, path[index]) if robot is not None else arc_distance
        if arc_distance >= minimum_distance and point_inside_map(
            path[index], geometry, edge_margin
        ) and z_slope_ok(
            path_z,
            index,
            robot_z,
            maximum_z_delta,
            xy_distance,
            maximum_slope,
        ):
            candidates.append(LocalGoalSelection(index, arc_distance))

    if not candidates:
        return None

    final_candidate = next(
        (candidate for candidate in candidates if candidate.index == len(path) - 1),
        None,
    )
    if final_candidate is not None:
        return final_candidate
    return min(
        candidates,
        key=lambda candidate: (
            abs(candidate.arc_distance - desired_distance),
            -candidate.arc_distance,
        ),
    )


def select_local_goal_without_map(
    path: Sequence[Point2],
    progress_index: int,
    robot: Point2,
    desired_distance: float,
    minimum_distance: float,
    maximum_distance: float,
    path_z: Optional[Sequence[float]] = None,
    robot_z: Optional[float] = None,
    maximum_z_delta: Optional[float] = None,
    maximum_slope: Optional[float] = None,
) -> Optional[LocalGoalSelection]:
    """Select a local goal by path progress and robot distance only."""
    if not path or not 0 <= progress_index < len(path):
        return None
    if not 0.0 <= minimum_distance <= desired_distance <= maximum_distance:
        raise ValueError('lookahead distances must be non-negative and ordered')

    candidates = []
    arc_distance = 0.0
    for index in range(progress_index + 1, len(path)):
        arc_distance += distance(path[index - 1], path[index])
        if arc_distance > maximum_distance:
            break
        robot_distance = distance(robot, path[index])
        if (
            arc_distance >= minimum_distance
            and robot_distance <= maximum_distance
            and z_slope_ok(
                path_z,
                index,
                robot_z,
                maximum_z_delta,
                robot_distance,
                maximum_slope,
            )
        ):
            candidates.append(LocalGoalSelection(index, arc_distance))

    if not candidates:
        return None

    final_candidate = next(
        (candidate for candidate in candidates if candidate.index == len(path) - 1),
        None,
    )
    if final_candidate is not None:
        return final_candidate
    return min(
        candidates,
        key=lambda candidate: (
            abs(candidate.arc_distance - desired_distance),
            -candidate.arc_distance,
        ),
    )


def tangent_yaw(path: Sequence[Point2], index: int) -> float:
    if not path or not 0 <= index < len(path):
        raise ValueError('invalid path index')
    if len(path) == 1:
        return 0.0
    if index < len(path) - 1:
        first, second = path[index], path[index + 1]
    else:
        first, second = path[index - 1], path[index]
    return math.atan2(second.y - first.y, second.x - first.x)


def validate_local_path(
    points: Sequence[Point2],
    robot: Point2,
    goal: Point2,
    maximum_start_distance: float,
    maximum_goal_distance: float,
) -> Tuple[bool, str]:
    if len(points) < 2:
        return False, 'ART path has fewer than two poses'
    if any(not math.isfinite(point.x) or not math.isfinite(point.y) for point in points):
        return False, 'ART path contains a non-finite point'
    start_distance = distance(points[0], robot)
    if start_distance > maximum_start_distance:
        return False, f'ART path starts {start_distance:.2f} m from robot'
    goal_distance = distance(points[-1], goal)
    if goal_distance > maximum_goal_distance:
        return False, f'ART path ends {goal_distance:.2f} m from local goal'
    return True, ''
