import math

from pct_art_local_navigation.coordinator_logic import (
    MapGeometry,
    Point2,
    closest_progress_index,
    point_inside_map,
    select_local_goal,
    select_local_goal_without_map,
    tangent_yaw,
    validate_local_path,
)


def test_rotated_map_boundary_and_margin():
    geometry = MapGeometry(1.0, 2.0, math.pi / 2.0, 10.0, 6.0)
    assert point_inside_map(Point2(1.0, 5.5), geometry, 0.5)
    assert not point_inside_map(Point2(1.0, 6.6), geometry, 0.5)
    assert not point_inside_map(Point2(3.6, 2.0), geometry, 0.5)


def test_progress_search_is_bounded_around_previous_index():
    path = [Point2(float(i), 0.0) for i in range(20)]
    # A geometrically close repeated section outside the search window cannot
    # cause progress to jump across the global route.
    path[18] = Point2(5.0, 0.01)
    index = closest_progress_index(path, Point2(5.0, 0.0), 4, 2, 5)
    assert index == 5


def test_local_goal_prefers_desired_arc_distance():
    path = [Point2(i * 0.5, 0.0) for i in range(15)]
    geometry = MapGeometry(0.0, 0.0, 0.0, 10.0, 10.0)
    selection = select_local_goal(path, 0, geometry, 0.8, 3.5, 0.8, 4.0)
    assert selection is not None
    assert selection.index == 7
    assert selection.arc_distance == 3.5


def test_local_goal_uses_final_goal_when_reachable():
    path = [Point2(0.0, 0.0), Point2(1.0, 0.0), Point2(2.0, 0.0)]
    geometry = MapGeometry(0.0, 0.0, 0.0, 10.0, 10.0)
    selection = select_local_goal(path, 0, geometry, 0.8, 3.5, 0.8, 4.0)
    assert selection is not None
    assert selection.index == 2


def test_local_goal_rejects_large_z_delta():
    path = [Point2(i, 0.0) for i in range(6)]
    path_z = [0.0, 2.0, 2.0, 2.0, 0.2, 0.2]
    geometry = MapGeometry(0.0, 0.0, 0.0, 10.0, 10.0)

    selection = select_local_goal(
        path,
        0,
        geometry,
        0.8,
        3.5,
        0.8,
        4.0,
        path_z,
        0.0,
        0.8,
    )

    assert selection is not None
    assert selection.index == 4


def test_local_goal_rejects_large_slope():
    path = [Point2(i, 0.0) for i in range(6)]
    path_z = [0.0, 0.6, 1.2, 1.2, 0.2, 0.2]
    geometry = MapGeometry(0.0, 0.0, 0.0, 10.0, 10.0)

    selection = select_local_goal(
        path,
        0,
        geometry,
        0.8,
        3.5,
        0.8,
        4.0,
        path_z,
        0.0,
        2.0,
        Point2(0.0, 0.0),
        0.35,
    )

    assert selection is not None
    assert selection.index == 4


def test_local_goal_rejects_points_near_map_edge():
    path = [Point2(i, 0.0) for i in range(6)]
    geometry = MapGeometry(0.0, 0.0, 0.0, 4.0, 4.0)
    selection = select_local_goal(path, 0, geometry, 0.8, 3.5, 0.8, 4.0)
    assert selection is not None
    assert selection.index == 1


def test_local_goal_without_map_uses_range_and_final_goal():
    path = [Point2(i, 0.0) for i in range(6)]
    selection = select_local_goal_without_map(
        path, 0, Point2(0.0, 0.0), 3.5, 0.8, 4.0
    )
    assert selection is not None
    assert selection.index == 4

    short_path = [Point2(0.0, 0.0), Point2(1.0, 0.0), Point2(2.0, 0.0)]
    selection = select_local_goal_without_map(
        short_path, 0, Point2(0.0, 0.0), 3.5, 0.8, 4.0
    )
    assert selection is not None
    assert selection.index == 2


def test_local_goal_without_map_rejects_large_z_delta():
    path = [Point2(i, 0.0) for i in range(6)]
    path_z = [0.0, 2.0, 2.0, 2.0, 0.2, 0.2]
    selection = select_local_goal_without_map(
        path,
        0,
        Point2(0.0, 0.0),
        3.5,
        0.8,
        4.0,
        path_z,
        0.0,
        0.8,
    )
    assert selection is not None
    assert selection.index == 4


def test_local_goal_without_map_rejects_large_slope():
    path = [Point2(i, 0.0) for i in range(6)]
    path_z = [0.0, 0.6, 1.2, 1.2, 0.2, 0.2]
    selection = select_local_goal_without_map(
        path,
        0,
        Point2(0.0, 0.0),
        3.5,
        0.8,
        4.0,
        path_z,
        0.0,
        2.0,
        0.35,
    )
    assert selection is not None
    assert selection.index == 4


def test_tangent_yaw_uses_previous_segment_at_final_point():
    path = [Point2(0.0, 0.0), Point2(0.0, 1.0)]
    assert math.isclose(tangent_yaw(path, 1), math.pi / 2.0)


def test_local_path_validation_checks_both_ends():
    robot = Point2(0.0, 0.0)
    goal = Point2(2.0, 0.0)
    valid, reason = validate_local_path(
        [Point2(0.1, 0.0), Point2(2.1, 0.0)], robot, goal, 1.0, 0.8
    )
    assert valid
    assert reason == ''

    valid, reason = validate_local_path(
        [Point2(1.1, 0.0), Point2(2.0, 0.0)], robot, goal, 1.0, 0.8
    )
    assert not valid
    assert 'starts' in reason

    valid, reason = validate_local_path(
        [Point2(0.0, 0.0), Point2(float('nan'), 0.0)], robot, goal, 1.0, 0.8
    )
    assert not valid
    assert 'non-finite' in reason
