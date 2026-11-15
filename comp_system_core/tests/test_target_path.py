"""
helpers/target_path.py: picking and ordering the stops of a path through several targets.

Plain geometry against the real deploy/pathplanner/navgrid.json - no robot is built.  The
end-to-end run (camera sim -> Vision -> path -> PathPlanner follower) is a step of
tests/test_simulation.py.
"""

import math

from wpimath import Translation2d

from helpers.target_path import NavGrid, order_targets, waypoint_poses, _route_length

T = Translation2d


def _blocked_and_free_cells(grid):
    blocked = [(c, r) for r in range(grid.rows) for c in range(grid.cols) if grid.grid[r][c]]
    free = [(c, r) for r in range(grid.rows) for c in range(grid.cols) if not grid.grid[r][c]]
    return blocked, free


def _centre(grid, cell):
    return T((cell[0] + 0.5) * grid.node_size, (cell[1] + 0.5) * grid.node_size)


def test_navgrid_matches_the_file():
    grid = NavGrid()
    blocked, free = _blocked_and_free_cells(grid)
    assert blocked and free, "navgrid.json is all one value - it is not describing a field"
    for cell in blocked[:: max(1, len(blocked) // 25)]:
        p = _centre(grid, cell)
        assert grid.is_blocked(p.x, p.y)
    for cell in free[:: max(1, len(free) // 25)]:
        p = _centre(grid, cell)
        assert not grid.is_blocked(p.x, p.y)
    # off the field is blocked, on every side
    for x, y in [(-0.1, 4), (4, -0.1), (grid.cols * grid.node_size + 0.1, 4), (4, grid.rows * grid.node_size + 0.1)]:
        assert grid.is_blocked(x, y)


def test_segment_through_an_obstacle_is_not_clear():
    grid = NavGrid()
    # find a blocked cell with free cells some way to its left and right on the same row
    for r in range(1, grid.rows - 1):
        row = grid.grid[r]
        for c in range(3, grid.cols - 3):
            if row[c] and not row[c - 3] and not row[c + 3]:
                a, b = _centre(grid, (c - 3, r)), _centre(grid, (c + 3, r))
                assert not grid.segment_clear(a, b)
                assert grid.segment_clear(a, a)
                return
    raise AssertionError("no obstacle with free space either side found in navgrid.json")


def test_order_visits_everything_reachable_and_nothing_blocked():
    grid = NavGrid()
    blocked, _ = _blocked_and_free_cells(grid)
    start = T(7.0, 4.0)
    # an open patch of the neutral zone, well spread so none is swept up in passing
    wanted = [T(8.0, 2.5), T(8.9, 5.5), T(7.6, 5.6), T(8.8, 3.2)]
    for p in wanted + [start]:
        assert not grid.is_blocked(p.x, p.y), f"test point {p} is blocked - navgrid.json changed, move it"
    inside_obstacle = _centre(grid, blocked[len(blocked) // 2])

    route = order_targets(start, wanted + [inside_obstacle], grid, swath_half_width=0.05)
    assert sorted((p.x, p.y) for p in route) == sorted((p.x, p.y) for p in wanted)
    here = start
    for stop in route:
        assert grid.segment_clear(here, stop)
        here = stop


def test_two_opt_does_not_make_the_route_longer_than_nearest_neighbour():
    grid = NavGrid()
    start = T(7.0, 2.4)
    # a zig-zag that nearest-neighbour alone handles badly
    targets = [T(7.6, 2.4), T(7.6, 5.4), T(8.2, 2.6), T(8.2, 5.2), T(8.8, 2.4), T(8.8, 5.4)]
    route = order_targets(start, targets, grid, swath_half_width=0.05)
    assert len(route) == len(targets)

    # plain nearest neighbour, for comparison
    here, left, greedy = start, list(targets), []
    while left:
        nxt = min(left, key=lambda t: here.distance(t))
        left.remove(nxt); greedy.append(nxt); here = nxt
    assert _route_length(start, route) <= _route_length(start, greedy) + 1e-9


def test_targets_swept_up_in_passing_get_no_stop():
    grid = NavGrid()
    start = T(7.0, 4.0)
    # three in a line: driving to the far one passes over the other two
    line = [T(7.5, 4.0), T(8.0, 4.05), T(8.5, 4.0)]
    route = order_targets(start, line, grid, swath_half_width=0.3)
    assert len(route) < len(line)
    # ...but every one of them is within the swath of the route that was chosen
    points = [start] + route
    for target in line:
        nearest = min(abs((b.x - a.x) * (a.y - target.y) - (a.x - target.x) * (b.y - a.y)) / a.distance(b)
                      for a, b in zip(points, points[1:]))
        assert nearest <= 0.3 or min(target.distance(p) for p in points) <= 0.3


def test_limits_max_targets_and_max_leg():
    grid = NavGrid()
    start = T(7.0, 4.0)
    targets = [T(7.5 + 0.2 * i, 2.5 + 0.4 * i) for i in range(8)]
    assert len(order_targets(start, targets, grid, max_targets=3, swath_half_width=0.01)) == 3
    assert order_targets(start, [T(8.5, 4.0)], grid, max_leg=1.0) == []
    assert order_targets(start, [], grid) == []


def test_waypoint_headings_are_the_direction_of_travel():
    start = T(1.0, 1.0)
    poses = waypoint_poses(start, [T(2.0, 1.0), T(2.0, 2.0)])
    assert [p.translation() for p in poses] == [start, T(2.0, 1.0), T(2.0, 2.0)]
    assert abs(poses[0].rotation().degrees() - 0) < 1e-6      # along the first leg
    assert abs(poses[1].rotation().degrees() - 45) < 1e-6     # parallel to start -> last
    assert abs(poses[2].rotation().degrees() - 90) < 1e-6     # along the last leg
    assert math.isfinite(poses[1].rotation().radians())
