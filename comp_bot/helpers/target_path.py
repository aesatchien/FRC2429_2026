"""
Pick an order to drive through a list of floor targets (game pieces), staying out of the
places the robot cannot go.

This is plain geometry - no robot, no NetworkTables, no PathPlanner - so it can be tested on
its own (tests/test_target_path.py) and could run on a coprocessor unchanged.
commands/drive_through_targets.py turns the result into a PathPlanner path and follows it.

WHERE THE ROBOT CANNOT GO
-------------------------
deploy/pathplanner/navgrid.json, the same file PathPlanner's own pathfinder (LocalADStar)
reads.  It is a grid of 0.3 m cells, True = the robot's CENTRE may not be there.  Edit it in
the PathPlanner GUI (Navigation Grid tab); nothing here needs to change when you do.
Anything off the grid counts as blocked.

WHAT THE ORDERING DOES
----------------------
1. throw away targets that sit in a blocked cell;
2. nearest-neighbour: from where we are, go to the closest target we can reach in a
   straight line that stays clear, and repeat from there;
3. targets that the intake will sweep up anyway on the way to another one (within
   swath_half_width of a leg already chosen) are not given their own stop;
4. 2-opt: reverse stretches of the route while that makes it shorter and keeps it clear.
It is a heuristic, not the optimal tour.  For the eight or so stops a path can usefully
hold it runs in well under a millisecond.
"""

import json
import math
from pathlib import Path

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

k_navgrid_file = Path(__file__).resolve().parent.parent / 'deploy' / 'pathplanner' / 'navgrid.json'


class NavGrid:
    """deploy/pathplanner/navgrid.json as something you can ask 'may the robot be here?'"""

    def __init__(self, path=None):
        with open(path or k_navgrid_file, 'r') as f:
            data = json.load(f)
        self.node_size = float(data['nodeSizeMeters'])
        self.grid = data['grid']            # grid[row][col]: row is y, col is x, True = blocked
        self.rows = len(self.grid)
        self.cols = len(self.grid[0])

    def is_blocked(self, x: float, y: float) -> bool:
        col = math.floor(x / self.node_size)
        row = math.floor(y / self.node_size)
        if not (0 <= col < self.cols and 0 <= row < self.rows):
            return True   # off the field
        return bool(self.grid[row][col])

    def segment_clear(self, a: Translation2d, b: Translation2d) -> bool:
        """True if the straight line a -> b touches no blocked cell (sampled every quarter cell)."""
        steps = max(1, math.ceil(a.distance(b) / (self.node_size / 4)))
        for i in range(steps + 1):
            t = i / steps
            if self.is_blocked(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t):
                return False
        return True

    def points_clear(self, points) -> bool:
        """True if none of these Translation2d are in a blocked cell - for checking a finished path."""
        return not any(self.is_blocked(p.x, p.y) for p in points)


def _distance_to_segment(p: Translation2d, a: Translation2d, b: Translation2d) -> float:
    dx, dy = b.x - a.x, b.y - a.y
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return p.distance(a)
    t = max(0.0, min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / length_sq))
    return math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy))


def _route_length(start: Translation2d, route: list) -> float:
    total, here = 0.0, start
    for stop in route:
        total += here.distance(stop)
        here = stop
    return total


def _route_clear(start: Translation2d, route: list, grid: NavGrid) -> bool:
    here = start
    for stop in route:
        if not grid.segment_clear(here, stop):
            return False
        here = stop
    return True


def order_targets(start: Translation2d, targets: list, grid: NavGrid, max_targets: int = 8,
                  max_leg: float = 4.0, swath_half_width: float = 0.3) -> list:
    """The stops to drive through, in order.  Empty if nothing can be reached.

    start             where the robot is now
    targets           Translation2d of every target seen, field coordinates
    max_targets       most stops in one path
    max_leg           longest single hop between stops, metres
    swath_half_width  a target this close to a leg is collected in passing and gets no stop
    """
    remaining = [t for t in targets if not grid.is_blocked(t.x, t.y)]
    route, here = [], start
    swept = []   # swept[i]: the targets collected on the leg INTO route[i], the stop included

    while remaining and len(route) < max_targets:
        reachable = [t for t in remaining if here.distance(t) <= max_leg and grid.segment_clear(here, t)]
        if not reachable:
            break
        nxt = min(reachable, key=lambda t: here.distance(t))
        # everything the intake passes over on the way there is done too, the stop included
        on_leg = [t for t in remaining if _distance_to_segment(t, here, nxt) <= swath_half_width]
        remaining = [t for t in remaining if _distance_to_segment(t, here, nxt) > swath_half_width]
        # A row of targets: the last stop was only on the way to this one.  If one straight
        # leg from the stop before it collects everything both legs would have, drop the stop
        # in the middle - it is a longer leg, not another stop, and does not use up max_targets.
        if route:
            before = route[-2] if len(route) > 1 else start
            both = swept[-1] + on_leg
            if (before.distance(nxt) <= max_leg and grid.segment_clear(before, nxt)
                    and all(_distance_to_segment(t, before, nxt) <= swath_half_width for t in both)):
                route.pop()
                swept.pop()
                on_leg = both
        route.append(nxt)
        swept.append(on_leg)
        here = nxt

    # 2-opt on an open route with a fixed start: reversing route[i..j] changes two legs
    improved = True
    while improved and len(route) > 2:
        improved = False
        for i in range(len(route) - 1):
            for j in range(i + 1, len(route)):
                candidate = route[:i] + route[i:j + 1][::-1] + route[j + 1:]
                if (_route_length(start, candidate) < _route_length(start, route) - 1e-6
                        and _route_clear(start, candidate, grid)):
                    route, improved = candidate, True
    return route


def waypoint_poses(start: Translation2d, route: list) -> list:
    """The start plus each stop as a Pose2d whose rotation is the DIRECTION OF TRAVEL there.

    That is what PathPlannerPath.waypointsFromPoses() wants: the rotation is the heading of
    the path through the point, not which way the robot faces.  Through a middle stop the
    path runs parallel to the line from the stop before to the stop after, which gives a
    smooth curve; at the two ends it points along the first and last leg.
    """
    points = [start] + list(route)
    poses = []
    for i, p in enumerate(points):
        before = points[max(i - 1, 0)]
        after = points[min(i + 1, len(points) - 1)]
        heading = math.atan2(after.y - before.y, after.x - before.x)
        poses.append(Pose2d(p, Rotation2d(heading)))
    return poses
