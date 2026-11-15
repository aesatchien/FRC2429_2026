import math
import time
import typing
import commands2
import ntcore
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, GoalEndState, RotationTarget

import constants
from constants import AutoConstants as cac
from subsystems.swerve_constants import AutoConstantsSwerve as ac
from subsystems.swerve import Swerve
from subsystems.led import Led
from helpers.log_command import log_command
from helpers.target_path import NavGrid, order_targets, waypoint_poses


def build_target_path(start: Pose2d, targets: list, grid: NavGrid):
    """One smooth PathPlanner path from `start` through as many of `targets` as it can reach.

    Returns (path, stops), or (None, []) if there is nothing to drive to.

    helpers/target_path.py picks the stops and their order from straight lines.  The path
    PathPlanner then draws through them is a curve, and a curve can bulge into a blocked cell
    that every straight leg missed - so the finished path is checked point by point against
    the same grid, and the LAST stop is dropped until it is clean.  A path with one stop is a
    straight line, which was already checked, so this always ends.
    """
    here = start.translation()
    targets = [t for t in targets if here.distance(t) >= cac.k_target_path_min_leg_m]
    stops = order_targets(here, targets, grid, max_targets=cac.k_target_path_max_targets,
                          max_leg=cac.k_target_path_max_leg_m,
                          swath_half_width=cac.k_target_path_swath_half_width_m)
    intake = Rotation2d.fromDegrees(cac.k_target_path_intake_heading_deg)

    while stops:
        poses = waypoint_poses(here, stops)   # rotation = direction of travel, not robot facing
        # the robot turns to lead with its intake: by each stop it faces along the path there
        facing = [pose.rotation() + intake for pose in poses]
        path = PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(poses),
            ac.k_target_path_constraints,
            None,   # no ideal starting state: the trajectory is generated from how we are moving now
            GoalEndState(0.0, facing[-1]),
            holonomic_rotations=[RotationTarget(i, facing[i]) for i in range(1, len(poses) - 1)],
        )
        path.preventFlipping = True   # these are real field coordinates, never mirror for red
        path.name = 'DriveThroughTargets'
        if grid.points_clear([point.position for point in path.getAllPathPoints()]):
            return path, stops
        stops = stops[:-1]
    return None, []


@log_command(console=True, nt=False, print_init=True, print_end=True)
class DriveThroughTargets(commands2.Command):
    """
    Drive one smooth path through every target the supplier reports, e.g. all the game
    pieces the cameras see (Vision.get_field_targets).  Run it alongside the intake.

    The path is planned ONCE, when the command starts, from where the targets are at that
    moment; it is not re-planned on the way.  Where the robot may not go comes from
    deploy/pathplanner/navgrid.json (see helpers/target_path.py).  The following itself is
    PathPlanner's - the same follower and gains every .path file in the autos uses - this
    command only builds the path and hands it over.

    With no reachable target it ends at once and flashes failure.  It ends by itself at the
    last stop, with the robot stopped; interrupted, it leaves the robot to whatever takes over.
    """

    def __init__(self, container, swerve: Swerve,
                 targets_supplier: typing.Callable[[], typing.List[Translation2d]], indent=0) -> None:
        super().__init__()
        self.setName('DriveThroughTargets')
        self.indent = indent
        self.container = container
        self.swerve = swerve
        self.targets_supplier = targets_supplier
        self.addRequirements(self.swerve)

        self.grid = NavGrid()
        self.follower = None    # PathPlanner's FollowPathCommand for the current path
        self.abort = False
        self.stops = []         # where the current path goes, in order (also read by the tests)
        self.path = None
        # the path being driven, for AdvantageScope's field view; empty when not running
        self.path_pub = ntcore.NetworkTableInstance.getDefault().getStructArrayTopic(
            f"{constants.auto_prefix}/target_path", Pose2d).publish()
        self.path_pub.set([])

    def initialize(self) -> None:
        self.follower, self.path, self.stops = None, None, []
        started = time.perf_counter()
        targets = list(self.targets_supplier() or [])
        self.path, self.stops = build_target_path(self.swerve.get_pose(), targets, self.grid)
        self.abort = self.path is None
        if self.abort:
            self.extra_log_info = f'no reachable target among {len(targets)} seen'
            return

        self.follower = AutoBuilder.followPath(self.path)
        self.follower.initialize()   # generates the trajectory - the slow part, timed below
        planning_ms = (time.perf_counter() - started) * 1000
        self.path_pub.set(self.path.getPathPoses())
        self.container.led.set_indicator(Led.Indicator.kPOLKA)
        stops = ' '.join(f'({s.x:.1f},{s.y:.1f})' for s in self.stops)
        self.extra_log_info = f'{len(self.stops)} stops of {len(targets)} seen, planned in {planning_ms:.0f} ms: {stops}'

    def execute(self) -> None:
        if self.follower is not None:
            self.follower.execute()

    def isFinished(self) -> bool:
        return self.abort or self.follower.isFinished()

    def end(self, interrupted: bool) -> None:
        if self.follower is not None:
            self.follower.end(interrupted)
            self.follower = None
        self.path_pub.set([])
        indicator = Led.Indicator.kFAILUREFLASH if (interrupted or self.abort) else Led.Indicator.kSUCCESSFLASH
        commands2.CommandScheduler.getInstance().schedule(self.container.led.set_indicator_with_timeout(indicator, 2))
