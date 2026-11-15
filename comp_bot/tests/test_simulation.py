"""
Can the simulated robot drive to a simulated vision target?  Runs the whole robot program
headless through pyfrc's test harness, physics.py included, with nothing mocked:

    simulation/vision_sim.py   sees a game piece from where the robot REALLY is and publishes
                               targets/distance/rotation/strafe on /Cameras/..., the same
                               topics the Raspberry Pis write
    subsystems/vision.py       reads them back, exactly as it would from a Pi
    the command                turns "2.6 m away, 4 deg right" into a field pose, drives there

The command is DriveToPoseCustomControl fed by Vision.get_vision_target_pose.  It is then
run the way the autos feed it (auto_reflect_pose), since that is what auto_shoot_and_pickup,
twocycle and through_trench_pickup run now that AutoToPoseClean is retired.

ONE test, for the reason tests/conftest.py gives: REV allows one robot per process.  This
file shares nothing with conftest's `container` fixture - `robotpy test` runs each test in
its own process, so the robot built here is the only one in it.

comp_system_core/tests/test_simulation.py has the same scenario for the SystemCore project.
"""

import commands2
import ntcore
from wpimath.geometry import Pose2d

import constants


def test_drive_to_simulated_vision_target(control, robot):
    # physics.py builds the camera sim in simulationInit, so this is in time
    constants.SimConstants.k_camera_source = 'sim'
    with control.run_robot():
        from commands.drive_to_pose_custom_control import DriveToPoseCustomControl
        from simulation.gamepiece_sim import GamepieceSim

        container = robot.container
        swerve, vision = container.swerve, container.vision
        camera = 'logi_front_hsv'
        scheduler = commands2.CommandScheduler.getInstance()
        # where the robot really is, published by physics.py
        truth_sub = ntcore.NetworkTableInstance.getDefault().getStructTopic(
            f"{constants.sim_prefix}/ground_truth", Pose2d).subscribe(Pose2d())
        piece_spots = [gp['pos'] for gp in GamepieceSim(robot_field_stub()).gamepieces]

        def teleop(seconds):
            control.step_timing(seconds=seconds, autonomous=False, enabled=True)

        def run(command, limit_s=10.0):
            scheduler.schedule(command)
            assert command.isScheduled(), f"{command.getName()} would not schedule - is the robot enabled?"
            for _ in range(int(limit_s / 0.02)):
                teleop(0.02)
                if not command.isScheduled():
                    return
            raise AssertionError(f"{command.getName()} never finished")

        def drive_to_vision_target(label, make_command):
            print(f"=== vision: {label} drives onto a simulated game piece ===")
            teleop(0.5)                                  # let the camera sim publish from here
            assert vision.target_available(camera), \
                "camera sim is publishing no targets - is SimConstants.k_camera_source 'sim'?"
            start = truth_sub.get()
            goal = vision.get_vision_target_pose(swerve.get_pose(), [camera])
            assert goal is not None
            assert min(goal.translation().distance(p) for p in piece_spots) < 0.05, \
                f"vision put the target at {goal}, which is not on a game piece"
            command = make_command()
            run(command)
            assert not command.abort, f"{label} aborted - it saw no target"
            truth = truth_sub.get()
            moved = truth.translation().distance(start.translation())
            off = truth.translation().distance(goal.translation())
            assert moved > 1.0, f"robot only moved {moved:.2f} m"
            assert off < 0.10, f"robot stopped {off:.2f} m from the target at {goal}"
            print(f"    drove {moved:.2f} m, stopped {off * 100:.1f} cm from the piece at "
                  f"({goal.X():.2f}, {goal.Y():.2f})")

        def creep(x, y, seconds):
            """Drive field-relative at a fraction of full speed, as a driver would."""
            run(commands2.cmd.run(
                lambda: swerve.drive(x, y, 0, fieldRelative=True, rate_limited=False, keep_angle=False),
                swerve).withTimeout(seconds))

        control.step_timing(seconds=1.0, autonomous=False, enabled=False)
        teleop(0.1)                                      # a command scheduled while disabled is dropped

        # the pieces are 4.7 m from the start pose and the camera sim sees 4 m: close in first
        creep(0.3, 0, 1.5)
        def vision_command():
            return DriveToPoseCustomControl(
                container, swerve, lambda: vision.get_vision_target_pose(swerve.get_pose(), [camera]))

        drive_to_vision_target('straight ahead', vision_command)

        # back off and across, then do it again from an angle
        creep(-0.3, 0.15, 1.5)
        drive_to_vision_target('off to one side', vision_command)

        # ------------------------------------------------------------------ a path through several targets
        # Same chain, but the camera sim's LIST of everything in view (field_positions)
        # instead of the nearest one, and PathPlanner following a path built on the spot.
        print("=== vision: DriveThroughTargets drives one path through the pieces the camera sees ===")
        from wpimath.geometry import Rotation2d, Translation2d
        from commands.drive_through_targets import DriveThroughTargets, build_target_path
        from helpers.target_path import NavGrid
        creep(-0.3, 0, 1.5)                              # back off so the camera has a field of pieces in view
        teleop(0.5)
        seen = vision.get_field_targets()
        assert len(seen) >= 3, f"the camera sim listed {len(seen)} targets - expected several"
        for target in seen:
            assert min(target.distance(p) for p in piece_spots) < 0.05, f"{target} is not on a game piece"
        assert len(seen) == len({(round(t.X(), 2), round(t.Y(), 2)) for t in seen}), "the same target is listed twice"

        command = DriveThroughTargets(container, swerve, vision.get_field_targets)
        scheduler.schedule(command)
        teleop(0.02)
        assert command.isScheduled() and not command.abort, "DriveThroughTargets found nothing to drive to"
        stops = list(command.stops)
        grid = NavGrid()
        assert len(stops) >= 2, f"only {len(stops)} stop planned from {len(seen)} targets"
        assert grid.points_clear([point.position for point in command.path.getAllPathPoints()])
        closest = [float('inf')] * len(stops)            # how near the robot really came to each stop
        for _ in range(1000):                            # 20 s
            teleop(0.02)
            here = truth_sub.get().translation()
            closest = [min(c, here.distance(stop)) for c, stop in zip(closest, stops)]
            if not command.isScheduled():
                break
        assert not command.isScheduled(), "DriveThroughTargets never finished"
        assert max(closest) < 0.25, f"missed a stop by {max(closest):.2f} m: {list(zip(stops, closest))}"
        off = truth_sub.get().translation().distance(stops[-1])
        assert off < 0.15, f"finished {off:.2f} m from the last stop {stops[-1]}"
        print(f"    {len(seen)} seen, {len(stops)} stops, passed within {max(closest) * 100:.0f} cm of every one, "
              f"finished {off * 100:.1f} cm from the last")

        # a target the robot may not reach is never a stop (off the field counts as blocked)
        outside = Translation2d(-1.0, 4.0)
        path, planned = build_target_path(Pose2d(7.0, 4.0, Rotation2d()),
                                          [outside, Translation2d(8.0, 3.0), Translation2d(8.6, 5.0)], grid)
        assert path is not None and outside not in planned and len(planned) == 2
        assert build_target_path(Pose2d(7.0, 4.0, Rotation2d()), [outside], grid) == (None, [])
        # nothing seen at all: the command ends at once instead of driving anywhere
        command = DriveThroughTargets(container, swerve, lambda: [])
        scheduler.schedule(command)
        teleop(0.1)
        assert command.abort and not command.isScheduled()

        print("=== auto: DriveToPoseCustomControl + auto_reflect_pose reaches the shooting pose ===")
        import wpilib
        from constants import AutoConstants
        from helpers.apriltag_utils import auto_reflect_pose
        goal = auto_reflect_pose(swerve.get_pose(), AutoConstants.k_shooting_pose,
                                 wpilib.DriverStation.getAlliance(), is_shooting=True)
        command = DriveToPoseCustomControl(
            container=container, swerve=swerve,
            target_pose_supplier=lambda: auto_reflect_pose(
                swerve.get_pose(), AutoConstants.k_shooting_pose, wpilib.DriverStation.getAlliance(), is_shooting=True),
            tolerance_type='exact')
        run(command, limit_s=15.0)
        truth = truth_sub.get()
        off = truth.translation().distance(goal.translation())
        turn_off = abs((truth.rotation() - goal.rotation()).degrees())
        assert off < 0.10 and turn_off < 5, f"stopped {off:.2f} m and {turn_off:.1f} deg from {goal}"
        print(f"    stopped {off * 100:.1f} cm and {turn_off:.1f} deg from the shooting pose")


def robot_field_stub():
    """GamepieceSim only needs somewhere to draw; this copy is just read for its piece list."""
    import wpilib
    return wpilib.Field2d()
