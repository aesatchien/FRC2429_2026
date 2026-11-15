"""
Does the simulation actually simulate?  Runs the whole robot program headless through
WPILib's own test harness, drives it from a simulated Xbox controller and checks the values
the robot READS BACK - not the plant models - move the way they should.

WHY THIS IS ONE TEST
--------------------
REV's simulated Sparks live in a process-wide registry that the test harness's robot
teardown does not clear, so a second robot in the same process dies with "A SparkMax
instance has already been created with this device ID".  The isolated runner does not help:
its workers re-run every earlier test in the module before their own.  So this file builds
ONE robot and walks it through every scenario in order, printing a heading for each so a
failure still says which step it was.  It shares nothing with conftest.py's `container`
fixture for the same reason.

WHY THE ASSERTIONS LOOK THE WAY THEY DO
---------------------------------------
The 2027 model puts simulation inside each subsystem (Subsystem.simulation_periodic), and
the point of that is the robot code runs the identical path in sim and on the robot:
odometry integrates simulated encoders, the shooter's is_at_speed() reads a simulated
flywheel, the intake profile closes on a simulated arm.  So every assertion here is "the
robot's own view changed" - the only kind that can catch a sim which runs but never feeds the
robot.  See docs/sim_migration_plan.md section 9.
"""

import math

from wpilib.simulation import XboxControllerSim, DualSenseControllerSim, RoboRioSim
from wpimath import Pose2d, Rotation2d

import constants


def _driver_pad(port: int):
    """A simulated driver controller that the DS reports as present.

    robot.py's report_gamepads() explains the trap: WPILib reads of an input the DS does not
    report return a falsy default forever, so a controller that is not marked connected is
    indistinguishable from a driver who never touches the sticks.
    """
    # The same kind of pad helpers/joysticks.py built.  Axes 0-2 are the same on both, but
    # the buttons and the turbo trigger are not - an Xbox sim driving a PS5 config presses
    # the wrong buttons.
    ps5 = constants.k_controller_type == "PS5"
    pad = (DualSenseControllerSim if ps5 else XboxControllerSim)(port)
    # a7: these take a BITMASK of which axes/buttons exist, whatever the docstring's "count"
    # says - DriverStationBackend.get_stick_axes_available() documents it as a mask.  The old
    # set_axes_available(6) was 0b110: only axes 1 and 2 existed, so left X read 0 forever and
    # no button ever pressed.  This test only used axes 1-3, which is why it still passed.
    pad.set_axes_available((1 << 6) - 1)
    pad.set_buttons_available((1 << 16) - 1)
    pad.set_name("sim ps5" if ps5 else "sim xbox")
    pad.notify_new_data()
    return pad


def _set_turbo(pad, value: float) -> None:
    (pad.set_r2 if constants.k_controller_type == "PS5" else pad.set_right_trigger)(value)


def _set_track_button(pad, pressed: bool) -> None:
    """js.driver_rb - R1 on a DualSense, right bumper on an Xbox pad."""
    (pad.set_r1_button if constants.k_controller_type == "PS5" else pad.set_right_bumper_button)(pressed)


def test_simulation_end_to_end(control, robot):
    # the camera sim is built on the first simulation_periodic(), so this is in time
    constants.SimConstants.k_camera_source = 'sim'
    with control.run_robot():
        container = robot.container
        swerve, shooter, intake, robot_state = container.swerve, container.shooter, container.intake, container.robot_state
        pad = _driver_pad(constants.k_driver_controller_port)

        def teleop(seconds):
            control.step_timing(seconds=seconds, autonomous=False, enabled=True)

        # ------------------------------------------------------------------ disabled
        print("\n=== disabled: every simulation_periodic runs, nothing raises ===")
        control.step_timing(seconds=2.0, autonomous=False, enabled=False)
        truth = swerve.sim_get_ground_truth()
        assert abs(truth.x - constants.k_start_x) < 1e-6 and abs(truth.y - constants.k_start_y) < 1e-6
        assert 10.0 < RoboRioSim.get_vin_voltage() <= 12.5, "battery is not being simulated"
        assert robot_state._gamepiece_sim is not None, "RobotState.simulation_periodic never ran"
        assert container.vision._sim is not None, "Vision.simulation_periodic never ran"

        # ------------------------------------------------------------------ drive forward
        print("=== teleop: left stick forward ===")
        start_pose = swerve.get_pose()
        start_truth = swerve.sim_get_ground_truth()
        start_drive_m = [m.get_position().distance for m in swerve.swerve_modules]
        pad.set_left_y(-1.0)          # stick forward is negative Y on both pads
        _set_turbo(pad, 1.0)    # turbo, so we are not sitting on the slow-mode floor
        pad.notify_new_data()
        teleop(3.0)
        pad.set_left_y(0.0); _set_turbo(pad, 0.0); pad.notify_new_data()
        teleop(0.5)

        pose, truth = swerve.get_pose(), swerve.sim_get_ground_truth()
        for before, module in zip(start_drive_m, swerve.swerve_modules):
            after = module.get_position().distance
            assert after - before > 0.5, f"{module.label} drive encoder only moved {after - before:.3f} m"
        moved = pose.translation().distance(start_pose.translation())
        assert moved > 0.5, f"odometry moved only {moved:.3f} m in 3 s of full stick"
        assert pose.x - start_pose.x > 0.5, "should have gone +x (start heading is 0)"
        assert truth.translation().distance(start_truth.translation()) > 0.5, "ground truth did not move"
        # both integrate the same measured module states, so with noise-free encoders they
        # agree - neither is copied from the other, which is what the next steps rely on
        assert pose.translation().distance(truth.translation()) < 0.25
        print(f"    odometry {start_pose.x:.2f} -> {pose.x:.2f} m   truth -> {truth.x:.2f} m")

        # ------------------------------------------------------------------ rotate
        print("=== teleop: right stick, robot spins ===")
        start_heading = swerve.get_gyro_angle()
        pad.set_right_x(-1.0)   # the drive command reads -right_x as +rotation, so this is CCW
        pad.notify_new_data()
        teleop(2.0)
        pad.set_right_x(0.0); pad.notify_new_data()
        teleop(0.5)
        turned = ((swerve.get_gyro_angle() - start_heading) + 180) % 360 - 180
        assert turned > 20, f"IMU heading moved {turned:.1f} deg - the modules are not feeding the IMU sim"
        assert abs(((swerve.get_pose().rotation().degrees() - swerve.get_gyro_angle()) + 180) % 360 - 180) < 15
        print(f"    IMU turned {turned:.1f} deg CCW")

        # ------------------------------------------------------------------ tracking
        print("=== teleop: hold the track button, the nose swings onto the hub ===")
        targeting = container.targeting
        # point the nose well away from the hub first, so there is something to converge
        pad.set_right_x(-1.0); pad.notify_new_data()
        teleop(1.0)
        pad.set_right_x(0.0); pad.notify_new_data()
        teleop(0.5)
        _set_track_button(pad, True); pad.notify_new_data()
        teleop(0.02)
        assert targeting.get_tracking_state(), "track button did not start tracking"
        start_err = targeting.debug_error_deg
        assert abs(start_err) > 20, f"only {start_err:.1f} deg off the hub - nothing to converge"
        teleop(1.5)
        err = targeting.debug_error_deg
        assert abs(err) < 3, f"tracking error {err:.1f} deg after 1.5 s (started at {start_err:.1f})"
        # (The a7 optimize() bug - wheels swinging ~180 instead of reversing - does NOT fail
        # this: sim turn motors are near-instant, so tracking still converges.  That bug is
        # caught directly by test_swerve_optimize.py.)
        _set_track_button(pad, False); pad.notify_new_data()
        teleop(0.2)
        assert not targeting.get_tracking_state()
        print(f"    error {start_err:+.1f} -> {err:+.1f} deg")
        # the track button also fires up the flywheel (robotcontainer), and release leaves it
        # running - spin it down so the shooter step below starts from rest
        shooter.stop_shooter()
        teleop(3.0)

        # ------------------------------------------------------------------ azimuths
        print("=== module azimuths: set_x() must show up on the absolute encoders ===")
        for _ in range(75):                              # 1.5 s of set_x() each loop
            swerve.set_x()
            control.step_timing(seconds=0.02, autonomous=False, enabled=True)
        for module, target in zip(swerve.swerve_modules, [45, -45, -45, 45]):
            got = math.degrees(module.get_turn_encoder())
            err = (got - target + 180) % 360 - 180
            err = min(abs(err), abs((err + 180) % 360 - 180))   # X is also satisfied at target+180
            assert err < 8, f"{module.label} absolute encoder reads {got:.1f} deg, wanted {target}"
        print("    all four pots read the X angles")

        # ------------------------------------------------------------------ shooter
        print("=== shooter: spin-up is visible in the flywheel encoder ===")
        shooter.set_shooter_rpm(3000)
        teleop(0.2)
        early = shooter.get_velocity()
        teleop(3.0)
        late = shooter.get_velocity()
        assert late > early, "flywheel should still be accelerating 0.2 s after the command"
        assert abs(late - 3000) < constants.ShooterConstants.k_shooter_rpm_tolerance, f"flywheel settled at {late:.0f} rpm"
        assert shooter.is_at_speed()
        shooter.stop_shooter()
        teleop(3.0)
        assert abs(shooter.get_velocity()) < 300, "flywheel should have spun down"
        print(f"    {early:.0f} rpm at 0.2 s, {late:.0f} rpm at 3.2 s, stopped again")

        # ------------------------------------------------------------------ intake
        print("=== intake: the deploy profile closes on the simulated arm, in degrees ===")
        start = intake.get_angle_deg()
        target = constants.IntakeConstants.k_bottom_angle if start > 90 else constants.IntakeConstants.k_top_angle
        intake.set_intake_position(target)
        teleop(4.0)
        got = intake.get_angle_deg()
        assert abs(got - target) < 10, f"intake arm at {got:.1f} deg, wanted {target} (started {start:.1f})"
        print(f"    arm {start:.0f} -> {got:.1f} deg (target {target})")

        # ------------------------------------------------------------------ gamepieces
        print("=== gamepieces: teleport ground truth onto a piece, it gets consumed ===")
        sim = robot_state._gamepiece_sim
        before = sum(gp['active'] for gp in sim.gamepieces)
        # the drive above already ran over the pieces nearest the start pose, so pick a live one
        piece = next(gp for gp in sim.gamepieces if gp['active'])
        swerve.sim_set_ground_truth(Pose2d(piece['pos'], Rotation2d()))
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        after = sum(gp['active'] for gp in sim.gamepieces)
        assert after == before - 1, f"{before} pieces before, {after} after"
        print(f"    {before} -> {after} pieces on the floor")

        # ------------------------------------------------------------------ drive to a vision target
        # The whole chain, nothing mocked: the camera sim sees a game piece from GROUND TRUTH
        # and publishes distance/rotation on /Cameras/..., Vision reads them back exactly as
        # it would from a Pi, the command turns "1.9 m away, 4 deg left" into a field pose
        # and drives there.  Then the same command fed the way the autos feed it
        # (auto_reflect_pose), since that is what auto_shoot_and_pickup, twocycle and
        # through_trench_pickup run now that AutoToPoseClean is retired.
        import commands2
        from commands.drive_to_pose_custom_control import DriveToPoseCustomControl
        vision = container.vision
        camera = 'logi_front_hsv'
        piece_spots = [gp['pos'] for gp in sim.gamepieces]

        def drive_to_vision_target(label, start: Pose2d, make_command):
            print(f"=== vision: {label} drives onto a simulated game piece ===")
            # the robot and its odometry both start here, as they would after a pose reset
            swerve.sim_set_ground_truth(start)
            swerve.resetOdometry(start)
            teleop(0.5)                                  # let the camera sim publish from the new spot
            assert vision.target_available(camera), \
                "camera sim is publishing no targets - is SimConstants.k_camera_source 'sim'?"
            goal = vision.get_vision_target_pose(swerve.get_pose(), [camera])
            assert goal is not None
            assert min(goal.translation().distance(p) for p in piece_spots) < 0.05, \
                f"vision put the target at {goal}, which is not on a game piece"
            command = make_command()
            commands2.CommandScheduler.get_instance().schedule(command)
            for _ in range(500):                         # 10 s is far more than it needs
                teleop(0.02)
                if not command.is_scheduled():
                    break
            assert not command.is_scheduled(), f"{label} never finished"
            assert not command.abort, f"{label} aborted - it saw no target"
            truth = swerve.sim_get_ground_truth()
            moved = truth.translation().distance(start.translation())
            off = truth.translation().distance(goal.translation())
            assert moved > 1.0, f"robot only moved {moved:.2f} m"
            assert off < 0.10, f"robot stopped {off:.2f} m from the target at {goal}"
            print(f"    drove {moved:.2f} m, stopped {off * 100:.1f} cm from the piece at "
                  f"({goal.x:.2f}, {goal.y:.2f})")

        def vision_command():
            return DriveToPoseCustomControl(
                container, swerve, lambda: vision.get_vision_target_pose(swerve.get_pose(), [camera]))

        drive_to_vision_target('straight ahead', Pose2d(5.5, 4.1, Rotation2d()), vision_command)
        drive_to_vision_target('off to one side', Pose2d(5.5, 3.1, Rotation2d.from_degrees(10)), vision_command)

        # ------------------------------------------------------------------ a path through several targets
        # Same chain, but the camera sim's LIST of everything in view (field_positions)
        # instead of the nearest one, and PathPlanner following a path built on the spot.
        print("=== vision: DriveThroughTargets drives one path through the pieces the camera sees ===")
        from wpimath import Translation2d
        from commands.drive_through_targets import DriveThroughTargets, build_target_path
        from helpers.target_path import NavGrid
        sim.reset_gamepieces()
        start = Pose2d(5.5, 4.1, Rotation2d())
        swerve.sim_set_ground_truth(start)
        swerve.resetOdometry(start)
        teleop(0.5)
        seen = vision.get_field_targets()
        assert len(seen) >= 3, f"the camera sim listed {len(seen)} targets - expected a field full"
        for target in seen:
            assert min(target.distance(p) for p in piece_spots) < 0.05, f"{target} is not on a game piece"
        assert len(seen) == len({(round(t.x, 2), round(t.y, 2)) for t in seen}), "the same target is listed twice"

        before = sum(gp['active'] for gp in sim.gamepieces)
        command = DriveThroughTargets(container, swerve, vision.get_field_targets)
        commands2.CommandScheduler.get_instance().schedule(command)
        teleop(0.02)
        assert command.is_scheduled() and not command.abort, "DriveThroughTargets found nothing to drive to"
        stops = list(command.stops)
        grid = NavGrid()
        assert len(stops) >= 2, f"only {len(stops)} stop planned from {len(seen)} targets"
        assert grid.points_clear([point.position for point in command.path.getAllPathPoints()])
        for _ in range(1000):                            # 20 s
            teleop(0.02)
            if not command.is_scheduled():
                break
        assert not command.is_scheduled(), "DriveThroughTargets never finished"
        truth = swerve.sim_get_ground_truth()
        off = truth.translation().distance(stops[-1])
        assert off < 0.15, f"finished {off:.2f} m from the last stop {stops[-1]}"
        for stop in stops:
            assert not any(gp['active'] and gp['pos'].distance(stop) < 0.05 for gp in sim.gamepieces), \
                f"the piece at {stop} was a stop on the path and is still on the floor"
        after = sum(gp['active'] for gp in sim.gamepieces)
        assert before - after >= len(stops)
        print(f"    {len(seen)} seen, {len(stops)} stops, {before - after} pieces collected, "
              f"finished {off * 100:.1f} cm from the last stop")

        # a target in a blocked cell is never a stop, and the path stays out of blocked cells
        blocked_cell = next((c, r) for r in range(grid.rows) for c in range(grid.cols)
                            if grid.grid[r][c] and 6 < c * grid.node_size < 10 and 1 < r * grid.node_size < 7) \
            if any(grid.grid[r][c] and 6 < c * grid.node_size < 10 and 1 < r * grid.node_size < 7
                   for r in range(grid.rows) for c in range(grid.cols)) else None
        inside = (Translation2d((blocked_cell[0] + 0.5) * grid.node_size, (blocked_cell[1] + 0.5) * grid.node_size)
                  if blocked_cell else Translation2d(-1.0, 4.0))   # off the field counts as blocked too
        path, planned = build_target_path(Pose2d(7.0, 4.0, Rotation2d()),
                                          [inside, Translation2d(8.0, 3.0), Translation2d(8.6, 5.0)], grid)
        assert path is not None and inside not in planned and len(planned) == 2
        assert build_target_path(Pose2d(7.0, 4.0, Rotation2d()), [inside], grid) == (None, [])
        # nothing seen at all: the command ends at once instead of driving anywhere
        command = DriveThroughTargets(container, swerve, lambda: [])
        commands2.CommandScheduler.get_instance().schedule(command)
        teleop(0.1)
        assert command.abort and not command.is_scheduled()

        print("=== auto: DriveToPoseCustomControl + auto_reflect_pose reaches the shooting pose ===")
        import wpilib
        from constants import AutoConstants
        from helpers.apriltag_utils import auto_reflect_pose
        start = Pose2d(5.0, 2.0, Rotation2d())
        swerve.sim_set_ground_truth(start)
        swerve.resetOdometry(start)
        teleop(0.2)
        goal = auto_reflect_pose(swerve.get_pose(), AutoConstants.k_shooting_pose,
                                 wpilib.MatchState.get_alliance(), is_shooting=True)
        command = DriveToPoseCustomControl(
            container=container, swerve=swerve,
            target_pose_supplier=lambda: auto_reflect_pose(
                swerve.get_pose(), AutoConstants.k_shooting_pose, wpilib.MatchState.get_alliance(), is_shooting=True),
            tolerance_type='exact')
        commands2.CommandScheduler.get_instance().schedule(command)
        for _ in range(500):
            teleop(0.02)
            if not command.is_scheduled():
                break
        assert not command.is_scheduled(), "the drive to the shooting pose never finished"
        truth = swerve.sim_get_ground_truth()
        off = truth.translation().distance(goal.translation())
        turn_off = abs((truth.rotation() - goal.rotation()).degrees())
        assert off < 0.10 and turn_off < 5, f"stopped {off:.2f} m and {turn_off:.1f} deg from {goal}"
        print(f"    stopped {off * 100:.1f} cm and {turn_off:.1f} deg from the shooting pose")

        # ------------------------------------------------------------------ sensor fusion
        # Odometry in sim is perfect, so nothing disagrees until something is kicked
        # (SimConstants.k_kick).  Each step makes one thing wrong and checks who wins.
        from wpimath import Translation2d
        questnav = container.questnav

        def error_m():
            return swerve.get_pose().translation().distance(swerve.sim_get_ground_truth().translation())

        def idle(seconds):
            control.step_timing(seconds=seconds, autonomous=False, enabled=False)

        def place(pose):
            swerve.sim_set_ground_truth(pose)
            swerve.resetOdometry(pose)
            idle(0.5)                                    # let the camera sim publish from the new spot

        def tags_in_view():
            return any(sub.get() > 0 for sub in swerve.count_subscribers)

        print("=== fusion: robot kicked in view of a tag, the tag pulls the estimate back ===")
        # 2 m out from a tag and looking straight at it (arducam_right points where the robot does)
        facing_a_tag = None
        for tag in vision._sim.tag_poses:
            spot = Pose2d(tag.translation() + Translation2d(2.0, 0.0).rotate_by(tag.rotation()),
                          tag.rotation() + Rotation2d.from_degrees(180))
            if 1.5 < spot.x < constants.FieldConstants.k_field_length - 1.5 and 1.5 < spot.y < constants.FieldConstants.k_field_width - 1.5:
                facing_a_tag = spot
                break
        assert facing_a_tag is not None
        place(facing_a_tag)
        assert tags_in_view(), "the camera sim sees no tag from 2 m in front of one"
        assert error_m() < 0.02
        swerve.sim_kick(0.3, 0.2, 5.0)
        kicked = error_m()
        assert kicked > 0.3, f"the kick only moved the robot {kicked:.2f} m off its estimate"
        idle(2.0)
        assert error_m() < 0.05, f"2 s after the kick the estimate is still {error_m():.2f} m off, with a tag in view"
        print(f"    estimate {kicked:.2f} m off after the kick, {error_m() * 100:.1f} cm off 2 s later")

        print("=== fusion: robot kicked with no tag in view, the Quest pulls the estimate back ===")
        questnav.mock_questnav = True                    # the rest of this test ran with no Quest at all
        nowhere = None
        for heading in range(0, 360, 15):
            place(Pose2d(constants.FieldConstants.k_field_length / 2, constants.FieldConstants.k_field_width / 2,
                         Rotation2d.from_degrees(heading)))
            if not tags_in_view():
                nowhere = heading
                break
        assert nowhere is not None, "no heading at midfield without a tag in view"
        questnav.quest_sync_odometry()
        idle(0.2)
        assert questnav.quest_has_synched and questnav.is_pose_accepted()
        swerve.sim_kick(0.3, 0.2, 5.0)
        kicked = error_m()
        assert not tags_in_view(), "the kick brought a tag into view - this step is about the Quest alone"
        idle(5.0)
        assert error_m() < 0.05, f"5 s after the kick the estimate is still {error_m():.2f} m off, with the Quest fused"
        print(f"    estimate {kicked:.2f} m off after the kick, {error_m() * 100:.1f} cm off 5 s later")

        print("=== fusion: the QUEST is wrong and nothing else can see - the estimate follows it ===")
        # not what anyone wants, but what the robot does: with no tag in view a Quest that is a
        # little wrong is the only thing with an opinion.  This is the case to watch on the field widget.
        questnav.sim_kick_quest(0.4, 0.0, 0.0)
        idle(3.0)
        assert 0.2 < error_m() < 0.45, f"estimate is {error_m():.2f} m off after the Quest jumped 0.4 m"
        print(f"    Quest jumped 0.4 m, estimate followed it {error_m():.2f} m")

        print("=== fusion: a Quest that is wildly wrong is ignored ===")
        questnav.sim_kick_quest(6.0, 0.0, 0.0)           # Swerve only fuses a Quest within 4 m of the estimate
        before = swerve.get_pose()
        idle(3.0)
        moved = swerve.get_pose().translation().distance(before.translation())
        assert moved < 0.02, f"the estimate moved {moved:.2f} m toward a Quest that is 6 m off"
        print(f"    Quest jumped 6 m, estimate moved {moved * 100:.1f} cm")
        questnav.mock_questnav = False
