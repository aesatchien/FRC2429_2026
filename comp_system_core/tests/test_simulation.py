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
