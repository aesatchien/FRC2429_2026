#!/usr/bin/env python3

import typing
import wpilib

# MUST come before anything that imports phoenix6 (motors.py -> TalonFX).  phoenix6
# 26.50.0a1 is still written against the camelCase WPILib API that a7 renamed.
from helpers import phoenix6_compat  # noqa: F401  (import for side effect)
import commands2
import constants
from constants import IntakeConstants as ic
from wpimath.units import inches_to_meters

from helpers import dashboard, log_command
from robotcontainer import RobotContainer
from subsystems.led import Led  # allows indexing of LED colors
from simulation.blockhead_mech import BlockheadMech

# 2027a7: register the telemetry/tunable backends BEFORE anything publishes.  Without this
# every log() goes to a DiscardTelemetryBackend - it does not raise, it just throws the value
# away after one "no backend for path" warning, and the whole dashboard comes up empty.
# RobotContainer publishes in its constructor, so this has to happen at import time.
dashboard.install()

# Kept muted: with controllers unplugged this warns every loop for every input and buries
# everything else in the console.
#
# But know what you are giving up.  This ALSO hides "Joystick POV 0 on port 5 not available"
# and the Button/Axis equivalents - the only signal WPILib gives that the DS is not reporting
# an input your code reads.  That is exactly how the dead D-pad went unnoticed.
# constants.k_debug_gamepads prints the same facts once a second instead (see
# report_gamepads below), which is the readable way to get them.
#
# 2027a7 renamed this warning -> alert: silenceJoystickConnectionWarning is gone and the
# replacement is silence_joystick_connection_alert.
wpilib.DriverStationBackend.silence_joystick_connection_alert(True)


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None
    def __init__(self) -> None:
        """
        Startup code.  WPILib 2027 REMOVED robotInit() - initialization belongs in the
        constructor now.  This is a silent failure mode if you miss it: the old robotInit()
        simply never gets called, nothing complains, and the program dies later on the first
        attribute it was supposed to have created.  That is exactly how this was found.
        """
        super().__init__()
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()
        self.alliance_zone = None
        self.mech = BlockheadMech()


    def disabled_init(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        self.disabled_counter = 1
        # self.container.swerve.use_photoncam = True

    def disabled_periodic(self) -> None:
        """This function is called periodically when disabled"""
        if constants.k_debug_gamepads and self.disabled_counter % 50 == 0:
            self.report_gamepads()

        if self.disabled_counter % 100 == 0:
            # set the LEDs
            # if wpilib.RobotState.isFMSAttached():
            alliance_color = wpilib.MatchState.get_alliance()
            is_connected = wpilib.RobotState.is_fms_attached() or wpilib.RobotState.is_ds_attached()
            # print(f'color is {alliance_color}, connected is {is_connected}')
            if alliance_color is not None and is_connected:
                if alliance_color == wpilib.Alliance.RED:
                    self.container.led.set_indicator(Led.Indicator.kHOTBOW)
                    self.alliance_zone = "Red"
                elif alliance_color == wpilib.Alliance.BLUE:
                    self.container.led.set_indicator(Led.Indicator.kCOOLBOW)
                    self.alliance_zone = "Blue"
            else:
                self.container.led.set_indicator(Led.Indicator.kPOLKA)
                self.alliance_zone = None

        if self.disabled_counter % 500 == 0:
            # check on the questnav - auto synch it if we have been up more than 10s and have not synched yet
            # but attempt to see if we have a good starting tag (logitech reef)
            # TODO: make this robust - arducam right is index 0, not 1
            if ( wpilib.Timer.get_timestamp() > 10 and
                    not self.container.questnav.quest_has_synched and
                    (self.container.swerve.count_subscribers[0]).get() > 0):
                self.container.swerve.questnav.quest_sync_odometry()  # this will mark that we have synched
            if wpilib.RobotBase.is_real() or wpilib.RobotBase.is_simulation():  # redundant to show we covered both cases
                pass
                # print(f"quest sync:{self.container.questnav.quest_has_synched} logitech tag count is:{self.container.swerve.count_subscribers[3].get()} at {self.container.timer.get():.1f}s")
        self.disabled_counter += 1

    def autonomous_init(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""

        # Reset the timer used by the @log_command decorator.
        # This ensures that command logs start at 0.0s for the beginning of Autonomous.
        log_command.reset()

        self.autonomousCommand = self.container.get_autonomous_command()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomous_periodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleop_init(self) -> None:

        # Reset the timer used by the @log_command decorator.
        # This ensures that command logs start at 0.0s for the beginning of Teleop.
        log_command.reset()
        # self.container.swerve.use_photoncam = False

        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
            
        # Ensure the actual selected auto command is also canceled,
        # in case it was scheduled independently by our auto delay wrapper!
        # NOTE this is the command get_autonomous_command() actually handed out, NOT
        # auto_chooser.getSelected().  Touching the chooser between auto and teleop - which
        # happens constantly in practice - used to cancel the newly selected command while
        # the one still running carried on driving into teleop.
        if self.container.scheduled_auto_command is not None:
            self.container.scheduled_auto_command.cancel()

        self.container.targeting.stop_tracking()  # make absolutely sure that tracking is exited in teleop
            
        self.stationary_counter = 0

    def teleop_periodic(self) -> None:
        """This function is called periodically during operator control"""

        # Auto-resync QuestNav if it drops during teleop (e.g., from a passthrough bump)
        # TODO - this really should only be enabled if the quest went into passthru and recovered - should be a callback?
        try_resync = (constants.QuestConstants.k_allow_quest_auto_resync and  # constants say it's ok to try
                      self.container.questnav.use_quest and   # quest has not been disabled on the dashboard
                      not self.container.questnav.quest_has_synched  # we're not already synced
                      and self.container.questnav.is_quest_connected())  # we're actually connected
        if try_resync:
            speeds = self.container.swerve.get_relative_speeds()
            is_stationary = abs(speeds.vx) < 0.1 and abs(speeds.vy) < 0.1 and abs(speeds.omega) < 0.1
            
            seeing_tag = any(sub.get() > 0 for sub in self.container.swerve.count_subscribers)
            
            if is_stationary and seeing_tag:
                self.stationary_counter += 1
            else:
                self.stationary_counter = 0
                
            # Wait 0.5 seconds (25 loops) of being stationary with a tag in view to let the pose settle
            if self.stationary_counter > 25:  
                print(f"*** Auto-resyncing QuestNav in Teleop at {wpilib.Timer.get_timestamp():.1f}s ***")
                self.container.questnav.quest_sync_odometry()
                self.stationary_counter = 0  # reset so we don't spam if headset isn't fully awake yet

    def report_gamepads(self) -> None:
        """What the Driver Station ACTUALLY reports for each controller.

        WPILib reads of an input the DS does not report do not raise - they return a falsy
        default forever (getPOV() -> POVDirection.CENTER, getRawButton() -> False).  So a
        control that "does nothing" looks identical to a control you never bound.  This
        prints the DS's own view so you can tell the difference.

        The line that matters for the D-pad is POVs=0: SystemCore reports no POV hat at all,
        which is why anything built on pov()/povUp() is dead on the robot even though it
        works in simulation, where the sim has to be told the hat exists.  The D-pad comes
        in as buttons instead - see the block at the top of helpers/joysticks.py.
        """
        from wpilib import DriverStationBackend as dsb
        from helpers import joysticks as js

        # a7: Command*Controller.get_hid() returns the CommandGenericHID wrapper, which has no
        # get_dpad_*_button().  get_controller() is the wpilib XboxController/DualSenseController.
        # This raised AttributeError in disabled_periodic as soon as a pad was plugged in -
        # only found by the simulation test, because with no DS attached the connected check
        # above skips the line that blows up.
        for label, port, pad in (
                ("driver ", constants.k_driver_controller_port, js.driver_controller.get_controller()),
                ("copilot", constants.k_co_driver_controller_port, js.copilot_controller.get_controller()),
                ("ps     ", constants.k_ps5_controller_port, js.play_station_controller.get_controller())):
            if not dsb.is_joystick_connected(port):
                print(f"[pads] {label} port {port}: NOT CONNECTED")
                continue
            # getStick*Available() return a BITMASK of which indices the DS reports, not a
            # count - 6 axes reads as 63 (0b111111).  Popcount them or the numbers lie.
            povs = dsb.get_stick_povs_available(port)
            buttons = dsb.get_stick_buttons_available(port)
            axes = dsb.get_stick_axes_available(port)
            print(f"[pads] {label} port {port}: name={dsb.get_joystick_name(port)!r} "
                  f"gamepad={dsb.get_joystick_is_gamepad(port)} type={dsb.get_joystick_gamepad_type(port)} "
                  f"| POVs={bin(povs).count('1')} "
                  f"buttons={bin(buttons).count('1')} (mask {buttons:#x}) "
                  f"axes={bin(axes).count('1')} "
                  f"| dpad U{int(pad.get_dpad_up_button())} D{int(pad.get_dpad_down_button())} "
                  f"L{int(pad.get_dpad_left_button())} R{int(pad.get_dpad_right_button())}")

    # ----------------------------- simulation -----------------------------
    # Simulation lives INSIDE the subsystems - the WPILib / Java model.  Each subsystem
    # overrides Subsystem.simulation_periodic(), which CommandScheduler.run() calls right
    # after periodic() whenever RobotBase.is_simulation(); it advances a plant model and
    # writes the result into its own motor controllers' and sensors' sim state, so the
    # robot code reads simulated motion through exactly the calls it uses on the real
    # robot.  See docs/sim_migration_plan.md for the whole story and where each piece went.
    #
    # The two things that are not mechanisms live here.  Both hooks are called by
    # IterativeRobotBase - simulation_init() once at startup, simulation_periodic() every
    # loop after robot_periodic() - and neither runs on the real robot, so no guard needed.
    def simulation_init(self) -> None:
        from simulation.hil_snap import HardwareInTheLoop
        from simulation.ghost_robot import GhostRobot
        self.hil = HardwareInTheLoop(self.container)   # snap ground truth to real cameras / Quest
        self.ghost = GhostRobot()                      # draw the auto goal pose and shot line

    def simulation_periodic(self) -> None:
        self.hil.update()
        self.ghost.update()

    def utility_init(self) -> None:
        # 2027: test mode was renamed to utility mode - testInit/testPeriodic/testExit are
        # now utilityInit/utilityPeriodic/utilityExit.  Same silent failure as robotInit:
        # a leftover testInit() is simply never called.
        # Cancels all running commands at the start of utility mode
        commands2.CommandScheduler.get_instance().cancel_all()

    def robot_periodic(self) -> None:
        # commented out 2025 0305 CJH - this should never have been in here

        super().robot_periodic()

        # 2027a7: pump the tunables.  Without this, values only ever flow OUT to the
        # dashboard - the auto chooser selection and every dashboard command button would
        # be written by the driver station and silently never read back by the robot.
        dashboard.update()

        # Update Mechanism2d visualization (works on Real Robot and Sim)
        if self.mech:
            # Intake - the MEASURED arm angle.  This used to draw the profile setpoint, and
            # while disabled it nudged that setpoint by a degree a loop toward 90 so the
            # picture would move in sim; the deploy encoder is simulated now, so the same
            # measurement the real robot reads is the right thing to draw in both places.
            self.mech.update_intake(angle=self.container.intake.get_angle_deg(),
                                    rpm=self.container.intake.current_rpm if self.container.intake.intake_on else 0)

            # Shooter
            self.mech.update_hopper(self.container.shooter.current_hopper_rpm / 6000)

            self.mech.update_indexer(self.container.shooter.current_indexer_rpm if self.container.shooter.indexer_on else 0)
            self.mech.update_shooter(self.container.shooter.current_rpm if self.container.shooter.shooter_on else 0)
            self.mech.update_rollers(self.container.shooter.current_roller_rpm if self.container.shooter.roller_on else 0)

            # Climber & Ball
            if hasattr(self.container, 'climber'):
                self.mech.update_climber(height_from_ground=inches_to_meters(self.container.climber.get_pos()))
            self.mech.update_ball(inches_to_meters(45), inches_to_meters(2))


if __name__ == "__main__":
    wpilib.run(MyRobot)
