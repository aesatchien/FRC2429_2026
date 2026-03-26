#!/usr/bin/env python3

import typing
import wpilib
import commands2
import constants
from wpimath.units import inchesToMeters

from helpers import log_command
from robotcontainer import RobotContainer
from subsystems.led import Led  # allows indexing of LED colors
from simulation.blockhead_mech import BlockheadMech

wpilib.DriverStation.silenceJoystickConnectionWarning(True)


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()
        self.alliance_zone = None
        self.mech = BlockheadMech()


    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        self.disabled_counter = 1
        # self.container.swerve.use_photoncam = True

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        if self.disabled_counter % 100 == 0:
            # set the LEDs
            # if wpilib.DriverStation.isFMSAttached():
            #alliance_color = wpilib.DriverStation.getAlliance()
            alliance_color = wpilib.DriverStation.Alliance.kBlue
            if alliance_color is not None:
                if alliance_color == wpilib.DriverStation.Alliance.kRed:
                    self.container.led.set_indicator(Led.Indicator.kHOTBOW)
                    self.alliance_zone = "Red"
                elif alliance_color == wpilib.DriverStation.Alliance.kBlue:
                    self.container.led.set_indicator(Led.Indicator.kCOOLBOW)
                    self.alliance_zone = "Blue"
                else:
                    self.container.led.set_indicator(Led.Indicator.kPOLKA)
                    self.alliance_zone = None

        if self.disabled_counter % 500 == 0:
            # check on the questnav - auto synch it if we have been up more than 10s and have not synched yet
            # but attempt to see if we have a good starting tag (logitech reef)
            # TODO: make this robust - arducam right is index 0, not 1
            if ( wpilib.Timer.getFPGATimestamp() > 10 and
                    not self.container.questnav.quest_has_synched and
                    (self.container.swerve.count_subscribers[0]).get() > 0):
                self.container.swerve.questnav.quest_sync_odometry()  # this will mark that we have synched
            if wpilib.RobotBase.isReal() or wpilib.RobotBase.isSimulation():  # redundant to show we covered both cases
                pass
                # print(f"quest sync:{self.container.questnav.quest_has_synched} logitech tag count is:{self.container.swerve.count_subscribers[3].get()} at {self.container.timer.get():.1f}s")
        self.disabled_counter += 1

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""

        # Reset the timer used by the @log_command decorator.
        # This ensures that command logs start at 0.0s for the beginning of Autonomous.
        log_command.reset()

        self.autonomousCommand = self.container.get_autonomous_command()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:

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
            
        self.stationary_counter = 0

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

        # Auto-resync QuestNav if it drops during teleop (e.g., from a passthrough bump)
        # TODO - this really should only be enabled if the quest went into passthru and recovered - should be a callback?
        try_resync = (constants.k_allow_quest_auto_resync and  # constants say it's ok to try
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
                print(f"*** Auto-resyncing QuestNav in Teleop at {wpilib.Timer.getFPGATimestamp():.1f}s ***")
                self.container.questnav.quest_sync_odometry()
                self.stationary_counter = 0  # reset so we don't spam if headset isn't fully awake yet

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

    def robotPeriodic(self) -> None:
        # commented out 2025 0305 CJH - this should never have been in here
        # wpilib.SmartDashboard.putNumber("ajs turn commanded", self.container.driver_command_controller.getRightX())
        super().robotPeriodic()

        # Update Mechanism2d visualization (works on Real Robot and Sim)
        if self.mech:
            # Intake
            self.mech.update_intake(angle=self.container.intake.get_setpoint(),
                                    speed=1.0 if self.container.intake.intake_on else 0.0)

            # Shooter
            self.mech.update_hopper(self.container.shooter.current_hopper_rpm / 6000)
            if self.container.shooter.indexer_on:
                indexer_speed = 1 if self.container.shooter.get_indexer_rpm() > 0 else -1
            else:
                indexer_speed = 0
            self.mech.update_indexer(indexer_speed)
            self.mech.update_shooter(self.container.shooter.current_rpm if self.container.shooter.shooter_on else 0)

            # Climber & Ball
            if hasattr(self.container, 'climber'):
                self.mech.update_climber(height_from_ground=inchesToMeters(self.container.climber.get_pos()))
            self.mech.update_ball(inchesToMeters(45), inchesToMeters(2))


if __name__ == "__main__":
    wpilib.run(MyRobot)
