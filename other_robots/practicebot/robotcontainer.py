# 2429 FRC code for 2025 season - Reefscape

import wpilib
from wpimath.geometry import Pose2d
import commands2
from commands2.printcommand import PrintCommand

# pathplanner stuff
from pathplannerlib.pathfinders import LocalADStar
from pathplannerlib.pathfinding import Pathfinding
from pathplannerlib.auto import NamedCommands

# 2429 helper files
import constants
from helpers import joysticks as js

# 2429 subsystems
from subsystems.led import Led
from subsystems.quest import Questnav
from subsystems.robot_state import RobotState
from subsystems.swerve import Swerve
from subsystems.vision import Vision
from subsystems.shooter import Shooter
from subsystems.intake import Intake
from subsystems.targeting import Targeting

# from subsystems.questnav_2429 import QuestnavModule

# 2429 "auto" commands - just an organizational division of commands

# 2429 commands
from commands.auto_to_pose_clean import AutoToPoseClean
from commands.auto_track_vision_target import AutoTrackVisionTarget
from commands.can_status import CANStatus
from commands.drive_by_distance_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.reset_field_centric import ResetFieldCentric
from commands.set_leds import SetLEDs
from commands.sim_show_fov import SimShowFOV
from commands.move_training_box import MoveTrainingBox

from commands.shooting_command import ShootingCommand
from commands.intake_set import Intake_Set


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
    little robot logic should actually be handled in the :class:`.Robot` periodic methods (other than the scheduler
    calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # ----------  SUBSYSTEMS  ---------------
        # The robot's subsystems
        self.targeting = Targeting()  # pure math for getting rotations values for target tracking
        self.questnav = Questnav()  # going to break the silo convention and let the Swerve see the quest for now
        self.swerve = Swerve(questnav=self.questnav)
        self.vision = Vision()
        self.shooter = Shooter()
        self.intake = Intake()
        self.robot_state = RobotState()  # currently has a callback that LED can register
        self.led = Led(robot_state=self.robot_state)  # may want LED last because it may want to know about other systems

        # ----------  CONTROLLERS & DEFAULTS  ---------------
        self.bind_driver_buttons()
        # self.bind_codriver_buttons()  # if we need to

        self.swerve.setDefaultCommand(DriveByJoystickSubsystemTargeting(
            container=self,
            swerve=self.swerve,
            targeting=self.targeting,
            controller=js.driver_controller,
        ))

        if not constants.k_swerve_only:
            pass

        # ----------  DASHBOARD & PATHPLANNER  ---------------
        self.register_commands()

        self.initialize_dashboard()

        Pathfinding.setPathfinder(LocalADStar())

    def bind_driver_buttons(self):
        # ----------  DRIVER BUTTONS  ---------------
        print("Binding driver buttons")

        # --- Drive & Navigation ---
        js.driver_y.onTrue(ResetFieldCentric(container=self, swerve=self.swerve, angle=0))

        # D-Pad: Slow, smooth robot-centric alignment (Nudge)
        dpad_driving = False
        if dpad_driving:
            dpad_output = 0.15
            js.driver_up.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(dpad_output, 0, 0), timeout=10))
            js.driver_down.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(-dpad_output, 0, 0), timeout=10))
            js.driver_left.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, dpad_output, 0), timeout=10))
            js.driver_right.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, -dpad_output, 0), timeout=10))
        else:
            js.driver_up.whileTrue(ShootingCommand(shooter=self.shooter))
            js.driver_right.whileTrue(Intake_Set(intake=self.intake, rpm=1000))
            js.driver_left.whileTrue(Intake_Set(intake=self.intake, rpm=0))

        # --- Subsystems ---
        js.driver_start.whileTrue(Intake_Set(intake=self.intake, rpm=3000))
        js.driver_back.whileTrue(Intake_Set(intake=self.intake, rpm=0))

        # --- Vision & Automation ---
        # Align to Pose (Front/Left)
        js.driver_a.debounce(0.1).whileTrue(AutoToPoseClean(self, self.swerve, target_pose=None, use_vision=True, cameras=['logi_front_hsv'], control_type='not_pathplanner'))
        js.driver_x.debounce(0.1).whileTrue(AutoToPoseClean(self, self.swerve, target_pose=None, use_vision=True, cameras=['logi_left_hsv'], control_type='not_pathplanner'))
        
        # Track Target
        js.driver_b.debounce(0.1).whileTrue(AutoTrackVisionTarget(self, camera_key='logi_front_hsv', target_distance=0.40))

        # --- Debug & Simulation ---
        js.driver_lb.whileTrue(SimShowFOV(self))
        js.driver_rb.onTrue(MoveTrainingBox(self))
        # js.driver_rb.whileTrue(SwerveTest(self, self.swerve))
        
        js.driver_l_trigger.onTrue(commands2.PrintCommand("Pushed L trigger"))
        js.driver_r_trigger.onTrue(commands2.PrintCommand("Pushed R trigger"))

        # --- Debug & Simulation ---
        # test a setting of the swerve modules straight before running the auto to tag
        # js.driver_a.whileTrue(commands2.cmd.run(lambda: self.swerve.set_straight(), self.swerve))
        #js.driver_a.whileTrue(TrackHub(self))
        #js.driver_a.whileTrue(ShootingCommand(container=self, shooter=self.shooter))


    def bind_codriver_buttons(self) -> None:
        # ----------  CO-DRIVER BUTTONS  ---------------
        print("Binding codriver buttons")

    def initialize_dashboard(self):
        # ----------  DASHBOARD COMMANDS  ---------------
        # --------------   COMMANDS FOR GUI (ROBOT DEBUGGING) - 20250224 CJH
        command_prefix = constants.command_prefix
        # --------------   TESTING LEDS ----------------
        self.led_mode_chooser = wpilib.SendableChooser()
        [self.led_mode_chooser.addOption(key, value) for key, value in self.led.modes_dict.items()]  # add all the indicators
        self.led_mode_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(SetLEDs(container=self, led=self.led, mode=selected_value)))
        wpilib.SmartDashboard.putData(f'{command_prefix}/LED Mode', self.led_mode_chooser)

        self.led_indicator_chooser = wpilib.SendableChooser()
        [self.led_indicator_chooser.addOption(key, value) for key, value in self.led.indicators_dict.items()]  # add all the indicators
        self.led_indicator_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
            SetLEDs(container=self, led=self.led, indicator=selected_value)))
        wpilib.SmartDashboard.putData(f'{command_prefix}/LED Indicator', self.led_indicator_chooser)

        # experimental, not used on dash
        wpilib.SmartDashboard.putData(f'{command_prefix}/SetSuccess', SetLEDs(container=self, led=self.led, indicator=Led.Indicator.kSUCCESS))

        # commands for pyqt dashboard - please do not remove
        COMMAND_LIST = [CANStatus(container=self), ]
        for cmd in COMMAND_LIST:
            wpilib.SmartDashboard.putData(f'{command_prefix}/{cmd.getName()}', cmd)
        #wpilib.SmartDashboard.putData(f'{command_prefix}/CANStatus', CANStatus(container=self))

        # You can and should use the exact same list of commands in the gui to watch for
        # These are left in to demonstrate a complete UI - the real one will be full of Commands (python classes), not strings
        FAKE_COMMAND_LIST = ['MoveElevatorTop', 'MoveElevatorUp', 'MoveElevatorDown', 'MovePivotUp', 'MovePivotDown',
            'MoveWristUp', 'MoveWristDown', 'IntakeOn', 'IntakeOff', 'IntakeReverse', 'MoveClimberDown',
            'MoveClimberUp', 'GoToStow', 'GoToL1', 'GoToL2', 'GoToL3', 'GoToL4', 'CanStatus', 'ResetFlex',
            'CalElevatorUp', 'CalElevatorDown', 'RecalWrist', 'CalWristUp', 'CalWristDown', 'GyroReset']
        for cmd in FAKE_COMMAND_LIST:
            # The `lambda cmd=cmd:` is a common Python technique called the "default argument hack".
            # Lambdas in loops have "late binding," meaning they capture the variable `cmd`, not its value.
            # Without `cmd=cmd`, EVERY button would print the LAST value of `cmd` ('GyroReset')
            # because that's what `cmd` is when the loop finishes.
            # By setting `cmd=cmd` as a default argument, we force the lambda to capture
            # the *current* value of `cmd` during each iteration of the loop.
            wpilib.SmartDashboard.putData(f'{command_prefix}/{cmd}', commands2.InstantCommand(lambda cmd=cmd: print(f'Called {cmd} at {wpilib.Timer.getFPGATimestamp():.1f}s'))
                                          .alongWith(commands2.WaitCommand(2)).ignoringDisable(True))

        # end pyqt dashboard section

        # quick way to test all scoring positions from dashboard
        self.score_test_chooser = wpilib.SendableChooser()
        [self.score_test_chooser.addOption(key, value) for key, value in self.robot_state.targets_dict.items()]  # add all the indicators
        self.score_test_chooser.onChange(
            # `setattr` is the programmatic way to set an attribute. It's equivalent to
            # `self.robot_state.target = selected_value`, but can be used inside a lambda.
            listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
                commands2.cmd.runOnce(lambda: setattr(self.robot_state, 'target', selected_value))))
        wpilib.SmartDashboard.putData(f'{command_prefix}/RobotScoringMode', self.score_test_chooser)

        # ----------  AUTONOMOUS CHOOSER SECTION  ---------------
        # self.auto_chooser = AutoBuilder.buildAutoChooser('')  # this loops through the path planner deploy directory - must exist 
        self.auto_chooser = wpilib.SendableChooser()  #  use this if you don't have any pathplanner autos defined
        self.auto_chooser.setDefaultOption('1:  Wait *CODE*', PrintCommand("** Running wait auto **").andThen(commands2.WaitCommand(15)))
        self.auto_chooser.addOption('2a: Drive 2s Straight *CODE*',
                                    PrintCommand("** Running drive by velocity swerve leave auto **").
                                    andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)))
        self.auto_chooser.addOption('2b: Drive 2s To Driver Station *CODE*',
                                    PrintCommand("** Running drive by velocity swerve leave auto **").
                                    andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2.5, field_relative=True)))
        wpilib.SmartDashboard.putData('autonomous routines', self.auto_chooser)  #

    def register_commands(self):
        # ----------  PATHPLANNER COMMANDS  ---------------
        # this is for PathPlanner, so it can call our commands.  Note they do not magically show up in pathplanner
        # you have to add them there, and then it remembers your list of commands.  so name them wisely
        NamedCommands.registerCommand('robot_state_left', commands2.cmd.runOnce(lambda: setattr(self.robot_state, 'side', RobotState.Side.LEFT)).ignoringDisable(True))

    def get_autonomous_command(self):
        # return DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)
        return self.auto_chooser.getSelected()
