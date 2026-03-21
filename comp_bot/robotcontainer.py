# 2429 FRC code for 2025 season - Reefscape
from sys import prefix

import wpilib
from wpimath.geometry import Pose2d
import commands2
from commands2.printcommand import PrintCommand

# pathplanner stuff
from pathplannerlib.pathfinders import LocalADStar
from pathplannerlib.pathfinding import Pathfinding
from pathplannerlib.auto import AutoBuilder, NamedCommands

# 2429 helper files
import constants
from helpers import joysticks as js
from constants import ShooterConstants as sc

# 2429 subsystems
from subsystems import swerve_constants
from subsystems.led import Led
from subsystems.quest import Questnav
from subsystems.robot_state import RobotState
from subsystems.swerve import Swerve
from subsystems.vision import Vision
from subsystems.shooter import Shooter
from subsystems.intake import Intake
from subsystems.climber import Climber
from subsystems.targeting import Targeting

# from subsystems.questnav_2429 import QuestnavModule

# 2429 "auto" commands - just an organizational division of commands
from autonomous.autonomous_shooting import AutoShootingGroup
from autonomous.auto_shoot_and_pickup import AutoShootAndPickup
from autonomous.twocycle import TwoCycle
from autonomous.fill_shoot_fill import FillShootFill
from autonomous.fill_shoot_fill_shoot import FillShootFillShoot
from autonomous.fill_shoot_fill_shoot_trench import FillShootFillShootTrench
from autonomous.teleop_cycle import TeleopCycle

# 2429 commands
#from commands.auto_to_pose import AutoToPose
from commands.auto_to_pose_clean import AutoToPoseClean
from commands.auto_track_vision_target import AutoTrackVisionTarget
from commands.can_status import CANStatus
from commands.drive_by_velocity_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.reset_field_centric import ResetFieldCentric
from commands.rumble_command import RumbleCommand
from commands.set_leds import SetLEDs
from commands.sim_show_fov import SimShowFOV
from commands.move_training_box import MoveTrainingBox
from commands.swerve_test import SwerveTest
from commands.swerve_set_x import SwerveSetX
from commands.robot_climb import RobotClimb
from commands.increment_shooter import IncrementShooter
from commands.stop_shooter import StopShooter

from commands.shooting_command import ShootingCommand
from commands.intake_set_rpm import Intake_Set_RPM
from commands.intake_deploy import Intake_Deploy
from commands.intake_calibrate import CalibrateIntake


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
    little robot logic should actually be handled in the :class:`.Robot` periodic methods (other than the scheduler
    calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # ----------  SUBSYSTEMS  ---------------
        # The robot's subsystems
        self.questnav = Questnav()  # going to break the silo convention and let the Swerve see the quest for now
        self.swerve = Swerve(questnav=self.questnav)
        self.targeting = Targeting(swerve=self.swerve)
        self.vision = Vision()
        self.shooter = Shooter()
        self.intake = Intake()
        # self.climber = Climber()
        self.robot_state = RobotState()  # currently has a callback that LED can register
        self.led = Led(robot_state=self.robot_state)  # may want LED last because it may want to know about other systems

        # ----------  CONTROLLERS & DEFAULTS  ---------------
        self.bind_driver_buttons()
        self.bind_codriver_buttons()  # if we need to
        self.bind_bbox_buttons()

        self.swerve.setDefaultCommand(DriveByJoystickSubsystemTargeting(
              container=self,
              swerve=self.swerve,
              controller=js.driver_controller,
              targeting=self.targeting,
         ))

        if not constants.k_swerve_only:
            pass

        # ----------  DASHBOARD & PATHPLANNER  ---------------
        self.register_commands()

        self.initialize_dashboard()

        self.position_index = 0

        Pathfinding.setPathfinder(LocalADStar())

    def bind_driver_buttons(self):
        # ----------  DRIVER BUTTONS  ---------------
        print("Binding driver buttons")

        self.bind_codriver_buttons()

        # --- Drive & Navigation ---
        js.driver_y.onTrue(ResetFieldCentric(container=self, swerve=self.swerve, angle=0))

        # --- The current shooting cycle
        # slow intake rollers, start the shooting cycle, then raise the intake after a short wait
        js.driver_a.whileTrue(commands2.ParallelCommandGroup(
            ShootingCommand(shooter=self.shooter, targeting=self.targeting),
            commands2.SequentialCommandGroup(commands2.WaitCommand(constants.AutoConstants.k_intake_raise_delay),
                                             Intake_Deploy(intake=self.intake, position='shoot')),
        ).beforeStarting(Intake_Set_RPM(intake=self.intake, rpm=500, led=self.led)))
        # does not work as an "andThen" for some reason
        js.driver_a.onFalse(Intake_Deploy(intake=self.intake, position='down').andThen(Intake_Set_RPM(intake=self.intake, rpm=0, led=self.led)))

        # start / stop tracking
        js.driver_rb.onTrue(commands2.InstantCommand(lambda: self.targeting.start_tracking()))
        js.driver_rb.onFalse(commands2.InstantCommand(lambda: self.targeting.stop_tracking()))

        # D-Pad: Slow, smooth robot-centric alignment (Nudge)
        dpad_driving = False
        if dpad_driving:
            dpad_output = 0.15
            js.driver_up.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(dpad_output, 0, 0), timeout=10))
            js.driver_down.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(-dpad_output, 0, 0), timeout=10))
            js.driver_left.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, dpad_output, 0), timeout=10))
            js.driver_right.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, -dpad_output, 0), timeout=10))
        else:
            # js.driver_up.whileTrue(CalibrateIntake(intake=self.intake))
            js.driver_up.onTrue(Intake_Deploy(intake=self.intake, position='up'))
            #js.driver_right.whileTrue(IncrementShooter(shooter=self.shooter, speed_change=1))
            #js.driver_left.whileTrue(IncrementShooter(shooter=self.shooter, speed_change=-1))
            js.driver_down.onTrue(Intake_Deploy(intake=self.intake, position='down'))

        # --- Subsystems ---
        # Giving Jeremy faster and slower fixed speeds
        js.driver_lb.onTrue(Intake_Set_RPM(intake=self.intake, rpm=2500, led=self.led))
        js.driver_l_trigger.whileTrue(SwerveSetX(container=self, swerve=self.swerve))
        js.driver_back.onTrue(Intake_Set_RPM(intake=self.intake, rpm=0, led=self.led))

        js.driver_x.whileTrue(commands2.ParallelCommandGroup(
            ShootingCommand(shooter=self.shooter, rpm=3300),
            Intake_Deploy(intake=self.intake, position='shoot'),
        ).beforeStarting(Intake_Set_RPM(intake=self.intake, rpm=0, led=self.led)))
        js.driver_x.onFalse(Intake_Deploy(intake=self.intake, position='down'))

        js.driver_b.whileTrue(commands2.ParallelCommandGroup(
            ShootingCommand(shooter=self.shooter, rpm=4500),
            Intake_Deploy(intake=self.intake, position='shoot'),
        ).beforeStarting(Intake_Set_RPM(intake=self.intake, rpm=0, led=self.led)))
        js.driver_b.onFalse(Intake_Deploy(intake=self.intake, position='down'))

        js.driver_start.whileTrue(TeleopCycle(container=self))

        #js.bbox_intake_in.whileTrue(Intake_Set_RPM(intake=self.intake, rpm=3000))
        #js.bbox_intake_out.whileTrue(Intake_Set_RPM(intake=self.intake, rpm=0))
        #js.bbox_intake_up.onTrue(Intake_Deploy(intake=self.intake, direction='up'))
        #js.bbox_intake_down.onTrue(Intake_Deploy(intake=self.intake, direction='down'))

        #js.bbox_shoot.onTrue(CalibrateIntake(intake=self.intake))
        #js.bbox_shoot.whileTrue(ShootingCommand(shooter=self.shooter, targeting=self.targeting))
        # js.driver_right.whileTrue(Increm
        # entShooter(shooter=self.shooter, speed_change=1))
        # js.driver_left.whileTrue(IncrementShooter(shooter=self.shooter, speed_change=-1))
        #js.bbox_shoot_override.whileTrue(StopShooter(shooter=self.shooter))

        # js.driver_l_trigger.whileTrue(Intake_Set(intake=self.intake, rpm=2500))
        # js.driver_r_trigger.whileTrue(ShootingCommand(shooter=self.shooter, rpm=5000))

        # --- Vision & Automation ---
        # Align to Pose (Front/Left)
        #js.driver_a.debounce(0.1).whileTrue(AutoToPoseClean(self, self.swerve, target_pose=None, use_vision=True, cameras=['logi_front_hsv'], control_type='not_pathplanner'))
        #js.driver_x.debounce(0.1).whileTrue(AutoToPoseClean(self, self.swerve, target_pose=None, use_vision=True, cameras=['logi_left_hsv'], control_type='not_pathplanner'))
        
        # Track Target
        #js.driver_b.debounce(0.1).whileTrue(AutoTrackVisionTarget(self, camera_key='logi_front_hsv', target_distance=0.40))

        # --- Debug & Simulation ---
        #js.driver_lb.whileTrue(SimShowFOV(self))
        #js.driver_rb.onTrue(MoveTrainingBox(self))

        # This kills targeting mode!  DO NOT USE RB
        # js.driver_back.whileTrue(SwerveTest(self, self.swerve))

        # js.driver_l_trigger.debounce(0.1).whileTrue(RobotClimb(climber=self.climber, move_up=False, indent=0))
        # js.driver_r_trigger.debounce(0.1).whileTrue(RobotClimb(climber=self.climber, move_up=True, indent=0))

        # --- Debug & Simulation ---
        # test a setting of the swerve modules straight before running the auto to tag
        # js.driver_a.whileTrue(commands2.cmd.run(lambda: self.swerve.set_straight(), self.swerve))
        #js.driver_a.whileTrue(TrackHub(self))
        #js.driver_a.whileTrue(ShootingCommand(container=self, shooter=self.shooter))


    def bind_codriver_buttons(self) -> None:
        # ----------  CO-DRIVER BUTTONS  ---------------
        print("Binding codriver buttons")

    def bind_bbox_buttons(self) -> None:
        print("Binding bbox buttons")

        js.bbox_1_1.onTrue(commands2.InstantCommand(lambda: self.shooter.set_shooting_offset(125)))
        js.bbox_1_2.onTrue(commands2.InstantCommand(lambda: self.shooter.set_shooting_offset(-125)))
        js.bbox_1_1.onFalse(commands2.InstantCommand(lambda: self.shooter.set_shooting_offset(0)))
        js.bbox_1_2.onFalse(commands2.InstantCommand(lambda: self.shooter.set_shooting_offset(0)))

        js.bbox_1_3.whileTrue(SwerveTest(container=self, swerve=self.swerve))

        js.bbox_1_4.whileTrue(SwerveTest(container=self, swerve=self.swerve))

        js.bbox_1_5.onTrue(commands2.InstantCommand(lambda: self.questnav.quest_enabled_toggle(force='off')).ignoringDisable(True))
        js.bbox_1_6.onTrue(commands2.InstantCommand(lambda: self.questnav.quest_sync_odometry()).ignoringDisable(True))
        js.bbox_1_7.onTrue(commands2.InstantCommand(lambda: self.questnav.quest_unsync_odometry()).ignoringDisable(True))

        js.bbox_1_8.whileTrue(
            Intake_Deploy(intake=self.intake, position='up').andThen(
            Intake_Set_RPM(intake=self.intake, rpm=0, led=self.led))
        )

        js.bbox_1_9.whileTrue(
            Intake_Deploy(intake=self.intake, position='shoot').andThen(
            Intake_Set_RPM(intake=self.intake, rpm=0, led=self.led))
        )

        js.bbox_1_10.whileTrue(
            Intake_Deploy(intake=self.intake, position='down').andThen(
            Intake_Set_RPM(intake=self.intake, rpm=3000, led=self.led))
        )

        js.bbox_1_11.onTrue(commands2.InstantCommand(lambda: self.shooter.set_shooting_offset(-250)))
        js.bbox_1_12.onTrue(commands2.InstantCommand(lambda: self.shooter.set_shooting_offset(250)))

        # test the intake deploy positions on the L1-L4 buttons
        js.bbox_2_1.whileTrue(CalibrateIntake(intake=self.intake))
        js.bbox_2_2.onTrue(Intake_Deploy(intake=self.intake, position='down'))
        js.bbox_2_3.onTrue(Intake_Deploy(intake=self.intake, position='shoot'))
        js.bbox_2_4.onTrue(Intake_Deploy(intake=self.intake, position='up'))

        # test the intake speed
        #js.bbox_AB.onTrue(Intake_Set_RPM(intake=self.intake, rpm=0, led=self.led))
        #js.bbox_CD.whileTrue(Intake_Set_RPM(intake=self.intake, rpm=2500, led=self.led))
        #js.bbox_EF.whileTrue(Intake_Set_RPM(intake=self.intake, rpm=3000, led=self.led))

        # test the shooting commands
        js.bbox_2_6.whileTrue(ShootingCommand(shooter=self.shooter, targeting=self.targeting))
        js.bbox_2_8.whileTrue(StopShooter(shooter=self.shooter))

        # this is a combo of shooting commands
        js.bbox_2_7.whileTrue(commands2.ParallelCommandGroup(
            ShootingCommand(shooter=self.shooter, targeting=self.targeting),
            Intake_Deploy(intake=self.intake, position='shoot'),
        ).beforeStarting(Intake_Set_RPM(intake=self.intake, rpm=0, led=self.led)))
        # does not work as an "andThen" for some reason
        js.bbox_2_7.onFalse(Intake_Deploy(intake=self.intake, position='down'))



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

        # set all subsystems - used on dash
        wpilib.SmartDashboard.putData(f'{command_prefix}/SetSuccess', SetLEDs(container=self, led=self.led, indicator=Led.Indicator.kSUCCESS))
        wpilib.SmartDashboard.putData(f'{command_prefix}/IntakeStow', Intake_Deploy(intake=self.intake, position='up'))
        wpilib.SmartDashboard.putData(f'{command_prefix}/IntakeDeploy', Intake_Deploy(intake=self.intake, position='down'))
        wpilib.SmartDashboard.putData(f'{command_prefix}/IntakeShoot', Intake_Deploy(intake=self.intake, position='shoot'))
        wpilib.SmartDashboard.putData(f'{command_prefix}/IntakeOn', Intake_Set_RPM(intake=self.intake, rpm=3000, led=self.led))
        wpilib.SmartDashboard.putData(f'{command_prefix}/IntakeOff', Intake_Set_RPM(intake=self.intake, rpm=0, led=self.led))
        wpilib.SmartDashboard.putData(f'{command_prefix}/IntakeReverse', Intake_Set_RPM(intake=self.intake, rpm=-500, led=self.led))
        # ---
        wpilib.SmartDashboard.putData(f'{command_prefix}/HopperOn', commands2.InstantCommand(lambda: self.shooter.set_hopper_rpm(constants.ShooterConstants.k_hopper_rpm)))
        wpilib.SmartDashboard.putData(f'{command_prefix}/HopperOff',commands2.InstantCommand(lambda: self.shooter.stop_hopper()))
        wpilib.SmartDashboard.putData(f'{command_prefix}/HopperReverse', commands2.InstantCommand(lambda: self.shooter.set_hopper_rpm(-500)))
        wpilib.SmartDashboard.putData(f'{command_prefix}/IndexerOn', commands2.InstantCommand(lambda: self.shooter.set_indexer_rpm(constants.ShooterConstants.k_indexer_rpm)))
        wpilib.SmartDashboard.putData(f'{command_prefix}/IndexerOff', commands2.InstantCommand(lambda: self.shooter.stop_indexer()))
        wpilib.SmartDashboard.putData(f'{command_prefix}/IndexerReverse', commands2.InstantCommand(lambda: self.shooter.set_indexer_rpm(-500)))
        # ---
        wpilib.SmartDashboard.putData(f'{command_prefix}/ShooterOn', commands2.InstantCommand(lambda: self.shooter.set_shooter_rpm(constants.ShooterConstants.k_shooter_test_speed)))
        wpilib.SmartDashboard.putData(f'{command_prefix}/ShooterOff', StopShooter(shooter=self.shooter))
        wpilib.SmartDashboard.putData(f'{command_prefix}/ShooterReverse', commands2.InstantCommand(lambda: self.shooter.set_shooter_rpm(-500)))

        # commands for pyqt dashboard - please do not remove
        COMMAND_LIST = [CANStatus(container=self),
                        ResetFieldCentric(container=self, swerve=self.swerve, angle=0)]
        for cmd in COMMAND_LIST:
            wpilib.SmartDashboard.putData(f'{command_prefix}/{cmd.getName()}', cmd)
        #wpilib.SmartDashboard.putData(f'{command_prefix}/CANStatus', CANStatus(container=self))

        # You can and should use the exact same list of commands in the gui to watch for
        # These are left in to demonstrate a complete UI - the real one will be full of Commands (python classes), not strings
        FAKE_COMMAND_LIST = [
            # 'IntakeReverse', 'MoveClimberDown', 'MoveClimberUp', 'ResetFlex', 'GyroReset',
            'FakeCommand']
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
        [self.score_test_chooser.addOption(key, value) for key, value in self.robot_state.states_dict.items()]  # add all the indicators
        self.score_test_chooser.onChange(
            # `setattr` is the programmatic way to set an attribute. It's equivalent to
            # `self.robot_state.target = selected_value`, but can be used inside a lambda.
            listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
                commands2.cmd.runOnce(lambda: setattr(self.robot_state, 'target', selected_value))))
        wpilib.SmartDashboard.putData(f'{command_prefix}/RobotScoringMode', self.score_test_chooser)

        # ----------  AUTONOMOUS CHOOSER SECTION  ---------------
        # self.auto_chooser = AutoBuilder.buildAutoChooser('')  # this loops through the path planner deploy directory - must exist 
        self.auto_chooser = wpilib.SendableChooser()  #  use this if you don't have any pathplanner autos defined
        self.auto_chooser.addOption('1:  Wait *CODE*', PrintCommand("** Running wait auto **").andThen(commands2.WaitCommand(15)))
        self.auto_chooser.addOption('2a: Drive 2s Straight *CODE*',
                                    PrintCommand("** Running drive by velocity swerve leave auto **").
                                    andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)))
        self.auto_chooser.addOption('2b: Drive 2s To Driver Station *CODE*',
                                    PrintCommand("** Running drive by velocity swerve leave auto **").
                                    andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2.5, field_relative=True)))
        self.auto_chooser.addOption('3a: Auto Shoot *CODE*', AutoShootingGroup(self, indent=0))
        self.auto_chooser.addOption('3b: Auto Shoot and Move *CODE*', AutoShootAndPickup(self, indent=0))
        self.auto_chooser.addOption('3c: Two Cycles *CODE*', TwoCycle(self, indent=0))
        self.auto_chooser.addOption('3d: Fill Shoot Fill *CODE*', FillShootFill(self, indent=0))
        self.auto_chooser.setDefaultOption('3e: Fill Shoot Fill Shoot *CODE*', FillShootFillShoot(self, indent=0))
        self.auto_chooser.setDefaultOption('3f Fill Shoot Fill Shoot Trench *CODE*', FillShootFillShootTrench(self, indent=0))
        wpilib.SmartDashboard.putData('autonomous routines', self.auto_chooser)  #

    def register_commands(self):
        # ----------  PATHPLANNER COMMANDS  ---------------
        # this is for PathPlanner, so it can call our commands.  Note they do not magically show up in pathplanner
        # you have to add them there, and then it remembers your list of commands.  so name them wisely
        NamedCommands.registerCommand('robot_state_left', commands2.cmd.runOnce(lambda: setattr(self.robot_state, 'side', RobotState.Side.LEFT)).ignoringDisable(True))
        NamedCommands.registerCommand('deploy_and_start_intake', Intake_Deploy(intake=self.intake, position='down').andThen(
                Intake_Set_RPM(intake=self.intake, rpm=2500, led=self.led)
            )
        )
        NamedCommands.registerCommand('start_shooter_nothing_else', commands2.InstantCommand(lambda: self.container.shooter.set_shooter_rpm(sc.k_fire_up_speed)))
        NamedCommands.registerCommand('shooting_command', ShootingCommand(shooter=self.shooter, targeting=self.targeting))
        


    def get_autonomous_command(self):
        # return DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)
        return self.auto_chooser.getSelected()
