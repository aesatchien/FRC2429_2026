import wpilib
import commands2
from commands2 import SequentialCommandGroup, WaitCommand

import constants
from constants import AutoConstants as ac
from commands.drive_by_velocity_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.intake_deploy import Intake_Deploy
from commands.intake_set_rpm import Intake_Set_RPM
from commands.shooting_command import ShootingCommand
from commands.drive_to_pose_custom_control import DriveToPoseCustomControl

from helpers import joysticks as js
from helpers.apriltag_utils import auto_reflect_pose

class FillShootFill(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'FillShootFill')
        self.container = container


        # -----  PHASE I:  DRIVE TO FILL HOPPER  -----
        # moves the intake down
        self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        # self.addCommands(commands2.WaitCommand(0.5))

        # activates the intake
        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=constants.IntakeConstants.k_intake_default_rpm))

        # moves to the neutral zone to intake fuel --> come back to shoot
        self.addCommands(DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
                            target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_first_ball_pickup_pose, wpilib.DriverStation.getAlliance(), is_shooting=False), tolerance_type='exact').withTimeout(5)
        )

        self.addCommands(DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
                            target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_shooting_pose, wpilib.DriverStation.getAlliance(), is_shooting=True), tolerance_type='exact').withTimeout(5)
        )

        # -----  PHASE II:  SHOOT INITIAL HOPPER -----
        #

        # Tracks the hub
        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.start_tracking()))

        # Starts the shooting cycle and then raises the intake after a delay to prevent compression and jams
        # forces it to die when the first command finishes
        self.addCommands(commands2.ParallelRaceGroup(
            ShootingCommand(shooter=container.shooter, targeting=container.targeting, indent=1, auto_timeout=ac.k_shooting_timeout),
            DriveByJoystickSubsystemTargeting(self.container, swerve=self.container.swerve, controller=js.driver_controller, targeting=container.targeting),
            SequentialCommandGroup(
                WaitCommand(ac.k_intake_raise_delay),
                Intake_Deploy(intake=self.container.intake, position='shoot', indent=1),
                Intake_Set_RPM(intake=self.container.intake, rpm=500),
                WaitCommand(5)
            ).withTimeout(ac.k_shooting_timeout)
        ))
        # stops tracking
        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.stop_tracking()))

        # -----  PHASE III:  FILL HOPPER AGAIN -----
        # Moves the intake down
        self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))
        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=constants.IntakeConstants.k_intake_default_rpm))

        # Repeat what happened above
        self.addCommands(DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
                            target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_second_ball_pickup_pose, wpilib.DriverStation.getAlliance(), is_shooting=False), tolerance_type='fast').withTimeout(4.5)
        )
