import wpilib

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.commands import FollowPathCommand
from pathplannerlib.path import PathPlannerPath

import commands2
from commands2 import SequentialCommandGroup, WaitCommand, ParallelCommandGroup, InstantCommand, ConditionalCommand

import constants
from constants import AutoConstants as ac
from constants import FieldConstants as fc

from commands.drive_by_velocity_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.intake_deploy import Intake_Deploy
from commands.intake_set_rpm import Intake_Set_RPM
from commands.shooting_command import ShootingCommand
from commands.drive_to_pose_custom_control import DriveToPoseCustomControl

from helpers import joysticks as js
from helpers.apriltag_utils import auto_reflect_pose
from wpimath.geometry import Pose2d

class PathingCenterOutpost(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'PathingCenterOutpost')
        self.container = container


        # -----  PHASE I:  INTAKE FROM THE OUTPOST  -----
        #self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        # self.addCommands(commands2.WaitCommand(0.5))

        # activates the intake
        #self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=ac.k_intake_roller_rpm))

        # move to outpost and wait to for drop, then intake fuel
        self.addCommands(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center_Outpost_Ground_Intake"))
        )

        self.addCommands(commands2.WaitCommand(ac.k_outpost_drop_delay))

        self.addCommands(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Outpost_to_Shoot"))
        )

        # self.addCommands(commands2.WaitCommand(0.5))

        # -----  PHASE II:  SHOOT INITIAL HOPPER -----
        # Tracks the hub
        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.start_tracking()))

        # Starts the shooting cycle and then raises the intake after a delay to prevent compression and jams
        # forces it to die when the first command finishes
        
        self.addCommands(commands2.ParallelRaceGroup(
            ShootingCommand(shooter=container.shooter, targeting=container.targeting, indent=1, auto_timeout=ac.k_shooting_timeout, delay_cycles=10),
            DriveByJoystickSubsystemTargeting(self.container, swerve=self.container.swerve, controller=js.driver_controller, targeting=container.targeting),
            SequentialCommandGroup(
                WaitCommand(ac.k_intake_raise_delay),
                Intake_Deploy(intake=self.container.intake, position='shoot', indent=1),
                Intake_Set_RPM(intake=self.container.intake, rpm=500),
                WaitCommand(ac.k_intake_raise_delay),
                Intake_Deploy(intake=self.container.intake, position='shoot2', indent=1),
                WaitCommand(5)
            ).withTimeout(ac.k_shooting_timeout)
        ))
        # stops tracking and kill intake
        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.stop_tracking()))

        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=0))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))