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
from autonomous.shoot_cycle import shoot_cycle
from commands.drive_to_pose_custom_control import DriveToPoseCustomControl

from helpers import joysticks as js
from helpers.apriltag_utils import auto_reflect_pose
from wpimath import Pose2d

class PathingCenterBack(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'PathingCenterBack')
        self.container = container


        # -----  PHASE I:  DRIVE TO FILL HOPPER  -----
        #self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        # self.addCommands(commands2.WaitCommand(0.5))

        # activates the intake
        #self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=ac.k_intake_roller_rpm))

        # moves to the neutral zone to intake fuel --> come back to shoot
        self.addCommands(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center_Back"))

        )

        # self.addCommands(commands2.WaitCommand(0.5))

        # -----  PHASE II:  SHOOT INITIAL HOPPER -----
        # Tracks the hub

        # Starts the shooting cycle and then raises the intake after a delay to prevent compression and jams
        # forces it to die when the first command finishes
        
        self.addCommands(shoot_cycle(self.container, indent=1))
        # stops tracking and kill intake

        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=0))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))