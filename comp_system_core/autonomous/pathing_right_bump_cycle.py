# Right Bump Cycle
# Created by Jeo
import wpilib

from constants import FieldConstants as fc
from constants import AutoConstants as ac
from commands2 import ConditionalCommand, WaitCommand, SequentialCommandGroup

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath

import commands2

from autonomous.shoot_cycle import shoot_cycle
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.intake_deploy import Intake_Deploy
from commands.intake_set_rpm import Intake_Set_RPM
from helpers import joysticks as js


class RightBumpCycle(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'Right Bump Cycle Auto')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        self.addCommands(
            ConditionalCommand(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile('Right_Bump_Cycle')),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile('Left_Bump_Cycle')),
                self.get_is_right
            )
        )

        self.addCommands(shoot_cycle(self.container, indent=1))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

    def get_is_right(self):
        alliance_color = wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue
        is_left = self.container.swerve.get_pose().Y() > fc.k_field_width / 2
        return alliance_color ^ is_left