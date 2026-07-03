# Going to use this for an easy explenation of how to use path planner + autos. - Trentan
import wpilib

from constants import FieldConstants as fc
from commands2 import ConditionalCommand

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath

import commands2

class DrawingAuto(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'Drawing Auto')
        self.container = container

        self.addCommands(
            ConditionalCommand(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile('Right_Drawing_Path')),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile('Left_Drawing_Path')),
                self.get_is_right
            )

        )

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

    def get_is_right(self):
        alliance_color = wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue
        is_left = self.container.swerve.get_pose().Y() > fc.k_field_width / 2
        return alliance_color ^ is_left