import commands2
import wpilib

from subsystems.swerve import Swerve
from wpimath.geometry import Pose2d
import math
from helpers.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=True)
class ResetFieldCentric(commands2.Command):

    def __init__(self, container, swerve: Swerve, angle: float=0, indent=0) -> None:
        super().__init__()
        self.indent = indent
        self.setName('Reset field centric')  # change this to something appropriate for this command
        self.container = container
        self.swerve = swerve
        
        self.angle_dict = {"Red": 0, "Blue": math.pi}
        self.angle = self.angle_dict['Blue']  # initialize

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        alliance = wpilib.DriverStation.getAlliance()
        self.angle = self.angle_dict['Red'] if alliance == wpilib.DriverStation.Alliance.kRed else self.angle_dict['Red']
        fixed_pose = Pose2d(x=self.swerve.get_pose().X(), y=self.swerve.get_pose().Y(), angle=self.angle)
        self.swerve.resetOdometry(fixed_pose)

    def execute(self) -> None:
        # because it fires once and the auto-log is a one-liner
        print(f"setting x to {self.swerve.get_pose().X():.2f}; y to {self.swerve.get_pose().Y():.2f}; theta to {self.angle:.0f}")

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass
