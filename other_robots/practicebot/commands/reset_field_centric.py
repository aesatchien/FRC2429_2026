import commands2
from subsystems.swerve import Swerve
from wpimath.geometry import Pose2d
from helpers.log_command import log_command

import robot as rb


@log_command(console=True, nt=False, print_init=True, print_end=True)
class ResetFieldCentric(commands2.Command):

    def __init__(self, container, swerve: Swerve, angle: float=0, indent=0) -> None:
        super().__init__()
        self.indent = indent
        self.setName('Reset field centric')  # change this to something appropriate for this command
        self.container = container
        self.swerve = swerve
        
        self.angle = {"Red": 0, "Blue": 180}.get(rb.MyRobot.allianceInform)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        fixed_pose = Pose2d(x=self.swerve.get_pose().X(), y=self.swerve.get_pose().Y(), angle=self.angle)
        self.swerve.resetOdometry(fixed_pose)

    def execute(self) -> None:
        # because it fires once and the auto-log is a one-liner
        print(f"setting x to {self.swerve.get_pose().X()}; y to {self.swerve.get_pose().Y()}; theta to {self.angle}")

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass
