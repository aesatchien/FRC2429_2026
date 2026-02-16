import commands2
import commands
from rev import SparkMax
import wpilib
from wpilib import SmartDashboard, Timer

from subsystems.climber import Climber
import math
import ntcore
import constants

from helpers.log_command import log_command  # outsource explicit logging clutter to a single line


@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class RobotClimb(commands2.Command):

    def __init__(self, climber: Climber, move_up=False, indent=0) -> None:

        super().__init__()
        self.setName('Robot_Climb')
        self.indent = indent
        self.climber = climber
        self.move_up = move_up
        self.timer = Timer()
        self.position_index = 0
        self.current_position = 0

        self.counter = 0

        # initializes
        # waits for either timeout or safe turret movement
        # if timed out: exit
        # if turret safe: move turret, exit

    def initialize(self) -> None:
        self.climber.move_climber(increment=self.move_up)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted) -> None:
        return None
