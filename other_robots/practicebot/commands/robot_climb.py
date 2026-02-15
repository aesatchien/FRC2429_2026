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

    def __init__(self, container, move_up: bool, incremental=False, wait_to_finish=False, closed_loop_slot=0, indent=0) -> None:

        super().__init__()
        self.setName('Robot_Climb')
        self.indent = indent
        self.container = container
        self.incremental = incremental
        self.wait_to_finish = wait_to_finish
        self.slot = closed_loop_slot
        self.timer = Timer()
        self.robotLocation = None
        self.rHubLocation = None
        self.bHubLocation = None
        self.turn_angle = None
        self.position_index = 0
        self.current_position = 0

        self.counter = 0

        if (move_up==False):
            self.container.climber.move_climber(increment=False)
        else:
            self.container.climber.move_climber(increment=True)

        # initializes
        # waits for either timeout or safe turret movement
        # if timed out: exit
        # if turret safe: move turret, exit

    def initialize(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted) -> None:
        return None
