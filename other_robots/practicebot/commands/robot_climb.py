import commands2
from rev import SparkMax
import wpilib
from wpilib import SmartDashboard, Timer
from wpimath.geometry import Translation2d, Rotation2d

from subsystems.climber import Climber
import math
from helpers.decorators import log_command  # outsource explicit logging clutter to a single line


@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class RobotClimb(commands2.command):

    def __init__(self, container, timeout, incremental=False, wait_to_finish=False, closed_loop_slot=0, indent=0) -> None:
        """
        :param wait_to_finish=False: will not make this command instantaneously execute.
        It will make this command end immediately after either the timeout has elapsed,
        or after the turret has begun moving, whichever is first. wait_to_finish=True
        will make the command end either after the timeout has elapsed, or after the
        turret has *finished* moving, whichever is first.

        To make it act kinda instantaneous, set a very small timeout. Then, it will not
        move the turret unless the arm is pretty much already at a safe position.
        """

        super().__init__()
        self.setName('Robot_Climb')
        self.indent = indent
        self.container = container
        self.incremental = incremental
        self.timeout = timeout
        self.wait_to_finish = wait_to_finish
        self.slot = closed_loop_slot
        self.timer = Timer()
        self.addRequirements(self.turret)
        self.robotLocation = None
        self.rHubLocation = None
        self.bHubLocation = None
        self.turn_angle = None

        self.counter = 0

        # initializes
        # waits for either timeout or safe turret movement
        # if timed out: exit
        # if turret safe: move turret, exit

    def initialize(self) -> None:

    def execute(self) -> None:
        # - to_low_bar: 27 in.
        # - to_mid_bar: 18 in.
        # - to_high_bar: 18 in.

    def isFinished(self) -> bool:
            return False

    def end(self) -> None:
