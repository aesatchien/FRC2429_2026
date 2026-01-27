import commands2
from rev import SparkMax
import wpilib
from wpilib import SmartDashboard, Timer
from wpimath.geometry import Translation2d, Rotation2d

from subsystems.turret import Turret
from subsystems.swerve import Swerve
import math
from helpers.decorators import log_command  # outsource explicit logging clutter to a single line


@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class TrackHub(commands2.Command):

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
        self.setName('Track_Hub')
        self.indent = indent
        self.container = container
        self.turret: Turret = self.container.turret
        self.swerve: Swerve = self.container.swerve
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
        """Called just before this Command runs the first time."""
        self.bHubLocation = Translation2d(4.74, 4.05)
        self.rHubLocation = Translation2d(12.1, 4.05)

        self.timer.reset()

    def execute(self) -> None:
        # The displacement vector from point A to point B
        #getPose gives robot position
        self.counter += 1
        self.robot_pose = self.swerve.get_pose()
        self.robot_location = self.robot_pose.translation()

        if (abs(self.rHubLocation - self.robot_location) < abs(self.bHubLocation - self.robot_location)):
            vector = self.rHubLocation - self.robot_location
        else:
            vector = self.bHubLocation - self.robot_location

        robot_to_hub_angle = vector.angle()  #angle between center of robot and the hub
        robot_angle = self.robot_pose.rotation()  #robot's angle with respect to the x-axis
        #turret_angle: Rotation2d = robot_to_hub_angle - robot_angle  #calculates the angle the turret has to point so it looks at the hub

        # Use the .angle() method of the translation

        # self.turret.set_position(turret_angle.radians())
        if (self.counter % 50 == 0):
            print(f"Setting turret to {robot_to_hub_angle.degrees():.1f}")

    def isFinished(self) -> bool:

       return False

    def end(self, interrupted: bool) -> None:

        # if (wpilib.DriverStation.isAutonomous()): self.turret.set_position(turn_angle=self.turn_angle, control_type=SparkMax.ControlType.kPosition)

        pass