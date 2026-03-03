import commands2
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line

from constants import ShooterConstants as sc
from subsystems.shooter import Shooter


@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class IncrementShooter(commands2.Command):  # change the name for your command

    def __init__(self, shooter: Shooter, indent=0, speed_change=0) -> None:
        super().__init__()
        self.setName('Increment Shooter')
        self.indent = indent
        self.shooter = shooter
        self.speed_change = speed_change
        self.addRequirements(self.shooter)  # commandsv2 version of requirements
        self.extra_log_info = None

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
        self.shooter.change_speed(self.speed_change)
        # self.shooter.set_shooter_rpm(self.shooter.default_rpm)

    def execute(self) -> None:
        # runs 50x per second, so be careful about messages and timing
        pass

    def isFinished(self) -> bool:
        # True: fire once and end; False: run forever until interrupted; logic has it end when code returns True
        return True

    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass