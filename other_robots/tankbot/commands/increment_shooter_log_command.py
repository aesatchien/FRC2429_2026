import commands2
from commands.log_command import log_command  # outsource explicit logging clutter to a single line

from wpilib import SmartDashboard

import constants
from constants import ShooterConstants as sc
from subsystems.shooter import Shooter


@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class IncrementShooter(commands2.Command):  # change the name for your command

    index = 0

    def __init__(self, container, shooter: Shooter, indent=0, speed_change='same') -> None:
        super().__init__()
        self.allowed_rpms = [i for i in range(0, 5001, 1000)]
        self.shooter = shooter
        self.addRequirements(self.shooter)
        self.setName('Increment Shooter')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.extra_log_info = None
        self.speed_change = speed_change
        # self.counter = 0  # add a counter if you need to track iterations, remember to initialize in below
        # self.addRequirements(self.container.??)  # commands2 version of requirements - add the subsystems you need

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
        if (self.speed_change == 'same'):
            self.shooter.set_shooter_rpm(self.allowed_rpms[IncrementShooter.index])
        elif (self.speed_change == 'faster'):
            if (IncrementShooter.index < len(self.allowed_rpms) - 1):
                IncrementShooter.index += 1
            self.shooter.set_shooter_rpm(self.allowed_rpms[IncrementShooter.index])
        elif (self.speed_change == 'slower'):
            if (IncrementShooter.index >= 1):
                IncrementShooter.index -= 1
            self.shooter.set_shooter_rpm(self.allowed_rpms[IncrementShooter.index])


            
    def execute(self) -> None:
        # runs 50x per second, so be careful about messages and timing
        pass

    def isFinished(self) -> bool:
        # True: fire once and end; False: run forever until interrupted; logic has it end when code returns True
        return False

    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass