import commands2
from commands.log_command import log_command  # outsource explicit logging clutter to a single line

from wpilib import SmartDashboard

import constants
from constants import IntakeConstants as ic
from subsystems.intake import Intake

@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class IntakeCommandLogging(commands2.Command):  # change the name for your command


    def __init__(self, container, intake: Intake, indent=0) -> None:
        super().__init__()
        self.setName('Intaking')  # change this to something appropriate for this command
        self.container = container
        self.intake = intake
        self.addRequirements(self.intake)  # commandsv2 version of requirements
        self.extra_log_info = None
        # self.counter = 0  # add a counter if you need to track iterations, remember to initialize in below
        # self.addRequirements(self.container.??)  # commands2 version of requirements - add the subsystems you need

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
        self.intake.set_intake_rpm(ic.k_test_speed)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        self.intake.stop_intake()
