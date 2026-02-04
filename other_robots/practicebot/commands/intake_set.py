import commands2
from commands.log_command import log_command  # outsource explicit logging clutter to a single line

import constants
from constants import IntakeConstants as ic
from subsystems.intake import Intake

@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class Intake_Set(commands2.Command):  # change the name for your command


    def __init__(self, intake: Intake, rpm=1000, indent=0) -> None:
        super().__init__()
        self.setName('Intake_Set')  # change this to something appropriate for this command
        self.rpm = rpm
        self.intake = intake
        self.indent = indent
        self.addRequirements(self.intake)  # commandsv2 version of requirements
        self.extra_log_info = f"RPM={self.rpm}"  # add extra info to the log messages

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
        self.intake.set_intake_rpm(self.rpm) if self.rpm > 1 else self.intake.stop_intake()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass
