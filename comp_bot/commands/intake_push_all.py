import commands2

from constants import IntakeConstants as ic
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line
from subsystems.intake import Intake

@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class Intake_Push_All(commands2.Command):  # change the name for your command


    def __init__(self, intake: Intake,  on_start=False, indent=0) -> None:
        super().__init__()
        self.setName('Intake_Push_All')  # change this to something appropriate for this command
        self.intake = intake
        self.indent = indent
        self.addRequirements(self.intake)  # commandsv2 version of requirements
        self.counter = 0

    def initialize(self) -> None:

        self.intake.set_intake_position(angle=ic.bottom_angle)
        self.counter = 0


    def execute(self) -> None:
        self.counter += 1
        if self.counter < 147 * 2:
            self.intake.set_intake_position(angle=self.counter / 2)


    def isFinished(self) -> bool:
        return self.counter >= 147 * 2
    
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass
