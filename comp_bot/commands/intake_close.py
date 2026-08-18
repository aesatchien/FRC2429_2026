import commands2

from constants import IntakeConstants as ic
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line
from subsystems.intake import Intake

@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class Intake_Close(commands2.Command):  # change the name for your command


    def __init__(self, intake: Intake,  on_start=False, time=2, indent=0) -> None:
        super().__init__()
        self.setName('Intake_Close')  # change this to something appropriate for this command
        self.intake = intake
        self.indent = indent
        self.addRequirements(self.intake)  # commandsv2 version of requirements
        self.counter = 0
        self.time = time

    def initialize(self) -> None:
        self.counter = 0
        self.intake.set_intake_position(angle=ic.k_bottom_angle)


    def execute(self) -> None:
        # In the amount of passed seconds, I want the intake to go from open to closed.
        # To caluclate that, perform math with counter:
        # time / 3 = mult, take mult * counter set that to angle
        self.counter += 1
        if self.counter * (3 / self.time) < ic.k_top_angle:
            self.intake.set_intake_position(angle=self.counter * (3 / self.time))


    def isFinished(self) -> bool:
        return self.counter >= ic.k_top_angle
    
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass
