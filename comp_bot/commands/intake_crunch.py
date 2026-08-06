import commands2

from constants import IntakeConstants as ic
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line
from subsystems.intake import Intake

@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class Intake_Crunch(commands2.Command):  # change the name for your command


    def __init__(self, intake: Intake,  on_start=False, indent=0) -> None:
        super().__init__()
        self.setName('Intake_Crunch')  # change this to something appropriate for this command
        self.intake = intake
        self.indent = indent
        self.addRequirements(self.intake)  # commandsv2 version of requirements
        self.counter = 0
        self.state = "crunch"  # used to switch back and forth

    def initialize(self) -> None:

        self.intake.set_intake_position(angle=ic.k_crunch_angle)


    def execute(self) -> None:
        self.counter += 1
        if self.counter % 20 == 0 and self.counter < 100:
            if self.state == "crunch":
                self.intake.set_intake_position(angle=ic.k_shooting_angle)
                self.state = "shooting"
            else:
                self.intake.set_intake_position(angle=ic.k_crunch_angle)
                self.state = "crunch"
        
        if self.counter > 100 and self.counter % 20 == 0:
            if self.state == "crunch" or self.state == "second_shooting":
                self.intake.set_intake_position(angle=ic.k_shooting_angle)
                self.state = "shooting"
            else:
                self.intake.set_intake_position(angle=ic.k_second_shooting_angle)
                self.state = "second_shooting"


    def isFinished(self) -> bool:
        return self.counter >= 200
    
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        self.intake.set_intake_position(angle=ic.k_second_shooting_angle)
        pass
