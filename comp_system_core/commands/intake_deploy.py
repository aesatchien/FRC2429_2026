import commands2

from constants import IntakeConstants as ic
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line
from subsystems.intake import Intake

@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class Intake_Deploy(commands2.Command):  # change the name for your command


    def __init__(self, intake: Intake, position="down", on_start=False, indent=0) -> None:
        super().__init__()
        self.setName('Intake_Deploy')  # change this to something appropriate for this command
        self.intake = intake
        self.position = position
        self.indent = indent
        self.addRequirements(self.intake)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.extra_log_info = f"Target={self.position}"  # (for example)

        if self.position == "down":
            self.intake.set_intake_position(angle=ic.k_bottom_angle)
        elif self.position == "up":
            self.intake.set_intake_position(angle=ic.k_top_angle)
        elif self.position == "shoot":
            self.intake.set_intake_position(angle=ic.k_shooting_angle)
        elif self.position == "shoot2":
            self.intake.set_intake_position(angle=ic.k_second_shooting_angle)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True
    
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass
