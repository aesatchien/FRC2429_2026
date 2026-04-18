import commands2
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line

import constants
from constants import DrivetrainConstants as dc


@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class Set_Afterburner(commands2.Command):  # change the name for your command

    def __init__(self, afterburner_on=False, indent=0) -> None:
        super().__init__()
        self.afterburner_on = afterburner_on
        self.indent = indent
        self.setName('Set_Afterburner')

    def initialize(self) -> None:
        if (self.afterburner_on == True):
            dc.k_AB_on = True
        else:
            dc.k_AB_on = False

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass