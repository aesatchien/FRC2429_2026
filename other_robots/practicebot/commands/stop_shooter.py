import commands2
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line

from subsystems.shooter import Shooter

@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class StopShooter(commands2.Command):  # change the name for your command


    def __init__(self, shooter: Shooter, rpm=0, indent=0) -> None:
        super().__init__()
        self.setName('Stop Shooter') # change this to something appropriate for this command
        self.indent = indent
        self.shooter = shooter
        self.addRequirements(self.shooter)  # commandsv2 version of requirements
        self.extra_log_info = None

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        self.shooter.stop_shooter()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        # True: fire once and end; False: run forever until interrupted; logic has it end when code returns True
        return True
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass
