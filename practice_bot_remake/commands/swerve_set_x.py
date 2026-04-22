import commands2
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line
from subsystems.swerve import Swerve

@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class SwerveSetX(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve:Swerve, indent=0) -> None:
        super().__init__()
        self.setName('Swerve Set X')  # change this to something appropriate for this command
        self.indent = indent
        self.swerve = swerve
        self.extra_log_info = None
        self.counter = 0
        self.addRequirements(self.swerve)  # commands2 version of requirements - add the subsystems you need

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
        pass

    def execute(self) -> None:
        self.swerve.setX()

    def isFinished(self) -> bool:
        # True: fire once and end; False: run forever until interrupted; logic has it end when code returns True
        return False

    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass