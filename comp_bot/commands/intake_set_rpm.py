import commands2
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line

from subsystems.intake import Intake
from subsystems.led import Led

@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class Intake_Set_RPM(commands2.Command):  # change the name for your command


    def __init__(self, intake: Intake, rpm=1000, on_start=False, indent=0, led:Led=None,) -> None:
        super().__init__()
        self.setName('Intake_Set')  # change this to something appropriate for this command
        self.rpm = rpm
        self.intake = intake
        self.indent = indent
        self.on_start = on_start
        self.led = led

        # self.previous_rpm = 1000  # assume we were on
        self.addRequirements(self.intake)  # commandsv2 version of requirements

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
         # if the intake is already running, then we want to stop it instead of changing the speed
        # self.previous_rpm = self.intake.get_rpm()
        self.intake.set_intake_rpm(self.rpm) if self.rpm > 0 else self.intake.stop_intake()

        self.extra_log_info = f"RPM={self.rpm}"  # updating log info here to get actual rpm

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True
        
    def end(self, interrupted: bool) -> None:
        pass
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        # if self.led is not None:
        #     if abs(self.rpm) > 1:
        #         self.led.set_indicator(Led.Indicator.kINTAKEON)
        #     elif abs(self.previous_rpm) > 1:  # flash only if we were already moving
        #         commands2.CommandScheduler.getInstance().schedule(
        #             self.led.set_indicator_with_timeout(Led.Indicator.kINTAKEOFF, timeout=1.5))
        #     else:
        #         pass  # do nothing if we are now off and were already off