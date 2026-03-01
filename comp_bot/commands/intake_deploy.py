import commands2

from constants import IntakeConstants as ic
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line

from subsystems.intake import Intake

@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class Intake_Deploy(commands2.Command):  # change the name for your command


    def __init__(self, intake: Intake, direction="down", on_start=False, indent=0) -> None:
        super().__init__()
        self.setName('Intake_Deploy')  # change this to something appropriate for this command
        self.direction = direction
        self.intake = intake
        self.indent = indent
        self.addRequirements(self.intake)  # commandsv2 version of requirements
        self.extra_log_info = f"direction={self.direction}"  # add extra info to the log messages
        self.on_start = on_start
        self.done = False

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)

        #self.intake.go_e()
        # self.intake.set_down(down=True) if self.direction == 'down' else self.intake.set_down(down=False)
        if not self.intake.calibrated:
            print("You need to calibrate first. Failed to drop")
            self.done = True
        else:
            if self.direction == "down":
                self.intake.deploy_motor.set(-.1)
                
            elif self.direction == "up":
                self.intake.deploy_motor.set(.1)

    def execute(self) -> None:
        if self.intake.get_average_current() > ic.k_deploy_current_peak:
            if self.direction == "up":
                self.intake.deployed_angle = 147
            elif self.direction == "down":
                self.intake.deployed_angle = 0
                self.intake.deployed = True
            self.done = True
            

        # intake_crank_voltage = ic.k_intake_crank_voltage
        # if self.direction == 'up':
        #     self.intake.run_crank(intake_crank_voltage)
        # else:
        #     self.intake.run_crank(-intake_crank_voltage)
    def isFinished(self) -> bool:
        return self.done
    
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        if interrupted:
            self.intake.calibrated = False
        self.intake.deploy_stop()  # stop the dropper crank when the command ends
        self.done = False
