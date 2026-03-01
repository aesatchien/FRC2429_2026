import commands2

from constants import IntakeConstants as ic


from helpers.log_command import log_command  # outsource explicit logging clutter to a single line

from subsystems.intake import Intake

@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class CalibrateIntake(commands2.Command):  # change the name for your command


    def __init__(self, intake: Intake, on_start=False, indent=0) -> None:
        super().__init__()
        self.setName('Calibrate Intake')  # change this to something appropriate for this command
        self.intake = intake
        self.indent = indent
        self.addRequirements(self.intake)  # commandsv2 version of requirements
        self.on_start = on_start

    def initialize(self) -> None:
        self.intake.deploy_motor.set(0.1)
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)

        #self.intake.go_e()
        # self.intake.set_down(down=True) if self.direction == 'down' else self.intake.set_down(down=False)

    def execute(self) -> None:
        # intake_crank_voltage = ic.k_intake_crank_voltage
        # if self.direction == 'up':
        #     self.intake.run_crank(intake_crank_voltage)
        # else:
        #     self.intake.run_crank(-intake_crank_voltage)
        pass

    def isFinished(self) -> bool:
        return self.intake.get_average_current() > ic.k_deploy_current_peak
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        self.intake.deploy_motor.set(0)  # stop the dropper crank when the command ends
        if interrupted:
            self.intake.calibrated = False
        else:
            self.intake.calibrated = True
            self.intake.deployed_angle = 147
            self.intake.deployed = False