from cmath import sqrt

import commands2
import wpilib

from helpers.log_command import log_command  # outsource explicit logging clutter to a single line

from constants import ShooterConstants as sc
from subsystems.shooter import Shooter
from subsystems.targeting import Targeting


@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class PredefinedShooting(commands2.Command):  # change the name for your command


    def __init__(self, shooter: Shooter, targeting: Targeting, indent=0, auto_timeout=None, rpm=0, delay_cycles=50) -> None:
        super().__init__()
        self.setName('PredefinedShooting') # change this to something appropriate for this command
        self.indent = indent
        self.shooter = shooter
        self.targeting = targeting
        self.addRequirements(self.shooter)  # commandsv2 version of requirements
        self.extra_log_info = None
        self.counter = 0  # add a counter if you need to track iterations, remember to initialize in below
        # we want indexer and hopper to start after .1 seconds or 1/10 seconds. 
        # if it runs 50x per second, 50 * 1/10 is 5, so after 5 cycles, start the indexer and hopper
        self.delay_cycles = delay_cycles  # Trentan: this is .75 seconds, was 37, 50 seems good for aluminum shooter
        self.auto_timeout = auto_timeout
        self.timer = wpilib.Timer()
        self.alliance_number = wpilib.DriverStation.getLocation()
        self.x_coords = {1: 3, 2: 3, 3: 3}  # change the value to the correct x coordinate for each alliance number
        self.y_coords = {1: 3, 2: 3, 3: 3}  # change the value to the correct y coordinate for each alliance number
        self.distance = sqrt(self.x_coords[self.alliance_number] ** 2 + self.y_coords[self.alliance_number] ** 2)  # pythagorean it
        self.rpm = self.targeting.rpm_map.get(self.effective_distance)


    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
        self.counter = 0
        # self.shooter.set_shooter_rpm(sc.k_shooter_test_speed)
        # self.shooter.set_
        # indexer_rpm(sc.k_indexer_rpm)
        # self.shooter.set_hopper_rpm(sc.k_hopper_rpm)
        self.shooter.set_shooter_rpm(self.rpm if self.rpm <= 5600 else sc.k_shooter_max_speed)

        # maintain a timer - need to implement this for auto time out and for delay time instead of cycles
        self.timer.reset()
        self.timer.start()

       # initialize indexer and hopper
        self.shooter.set_indexer_rpm(-1000)
        self.shooter.stop_hopper()

        self.extra_log_info = f"delay={self.delay_cycles}"  # (for example)

    def execute(self) -> None:
        self.counter += 1

        hopper_sign = 1 # WAS agitating with  "if self.counter % 30 < 25 else -1" but we don't need that

        self.rpm = self.rpm if self.counter > self.delay_cycles + 25 else self.rpm + 150  # leo making it faster during spinup
        self.shooter.set_shooter_rpm(self.rpm if self.rpm <= 5600 else sc.k_shooter_max_speed)

        if self.counter > self.delay_cycles: # and (not self.shooter.indexer_on or not self.shooter.hopper_on):
            self.shooter.set_indexer_rpm(sc.k_indexer_rpm)
        if self.counter > self.delay_cycles + 10:  # let the indexer spin up before we start forcing balls in
            self.shooter.set_hopper_rpm(sc.k_hopper_rpm * hopper_sign)

        else:
            pass
            #self.shooter.stop_indexer()
            #self.shooter.stop_hopper()
        # runs 50x per second, so be careful about messages and timing

    def isFinished(self) -> bool:
        # True: fire once and end; False: run forever until interrupted; logic has it end when code returns True
        if self.auto_timeout is None:
            return False
        else:
            return self.timer.get() > self.auto_timeout
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        self.shooter.stop_shooter()
