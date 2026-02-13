
# Subsystem: deploy / retract, go_up, go_down, needs states fore deploted and where it is at
# Command: GoToL1, GoToL2, GoToL3, UnClimb)

#Things to import:
from time import sleep

#Subsystem methods:
# - deploy: open the motor
# - retract: close the motor
# - go_up: return a certain incremental amount based on the position of the robot --> use that to give to the commands
# - go_down: same as go_up but reverse
#
# #Commands:
# - in initialize: set the variable for current and desired position
# - in execute: make the climber move to the destination
# - in end: print the robot position to command line
# - low_bar: 27 in.
# - mid_bar: 45 in.
# - high_bar: 63 in.
# 18 inches between each bar

# def go_up(self):
    #
    #  # def go_down(self):
    # #
    # # def set_position(self):
    # #     # takes current position, then increment it by delta x
    # #     # 1 rotation = x inch
    # #     # Position convertion factor --> Take a look at 2026 tankbot
    # # def get_distance(self):

import rev
from commands2.subsystem import Subsystem
import math
import ntcore
import wpilib
from rev import ClosedLoopSlot, SparkMax
from commands2 import SubsystemBase
import constants
from constants import ClimberConstants as cc
from rev import SparkBase, SparkLowLevel

class Climber(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Climber')
        self.counter = cc.k_counter_offset  # note this should be an offset in constants
        self.default_rpm = cc.k_test_rpm

        # --------------- add motors and set motor rpm ----------------

        motor_type = rev.SparkMax.MotorType.kBrushless
        self.motor = rev.SparkMax(cc.k_CANID_motor, motor_type)

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.motor]

        # you need a controller to set velocity
        self.climber_controller = self.climber.getClosedLoopController()
        self.climber_encoder = self.climber.getEncoder()

        # default parameters for the sparkmaxes reset and persist modes
        self.rev_resets = rev.ResetMode.kResetSafeParameters
        self.rev_persists = rev.PersistMode.kPersistParameters if constants.k_burn_flash \
            else rev.PersistMode.kNoPersistParameters

        # put the configs in a list matching the motors
        self.configs = cc.k_climber_configs

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]

        # initialize states
        self.motor_on = False
        self.current_rpm = 0
        self._init_networktables()
    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()

        self.nt_prefix = constants.climber_prefix
        self.motor_on_pub = self.inst.getBooleanTopic(f"{self.nt_prefix}/motor_on").publish()
        self.motor_rpm_pub = self.inst.getDoubleTopic(f"{self.nt_prefix}/motor_rpm").publish()

        self.motor_on_pub.set(self.motor_on)
        self.motor_rpm_pub.set(self.current_rpm)

    def go_up(self):
        pass
    def go_down(self):
        pass
    def set_position(self, inches_from_ground):
     # takes current position, then increment it by delta x
     # 1 rotation = x inch
     # Position convertion factor --> Take a look at 2026 tankbot
        rpm = 0  # this is just a placeholder
        if cc.k_control_type == 'max_motion':
            ks = 0 if rpm < 1 else cc.ks_volts  # otherwise it still just turns at 0
            self.flywheel_controller.setSetpoint(setpoint= inches_from_ground, ctrl=SparkLowLevel.ControlType.kMAXMotionPositionControl,
                                                 slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=0)
        else:
            feed_forward = min(12.0, 12.0 * rpm / 6000)  # if there is no gearing, then this gets you close
            # rev is a pain in the ass - you have to pass EXACTLY the types it wants - no using "0" for the slots anymore
            self.flywheel_controller.setSetpoint(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
            self.voltage = feed_forward  # 12 * rpm / max rpm  # Guess
    def get_distance(self):
        pass
