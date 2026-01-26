import wpilib
import ntcore
from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation
import rev
from rev import SparkBase, SparkLowLevel  # trying to save some typing

import constants
from constants import IntakeConstants as ic


class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Intake')
        self.counter = ic.k_intake_counter_offset  # note this should be an offset in constants
        self.default_rpm = ic.k_test_rpm

        # --------------- add motors and set intake rpm ----------------
        
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.intake = rev.SparkMax(ic.k_CANID_intake, motor_type)

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.intake]

        # you need a controller to set velocity
        self.intake_controller = self.intake.getClosedLoopController()
        self.intake_encoder = self.intake.getEncoder()

        # default parameters for the sparkmaxes reset and persist modes -
        self.rev_resets = SparkBase.ResetMode.kResetSafeParameters
        self.rev_persists = SparkBase.PersistMode.kPersistParameters if constants.k_burn_flash \
            else SparkBase.PersistMode.kNoPersistParameters

        # put the configs in a list matching the motors
        self.configs = ic.k_intake_configs + [ic.k_intake_config]  # FIXME - make this consistent

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]

        # networktables
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("datatable")  # for example

        # initialize states
        self.intake_on = False
        SmartDashboard.putBoolean('intake_on', self.intake_on)

    def stop_intake(self):
        # three different ways to stop the intake
        self.intake.set(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"
        # self.intake_l.setVoltage(0)  # this sets the voltage to zero (number between -12 and 12) - it is also "dumb"
        # self.intake_controller.setReference(value=0, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=0)

        self.intake_on = False
        self.voltage = 0  # CJH for 2024 testing
        SmartDashboard.putBoolean('intake_on', self.intake_on)

    def set_intake_rpm(self, rpm=1000):
        # multiple different ways to set the intake
        # self.intake.set(rpm)
        feed_forward = min(12, 12 * rpm / 5600)  # if there is no gearing, then this gets you close
        # rev is a pain in the ass - you have to pass EXACTLY the types it wants - no using "0" for the slots anymore
        self.intake_controller.setReference(value=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        print(f'set intake rpm to {rpm:.0f}')  # want to say what time it is, but can't import the container's timer easily
        self.intake_on = True
        self.voltage = feed_forward  # 12 * rpm / 5600  # Guess
        SmartDashboard.putBoolean('intake_on', self.intake_on)

    def get_velocity(self):
        return self.intake_encoder.getVelocity()

    def toggle_intake(self, rpm):
        if self.intake_on:
            self.stop_intake()
        else:
            self.rpm = self.default_rpm if rpm is None else rpm
            self.set_intake_rpm(self.rpm)

    def periodic(self) -> None:
        self.counter += 1

        # SmartDashboard.putBoolean('intake_enable', self.intake_enable)
        if self.counter % 20 == 0:
            SmartDashboard.putNumber('intake_velocity', self.intake_encoder.getVelocity())

            # not too often
            #SmartDashboard.putNumber('intake_rpm', self.intake.getVelocity())
            #SmartDashboard.putNumber('intake_rpm_target', self.rpm)
            #at_velocity = math.fabs(self.get_velocity() - self.rpm) < 200  # need to figure out this tolerance
            #SmartDashboard.putBoolean('intake_ready', at_velocity)
            #SmartDashboard.putNumber('intake_current', self.intake.getOutputCurrent())
            #SmartDashboard.putNumber('intake_output', self.intake.getAppliedOutput())