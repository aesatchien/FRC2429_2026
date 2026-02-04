import wpilib
import ntcore
from commands2 import Subsystem
from wpilib import DriverStation
import rev
from rev import SparkBase, SparkLowLevel  # trying to save some typing

import constants
from constants import IntakeConstants as ic


class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Intake')
        self.counter = ic.k_counter_offset  # note this should be an offset in constants
        self.default_rpm = ic.k_test_rpm

        # --------------- add motors and set intake rpm ----------------
        
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.intake = rev.SparkMax(ic.k_CANID_intake, motor_type)
        self.dropper = rev.SparkMax(ic.k_CANID_dropper, motor_type)

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.intake, self.dropper]

        # you need a controller to set velocity
        self.intake_controller = self.intake.getClosedLoopController()
        self.intake_encoder = self.intake.getEncoder()
        self.dropper_controller = self.dropper.getClosedLoopController()
        self.dropper_encoder = self.dropper.getEncoder()

        # default parameters for the sparkmaxes reset and persist modes -
        self.rev_resets = rev.ResetMode.kResetSafeParameters
        self.rev_persists = rev.PersistMode.kPersistParameters if constants.k_burn_flash \
            else rev.PersistMode.kNoPersistParameters

        # put the configs in a list matching the motors
        self.configs = [ic.k_intake_configs, ic.k_dropper_configs]

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]

        # initialize states
        self.intake_on = False
        self.dropper_down = False
        self.current_rpm = 0
        self._init_networktables()

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()

        self.intake_prefix = constants.intake_prefix
        self.intake_on_pub = self.inst.getBooleanTopic(f"{self.intake_prefix}/intake_on").publish()
        self.intake_rpm_pub = self.inst.getDoubleTopic(f"{self.intake_prefix}/intake_rpm").publish()
        self.dropper_down_pub = self.inst.getBooleanTopic(f"{self.intake_prefix}/dropper_down").publish()
        
        self.intake_on_pub.set(self.intake_on)
        self.intake_rpm_pub.set(self.current_rpm)
        self.dropper_down_pub.set(self.dropper_down)

    def stop_intake(self):
        # three different ways to stop the intake
        self.intake.set(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"
        # self.intake_l.setVoltage(0)  # this sets the voltage to zero (number between -12 and 12) - it is also "dumb"
        # self.intake_controller.setReference(value=0, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=0)

        self.intake_on = False
        self.current_rpm = 0
        self.dropper_down = False
        self.intake_on_pub.set(self.intake_on)
        self.intake_rpm_pub.set(self.current_rpm)
        self.dropper_down_pub.set(self.dropper_down)

    def set_intake_rpm(self, rpm=1000):
        # TODO - incorporate a PID to handle voltage sag from multiple balls
        feed_forward = min(12, 12 * rpm / 5600)  # if there is no gearing, then this gets you close
        self.set_dropper_down(down=True) if self.dropper_down == False else None
        self.intake_controller.setReference(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        print(f'set intake rpm to {rpm:.0f}')  # want to say what time it is, but can't import the container's timer easily - could use the wpilib timer
        self.intake_on = True
        self.current_rpm = rpm


        self.intake_on_pub.set(self.intake_on)
        self.intake_rpm_pub.set(self.current_rpm)
        self.dropper_down_pub.set(self.dropper_down)

    def get_rpm(self):
        return self.intake_encoder.getVelocity()

    def set_dropper_down(self, down=True):
        # function that moves intake down to the ground, or up to stow it
        pass

    def toggle_intake(self, rpm):
        if self.intake_on:
            self.stop_intake()
        else:
            self.rpm = self.default_rpm if rpm is None else rpm
            self.set_intake_rpm(self.rpm)

    def periodic(self) -> None:
        self.counter += 1

        # SmartDashboard.putBoolean('intake_enable', self.intake_enable)
        # if self.counter % 20 == 0:
        #     self.intake_rpm_pub.set(self.intake_encoder.getVelocity())
