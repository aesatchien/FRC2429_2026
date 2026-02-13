import ntcore
from commands2 import Subsystem
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
        self.intake_motor = rev.SparkMax(ic.k_CANID_intake_left_leader, motor_type)
        self.intake_motor_follower = rev.SparkMax(ic.k_CANID_intake_right_follower, motor_type)
        self.deploy_motor = rev.SparkMax(ic.k_CANID_dropper, motor_type)

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.intake_motor, self.intake_motor_follower, self.deploy_motor]

        # you need a controller to set velocity
        self.intake_controller = self.intake_motor.getClosedLoopController()
        self.intake_encoder = self.intake_motor.getEncoder()
        self.deploy_controller = self.deploy_motor.getClosedLoopController()
        self.deploy_encoder = self.deploy_motor.getEncoder()

        # default parameters for the sparkmaxes reset and persist modes -
        self.rev_resets = rev.ResetMode.kResetSafeParameters
        self.rev_persists = rev.PersistMode.kPersistParameters if constants.k_burn_flash \
            else rev.PersistMode.kNoPersistParameters

        # put the configs in a list matching the motors
        self.configs = ic.k_intake_configs + ic.k_deploy_configs

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]

        # initialize states
        self.intake_on = False
        self.deployed = False
        self.current_rpm = 0
        self._init_networktables()

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()

        self.intake_prefix = constants.intake_prefix
        self.intake_on_pub = self.inst.getBooleanTopic(f"{self.intake_prefix}/intake_on").publish()
        self.intake_rpm_pub = self.inst.getDoubleTopic(f"{self.intake_prefix}/intake_rpm").publish()
        self.deployed_pub = self.inst.getBooleanTopic(f"{self.intake_prefix}/deployed").publish()
        
        self.intake_on_pub.set(self.intake_on)
        self.intake_rpm_pub.set(self.current_rpm)
        self.deployed_pub.set(self.deployed)

    def stop_intake(self):
        # three different ways to stop the intake
        self.intake_motor.set(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"
        # self.intake_l.setVoltage(0)  # this sets the voltage to zero (number between -12 and 12) - it is also "dumb"
        # self.intake_controller.setReference(value=0, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=0)

        self.intake_on = False
        self.current_rpm = 0

        self.update_nt()  # update all relevant state variables on networktables

    def set_intake_rpm(self, rpm=1000):
        # TODO - incorporate a PID to handle voltage sag from multiple balls
        feed_forward = min(12, 12 * rpm / 5600)  # if there is no gearing, then this gets you close
        # self.set_dropper_down(down=True) if self.dropper_down == False else None
        self.intake_controller.setReference(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        print(f'set intake rpm to {rpm:.0f}')  # can now get time from the log command's timer
        self.intake_on = True
        self.current_rpm = rpm

        self.update_nt()  # update all relevant state variables on networktables

    def update_nt(self):
        self.intake_on_pub.set(self.intake_on)
        self.intake_rpm_pub.set(self.current_rpm)
        self.deployed_pub.set(self.deployed)

    def get_rpm(self):
        return self.intake_encoder.getVelocity()

    # TODO - get dropper position to ground and back up
    def set_down(self, down=True):
        # function that moves intake down to the ground, or up to stow it
        # passing a false would move the dropper up to stow
        self.deployed = down
        self.update_nt()
        return down

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
