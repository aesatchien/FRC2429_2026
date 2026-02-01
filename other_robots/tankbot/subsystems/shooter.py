import wpilib
import ntcore
from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation
import rev
from rev import SparkBase, SparkLowLevel  # trying to save some typing

import constants
from constants import ShooterConstants as sc


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Shooter')
        self.allowed_rpms = [0, 60] + [i for i in range(2000, 5001, 250)]  # CJH putting in other test RPMS
        self.current_index = 4  # start at 4000 rpm
        self.counter = sc.k_flywheel_counter_offset  # note this should be an offset in constants
        self.current_rpm = sc.k_test_rpm

        # --------------- add motors and shooter rpm ----------------
        
        motor_type = rev.SparkFlex.MotorType.kBrushless
        self.flywheel_left_leader = rev.SparkFlex(sc.k_CANID_flywheel_left_leader, motor_type)
        self.flywheel_right_follower = rev.SparkFlex(sc.k_CANID_flywheel_right_follower, motor_type)
        self.flywheel_back_follower = rev.SparkFlex(sc.k_CANID_flywheel_back_follower, motor_type)
        self.indexer_motor = rev.SparkMax(sc.k_CANID_indexer, rev.SparkMax.MotorType.kBrushless)

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.flywheel_left_leader, self.flywheel_right_follower, self.indexer_motor]

        # you need a controller to set velocity
        self.flywheel_controller = self.flywheel_left_leader.getClosedLoopController()
        self.flywheel_encoder = self.flywheel_left_leader.getEncoder()

        self.indexer_controller = self.indexer_motor.getClosedLoopController()
        self.indexer_encoder = self.indexer_motor.getEncoder()
        self.indexer_encoder.setPosition(0)

        # default parameters for the sparkmaxes reset and persist modes -
        self.rev_resets = rev.ResetMode.kResetSafeParameters
        self.rev_persists = rev.PersistMode.kPersistParameters if constants.k_burn_flash \
            else rev.PersistMode.kNoPersistParameters

        # put the configs in a list matching the motors
        self.configs = sc.k_flywheel_configs + [sc.k_indexer_config]  # FIXME - make this consistent

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]

        # networktables
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("datatable")  # for example

        # initialize states
        self.shooter_on = False
        self.indexer_on = False
        SmartDashboard.putBoolean('shooter_on', self.shooter_on)
        SmartDashboard.putBoolean('indexer_on', self.indexer_on)


    def stop_shooter(self):
        # three different ways to stop the shooter
        self.flywheel_left_leader.set(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"
        # self.shooter_l.setVoltage(0)  # this sets the voltage to zero (number between -12 and 12) - it is also "dumb"
        # self.flywheel_controller.setReference(value=0, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=0)

        self.shooter_on = False
        self.voltage = 0  # CJH for 2024 testing
        self.current_rpm = 0
        SmartDashboard.putBoolean('shooter_on', self.shooter_on)
        SmartDashboard.putNumber('shooter_rpm', self.current_rpm)


    def stop_indexer(self):
        self.indexer_motor.set(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"

        self.indexer_on = False
        SmartDashboard.putBoolean('indexer_on', self.indexer_on)

    def set_shooter_rpm(self, rpm=1000):
        # multiple different ways to set the shooter - simplest is self.flywheel_left_leader.set(rpm)

        if sc.k_control_type == 'max_motion':
            ks = 0 if rpm < 1 else sc.ks_volts  # otherwise it still just turns at 0
            self.flywheel_controller.setSetpoint(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kMAXMotionVelocityControl,
                                                 slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=ks)
        else:
            feed_forward = min(12.0, 12.0 * rpm / sc.motor_max_rpm)  # if there is no gearing, then this gets you close
            # rev is a pain in the ass - you have to pass EXACTLY the types it wants - no using "0" for the slots anymore
            self.flywheel_controller.setSetpoint(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
            self.voltage = feed_forward  # 12 * rpm / max rpm  # Guess
        print(f'set flywheel rpm to {rpm:.0f}')  # want to say what time it is, but can't import the container's timer easily
        self.shooter_on = True

        SmartDashboard.putBoolean('shooter_on', self.shooter_on)  # ideally these will be publishers later
        SmartDashboard.putNumber('shooter_rpm', self.current_rpm)

    def set_indexer_rpm(self, rpm=60):
        gear_ratio = 5
        feed_forward = min(12, gear_ratio * 12 * rpm / 5600)  # geared down 5x
        # rev is a pain in the ass - you have to pass EXACTLY the types it wants - no using "0" for the slots anymore
        self.indexer_controller.setSetpoint(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        print(f'set indexer rpm to {rpm:.0f}')  # want to say what time it is, but can't import the container's timer easily
        self.indexer_on = True
        SmartDashboard.putBoolean('indexer_on', self.indexer_on)

    def change_speed(self, direction):
        # direction: 1 for faster, -1 for slower, 0 for same
        self.current_index = max(0, min(len(self.allowed_rpms) - 1, self.current_index + direction))
        self.current_rpm = self.allowed_rpms[self.current_index]
        self.set_shooter_rpm(self.current_rpm)

    def get_velocity(self):
        return self.flywheel_encoder.getVelocity()

    def toggle_shooter(self, rpm):
        if self.shooter_on:
            self.stop_shooter()
        else:
            self.rpm = self.current_rpm if rpm is None else rpm
            self.set_shooter_rpm(self.rpm)

    def get_indexer_position(self):
        return self.indexer_encoder.getPosition()

    def periodic(self) -> None:
        self.counter += 1

        # SmartDashboard.putBoolean('shooter_enable', self.shooter_enable)
        SmartDashboard.putNumber('shooter_rpm', self.flywheel_encoder.getVelocity())
        if self.counter % 20 == 0:
            SmartDashboard.putNumber('indexer_position', self.indexer_encoder.getPosition())
            #SmartDashboard.putNumber('shooter_rpm', self.flywheel_encoder.getVelocity())
            # not too often
            #SmartDashboard.putNumber('shooter_rpm', self.shooter_l.getVelocity())
            #SmartDashboard.putNumber('shooter_rpm_target', self.rpm)
            #at_velocity = math.fabs(self.get_velocity() - self.rpm) < 200  # need to figure out this tolerance
            #SmartDashboard.putBoolean('shooter_ready', at_velocity)
            #SmartDashboard.putNumber('shooter_current', self.shooter_l.getOutputCurrent())
            #SmartDashboard.putNumber('shooter_output', self.shooter_l.getAppliedOutput())




