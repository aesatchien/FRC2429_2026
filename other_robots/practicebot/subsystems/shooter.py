import ntcore
from commands2 import Subsystem
import rev
from rev import SparkBase, SparkLowLevel  # trying to save some typing

import constants
from constants import ShooterConstants as sc


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Shooter')
        self.counter = sc.k_counter_offset  # note this should be an offset in constants
        self.default_rpm = sc.k_test_rpm
        self.default_indexer_rpm = sc.k_indexer_rpm
        self.default_hopper_rpm = sc.k_hopper_rpm
        self.current_index = 4  # for increment shooter, we start at 4000 rpm

        # --------------- add motors and shooter rpm ----------------
        
        motor_type = rev.SparkMax.MotorType.kBrushless

        self.hopper = rev.SparkMax(sc.k_CANID_hopper, motor_type)
        self.indexer_left_leader = rev.SparkMax(sc.k_CANID_indexer_left_leader, motor_type)
        self.indexer_right_follower = rev.SparkMax(sc.k_CANID_indexer_right_follower, motor_type)

        self.flywheel_left_leader = rev.SparkMax(sc.k_CANID_flywheel_left_leader, motor_type)
        self.flywheel_right_follower = rev.SparkMax(sc.k_CANID_flywheel_right_follower, motor_type)
        # TODO - add rollers here and in list - decide if they are just followers or independent

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.flywheel_left_leader, self.flywheel_right_follower,
                       self.indexer_left_leader, self.indexer_right_follower, self.hopper]

        # you need a controller to set velocity
        self.flywheel_controller = self.flywheel_left_leader.getClosedLoopController()
        self.flywheel_encoder = self.flywheel_left_leader.getEncoder()

        self.indexer_controller = self.indexer_left_leader.getClosedLoopController()
        self.indexer_encoder = self.indexer_left_leader.getEncoder()

        self.hopper_controller = self.hopper.getClosedLoopController()
        self.hopper_encoder = self.hopper.getEncoder()

        # default parameters for the sparkmaxes reset and persist modes -
        self.rev_resets = rev.ResetMode.kResetSafeParameters
        self.rev_persists = rev.PersistMode.kPersistParameters if constants.k_burn_flash \
            else rev.PersistMode.kNoPersistParameters

        # put the configs in a list matching the motors
        self.configs:list = sc.k_shooter_configs
 
        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]

        # initialize states
        self.shooter_on = False
        self.indexer_on = False
        self.hopper_on = False
        self.current_rpm = 0
        self.current_indexer_rpm = 0
        self.current_hopper_rpm = 0
        self.voltage = 0
        self._init_networktables()

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.nt_prefix = "/SmartDashboard/Shooter"  # TODO = move to constants
        self.shooter_on_pub = self.inst.getBooleanTopic(f"{self.nt_prefix}/shooter_on").publish()
        self.shooter_rpm_pub = self.inst.getDoubleTopic(f"{self.nt_prefix}/shooter_position").publish()
        self.indexer_on_pub = self.inst.getBooleanTopic(f"{self.nt_prefix}/indexer_on").publish()
        self.indexer_rpm_pub = self.inst.getDoubleTopic(f"{self.nt_prefix}/indexer_position").publish()
        self.hopper_on_pub = self.inst.getBooleanTopic(f"{self.nt_prefix}/hopper_on").publish()
        self.hopper_rpm_pub = self.inst.getDoubleTopic(f"{self.nt_prefix}/hopper_rpm").publish()


    def update_nt(self):
        self.shooter_on_pub.set(self.shooter_on)
        self.shooter_rpm_pub.set(self.current_rpm)
        self.indexer_on_pub.set(self.indexer_on)
        self.indexer_rpm_pub.set(self.current_indexer_rpm)
        self.hopper_on_pub.set(self.hopper_on)
        self.hopper_rpm_pub.set(self.current_hopper_rpm)


    def stop_shooter(self):
        # three different ways to stop the shooter
        self.flywheel_left_leader.set(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"
        # self.shooter_l.setVoltage(0)  # this sets the voltage to zero (number between -12 and 12) - it is also "dumb"
        # self.flywheel_controller.setReference(value=0, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=0)
        print("Setting shooter rpm to 0")

        self.stop_indexer()
        self.stop_hopper()

        self.shooter_on = False
        self.current_rpm = 0

        self.update_nt()

    def stop_indexer(self):
        # setting everything off, then updating
        self.indexer_left_leader.set(0)
        print("Setting indexer rpm to 0")
        self.indexer_on = False
        self.current_indexer_rpm = 0

        self.update_nt()
    
    def stop_hopper(self):
        # setting everything off, then updating
        self.hopper.set(0)
        print("Setting hopper rpm to 0")
        self.hopper_on = False
        self.current_hopper_rpm = 0

        self.update_nt()

    # keeping indexer and shooter separate, and combining them in commands.
    def set_indexer_rpm(self, rpm=1000):
        feed_forward = min(12, 12 * rpm / 5600)
        self.indexer_controller.setReference(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        print(f"Setting indexer rpm to {rpm:.0f}")
        self.current_indexer_rpm = rpm
        self.indexer_on = True
        self.update_nt()

    def set_hopper_rpm(self, rpm=1000):
        feed_forward = min(12, 12 * rpm / 5600)
        self.indexer_controller.setReference(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        print(f"Setting hopper rpm to {rpm:.0f}")
        self.current_hopper_rpm = rpm
        self.hopper_on = True
        self.update_nt()

    def set_shooter_rpm(self, rpm=1000):
        # multiple different ways to set the shooter
        # self.flywheel_left_leader.set(rpm)
        feed_forward = min(12, 12 * rpm / 5600)  # if there is no gearing, then this gets you close
        # rev is a pain in the ass - you have to pass EXACTLY the types it wants - no using "0" for the slots anymore
        self.flywheel_controller.setReference(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        print(f'set flywheel rpm to {rpm:.0f}')  # want to say what time it is, but can't import the container's timer easily
        self.current_rpm = rpm
        self.shooter_on = True
        self.update_nt()

    def get_velocity(self):
        return self.flywheel_encoder.getVelocity()

    def toggle_shooter(self, rpm):
        if self.shooter_on:
            self.stop_shooter()
        else:
            self.rpm = self.default_rpm if rpm is None else rpm
            self.set_shooter_rpm(self.rpm)


    def periodic(self) -> None:
        self.counter += 1

        # SmartDashboard.putBoolean('shooter_enable', self.shooter_enable)
        if self.counter % 20 == 0:
            pass
            # if self.shooter_on:
            #     self.shooter_rpm_pub.set(self.flywheel_encoder.getVelocity())
            # else:
            #     self.shooter_rpm_pub.set(0)
