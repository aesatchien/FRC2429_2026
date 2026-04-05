import math

import ntcore
import wpilib
from wpimath.filter import MedianFilter
from wpimath.controller import ProfiledPIDController, ArmFeedforward
from wpimath.trajectory import TrapezoidProfile
from commands2 import Subsystem
import rev
from rev import SparkBase, SparkLowLevel  # trying to save some typing

import constants
from constants import IntakeConstants as ic
from helpers.utilities import _get_motor_state, compare_motors


class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Intake')
        self.counter = ic.k_counter_offset  # note this should be an offset in constants
        self.default_rpm = ic.k_test_rpm
        self.current_index = 4  # for increment intake, we start at 4000 rpm

        # --------------- add motors and set intake rpm ----------------
        
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.intake_motor = rev.SparkMax(ic.k_CANID_intake_left_leader, motor_type)
        self.intake_motor_follower = rev.SparkMax(ic.k_CANID_intake_right_follower, motor_type)

        motor_type = rev.SparkFlex.MotorType.kBrushless
        self.deploy_motor = rev.SparkFlex(ic.k_CANID_dropper, motor_type)

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
        self.configs = ic.k_intake_configs

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]

        # initialize states
        self.intake_on = False
        self.deployed = True
        self.current_rpm = 0
        #  self.last_currents = [0] * 10  # prefer a current filter below
        self.current_filter = MedianFilter(10)
        self.bumper_switch = wpilib.DigitalInput(9)
        self.is_calibrated = False
        self._allow_calibration = False
        self.deployed_angle = ic.k_bottom_angle if constants.k_at_home else ic.k_top_angle
        self.setpoint = self.deployed_angle

        # --- WPILib Profiled PID & Arm Feedforward ---
        # Using P = 0.05 Volts per degree of error as a starting point
        self.arm_profile = ProfiledPIDController(
            0.05, 0.0, 0.0, 
            TrapezoidProfile.Constraints(
                math.degrees(ic.k_max_velocity_rad_per_second),
                math.degrees(ic.k_max_acceleration_rad_per_sec_squared)
            )
        )
        self.arm_feedforward = ArmFeedforward(
            ic.k_kS_volts, ic.k_kG_volts, 
            ic.k_kV_volt_second_per_radian, ic.k_kA_volt_second_squared_per_meter
        )
        self.arm_profile.reset(self.setpoint)
        self.arm_profile.setGoal(self.setpoint)

        # the functions below this may need to use networktables
        self._init_networktables()

        # tell encoder where we are - TODO - try the absolute encoder - may not work because of the gearing
        self.deploy_encoder.setPosition(self.setpoint)   # this sets the current value of the encoder, not the setpoint
        self.set_intake_position(self.setpoint)  # this should maintain the current position


    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()

        self.intake_prefix = constants.intake_prefix
        self.intake_on_pub = self.inst.getBooleanTopic(f"{self.intake_prefix}/intake_on").publish()
        self.intake_rpm_pub = self.inst.getDoubleTopic(f"{self.intake_prefix}/intake_rpm").publish()
        self.deployed_pub = self.inst.getBooleanTopic(f"{self.intake_prefix}/deployed").publish()
        self.deployer_angle_pub = self.inst.getDoubleTopic(f"{self.intake_prefix}/deploy_angle").publish()
        self.deployer_average_current_pub = self.inst.getDoubleTopic(f"{self.intake_prefix}/deployer_average_current").publish()
        self.deployer_output_pub = self.inst.getDoubleTopic(f"{self.intake_prefix}/deployer_output").publish()
        self.deployer_velocity_pub = self.inst.getDoubleTopic(f"{self.intake_prefix}/deployer_velocity").publish()
        self.deployer_setpoint_pub = self.inst.getDoubleTopic(f"{self.intake_prefix}/deployer_setpoint").publish()
        self.deployer_internal_setpoint_pub = self.inst.getDoubleTopic(f"{self.intake_prefix}/deployer_internal_setpoint").publish()
        self.intake_calibration_pub = self.inst.getBooleanTopic(f"{self.intake_prefix}/intake_calibration").publish()
        
        self.intake_on_pub.set(self.intake_on)
        self.intake_rpm_pub.set(self.current_rpm)
        self.deployed_pub.set(self.deployed)
        self.deployer_angle_pub.set(self.deploy_encoder.getPosition())
        self.deployer_average_current_pub.set(0)
        self.intake_calibration_pub.set(self.is_calibrated)

    def update_nt(self):
        self.intake_on_pub.set(self.intake_on)
        self.intake_rpm_pub.set(self.current_rpm)
        self.deployed_pub.set(self.deployed)
        self.deployer_average_current_pub.set(self.current_filter.lastValue())
        self.deployer_setpoint_pub.set(self.setpoint)
        self.intake_calibration_pub.set(self.is_calibrated)

    def stop_intake(self):
        # three different ways to stop the intake
        self.intake_motor.set(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"
        # self.intake_l.setVoltage(0)  # this sets the voltage to zero (number between -12 and 12) - it is also "dumb"
        # self.intake_controller.setReference(value=0, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=0)

        self.intake_on = False
        self.current_rpm = 0
        self.update_nt()  # update all relevant state variables on networktables
    
    def change_speed(self, change_speed=0):
        # direction: 1 for faster, -1 for slower, 0 for same
        self.current_index = max(0, min(len(ic.allowed_rpms) - 1, self.current_index + change_speed))
        self.default_rpm = ic.allowed_rpms[self.current_index]
    
    def zero_intake(self):
        print("Setting intake encoder to ", ic.k_bottom_angle)
        self.deploy_stop()
        self.deployed_angle = ic.k_bottom_angle
        self.deployed = True
        self.setpoint = ic.k_bottom_angle
        self.deploy_encoder.setPosition(ic.k_bottom_angle)
        self.arm_profile.reset(ic.k_bottom_angle)
        self.arm_profile.setGoal(ic.k_bottom_angle)
        self.update_nt()
    
    def set_angle_max(self):
        print("Setting intake encoder to ", ic.k_top_angle)
        self.deploy_stop()
        self.deployed_angle = ic.k_top_angle
        self.deployed = False
        self.setpoint = ic.k_top_angle
        self.deploy_encoder.setPosition(self.deployed_angle)
        self.arm_profile.reset(ic.k_top_angle)
        self.arm_profile.setGoal(ic.k_top_angle)
        self.update_nt()


    def set_intake_rpm(self, rpm=3500):
        # TODO - incorporate a PID to handle voltage sag from multiple balls
        feed_forward = min(12, 12 * rpm / 5600)  # if there is no gearing, then this gets you close
        self.intake_controller.setReference(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        self.intake_on = True
        self.current_rpm = rpm

        self.update_nt()  # update all relevant state variables on networktables

    def set_intake_position(self, angle=0):
        # Clamp the angle to our physical limits so we don't drive it into the frame
        angle = max(ic.k_bottom_angle, min(ic.k_top_angle, angle))

        # self.deploy_controller.setReference(setpoint=angle, ctrl=SparkLowLevel.ControlType.kPosition, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=ks)
        # self.deploy_controller.setReference(setpoint=angle, ctrl=SparkLowLevel.ControlType.kMAXMotionPositionControl, slot=rev.ClosedLoopSlot.kSlot1, arbFeedforward=ks)
        self.deployed_angle = angle
        self.setpoint = angle
        self.arm_profile.setGoal(angle)
        self.deployed = True if angle < 45 else False  # not sure about this - we will have a shooting position too
        self.update_nt()

    def reset_encoder(self, angle):
        self.deploy_encoder.setPosition(angle)
        self.arm_profile.reset(angle)
        self.set_intake_position(angle)  # now tell it to maintain the current position

    def get_setpoint(self):
        return self.setpoint

    def get_profile_setpoint(self):
        return self.arm_profile.getSetpoint().position

    def get_rpm(self):
        return self.current_rpm

    # TODO - get dropper position to ground and back up

    def get_average_current(self):

        return self.current_filter.lastValue()
        # return sum(self.last_currents) / len(self.last_currents)

    def deploy_stop(self):
        current_pos = self.deploy_encoder.getPosition()
        self.arm_profile.reset(current_pos)
        self.arm_profile.setGoal(current_pos)
        self.deploy_motor.setVoltage(0)

    def set_down(self, position_to_go_to="down"):
        # when position_to_go_to is "down", intake is lowered
        pass
        if position_to_go_to == "down":
            self.set_intake_position(ic.k_bottom_angle)
            print("down boy")

        elif position_to_go_to == "up":
            self.set_intake_position(ic.k_top_angle)
            print("giddy up")

        if self.deploy_controller.get_average_current() > ic.k_deploy_current_peak:
            print("Something got cooked.")
            self.deploy_stop()
            self.done = True
            self.deployed = False
            self.deployed_angle = 0

        self.update_nt()
        return position_to_go_to == "down"

    def set_brake_mode(self, brake_on=True):
        
        idle_mode = rev.SparkBaseConfig.IdleMode.kBrake if brake_on else rev.SparkBaseConfig.IdleMode.kCoast

        # Non-persistent - just change  things temporarily - these settings leave the current config untouched
        no_resets = rev.ResetMode.kNoResetSafeParameters
        no_persists = rev.PersistMode.kNoPersistParameters

        # make a temporary config just to set break or coast
        tmp_config = rev.SparkBaseConfig().setIdleMode(idle_mode)
        print(f'Temp config on intake: {tmp_config.Presets}')  # just wondering what is in there; delete after testing

        state_before = _get_motor_state(self.deploy_motor)
        rev_errors = self.deploy_motor.configure(tmp_config, no_resets, no_persists)
        state_after = _get_motor_state(self.deploy_motor)
        # If you are paranoid you can see each state - only the idle mode changes
        # compare_motors(state_before, state_after, name_a='INTAKE BEFORE', name_b='INTAKE AFTER')

        # report our results - but not the best way since there is no timestamp here
        print(f'Setting intake to idle mode {idle_mode}: {rev_errors} at {wpilib.Timer.getFPGATimestamp():.1f}s')


    def periodic(self) -> None:
        self.counter += 1

        # keep track of the deploy currents in case we want to check for calibrating or a stall condition
        # self.last_currents[self.counter % len(self.last_currents)] = self.deploy_motor.getOutputCurrent()
        self.current_filter.calculate(self.deploy_motor.getOutputCurrent())

        # get the state of the magnetic switch and calibrate the intake if at bottom position
        at_bumper = not self.bumper_switch.get()
        if self._allow_calibration:
            if at_bumper and not self.is_calibrated:
                self.set_intake_position()
                self.is_calibrated = True
            elif self.is_calibrated and not at_bumper:
                self.is_calibrated = False

        # --- Run WPILib Profiled PID and Gravity Feedforward ---
        current_pos = self.deploy_encoder.getPosition()
        pid_voltage = self.arm_profile.calculate(current_pos)
        
        # Get the internal setpoint of the trajectory for feedforward
        setpoint = self.arm_profile.getSetpoint()
        
        # ArmFeedforward takes radians. 
        # NOTE: If 0 degrees is NOT horizontal, add an offset here. e.g., math.radians(setpoint.position) + offset
        ff_voltage = self.arm_feedforward.calculate(math.radians(setpoint.position), math.radians(setpoint.velocity))
        
        total_voltage = pid_voltage + ff_voltage
        self.deploy_motor.setVoltage(total_voltage)
        # -------------------------------------------------------

        if self.counter % 5 == 0:
            self.deployer_average_current_pub.set(self.get_average_current())
            self.deployer_internal_setpoint_pub.set(setpoint.position)

        if self.counter % 20 == 0:
             self.intake_rpm_pub.set(self.intake_encoder.getVelocity())
             self.deployer_angle_pub.set(self.deploy_encoder.getPosition())
             self.deployer_output_pub.set(self.deploy_motor.getAppliedOutput())
             self.deployer_velocity_pub.set(self.deploy_encoder.getVelocity())
             self.intake_calibration_pub.set(self.is_calibrated)

             # this is not right in the simulation
             if wpilib.RobotBase.isSimulation():
                 self.intake_rpm_pub.set(self.current_rpm)
                 self.deployer_angle_pub.set(self.setpoint)
