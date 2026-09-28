import math

import ntcore
import wpilib
import wpilib.simulation
from wpimath import DCMotor, Models
from wpimath import MedianFilter
from wpimath import ProfiledPIDController, ArmFeedforward
from wpimath import TrapezoidProfile
from commands2 import Subsystem
import rev
from rev import SparkBase, SparkLowLevel  # trying to save some typing

import constants
from constants import IntakeConstants as ic, SimConstants as simc
from helpers.utilities import _get_motor_state, compare_motors, configure_sparks


class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.set_name('Intake')
        self.counter = ic.k_counter_offset  # note this should be an offset in constants
        self.default_rpm = ic.k_test_rpm
        self.current_index = 4  # for increment intake, we start at 4000 rpm

        # --------------- add motors and set intake rpm ----------------
        
        motor_type = rev.SparkMax.MotorType.BRUSHLESS
        self.intake_motor = rev.SparkMax(constants.k_can_bus_other, ic.k_CANID_intake_left_leader, motor_type)
        self.intake_motor_follower = rev.SparkMax(constants.k_can_bus_other, ic.k_CANID_intake_right_follower, motor_type)

        motor_type = rev.SparkFlex.MotorType.BRUSHLESS
        self.deploy_motor = rev.SparkFlex(constants.k_can_bus_other, ic.k_CANID_dropper, motor_type)

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.intake_motor, self.intake_motor_follower, self.deploy_motor]

        # you need a controller to set velocity
        self.intake_controller = self.intake_motor.get_closed_loop_controller()
        self.intake_encoder = self.intake_motor.get_encoder()
        self.deploy_controller = self.deploy_motor.get_closed_loop_controller()
        self.deploy_encoder = self.deploy_motor.get_encoder()

        # Each motor is paired with its own named config, so you cannot silently mis-order them.
        configure_sparks([
            (self.intake_motor,          ic.k_intake_left_leader_config),
            (self.intake_motor_follower, ic.k_intake_right_follower_config),
            (self.deploy_motor,          ic.k_deploy_config),
        ], subsystem_name='intake')

        # initialize states
        self.intake_on = False
        self.deployed = True
        self.current_rpm = 0
        #  self.last_currents = [0] * 10  # prefer a current filter below
        self.current_filter = MedianFilter(10)
        # Disabled by default - see IntakeConstants.k_enable_bumper_switch.  Left as None so
        # it claims no GPIO port and any accidental use fails loudly rather than silently.
        self.bumper_switch = (wpilib.DigitalInput(ic.k_bumper_switch_port)
                              if ic.k_enable_bumper_switch else None)
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
        self.arm_profile.set_goal(self.setpoint)

        # the functions below this may need to use networktables
        self._init_networktables()

        self._sim = None   # simulation only, built on the first simulation_periodic()

        # tell encoder where we are - TODO - try the absolute encoder - may not work because of the gearing
        self._set_deploy_angle_deg(self.setpoint)   # this sets the current value of the encoder, not the setpoint
        self.set_intake_position(self.setpoint)  # this should maintain the current position

    # ---------------------------------------------------------------------------------
    # DEPLOY ENCODER UNITS.  rev 2027a7 deleted positionConversionFactor, so the Spark reports
    # raw MOTOR ROTATIONS and RPM.  The profile, the feedforward and every setpoint in this
    # file are in DEGREES of arm angle, so the conversion happens in exactly two places - here -
    # with the factors constants.py defines for the purpose.  Before this the encoder was
    # written and read as if it were still degrees, which made one motor rotation look like
    # one degree: the arm would have moved ~6.6x too far for every degree of setpoint change.
    # ---------------------------------------------------------------------------------
    def get_angle_deg(self) -> float:
        """Measured arm angle, degrees.  This is what the mechanism view should show."""
        return self.deploy_encoder.get_position().get() * ic.k_deploy_position_factor

    def get_angle_velocity_dps(self) -> float:
        return self.deploy_encoder.get_velocity().get() * ic.k_deploy_velocity_factor

    def _set_deploy_angle_deg(self, degrees: float) -> None:
        """Re-seed the encoder so it reads `degrees` of arm angle from here on."""
        self.deploy_encoder.set_position(degrees / ic.k_deploy_position_factor)
        if self._sim is not None:
            # REV's sim overwrites the encoder from its own position every iterate(), so the
            # robot code's re-zero has to be mirrored into it or it is undone next loop.
            # (Measured on rev 2027.0.0a7.post1 - see the note in subsystems/motors.py.)
            self._sim[0].set_position(degrees / ic.k_deploy_position_factor)


    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.get_default()

        self.intake_prefix = constants.intake_prefix
        self.intake_on_pub = self.inst.get_boolean_topic(f"{self.intake_prefix}/intake_on").publish()
        self.intake_rpm_pub = self.inst.get_double_topic(f"{self.intake_prefix}/intake_rpm").publish()
        self.deployed_pub = self.inst.get_boolean_topic(f"{self.intake_prefix}/deployed").publish()
        self.deployer_angle_pub = self.inst.get_double_topic(f"{self.intake_prefix}/deploy_angle").publish()
        self.deployer_average_current_pub = self.inst.get_double_topic(f"{self.intake_prefix}/deployer_average_current").publish()
        self.deployer_output_pub = self.inst.get_double_topic(f"{self.intake_prefix}/deployer_output").publish()
        self.deployer_velocity_pub = self.inst.get_double_topic(f"{self.intake_prefix}/deployer_velocity").publish()
        self.deployer_setpoint_pub = self.inst.get_double_topic(f"{self.intake_prefix}/deployer_setpoint").publish()
        self.deployer_internal_setpoint_pub = self.inst.get_double_topic(f"{self.intake_prefix}/deployer_internal_setpoint").publish()
        self.intake_calibration_pub = self.inst.get_boolean_topic(f"{self.intake_prefix}/intake_calibration").publish()
        
        self.intake_on_pub.set(self.intake_on)
        self.intake_rpm_pub.set(self.current_rpm)
        self.deployed_pub.set(self.deployed)
        self.deployer_angle_pub.set(self.get_angle_deg())
        self.deployer_average_current_pub.set(0)
        self.intake_calibration_pub.set(self.is_calibrated)

    def update_nt(self):
        self.intake_on_pub.set(self.intake_on)
        self.intake_rpm_pub.set(self.current_rpm)
        self.deployed_pub.set(self.deployed)
        self.deployer_average_current_pub.set(self.current_filter.last_value())
        self.deployer_setpoint_pub.set(self.setpoint)
        self.intake_calibration_pub.set(self.is_calibrated)

    def stop_intake(self):
        # three different ways to stop the intake
        self.intake_motor.set_throttle(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"
        # self.intake_l.setVoltage(0)  # this sets the voltage to zero (number between -12 and 12) - it is also "dumb"
        # self.intake_controller.setSetpoint(value=0, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=0)

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
        self._set_deploy_angle_deg(ic.k_bottom_angle)
        self.arm_profile.reset(ic.k_bottom_angle)
        self.arm_profile.set_goal(ic.k_bottom_angle)
        self.update_nt()
    
    def set_angle_max(self):
        print("Setting intake encoder to ", ic.k_top_angle)
        self.deploy_stop()
        self.deployed_angle = ic.k_top_angle
        self.deployed = False
        self.setpoint = ic.k_top_angle
        self._set_deploy_angle_deg(self.deployed_angle)
        self.arm_profile.reset(ic.k_top_angle)
        self.arm_profile.set_goal(ic.k_top_angle)
        self.update_nt()


    def set_intake_rpm(self, rpm=3500):
        # TODO - incorporate a PID to handle voltage sag from multiple balls
        feed_forward = min(12, 12 * rpm / 5600)  # if there is no gearing, then this gets you close
        self.intake_controller.set_setpoint(setpoint=rpm, ctrl=SparkLowLevel.ControlType.VELOCITY, slot=rev.ClosedLoopSlot.SLOT0, arb_feedforward=feed_forward)
        self.intake_on = True
        self.current_rpm = rpm

        self.update_nt()  # update all relevant state variables on networktables

    def set_intake_position(self, angle=0):
        # Clamp the angle to our physical limits so we don't drive it into the frame
        angle = max(ic.k_bottom_angle, min(ic.k_top_angle, angle))

        # self.deploy_controller.setSetpoint(setpoint=angle, ctrl=SparkLowLevel.ControlType.kPosition, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=ks)
        # self.deploy_controller.setSetpoint(setpoint=angle, ctrl=SparkLowLevel.ControlType.kMAXMotionPositionControl, slot=rev.ClosedLoopSlot.kSlot1, arbFeedforward=ks)
        self.deployed_angle = angle
        self.setpoint = angle
        self.arm_profile.set_goal(angle)
        self.deployed = True if angle < 45 else False  # not sure about this - we will have a shooting position too
        self.update_nt()

    def reset_encoder(self, angle):
        self._set_deploy_angle_deg(angle)
        self.arm_profile.reset(angle)
        self.set_intake_position(angle)  # now tell it to maintain the current position

    def get_setpoint(self):
        return self.setpoint

    def get_profile_setpoint(self):
        return self.arm_profile.get_setpoint().position

    def set_profile_setpoint(self, angle):
        return self.arm_profile.set_goal(angle)

    def get_rpm(self):
        return self.current_rpm

    # TODO - get dropper position to ground and back up

    def get_average_current(self):

        return self.current_filter.last_value()
        # return sum(self.last_currents) / len(self.last_currents)

    def deploy_stop(self):
        current_pos = self.get_angle_deg()
        self.arm_profile.reset(current_pos)
        self.arm_profile.set_goal(current_pos)
        self.deploy_motor.set_voltage(0)

    def set_brake_mode(self, brake_on=True):
        
        idle_mode = rev.SparkBaseConfig.IdleMode.BRAKE if brake_on else rev.SparkBaseConfig.IdleMode.COAST

        # Non-persistent - just change  things temporarily - these settings leave the current config untouched
        no_resets = rev.ResetMode.NO_RESET_SAFE_PARAMETERS
        no_persists = rev.PersistMode.NO_PERSIST_PARAMETERS

        # make a temporary config just to set break or coast
        tmp_config = rev.SparkBaseConfig().set_idle_mode(idle_mode)
        print(f'Temp config on intake: {tmp_config.Presets}')  # just wondering what is in there; delete after testing

        rev_errors = self.deploy_motor.configure(tmp_config, no_resets, no_persists)
        # To see exactly what changed, wrap the configure() above in:
        #   before = _get_motor_state(self.deploy_motor);  ... ;  after = _get_motor_state(self.deploy_motor)
        #   compare_motors(before, after, name_a='INTAKE BEFORE', name_b='INTAKE AFTER')

        # report our results - but not the best way since there is no timestamp here
        print(f'Setting intake to idle mode {idle_mode}: {rev_errors} at {wpilib.Timer.get_timestamp():.1f}s')


    def periodic(self) -> None:
        self.counter += 1

        # keep track of the deploy currents in case we want to check for calibrating or a stall condition
        # self.last_currents[self.counter % len(self.last_currents)] = self.deploy_motor.getOutputCurrent()
        self.current_filter.calculate(self.deploy_motor.get_output_current().get())

        # get the state of the magnetic switch and calibrate the intake if at bottom position.
        # With the switch disabled there is no bottom-position signal at all, so report False
        # and let auto-calibration stay inert rather than fire on a value we do not have.
        at_bumper = (not self.bumper_switch.get()) if self.bumper_switch is not None else False
        if self._allow_calibration:
            if at_bumper and not self.is_calibrated:
                self.set_intake_position()
                self.is_calibrated = True
            elif self.is_calibrated and not at_bumper:
                self.is_calibrated = False

        # --- Run WPILib Profiled PID and Gravity Feedforward ---
        current_pos = self.get_angle_deg()

        # While disabled the arm cannot move, but the profile's internal setpoint marches to the
        # goal anyway - so on enable the PID sees the full error at once and steps.  Worst case:
        # press IntakeIdle (coast, and ignoringDisable) while disabled, the arm falls to 0 deg,
        # the profile is already parked at 148, and enabling hands the gearbox the whole error.
        # Keeping the profile pinned to where the arm actually is makes enable a no-op.
        if wpilib.RobotState.is_disabled():
            self.arm_profile.reset(current_pos)
            self.arm_profile.set_goal(current_pos)
            self.setpoint = current_pos

        pid_voltage = self.arm_profile.calculate(current_pos)

        # Get the internal setpoint of the trajectory for feedforward
        setpoint = self.arm_profile.get_setpoint()

        # ArmFeedforward takes radians.
        # NOTE: If 0 degrees is NOT horizontal, add an offset here. e.g., math.radians(setpoint.position) + offset
        ff_voltage = self.arm_feedforward.calculate(math.radians(setpoint.position), math.radians(setpoint.velocity))

        # Clamp.  The Spark config's outputRange(+/-0.6) bounds the CLOSED-LOOP path only; it does
        # nothing to setVoltage(), which was previously unbounded.
        total_voltage = max(-ic.k_deploy_max_voltage, min(ic.k_deploy_max_voltage, pid_voltage + ff_voltage))
        self.deploy_motor.set_voltage(total_voltage)
        # -------------------------------------------------------

        if self.counter % 1 == 0:
            self.deployer_average_current_pub.set(self.get_average_current())
            self.deployer_internal_setpoint_pub.set(setpoint.position)

        if self.counter % 20 == 0:
             self.intake_rpm_pub.set(self.intake_encoder.get_velocity().get())
             self.deployer_angle_pub.set(self.get_angle_deg())
             self.deployer_output_pub.set(self.deploy_motor.get_applied_output().get())
             self.deployer_velocity_pub.set(self.get_angle_velocity_dps())
             self.intake_calibration_pub.set(self.is_calibrated)
             # (the is_simulation() override that used to sit here is gone - the encoders are
             # simulated now, so the measured values above are the right ones to publish)

    # -------------- simulation --------------
    # SingleJointedArmSim for the deploy arm and a FlywheelSim for the rollers, each behind
    # its Spark's own sim object, so periodic() above closes its profiled loop on a real
    # (if rough) plant and the encoder reads simulated motor rotations.  The bumper switch,
    # when it is enabled, is driven from the simulated arm angle - it is a "the intake is at
    # the bottom" switch, which is what auto-calibration uses it for.
    #
    # Gravity is simulated with 0 deg = horizontal because that is what the ArmFeedforward in
    # periodic() assumes (see the NOTE there).  If the real zero is not horizontal, both the
    # feedforward and this model are wrong together, which is the honest thing for a sim.
    def simulation_periodic(self) -> None:
        if self._sim is None:
            self._sim = self._build_sims()
        deploy_sim, arm, roller_sim, follower_sim, roller = self._sim
        vbus = wpilib.RobotController.get_battery_voltage()
        dt = 0.02

        arm.set_input_voltage(deploy_sim.get_applied_output() * vbus)
        arm.update(dt)
        # arm deg/s -> motor RPM through the same factor periodic() reads with
        deploy_sim.iterate(math.degrees(arm.get_velocity()) / ic.k_deploy_velocity_factor, vbus, dt)

        roller.set_input_voltage(roller_sim.get_applied_output() * vbus)
        roller.update(dt)
        rpm = roller.get_angular_velocity() * 60 / math.tau
        roller_sim.iterate(rpm, vbus, dt)
        follower_sim.iterate(rpm, vbus, dt)

        if self.bumper_switch is not None:
            at_bottom = math.degrees(arm.get_angle()) < ic.k_bottom_angle + 3
            wpilib.simulation.DIOSim(self.bumper_switch).set_value(not at_bottom)   # active low

    def _build_sims(self):
        vortex, neo = DCMotor.neo_vortex(1), DCMotor.neo(2)
        gearing = 1 / ic.gear_ratio                          # ~54 motor turns per arm turn
        moi = wpilib.simulation.SingleJointedArmSim.estimate_moi(simc.k_intake_arm_length_m, simc.k_intake_arm_mass_kg)
        arm = wpilib.simulation.SingleJointedArmSim(
            Models.single_jointed_arm_from_physical_constants(vortex, moi, gearing), vortex, gearing,
            simc.k_intake_arm_length_m,
            math.radians(ic.k_bottom_angle - 5), math.radians(ic.k_top_angle + 5),   # hard stops
            True, math.radians(self.get_angle_deg()))
        roller = wpilib.simulation.FlywheelSim(
            Models.flywheel_from_physical_constants(neo, simc.k_intake_roller_moi, 1.0), neo)
        deploy_sim = rev.SparkFlexSim(self.deploy_motor, vortex)
        deploy_sim.set_position(self.deploy_encoder.get_position().get())   # keep the boot seed
        return (deploy_sim, arm,
                rev.SparkMaxSim(self.intake_motor, DCMotor.neo(1)),
                rev.SparkMaxSim(self.intake_motor_follower, DCMotor.neo(1)), roller)
