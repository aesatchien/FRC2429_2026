import math

import wpilib.simulation
from wpilib import AnalogPotentiometer
from wpimath import Rotation2d
from wpimath import SwerveModuleVelocity, SwerveModulePosition
from wpimath import PIDController

from .swerve_constants import ModuleConstants
from .swerve_constants import DriveConstants as dc
from .motors import build_drive_motor, build_turn_motor, DriveMotor, TurnMotor


class SwerveModule:
    """
    One MK4i module.

    The drive and turn motors come from subsystems/motors.py and may be different vendors -
    the comp_kraken config runs a Kraken X60 on the drive and a SparkFlex on the turn.  This
    file names no vendor and imports neither `rev` nor `phoenix6`.

    Turning is closed on the roboRIO against an AnalogPotentiometer absolute encoder, so the
    turn motor only ever receives a duty cycle.  That is why the vendor seam is so small.
    """

    def __init__(self, drivingCANId: int, turningCANId: int, encoder_analog_port: int,
                 turning_encoder_offset: float, label='') -> None:

        self.label = label
        self.desiredState = SwerveModuleVelocity(0.0, Rotation2d())  # initialize desired state
        self.turning_output = 0

        #  ---------------- MOTORS (vendor chosen by the active config)  ------------------
        self.drive_motor: DriveMotor = build_drive_motor(dc.k_drive_vendor, drivingCANId, ModuleConstants,
                                                         label=f'{label} drive')
        self.turn_motor: TurnMotor = build_turn_motor(dc.k_turn_vendor, turningCANId, ModuleConstants,
                                                      label=f'{label} turn')

        #  ---------------- ABSOLUTE ENCODER AND PID FOR TURNING  ------------------
        # create the AnalogPotentiometer with the offset.  TODO: this probably has to be 5V hardware but need to check
        # automatically always in radians and the turnover offset is built in, so the PID is easier
        # TODO: double check that the scale factor is the same on the new thrifty potentiometers
        self.absolute_encoder = AnalogPotentiometer(channel=encoder_analog_port,
                                full_range=dc.k_analog_encoder_scale_factor, offset= -turning_encoder_offset)
        self.turning_PID_controller = PIDController(kp=ModuleConstants.kTurningP, ki=ModuleConstants.kTurningI, kd=ModuleConstants.kTurningD)
        self.turning_PID_controller.enable_continuous_input(minimum_input=-math.pi, maximum_input=math.pi)

        # TODO: use the absolute encoder to set this - need to check the math carefully
        # (the drive motor zeroes itself in its own constructor)
        self.turn_motor.seed_position_rad(self.get_turn_encoder())

        # self.chassisAngularOffset = chassisAngularOffset  # not yet
        self.desiredState.angle = Rotation2d(self.get_turn_encoder())

        # simulation only - built on the first simulation_periodic(), see below
        self._encoder_analog_port = encoder_analog_port
        self._turning_encoder_offset = turning_encoder_offset
        self._analog_sim = None

    def get_turn_encoder(self):
        # how we invert the absolute encoder if necessary (which it probably isn't in the standard mk4i config)
        analog_reverse_multiplier = -1 if dc.k_reverse_analog_encoders else 1
        return analog_reverse_multiplier * self.absolute_encoder.get()

    def get_turn_motor_position(self) -> float:
        """Relative position of the turn motor, radians.  Diagnostics only - the absolute
        encoder is what actually closes the loop."""
        return self.turn_motor.get_position_rad()

    def describe(self) -> tuple:
        """(drive, turn) config snapshots for the boot-time printout in Swerve."""
        return self.drive_motor.describe(), self.turn_motor.describe()

    def getState(self) -> SwerveModuleVelocity:
        """Returns the current state of the module.
        :returns: The current state of the module.
        """
        return SwerveModuleVelocity(self.drive_motor.get_velocity_mps(),
            Rotation2d(self.get_turn_encoder()),)

    def get_position(self) -> SwerveModulePosition:
        """Returns the current position of the module.
        :returns: The current position of the module.
        """
        return SwerveModulePosition(self.drive_motor.get_position_m(),
            Rotation2d(self.get_turn_encoder()),)

    def getDesiredState(self):
        return self.desiredState

    def setDesiredState(self, desiredState: SwerveModuleVelocity) -> None:
        """Sets the desired state for the module.
        :param desiredState: Desired state with velocity and angle.
        """

        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleVelocity()
        correctedDesiredState.velocity = desiredState.velocity
        correctedDesiredState.angle = desiredState.angle

        # Optimize the reference state to avoid spinning further than 90 degrees
        correctedDesiredState.optimize(Rotation2d(self.get_turn_encoder()))

        # don't let wheels servo back if we aren't asking the module to move
        if math.fabs(desiredState.velocity) < 0.002:  # need to see what is this minimum m/s that makes sense
            correctedDesiredState.velocity = 0
            correctedDesiredState.angle = self.getState().angle

        # Command the drive motor.  The adapter takes m/s whichever vendor is underneath.
        self.drive_motor.set_velocity_mps(correctedDesiredState.velocity)

        # calculate the PID value for the turning motor  - use the roborio instead of the sparkflex. todo: explain why
        self.turning_output = self.turning_PID_controller.calculate(self.get_turn_encoder(), correctedDesiredState.angle.radians())
        # clean up the turning Spark LEDs by cleaning out the noise - 20240226 CJH
        self.turning_output = 0 if math.fabs(self.turning_output) < 0.01 else self.turning_output
        self.turn_motor.set_duty_cycle(self.turning_output)

        self.desiredState = desiredState

    def set_drive_current_limit(self, amps: int) -> None:
        """Temporarily changes the drive motor current limit without resetting other configuration."""
        self.drive_motor.set_current_limit(amps)

    def resetEncoders(self) -> None:
        """ Zeroes all the SwerveModule encoders. """
        self.drive_motor.zero_position()

    def stop(self):
        pass

    # ---------------------------------------------------------------------------------
    #  SIMULATION
    # ---------------------------------------------------------------------------------
    # The analog rail is 3.3 V in simulation: AnalogPotentiometer(port, tau, 0) reads exactly
    # one turn at 3.3 V (re-verified on a7), so the inverse below scales against 3.3, not 5.
    # Get this wrong and every wheel reads ~1.5x its true angle and the turn loop hunts.
    k_analog_rail_volts = 3.3

    def simulation_periodic(self, dt: float, vbus: float) -> float:
        """Advance both motor plants and write the azimuth back into the absolute encoder.

        Returns the module's current draw in amps.  After this the drive encoder, the turn
        motor's relative encoder and the AnalogPotentiometer all read simulated motion, so
        getState() / get_position() - and therefore odometry - run exactly the real code path.
        """
        amps = self.drive_motor.sim_update(dt, vbus)
        amps += self.turn_motor.sim_update(dt, vbus)

        if self._analog_sim is None:
            self._analog_sim = wpilib.simulation.AnalogInputSim(self._encoder_analog_port)
            self._analog_sim.set_initialized(True)

        # get_turn_encoder() = mult * (voltage / 3.3 * full_range - offset), so invert that:
        # the pot must read the plant's azimuth after the module's own reverse flag and
        # per-module offset are applied to it.
        azimuth = self.turn_motor.sim_azimuth_rad()
        mult = -1 if dc.k_reverse_analog_encoders else 1
        pot_reading = mult * azimuth + self._turning_encoder_offset           # radians the pot must report
        volts = (pot_reading % dc.k_analog_encoder_scale_factor) / dc.k_analog_encoder_scale_factor * self.k_analog_rail_volts
        self._analog_sim.set_voltage(volts)
        return amps
