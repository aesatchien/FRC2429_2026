import math

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
                                fullRange=dc.k_analog_encoder_scale_factor, offset= -turning_encoder_offset)
        self.turning_PID_controller = PIDController(Kp=ModuleConstants.kTurningP, Ki=ModuleConstants.kTurningI, Kd=ModuleConstants.kTurningD)
        self.turning_PID_controller.enableContinuousInput(minimumInput=-math.pi, maximumInput=math.pi)

        # TODO: use the absolute encoder to set this - need to check the math carefully
        # (the drive motor zeroes itself in its own constructor)
        self.turn_motor.seed_position_rad(self.get_turn_encoder())

        # self.chassisAngularOffset = chassisAngularOffset  # not yet
        self.desiredState.angle = Rotation2d(self.get_turn_encoder())

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

    def getPosition(self) -> SwerveModulePosition:
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
