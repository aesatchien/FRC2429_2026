import commands2

from subsystems.swerve import Swerve
from subsystems.targeting import Targeting
from helpers.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=True)
class RotateToTarget(commands2.Command):
    """
    Hold position and let Targeting rotate the nose onto the hub.  For AUTONOMOUS.

    Why this exists:  the autos used to drop DriveByJoystickSubsystemTargeting into their
    shooting race groups, because *something* has to own the swerve while the shooter runs
    and that command already applied targeting.get_rotation_output().  But it also reads
    hid.getLeftY()/getLeftX()/getRightTriggerAxis() at 50 Hz - and the FMS delivers joystick
    data during autonomous.  A driver leaning on a stick, or a controller whose drift exceeds
    the 0.10 deadband, drives the robot mid-path.

    This does the one thing the autos actually wanted, and cannot read a joystick at all.
    """

    def __init__(self, container, swerve: Swerve, targeting: Targeting, indent=0) -> None:
        super().__init__()
        self.setName('RotateToTarget')
        self.indent = indent
        self.container = container
        self.swerve = swerve
        self.targeting = targeting
        self.addRequirements(self.swerve)  # this is the point - we own swerve so the default command can't drive

    def execute(self) -> None:
        # Targeting.periodic() already recomputed rotation_output this loop; just apply it.
        self.swerve.drive(0, 0, self.targeting.get_rotation_output(),
                          fieldRelative=False, rate_limited=False, keep_angle=False)

    def isFinished(self) -> bool:
        return False  # runs until the race group's shooter finishes

    def end(self, interrupted: bool) -> None:
        self.swerve.drive(0, 0, 0, fieldRelative=False, rate_limited=False, keep_angle=False)
