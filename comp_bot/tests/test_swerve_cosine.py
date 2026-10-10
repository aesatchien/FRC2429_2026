"""
Swerve module: optimize() and cosine scaling, through the real SwerveModule.setDesiredState().

- A wheel asked to point nearly backwards reverses the drive instead of steering ~180 deg
  (optimize).
- A wheel still turning toward its target drives at speed * cos(angle error) - 60 deg off is
  half speed - instead of full speed in the wrong direction (cosine scaling).

2026 note: SwerveModuleState.optimize() and cosineScale() modify the state IN PLACE and return
None.  comp_system_core (2027a7) is the opposite - both RETURN a new state - and has the same
tests in tests/test_swerve_optimize.py.  Keep the two in step.
"""

import math

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState


def _command(module, speed, offset_deg):
    """setDesiredState() with the target `offset_deg` from where the wheel points now; returns
    the m/s actually sent to the drive motor."""
    sent = []
    real_set = module.drive_motor.set_velocity_mps
    module.drive_motor.set_velocity_mps = sent.append
    try:
        wheel = module.get_turn_encoder()
        module.setDesiredState(SwerveModuleState(speed, Rotation2d(wheel + math.radians(offset_deg))))
    finally:
        module.drive_motor.set_velocity_mps = real_set
    assert len(sent) == 1
    return sent[0]


def test_reverses_drive_instead_of_turning_180(container):
    module = container.swerve.swerve_modules[0]
    # 170 deg away -> optimize reverses the drive and steers -10 deg; cosine scales by cos(10)
    assert abs(_command(module, 2.0, 170) - (-2.0 * math.cos(math.radians(10)))) < 1e-3


def test_cosine_scaling_slows_a_wheel_that_is_still_turning(container):
    module = container.swerve.swerve_modules[0]
    assert abs(_command(module, 2.0, 60) - 1.0) < 1e-3       # cos 60 = 0.5
    assert abs(_command(module, 2.0, 0) - 2.0) < 1e-3        # on target: full speed
    assert abs(_command(module, 2.0, 89.9)) < 0.01           # 90 deg off: ~stopped until it turns
