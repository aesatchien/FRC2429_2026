"""
A wheel asked to point nearly backwards must REVERSE THE DRIVE, not steer ~180 degrees.

WHY THIS FILE EXISTS
--------------------
2026's SwerveModuleState.optimize() changed the state in place and returned None.  a7's
SwerveModuleVelocity.optimize() leaves the state alone and RETURNS the optimized one.  The
module code still called it without keeping the result, so on a7 the optimization silently
stopped happening - no error, the wheels just took the long way round.  This drives the real
SwerveModule.setDesiredState() so an API change like that fails here instead of on the carpet.
"""

import math

from wpimath import Rotation2d, SwerveModuleVelocity


def _wrap(rad):
    return math.atan2(math.sin(rad), math.cos(rad))


def test_module_reverses_drive_instead_of_turning_180(container):
    module = container.swerve.swerve_modules[0]
    sent = []
    real_set = module.drive_motor.set_velocity_mps
    module.drive_motor.set_velocity_mps = sent.append
    try:
        wheel = module.get_turn_encoder()
        module.setDesiredState(SwerveModuleVelocity(2.0, Rotation2d(wheel + math.radians(170))))
    finally:
        module.drive_motor.set_velocity_mps = real_set

    assert sent == [-2.0], f"drive should reverse to -2.0 m/s, got {sent}"
    steer = _wrap(module.turning_PID_controller.get_setpoint() - wheel)
    assert abs(math.degrees(steer) - (-10)) < 0.5, f"wheel should steer -10 deg, not {math.degrees(steer):.1f}"
