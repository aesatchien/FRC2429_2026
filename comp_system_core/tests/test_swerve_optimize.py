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

    # reversed (optimize) and scaled by cos(10 deg) (cosine scaling) - the wheel is 10 deg off
    expected = -2.0 * math.cos(math.radians(10))
    assert len(sent) == 1 and abs(sent[0] - expected) < 1e-3, f"drive should reverse to {expected:.3f} m/s, got {sent}"
    steer = _wrap(module.turning_PID_controller.get_setpoint() - wheel)
    assert abs(math.degrees(steer) - (-10)) < 0.5, f"wheel should steer -10 deg, not {math.degrees(steer):.1f}"

    # AdvantageScope's "Setpoints" publish getCommandedState() - it must be what the motors got,
    # not the raw request, or the setpoint arrow points 180 deg away from a correct wheel.
    commanded = module.getCommandedState()
    assert abs(commanded.velocity - expected) < 1e-3
    assert abs(math.degrees(_wrap(commanded.angle.radians() - wheel)) - (-10)) < 0.5


def _command(module, velocity, offset_deg):
    """setDesiredState() with the target `offset_deg` away from where the wheel points now;
    returns the m/s the drive motor was actually sent."""
    sent = []
    real_set = module.drive_motor.set_velocity_mps
    module.drive_motor.set_velocity_mps = sent.append
    try:
        wheel = module.get_turn_encoder()
        module.setDesiredState(SwerveModuleVelocity(velocity, Rotation2d(wheel + math.radians(offset_deg))))
    finally:
        module.drive_motor.set_velocity_mps = real_set
    assert len(sent) == 1
    return sent[0]


def test_cosine_scaling_slows_a_wheel_that_is_still_turning(container):
    """A wheel 60 deg from its target drives at half speed (cos 60), not full speed sideways.

    a7's cosine_scale() RETURNS the scaled state, like optimize().  If the module ever calls it
    without keeping the result, this sees 2.0 instead of 1.0 - the same silent failure the
    optimize test above guards against."""
    module = container.swerve.swerve_modules[0]
    assert abs(_command(module, 2.0, 60) - 1.0) < 1e-3
    assert abs(_command(module, 2.0, 0) - 2.0) < 1e-3          # on target: full speed
    assert abs(_command(module, 2.0, 89.9)) < 0.01             # 90 deg off: ~stopped until it turns


def test_gamepad_axes_are_raw():
    """2027 gamepads apply their own 0.1 deadband by default.  Ours (dc.k_inner_deadband) was
    tuned on raw 2026 values, so helpers/joysticks.py turns WPILib's off.  If this fails the
    two are stacked again and the stick is dead to ~0.19."""
    import wpilib.simulation
    from helpers import joysticks as js
    from constants import k_controller_type
    hid = js.driver_controller.get_controller()
    sim = (wpilib.simulation.DualSenseControllerSim if k_controller_type == "PS5"
           else wpilib.simulation.XboxControllerSim)(hid)
    sim.set_left_x(0.15)
    wpilib.simulation.DriverStationSim.notify_new_data()
    assert abs(hid.get_left_x() - 0.15) < 1e-3
    sim.set_left_x(0.0)
    wpilib.simulation.DriverStationSim.notify_new_data()
