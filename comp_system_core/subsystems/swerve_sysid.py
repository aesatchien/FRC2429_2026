"""
Shop-only SysId test of the swerve drive feedforward (kS / kV / kA).

Procedure and analysis: gui/scope2429/SYSID_SHOP_GUIDE.md.  Off by default - flip
constants.k_enable_sysid for a shop session.  With it off none of this is built, nothing is
published, and the robot is exactly what it was.

When on, four commands appear under <command_prefix>/SysId/ (the same dashboard buttons as the
other Command/ entries - the driver GUI, AdvantageScope and Elastic all show them).  Each one
drives all four drive Krakens with the same raw voltage while the turn motors hold 0 degrees.  The
commands require the swerve subsystem, so they replace the joystick drive for as long as they run,
and they only run while the robot is ENABLED.  The data lands in the Phoenix hoot (SignalLogger),
with a "state" string that the analyzer uses to split the four tests.
"""

import constants
from commands2.sysid import SysIdRoutine
from helpers import dashboard


def build_sysid(swerve):
    """Return the SysIdRoutine for the swerve drive, or None (with a printed reason) if it cannot run here."""
    if not hasattr(swerve.swerve_modules[0].drive_motor, 'set_voltage'):
        print('  *** k_enable_sysid is on, but the drive motors are not Krakens (no set_voltage) - SysId skipped ***')
        return None

    try:
        from phoenix6 import SignalLogger
        def record_state(state):
            SignalLogger.write_string('state', SysIdRoutine.state_enum_to_string(state))
    except ImportError:
        print('  *** phoenix6 not available - SysId states will not be written to the hoot ***')
        record_state = None

    return SysIdRoutine(
        SysIdRoutine.Config(ramp_rate=constants.k_sysid_ramp_rate, step_voltage=constants.k_sysid_step_voltage,
                            timeout=constants.k_sysid_timeout, record_state=record_state),
        SysIdRoutine.Mechanism(swerve.sysid_drive,
                               lambda log: None,   # the hoot already has voltage / position / velocity
                               swerve, 'swerve-drive'))


def register_sysid(swerve):
    """Publish the four SysId commands on the dashboard if constants.k_enable_sysid.  Returns the routine or None."""
    if not constants.k_enable_sysid:
        return None
    routine = build_sysid(swerve)
    if routine is None:
        return None
    prefix = f'{constants.command_prefix}/SysId'
    forward, reverse = SysIdRoutine.Direction.FORWARD, SysIdRoutine.Direction.REVERSE
    for name, command in (('1 QuasistaticForward', routine.quasistatic(forward)),
                          ('2 QuasistaticReverse', routine.quasistatic(reverse)),
                          ('3 DynamicForward', routine.dynamic(forward)),
                          ('4 DynamicReverse', routine.dynamic(reverse))):
        dashboard.SmartDashboard.put_data(f'{prefix}/{name}', command)
    print(f'  *** SysId commands published under {prefix}.  SHOP TESTING ONLY - robot will drive on its own. ***')
    return routine
