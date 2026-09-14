"""Make phoenix6 26.50.0a1 work on WPILib 2027a7.

THE PROBLEM
-----------
a7 renamed the whole WPILib Python surface to snake_case.  phoenix6 26.50.0a1 - the newest
build CTRE has published, checked 2026-09-13 - is still written against the camelCase API,
so the moment you construct a TalonFX you get:

    File ".../phoenix6/hardware/talon_fx.py", line 72, in __init__
        if RobotBase.isSimulation():
    AttributeError: type object 'wpilib._wpilib.RobotBase' has no attribute 'isSimulation'

That is every Kraken on the drivetrain, so the robot does not finish constructing.

This is the same shape as the phoenix6 26.3.0 failure recorded in pyproject.toml (a vendor
wheel calling a WPILib symbol that 2027 deleted), and it will not be fixed by pinning: pip
resolves phoenix6 against a7 happily because phoenix6 declares no wpilib version bound.
Dependency resolution succeeding tells you nothing about runtime here.

THE FIX
-------
phoenix6 only touches four WPILib symbols, all of which still exist under snake_case names.
pybind11 classes accept new attributes, so we alias the old spellings back on.

IMPORT THIS BEFORE phoenix6 IS IMPORTED.  robot.py does it first thing; subsystems/motors.py
also imports it defensively because that is where TalonFX actually gets constructed and it
may be imported by tests that never touch robot.py.

DELETE THIS FILE once CTRE ships a phoenix6 built for a7 - check with
    python -m pip index versions phoenix6 --pre
and try removing the import; if TalonFX constructs, the shim has done its job and is over.
"""

import hal
import hal.simulation
import wpilib

# (owner class, old camelCase name, new snake_case name)
#
# Derived by scanning phoenix6 for every camelCase attribute call, then resolving each
# against the real a7 API by letters-only match - not by guessing the snake spelling, which
# gets cases like isDSAttached -> is_ds_attached wrong.
_ALIASES = (
    (wpilib.RobotBase, "isSimulation", "is_simulation"),
    (wpilib.Timer, "getTimestamp", "get_timestamp"),
    (wpilib.RobotController, "getMonotonicTime", "get_monotonic_time"),
    (wpilib.RobotController, "setTimeSource", "set_time_source"),
    (wpilib.Notifier, "startPeriodic", "start_periodic"),
    # wpiutils/auto_feed_enable.py - runs for EVERY Phoenix device, so this is on the
    # TalonFX path, not an optional extra.
    (wpilib.DriverStationBackend, "isEnabled", "is_enabled"),
    (wpilib.DriverStationBackend, "getRobotMode", "get_robot_mode"),
    (wpilib.DriverStationBackend, "isEStopped", "is_e_stopped"),
    (wpilib.DriverStationBackend, "getAlliance", "get_alliance"),
    (wpilib.DriverStationBackend, "getLocation", "get_location"),
    (wpilib.DriverStationBackend, "isDSAttached", "is_ds_attached"),
    (wpilib.DriverStationBackend, "isFMSAttached", "is_fms_attached"),
    # phoenix6's auto_feed_enable runs on a THREAD and reads RobotState, not
    # DriverStationBackend.  Missing these does not fail at construction - it fails a
    # fraction of a second later on a background thread, where the traceback is easy to
    # scroll past and the robot just never gets enable frames.
    (wpilib.RobotState, "isEnabled", "is_enabled"),
    (wpilib.RobotState, "isEStopped", "is_e_stopped"),
    (wpilib.RobotState, "isDSAttached", "is_ds_attached"),
    (wpilib.RobotState, "isFMSAttached", "is_fms_attached"),
    (wpilib.RobotState, "getRobotMode", "get_robot_mode"),
    (wpilib.MatchState, "getAlliance", "get_alliance"),
    (wpilib.MatchState, "getLocation", "get_location"),
    # hal.SimDevice - phoenix6 builds its simulated signals through these.
    (hal.SimDevice, "createDouble", "create_double"),
    (hal.SimDevice, "createBoolean", "create_boolean"),
    (hal.SimDevice, "createEnum", "create_enum"),
    # hal.simulation module-level functions.  Reached by EVERY Phoenix device in sim, not
    # just CANcoder/CANdi - the TalonFX sim path goes through the periodic callback.
    (hal.simulation, "registerSimPeriodicBeforeCallback", "register_sim_periodic_before_callback"),
    (hal.simulation, "registerSimValueChangedCallback", "register_sim_value_changed_callback"),
    (hal.simulation, "getSimDeviceName", "get_sim_device_name"),
    (hal.simulation, "getSimValueDeviceHandle", "get_sim_value_device_handle"),
)

_applied = False


def install() -> None:
    global _applied
    if _applied:
        return
    missing = []
    for owner, old, new in _ALIASES:
        if hasattr(owner, old):
            continue                      # phoenix6 fixed, or WPILib kept it
        target = getattr(owner, new, None)
        if target is None:
            missing.append(f"{getattr(owner, '__name__', owner)}.{new}")
            continue
        setattr(owner, old, target)
    if missing:
        # Loud, because the alternative is an AttributeError from inside a vendor library
        # that gives no hint this shim exists.
        print("*** phoenix6_compat: WPILib no longer provides " + ", ".join(missing) +
              " - the shim is out of date and TalonFX construction will fail ***")
    _applied = True


install()
