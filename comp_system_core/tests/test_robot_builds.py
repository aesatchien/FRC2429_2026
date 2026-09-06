"""
Pre-deploy smoke test.  Run before you push code to the robot:

    robotpy test

WHY THIS FILE EXISTS
--------------------
The vendor-abstraction change broke commands/can_status.py, which held a direct reference
to module.turningSpark.  Nothing caught it, because can_status is only constructed inside
RobotContainer.initialize_dashboard() - so the failure was an AttributeError during
robotInit.  The robot program never started, and we found out by deploying.

Constructing RobotContainer exercises, in one go:
  - every subsystem constructor, and therefore every motor adapter
  - every button binding, which constructs every command it binds
  - every dashboard command in initialize_dashboard()
  - every autonomous routine in the chooser, and every command inside it

That is the whole "does robotInit work" surface.  Runs headless in a few seconds, needs no
hardware.

NOTE: RobotContainer is built ONCE for the whole session.  REV raises
"A SparkMax instance has already been created with this device ID" if you construct a
second one, so this is a module-scoped fixture rather than a call per test.

WHAT THIS DOES NOT TELL YOU
---------------------------
Nothing about whether the hardware is actually there.  A TalonFX object constructs happily
whether or not a Kraken is on the other end of the bus.  You find that out from the
"*** CONFIG FAILED ***" lines motors.py prints on a real robot at boot.
"""

import pytest

# `container` is a session fixture in conftest.py - see the note there about why it can only
# be built once per process.


def test_all_four_modules_have_working_adapters(container):
    """Each module's drive and turn motor must satisfy the motors.py interface, whichever
    vendor the active config selected."""
    drive_required = ('set_velocity_mps', 'get_position_m', 'get_velocity_mps',
                      'zero_position', 'set_current_limit', 'describe',
                      'get_sticky_faults', 'clear_faults')
    turn_required = ('set_duty_cycle', 'get_position_rad', 'seed_position_rad',
                     'describe', 'get_sticky_faults', 'clear_faults')

    assert len(container.swerve.swerve_modules) == 4
    for module in container.swerve.swerve_modules:
        for method in drive_required:
            assert hasattr(module.drive_motor, method), \
                f'{type(module.drive_motor).__name__} is missing {method}()'
        for method in turn_required:
            assert hasattr(module.turn_motor, method), \
                f'{type(module.turn_motor).__name__} is missing {method}()'


def test_modules_report_metres_not_rotations(container):
    """The adapter contract: getPosition() is metres, getState() is m/s, whatever the
    vendor.  A module that has not moved reads ~0, and the values must be finite."""
    import math
    for module in container.swerve.swerve_modules:
        position = module.getPosition().distance
        speed = module.getState().velocity
        assert math.isfinite(position) and abs(position) < 1.0, \
            f'{module.label} position {position} - zeroed at construction, so should be ~0 m'
        assert math.isfinite(speed) and abs(speed) < 10.0, \
            f'{module.label} speed {speed} m/s is not a plausible module speed'


def test_dashboard_commands_construct(container):
    """initialize_dashboard() builds CANStatus; this is exactly where the turningSpark
    reach-through blew up on the robot."""
    from commands.can_status import CANStatus

    status = CANStatus(container=container)
    assert len(status.motors) == 8, 'expected 4 drive + 4 turn motors'
    # Names are derived from the modules, not a hand-written CAN id table.
    expected = sorted(f'{m.label}_{kind}'
                      for m in container.swerve.swerve_modules
                      for kind in ('drive', 'turn'))
    assert sorted(status.motors) == expected


def test_can_status_reads_faults_from_both_vendors(container):
    """Faults must come back as a list of names regardless of vendor - REV reports one
    bitmask, Phoenix reports 27 separate signals."""
    for module in container.swerve.swerve_modules:
        for motor in (module.drive_motor, module.turn_motor):
            faults = motor.get_sticky_faults()
            assert isinstance(faults, list), f'{type(motor).__name__} returned {type(faults)}'
            assert all(isinstance(f, str) for f in faults)


def test_autonomous_chooser_has_a_default(container):
    """Autos are constructed eagerly and handed to the chooser, so a broken one is a boot
    failure, not a run-time surprise.  Building the container already proved they all
    construct; this checks the chooser is actually wired."""
    assert container.auto_chooser.getSelected() is not None, \
        'no default auto - setDefaultOption was never called'
    assert container.get_autonomous_command() is not None


def test_pathplanner_config_matches_our_constants():
    """The boot check in Swerve prints a banner when deploy/pathplanner/settings.json
    disagrees with swerve_constants.  Assert it here so the banner is never the first
    time anyone notices."""
    import math
    from pathplannerlib.config import RobotConfig
    from subsystems.swerve_constants import ModuleConstants as mc

    module = RobotConfig.fromGUISettings().moduleConfig
    expected_free_rad_s = mc.kDrivingMotorFreeSpeedRps * math.tau / mc.kDrivingMotorReduction

    assert abs(module.wheelRadiusMeters - mc.kWheelDiameterMeters / 2) < 0.001, \
        'settings.json driveWheelRadius does not match kWheelDiameterMeters'
    assert abs(module.driveCurrentLimit - mc.kDrivingMotorCurrentLimit) < 0.5, \
        'settings.json driveCurrentLimit does not match kDrivingMotorCurrentLimit'
    assert abs(module.driveMotor.freeSpeed - expected_free_rad_s) < 0.5, \
        'settings.json driveMotorType/driveGearing does not match the active drive motor'


def test_gpio_ports_are_in_systemcore_range():
    """SystemCore has SIX general-purpose ports, 0..5.

    This cannot be caught by constructing the objects: the desktop sim HAL still
    advertises roboRIO-sized banks (DIO 0..30, analog 0..7), so DigitalInput(9) builds
    happily here and then throws on the robot:

        RuntimeError: Invalid Index for DIO
          Minimum: 0 Maximum: 6 Requested: 9

    So the limit is asserted explicitly against the number the hardware reported.
    """
    import constants
    from subsystems.swerve_constants import DriveConstants

    K_GPIO_PORTS = 6  # from the robot: "Minimum: 0 Maximum: 6"

    claimed = {f"swerve {key} absolute encoder": mod['port']
               for key, mod in DriveConstants.swerve_dict.items()}
    if constants.IntakeConstants.k_enable_bumper_switch:   # disabled -> claims no port
        claimed['intake bumper switch'] = constants.IntakeConstants.k_bumper_switch_port
    claimed['LED strip'] = constants.LedConstants.k_led_pwm_port

    duplicates = {ch for ch in claimed.values() if list(claimed.values()).count(ch) > 1}
    assert not duplicates, (
        f'two devices claim the same GPIO port {duplicates}: '
        f'{ {n: c for n, c in claimed.items() if c in duplicates} }')

    bad = {name: ch for name, ch in claimed.items() if not 0 <= ch < K_GPIO_PORTS}
    assert not bad, (
        f'GPIO port(s) outside 0..{K_GPIO_PORTS - 1} on SystemCore: {bad}\n'
        f'These construct fine in sim and throw at boot on the robot.')
