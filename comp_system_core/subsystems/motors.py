"""
Vendor adapters for the swerve module motors.

WHY THIS FILE EXISTS
--------------------
We can populate a swerve module with REV (SparkMax / SparkFlex) or CTRE (Kraken X60 /
TalonFX) motors, and we want the choice to live in ONE place instead of being smeared
through swerve_constants, swervemodule and swerve.

The seam is deliberately tiny, and it is tiny because of a decision we already made:
the turning PID runs on the roboRIO against an AnalogPotentiometer, so a turn motor
only ever has to accept a duty cycle.  Teams whose turn loop runs on the controller
have to abstract a whole closed-loop position controller; we don't.

    DriveMotor - 5 methods.   TurnMotor - 2 methods.   That is the whole interface.

THE CONTRACT: METRES AND METRES-PER-SECOND, ALWAYS.
Nothing outside this file should know a gear ratio or a wheel circumference.
  - REV gets there with encoder positionConversionFactor / velocityConversionFactor,
    so RevDriveMotor is a passthrough.
  - Phoenix has no equivalent - sensor_to_mechanism_ratio gives you mechanism
    *rotations*.  CTRE explicitly recommends against faking units with it
    ("unit conversions should be performed in robot code"), so TalonDriveMotor
    multiplies by the circumference here.

WHY CLASSES AND NOT A DICT OF FUNCTIONS
Phoenix wants control-request objects allocated once and mutated (see self._velocity_req
below), and status signals cached and refreshed rather than re-fetched.  A bare function
in a dict has nowhere to keep that, so you would allocate hundreds of objects per second
on the roboRIO or thread a context object through by hand.  Same line count, worse.
"""

import math
import typing

import rev
import wpilib

import constants

# ---------------------------------------------------------------------------------
# phoenix6 is only importable if someone has installed it.  Keep the REV path working
# on a machine that has never seen a Kraken, and give a real error message if not.
#     python -m pip install phoenix6
# ---------------------------------------------------------------------------------
try:
    from phoenix6 import BaseStatusSignal, CANBus, StatusCode
    from phoenix6.hardware import TalonFX
    from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs
    from phoenix6.controls import VelocityVoltage, DutyCycleOut
    from phoenix6.signals import InvertedValue, NeutralModeValue
    k_phoenix_available = True
    _phoenix_import_error = None
except ImportError as err:  # pragma: no cover - depends on the dev machine
    k_phoenix_available = False
    _phoenix_import_error = err


def _require_phoenix(what: str):
    if not k_phoenix_available:
        raise ImportError(
            f"{what} needs the phoenix6 package, which is not installed.\n"
            f"  fix:  python -m pip install phoenix6\n"
            f"  (original import error: {_phoenix_import_error})")


# =================================================================================
#  THE INTERFACE
# =================================================================================

class DriveMotor(typing.Protocol):
    """A swerve drive motor.  Speaks metres and metres per second, never rotations."""
    def set_velocity_mps(self, mps: float) -> None: ...
    def get_position_m(self) -> float: ...
    def get_velocity_mps(self) -> float: ...
    def zero_position(self) -> None: ...
    def set_current_limit(self, amps: int) -> None: ...
    def describe(self) -> dict: ...
    def get_sticky_faults(self) -> list: ...
    def clear_faults(self) -> None: ...


class TurnMotor(typing.Protocol):
    """A swerve turn motor.  The RIO closes the loop, so this only takes duty cycle."""
    def set_duty_cycle(self, output: float) -> None: ...
    def get_position_rad(self) -> float: ...
    def seed_position_rad(self, radians: float) -> None: ...
    def describe(self) -> dict: ...
    def get_sticky_faults(self) -> list: ...
    def clear_faults(self) -> None: ...


# =================================================================================
#  REV  -  SparkMax / SparkFlex
# =================================================================================

def _rev_persist_mode():
    return rev.PersistMode.kPersistParameters if constants.k_burn_flash else rev.PersistMode.kNoPersistParameters


# REV reports sticky faults as one bitmask.  Bit -> name, from the REVLib fault enum.
k_rev_fault_names = {0: 'kBrownout', 1: 'kOvercurrent', 2: 'kIWDTReset', 3: 'kMotorFault',
                     4: 'kSensorFault', 5: 'kStall', 6: 'kEEPROMCRC', 7: 'kCANTX', 8: 'kCANRX',
                     9: 'kHasReset', 10: 'kDRVFault', 11: 'kOtherFault', 12: 'kSoftLimitFwd',
                     13: 'kSoftLimitRev', 14: 'kHardLimitFwd', 15: 'kHardLimitRev'}


def _rev_sticky_faults(spark) -> list:
    """Decode the bitmask into names.  Vendors disagree completely on fault reporting -
    REV hands you one integer, Phoenix hands you 27 separate boolean signals - so the
    adapters normalise both to a list of strings."""
    # 2027: getStickyFaults() returns a Signal_SparkFaults, not the Faults value itself.
    # .get() unwraps it, same as the encoder getters above.
    mask = spark.getStickyFaults().get().rawBits
    return [name for bit, name in k_rev_fault_names.items() if mask & (1 << bit)]


def _rev_describe(spark, kind: str) -> dict:
    """Pull the interesting settings back off a Spark so we can eyeball them at boot."""
    ca = spark.configAccessor
    return {
        'vendor': 'REV',
        'kind': kind,
        'can_id': spark.getDeviceId(),
        'inverted': ca.getInverted(),
        'idle_mode': str(ca.getIdleMode()).split('.')[-1],
        'current_limit_a': ca.getSmartCurrentLimit(),
        'position_units_per_rev': ca.encoder.getPositionConversionFactor(),
        'velocity_units_per_rev': ca.encoder.getVelocityConversionFactor(),
        'kP': ca.closedLoop.getP(rev.ClosedLoopSlot.kSlot0),
        'kFF_or_kV': ca.closedLoop.feedForward.getkV(rev.ClosedLoopSlot.kSlot0),  # 2027: volts, not duty
    }


class RevDriveMotor:
    """
    SparkMax / SparkFlex on the drive.  The config's positionConversionFactor and
    velocityConversionFactor already put the controller in metres and m/s, so every
    method here is a straight passthrough.
    """

    def __init__(self, can_id: int, controller_cls, config, label: str = '') -> None:
        self.label = label
        self.can_id = can_id
        self.spark = controller_cls(constants.k_can_bus, can_id, rev.SparkLowLevel.MotorType.kBrushless)

        error = self.spark.configure(config, rev.ResetMode.kResetSafeParameters, _rev_persist_mode())
        if error != rev.REVLibError.kOk:
            print(f'*** CONFIG FAILED: REV drive {can_id} ({label}) returned {error} ***')

        self.encoder = self.spark.getEncoder()
        self.controller = self.spark.getClosedLoopController()
        self.encoder.setPosition(0)

    def set_velocity_mps(self, mps: float) -> None:
        # The controller is already in m/s thanks to velocityConversionFactor.
        self.controller.setSetpoint(mps, rev.SparkLowLevel.ControlType.kVelocity)

    def get_position_m(self) -> float:
        return self.encoder.getPosition().get()   # 2027: REV getters return a Signal

    def get_velocity_mps(self) -> float:
        return self.encoder.getVelocity().get()   # 2027: REV getters return a Signal

    def zero_position(self) -> None:
        self.encoder.setPosition(0)

    def set_current_limit(self, amps: int) -> None:
        # Non-persistent partial config: leaves everything else on the controller alone.
        tmp = rev.SparkBaseConfig().smartCurrentLimit(stallLimit=amps, freeLimit=amps)
        error = self.spark.configure(tmp, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        print(f'  [{self.label}] REV drive current limit -> {amps}A  ({error})')

    def get_sticky_faults(self) -> list:
        return _rev_sticky_faults(self.spark)

    def clear_faults(self) -> None:
        self.spark.clearFaults()

    def describe(self) -> dict:
        return _rev_describe(self.spark, 'drive')


class RevTurnMotor:
    """
    SparkMax / SparkFlex on the turn.  We never use its closed loop or its encoder for
    control - the RIO runs the PID against the analog absolute encoder and hands us a
    duty cycle.  get_position_rad() exists only so the dashboard can watch it.
    """

    def __init__(self, can_id: int, controller_cls, config, label: str = '') -> None:
        self.label = label
        self.can_id = can_id
        self.spark = controller_cls(constants.k_can_bus, can_id, rev.SparkLowLevel.MotorType.kBrushless)

        error = self.spark.configure(config, rev.ResetMode.kResetSafeParameters, _rev_persist_mode())
        if error != rev.REVLibError.kOk:
            print(f'*** CONFIG FAILED: REV turn {can_id} ({label}) returned {error} ***')

        self.encoder = self.spark.getEncoder()

    def set_duty_cycle(self, output: float) -> None:
        self.spark.setThrottle(output)   # 2027: SparkBase.set() -> setThrottle()

    def get_position_rad(self) -> float:
        return self.encoder.getPosition().get()  # positionConversionFactor puts this in radians

    def seed_position_rad(self, radians: float) -> None:
        self.encoder.setPosition(radians)

    def get_sticky_faults(self) -> list:
        return _rev_sticky_faults(self.spark)

    def clear_faults(self) -> None:
        self.spark.clearFaults()

    def describe(self) -> dict:
        return _rev_describe(self.spark, 'turn')


# =================================================================================
#  CTRE  -  Kraken X60 / TalonFX
# =================================================================================

def _talon_enable_fault_signals(talon) -> dict:
    """{short_name: StatusSignal} for every sticky fault the device exposes (27 of them).

    Deferred until something actually asks for faults, for two reasons:
      - optimize_bus_utilization() slows every signal we did not explicitly request, so
        fault frames are not flowing during a match.  That is what we want on a shared bus;
        we do not spend bandwidth on diagnostics all match.  Turn them on here instead.
      - constructing a StatusSignal performs an initial refresh, and doing 27 of those per
        motor at boot prints a wall of "CAN frame not received" warnings before the bus has
        settled.

    Discovered by introspection rather than hand-listed, so a Phoenix update that adds a
    fault does not silently stop being reported.
    """
    prefix = 'get_sticky_fault_'
    signals = {name[len(prefix):]: getattr(talon, name)()
               for name in dir(talon) if name.startswith(prefix)}
    values = list(signals.values())
    # Faults share status frames, so asking for a few enables the frame for all of them.
    BaseStatusSignal.set_update_frequency_for_all(10, *values)
    BaseStatusSignal.wait_for_all(0.25, *values, report_error=False)  # let one frame arrive
    return signals


def _talon_sticky_faults(signals: dict) -> list:
    """Phoenix caches signal values off the CAN stream, so this is a local read."""
    values = list(signals.values())
    BaseStatusSignal.refresh_all(*values, report_error=False)
    return sorted(name for name, sig in signals.items() if sig.value)


class TalonDriveMotor:
    """
    Kraken X60 on the drive.

    Units: feedback.sensor_to_mechanism_ratio is set to the gearbox reduction, so the
    device reports WHEEL rotations and wheel rotations/sec.  We multiply by the wheel
    circumference here so the rest of the code only ever sees metres.

    Control: VelocityVoltage, because voltage-mode closed loop is inherently
    battery-compensated - it is the Phoenix equivalent of REV's voltageCompensation(12).
    """

    def __init__(self, can_id: int, config, circumference_m: float,
                 stator_limit_a: float, supply_limit_a: float,
                 enable_foc: bool, canbus: str = 'rio', label: str = '') -> None:
        _require_phoenix('TalonDriveMotor')
        self.label = label
        self.can_id = can_id
        self.circumference_m = circumference_m
        self.stator_limit_a = stator_limit_a  # remembered so set_current_limit can preserve it
        self.supply_limit_a = supply_limit_a

        self.talon = TalonFX(can_id, CANBus(canbus))
        self._apply(config, 'initial config')

        # Allocate the control request ONCE and mutate it.  Phoenix requests are designed
        # to be reused; building a new one every loop allocates ~200 objects/sec on the RIO.
        # enable_foc must be explicit: it defaults to True, and True needs a Phoenix Pro licence.
        self._velocity_req = VelocityVoltage(0, enable_foc=enable_foc)

        # Cache the status signals too, for the same reason, then set their rates.
        self._position_sig = self.talon.get_position()
        self._velocity_sig = self.talon.get_velocity()
        self._position_sig.set_update_frequency(100)  # 2x our 50 Hz odometry loop
        self._velocity_sig.set_update_frequency(100)
        # Everything we did NOT ask for drops to 4 Hz.  We share the roboRIO CAN bus with
        # nine REV controllers, so this is not optional.
        self.talon.optimize_bus_utilization()
        self._fault_signals = None  # built on first get_sticky_faults() - see the helper

        self.zero_position()

    def _apply(self, config, what: str) -> None:
        error = self.talon.configurator.apply(config)
        if error != StatusCode.OK:
            # One retry - apply() has a 100 ms timeout and a busy bus at boot can trip it.
            error = self.talon.configurator.apply(config)
        if error != StatusCode.OK:
            print(f'*** CONFIG FAILED: Kraken {self.can_id} ({self.label}) {what} returned {error} ***')

    def set_velocity_mps(self, mps: float) -> None:
        self._velocity_req.velocity = mps / self.circumference_m  # m/s -> wheel rot/s
        self.talon.set_control(self._velocity_req)

    def get_position_m(self) -> float:
        return self._position_sig.refresh().value * self.circumference_m

    def get_velocity_mps(self) -> float:
        return self._velocity_sig.refresh().value * self.circumference_m

    def zero_position(self) -> None:
        self.talon.set_position(0)

    def set_current_limit(self, amps: int) -> None:
        """Brownout mode. This moves the SUPPLY limit - the one that protects the battery.

        Careful: applying a config group replaces the WHOLE group, so the stator limit has
        to be restated or it reverts to the Phoenix default.
        """
        self.supply_limit_a = amps
        limits = (CurrentLimitsConfigs()
                  .with_supply_current_limit(amps).with_supply_current_limit_enable(True)
                  .with_stator_current_limit(self.stator_limit_a).with_stator_current_limit_enable(True))
        self._apply(limits, f'supply current limit -> {amps}A')
        print(f'  [{self.label}] Kraken supply current limit -> {amps}A (stator held at {self.stator_limit_a}A)')

    def get_sticky_faults(self) -> list:
        if self._fault_signals is None:
            self._fault_signals = _talon_enable_fault_signals(self.talon)
        return _talon_sticky_faults(self._fault_signals)

    def clear_faults(self) -> None:
        self.talon.clear_sticky_faults()

    def describe(self) -> dict:
        cfg = TalonFXConfiguration()
        self.talon.configurator.refresh(cfg)
        return {
            'vendor': 'CTRE',
            'kind': 'drive',
            'can_id': self.can_id,
            'inverted': str(cfg.motor_output.inverted).split('.')[-1],
            'idle_mode': str(cfg.motor_output.neutral_mode).split('.')[-1],
            'current_limit_a': f'{cfg.current_limits.supply_current_limit:.0f} sup / {cfg.current_limits.stator_current_limit:.0f} stat',
            'position_units_per_rev': self.circumference_m,   # we convert in software
            'velocity_units_per_rev': self.circumference_m,
            'kP': cfg.slot0.k_p,
            'kFF_or_kV': cfg.slot0.k_v,
        }


class TalonTurnMotor:
    """
    Kraken X60 on the turn.  Not used by any current robot config - we run REV turns -
    but it is the other half of the interface, so flipping a module to all-CTRE is a
    one-word change in swerve_constants rather than a rewrite.
    """

    def __init__(self, can_id: int, config, turn_gear_ratio: float,
                 canbus: str = 'rio', label: str = '') -> None:
        _require_phoenix('TalonTurnMotor')
        self.label = label
        self.can_id = can_id
        self.turn_gear_ratio = turn_gear_ratio

        self.talon = TalonFX(can_id, CANBus(canbus))
        error = self.talon.configurator.apply(config)
        if error != StatusCode.OK:
            print(f'*** CONFIG FAILED: Kraken turn {can_id} ({label}) returned {error} ***')

        self._duty_req = DutyCycleOut(0, enable_foc=False)
        self._position_sig = self.talon.get_position()
        self._position_sig.set_update_frequency(50)
        self.talon.optimize_bus_utilization()
        self._fault_signals = None  # built on first get_sticky_faults()

    def set_duty_cycle(self, output: float) -> None:
        self._duty_req.output = output
        self.talon.set_control(self._duty_req)

    def get_position_rad(self) -> float:
        return self._position_sig.refresh().value * math.tau

    def seed_position_rad(self, radians: float) -> None:
        self.talon.set_position(radians / math.tau)

    def get_sticky_faults(self) -> list:
        if self._fault_signals is None:
            self._fault_signals = _talon_enable_fault_signals(self.talon)
        return _talon_sticky_faults(self._fault_signals)

    def clear_faults(self) -> None:
        self.talon.clear_sticky_faults()

    def describe(self) -> dict:
        return {'vendor': 'CTRE', 'kind': 'turn', 'can_id': self.can_id,
                'inverted': '-', 'idle_mode': '-', 'current_limit_a': '-',
                'position_units_per_rev': math.tau, 'velocity_units_per_rev': math.tau,
                'kP': '-', 'kFF_or_kV': '-'}


# =================================================================================
#  FACTORIES  -  the one place that turns a vendor string into an object
# =================================================================================

def build_drive_motor(vendor: str, can_id: int, mc, label: str = '') -> DriveMotor:
    """mc is subsystems.swerve_constants.ModuleConstants (passed in to avoid a circular import)."""
    if vendor == 'rev':
        return RevDriveMotor(can_id, mc.k_drive_controller_cls, mc.k_driving_config, label=label)
    if vendor == 'ctre':
        return TalonDriveMotor(can_id, mc.k_kraken_driving_config,
                               circumference_m=mc.kWheelCircumferenceMeters,
                               stator_limit_a=mc.k_kraken_stator_current_limit,
                               supply_limit_a=mc.k_kraken_supply_current_limit,
                               enable_foc=mc.k_kraken_enable_foc,
                               canbus=mc.k_kraken_canbus, label=label)
    raise ValueError(f"drive vendor must be 'rev' or 'ctre', got {vendor!r}")


def build_turn_motor(vendor: str, can_id: int, mc, label: str = '') -> TurnMotor:
    if vendor == 'rev':
        return RevTurnMotor(can_id, mc.k_turn_controller_cls, mc.k_turning_config, label=label)
    if vendor == 'ctre':
        return TalonTurnMotor(can_id, mc.k_kraken_turning_config,
                              turn_gear_ratio=mc.k_turning_motor_gear_ratio,
                              canbus=mc.k_kraken_canbus, label=label)
    raise ValueError(f"turn vendor must be 'rev' or 'ctre', got {vendor!r}")
