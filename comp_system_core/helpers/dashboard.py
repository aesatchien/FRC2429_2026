"""SmartDashboard replacement for WPILib 2027a7.

WHY THIS EXISTS
---------------
a7 deleted the entire Sendable ecosystem.  There is no Sendable, no SendableRegistry, no
SendableBuilder, no SmartDashboard and no SendableChooser anywhere in the a7 wheels.  They
are replaced by two new packages:

    robotpy-telemetry  TelemetryRegistry / TelemetryTable  - one-way logging (put_number)
    robotpy-tunables   TunableRegistry  / TunableTable     - two-way values the dashboard
                                                             can write BACK to the robot

The split matters and is the whole reason put_data() below branches.  In a6,
SmartDashboard.putData(key, command) gave you a dashboard BUTTON that ran the command.  In
a7 that behaviour lives in Command.publish_tunables(), which publishes a mutable "running"
boolean whose on_set() schedules or cancels.  Command.log_to() is read-only telemetry and
would give you a label you cannot press.  Same for Selectable: the selection comes back
from the dashboard, so it is a tunable, not telemetry.

    publish_tunables()  -> Command, Selectable          (interactive)
    log_to()            -> Field2d, Mechanism2d         (display only)

TWO THINGS THAT WILL BITE YOU IF YOU SKIP THEM
----------------------------------------------
1. Backends must be registered or EVERYTHING IS SILENTLY DISCARDED.  With no backend the
   registry hands you a DiscardTelemetryBackend: log() succeeds, prints one
   "no backend for path" warning, and drops the value.  install() below does the
   registration and robot.py calls it before anything else publishes.

2. TunableRegistry.update() must run every loop or values never come BACK from the
   dashboard - the auto chooser would never change and command buttons would never fire.
   robot.py calls it in robot_periodic().

The camelCase method names are gone on purpose; this module is snake_case like the rest of
the a7 codebase.
"""

import ntcore
import wpilib

from helpers import mechanism_publisher
from telemetry import TelemetryRegistry
from tunables import TunableRegistry

_installed = False


def install(prefix: str = "") -> None:
    """Register the NetworkTables backends.  Call once, before anything publishes."""
    global _installed
    if _installed:
        return
    inst = ntcore.NetworkTableInstance.get_default()
    TelemetryRegistry.register_backend("/", wpilib.NetworkTablesTelemetryBackend(inst, prefix))
    TunableRegistry.register_backend("/", wpilib.NetworkTablesTunableBackend(inst, prefix))
    # Must happen before any Mechanism2d is constructed - it records the tree as it is
    # built, because a7 cannot walk one afterwards.
    mechanism_publisher.install()
    _installed = True


def update() -> None:
    """Pump the tunables so dashboard edits reach the robot, and re-publish any Field2d we
    are hand-publishing because of the a7 binding gap.  Call every loop."""
    TunableRegistry.update()
    publish_fields()
    mechanism_publisher.update()


def _split(key: str) -> tuple[str, str]:
    """'/SmartDashboard/Swerve/angle' -> ('/SmartDashboard/Swerve', 'angle').

    Bare keys keep a6's behaviour of living under /SmartDashboard.
    """
    key = key if key.startswith("/") else f"/SmartDashboard/{key}"
    path, _, name = key.rpartition("/")
    return (path or "/SmartDashboard"), name


class SmartDashboard:
    """The subset of a6's SmartDashboard this project used, on the a7 registries."""

    @staticmethod
    def put_number(key: str, value: float) -> None:
        path, name = _split(key)
        TelemetryRegistry.get_table(path).log(name, float(value))

    @staticmethod
    def put_string(key: str, value: str) -> None:
        path, name = _split(key)
        TelemetryRegistry.get_table(path).log(name, str(value))

    @staticmethod
    def put_boolean(key: str, value: bool) -> None:
        path, name = _split(key)
        TelemetryRegistry.get_table(path).log(name, bool(value))

    @staticmethod
    def put_data(key: str, obj) -> None:
        """Publish a rich object.

        Interactive things (Command, Selectable) go to the TUNABLE registry so the dashboard
        can write back - that is what makes a command button pressable and a chooser
        selectable.  Display-only things (Field2d, Mechanism2d) go to telemetry.
        """
        key = key if key.startswith("/") else f"/SmartDashboard/{key}"
        if hasattr(obj, "publish_tunables"):
            obj.publish_tunables(TunableRegistry.get_table(key))
            return
        if hasattr(obj, "log_to"):
            try:
                obj.log_to(TelemetryRegistry.get_table(key))
                return
            except TypeError:
                # a7 BINDING GAP - see _NATIVE_GAP below.  Native C++ objects (Field2d,
                # Mechanism2d) declare log_to(_NativeTelemetryTable), and Python has no way
                # to construct or obtain one: TelemetryRegistry.get_table() returns the
                # Python TelemetryTable, and _NativeTelemetryTable has "No constructor
                # defined".  So we publish what we can by hand.
                if isinstance(obj, wpilib.Field2d):
                    _publish_field2d(key, obj)
                    _field2ds[key] = obj
                    return
                if isinstance(obj, wpilib.Mechanism2d):
                    if mechanism_publisher.publish(key, obj):
                        return
                    # Only happens if the mechanism was built before install() ran.
                    print(f"*** dashboard: Mechanism2d at {key} was constructed before "
                          f"dashboard.install() - its tree was never recorded, so it cannot "
                          f"be published.  See helpers/mechanism_publisher.py ***")
                    return
                _warn_native_gap(key, obj)
                return
        raise TypeError(
            f"put_data({key!r}): {type(obj).__name__} has neither publish_tunables() nor "
            "log_to().  In 2027 an object must implement one of them to be publishable - "
            "Sendable no longer exists."
        )


# ---------------------------------------------------------------------------
# _NATIVE_GAP
#
# On 2027.0.0a7 a Field2d or Mechanism2d CANNOT be published through the telemetry registry
# from Python.  Both expose log_to(_NativeTelemetryTable), but nothing hands Python one:
#     TelemetryRegistry.get_table(...)  -> telemetry.TelemetryTable   (wrong type)
#     _NativeTelemetryTable(path)       -> TypeError: No constructor defined!
# This is an alpha binding gap, not something our code is doing wrong.  Re-test it on each
# new alpha; when log_to() stops raising TypeError, everything below can be deleted.
#
# Both are reproduced by hand instead: Field2d directly below, Mechanism2d in
# helpers/mechanism_publisher.py (it needs more machinery because a7 cannot walk the
# ligament tree, so the tree is recorded as it is built).
# ---------------------------------------------------------------------------
_field2ds: dict[str, "wpilib.Field2d"] = {}
_warned: set[str] = set()


def _warn_native_gap(key: str, obj) -> None:
    if key not in _warned:
        _warned.add(key)
        print(f"*** dashboard: cannot publish {type(obj).__name__} at {key} on WPILib 2027a7 "
              f"- log_to() needs a _NativeTelemetryTable that Python cannot obtain. "
              f"See _NATIVE_GAP in helpers/dashboard.py. ***")


def _pose_array(pose) -> list[float]:
    return [pose.x, pose.y, pose.rotation().degrees()]


def _publish_field2d(key: str, field: "wpilib.Field2d") -> None:
    """Write a Field2d's NT representation by hand: .type plus one double[] per object."""
    table = ntcore.NetworkTableInstance.get_default().get_table(key.lstrip("/"))
    table.put_string(".type", "Field2d")
    table.put_number_array("Robot", _pose_array(field.get_robot_pose()))


def publish_fields() -> None:
    """Re-publish every Field2d handed to put_data().  Call each loop from robot_periodic;
    unlike a real Sendable these do not update themselves."""
    for key, field in _field2ds.items():
        _publish_field2d(key, field)
