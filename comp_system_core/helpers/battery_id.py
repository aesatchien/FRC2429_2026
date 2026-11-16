"""
Which battery is in the robot, picked on the driver dashboard (gui/dashboard_2026.py), written into the console log.

EVERY BOOT STARTS WITH NO BATTERY.  A battery is never carried over from an earlier boot: a reboot is usually a battery swap.
To make that a guarantee, not a timing question, the robot makes a random BOOT ID each time it starts and publishes it on
BOOT_TOPIC.  The dashboard sends its choice as "<battery>@<boot id>" on BATTERY_TOPIC, and the robot accepts only a choice
carrying its OWN current boot id.  So a value from before the reboot - NetworkTables re-sends a client's last value when it
reconnects, which a quick robot-code restart would otherwise turn into the old battery - is ignored, and the log says nothing
until someone picks the battery for this boot.

Whenever a new, valid choice arrives, this prints one line,

    BATTERY: 2026-C

which helpers/console_log.py saves (with the robot time) in console_<boot UTC>.log, next to the .wpilog and the .hoot of the same
boot.  Scope2429 reads the last such line of a log's console file (gui/scope2429/helpers/console_file.py: battery()); a boot with
no such line shows its battery as '?'.  The id is echoed back on CONFIRM_TOPIC, so the dashboard can show that the robot has it.
"""
import re
import secrets

import ntcore

BATTERY_TOPIC = '/SmartDashboard/Battery/selected'     # written by the dashboard: "<battery>@<boot id>"
BOOT_TOPIC = '/SmartDashboard/Battery/boot'            # written here: this boot's id
CONFIRM_TOPIC = '/SmartDashboard/Battery/in_robot'     # written here: what the robot has logged this boot ('' = nothing yet)
ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9 _.-]{0,23}$')
PREFIX = 'BATTERY: '


def clean(value) -> str:
    """The id as it is logged, or '' for anything that is not one (empty, too long, odd characters)."""
    v = str(value or '').strip().upper()
    return v if ID_RE.match(v) else ''


def accept(value, boot: str) -> str:
    """The battery in a dashboard message "<battery>@<boot id>", or '' unless it was sent for THIS boot."""
    text = str(value or '')
    bid, sep, for_boot = text.rpartition('@')
    if not sep or for_boot != boot:
        return ''
    return clean(bid)


class BatteryId:
    def __init__(self, inst: ntcore.NetworkTableInstance | None = None) -> None:
        inst = inst or ntcore.NetworkTableInstance.get_default()
        self.boot = secrets.token_hex(4)                 # new every time the robot program starts
        self._sub = inst.get_string_topic(BATTERY_TOPIC).subscribe('')
        self._boot_pub = inst.get_string_topic(BOOT_TOPIC).publish()
        self._boot_pub.set(self.boot)
        self._confirm = inst.get_string_topic(CONFIRM_TOPIC).publish()
        self._confirm.set('')
        self.current = ''                                # no battery until one is picked for this boot

    def poll(self) -> str:
        """Call every robot_periodic().  Logs a new choice once; returns the battery in the robot ('' until one is picked)."""
        v = accept(self._sub.get(), self.boot)
        if v and v != self.current:
            was = self.current
            self.current = v
            print(f'{PREFIX}{v}' + (f'   (was {was})' if was else ''))
            self._confirm.set(v)
        return self.current
