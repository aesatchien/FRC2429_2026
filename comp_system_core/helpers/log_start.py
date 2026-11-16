"""Start the datalog once the USB stick is actually mounted.

WHY THIS EXISTS
---------------
On the SystemCore the robot program starts about 2-3 s BEFORE the USB stick is mounted.
From the robot's own boot log, 2026-10-10:

    09:54:46  RobotPy version 2027.0.0a7                       <- robot program starts
    09:54:47  *** /U/logs not found - logging to the default (internal) location ***
    09:54:49  FAT-fs (sdb1): ...                               <- stick mounted at /U

PR #16 checked for /U/logs once, while Swerve was being constructed, so whether the logs
reached the stick was a race decided fresh on every power-up.  Whoever lost, the whole
session went to internal storage.

So the start waits: robot_periodic() calls poll() every loop, and the log starts the first
time /U is a mount point, or after k_wait_s without one (internal storage, with a warning).
In simulation it starts on the first loop.  The first few seconds of a boot are not logged;
nothing happens in them.

The directory is passed explicitly.  WPILib's own default only looks at lowercase /u/logs;
there is a /u -> /U symlink on the competition SystemCore, but it was made by hand and is
not part of the image, so a reimaged or replacement controller will not have it.

Everything that has to land in the same place starts here, together: the WPILib .wpilog,
DriverStation logging, URCL (if enabled) and Phoenix's .hoot files.
"""

import os
import time

from wpilib import DataLogManager, DriverStation, RobotBase

import constants
from subsystems.swerve_constants import DriveConstants as dc


def session_name(now: float | None = None) -> str:
    """The folder name for one boot's logs, local wall-clock time like Phoenix's own hoot folders: 2026-11-15_14-03-27.
    Before the Driver Station has set the controller's clock it can read 1970 or a stale date; it still gives a new, unique
    name (the .wpilog inside is renamed with the real time once the clock is set, as before)."""
    return time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime(time.time() if now is None else now))


def new_session_dir(base: str, before: set) -> str:
    """The folder that appeared in `base` since `before` was listed: the one Phoenix just made for its .hoot, or ''."""
    try:
        now = {d for d in os.listdir(base) if os.path.isdir(os.path.join(base, d))}
    except OSError:
        return ''
    fresh = sorted(now - before)
    return os.path.join(base, fresh[-1]) if fresh else ''


def subdirs(base: str) -> set:
    try:
        return {d for d in os.listdir(base) if os.path.isdir(os.path.join(base, d))}
    except OSError:
        return set()


def decide(is_real: bool, mounted: bool, waited_s: float, wait_limit_s: float) -> str:
    """Pure decision so it can be tested: 'sim', 'usb', 'internal' or 'wait'."""
    if not is_real:
        return 'sim'
    if mounted:
        return 'usb'
    if waited_s >= wait_limit_s:
        return 'internal'
    return 'wait'


class DeferredLogStart:
    k_wait_s = 20.0   # the stick has been seen mounting ~3 s after the program starts
    k_hoot_wait_s = 3.0   # how long to wait for Phoenix to make its dated folder before making one ourselves
    k_internal_dir = '/home/systemcore/logs'   # where the SystemCore logs with no USB stick (Phoenix's default there too)

    def __init__(self, enabled: bool = constants.k_enable_logging,
                 usb_mount: str = constants.k_usb_mount, usb_log_dir: str = constants.k_usb_log_dir) -> None:
        self.started = not enabled
        self.usb_mount = usb_mount
        self.usb_log_dir = usb_log_dir
        self._t0 = time.monotonic()
        self._hoot = None          # (base, folders before, time) while waiting for Phoenix's dated folder
        self._choice, self._waited = '', 0.0
        self.session_dir = ''

    def poll(self) -> None:
        """Call every robot_periodic().  Does nothing once the log has started.

        All three logs of one boot go in ONE folder named with the date and time (2026-11-15_14-03-27/): the .hoot, the
        .wpilog and console_<boot UTC>.log.  Phoenix makes its own dated folder for every .hoot under the path it is given
        (SignalLogger.set_path), and that cannot be turned off, so it is started first, and the .wpilog and the console log
        then start in the folder it made.  Without Phoenix, or if its folder does not show up within k_hoot_wait_s, the
        folder is made here with the same kind of name."""
        if self._hoot is not None:
            self._finish_after_hoot()
            return
        if self.started:
            return
        waited = time.monotonic() - self._t0
        choice = decide(RobotBase.is_real(), os.path.ismount(self.usb_mount), waited, self.k_wait_s)
        if choice == 'wait':
            return
        self.started = True
        self._choice, self._waited = choice, waited

        base = 'logs' if choice == 'sim' else self.k_internal_dir   # sim: ./logs, as WPILib and Phoenix do by default
        if choice == 'usb':
            base = self.usb_log_dir
        elif choice == 'internal':
            print(f'  *** no USB stick at {self.usb_mount} after {waited:.0f} s'
                  f' - logging to the internal storage ({self.k_internal_dir}) ***')
        try:
            os.makedirs(base, exist_ok=True)
        except OSError as e:
            print(f'  *** {base} could not be created ({e}) - logging to WPILib\'s default location ***')
            base = ''

        if base and (dc.k_drive_vendor == 'ctre' or dc.k_turn_vendor == 'ctre'):
            # Phoenix first: its .hoot gets a dated folder of its own, and the other two logs then go in it
            from phoenix6 import SignalLogger
            before = subdirs(base)
            status = SignalLogger.set_path(base)
            if not status.is_ok():
                print(f'  *** Phoenix SignalLogger.set_path({base}) failed: {status} ***')
            SignalLogger.start()
            print('  started Phoenix SignalLogger (.hoot)')
            self._hoot = (base, before, time.monotonic())
            self._finish_after_hoot()
            return
        self._start_logs(os.path.join(base, session_name()) if base else '')

    def _finish_after_hoot(self) -> None:
        """Once Phoenix's dated folder exists (or k_hoot_wait_s has passed), start the .wpilog and the console log there."""
        base, before, t = self._hoot
        folder = new_session_dir(base, before)
        if not folder and time.monotonic() - t < self.k_hoot_wait_s:
            return                                   # not there yet: look again next loop (20 ms)
        self._hoot = None
        if not folder:
            folder = os.path.join(base, session_name())
            print(f'  *** Phoenix made no new folder in {base} within {self.k_hoot_wait_s:.0f} s: the .hoot is wherever Phoenix'
                  f' put it, the .wpilog and the console log go in {folder} ***')
        self._start_logs(folder)

    def _start_logs(self, log_dir: str) -> None:
        """The .wpilog (with the Driver Station data), the console log and URCL, all in log_dir ('' = WPILib's default)."""
        choice, waited = self._choice, self._waited
        if log_dir:
            try:
                os.makedirs(log_dir, exist_ok=True)
            except OSError as e:
                print(f'  *** {log_dir} could not be created ({e}) - logging to WPILib\'s default location ***')
                log_dir = ''
        self.session_dir = log_dir
        DataLogManager.start(dir=log_dir)
        DriverStation.start_data_log(DataLogManager.get_log())   # DS control and joystick data
        print(f'  logging to: {DataLogManager.get_log_dir()}  (started {waited:.1f} s after boot)')

        # the robot's console output goes in the same folder, next to the .wpilog it explains (helpers/console_log.py)
        from helpers import console_log
        console = console_log.get()
        if console is not None:
            path = console.open_in(DataLogManager.get_log_dir(), note=f'logging to {choice}, {waited:.1f} s after boot')
            print(f'  console log: {path or "NOT OPENED (see above)"}')

        # URCL is optional and imported lazily, so a missing package is a message rather than an
        # import error.  Toggle constants.k_enable_urcl.
        if constants.k_enable_urcl:
            try:
                import urcl
                urcl.URCL.start()   # unofficial REV logger for AdvantageScope
                print('  started URCL (REV device logging)')
            except ImportError:
                print('  *** k_enable_urcl is True but robotpy-urcl is not installed - skipping ***')
                print('      (no 2027 build exists yet; set constants.k_enable_urcl = False to silence)')

        # Krakens log through Phoenix's own SignalLogger (.hoot): started in poll(), before this, so that its dated folder
        # is the one this boot's .wpilog and console log go in.
