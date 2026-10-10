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

    def __init__(self, enabled: bool = constants.k_enable_logging,
                 usb_mount: str = constants.k_usb_mount, usb_log_dir: str = constants.k_usb_log_dir) -> None:
        self.started = not enabled
        self.usb_mount = usb_mount
        self.usb_log_dir = usb_log_dir
        self._t0 = time.monotonic()

    def poll(self) -> None:
        """Call every robot_periodic().  Does nothing once the log has started."""
        if self.started:
            return
        waited = time.monotonic() - self._t0
        choice = decide(RobotBase.is_real(), os.path.ismount(self.usb_mount), waited, self.k_wait_s)
        if choice == 'wait':
            return
        self.started = True

        log_dir = ''   # '' = WPILib's default (./logs in sim, internal storage on the robot)
        if choice == 'usb':
            try:
                os.makedirs(self.usb_log_dir, exist_ok=True)
                log_dir = self.usb_log_dir
            except OSError as e:
                print(f'  *** {self.usb_mount} is mounted but {self.usb_log_dir} could not be created ({e})'
                      f' - logging to the default (internal) location ***')
        elif choice == 'internal':
            print(f'  *** no USB stick at {self.usb_mount} after {waited:.0f} s'
                  f' - logging to the default (internal) location ***')

        DataLogManager.start(dir=log_dir)
        DriverStation.start_data_log(DataLogManager.get_log())   # DS control and joystick data
        print(f'  logging to: {DataLogManager.get_log_dir()}  (started {waited:.1f} s after boot)')

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

        # URCL only sees REV devices.  Krakens log through Phoenix's own SignalLogger, whose .hoot
        # files are separate from the .wpilog and need the same destination.
        if dc.k_drive_vendor == 'ctre' or dc.k_turn_vendor == 'ctre':
            from phoenix6 import SignalLogger
            if log_dir:
                status = SignalLogger.set_path(log_dir)
                if not status.is_ok():
                    print(f'  *** Phoenix SignalLogger.set_path({log_dir}) failed: {status} ***')
            SignalLogger.start()
            print('  started Phoenix SignalLogger (.hoot)')
