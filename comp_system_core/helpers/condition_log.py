"""Console logging for a condition that can persist: say when it STARTS and when it ENDS.

The console is for events.  A warning re-printed every 0.5 s for as long as a condition
lasts - "odometry clamped", "tag rejected" - is not more information, it is the same line
burying everything else.  This prints once on the way in, and once on the way out with how
many times it happened and for how long:

    *** Odometry clamped to field bounds: X=-0.40, Y=2.10 ***
    *** Odometry clamped to field bounds - cleared after 83 hits over 4.2 s ***

Usage - call hit() every loop the condition is true, clear() every loop it is not:

    self._clamp_log = ConditionLog("Odometry clamped to field bounds")
    if clamped:
        self._clamp_log.hit(f"X={x:.2f}, Y={y:.2f}")
    else:
        self._clamp_log.clear()
"""

import wpilib


class ConditionLog:
    def __init__(self, what: str) -> None:
        self.what = what
        self.active = False
        self.count = 0
        self.started = 0.0

    def hit(self, detail: str = "") -> None:
        """The condition is true this loop.  Prints only on the first hit."""
        if not self.active:
            self.active = True
            self.count = 0
            self.started = wpilib.Timer.get_timestamp()
            print(f"*** {self.what}{': ' + detail if detail else ''} ***")
        self.count += 1

    def clear(self) -> None:
        """The condition is false this loop.  Prints once, if it had been active."""
        if self.active:
            self.active = False
            elapsed = wpilib.Timer.get_timestamp() - self.started
            print(f"*** {self.what} - cleared after {self.count} hits over {elapsed:.1f} s ***")
